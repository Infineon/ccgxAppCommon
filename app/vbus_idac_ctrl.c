#include "config.h"
#include <stdbool.h>
#include "cy_pdstack_common.h"
#include <vbus_idac_ctrl.h>
#include <cy_pdutils_sw_timer.h>
#include <app.h>
#include "cy_usbpd_defines.h"
#include "app_timer_id.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_idac_ctrl.h"
#include "cy_usbpd_config_table.h"
#include "srom.h"
#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

/************************* PWM based VBUS Control. *****************************/

#define VBUS_C_PWM_PERIOD                               (480u)

uint16_t pwm_get_duty_cycle(uint16_t vbus, uint16_t min_volt, uint16_t max_volt,
    uint16_t pwm_period)
{
    uint32_t pwm_duty = 0;

    /*
     * Formula for Vout based on PWM Duty cycle is:
     * Vout = VBUS_MAX * (1/6 + 5/6 * PWM_DUTY_CYCLE)
     * Substituting PWM Period value and compare value (pwm_duty) in this formula
     * Vout = VBUS_MAX * (1/6 + 5/6 * pwm_duty/PWM_PERIOD)
     * Solving for pwm_duty
     * pwm_duty = PWM_PERIOD * ((Vout * 6 - VBUS_MAX)/VBUS_MAX * 5)
     */

    /* Ensure VBUS requested is not less than VBUS_MIN and more than VBUS_MAX. */
    if((vbus >= min_volt) && (vbus <= max_volt))
    {
        /* Applying formula as above. */
        pwm_duty = (vbus * 6) - max_volt;
        pwm_duty *= pwm_period;
        pwm_duty /=  max_volt * 5;
    }

    if(pwm_duty == 0)
    {
        pwm_duty = 1;
    }

    return pwm_duty;
}

void vbus_ctrl_pwm_turn_on(cy_stc_pdstack_context_t * context)
{
   uint8_t port=context->port;
   if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
       Cy_GPIO_Write(DIR_CTRL_C_PORT, DIR_CTRL_C_PIN, 1);
        /* Need to determine the FET to be used. */
        if (CCG_SRC_FET)
        {
            Cy_USBPD_Vbus_GdrvPfetOn(0, false);
        }
        else
        {
            Cy_USBPD_Vbus_GdrvCfetOn(0, false);
        }
        Cy_GPIO_Write(BUCK_BOOST_EN_C_PORT, BUCK_BOOST_EN_C_PIN, 0);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        type_a_enable_vbus ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
    else
    {
        /* Do Nothing */
    }
}
void vbus_ctrl_pwm_turn_off(cy_stc_pdstack_context_t * context)
{
    uint8_t port=context->port;

    /* Turn off PWM. */
    if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
        Cy_GPIO_Write(BUCK_BOOST_EN_C_PORT, BUCK_BOOST_EN_C_PIN, 1);
        /* Need to determine the FET to be used. */
        if (CCG_SRC_FET)
        {
            Cy_USBPD_Vbus_GdrvPfetOff(0, false);
        }
        else
        {
            Cy_USBPD_Vbus_GdrvCfetOff(0, false);
        }
        PWM_C_WriteCompare (1);
        PWM_C_Stop ();
        Cy_GPIO_SetDriveMode(PWM_C_PORT,PWM_C_PIN,CY_GPIO_DM_ALG_HIZ);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
      type_a_disable_vbus ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
    else
    {
        /* Do nothing */
    }
}

void vbus_ctrl_pwm_set_volt(cy_stc_pdstack_context_t * context, uint16_t volt_mV)
{
    uint8_t port=context->port;
    (void)volt_mV;

    /* Invoke PWM Control routine. */
    if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
        uint16_t pwm_duty;
        pwm_duty = pwm_get_duty_cycle (volt_mV, pd_get_ptr_pwr_tbl(TYPEC_PORT_0_IDX)->vbus_min_volt,
            pd_get_ptr_pwr_tbl(TYPEC_PORT_0_IDX)->vbus_max_volt, VBUS_C_PWM_PERIOD);

        PWM_C_Start ();
        PWM_C_WriteCompare (pwm_duty);
        Cy_GPIO_SetDriveMode(PWM_C_PORT,PWM_C_PIN,CY_GPIO_DM_STRONG);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
       type_a_set_voltage (volt_mV);
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
    else
    {
        /* Do nothing */
    }
}
/************************* FB based VBUS Control. *****************************/
typedef enum
{
    VBUS_CTRL_STATE_ENTRY = 0,      /* Entry state for VBUS change. */
    VBUS_CTRL_STATE_REACH_CLOSER,   /* State updates voltage to the requested level. */
    VBUS_CTRL_STATE_SLOPE_CHECK,    /* State waits for the voltage to stabilize. */
    VBUS_CTRL_STATE_FINE_TUNE,      /* State adjusts voltage based on ADC reading. */
    VBUS_CTRL_STATE_VBUS_SETTLE,    /* State provides required hysteresis wait time. */
    VBUS_CTRL_STATE_OVER            /* Voltage control completed state.*/

}vbus_ctrl_state_t;

/* 
 * This variable is used by the voltage update state machine. It holds whether to 
 * skip slope check or not after adjusting the voltage. This should not be modified
 * outside of the state machine logic.
 */
static bool gl_skip_slope_check[NO_OF_TYPEC_PORTS];
/* This flag holds whether the feedback control logic is enabled or not. */
static bool gl_vbus_ctrl_enable[NO_OF_TYPEC_PORTS];
/*
 * This variable holds the current iDAC setting loaded in hardware. This variable
 * avoids reading from the hardware registers every time. It should always remain
 * in sync with hardware setting.
 */
static int16_t gl_idac[NO_OF_TYPEC_PORTS];
/*
 * This variable is used by the voltage update state machine. It holds the required
 * change in steps of iDAC change pending to reach the requested voltage. IT should
 * not be modified outside of the state machine.
 */
static int16_t gl_idac_change[NO_OF_TYPEC_PORTS];
/*
 * This variable is the current voltage level set. This value corresponds to the
 * iDAC setting done. It should not be modified outside of the state machine.
 */
static uint16_t gl_cur_volt[NO_OF_TYPEC_PORTS];
/*
 * This variable is used by the slope check state machine to store the previously
 * read data. This variable should not be used as actual voltage data and should
 * not be used outside of the state machine.
 */
uint16_t gl_prev_volt[NO_OF_TYPEC_PORTS];
/*
 * This variable is used by the slope check state machine to store the new
 * request data. This variable should not be used as actual voltage data and should
 * not be used outside of the state machine.
 */
static uint16_t gl_req_volt[NO_OF_TYPEC_PORTS];

/*
 * This variable is used by the slope check state machine to store the number of
 * cycles of stable voltage attained. This variable should not be modified outside
 * of the state machine.
 */
static uint8_t gl_vbus_settle_cycle_count[NO_OF_TYPEC_PORTS];


#if VBUS_CTRL_RIPPLE_CHECK_ENABLE
/*
 * This variable is used by the slope check state machine to store the number of
 * cycles of stable voltage attained. This variable should not be modified outside
 * of the state machine.
 */
static uint8_t gl_vbus_ripple_cycle_count[NO_OF_TYPEC_PORTS];
#endif /* VBUS_CTRL_RIPPLE_CHECK_ENABLE */

/* Current state of the VBUS control state machine. */
static vbus_ctrl_state_t gl_vbus_ctrl_state[NO_OF_TYPEC_PORTS];

/*
 * This macro holds the accuracy requirement for voltage control. This parameter
 * is used to determine whether we should apply voltage correction algorithm or
 * not. This value is used in signed number operation and should be left as signed.
 */
#define VBUS_CLOSE_PERCENT                  (5u)
/*
 * This macro holds the limit after which we need to do slope detection for opto
 * isolator based designs. Since opto-isolator based designs are slow in voltage
 * correction, the slope check is done only beyond the threshold. Any voltage
 * change below this, is expected to meet PPS timings and should be covered by
 * the default timing provided. This value is used in signed number operation
 * and should be left as signed.
 */
#define VBUS_CTRL_OPTO_SLOPE_THRESHOLD      (500u)

/* 
 * This macro is used by the slope detection algorithm to determine stable voltage.
 * If the variation in voltage is below this threshold, it is considered as stable.
 * This value is used in signed number operation and should be left as signed.
 */
#define VBUS_STABLE_MAX_VAR_MV              (200u)

/* 
 * These macros hold the maximum change in iDAC steps allowed for the system.
 * These values are used in signed number operations and should be left as signed.
 */
#define VBUS_CTRL_MAX_SNK_UP_STEP       (((int16_t)VBUS_CTRL_ABOVE_5V_UP_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))
#define VBUS_CTRL_MAX_SNK_DOWN_STEP     (((int16_t)VBUS_CTRL_ABOVE_5V_DOWN_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))
#define VBUS_CTRL_MAX_SRC_UP_STEP       (((int16_t)VBUS_CTRL_BELOW_5V_UP_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))
#define VBUS_CTRL_MAX_SRC_DOWN_STEP     (((int16_t)VBUS_CTRL_BELOW_5V_DOWN_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))

extern void app_psrc_tmr_cbk(cy_timer_id_t id, void * callbackCtx);

/*
 * This function returns the ideal voltage matching the providing iDAC value.
 * It does not apply any accuracy improvement algorithm and is used by the state
 * machine to achieve the required accuracy.
 */
static int16_t vbus_ctrl_fb_get_volt(int16_t idac)
{
    return ((int16_t)CY_PD_VSAFE_5V + (idac * VBUS_CHANGE_PER_DAC_BIT));
}

static void vbus_ctrl_fb_set_volt_cbk(cy_timer_id_t id, void * callbackCtx);

#if !VBTR_ENABLE
/*
 * This function updates the iDAC values within the maximum change restrictions.
 * It takes in the signed iDAC change required and identifies the maximum change
 * allowed. It updates the hardware iDAC setting to this and returns the actual
 * change done. This function is repeatedly called by the state machine until 
 * all the required change is completed. The function does not do any error checks
 * and should not be called with incorrect or zero parameter values.
 */
static int16_t vbus_ctrl_fb_reach_closer(cy_stc_pdstack_context_t * context, int16_t idac_change)
{
    uint8_t port = context->port;
    int16_t step;

    if ((gl_idac[port] > 0) || ((gl_idac[port] == 0) && (idac_change > 0)))
    {
        /* We are using the sink iDAC. */
        if (idac_change > 0)
        {
            /*
             * We are increasing the voltage. But we can only do it within 
             * the maximum voltage step.
             */
            step = CY_USBPD_GET_MIN(idac_change, VBUS_CTRL_MAX_SNK_UP_STEP);
        }
        else
        {
            /*
             * We are decreasing the voltage. We can only do this till zero
             * or the maximum threshold level allowed.
             */
            step = idac_change * -1;
            if (step > VBUS_CTRL_MAX_SNK_DOWN_STEP)
            {
                step = VBUS_CTRL_MAX_SNK_DOWN_STEP;
            }
            if (gl_idac[port] != 0)
            {
                step = CY_USBPD_GET_MIN(gl_idac[port], step);
            }

            /* Step needs to be signed. */
            step = (step * -1);
        }
    }
    else
    {
        /* We are using the source iDAC. */
        if (idac_change > 0)
        {
            /*
             * We are increasing the voltage. But we can only do it within 
             * the maximum voltage step. Also, we cannot cross zero.
             */
            step = idac_change;
            if ((gl_idac[port] != 0) && ((gl_idac[port] + idac_change) > 0))
            {
                step = gl_idac[port] * -1;
            }
            step = CY_USBPD_GET_MIN(step, VBUS_CTRL_MAX_SRC_UP_STEP);
        }
        else
        {
            /*
             * We are decreasing the voltage. We can only do this for maximum
             * step size allowed.
             */
            step = (idac_change  * -1);
            if (step > VBUS_CTRL_MAX_SRC_DOWN_STEP)
            {
                step = VBUS_CTRL_MAX_SRC_DOWN_STEP;
            }

            /* Step needs to be signed. */
            step = (step * -1);
        }
    }

   /* Now we know the actual step size, we can apply it. */
   gl_idac[port] += step;
   Cy_USBPD_Hal_Set_Fb_Dac(context->ptrUsbPdContext, gl_idac[port]);

    return step;
}

#if (VBUS_CTRL_STEP_US_ENABLE)
static void vbus_ctrl_timer_isr(void)
{
    /* Clear the interrupt and invoke the voltage control callback. */
    VBUS_CTRL_TIMER_ClearInterrupt(VBUS_CTRL_TIMER_INTR_MASK_TC);
    vbus_ctrl_fb_set_volt_cbk(APP_PSOURCE_VBUS_SET_TIMER_ID,TYPEC_PORT_0_IDX);
}
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */
#else /* VBTR_ENABLE */
/* This is a callback to handle vbus control vbtr events */
static void vbus_ctrl_vbtr_cbk(void * callbackCtx, bool value)
{
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackCtx;
    cy_stc_pdstack_context_t * pdstack_ctx = (cy_stc_pdstack_context_t *)usbpd_ctx->pdStackContext;
    (void)value;
    /* 
     * Transition is completed.
     * Now invoke the psource set timer callback for the slope check.
     */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(pdstack_ctx->ptrTimerContext, pdstack_ctx,  GET_APP_TIMER_ID(pdstack_ctx,APP_PSOURCE_VBUS_SET_TIMER_ID),
        APP_PSOURCE_VBUS_SET_TIMER_PERIOD, vbus_ctrl_fb_set_volt_cbk);
}
#endif /* !VBTR_ENABLE */

/*
 * This function runs the voltage control state machine. This function is expected
 * to be called from the VBUS transition control timer running every 1ms until
 * the voltage transition is completed. The callback function itself determines
 * when to re-start the timer and should not be invoked outside of the current
 * vbus_ctrl_fb_set_volt() function.
 */
static void vbus_ctrl_fb_set_volt_cbk(cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    uint8_t port=context->port;
#if (((!PSVP_FPGA_ENABLE) && (!VBTR_ENABLE)) || ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE)))
    int16_t diff;
#endif /* #if (((!PSVP_FPGA_ENABLE) && (!VBTR_ENABLE)) || ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE))) */

#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
    uint16_t close_volt = 0;
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */    
    uint16_t volt_mV;
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
    int16_t idac_change;
#endif /* (VBUS_CTRL_ADC_ADJUST_ENABLE) */

#if (!PSVP_FPGA_ENABLE)
    uint32_t tmp;
#endif /* (!PSVP_FPGA_ENABLE) */
#if ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE))
    uint32_t abs_diff, thres;
    int16_t abs_diff_signed;
#endif /* ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE)) */
#if (VBTR_ENABLE)
    bool skip_timer = false;
#endif /* (VBTR_ENABLE) */
    app_status_t* app_stat = app_get_status(port);
    bool vbus_high_flag, vbus_low_flag;

    (void)id;

    volt_mV = gl_req_volt[port];

    switch(gl_vbus_ctrl_state[port])
    {
        case VBUS_CTRL_STATE_ENTRY:        
#if ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE))
            diff = (int16_t)volt_mV - (int16_t)gl_cur_volt[port];
            abs_diff_signed = (diff < 0) ? (diff * -1) : diff;
            abs_diff = (uint32_t)abs_diff_signed;
            thres = (((uint32_t)volt_mV * VBUS_CLOSE_PERCENT) / 100u);

            /* 
             * gl_skip_slope_check can be true only when re-entering here from
             * the VBUS_CTRL_STATE_FINE_TUNE state. In this case, do not try to
             * again break the transition; continue with a single load.
             */

            if (((abs_diff > thres)
#if BATTERY_CHARGING_ENABLE
#if !QC_AFC_CHARGING_DISABLED
            || (bc_get_status(context)->cur_mode == BC_CHARGE_QC3)
#endif /* !QC_AFC_CHARGING_DISABLED */
#endif /* BATTERY_CHARGING_ENABLE */
            ) && (gl_skip_slope_check[port] == false))
            {
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
                /*
                 * For direct feedback applications, apply ADC correction. This requires
                 * that we setup voltage to within 5% of the requested voltage and then
                 * correct the required level. In case of current limited state of PPS
                 * operation, we may not be able to measure via ADC. In this case, we
                 * may have to adjust the voltage based on absolute value. This will
                 * require us to store the voltage which was used for this adjustment
                 * as well.
                 */
                if (diff > 0)
                {
                    close_volt = (uint16_t)(volt_mV - thres);
                }
                else
                {
                    close_volt = (uint16_t)(volt_mV + thres);
                }

                /*
                 * If we are in current limit mode, then do not try to adjust 
                 * the voltage based on ADC. Just jump based on the iDAC step.
                 */
                if(context->dpmStat.curFb)
                {
                    gl_idac_change[port] = Cy_USBPD_Vbus_Ctrl_FbGetIdacStep(volt_mV, gl_cur_volt[port]);
                    close_volt = 0;
                }
                else
                {
                    /* Calculate relative idac value for close_volt w.r.t previous voltage */
                    gl_idac_change[port] = Cy_USBPD_Vbus_Ctrl_FbGetIdacStep(close_volt, gl_cur_volt[port]);

                    /* Calculate absolute idac value for close_volt */
                    idac_change = Cy_USBPD_Vbus_Ctrl_FbGetIdacStep(close_volt, CY_PD_VSAFE_5V) - gl_idac[port];

                    /* 
                     * Consider favorable value between relative idac and absolute idac values.
                     * favorable value shall be lower value for upward transition and 
                     * higher value for downward transition
                     */
                     
                    if (diff > 0)
                    {
                        if(gl_idac_change[port] > idac_change)
                        {
                            gl_idac_change[port] = idac_change;
                        }
                    }
                    else
                    {
                        if(gl_idac_change[port] < idac_change)
                        {
                            gl_idac_change[port] = idac_change;
                        }
                    }
                }
#else /* VBUS_CTRL_TRIM_ADJUST_ENABLE */
                /*
                 * For trim based application, we need to calculate the TRIM value
                 * required for getting to the relevant voltage.
                 */
                gl_idac_change[port] = (Cy_USBPD_Vbus_GetTrimIdac(context->ptrUsbPdContext, volt_mV) - gl_idac[port]);
                /*
                 * Since opto based designs are slow, we cannot impose same slope
                 * rules for all voltage changes and still meet PPS timings. To
                 * avoid this, the minimum time is adjusted to account for all
                 * transitions within 500mV. Anything above shall go with the
                 * slope check.
                 */
                if (abs_diff <= VBUS_CTRL_OPTO_SLOPE_THRESHOLD)
                {
                    gl_skip_slope_check[port] = true;
                }
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */
            }
            else
#endif /* ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE)) */
            {
                /* Take direct value. */
                gl_idac_change[port] = Cy_USBPD_Vbus_Ctrl_FbGetIdacStep(volt_mV, gl_cur_volt[port]);
#if (VBUS_CTRL_TRIM_ADJUST_ENABLE)
                gl_skip_slope_check[port] = true;
#endif /* (VBUS_CTRL_TRIM_ADJUST_ENABLE) */
            }

            /* Update the current voltage variable. */
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
            if (close_volt == 0u)
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */
            {
                gl_cur_volt[port] = volt_mV;
            }
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
            else
            {
                gl_cur_volt[port] = close_volt;
            }
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */

            gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_REACH_CLOSER;

            /* QAC suppression 2003: Intentional fall through */
            /* Intentional fall through */
        case VBUS_CTRL_STATE_REACH_CLOSER:
#if !VBTR_ENABLE
            /*
             * Keep loading the iDAC in steps until final value is reached.
             * The maximum step size allowed depends on the system.
             */
            gl_idac_change[port] -= vbus_ctrl_fb_reach_closer(context,gl_idac_change[port]);
            if (gl_idac_change[port] != 0)
            {
                break;
            }
#endif /* !VBTR_ENABLE */
            gl_prev_volt[port] = 0;
#if (PSVP_FPGA_ENABLE) || (VBTR_ENABLE)
            gl_vbus_settle_cycle_count[port] = 0;
#endif /* (PSVP_FPGA_ENABLE) || (VBTR_ENABLE) */
            if (gl_skip_slope_check[port])
            {
                gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_VBUS_SETTLE;
                gl_skip_slope_check[port] = false;
            }
            else
            {
#if VBUS_CTRL_RIPPLE_CHECK_ENABLE
                gl_vbus_ripple_cycle_count[port] = 0;
#endif /* VBUS_CTRL_RIPPLE_CHECK_ENABLE */
                gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_SLOPE_CHECK;
            }
#if VBTR_ENABLE
            /* 
             * Configure VBTR module for the required VBUS transition.
             * APP_PSOURCE_VBUS_SET_TIMER_ID timer will be re-enabled for
             * the slope check once the VBTR transition is completed.
             */
            skip_timer = true;
            gl_cur_volt[port] = gl_req_volt[port];
            gl_idac[port] += gl_idac_change[port];
            if(gl_idac_change[port] != 0x00)
            {
                Cy_USBPD_VBTR_SetIdac(context->ptrUsbPdContext, gl_idac[port], vbus_ctrl_vbtr_cbk);
            }
            else
            {

                vbus_ctrl_vbtr_cbk(context->ptrUsbPdContext, false);
            }
#endif /* VBTR_ENABLE */
            break;

        case VBUS_CTRL_STATE_SLOPE_CHECK:
#if (!PSVP_FPGA_ENABLE)
            tmp = vbus_get_value(context);
            if (gl_prev_volt[port] > tmp)
            {
                diff = (int16_t)gl_prev_volt[port] - (int16_t)tmp;
            }
            else
            {
                diff = (int16_t)tmp - (int16_t)gl_prev_volt[port];
            }
#if VBUS_CTRL_RIPPLE_CHECK_ENABLE
            /* Check for Ripples while moving Down */
            if(app_stat->psrc_rising == false)
            {
                if(tmp > gl_prev_volt[port])
                {
                    if((tmp - gl_prev_volt[port]) > VBUS_STABLE_MAX_VAR_MV)
                    {
                        /*
                         * If the current Vbus reading and the previous voltage differ by more than 200 mV
                         * on transitioning to lower voltage, treat the delta voltage as a ripple and increment the ripple counter.
                         */
                        gl_vbus_ripple_cycle_count[port]++;
                    }
                }
            }
            else
            {
                if(gl_prev_volt[port] > tmp)
                {
                    if((gl_prev_volt[port] - tmp) > VBUS_STABLE_MAX_VAR_MV)
                    {
                        /*
                         * If the current Vbus reading and the previous voltage differ by more than 200 mV
                         * on transitioning to higher voltage, treat the delta voltage as a ripple and increment the ripple counter.
                         */
                        gl_vbus_ripple_cycle_count[port]++;
                    }
                }
            }
            /* Check for Ripple Count */
            if(gl_vbus_ripple_cycle_count[port] >= VBUS_CTRL_RIPPLE_DEBOUNCE_PERIOD)
            {
                gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_VBUS_SETTLE;
                gl_vbus_ripple_cycle_count[port] = 0;
            }
#endif /* VBUS_CTRL_RIPPLE_CHECK_ENABLE */
            /*
             * If the slope for change is voltage has gone below the maximum,
             * variation in stable condition, continue to the next phase.
             */
            if (diff > (int16_t)VBUS_STABLE_MAX_VAR_MV)
            {
                /* Now store the current reading as previous. */
                gl_prev_volt[port] = (uint16_t)tmp;
#if (PSVP_FPGA_ENABLE) || (VBTR_ENABLE)
                gl_vbus_settle_cycle_count[port] = 0;
#endif /* (PSVP_FPGA_ENABLE) || (VBTR_ENABLE) */
                break;
            }
#endif /* ((!PSVP_FPGA_ENABLE) */
            /*
             * We now need to debounce the slope check as the change can happen
             * slower than what we can measure. This debounce does an extended
             * slope check. gl_vbus_settle_cycle_count is being used for this
             * debounce.
             */

            gl_vbus_settle_cycle_count[port]++;
            if (gl_vbus_settle_cycle_count[port] < VBUS_CTRL_SLOPE_DEBOUNCE_PERIOD)
            {
                break;
            }
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
            /*
             * If the final voltage is not yet achieved, then we need to proceed
             * with fine tuning the voltage based on ADC reading. Else proceed
             * to wait for settling time.
             */
            if (gl_cur_volt[port] != volt_mV)
            {
                gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_FINE_TUNE;
            }
            else
#endif /* (VBUS_CTRL_ADC_ADJUST_ENABLE) */
            {
                gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_VBUS_SETTLE;
            }
            break;

#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
        case VBUS_CTRL_STATE_FINE_TUNE:
            /*
             * This means that we have not attained the final voltage. Check
             * again to ensure that we are not in CF mode. If we are, then do
             * not adjust. Just update based on the stored voltage value.
             */
            if((context->dpmStat.curFb)==false)
            {
                gl_cur_volt[port] = vbus_get_value(context);
            }

            /* Now adjust the voltage as required. */
            gl_skip_slope_check[port] = true;
            gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_ENTRY;
            break;
#endif /* (VBUS_CTRL_ADC_ADJUST_ENABLE) */

        case VBUS_CTRL_STATE_VBUS_SETTLE:
            /*
             * If this is an active connection, then trigger the hysteresis timer.
             * In case of non-PPS PDO, we need to wait for voltage to attain the
             * required voltage. In case of PPS PDO, we need to enforce a minimum
             * settling time as there is no guaranteed voltage to look for.
             */
            if (CALL_MAP(Cy_PdUtils_SwTimer_IsRunning)(context->ptrTimerContext,  GET_APP_TIMER_ID(context,APP_PSOURCE_EN_TIMER)))
            {
                uint16_t delay;

                if (app_stat->cur_fb_enabled)
                {
                    delay = (APP_PSOURCE_EN_TIMER_PERIOD - 
                            Cy_PdUtils_SwTimer_GetCount(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_PSOURCE_EN_TIMER)));
                    if (delay < VBUS_CTRL_SETTLE_TIME_PERIOD)
                    {
                        delay = (APP_PSOURCE_EN_HYS_TIMER_PERIOD + 
                                (VBUS_CTRL_SETTLE_TIME_PERIOD - delay));
                    }
                    else
                    {
                        delay = APP_PSOURCE_EN_HYS_TIMER_PERIOD;
                    }
                }
                else
                {
                    vbus_high_flag = (vbus_is_present(context, gl_req_volt[port], VBUS_TURN_ON_MARGIN) == true);
                    vbus_low_flag = (vbus_is_present(context, gl_req_volt[port], VBUS_DISCHARGE_MARGIN) == false);

                    /* Wait for the voltage to be within range. */
                    if (!(((app_stat->psrc_rising == true) && vbus_high_flag) ||
                            ((app_stat->psrc_rising == false) && vbus_low_flag)))
                    {
                        break;
                    }

                    delay = APP_PSOURCE_EN_HYS_TIMER_PERIOD;
                }

                CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context,GET_APP_TIMER_ID(context,APP_PSOURCE_EN_HYS_TIMER), delay, app_psrc_tmr_cbk);
            }
            gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_OVER;
            /* Intentional fall through */

        case VBUS_CTRL_STATE_OVER:
            /* Do nothing. Stay in this state until a new request comes. */
            break;

        default:
            /* Will never come here. */
            break;
    }

    /* 
     * Restart the timer if VBUS transition is not completed.The timer is expected
     * to be running at 1ms. If the voltage transition steps needs to be done
     * at a smaller step size than this, then the timer call can be updated to
     * use a TCPWM timer when in VBUS_CTRL_STATE_REACH_CLOSER state.
     * If VBTR module is used, then timer is not needed for the VBUS transition,
     * hence timer is not enabled when in VBUS_CTRL_STATE_REACH_CLOSER state.
     * All other states still requires timer to be operating at 1ms step size.
     */
#if (VBUS_CTRL_STEP_US_ENABLE)
    if ((gl_vbus_ctrl_state[port] == VBUS_CTRL_STATE_REACH_CLOSER)
#if VBUS_CTRL_STEP_US_RISING_ONLY
    && (app_stat->psrc_rising)
#endif /* VBUS_CTRL_STEP_US_RISING_ONLY */
    )
    {
#if !VBTR_ENABLE
        /* 
         * Start a one shot TCPWM timer to trigger a voltage change. The timer
         * block is expected to be configured with the required period and so
         * just need to be started.
         */
        VBUS_CTRL_TIMER_ISR_StartEx(vbus_ctrl_timer_isr);
        VBUS_CTRL_TIMER_Start();
#endif /* !VBTR_ENABLE */
    }
    else
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */
    {
        if (
            (gl_vbus_ctrl_state[port] != VBUS_CTRL_STATE_OVER)
#if (VBTR_ENABLE)
            && (skip_timer == false)
#endif /* (VBTR_ENABLE) */
            )
        {
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context,GET_APP_TIMER_ID(context,APP_PSOURCE_VBUS_SET_TIMER_ID),
                    APP_PSOURCE_VBUS_SET_TIMER_PERIOD, vbus_ctrl_fb_set_volt_cbk);
        }
    }
}

/* This function returns if the vbus voltage transition is over or not. */
bool vbus_ctrl_set_is_idle(cy_stc_pdstack_context_t * context)
{
    uint8_t port=context->port;
    bool ret_val = false;
    (void)port;

     if (gl_vbus_ctrl_state[port] == VBUS_CTRL_STATE_OVER)
     {
         ret_val = true;
     }
     return ret_val;
}

#if VBUS_CTRL_ADC_ADJUST_ENABLE
static void vbus_ctrl_correct_init_volt(cy_stc_pdstack_context_t * context)
{
    uint8_t port=context->port;
    (void)port;
    uint16_t thres = 0;
    uint16_t vbus_diff = 0;
    uint16_t vbus_volt = 0;
    
    vbus_volt = Cy_USBPD_Adc_MeasureVbusIn(context->ptrUsbPdContext,CY_USBPD_ADC_ID_0,CY_USBPD_ADC_INPUT_AMUX_A);

    if(vbus_volt != CY_PD_VSAFE_5V)
    {
        if(vbus_volt > CY_PD_VSAFE_5V)
        {
            vbus_diff = (vbus_volt - CY_PD_VSAFE_5V);
        }
        else
        {
            vbus_diff = (CY_PD_VSAFE_5V - vbus_volt);
        }

        thres = ((CY_PD_VSAFE_5V * VBUS_CLOSE_PERCENT) / 100u);

        /* 
         * Direct voltage correction is allowed only if the
         * voltage required is less than threshold.
         */
        if(vbus_diff < thres)
        {
            /* Adjust voltage and keep track of IDAC value. */
            gl_idac[port] = Cy_USBPD_Vbus_Ctrl_FbGetIdacStep(CY_PD_VSAFE_5V, vbus_volt);
            Cy_USBPD_Hal_Set_Fb_Dac(context->ptrUsbPdContext, gl_idac[port]);
        }
        else
        {
            /* Do not adjust voltage */
        }
    }
    else
    {
        /* Initial voltage adjust is not required */ 
    }
}
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */

void vbus_ctrl_fb_enable(cy_stc_pdstack_context_t * context)
{
    uint8_t port=context->port;
    (void)port;

    /* Do this only if the control is disabled. */
    if (gl_vbus_ctrl_enable[port] == false)
    {
        Cy_USBPD_Hal_Enable_CV(context->ptrUsbPdContext);
#if VBUS_CTRL_TRIM_ADJUST_ENABLE
        gl_idac[port] = Cy_USBPD_Vbus_GetTrimIdac(context->ptrUsbPdContext, CY_PD_VSAFE_5V);
        Cy_USBPD_Hal_Set_Fb_Dac(context->ptrUsbPdContext, gl_idac[port]);
#else
        gl_idac[port] = 0;
#endif /* VBUS_CTRL_TRIM_ADJUST_ENABLE */
        gl_cur_volt[port] = CY_PD_VSAFE_5V;
#if VBUS_CTRL_ADC_ADJUST_ENABLE
        vbus_ctrl_correct_init_volt(context);
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */
        gl_vbus_ctrl_enable[port] = true;
        gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_OVER;
    }
}

void vbus_ctrl_fb_disable(cy_stc_pdstack_context_t * context)
{
    uint8_t port=context->port;
    (void)port;

    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_PSOURCE_VBUS_SET_TIMER_ID));
#if (VBUS_CTRL_STEP_US_ENABLE)
    VBUS_CTRL_TIMER_Stop();
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */
#if VBTR_ENABLE
    /* Exit any running VBTR operation */
    if (gl_vbus_ctrl_state[port] != VBUS_CTRL_STATE_OVER)
    {
        Cy_USBPD_VBTR_Abort(context->ptrUsbPdContext);
    }
#endif /* VBTR_ENABLE */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /*
     * EA need to stay active until Buck-Boost controller is active.
     * EA disable shall be done once Buck-Boost disable is done.
     */
#else /* !(defined(CCG7D) || defined(CCG7S)) */
    Cy_USBPD_Hal_Disable_CV(context->ptrUsbPdContext);
#endif /* CCGx */

    gl_cur_volt[port] = CY_PD_VSAFE_5V;
    gl_idac[port] = 0;
    gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_OVER;
    gl_vbus_ctrl_enable[port] = false;
}

void vbus_ctrl_fb_set_volt(cy_stc_pdstack_context_t * context, uint16_t volt_mV)
{
     uint8_t port=context->port;
     (void)port;
    /* Stop any previous state machine execution on receiving a new request. */
     CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_PSOURCE_VBUS_SET_TIMER_ID));
#if (VBUS_CTRL_STEP_US_ENABLE)
    VBUS_CTRL_TIMER_Stop();
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */

    /* If a request for 0V comes, then do nothing. Disable the FB block and exit. */
    if (volt_mV <= CY_PD_VSAFE_0V)
    {
        vbus_ctrl_fb_disable(context);
        return;
    }

    /* 
     * Adjust the requested voltage to the nearest step size. This is to
     * ensure that the requests for voltage levels are aligned to step sizes.
     */
    volt_mV = ((volt_mV / (uint8_t)VBUS_CHANGE_PER_DAC_BIT) * (uint8_t)VBUS_CHANGE_PER_DAC_BIT);

    /*
     * First calculate the new iDAC setting required for the operation. Simplest
     * is to use absolute based on the iDAC step size. However, using this can
     * result in loss of accuracy. In opto-isolator based designs, we can apply
     * a TRIM based calibration to calculate the required iDAC setting. In case
     * of direct feedback implementation, the voltage correction needs to be done
     * with ADC.
     * In all cases, if the voltage step size is too small, we do not apply
     * improvement algorithm as the PPS specification expects voltage change on
     * every request.
     */
    if (gl_cur_volt[port] < CY_PD_VSAFE_0V)
    {
        /* Ensure default values are correct. */
        gl_cur_volt[port] = CY_PD_VSAFE_0V;
        gl_idac[port] = 0;
    }

    /*
     * If the timer was stopped before completion, the gl_cur_volt is
     * wrong and needs to be fixed. This check needs to be taken up.
     */
    if (gl_vbus_ctrl_state[port] != VBUS_CTRL_STATE_OVER)
    {
#if VBTR_ENABLE
        Cy_USBPD_VBTR_Abort(context->ptrUsbPdContext);
        gl_idac[port] = Cy_USBPD_Hal_Get_Fb_Dac(context->ptrUsbPdContext);
#endif /* VBTR_ENABLE */
        gl_cur_volt[port] = (uint16_t)vbus_ctrl_fb_get_volt(gl_idac[port]);
    }

    /* Store the request voltage. */
    gl_req_volt[port] = volt_mV;

    /* Ensure that the default values are loaded correctly. */
    gl_skip_slope_check[port] = false;


    /* If the requested voltage is same as previous, then just hold-off for minimum time. */
    if (gl_cur_volt[port] == volt_mV)
    {
        /*
         * Evaluate if we need to adjust for missed ADC correction during
         * current foldback mode. This seems to make things worse and so is not
         * done for now.
         */
        gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_VBUS_SETTLE;
        /*
         * Start the timer so that we give time for APP_PSOURCE_EN_TIMER to get
         * started where applicable. This is so that we can trigger the callback
         * locally without having another monitor timer running from the psrc_enable()
         * function.
         */
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context,APP_PSOURCE_VBUS_SET_TIMER_ID,
            APP_PSOURCE_VBUS_SET_TIMER_PERIOD, vbus_ctrl_fb_set_volt_cbk);
    }
    else
    {
        gl_vbus_ctrl_state[port] = VBUS_CTRL_STATE_ENTRY;
        /* Now invoke the timer callback directly to avoid any delay. */
        vbus_ctrl_fb_set_volt_cbk(GET_APP_TIMER_ID(context,APP_PSOURCE_VBUS_SET_TIMER_ID), context);
    }
}

/* [] */

