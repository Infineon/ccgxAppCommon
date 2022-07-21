/***************************************************************************//**
* \file psource.c
* \version 1.1.0 
*
* Power source (Provider) manager source file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "config.h"
#if CY_PD_SINK_ONLY
#include <psink.h>
#endif /* CY_PD_SINK_ONLY */
#include <app.h>
#include <psource.h>
#include <cy_sw_timer.h>
#include <cy_sw_timer_id.h>
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_pdstack_dpm.h"
#include <battery_charging.h>
#if CY_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CY_CABLE_COMP_ENABLE */

#if (!CY_PD_SINK_ONLY)
#if (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D))
#include  <vbus_idac_ctrl.h>
#include "cy_usbpd_idac_ctrl.h"
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D)) */

#if ICL_ENABLE
#include <icl.h>
#endif /* ICL_ENABLE */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#include "cy_usbpd_buck_boost.h"
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

/* Type-C current levels in 10mA units. */
#define CUR_LEVEL_3A                        (300)
#define CUR_LEVEL_1_5A                      (150)
#define CUR_LEVEL_DEF                       (90)

/* VBUS absolute maximum voltage in mV units */
#define VBUS_MAX_VOLTAGE                      (21500u)

/* Allowed margin (%) over 5V before the provider FET is turned ON. */
#define PSOURCE_SAFE_FET_ON_MARGIN          (5u)

static void psrc_dis_ovp(cy_stc_pdstack_context_t * context);
void psrc_en_ovp(cy_stc_pdstack_context_t * context);
void psrc_en_uvp(cy_stc_pdstack_context_t * context);
static void psrc_dis_uvp(cy_stc_pdstack_context_t * context);
void psrc_en_rcp(cy_stc_pdstack_context_t * context);
static void psrc_dis_ocp(cy_stc_pdstack_context_t * context);
static void psrc_shutdown(cy_stc_pdstack_context_t * context, bool discharge_dis);

extern void Cy_USBPD_BB_Enable(cy_stc_usbpd_context_t *context);
extern void Cy_USBPD_BB_Disable(cy_stc_usbpd_context_t *context);
extern bool Cy_USBPD_BB_IsEnabled(cy_stc_usbpd_context_t *context);
extern bool Cy_USBPD_BB_IsReady(cy_stc_usbpd_context_t *context);

#if APP_PSOURCE_SAFE_FET_ON_ENABLE

/* VBUS_IN lower limit for Safe FET ON, in mV */
#define APP_PSOURCE_SAFE_FET_ON_LEVEL_LOW   (4750u)

/* VBUS_IN upper limit for Safe FET ON, in mV */
#define APP_PSOURCE_SAFE_FET_ON_LEVEL_HIGH  (5250u)

static void psrc_fet_on_check_cbk(cy_timer_id_t id, void *callbackContext);

#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

void app_psrc_tmr_cbk(cy_timer_id_t id,  void * callbackCtx);

#if (VBUS_OVP_ENABLE ||  VBUS_UVP_ENABLE)
bool app_psrc_vbus_ovp_cbk(void *context, bool compOut);
#endif

#if VBUS_OCP_ENABLE
bool app_psrc_vbus_ocp_cbk(void * callbackCtx, bool compOut);
#endif

#if VBUS_SCP_ENABLE
bool app_psrc_vbus_scp_cbk(void * callbackCtx, bool compOut);
#endif

#if VBUS_RCP_ENABLE
void app_psrc_vbus_rcp_cbk(void * callbackCtx);
#endif

void psrc_select_voltage(cy_stc_pdstack_context_t *context);

#if NCP_MANAGEMENT
extern void ncp_manage_cbk(cy_timer_id_t id,  void * callbackCtx);
#endif /* NCP_MANAGEMENT */

static void vbus_fet_on(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
#if APP_PSOURCE_SAFE_FET_ON_ENABLE
    uint16_t vbus_in_volt;
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

    /* If fet is already On then no need to enable it again */    
    if(app_get_status(port)->is_vbus_on == false)
    {
#if APP_VBUS_SRC_FET_BYPASS_EN
        Cy_USBPD_BB_Enable(context->ptrUsbPdContext);
        app_get_status(port)->is_vbus_on = true;
#else /* !APP_VBUS_SRC_FET_BYPASS_EN */
#if APP_PSOURCE_SAFE_FET_ON_ENABLE
        
       /* Don't trun on the FET if Vbus-in voltage is not in safe range. */
        vbus_in_volt = Cy_USBPD_Adc_MeasureVbusIn(context->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);

        if ((vbus_in_volt < APP_PSOURCE_SAFE_FET_ON_LEVEL_LOW) || 
            (vbus_in_volt > APP_PSOURCE_SAFE_FET_ON_LEVEL_HIGH))
        {
            /*
             * Vbus-In is monitored with APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID.
             * If Vbus-in is not brought into the safe range within
             * APP_PSOURCE_SAFE_FET_ON_TIMER_ID timeout, then a manual OVP/UVP is 
             * triggered.
             */
#if CCG_TYPE_A_PORT_ENABLE
            if(port != TYPE_A_PORT_ID)
#endif /* CCG_TYPE_A_PORT_ENABLE */     
            {
                /*
                 * If VBUS_IN is above high threshold, turn on discharge.
                 * If VBUS_IN is below lower threshold, wait for the voltage to build up.
                 */
                if (vbus_in_volt > APP_PSOURCE_SAFE_FET_ON_LEVEL_HIGH)
                {
                    Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);
                }
                /* Start timer for safe FET ON timeout */
                (void)cy_sw_timer_start(context->ptrTimerContext,context,APP_PSOURCE_SAFE_FET_ON_TIMER_ID,
                        APP_PSOURCE_SAFE_FET_ON_TIMER_PERIOD, psrc_fet_on_check_cbk);

                /* Start timer to keep track of VBUS_IN voltage. */
                (void)cy_sw_timer_start(context->ptrTimerContext,context,APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
                        APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_PERIOD, psrc_fet_on_check_cbk);
            }
#if CCG_TYPE_A_PORT_ENABLE
            else
            {
                APP_VBUS_SNK_FET_OFF_P2();
                Cy_SysLib_DelayUs(10);
                APP_VBUS_SRC_FET_ON_P2();
            }
#endif /* CCG_TYPE_A_PORT_ENABLE */            
        }
        else
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */
        {
            app_get_status(port)->is_vbus_on = true;
#if NCP_POWER_SAVE
#if NCP_MANAGEMENT
            (void)cy_sw_timer_start(context->ptrTimerContext, context, NCP_ENABLE_DELAY_ID,NCP_ENABLE_DELAY_PERIOD,ncp_manage_cbk);
#else
            PD_CTRL_EN(port);
#endif /* NCP_MANAGEMENT */
#endif /* NCP_POWER_SAVE */

            if(port == TYPEC_PORT_0_IDX)
            {
                /*
                 * In case of REGULATOR_REQUIRE_STABLE_ON_TIME, the regulator is
                 * already turned on. POWER_BANK implementation, uses a single
                 * regulator and FET control for both source and sink operation.
                 * Turning OFF sink FET here will cause the regulator to get wrongly
                 * shut down. We should not disable sink here in this case.
                 */
#if (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK)))
                Cy_USBPD_Vbus_GdrvCfetOff(context->ptrUsbPdContext, false);
                Cy_SysLib_DelayUs(10);
#endif /* (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK))) */
                Cy_USBPD_Vbus_GdrvPfetOn(context->ptrUsbPdContext, true);
            }
#if CCG_PD_DUALPORT_ENABLE
            if(port == TYPEC_PORT_1_IDX)
            {
                APP_VBUS_SNK_FET_OFF_P2();
                Cy_SysLib_DelayUs(10);
                APP_VBUS_SRC_FET_ON_P2();
            }
#endif /* CCG_PD_DUALPORT_ENABLE */
        }
#endif /* APP_VBUS_SRC_FET_BYPASS_EN */
    }
}

static void vbus_fet_off(cy_stc_pdstack_context_t *context)
{
    app_get_status(context->port)->is_vbus_on = false;
#if NCP_POWER_SAVE
    PD_CTRL_DIS(context->port);
#endif /* NCP_POWER_SAVE */

#if APP_VBUS_SRC_FET_BYPASS_EN
    Cy_USBPD_BB_Disable(context->ptrUsbPdContext);
#else /* !APP_VBUS_SRC_FET_BYPASS_EN */

    if(context->port == TYPEC_PORT_0_IDX)
    {
        Cy_USBPD_Vbus_GdrvPfetOff(context->ptrUsbPdContext, true);
    }
#if CCG_PD_DUALPORT_ENABLE
    if(context->port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* APP_VBUS_SRC_FET_BYPASS_EN */
}

#endif /* (!CCG_SINK_ONLY) */

void vbus_discharge_on(cy_stc_pdstack_context_t * context)
{
    if(context->port == TYPEC_PORT_0_IDX)
    {
        Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);
    }
#if CCG_PD_DUALPORT_ENABLE
    if(context->port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

void vbus_discharge_off(cy_stc_pdstack_context_t * context)
{
    if(context->port == TYPEC_PORT_0_IDX)
    {
        Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
    }
#if CCG_PD_DUALPORT_ENABLE
    if(context->port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

#if (!CY_PD_SINK_ONLY)
#if CY_PD_PPS_SRC_ENABLE

#if VBUS_CF_SOFT_START_ENABLE
uint16_t gl_cf_cur_new;
uint16_t gl_cf_cur_prev = 0;

void vbus_ctrl_timer_isr(void)
{
    uint16_t step = 0;

    if (gl_cf_cur_prev < I_1A)
    {
        gl_cf_cur_prev = I_1A;
    }

    if (gl_cf_cur_new > gl_cf_cur_prev)
    {
        step = gl_cf_cur_new - gl_cf_cur_prev;
        if (step > VBUS_CF_SOFT_START_STEP)
        {
            step = VBUS_CF_SOFT_START_STEP;
        }

        step += gl_cf_cur_prev;
    }
    else
    {
        step = gl_cf_cur_prev - gl_cf_cur_new;
        if (step > VBUS_CF_SOFT_START_STEP)
        {
            step = VBUS_CF_SOFT_START_STEP;
        }

        step = gl_cf_cur_prev - step;
    }

    /* Load the CF setting. */
    pd_cf_enable(0, step);
    gl_cf_cur_prev = step;

    /* Clear the interrupt */
    VBUS_CTRL_TIMER_ClearInterrupt(VBUS_CTRL_TIMER_INTR_MASK_TC);

    /* If there is pending transition, queue an interrupt. */
    if (step != gl_cf_cur_new)
    {
        VBUS_CTRL_TIMER_ISR_StartEx(vbus_ctrl_timer_isr);
        VBUS_CTRL_TIMER_Start();
    }
    else
    {
        VBUS_CTRL_TIMER_Stop();
    }
}
#endif /* VBUS_CF_SOFT_START_ENABLE */

#if CCG_CF_HW_DET_ENABLE
static void psrc_cf_cbk(void * callbackContext, bool state)
{
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *pdstack_ctx = (cy_stc_pdstack_context_t *)(usbpd_ctx->pdStackContext);
    (void)state;

    if (app_get_status(usbpd_ctx->port)->cur_fb_enabled)
    {
        Cy_PdStack_Dpm_PpsTask(pdstack_ctx);
    }
}
#endif /* CCG_CF_HW_DET_ENABLE */

static void psrc_en_cf(cy_stc_pdstack_context_t * context)
{
#if CY_PD_VBUS_CF_EN
    cy_stc_pdstack_dpm_status_t dpm_stat = context->dpmStat;

    cy_stc_pdstack_dpm_ext_status_t* ptrDpmExtStat = &(context->dpmExtStat);

    app_status_t* app_stat = app_get_status(context->port);

    if (dpm_stat.srcLastRdo.val != dpm_stat.srcCurRdo.val)
    {
        uint32_t opCur = ((uint32_t)dpm_stat.srcCurRdo.rdo_pps.opCur * 5u); /* In 10mA units */

        /* Minimum supported limit is 1A. */
        if(opCur < CY_PD_I_1A)
        {
            opCur = CY_PD_I_1A;
        }

        /*
         * The PDO is power limited, then we should limit the current to the
         * maximum allowed by PDP limit. The PDP information is retrieved from
         * the extended source cap information.
         */
        if ((bool)dpm_stat.srcSelPdo.pps_src.ppsPwrLimited)
        {
            uint32_t limit = (ptrDpmExtStat->extSrcCap[CY_PD_EXT_SRCCAP_PDP_INDEX] * 1000u);

            /*
             * To improve arithmetic accuracy, the limit is calculated by
             * (mW * 100) / (mV) to get current in 10mA units.
             */
            limit = ((limit * 100u) / app_stat->psrc_volt);
            opCur = CY_USBPD_GET_MIN(opCur, limit);
        }
#if CCG_ENABLE_DYNAMIC_CL_LEVEL_3A
        /* Reduce the CL by 50mA for CC limit greater than 3A */
        if(opCur > CY_PD_I_3A)
        {
            opCur = opCur - 5u;
        }
#endif /* CCG_ENABLE_DYNAMIC_CL_LEVEL_3A */

#if !VBUS_CF_SOFT_START_ENABLE
        Cy_USBPD_CF_Enable(context->ptrUsbPdContext, opCur);
#else /* VBUS_CF_SOFT_START_ENABLE */
        /*
         * Do soft start on CF only if we are already in PPS mode. Otherwise do a direct load.
         * Slowly starting CF setting can overloading as the CF setting may not have caught up
         * with the previously set load.
         */
        if (app_stat->cur_fb_enabled)
        {
            gl_cf_cur_new = opCur;
            vbus_ctrl_timer_isr();
        }
        else
        {
            gl_cf_cur_prev = opCur;
            Cy_USBPD_CF_Enable(context->ptrUsbPdContext, opCur);
        }
#endif /* VBUS_CF_SOFT_START_ENABLE */
        app_stat->cur_fb_enabled = true;
    }
#else
    CY_UNUSED_PARAMETER(context);
#endif /* CY_PD_VBUS_CF_EN */
}

static void psrc_dis_cf(cy_stc_pdstack_context_t * context)
{
#if CY_PD_VBUS_CF_EN
    app_status_t* app_stat = app_get_status(context->port);

    if (app_stat->cur_fb_enabled)
    {
        Cy_USBPD_CF_Disable(context->ptrUsbPdContext);
        Cy_PdStack_Dpm_SetCf(context, false);
        app_stat->cur_fb_enabled = false;
#if !CCG_CF_HW_DET_ENABLE
        cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_CF_TIMER);
#endif /* !CCG_CF_HW_DET_ENABLE */
#if VBUS_CF_SOFT_START_ENABLE
        VBUS_CTRL_TIMER_Stop();
        gl_cf_cur_prev = 0;
#endif /* VBUS_CF_SOFT_START_ENABLE */
    }
#else
    CY_UNUSED_PARAMETER(context);
#endif /* CY_PD_VBUS_CF_EN */
}
#endif /* CY_PD_PPS_SRC_ENABLE */

static void call_psrc_ready_cbk(cy_stc_pdstack_context_t * context)
{
    app_status_t* app_stat = app_get_status(context->port);

    if (app_stat->pwr_ready_cbk != NULL)
    {
        app_stat->pwr_ready_cbk (context);
        app_stat->pwr_ready_cbk = NULL;
    }
#if CY_CABLE_COMP_ENABLE
    /* 
     * Enable cable compensation comparators for the first time when 
     * PS_RDY is sent.
     * Cable compensation task will take care of re-adjusting the comparators 
     * as and when comparator interrupts are received.
     */
    context->ptrUsbPdContext->cableStat.start=true;
#endif /* CY_CABLE_COMP_ENABLE */
}

#if APP_PSOURCE_SAFE_FET_ON_ENABLE
static void psrc_fet_on_check_cbk(cy_timer_id_t id, void *callbackContext)
{
    uint16_t vbus_in_volt;
    cy_stc_pdstack_context_t * context = callbackContext;
    /* 
     * If it is safe FET ON timeout, manually raise an OVP/UVP.
     * If it is VBUS_IN monitor callback:
     * - If VBUS_IN is in the safe range, then turn on the FET.
     * - If VBUS_IN is not in the safe range, then monitor VBUS_IN range further. 
     */

    vbus_in_volt = Cy_USBPD_Adc_MeasureVbusIn(context->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);

    if(id == APP_PSOURCE_SAFE_FET_ON_TIMER_ID)
    {
        /* VBUS_IN safe FET ON timeout */

        /* Make sure that monitor timer is OFF. */
        cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID);

        /* Turn off VBUS_IN discharge if it was enabled. */
        Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if VBUS_IN_DISCHARGE_EN
        Cy_USBPD_VbusIn_DischargeOff(context->ptrUsbPdContext);
#endif /* VBUS_IN_DISCHARGE_EN */
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

        if (vbus_in_volt < APP_PSOURCE_SAFE_FET_ON_LEVEL_LOW)
        {
#if VBUS_UVP_ENABLE
            app_psrc_vbus_ovp_cbk(context->ptrUsbPdContext, false);
#endif /* VBUS_UVP_ENABLE */
        }
        else
        {
#if VBUS_OVP_ENABLE
            app_psrc_vbus_ovp_cbk(context->ptrUsbPdContext, true);
#endif /* VBUS_OVP_ENABLE */
        }
    }
    else if(id == APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID)
    {
        /* VBUS_IN safe voltage monitor callback */

        if ((vbus_in_volt > APP_PSOURCE_SAFE_FET_ON_LEVEL_HIGH) || 
            (vbus_in_volt < APP_PSOURCE_SAFE_FET_ON_LEVEL_LOW))
        {
            (void)cy_sw_timer_start(context->ptrTimerContext,context,APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
                    APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_PERIOD, psrc_fet_on_check_cbk);
        }
        else
        {
            /* Make sure that timeout timer is off. */
            cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_SAFE_FET_ON_TIMER_ID);

            /* Turn off VBUS_IN discharge if it was enabled. */
            Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if VBUS_IN_DISCHARGE_EN
            Cy_USBPD_VbusIn_DischargeOff(context->ptrUsbPdContext);
#endif /* VBUS_IN_DISCHARGE_EN */
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

            /* Turn on the provider FET as VBUS_IN is in safe range */
            vbus_fet_on(context);
        }
    }
    else
    {
        /* Do Nothing */
    }
}
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

#if PSVP_FPGA_ENABLE
#define TRAN_FIXED_DELAY_MS    (20u)
volatile bool gl_tran_tmr_status = false;
#endif /* PSVP_FPGA_ENABLE */

/*Timer Callback*/
void app_psrc_tmr_cbk(cy_timer_id_t id,  void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    app_status_t* app_stat = app_get_status(context->port);

#if ((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S)) && (!defined(CY_DEVICE_CCG7D))&& (!defined(CY_DEVICE_CCG7S)))
#elif REGULATOR_REQUIRE_STABLE_ON_TIME
    bool stat1;
#if IBTR_ENABLE
    bool stat2;
#endif /* IBTR_ENABLE */
#endif /* ((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S)) && (!defined(CY_DEVICE_CCG7D)&& (!defined(CY_DEVICE_CCG7S)))) */

#if (VBUS_UVP_ENABLE)
    uint8_t uvp_en = GET_VBUS_UVP_TABLE(context->ptrUsbPdContext)->enable;
#endif /* VBUS_UVP_ENABLE */

    switch(id)
    {
        case APP_PSOURCE_EN_TIMER:
            /* Supply did not reach expected level. Turn off power and do error recovery. */
            cy_sw_timer_stop_range(context->ptrTimerContext, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_HYS_TIMER);
            app_stat->psrc_volt_old = CY_PD_VSAFE_0V;
            psrc_shutdown(context, true);

#if (VBUS_UVP_ENABLE)
            /*
             *If the VBUS does not reach VSAFE5V, then we need to treat this as an
             * under voltage condition. Since the UVP hardware detection cannot be
             * enabled when turning on the VBUS, this has to be manually triggered
             * from here by invoking the callback directly. Do this only if UVP is
             * enabled from the configuration table.
             */
            if (uvp_en != 0u)
            {
                app_psrc_vbus_ovp_cbk(context->ptrUsbPdContext, false);
            }
#endif /* (VBUS_UVP_ENABLE) */
            break;

#if (((!defined(CY_DEVICE_CCG3PA)) && (!defined(CY_DEVICE_CCG3PA2)) && (!defined(CY_DEVICE_PAG1S))\
            && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))))
        case APP_PSOURCE_EN_MONITOR_TIMER:
#if !PSVP_FPGA_ENABLE
            if (((app_stat->psrc_rising == true) &&
                        (vbus_is_present(context, app_stat->psrc_volt, VBUS_TURN_ON_MARGIN) == true)) ||
                    ((app_stat->psrc_rising == false) &&
                     (vbus_is_present(context, app_stat->psrc_volt, VBUS_DISCHARGE_MARGIN) == false)))
            {
                /* Start Source enable hysteresis Timer */
                cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_HYS_TIMER,
                        APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);
                break;
            }

            /* Start Monitor Timer again */
            cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
#else /* PSVP_FPGA_ENABLE */
            /*
             * Since, there is no feedback on vbus_is_present() in PSVP,
             * Use fixed delay for any voltage transition and proceed with PS_RDY.
             */
            if(gl_tran_tmr_status == false)
            {
                gl_tran_tmr_status = true;
                cy_sw_timer_start(context->ptrTimerContext, NULL, APP_TIMER_RESERVED_127, TRAN_FIXED_DELAY_MS, NULL);
                /* Start Monitor Timer again */
                cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_MONITOR_TIMER,
                        APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
            }
            else
            {
                if(cy_sw_timer_is_running(context->ptrTimerContext, APP_TIMER_RESERVED_127) == false)
                {
                    gl_tran_tmr_status = false;
                    /* Start Source enable hysteresis Timer */
                    cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_HYS_TIMER,
                            APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);
                }
                else
                {
                    /* Start Monitor Timer again */
                    cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_MONITOR_TIMER,
                            APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
                }
            }
#endif /* PSVP_FPGA_ENABLE */

            break;
#elif REGULATOR_REQUIRE_STABLE_ON_TIME
        case APP_PSOURCE_EN_MONITOR_TIMER:
            stat1 = vbus_ctrl_set_is_idle(context);
#if IBTR_ENABLE
            stat2 = Cy_USBPD_IBTR_IsIdle(context->ptrUsbPdContext);
            stat1 = (stat1 && stat2);
#endif /* IBTR_ENABLE */

            if (stat1 != false)
            {
                /* Start Source enable hysteresis Timer */
                (void)cy_sw_timer_start(context->ptrTimerContext,context,APP_PSOURCE_EN_HYS_TIMER,
                        APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);
            }
            break;   
#endif /* ((!defined(CY_DEVICE_CCG3PA)) && (!defined(CY_DEVICE_CCG3PA2)) && (!defined(CY_DEVICE_PAG1S))
            && (((!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))))) */

        case APP_PSOURCE_EN_HYS_TIMER:
#if REGULATOR_REQUIRE_STABLE_ON_TIME
            if (cy_sw_timer_is_running(context->ptrTimerContext, APP_PSOURCE_EN_MONITOR_TIMER))
            {
                return;
            }
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */           
            cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_EN_TIMER);
            app_stat->psrc_volt_old = app_stat->psrc_volt;
            vbus_discharge_off(context);

#if CCG_DYN_PFET_GATE_DRV_ENABLE
            Cy_USBPD_PfetDynDsEnable(context->ptrUsbPdContext);
#endif /* CCG_DYN_PFET_GATE_DRV_ENABLE */

            /*
             * Cannot trust the transition variable If there are aborted transitions
             * in opposite direction, then psrc_volt_old never gets updated. So it
             * is better to update all fault settings here. This is checked from
             * psrc_set_voltage() function before updating before transition.
             */
#if (((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))) || (PSVP_FPGA_ENABLE))
            if(app_stat->psrc_rising == false)
#endif /* (((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))) || (PSVP_FPGA_ENABLE)) */
            {
                /* VBus voltage has stabilized at the new lower level. Update the OVP and RCP limits. */
                psrc_en_ovp (context);
                psrc_en_rcp (context);
            }
#if (((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))) || (PSVP_FPGA_ENABLE))
            else
#endif /* (((!defined(CY_DEVICE_CCG3PA)) && (!defined(CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))) || (PSVP_FPGA_ENABLE)) */
            {
                psrc_en_uvp (context);
            }

#if CY_PD_PPS_SRC_ENABLE
            if (app_stat->cur_fb_enabled)
            {
#if !CCG_CF_HW_DET_ENABLE
                (void)cy_sw_timer_start(context->ptrTimerContext,context,APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD, app_psrc_tmr_cbk);
#else /* CCG_CF_HW_DET_ENABLE */
                Cy_USBPD_CF_Mon_Enable(context->ptrUsbPdContext, context->dpmStat.curFb, psrc_cf_cbk);
#if PSVP_FPGA_ENABLE
                ctrl_inf_get_omf_stat(app_stat->psrc_volt);
#endif /* PSVP_FPGA_ENABLE */
#endif /* CCG_CF_HW_DET_ENABLE */
            }
#endif /* CY_PD_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE */

#if CCG_SRGDRV_DISABLE_ON_TRANSITION
            pd_srgdrv_enable(port);
#endif /* CCG_SRGDRV_DISABLE_ON_TRANSITION */

            call_psrc_ready_cbk(context);
            break;

        case APP_PSOURCE_DIS_TIMER:
            /* Discharge operation timed out. */
            cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_DIS_MONITOR_TIMER);
            psrc_shutdown(context, true);
            break;

        case APP_PSOURCE_DIS_MONITOR_TIMER:

            if (vbus_is_present(context, CY_PD_VSAFE_5V, VBUS_DISCHARGE_TO_5V_MARGIN) == false)
            {
                /* Voltage has dropped below 5 V. We can now turn off the FET and continue discharge. */
                psrc_shutdown(context, false);
            }

            if (vbus_is_present(context, CY_PD_VSAFE_0V, VBUS_TURN_ON_MARGIN) == false)
            {
                /* Start Extra discharge to allow proper discharge below Vsafe0V */
                (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_DIS_EXT_DIS_TIMER,
                        APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            else
            {
                /* Start Monitor Timer again */
                (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_DIS_MONITOR_TIMER,
                        APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            break;

        case APP_PSOURCE_DIS_EXT_DIS_TIMER:
            cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_DIS_TIMER);
            vbus_discharge_off(context);

            /* Notify the caller that psrc_disable is complete. */
            call_psrc_ready_cbk(context);
            break;

#if CY_PD_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE
        case APP_PSOURCE_CF_TIMER:
            if (app_stat->cur_fb_enabled)
            {
                /*
                 * This task needs to be invoked every 100ms when in PPS mode
                 * operation for the state machine to function correctly. Since
                 * the function expects the VBUS to have stabilized before
                 * calling, it is being called from this timer interrupt handler.
                 */
                Cy_PdStack_Dpm_PpsTask(context);
                (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD,
                            app_psrc_tmr_cbk);
            }
            break;
#endif /* CY_PD_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE */

        default:
            /*
             * All the expected timers are taken care by the switch cases.
             * Hence, the default handler is left empty.
             */
            break;
    }
}

#if VBUS_OCP_ENABLE

#if HX3PD_DS1_OCP_HANDLING_WORKAROUND
CY_ISR (pwren_intr_handler)
{
    if(gpio_get_intr(HX3PD_DS1_PWREN_PIN))
    {
        CyIntDisable (HX3PD_DS1_PWREN_PIN >> 4);
        gpio_clear_intr (HX3PD_DS1_PWREN_PIN);
        dpm_clear_fault_active(TYPEC_PORT_1_IDX);
        dpm_start(TYPEC_PORT_1_IDX);
    }
}
#endif /* HX3PD_DS1_OCP_HANDLING_WORKAROUND */

#if HX3PD_DS2_OCP_HANDLING_WORKAROUND
CY_ISR (pwren_0_intr_handler)
{
    if(gpio_get_intr(HX3PD_DS2_PWREN_PIN))
    {
        CyIntDisable (HX3PD_DS2_PWREN_PIN >> 4);
        gpio_clear_intr (HX3PD_DS2_PWREN_PIN);
        dpm_clear_fault_active(TYPEC_PORT_0_IDX);
        dpm_start(TYPEC_PORT_0_IDX);
    }
}
#endif /* HX3PD_DS2_OCP_HANDLING_WORKAROUND */

bool app_psrc_vbus_ocp_cbk(void * callbackCtx, bool compOut)
{
    (void)compOut;

    cy_stc_usbpd_context_t *usbpdcontext=callbackCtx;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpdcontext->pdStackContext;
    uint8_t port=context->port;
    /* 
     * Stop all psource transition timers and notify stack about voltage
     * transition complete to process the SCP hard reset sequence. 
     * Also disable all other fault detection by calling psource shutdown.
     */
    cy_sw_timer_stop_range(context->ptrTimerContext, APP_PSOURCE_EN_TIMER, APP_PSOURCE_EN_HYS_TIMER);
    if(port == TYPEC_PORT_0_IDX)
    {
#if HX3PD_DS2_OCP_HANDLING_WORKAROUND
        if(gpio_read_value(HX3PD_DS2_PWREN_PIN))
        {
            /* Configure input from PWREN to handle events. */
            CyIntDisable (HX3PD_DS2_PWREN_PIN >> 4);
            gpio_clear_intr (HX3PD_DS2_PWREN_PIN);
#if (HX3PD_DS2_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_LOW)
            gpio_int_set_config (HX3PD_DS2_PWREN_PIN, GPIO_INTR_FALLING);
#elif (HX3PD_DS2_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_HIGH)
            gpio_int_set_config (HX3PD_DS2_PWREN_PIN, GPIO_INTR_RISING);
#endif /* (HX3PD_DS2_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_LOW) */
            CyIntSetVector (HX3PD_DS2_PWREN_PIN >> 4, pwren_0_intr_handler);
            CyIntEnable (HX3PD_DS2_PWREN_PIN >> 4);
        }
#endif /* HX3PD_DS2_OCP_HANDLING_WORKAROUND */
    }
    else
    {
#if HX3PD_DS1_OCP_HANDLING_WORKAROUND 
        if(gpio_read_value(HX3PD_DS1_PWREN_PIN))
        {
            /* Configure input from PWREN to handle events. */
            CyIntDisable (HX3PD_DS1_PWREN_PIN >> 4);
            gpio_clear_intr (HX3PD_DS1_PWREN_PIN);
#if (HX3PD_DS1_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_LOW)
            gpio_int_set_config (HX3PD_DS1_PWREN_PIN, GPIO_INTR_FALLING);
#elif (HX3PD_DS1_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_HIGH)
            gpio_int_set_config (HX3PD_DS1_PWREN_PIN, GPIO_INTR_RISING);
#endif /* (HX3PD_DS1_PWREN_POLARITY == HX3PD_PWREN_ACTIVE_LOW) */
            CyIntSetVector (HX3PD_DS1_PWREN_PIN >> 4, pwren_intr_handler);
            CyIntEnable (HX3PD_DS1_PWREN_PIN >> 4);
        }
#endif /* HX3PD_DS1_OCP_HANDLING_WORKAROUND */
    }
    call_psrc_ready_cbk(context);

#ifdef CY_DEVICE_CCG6
    /* Review if this is really required. */
    PDSS->pgdo_pu_1_cfg |= PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
#endif /* CCG6 */

    /* OCP fault. */
    psrc_shutdown(context, true);
    
#if OCP_FAULT_OUT
    OC_FAULT_OUT_Write(0);
#endif /* OCP_FAULT_OUT */    

    /* Set alert message */
    cy_pd_pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = 1u;
    context->dpmStat.alert = alert;

    /* Enqueue HPI OVP fault event. */
    app_event_handler(context, APP_EVT_VBUS_OCP_FAULT, NULL);
    return true;

}
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
bool app_psrc_vbus_scp_cbk(void * callbackCtx,  bool compOut)
{
    cy_stc_usbpd_context_t *usbpdcontext=callbackCtx;
    cy_stc_pdstack_context_t* context = (cy_stc_pdstack_context_t*)usbpdcontext->pdStackContext;
    /* 
     * Stop all psource transition timers and notify stack about voltage
     * transition complete to process the SCP hard reset sequence. 
     * Also disable all other fault detection by calling psource shutdown.
     */ 
    cy_sw_timer_stop_range(context->ptrTimerContext, APP_PSOURCE_EN_TIMER, APP_PSOURCE_EN_HYS_TIMER);
    call_psrc_ready_cbk(context);

    /* SCP fault. */
    psrc_shutdown(context, true);

    /* Set alert message */
    cy_pd_pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = 1u;
    context->dpmStat.alert = alert;

    /* Enqueue HPI SCP fault event. */
    app_event_handler(context, APP_EVT_VBUS_SCP_FAULT, NULL);
    CY_UNUSED_PARAMETER(compOut);
    return true;
}
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
void app_psrc_vbus_rcp_cbk(uint8_t port)
{
    cy_pd_pd_do_t alert;

    /* RCP fault. */
    psrc_shutdown(context, true);

    /* Treat RCP as equivalent to OVP and send an alert post fault recovery. */
    alert.val = 0;
    alert.ado_alert.ovp = 1u;
    context->dpmStat.alert = alert;

    /* Notify the solution layer about the fault. */
    app_event_handler(context, APP_EVT_VBUS_RCP_FAULT, NULL);
}
#endif /* VBUS_RCP_ENABLE */

#if ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE))

static void ovp_pwr_ready_cbk(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    /* Dummy callback to allow vbus discharge */
    (void)ptrPdStackContext;
}

bool app_psrc_vbus_ovp_cbk(void *callbackCtx,bool compOut)
{
    cy_stc_usbpd_context_t *usbpdcontext=callbackCtx;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpdcontext->pdStackContext;
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
    uint8_t port = context->port;
    app_status_t *app_stat = app_get_status(port);
    cy_pd_pd_do_t alert;

#if ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE))
    bool delay_shutdown = false;

    /*
     * If this is UVP and we are in PPS mode, we should not increment the
     * fault count. Also, we should be disabling the port before sending the
     * hard reset.
     */
    if ((app_stat->cur_fb_enabled) && (compOut == false))
    {
        delay_shutdown = true;
    }

    /* Drop source voltage to VSAFE_0V. */
    if (delay_shutdown == false)
#endif /* ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE)) */
    {
        app_stat->psrc_volt = CY_PD_VSAFE_0V;
        psrc_select_voltage(context);

        /*OVP fault*/
        psrc_shutdown(context, true);
    }

#ifndef CY_DEVICE_CCG4
    /* Enqueue HPI fault event */
    if (compOut == true)
#endif /* CCG4 */
    {
        /* OVP fault condition. */

        /* Set alert message to be sent after fault recovery. */
        alert.val = 0;
        alert.ado_alert.ovp = 1u;
        dpm_stat->alert = alert;

        app_event_handler(context, APP_EVT_VBUS_OVP_FAULT, NULL);
        psrc_disable(context, ovp_pwr_ready_cbk);
    }
#if ((!defined(CY_DEVICE_CCG4)) && (VBUS_UVP_ENABLE))
    else
    {
        /* UVP fault condition. */

        /* 
         * UVP is a hardware cutoff in micro seconds and OCP is a software
         * debounce and cutoff in milli seconds. When there is a sudden change
         * in Vbus current, Vbus voltage dips and causes UVP to react first
         * rather than OCP. Sink expects an alert message for OCP, that will be
         * missed if UVP is received. Hence, mimic OCP alert in case of UVP as
         * well. This will be taken care only for non-PPS contracts.
         */
        if(dpm_stat->srcSelPdo.src_gen.supplyType != CY_PDSTACK_PDO_AUGMENTED)
        {
            /* Set alert message */
            cy_pd_pd_do_t alert;
            alert.val = 0;
            alert.ado_alert.ocp = 1u;
            context->dpmStat.alert = alert;
        }

        app_event_handler(context, APP_EVT_VBUS_UVP_FAULT, NULL);
#if ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE))
        if (delay_shutdown == true)
        {
            psrc_shutdown(context, true);
        }
#endif /* ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE)) */

        psrc_disable(context, ovp_pwr_ready_cbk);
    }
#endif /* CCG4 */
    return true;
}
#endif /* ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE)) */

void psrc_select_voltage(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
#if (!HIGHER_VOLTAGES_SUPP_DISABLE)
    app_status_t *app_stat = app_get_status(port);
#if VBUS_OFFSET_VOLTAGE_ENABLE
    pwr_params_t *ptr = pd_get_ptr_pwr_tbl(port);
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */

#if CCG_PROG_SOURCE_ENABLE
    uint32_t select_volt = app_stat->psrc_volt;

    /* Don't drop voltage below 5 V. */
    if (app_stat->psrc_volt == CY_PD_VSAFE_0V)
    {
        app_stat->psrc_volt = CY_PD_VSAFE_5V;
    }

#if VBUS_OFFSET_VOLTAGE_ENABLE
    /*
     * Add Vbus offset voltage.
     * Vbus offset is not applicable for PPS supply type.
     */
    if(context->dpmStat->srcSelPdo.src_gen.supplyType != CY_PDSTACK_PDO_AUGMENTED)
    {
        select_volt += ptr->vbus_offset_volt ;
    }
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */
    
#if CY_CABLE_COMP_ENABLE
    if (context->ptrUsbPdContext->cableStat.enable == true)
    {
        select_volt += context->ptrUsbPdContext->cableStat.cableCompDrop;
    }
#endif /* CY_CABLE_COMP_ENABLE */

    /* 
     * Cap the selected voltage to the absolute maximum voltage that can be
     * applied to the cable. 
     */ 
    if (select_volt > VBUS_MAX_VOLTAGE)
    {
        select_volt = VBUS_MAX_VOLTAGE;
    }

    if(port == TYPEC_PORT_0_IDX)
    {
#if NCP_MANAGEMENT
        (void)cy_sw_timer_start(context->ptrTimerContext, context, NCP_VOLTCHANGE_DELAY_ID,NCP_VOLTCHANGE_DELAY_PERIOD,ncp_manage_cbk);
#else
        APP_VBUS_SET_VOLT_P1(context, select_volt);
#endif /* NCP_MANAGEMENT */
    }
#if CCG_PD_DUALPORT_ENABLE
    else
    {
       APP_VBUS_SET_VOLT_P2(select_volt);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

#else /* CCG_PROG_SOURCE_ENABLE */

    uint8_t intr_state = Cy_SysLib_EnterCriticalSection();

    if (port == TYPEC_PORT_0_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case CY_PD_VSAFE_9V:
                APP_VBUS_SET_9V_P1();
                break;
            case CY_PD_VSAFE_12V:
                APP_VBUS_SET_12V_P1();
                break;
            case CY_PD_VSAFE_13V:
                APP_VBUS_SET_13V_P1();
                break;
            case CY_PD_VSAFE_15V:
                APP_VBUS_SET_15V_P1();
                break;
            case CY_PD_VSAFE_19V:
                APP_VBUS_SET_19V_P1();
                break;
            case CY_PD_VSAFE_20V:
                APP_VBUS_SET_20V_P1();
                break;
            default:
                app_stat->psrc_volt = CY_PD_VSAFE_5V;
                APP_VBUS_SET_5V_P1();
                break;
        }
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case CY_PD_VSAFE_9V:
                APP_VBUS_SET_9V_P2();
                break;
            case CY_PD_VSAFE_12V:
                APP_VBUS_SET_12V_P2();
                break;
            case CY_PD_VSAFE_13V:
                APP_VBUS_SET_13V_P2();
                break;
            case CY_PD_VSAFE_15V:
                APP_VBUS_SET_15V_P2();
                break;
            case CY_PD_VSAFE_19V:
                APP_VBUS_SET_19V_P2();
                break;
            case CY_PD_VSAFE_20V:
                APP_VBUS_SET_20V_P2();
                break;
            default:
                app_stat->psrc_volt = CY_PD_VSAFE_5V;
                APP_VBUS_SET_5V_P2();
                break;
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
    Cy_SysLib_ExitCriticalSection(intr_state);
#endif  /* CCG_PROG_SOURCE_ENABLE */
#endif /* HIGHER_VOLTAGES_SUPP */
}

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
void app_psrc_ld_sw_cbk(uint8_t port, bool state)
{
    uint16_t volt_mV = app_get_status(port)->psrc_volt;

    /* We need to ensure that we are still active to filter out race conditions. */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * The load is above the threshold. Disable the load switch and configure
         * SR comparator to trigger on falling edge.
         */
        if (state)
        {
            APP_PSRC_LD_SW_OFF(port);
#if (!CY_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#endif /* (!CY_CABLE_COMP_ENABLE) */
        }
        else
        {
            APP_PSRC_LD_SW_ON(port);
#if (!CY_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_EN_NEG_DIS, app_psrc_ld_sw_cbk);
#endif /* (!CY_CABLE_COMP_ENABLE) */
        }
    }
    else
    {
#if (!CY_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CY_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_get_status(port)->ld_sw_ctrl = false;
    }
}

void psrc_ld_sw_ctrl(uint8_t port, uint16_t volt_mV)
{
    uint8_t intr_state;
    app_status_t *app_stat = app_get_status(port);

    intr_state = Cy_SysLib_EnterCriticalSection();
    /*
     * Turn ON the load switch if the requested voltage and current are below
     * the threshold specified. Since this voltage is always expected to be
     * below 5V, it is possible only when PPS is active.
     */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * If the load switch is already active, then do nothing. If not, configure
         * the SR comparator to interrupt when the current goes below the threshold.
         */
        if (app_stat->ld_sw_ctrl == false)
        {
#if (!CY_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#else
            /* 
             * Use Vbus current change from cable compensation callback.
             * And enable load switch to add initial load on the Vbus.
             */
            APP_PSRC_LD_SW_ON(port);
#endif /* (!CY_CABLE_COMP_ENABLE) */
            app_stat->ld_sw_ctrl = true;
        }
    }
    else
    {
        /*
         * Disable the SR comparator and disable the load switch.
         */
#if (!CY_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CY_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_stat->ld_sw_ctrl = false;
    }

    Cy_SysLib_ExitCriticalSection(intr_state);
}
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if (CY_CABLE_COMP_ENABLE)
/* 
 * This function will be called on every CABLE_COMP_HYS change in Vbus current.
 * vbusCur is in 10mA units.
 */
void app_psrc_vbus_cur_cbk(void *context, uint16_t vbusCur)
{
#if APP_VBUS_LOAD_SWITCH_ENABLE 
    app_status_t *app_stat = app_get_status(port);

    if (app_stat->ld_sw_ctrl == true)
    {
        if(vbusCur > (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, true);
        }
        else if(vbusCur < (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, false);
        }
    }
#else 
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(vbusCur);
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */
}
#endif /* (CY_CABLE_COMP_ENABLE) */

void psrc_set_voltage(cy_stc_pdstack_context_t * context, uint16_t volt_mV)
{
    uint8_t port = context->port;
    app_status_t *app_stat = app_get_status(port);
    const cy_stc_pd_dpm_config_t *ptrDpmConfig = &(context->dpmConfig);
#if CY_PD_PPS_SRC_ENABLE
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
#endif /* CY_PD_PPS_SRC_ENABLE */

#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
    uint16_t vbus_val;
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */

    app_stat->psrc_volt = volt_mV;

#if ((BATTERY_CHARGING_ENABLE) && (!QC_AFC_CHARGING_DISABLED))
    const bc_status_t *bc_stat = bc_get_status(context);
#endif /* ((BATTERY_CHARGING_ENABLE) && (!QC_AFC_CHARGING_DISABLED)) */

#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        APP_VBUS_SET_VOLT_P2 (volt_mV);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if CCG_SRGDRV_DISABLE_ON_TRANSITION
    pd_srgdrv_disable(port);
#endif /* CCG_SRGDRV_DISABLE_ON_TRANSITION */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Ensure that the feedback control logic is turned on. */
    vbus_ctrl_fb_enable(context);
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))*/

#if CY_CABLE_COMP_ENABLE
    cable_comp_enable(context, app_psrc_vbus_cur_cbk);
    cable_comp_cfg(context, volt_mV);
#endif /* CY_CABLE_COMP_ENABLE */

#if CY_PD_PPS_SRC_ENABLE

#if CCG_CF_HW_DET_ENABLE
    Cy_USBPD_CF_Mon_Disable(context->ptrUsbPdContext);
#endif /* CCG_CF_HW_DET_ENABLE */

    if (dpm_stat->srcSelPdo.fixed_src.supplyType == CY_PDSTACK_PDO_AUGMENTED)
    {
#if (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE)
        cable_comp_disable(context);
#endif /* (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE) */
        psrc_en_cf(context);
    }
    else
    {
        psrc_dis_cf(context);
    }
#endif /* CY_PD_PPS_SRC_ENABLE */

#if ((BATTERY_CHARGING_ENABLE) && (!QC_AFC_CHARGING_DISABLED))
    if (bc_stat->cur_mode == BC_CHARGE_QC3)
    {
#if (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_QC_3_0_DISABLE)
        /*
         * Disable cable compensation for QC3.0, as it can do
         * continuous voltage transition requests similar to PPS/QC4.0.
         */
        cable_comp_disable(context);
#endif /* (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_QC_3_0_DISABLE) */
    }
#endif /* ((BATTERY_CHARGING_ENABLE) && (!QC_AFC_CHARGING_DISABLED)) */

    /* Setup the load switch control. */
#if APP_VBUS_LOAD_SWITCH_ENABLE
    psrc_ld_sw_ctrl(port, volt_mV);
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if VBUS_OCP_ENABLE
    /* Leave OCP detection disabled while doing the voltage transition. */
    psrc_dis_ocp (context);
#endif /* VBUS_OCP_ENABLE */

    if (volt_mV != CY_PD_VSAFE_0V)
    {
#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
        vbus_val = vbus_get_value(context);
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */
        if((app_stat->psrc_volt >= app_stat->psrc_volt_old)
#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
            && ((app_stat->psrc_volt >= vbus_val) || (app_stat->is_vbus_on == false))
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */
        )
        {
            /* If voltage is being increased, make sure that OVP and RCP limits are moved up. */
            psrc_en_ovp (context);
            psrc_en_rcp (context);
        }
        else if ((app_stat->psrc_volt < app_stat->psrc_volt_old)
#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
                && (app_stat->psrc_volt < vbus_val)
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */
        )
        {
            /*
             * Enable UVP only if port partner is attached. We need to ensure that
             * UVP does not get enabled if VBUS is not applied, like in case of
             * HARD_RESET.
             */
            if ((ptrDpmConfig->attach == true) && (app_stat->is_vbus_on == true))
            {
                psrc_en_uvp (context);
            }
        }
        else
        {
            /* Do Nothing */
        }
    }

    psrc_select_voltage(context);
}

uint32_t psrc_get_voltage (cy_stc_pdstack_context_t *context)
{
#if CY_CABLE_COMP_ENABLE
    return app_get_status(context->port)->psrc_volt + (context->ptrUsbPdContext->cableStat.cableCompDrop);
#else /* !CY_CABLE_COMP_ENABLE */
    return app_get_status(context->port)->psrc_volt;
#endif /* CY_CABLE_COMP_ENABLE */
}

#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
static const uint32_t cc_rp_to_cur_map[] = {
    CUR_LEVEL_DEF,
    CUR_LEVEL_1_5A,
    CUR_LEVEL_3A
};
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

void psrc_set_current (cy_stc_pdstack_context_t *context, uint16_t cur_10mA)
{
    (void)cur_10mA;
    (void)context;
#if BATTERY_CHARGING_ENABLE
    const bc_status_t *bc_stat=NULL;
    (void)(*bc_stat);
#endif /* BATTERY_CHARGING_ENABLE*/

#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
    uint32_t ocp_cur;
    /* Update the OCP/SCP thresholds when required. */
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);

#if (CY_PD_VBUS_CF_EN && VBUS_OCP_ENABLE)
    app_status_t* app_stat = app_get_status(context->port);
#endif /* CY_PD_VBUS_CF_EN */

    if (ptrDpmConfig->contractExist)
    {
#if CY_PD_PPS_SRC_ENABLE
        if (dpm_stat->srcSelPdo.src_gen.supplyType == CY_PDSTACK_PDO_AUGMENTED)
        {
            /* Convert in 10mA units */
            ocp_cur = ((uint32_t)dpm_stat->srcSelPdo.pps_src.maxCur * 5u);
        }
        else
#endif /* CY_PD_PPS_SRC_ENABLE */
        {
            ocp_cur = dpm_stat->srcSelPdo.src_gen.maxCurPower;
        }
    }
    else
    {
        ocp_cur = cc_rp_to_cur_map[dpm_stat->srcCurLevel];

#if BATTERY_CHARGING_ENABLE
        bc_stat = bc_get_status(context);
        /* Update current limit as per battery charging module */
        if (
                (bc_get_status(context)->cur_mode != BC_CHARGE_NONE) &&
                (bc_get_status(context)->cur_mode != BC_CHARGE_CDP)
           )
        {
            ocp_cur = CY_USBPD_GET_MAX(ocp_cur, bc_get_status(context)->cur_amp);
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

#if VBUS_OCP_ENABLE
    if (GET_VBUS_OCP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
#if CY_PD_VBUS_CF_EN
        /* Enable OCP only if not in current foldback mode. */
        if (app_stat->cur_fb_enabled == false)
#endif /* CY_PD_VBUS_CF_EN */
        {
            Cy_USBPD_Fault_Vbus_OcpEnable(context->ptrUsbPdContext, ocp_cur, app_psrc_vbus_ocp_cbk);
        }
    }
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
    /* Enable SCP. */
    if (GET_VBUS_SCP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
        Cy_USBPD_Fault_Vbus_ScpEnable(context->ptrUsbPdContext, ocp_cur, app_psrc_vbus_scp_cbk);
    }
#endif /* VBUS_SCP_ENABLE */

#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */
}

#if PASC_FF_OV_ENABLE
static void psrc_ff_ov_cbk(cy_stc_pdstack_context_t * context)
{
    /* 
     * Perform Type-C error recovery to shut down port temporarily.
     * This will give some time to primary inductor to release
     * excessive energy in case in surge in input voltage.
     */
    Cy_PdStack_Dpm_SendTypecCommand(context, CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
}
#endif /* PASC_FF_OV_ENABLE */

#if ((defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && REGULATOR_REQUIRE_STABLE_ON_TIME)
/* Regulator enable status monitor interval */
#define REGULATOR_STAT_MON_TIME_MS         (1u)

/* This functions checks if regulator is ready or not. */
static void app_reg_stat_mon_tmr_cbk(cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)callbackContext;
    if(Cy_USBPD_BB_IsReady(context->ptrUsbPdContext) == false)
    {
        /* Regulator is not yet stable, start monitor timer */
        (void)cy_sw_timer_start(context->ptrTimerContext, context, (uint8_t)APP_PSOURCE_REGULATOR_MON_TIMER,
            REGULATOR_STAT_MON_TIME_MS, app_reg_stat_mon_tmr_cbk);
    }
    else
    {
        /* Regulator is now stable. Enable the FET. */
        vbus_fet_on(context);
    }
}
#endif /* ((defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && REGULATOR_REQUIRE_STABLE_ON_TIME) */

void psrc_enable (cy_stc_pdstack_context_t * context,
        cy_pdstack_pwr_ready_cbk_t pwr_ready_handler)
{
#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S))
    uint16_t vbus_val;
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S)) */
#endif /* !PSVP_FPGA_ENABLE */

#if CCG_TYPE_A_PORT_ENABLE
    if (context->port == TYPE_A_PORT_ID)
    {
        /* Turn on PSource FET for TYPE-A port. */
        vbus_fet_on(context);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    app_status_t* app_stat = app_get_status(context->port);
    const cy_stc_pdstack_dpm_status_t *dpm_stat = &(context->dpmStat);
    uint32_t intr_state;

    intr_state = Cy_SysLib_EnterCriticalSection();

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
    /*
     * Notify EC that we are about to provide power to a Type-C sink. Add some explicit delay before turning
     * provider path ON.
     */
    ProcHot_Assert ();
    Cy_SysLib_DelayUs (200);
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

    cy_sw_timer_stop_range(context->ptrTimerContext, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

#if APP_PSOURCE_SAFE_FET_ON_ENABLE
    cy_sw_timer_stop_range(context->ptrTimerContext, (uint8_t)APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
        (uint8_t)APP_PSOURCE_SAFE_FET_ON_TIMER_ID);
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

    /* Turn on FETs only if dpm is enabled and there is no active fault condition. */
    if ((context->dpmConfig.dpmEnabled) && (dpm_stat->faultActive == false))
    {
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
        /* Make sure OCP/SCP is enabled where required. The current parameter is not used. */
        psrc_set_current (context, CUR_LEVEL_3A);
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

#if REGULATOR_REQUIRE_STABLE_ON_TIME
        /* Review macro usage in this path*/
        if(Cy_USBPD_BB_IsReady(context->ptrUsbPdContext) == false)
        {
            /* Enable the regulator before turning on the FET */
            /* Need to review */
            // REGULATOR_ENABLE(port);
            Cy_USBPD_BB_Enable(context->ptrUsbPdContext);
            /*
             * CCG7D requires additional status check which is
             * taken care of by additional status check.
             */
#if (!(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)))
            (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_MONITOR_TIMER, REGULATOR_TURN_ON_DELAY, app_psrc_tmr_cbk);
#endif /* (!(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))) */
        }

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
        cy_sw_timer_stop(context->ptrTimerContext, APP_PSOURCE_REGULATOR_MON_TIMER);
        if (Cy_USBPD_BB_IsReady(context->ptrUsbPdContext) == false)
        {
            (void)cy_sw_timer_start(context->ptrTimerContext, context, (uint8_t)APP_PSOURCE_REGULATOR_MON_TIMER,
                REGULATOR_STAT_MON_TIME_MS, app_reg_stat_mon_tmr_cbk);
        }
        else
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */    
        {
            /* Turn off VBus Discharge by default. */
            vbus_discharge_off(context);

#if PASC_FF_OV_ENABLE
            pd_internal_ff_ov_en(port, PASC_FF_OV_VOLT_THRESHOLD, CCG_SRC_FET,
                    PASC_FF_OV_MODE_HW_CTRL, psrc_ff_ov_cbk);
#endif /* PASC_FF_OV_ENABLE */

            /* Turn on PSource FET */
            vbus_fet_on(context);
        }

        if (pwr_ready_handler != NULL)
        {
            app_stat->psrc_rising = true;

#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
            vbus_val = vbus_get_value(context);
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */
            /* If the VBus voltage is dropping, turn the discharge path on. */
            if((app_stat->psrc_volt_old > app_stat->psrc_volt)
#if !PSVP_FPGA_ENABLE
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
                    || ((app_stat->cur_fb_enabled == false) && (vbus_val > app_stat->psrc_volt))
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
#endif /* !PSVP_FPGA_ENABLE */
            )
            {
                app_stat->psrc_rising = false;
#if REGULATOR_REQUIRE_STABLE_ON_TIME
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
                /* Do not enable discharge if we are enabling the regulator. */
                if (Cy_USBPD_BB_IsReady(context->ptrUsbPdContext) != false)
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */
                {
                    vbus_discharge_on(context);
                }
            }
            app_stat->pwr_ready_cbk = pwr_ready_handler;

            /* Start Power source enable and monitor timers */
            (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_TIMER,
                    APP_PSOURCE_EN_TIMER_PERIOD, app_psrc_tmr_cbk);

#if ((!defined(CY_DEVICE_CCG3PA)) && (!defined(CY_DEVICE_CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S)))
            /* For CCG3PA/CCG3PA2/PAG1S APP_PSOURCE_EN_MONITOR_TIMER is not required */
            (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
#endif /* ((!defined(CY_DEVICE_CCG3PA)) && (!defined(CY_DEVICE_CCG3PA2)) && (!defined(CY_DEVICE_PAG1S)) && (!defined(CY_DEVICE_CCG7D)) && (!defined(CY_DEVICE_CCG7S))) */

        }
    }

    Cy_SysLib_ExitCriticalSection(intr_state);
}

void psrc_disable(cy_stc_pdstack_context_t * context, cy_pdstack_pwr_ready_cbk_t pwr_ready_handler)
{
    uint8_t port = context->port;
    app_status_t* app_stat = app_get_status(port);
    uint32_t intr_state;

#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
    bool stat;
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */

#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Turn off PSource FET for TYPE-A port. */
        vbus_fet_off(context);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    intr_state = Cy_SysLib_EnterCriticalSection();

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
    /*
     * If provider path is getting disabled while Type-C connection is present (PR-Swap or Hard Reset), assert
     * PROCHOT indication to the EC.
     */
    if (dpm_get_info(port)->connect)
    {
        ProcHot_Assert ();
    }
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

    cy_sw_timer_stop_range(context->ptrTimerContext, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

#if CY_PD_PPS_SRC_ENABLE
    psrc_dis_cf(context);
#endif /* CY_PD_PPS_SRC_ENABLE */

#if VBUS_UVP_ENABLE
    psrc_dis_uvp(context);
#endif /* VBUS_UVP_ENABLE */

#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S))
    stat = vbus_is_present(context, CY_PD_VSAFE_5V, 5);
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S)) */

    if((app_stat->psrc_volt_old <= CY_PD_VSAFE_5V)
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)))
        && (false == stat)
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S)) */
        )
    {
        psrc_shutdown(context, false);
        Cy_SysLib_DelayUs(20);
    }
    else
    {
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CY_DEVICE_CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S))
        /*
         * If we are in the middle of transition, then we cannot
         * use the psrc_volt_old variable as the actual voltage
         * may be different. To avoid false UV/OV trips, the better
         * option is to use the actual VBUS voltage instead.
         */
        if (app_stat->psrc_volt != app_stat->psrc_volt_old)
        {
            app_stat->psrc_volt_old = vbus_get_value(context);
        }
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CY_DEVICE_CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
        psrc_set_voltage(context, CY_PD_VSAFE_5V);
    }

    app_stat->psrc_volt_old = CY_PD_VSAFE_0V;

    if ((pwr_ready_handler != NULL) && (context->dpmConfig.dpmEnabled))
    {
        /* Turn on discharge to get the voltage to drop faster. */
        vbus_discharge_on(context);
        app_stat->pwr_ready_cbk = pwr_ready_handler;

        /*Start Power source enable and monitor timer*/
        (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_DIS_TIMER,
                APP_PSOURCE_DIS_TIMER_PERIOD, app_psrc_tmr_cbk);
        (void)cy_sw_timer_start(context->ptrTimerContext, context, APP_PSOURCE_DIS_MONITOR_TIMER,
                APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
    }
    else
    {
        psrc_shutdown(context, true);
    }

    Cy_SysLib_ExitCriticalSection(intr_state);
}

static void psrc_dis_ovp(cy_stc_pdstack_context_t * context)
{
#if VBUS_OVP_ENABLE
#if (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || \
        defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    app_ovp_disable (context, CCG_SRC_FET);
#else /* CCG4 */
    app_ovp_disable (context, false);
#endif /* CCGx */
#else
    CY_UNUSED_PARAMETER(context);
#endif /* VBUS_OVP_ENABLE */
}

static void psrc_dis_ocp(cy_stc_pdstack_context_t * context)
{
#if VBUS_OCP_ENABLE
    if (GET_VBUS_OCP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
        Cy_USBPD_Fault_Vbus_OcpDisable(context->ptrUsbPdContext,CCG_SRC_FET);
    }
#else
    (void)context;
#endif /* VBUS_OCP_ENABLE */
}

static void psrc_dis_scp(cy_stc_pdstack_context_t * context)
{
#if VBUS_SCP_ENABLE
    if (GET_VBUS_SCP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
        Cy_USBPD_Fault_Vbus_ScpDisable(context->ptrUsbPdContext);
    }
#else
    (void)context;    
#endif /* VBUS_SCP_ENABLE */
}

static void psrc_dis_uvp(cy_stc_pdstack_context_t * context)
{
#if VBUS_UVP_ENABLE
#if CCG4_DOCK
    app_uvp_disable (context->port, true);
#else /* !CCG4_DOCK */
    app_uvp_disable (context, CCG_SRC_FET);
#endif /* CCG4_DOCK */
#else
    (void)context;
#endif /* VBUS_UVP_ENABLE */
}

static void psrc_dis_rcp(cy_stc_pdstack_context_t * context)
{
#if VBUS_RCP_ENABLE
    system_vbus_rcp_dis(context);
#else
    (void)context;    
#endif /* VBUS_RCP_ENABLE */
}

static void psrc_shutdown(cy_stc_pdstack_context_t * context, bool discharge_dis)
{
    /*Turn Off Source FET*/
    vbus_fet_off(context);

#if PASC_FF_OV_ENABLE
    pd_internal_ff_ov_dis(port, CCG_SRC_FET);
#endif /* PASC_FF_OV_ENABLE */

    if(discharge_dis == true)
    {
        vbus_discharge_off(context);
    }

#if CY_CABLE_COMP_ENABLE
    cable_comp_disable(context);
#endif /* CY_CABLE_COMP_ENABLE */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S)\
    || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Ensure that the feedback control logic is turned off. */
    vbus_ctrl_fb_disable(context);
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S)
    || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
    /* Ensure that the load switch is disabled. */
    psrc_ld_sw_ctrl(port, VSAFE_5V);
#endif /* (APP_VBUS_LOAD_SWITCH_ENABLE) */

    /* Disable OVP/OCP/UVP */
    psrc_dis_ovp(context);
    psrc_dis_uvp(context);
    psrc_dis_ocp(context);
    psrc_dis_scp(context);
    psrc_dis_rcp(context);

#if CY_PD_PPS_SRC_ENABLE
    psrc_dis_cf(context);
#endif /* CY_PD_PPS_SRC_ENABLE */

#if APP_PSOURCE_SAFE_FET_ON_ENABLE
    cy_sw_timer_stop_range(context->ptrTimerContext, (uint8_t)APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
        (uint8_t)APP_PSOURCE_SAFE_FET_ON_TIMER_ID);
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /*
     * In CCG7D, internal Buk-Boost operates based on EA,
     * since this is a EA disable path, disable Buck-boost here.
     */
#if REGULATOR_REQUIRE_STABLE_ON_TIME
    Cy_USBPD_BB_Disable(context->ptrUsbPdContext);
    cy_sw_timer_stop(context->ptrTimerContext, (uint8_t)APP_PSOURCE_REGULATOR_MON_TIMER);
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
}

void psrc_en_ovp(cy_stc_pdstack_context_t * context)
{
#if (VBUS_OVP_ENABLE)
    uint8_t port=context->port;
#if (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG5C) || \
        defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    app_ovp_enable(context, app_get_status(port)->psrc_volt, CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#endif /* CCGx */
#else
    (void)context;
#endif /* (VBUS_OVP_ENABLE) */
}

void psrc_en_rcp(cy_stc_pdstack_context_t * context)
{
#if VBUS_RCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_RCP_EN_MASK)
    {
        system_vbus_rcp_en (port, app_psrc_vbus_rcp_cbk);
    }
#else
    (void)context;    
#endif /* VBUS_RCP_ENABLE */
}

void psrc_en_uvp(cy_stc_pdstack_context_t * context)
{
#if VBUS_UVP_ENABLE
    /* Using the same callback as OVP as behavior is same. */
#if CCG4_DOCK
    app_uvp_enable(context->port, app_get_status(port)->psrc_volt, true, app_psrc_vbus_ovp_cbk);
#else
    app_uvp_enable(context,app_get_status(context->port)->psrc_volt, CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#endif /* CCG4_DOCK */
#else
    (void)context;
#endif /* VBUS_UVP_ENABLE */
}

#endif /* (!CY_PD_SINK_ONLY) */

/* End of File */
