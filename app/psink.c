/******************************************************************************
* File Name:   psink.c
* \version 2.0
*
* Description: Power Sink (Consumer) manager source file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/
#if CY_PD_SINK_ONLY
#include <psink.h>
#include <app.h>
#include <cy_pdutils_sw_timer.h>
#include <app_timer_id.h>
#include "cy_usbpd_vbus_ctrl.h"
#include "srom.h"
#include "config.h"

#if CCG_HPI_ENABLE
#include <hpi_internal.h>
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

#if VBUS_OVP_ENABLE
bool app_psnk_vbus_ovp_cbk(void * cbkContext, bool comp_out);
#endif /* VBUS_OVP_ENABLE */

static bool gl_psnk_enabled[NO_OF_TYPEC_PORTS] = {
    false
#if CCG_PD_DUALPORT_ENABLE
    ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

#if CCG_HPI_ENABLE
#if CCG_HPI_VBUS_C_CTRL_ENABLE
static uint8_t gl_vbus_cfet_on_ctrl[NO_OF_TYPEC_PORTS] = {
    false
#if CCG_PD_DUALPORT_ENABLE
    ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};


void psnk_set_vbus_cfet_on_ctrl (uint8_t port, uint8_t enable)
{
    gl_vbus_cfet_on_ctrl[port] = enable;
}

void psnk_update_cfet_status (uint8_t port, uint8_t cfet_on)
{
    hpi_update_vbus_cfet_status (port, cfet_on);
}
#endif /* CCG_HPI_VBUS_C_CTRL_ENABLE */

void psnk_set_psnk_enabled(uint8_t port, uint8_t enable)
{
    gl_psnk_enabled[port] = enable;
}
#if CCG_HPI_VBUS_C_CTRL_ENABLE
hpi_response_t psnk_handle_vbus_cfet_ctrl_reg(uint8_t port, uint8_t cfet_ctrl)
{
    hpi_response_t code = HPI_RESPONSE_SUCCESS;
    const dpm_status_t* dpm_stat = CALL_MAP(dpm_get_info)(port);

    /*
     * Check if dead battery connection is present.
     * if Yes, then do not allow EC to control VBUS CFET.
     */
    if (false == dpm_stat->dead_bat)
    {
        /* Update vbus_cfet_on_ctrl appropriately. */
        psnk_set_vbus_cfet_on_ctrl(port, (uint8_t)((cfet_ctrl >> VBUS_CFET_CTRL_EC_CTRL_EN_POS) & 0x01u));

        /* check if EC VBUS CFET control is enabled or not. */
        if (cfet_ctrl & VBUS_CFET_CTRL_EC_CTRL_EN)
        {
            if (
                    (dpm_stat->attach) &&
                    (dpm_stat->fault_active == false) &&
                    (dpm_stat->cur_port_role == PRT_ROLE_SINK)
               )
            {
                uint8_t cfet_enable;
                /* EC CFET control is enabled. */
                if (cfet_ctrl & VBUS_CFET_CTRL_EC_CFET_ON)
                {
                    /* EC requested to tun on the CFET. */
                    sink_fet_on (port);
                    cfet_enable = true;
                }
                else
                {
                    /* EC requested to turn off the CFET. */
                    sink_fet_off (port);
                    cfet_enable = false;
                }
                psnk_set_psnk_enabled(port, cfet_enable);
            }
            else
            {
                if (cfet_ctrl & VBUS_CFET_CTRL_EC_CFET_ON)
                {
                    /* Return invalid command response
                     * as sink FET can not be enabled as requested by EC.
                     */
                    code = HPI_RESPONSE_INVALID_COMMAND;
                }
                /* SINK FET is not turned ON. Hence clear it. */
                cfet_ctrl &= (uint8_t)(~VBUS_CFET_CTRL_EC_CFET_ON);
            }
        }

        /* Update the VBUS cfet control register value.*/
        hpi_set_vbus_cfet_ctrl_reg (port, cfet_ctrl);
    }
    else
    {
        /* Return invalid command response as this can not be
         * accepted now.
         */
        code = HPI_RESPONSE_INVALID_COMMAND;
    }
    return code;
}
#endif /* CCG_HPI_VBUS_C_CTRL_ENABLE */
#endif /* CCG_HPI_ENABLE */

void sink_fet_off(cy_stc_pdstack_context_t * context)
{
#if defined(CY_DEVICE_CCG3PA)
    Cy_USBPD_Vbus_GdrvCfetOff(context->ptrUsbPdContext, VBUS_FET_CTRL);
#else
    Cy_USBPD_Vbus_GdrvCfetOff(context->ptrUsbPdContext, false);
#endif /* defined(CY_DEVICE_CCG3PA) */

#if CCG_HPI_VBUS_C_CTRL_ENABLE
    psnk_update_cfet_status(context->port, false);
#endif /*CCG_HPI_VBUS_CTRL_ENABLE */

}

void sink_fet_on(cy_stc_pdstack_context_t * context)
{
#if defined(CY_DEVICE_CCG3PA)
    Cy_USBPD_Vbus_GdrvCfetOn(context->ptrUsbPdContext, VBUS_FET_CTRL);
#else
    Cy_USBPD_Vbus_GdrvCfetOn(context->ptrUsbPdContext, false);
#endif /* defined(CY_DEVICE_CCG3PA) */

#if CCG_HPI_VBUS_C_CTRL_ENABLE
    psnk_update_cfet_status(context->port, true);
#endif /*CCG_HPI_VBUS_CTRL_ENABLE */
}

#if VBUS_OVP_ENABLE
/* Review which context to pass as this cb gets called from the driver */
bool  app_psnk_vbus_ovp_cbk(void * cbkContext, bool comp_out)
{
    (void)comp_out;
    cy_stc_pdstack_context_t * context = (cy_stc_pdstack_context_t *) cbkContext;

    /*OVP fault*/
    sink_fet_off(context);

    /* Set alert message */
    cy_pd_pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ovp = true;
    context->dpmStat.alert = alert;

    /*Enqueue HPI OVP fault event*/
    app_event_handler(context, APP_EVT_VBUS_OVP_FAULT, NULL);

    return false;
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
/* Review which context to pass as this cb gets called from the driver */
bool snk_vbus_uvp_cbk (cy_stc_usbpd_context_t * context, bool comp_out)
{
    (void)comp_out;
    (void)context;
    /*UVP fault*/
#if !CY_PD_SOURCE_ONLY
    sink_fet_off(pdstack_ctx);

    /*Enqueue HPI OVP fault event*/
    app_event_handler(pdstack_ctx, APP_EVT_VBUS_UVP_FAULT, NULL);
#endif /* !CY_PD_SOURCE_ONLY */
    return false;
}
#endif /* VBUS_UVP_ENABLE */


void psnk_set_voltage (cy_stc_pdstack_context_t * context, uint16_t volt_mV)
{
    app_status_t* app_stat = app_get_status(context->port);
    app_stat->psnk_volt = volt_mV;

    /* Disable VBus discharge when starting off as a SINK device. */
    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);

#if VBUS_OVP_ENABLE
#if (defined(CCG3) || defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF))
    app_ovp_enable(context, volt_mV, VBUS_FET_CTRL, app_psnk_vbus_ovp_cbk);
#else
    app_ovp_enable(context, volt_mV, false, app_psnk_vbus_ovp_cbk);
#endif /* CCGx */
#endif /* VBUS_OVP_ENABLE */
}

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
static uint32_t gl_sink_power_avail[NO_OF_TYPEC_PORTS] = {
    0
#if CCG_PD_DUALPORT_ENABLE
    ,
    0
#endif /* CCG_PD_DUALPORT_ENABLE */
};
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

void psnk_set_current (cy_stc_pdstack_context_t * context, uint16_t cur_10mA)
{
    app_status_t* app_stat = app_get_status(context->port);

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
    uint32_t snk_pwr = cur_10mA * app_stat->psnk_volt;

    /* Power available has reduced. Update PROCHOT signal. */
    if (snk_pwr < gl_sink_power_avail[port])
    {
        if (dpm_stat->contractExist)
        {
            /* Power available from source has reduced while in contract. Assert PROCHOT to notify EC. */
            ProcHot_Assert ();
        }
    }

    gl_sink_power_avail[port] = snk_pwr;
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

    /*
     * There is no implementation to update the current settings at present.
     * We are just storing the current value into a variable. This implementation
     * needs to be updated when the CCGx solution has capability to control the
     * sink current capability.
     */
    app_stat->psnk_cur = cur_10mA;
    if (cur_10mA <= CY_PD_ISAFE_DEF)
    {
        /* Notify the application layer to reduce current consumption to Stand by
         * Current. */
        app_event_handler(context, APP_EVT_STANDBY_CURRENT, NULL);

#if SNK_FET_SHUTDOWN_ENABLE
        /* Turn off the Sink FET if not in dead battery condition. */
        if(context->dpmStat.deadBat == false)
        {
            psnk_disable(context, NULL);
        }
#endif /* SNK_FET_SHUTDOWN_ENABLE */
    }

#if (POWER_BANK == 1)
    /* This is the implementation for CCG3 and CCG3PA Power banks. */

    if (port == TYPEC_PORT_0_IDX)
    {
        APP_SINK_SET_CURRENT_P1(cur_10mA);
    }
#if (CCG_PD_DUALPORT_ENABLE == 1)
    else
    {
        APP_SINK_SET_CURRENT_P2(cur_10mA);
    }
#endif /* (CCG_PD_DUALPORT_ENABLE == 1) */
#endif /* (POWER_BANK == 1) */
}

void psnk_enable (cy_stc_pdstack_context_t * context)
{
    uint8_t intr_state;
    
    intr_state = Cy_SysLib_EnterCriticalSection();

    /* Make sure discharge path is disabled at this stage. */
    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);

    /* Turn on FETs only if dpm is enabled and there is no active fault condition. */
    if ((context->dpmConfig.dpmEnabled) && (context->dpmStat.faultActive == false))
    {
#if ((SNK_FET_SHUTDOWN_ENABLE) || ((ICL_ENABLE) && (!ADP_DISABLE)))
        if (
#if (SNK_FET_SHUTDOWN_ENABLE)
                /* Enable the sink path only if we are allowed to draw more than 0.5 A of current. */
                (app_get_status(context->port)->psnk_cur > CY_PD_ISAFE_DEF)
#else
                (1)
#endif /* (SNK_FET_SHUTDOWN_ENABLE) */

                &&

#if ((ICL_ENABLE) && (!ADP_DISABLE))
                /* On ICL/TGL Platforms, there is no need to enable the consumer FET if a power adapter is connected. */
                (ADP_DETECT_Read() == 0)
#else
                (1)
#endif /* ((ICL_ENABLE) && (!ADP_DISABLE)) */
           )
#endif /* ((SNK_FET_SHUTDOWN_ENABLE) || ((ICL_ENABLE) && (!ADP_DISABLE))) */
        {
            {
                gl_psnk_enabled[context->port] = true;
                sink_fet_on(context);
            }
        }
    }

    Cy_SysLib_ExitCriticalSection(intr_state);
}


#if VBUS_OCP_ENABLE
void app_psnk_vbus_ocp_cbk(void * cbkContext, bool comp_out)
{
    (void)comp_out;
    cy_stc_pdstack_context_t * context = (cy_stc_pdstack_context_t *) cbkContext;

    /* OCP fault - it is sink problem, just remove the fet. */
    sink_fet_off(context);

    /* Enqueue HPI OCP fault event. */
    app_event_handler(context, APP_EVT_VBUS_OCP_FAULT, NULL);
}
#endif /* VBUS_OCP_ENABLE */

/*Timer Callback*/
static void app_psnk_tmr_cbk(cy_timer_id_t id,  void * callbackCtx)
{
    cy_stc_pdstack_context_t * context = callbackCtx;
    uint8_t port = context->port;
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSINK_DIS_TIMER:
            Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, APP_PSINK_DIS_MONITOR_TIMER);
            Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
            break;

        case APP_PSINK_DIS_MONITOR_TIMER:
            if(vbus_is_present(context, CY_PD_VSAFE_5V, 0) == false)
            {
                Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, APP_PSINK_DIS_TIMER);
                Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
                app_stat->snk_dis_cbk(context);
            }
            else
            {
                /*Start Monitor Timer again*/
                CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
            }
            break;
        default:
            break;
    }
}

void psnk_disable (cy_stc_pdstack_context_t * context, cy_pdstack_sink_discharge_off_cbk_t snk_discharge_off_handler)
{
    uint8_t intr_state;
    uint8_t port = context->port;
    app_status_t* app_stat = app_get_status(port);

    intr_state = Cy_SysLib_EnterCriticalSection();

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
    /* If consumer path is getting disabled, assert PROCHOT to notify the EC. */
    if (gl_psnk_enabled[port])
    {
        ProcHot_Assert ();
    }
    gl_sink_power_avail[port] = 0;
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

#if VBUS_OVP_ENABLE
    app_ovp_disable (context, false);
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
    app_uvp_disable (context, false);
#endif /* VBUS_UVP_ENABLE */

    sink_fet_off(context);
    gl_psnk_enabled[port] = false;
    /* Temporary build error fix */
    gl_psnk_enabled[port] = gl_psnk_enabled[port];

    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
    CALL_MAP(Cy_PdUtils_SwTimer_StopRange)(context->ptrTimerContext, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_MONITOR_TIMER);

    if ((snk_discharge_off_handler != NULL) && (context->dpmConfig.dpmEnabled))
    {
        Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);

        app_stat->snk_dis_cbk = snk_discharge_off_handler;

        /* Start Power source enable and monitor timer. */
        CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_TIMER_PERIOD, app_psnk_tmr_cbk);
        CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
    }
#if ((VBUS_IN_DISCHARGE_EN) && (POWER_BANK))
    else
    {
        /* It does not make sense to extra discharge even if we are not Type-C attached */
        if ((dpm_get_info(port)->attach == true) && (app_get_status(port)->psnk_volt > VSAFE_5V))
        {
            /*
             * If the same FET controls both source and sink operation, the VBUS_IN
             * side of the FET needs to be discharged to less than 5V. This operation
             * needs to be done. It is expected that this discharge shall happen 
             * before the tSrcTransition period. A fixed delay is used for this. It
             * can be extended to monitor VBUS based on the design as required.
             */
            pd_internal_vbus_in_discharge_on(port);
            timer_start(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_VBUS_IN_DIS_PERIOD, app_psnk_tmr_cbk);
        }
    }
#endif /* ((VBUS_IN_DISCHARGE_EN) && (POWER_BANK)) */

    /* Update the psnk_volt data structure so that we do not have stale value
     * till the next sink attach */
    app_stat->psnk_volt = CY_PD_VSAFE_5V;
    
    Cy_SysLib_ExitCriticalSection(intr_state);
}

/* End of File */
#endif /* CY_PD_SINK_ONLY */
