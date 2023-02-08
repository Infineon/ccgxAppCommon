/******************************************************************************
* File Name:   fault_handlers.c
* \version 2.0
*
* Description: PD power related fault handling source file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "config.h"
#if (!CY_PD_SINK_ONLY)
#include <psource.h>
#endif /* !CY_PD_SINK_ONLY */
#if (CY_PD_SINK_ONLY)
#include <psink.h>
#endif /* (CY_PD_SINK_ONLY) */
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <cy_pdaltmode_vdm_task.h>
#include <cy_pdutils_sw_timer.h>
#include <app_timer_id.h>
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_config_table.h"
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#include <cy_pdaltmode_hw.h>
#include <cy_pdaltmode_mngr.h>
#endif /*(DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
#if SYS_BLACK_BOX_ENABLE
#include "blackbox.h"
#endif /*SYS_BLACK_BOX_ENABLE*/
#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

#if CCG_LOAD_SHARING_ENABLE
#include <loadsharing.h>
#endif /* CCG_LOAD_SHARING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
#include <sensor_check.h>
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CY_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CY_CABLE_COMP_ENABLE */

#if DP_UFP_SUPP
#include "cy_pdaltmode_dp_sid.h"
#endif /* DP_UFP_SUPP */

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#include <intel_ridge.h>
#endif /* RIDGE_SLAVE_ENABLE */

#include "srom.h"

enum
{
    FAULT_TYPE_VBUS_OVP = 0,    /* 0 */
    FAULT_TYPE_VBUS_UVP,        /* 1 */
    FAULT_TYPE_VBUS_OCP,        /* 2 */
    FAULT_TYPE_VBUS_SCP,        /* 3 */
    FAULT_TYPE_CC_OVP,          /* 4 */
    FAULT_TYPE_VCONN_OCP,       /* 5 */
    FAULT_TYPE_SBU_OVP,         /* 6 */
    FAULT_TYPE_OTP,             /* 7 */
    FAULT_TYPE_VBUS_RCP,        /* 8 */
    FAULT_TYPE_VBAT_GND_SCP,    /* 9 */
    FAULT_TYPE_VIN_UVP,         /* 10 */
    FAULT_TYPE_VIN_OVP,         /* 11 */
    FAULT_TYPE_ILIM_DET,        /* 12 */
    FAULT_TYPE_VREG_INRSH_DET,  /* 13 */
    FAULT_TYPE_VREG_BOD,        /* 14 */
    FAULT_TYPE_VCONN_SCP,       /* 15 */
    FAULT_TYPE_COUNT            /* 16 */
};

#if (VREG_INRUSH_DET_ENABLE || VREG_BROWN_OUT_DET_ENABLE)
extern app_sln_handler_t *solution_fn_handler;
#endif

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE) || (CCGX_V5V_CHANGE_DETECT))

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
static void vconn_restore_timer_cb (cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);

    if ((ptrDpmConfig->attach) && (ptrDpmConfig->vconnLogical))
    {
        /* If CCG is still the VConn source, start recovery actions. */
        vconn_change_handler (context, true);
    }
}
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

#if DP_UFP_SUPP
static void app_ufp_5v_recov_cb(cy_timer_id_t id, void * callbackCtx)
{
  /* Create app alt mode command to initiate DP related Vconn Swap procedure */
    uint8_t dp_cmd[4]  = {0xA, 0x00, 0x01, 0xFF};
    uint8_t dp_data[4] = {0x00, 0x00, 0x00, CY_PDALTMODE_DP_APP_VCONN_SWAP_CFG_CMD};

    if (Cy_PdAltMode_Mngr_EvalAppAltModeCmd((cy_stc_pdaltmode_context_t *)(((cy_stc_pdstack_context_t *)(callbackCtx))->ptrAltModeContext), dp_cmd, dp_data) == false)
    {
        /* Init Hard reset if DP alt mode not entered */
        Cy_PdStack_Dpm_SendPdCommand((cy_stc_pdstack_context_t *)callbackCtx, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL);
    }
    CY_UNUSED_PARAMETER(id);
}
#endif /* #if DP_UFP_SUPP */

void vconn_change_handler(cy_stc_pdstack_context_t * context, bool vconn_on)
{
#if (DFP_ALT_MODE_SUPP)
   cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
#endif /* (DFP_ALT_MODE_SUPP) */
   cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
   uint8_t port=context->port;

    if (!vconn_on)
    {
#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
        /* Store VConn fault status. */
        app_get_status(port)->fault_status |= APP_PORT_VCONN_FAULT_ACTIVE;
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

        /* Ensure that the VConn switch is turned off. */
        vconn_disable (context, ptrDpmConfig->revPol);

#if (DFP_ALT_MODE_SUPP)
        /* If we are the DFP, the cable requires VConn and Alt. Modes are Active, exit alt. modes. */
        if (
                (ptrDpmConfig->curPortType == CY_PD_PRT_TYPE_DFP) &&
                (dpm_stat->cblVdo.std_cbl_vdo.cblTerm != CY_PDSTACK_CBL_TERM_BOTH_PAS_VCONN_NOT_REQ) &&
                (app_status[port].alt_mode_entered != 0)
           )
        {
            vdm_task_mngr_exit_modes(port);
        }
#endif /* (DFP_ALT_MODE_SUPP) */
    }
    else
    {
#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
        app_get_status(port)->fault_status &= (~((uint8_t)APP_PORT_VCONN_FAULT_ACTIVE));
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

#if (DFP_ALT_MODE_SUPP)
        /*
         * Alt. Mode operation was previously suspended due to V5V not being present.
         * We can restart the alt. mode state machine so that mode discovery and operation
         * can take place.
         */
        app_vdm_layer_reset(port);
#endif /* (DFP_ALT_MODE_SUPP) */

#if DP_UFP_SUPP
        if ((ptrDpmConfig->contractExist) && (ptrDpmConfig->curPortType == CY_PD_PRT_TYPE_DFP))
        {
            CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,GET_APP_TIMER_ID(context,APP_VCONN_RECOVERY_TIMER),
                    APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, app_ufp_5v_recov_cb);
        }
#endif /* DP_UFP_SUPP */
    }
}

#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE) || (CCGX_V5V_CHANGE_DETECT)) */

#if ((VBAT_GND_SCP_ENABLE) && (VBAT_GND_SCP_RECOVERY_ENABLE))

/* Variable to keep status of VBAT_GND SCP. */
static bool gl_vbat_gnd_scp_status[NO_OF_TYPEC_PORTS];

/* Variable to keep status of VBAT_GND SCP pending retry. */
static bool gl_vbat_gnd_scp_retry_status[NO_OF_TYPEC_PORTS];

/* Variable to keep track of VBAT_GND SCP retry time. */
static uint16_t gl_vbat_gnd_scp_retry_time[NO_OF_TYPEC_PORTS];

/* Timer callback for retry/recovery of VBAT_GND_SCP fault */
static void app_vbat_gnd_scp_retry_cb(cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    uint8_t port=context->port;

    /* 
     * Recovery timer shall be running when fault is persistent.
     * Take recovery action during configured timeout intervals only.
     */

    if(CALL_MAP(Cy_PdUtils_SwTimer_IsRunning)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_VBAT_GND_SCP_TIMER_ID)) == true)
    {
        /* Do nothing. Wait until timeout happens to take action */
    }
    else
    {
        /* Handle recovery only if fault has occurred, otherwise reset retry state */
        if(gl_vbat_gnd_scp_status[port] == true)
        {
            /* 
             * Retry status shall not be set for the first fault and 
             * when recovery time limit is exceeded.
             */
            if(gl_vbat_gnd_scp_retry_status[port] == true)
            {
                /* 
                 * Start DPM, which implicitly re-enables VBAT_GND SCP
                 * during Type-C start.
                 */
                gl_vbat_gnd_scp_status[port] = false;
                context->dpmStat.faultActive = false;
                (void) Cy_PdStack_Dpm_Start(context);
            }
            else
            {
                /* 
                 * First fault is received or recovery timeout exceeded, 
                 * start timers without immediate retry.
                 */
            }

            /* 
             * When Battery voltage is not sufficient, do a long retries for 
             * only configured recovery time limit and give up.
             * Otherwise, if battery voltage is good, do infinite long 
             * retries with initial few short retries.
             */
            if(Cy_USBPD_MeasureVbat(context->ptrUsbPdContext) < VBAT_GND_SCP_BAT_THRESHOLD_MV)
            {
                if(gl_vbat_gnd_scp_retry_time[port] < (VBAT_GND_SCP_RETRY_MAX_PERIOD_S - 
                    VBAT_GND_SCP_LONG_RETRY_PERIOD_S))
                {
                    gl_vbat_gnd_scp_retry_status[port] = true;
                    gl_vbat_gnd_scp_retry_time[port] += VBAT_GND_SCP_LONG_RETRY_PERIOD_S;
                }
                else
                {
                    gl_vbat_gnd_scp_retry_status[port] = false;
                    gl_vbat_gnd_scp_retry_time[port] = VBAT_GND_SCP_RETRY_MAX_PERIOD_S;
                }

                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context,GET_APP_TIMER_ID(context,APP_VBAT_GND_SCP_TIMER_ID),
                    (VBAT_GND_SCP_LONG_RETRY_PERIOD_S * 1000u), app_vbat_gnd_scp_retry_cb);
            }
            else
            {
                if(gl_vbat_gnd_scp_retry_time[port] < (VBAT_GND_SCP_RETRY_MAX_PERIOD_S - 
                    VBAT_GND_SCP_SHORT_RETRY_PERIOD_S))
                {
                    gl_vbat_gnd_scp_retry_time[port] += VBAT_GND_SCP_SHORT_RETRY_PERIOD_S;
                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context, GET_APP_TIMER_ID(context,APP_VBAT_GND_SCP_TIMER_ID),
                        (VBAT_GND_SCP_SHORT_RETRY_PERIOD_S * 1000u), app_vbat_gnd_scp_retry_cb);
                }
                else
                {
                    gl_vbat_gnd_scp_retry_time[port] = VBAT_GND_SCP_RETRY_MAX_PERIOD_S;
                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context, GET_APP_TIMER_ID(context,APP_VBAT_GND_SCP_TIMER_ID),
                        (VBAT_GND_SCP_LONG_RETRY_PERIOD_S * 1000u), app_vbat_gnd_scp_retry_cb);
                }

                gl_vbat_gnd_scp_retry_status[port] = true;
            }
        }
        else
        {
            /* Device has recovered from the fault, stop retry and reset counters. */
            gl_vbat_gnd_scp_retry_status[port] = false;
            gl_vbat_gnd_scp_retry_time[port] = 0;
        }
    }
    CY_UNUSED_PARAMETER(id);
}
#endif /* ((VBAT_GND_SCP_ENABLE) && (VBAT_GND_SCP_RECOVERY_ENABLE)) */

#if ((VREG_INRUSH_DET_ENABLE) || (VREG_BROWN_OUT_DET_ENABLE))
/* Vreg faults recovery timer period in mS */
#define APP_VREG_RETRY_TIMER__PERIOD_MS         (150u)

static void app_vreg_fault_recovery_retry_cb(cy_timer_id_t id, void * callbackCtx)
{
    
    /* 
     * Backup any data if required and trigger a soft reset
     * as VDDD is not in safe range for silicon to work. 
     */
    NVIC_SystemReset ();
}

void app_vreg_inrush_fault_recovery_retry_cb(cy_timer_id_t id, void * callbackCtx)
{
    CY_UNUSED_PARAMETER(id);
    CY_UNUSED_PARAMETER(callbackCtx);

    /* Check if Vreg inrush fault persists after APP_VREG_RETRY_TIMER__PERIOD_MS */
    cy_stc_pdstack_context_t *pdstack_ctx = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_0_IDX);
    cy_stc_usbpd_context_t *usbpd_ctx = pdstack_ctx->ptrUsbPdContext;
    uint8_t i;

    if(false == Cy_USBPD_Fault_VregInrushStatus(usbpd_ctx))
    {
        pdstack_ctx = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_0_IDX);

        /* Enable interrupt handling until recovery is tried */
        Cy_USBPD_Fault_VregInrushDetEn(pdstack_ctx->ptrUsbPdContext);

        if(false == app_is_typec_attached())
        {
            /* Start DPM only if Vreg inrush fault condition has elapsed. */
            for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
            {
                pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);
                (void)Cy_PdStack_Dpm_Start(pdstack_ctx);
            }
        }
    }
    else
    {
        /* Re-enable the retry timer for next check. */
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (pdstack_ctx->ptrTimerContext, pdstack_ctx, GET_APP_TIMER_ID(pdstack_ctx,APP_HAL_GENERIC_TIMER),
            APP_VREG_RETRY_TIMER__PERIOD_MS, app_vreg_inrush_fault_recovery_retry_cb);
    }
}
#endif /* ((VREG_INRUSH_DET_ENABLE) || (VREG_BROWN_OUT_DET_ENABLE)) */

#if (UVP_OVP_RECOVERY || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_SCP_ENABLE)

/* Shorthand for any faults enabled. */
#define FAULT_HANDLER_ENABLE            (1u)

#endif /* (UVP_OVP_RECOVERY || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_SCP_ENABLE) */

#if (FAULT_HANDLER_ENABLE || CCG_REV3_HANDLE_BAD_SINK)

/* Number of retries defined by user for each fault type. */
static uint8_t gl_app_fault_retry_limit[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT];

/* Number of times each fault condition has been detected during current connection. */
static volatile uint8_t gl_app_fault_count[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT];

/* Check whether any fault count has exceeded limit for the specified PD port. */
bool app_port_fault_count_exceeded(cy_stc_pdstack_context_t * context)
{
    uint32_t i;
    bool     retval = false;
    uint8_t port = context->port;
    /*
     * Check whether the count for any fault type has exceeded the limit specified.
     */
    for (i = 0u; i < (uint8_t)FAULT_TYPE_COUNT; i++)
    {
        if (gl_app_fault_count[port][i] > gl_app_fault_retry_limit[port][i])
        {
            retval = true;
            break;
        }
    }

    return (retval);
}
#endif /* (FAULT_HANDLER_ENABLE || CCG_REV3_HANDLE_BAD_SINK) */

#if FAULT_HANDLER_ENABLE

/* 
 * If fault retry count in configuration table is set to this value, then
 * faults are not counted. That is, infinite fault recovery is enabled.
 */
#define FAULT_COUNTER_SKIP_VALUE        (255u)

#if CCG_HPI_AUTO_CMD_ENABLE
/*
 * Byte 0 is dynamic status and Byte 1 is sticky status.
 * Sticky status is set as soon as fault is hit and is maintained until
 * HPI read for fault status is requested. This status is clear on read.
 * Dynamic status is set as long as device is in the fault state.
 * Once the device is out of fault state, status is cleared.
 */
static uint16_t gl_app_fault_status[NO_OF_TYPEC_PORTS] = {0u};

uint32_t app_retrieve_fault_status(cy_stc_pdstack_context_t * context)
{
    uint8_t port = context->port;
    uint16_t status = gl_app_fault_status[port];
    /* Clear sticky fault status */
    gl_app_fault_status[port] &= 0x00FFu;
    return ((uint32_t)status);
}
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

/* This function stops PD operation and configures Type-C to look for detach of faulty device. */
void app_conf_for_faulty_dev_removal(cy_stc_pdstack_context_t * context)
{
#if !CY_PD_SOURCE_ONLY
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    uint8_t port = context->port;

    if ((!ptrDpmConfig->attach) || (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SINK))
    {
        /* Set flag to trigger port disable sequence. */
        app_get_status(port)->fault_status |= APP_PORT_SINK_FAULT_ACTIVE;
    }
#endif /* !CY_PD_SOURCE_ONLY */

    /* Stop PE */
    Cy_PdStack_Dpm_PeStop(context);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Make sure any alternate mode related state is cleared. */
    Cy_PdAltMode_VdmTask_MngrDeInit (context->ptrAltModeContext);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE\
        || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_OCP_ENABLE || VBUS_UVP_ENABLE)

#if ((FAULT_RETRY_DELAY_EN) || (FAULT_INFINITE_RECOVERY_EN))
/* Timer delay callback to recover from faults. */
static void fault_delayed_recovery_timer_cb(cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    /* 
     * Start DPM for retry.
     */
    Cy_PdStack_Dpm_ClearFaultActive(context);
    Cy_PdStack_Dpm_Start(context);
}
#endif /* ((FAULT_RETRY_DELAY_EN) || (FAULT_INFINITE_RECOVERY_EN)) */

/* Generic routine that notifies the stack about recovery actions for a fault. */
static void app_handle_fault(cy_stc_pdstack_context_t * context, uint32_t fault_type)
{
    uint8_t port = context->port;
    uint8_t reason = (uint8_t)CY_PDSTACK_HARDRES_REASON_VBUS_OVP;

    if (fault_type == ((uint32_t)FAULT_TYPE_VBUS_OCP))
    {
            reason = (uint8_t)CY_PDSTACK_HARDRES_REASON_VBUS_OCP;
    }

    if ((fault_type != ((uint32_t)FAULT_TYPE_VCONN_OCP)) && (fault_type != ((uint32_t)FAULT_TYPE_VCONN_SCP)))
    {
        (void)Cy_PdStack_Dpm_SetFaultActive(context);
    }

#if BCR
    switch(fault_type)
    {
        case FAULT_TYPE_VBUS_OVP:
            Cy_PdStack_Dpm_SetBCRFaultStat(context, BCR_FAULT_OV);
            break;
        case FAULT_TYPE_VBUS_OCP:
            Cy_PdStack_Dpm_SetBCRFaultStat(context, BCR_FAULT_OC);
            /* for BCR need only disable DC_OUT, do not check retry and avoid recovery or port_disable */
            return;
            break;
        case FAULT_TYPE_OTP:
            Cy_PdStack_Dpm_SetBCRFaultStat(context, BCR_FAULT_OT);
            /* for BCR need only disable DC_OUT, do not check retry and avoid recovery or port_disable */
            return;
            break;
        default:
            break;
    }
#endif /* BCR */

#if ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE))
    /*
     * If current foldback mode is enabled, then we should recover from the
     * failure as the behaviour is expected. But we should still continue to
     * handle the fault with hard reset. So, we do not let the fault count
     * to be incremented.
     */
    if ((fault_type != FAULT_TYPE_VBUS_UVP) || (app_get_status(context->port)->cur_fb_enabled == false))
#endif /* ((VBUS_UVP_ENABLE) && (CY_PD_PPS_SRC_ENABLE)) */
    {
#if CCG_REG_SEC_CTRL
        /*
         * If the line voltage is not available, do not register the count.
         * This is due to input removal. We still want to handle the fault
         * to disconnect the sink.
         */
        if (pd_hal_measure_line_volt(port) > PASC_LINE_FF_ON_THRESHOLD)
#endif /* CCG_REG_SEC_CTRL */
        {
            /* Update the fault count. */
            if(gl_app_fault_retry_limit[port][fault_type] == FAULT_COUNTER_SKIP_VALUE)
            {
                /* Do not count faults if infinite fault retry is set. */    
            }
            else
            {
                gl_app_fault_count[port][fault_type]++;
            }
        }
    }

#if CCG_HPI_AUTO_CMD_ENABLE
    if (gl_app_fault_count[port][fault_type] == (gl_app_fault_retry_limit[port][fault_type] + 1u))
    {
        if (((uint32_t)FAULT_TYPE_VBUS_OCP) == fault_type)
        {
            gl_app_fault_status[port] |= 0x01u;
        }
        else if (((uint32_t)FAULT_TYPE_VBUS_OVP) == fault_type)
        {
            gl_app_fault_status[port] |= 0x02u;
        }
        else if (((uint32_t)FAULT_TYPE_VBUS_UVP) == fault_type)
        {
            gl_app_fault_status[port] |= 0x04u;
        }
        else if (((uint32_t)FAULT_TYPE_VBUS_SCP) == fault_type)
        {
            gl_app_fault_status[port] |= 0x08u;
        }
        else
        {
            /* No code execution */
        }
    }
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

    if (gl_app_fault_count[port][fault_type] < (gl_app_fault_retry_limit[port][fault_type] + 1u))
    {
#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
        if ((fault_type == ((uint32_t)FAULT_TYPE_VCONN_OCP)) || (fault_type == ((uint32_t)FAULT_TYPE_VCONN_SCP)))
        {
            /* Start VConn turn-off procedure and start a timer to restore VConn after a delay. */
            vconn_change_handler (context, false);
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext,context, GET_APP_TIMER_ID(context,APP_VCONN_RECOVERY_TIMER), APP_VCONN_RECOVERY_PERIOD, vconn_restore_timer_cb);
        }
        else
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */
        {
#if FAULT_RETRY_DELAY_EN
            /* 
             * Fault retry delay timer shall be enabled.
             * Disable the port until configured timeout elapses.
             */
            (void)Cy_PdStack_Dpm_Stop(context);
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext,context, GET_APP_TIMER_ID(context,APP_FAULT_RECOVERY_TIMER),
                    FAULT_RETRY_DELAY_MS, fault_delayed_recovery_timer_cb);
            (void)reason;
#else /* !FAULT_RETRY_DELAY_EN */
            (void)Cy_PdStack_Dpm_ClearHardResetCount(context);
            /*
             * Try a Hard Reset to recover from fault.
             * If not successful (not in PD contract), try Type-C error recovery.
             */
            if (Cy_PdStack_Dpm_SendPdCommand(context, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL) != CY_PDSTACK_STAT_SUCCESS)
            {
                (void)Cy_PdStack_Dpm_SendTypecCommand(context, CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
            }
#endif /* FAULT_RETRY_DELAY_EN */

#if OCP_FAULT_OUT
            if(fault_type == FAULT_TYPE_VBUS_OCP)
            {
                Cy_GPIO_Write(OC_FAULT_OUT_PORT, OC_FAULT_OUT_PIN, 1);
            }
#endif /* OCP_FAULT_OUT */        
        }
    }
    else
    {
#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
        if ((fault_type == ((uint32_t)FAULT_TYPE_VCONN_OCP)) || (fault_type == ((uint32_t)FAULT_TYPE_VCONN_SCP)))
        {
            vconn_change_handler (context, false);
        }
        else
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */
        {
            app_conf_for_faulty_dev_removal(context);

#if FAULT_INFINITE_RECOVERY_EN
            /* 
             * Start a timer to try infinite fault recovery with a timeout.
             */
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext,context, GET_APP_TIMER_ID(context,APP_FAULT_RECOVERY_TIMER),
                    FAULT_INFINITE_RECOVERY_DELAY_MS, fault_delayed_recovery_timer_cb);
#endif /* FAULT_INFINITE_RECOVERY_EN */

#if ((VBUS_OCP_ENABLE) || (VBUS_SCP_ENABLE))
#if RIDGE_SLAVE_ENABLE
            if ((fault_type == FAULT_TYPE_VBUS_OCP) || (fault_type == FAULT_TYPE_VBUS_SCP))
            {
#if ICL_SLAVE_ENABLE
                /* Set into safe state with OCP bit set. */
                Cy_PdAltMode_HW_SetMux(context->ptrAltModeContext, MUX_CONFIG_SAFE, 0x08);
#else
                ridge_slave_set_ocp_status (port);
#endif /* ICL_SLAVE_ENABLE */
            }
#endif /* RIDGE_SLAVE_ENABLE */

#if OCP_FAULT_OUT
            if(fault_type == FAULT_TYPE_VBUS_OCP)
            {
                Cy_GPIO_Write(OC_FAULT_OUT_PORT, OC_FAULT_OUT_PIN, 0);
            }
#endif /* OCP_FAULT_OUT */
#endif /* ((VBUS_OCP_ENABLE) || (VBUS_SCP_ENABLE)) */
        }
    }
    CY_UNUSED_PARAMETER(reason);
}

#endif /* UVP_OVP_RECOVERY || VBUS_OVP_ENABLE || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE \
          || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_SCP_ENABLE || VBUS_UVP_ENABLE */

/* Timer used to re-enable the PD port after a fault. */
static void fault_recovery_timer_cb(cy_timer_id_t id, void *context)
{
    cy_stc_pdstack_context_t *callbackContext = (cy_stc_pdstack_context_t *) context;
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;
    uint8_t port = callbackContext->port;
    (void)id;

    if (
            (vbus_is_present(callbackContext, CY_PD_VSAFE_0V, 0) == false)
       )
    {
        if ((app_get_status(port)->fault_status & ((uint8_t)APP_PORT_VBUS_DROP_WAIT_ACTIVE)) != 0u)
        {
            app_get_status(port)->fault_status &= (~((uint8_t)APP_PORT_VBUS_DROP_WAIT_ACTIVE));

            /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
            Cy_USBPD_TypeC_RdEnable (callbackContext->ptrUsbPdContext);
            period = APP_FAULT_RECOVERY_MAX_WAIT;
        }
        else
        {
            /*
             * If VBus is not detected, we can re-enable the PD port.
             */
            app_get_status(port)->fault_status &= (~((uint8_t)APP_PORT_DISABLE_IN_PROGRESS));
            (void)Cy_PdStack_Dpm_ClearFaultActive(context);

            Cy_USBPD_TypeC_DisableRd(callbackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_1);
            Cy_USBPD_TypeC_DisableRd(callbackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_2);
            (void)Cy_PdStack_Dpm_Start(callbackContext);

            /* Return without restarting the timer. */
            return;
        }
    }

    /* Restart the timer to check VBus and Rp status again. */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (callbackContext->ptrTimerContext, callbackContext, GET_APP_TIMER_ID(callbackContext,APP_FAULT_RECOVERY_TIMER), period, fault_recovery_timer_cb);
}

/* Callback used to get notification that PD port disable has been completed. */
static void app_port_disable_cb(cy_stc_pdstack_context_t * context, cy_en_pdstack_dpm_typec_cmd_resp_t resp)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;
    uint8_t port = context->port;

    if (
            (vbus_is_present(context, CY_PD_VSAFE_0V, 0) == false)
       )
    {
        /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
        Cy_USBPD_TypeC_RdEnable (context->ptrUsbPdContext);
        period = APP_FAULT_RECOVERY_MAX_WAIT;
    }
    else
    {
        /* VBus has not been removed. Start a task which waits for VBus removal. */
        app_get_status(port)->fault_status |= ((uint8_t)APP_PORT_VBUS_DROP_WAIT_ACTIVE);
    }

    /* Provide a delay to allow VBus turn-on by port partner and then enable the port. */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, GET_APP_TIMER_ID(context, APP_FAULT_RECOVERY_TIMER), period, fault_recovery_timer_cb);
    CY_UNUSED_PARAMETER(resp);
}

#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
#if (!CY_PD_SINK_ONLY)
static void src_disable_cbk(cy_stc_pdstack_context_t * context)
{
    CY_UNUSED_PARAMETER(context);
    /* Dummy callback used to ensure VBus discharge happens on CC/SBU OVP. */
}
#endif /* (!CY_PD_SINK_ONLY) */
#endif /* defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1) */

#endif /* FAULT_HANDLER_ENABLE */

/* Clear all fault counters associated with the specified port. */
void fault_handler_clear_counts (cy_stc_pdstack_context_t * context)
{
#if FAULT_HANDLER_ENABLE
    /* Clear all fault counters on disconnect. */
    /* QAC suppression 0312: Pointer to the object is used for write only purpose
     * and will never get optimized. */
    uint8_t port = context->port;
    memset ((uint8_t *)gl_app_fault_count[port], 0u, FAULT_TYPE_COUNT);  /* PRQA S 0312 */
#else
    CY_UNUSED_PARAMETER(context);
#endif /* FAULT_HANDLER_ENABLE */
}

/* Fault-handling specific actions to be performed for various event callbacks. */
bool fault_event_handler(cy_stc_pdstack_context_t * context, cy_en_pdstack_app_evt_t evt, const void *dat)
{
    bool skip_soln_cb = false;
    uint8_t port = context->port;
    app_status_t *app_stat = app_get_status(port);
    (void)app_stat;
    (void)dat;
    (void)evt;

#if FAULT_HANDLER_ENABLE
#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
#endif /* (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_SERIES_WLC1)) */

#if ((VREG_INRUSH_DET_ENABLE) || (VREG_BROWN_OUT_DET_ENABLE))
    uint8_t i;
#endif /* ((VREG_INRUSH_DET_ENABLE) || (VREG_BROWN_OUT_DET_ENABLE)) */

    switch (evt)
    {
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            if (app_port_fault_count_exceeded(context))
            {
                break;
            }
            /* QAC suppression 2003: Fall-through to below case when fault counts are within limits. */
            /* Intentional fall-through. */
        case APP_EVT_DISCONNECT: /* PRQA S 2003 */
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_HARD_RESET_SENT:
            /* Intentional fall-through. */        
            /* Clear the port-in-fault status. */
            if ((app_stat->fault_status & ((uint8_t)APP_PORT_DISABLE_IN_PROGRESS)) == 0u)
            {
                (void)Cy_PdStack_Dpm_ClearFaultActive(context);
            }

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Clear fault counters in cases where an actual disconnect has been detected. */
                fault_handler_clear_counts (context);
            }

#if CCG_HPI_AUTO_CMD_ENABLE
            /* Clear only dynamic fault status, that is Byte 0 */
            gl_app_fault_status[port] &= 0xFF00u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
#if (CCG_CC_SBU_OVP_RETRY_LIMIT != 0)
            /* Clear the CC/SBU OVP fault count if we have successfully negotiated a PD contract. */
            gl_app_fault_count[port][FAULT_TYPE_CC_OVP]  = 0;
            gl_app_fault_count[port][FAULT_TYPE_SBU_OVP] = 0;
#endif /* (CCG_CC_SBU_OVP_RETRY_LIMIT != 0) */
            break;

#if VBUS_OCP_ENABLE
        case APP_EVT_VBUS_OCP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status[port] |= 0x0100u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

            app_handle_fault(context, FAULT_TYPE_VBUS_OCP);
            break;
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
        case APP_EVT_VBUS_SCP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status[port] |= 0x0800u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            app_handle_fault(context, FAULT_TYPE_VBUS_SCP);            
            break;
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
        case APP_EVT_VBUS_RCP_FAULT:
            app_handle_fault(context, FAULT_TYPE_VBUS_RCP);           
            break;
#endif /* VBUS_RCP_ENABLE */

#if VBAT_GND_SCP_ENABLE
        case APP_EVT_VBAT_GND_SCP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status[port] |= 0x2020u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            /* Stop DPM and keep port disabled until recovery */
            (void)Cy_PdStack_Dpm_Stop(context);

#if VBAT_GND_SCP_RECOVERY_ENABLE
            /* 
             * When battery voltage is not in safe range, do a long retries for 
             * only configured recovery time limit and give up.
             * Otherwise, if battery voltage is good, do infinite long 
             * retries with initial few short retries.
             */
            gl_vbat_gnd_scp_status[port] = true;
            app_vbat_gnd_scp_retry_cb(GET_APP_TIMER_ID(context,APP_VBAT_GND_SCP_TIMER_ID), context);
#endif /* VBAT_GND_SCP_RECOVERY_ENABLE */
            break;
#endif /* VBAT_GND_SCP_ENABLE */

#if BB_ILIM_DET_ENABLE
        case APP_EVT_ILIM_FAULT:
            app_handle_fault(context, FAULT_TYPE_ILIM_DET);
            break;
#endif /* BB_ILIM_DET_ENABLE */

#if VREG_INRUSH_DET_ENABLE
        case APP_EVT_VREG_INRUSH_FAULT:
#if defined(CY_DEVICE_CCG7S)
            if(false == app_is_typec_attached())
#endif /* defined(CY_DEVICE_CCG7S) */
            {
                /* Stop DPM and keep both ports disabled until recovery */
                for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
                {
                    cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);
                    (void)Cy_PdStack_Dpm_Stop(pdstack_ctx);
                }
            }

#if defined(CY_DEVICE_CCG7D)
            /*
             * Start a timer to initiate device soft reset after a short delay
             * as Vreg inrush current is not in valid range for firmware to work.
             */
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, GET_APP_TIMER_ID(context, APP_HAL_GENERIC_TIMER),
                APP_VREG_RETRY_TIMER__PERIOD_MS, app_vreg_fault_recovery_retry_cb);
#elif defined(CY_DEVICE_CCG7S)
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER),
                APP_VREG_RETRY_TIMER__PERIOD_MS, app_vreg_inrush_fault_recovery_retry_cb);
#endif /* defined(CY_DEVICE_CCG7D) */
            break;
#endif /* VREG_INRUSH_DET_ENABLE */

#if VREG_BROWN_OUT_DET_ENABLE
        case APP_EVT_VREG_BOD_FAULT:

            /* Stop DPM and keep both ports disabled until recovery */
            for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
            {
                cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);
                (void)Cy_PdStack_Dpm_Stop(pdstack_ctx);
            }

            /* 
             * Start a timer to initiate device soft reset after a short delay
             * as VDDD is not in valid range for firmware to work.
             */
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER),
                APP_VREG_RETRY_TIMER__PERIOD_MS, app_vreg_fault_recovery_retry_cb);

            break;
#endif /* VREG_BROWN_OUT_DET_ENABLE */

#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE)
        case APP_EVT_VBUS_OVP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status[port] |= 0x0200u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            app_handle_fault(context, FAULT_TYPE_VBUS_OVP);
            break;
#endif /* UVP_OVP_RECOVERY || VBUS_OVP_ENABLE */

#if BCR_OTP_ENABLE
        case APP_EVT_TEMPERATURE_FAULT:
            app_handle_fault(context, FAULT_TYPE_OTP);
            break;
#endif /* BCR_OTP_ENABLE */

#if (UVP_OVP_RECOVERY || VBUS_UVP_ENABLE)
        case APP_EVT_VBUS_UVP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status[port] |= 0x0400u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            app_handle_fault(context, FAULT_TYPE_VBUS_UVP);
            break;
#endif /* UVP_OVP_RECOVERY || VBUS_UVP_ENABLE */

#if (!CCG_BACKUP_FIRMWARE)
#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
        case APP_EVT_CC_OVP:
        case APP_EVT_SBU_OVP:
            {
#if (!CCG_VCONN_DISABLE)
                /* Make sure SOURCE/SINK FETs and VConn supply are turned OFF. */
                Cy_USBPD_Vconn_Disable(context->ptrUsbPdContext, ptrDpmConfig->revPol);
#endif /* (!CCG_VCONN_DISABLE) */
#if (!CY_PD_SINK_ONLY)
                /* CC1/2 faults can happen even in detach condition */
                if (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SOURCE)
                {
                    /* Remove the Rp termination and notify the HAL that OVP is pending. */
                    Cy_USBPD_TypeC_DisableRp(context->ptrUsbPdContext, ptrDpmConfig->polarity);
                    Cy_USBPD_Fault_CcOvp_SetPending(context->ptrUsbPdContext);
                    psrc_disable(context, src_disable_cbk);
                }
                else
#endif /* (!CY_PD_SINK_ONLY) */
                {
#if CY_PD_HW_DRP_TOGGLE_ENABLE
                    /* Abort auto toggle if enabled. */
                    Cy_USBPD_TypeC_AbortAutoToggle(context);
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */

#if (!(CY_PD_SOURCE_ONLY))
                    psnk_disable(context, 0);
#endif /* (!(CY_PD_SOURCE_ONLY)) */
                }

#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE)
                /* No need to take new action as long as previous fault handling is still pending. */
                if ((app_stat->fault_status & (APP_PORT_SINK_FAULT_ACTIVE |
                                APP_PORT_DISABLE_IN_PROGRESS)) == 0)
                {
                    app_handle_fault(context, FAULT_TYPE_CC_OVP);
                }
                else
                {
                    skip_soln_cb = true;
                }
#endif /* UVP_OVP_RECOVERY || VBUS_OVP_ENABLE */
            }
            break;
#endif /* defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1)*/
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if (VCONN_OCP_ENABLE && !DISABLE_FOR_CCG4_DOCK)
        case APP_EVT_VCONN_OCP_FAULT:
            app_handle_fault(context, FAULT_TYPE_VCONN_OCP);
            break;
#endif /* VCONN_OCP_ENABLE */

#if VCONN_SCP_ENABLE
        case APP_EVT_VCONN_SCP_FAULT:
            app_handle_fault(context, FAULT_TYPE_VCONN_SCP);
            break;
#endif /* VCONN_SCP_ENABLE */

        default:
            /* No Statement */
            break;
    }
#endif /* FAULT_HANDLER_ENABLE */

    CY_UNUSED_PARAMETER(port);
    return skip_soln_cb;
}

bool fault_handler_init_vars (cy_stc_pdstack_context_t * context)
{
#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_SCP_ENABLE || BB_ILIM_DET_ENABLE || VREG_INRUSH_DET_ENABLE || VREG_BROWN_OUT_DET_ENABLE)
    cy_stc_usbpd_config_t * fault_config = context->ptrUsbPdContext->usbpdConfig;
    uint8_t port = context->port;
#else
    CY_UNUSED_PARAMETER(context);
#endif /*(UVP_OVP_RECOVERY || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE || VCONN_SCP_ENABLE || BB_ILIM_DET_ENABLE || VREG_INRUSH_DET_ENABLE || VREG_BROWN_OUT_DET_ENABLE) */

#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE)
    if (fault_config->vbusOvpConfig == NULL  && get_pd_port_config(context->ptrUsbPdContext)->port_n_ovp_table_offset == 0)
    {
        return false;
    }
#if (BCR)
    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = app_get_status(port)->bcr_ovp.cfg.retry_cnt;
#else
    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->retryCount;
#endif /* BCR */
#endif /* UVP_OVP_RECOVERY || VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE
    if (fault_config->vbusOcpConfig == NULL && get_pd_port_config(context->ptrUsbPdContext)->port_n_ocp_table_offset == 0)
    {
        return false;
    }
#if (BCR)
    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OCP] = app_get_status(port)->bcr_ocp.cfg.retry_cnt;
#else
    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OCP] = GET_VBUS_OCP_TABLE(context->ptrUsbPdContext)->retryCount;
#endif /* BCR */
#endif /* VBUS_OCP_ENABLE */

#if VBUS_RCP_ENABLE
    if (fault_config->vbusRcpConfig == NULL && get_pd_port_config(context->ptrUsbPdContext)->port_n_rcp_table_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_RCP] = GET_VBUS_RCP_TABLE(context->ptrUsbPdContext)->retryCount;
#endif /* VBUS_RCP_ENABLE */

#if (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE)
    if (fault_config->vbusUvpConfig == NULL && get_pd_port_config(context->ptrUsbPdContext)->port_n_uvp_table_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_UVP] = GET_VBUS_UVP_TABLE(context->ptrUsbPdContext)->retryCount;
#endif /* (UVP_OVP_RECOVERY || VBUS_OVP_ENABLE) */

#if VBUS_SCP_ENABLE
    if (fault_config->vbusScpConfig == NULL && get_pd_port_config(context->ptrUsbPdContext)->port_n_scp_table_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_SCP] = 0;
#endif /* VBUS_SCP_ENABLE */

#if VCONN_OCP_ENABLE
    if (fault_config->vconnOcpConfig == NULL && get_pd_port_config(context->ptrUsbPdContext)->port_n_vconn_ocp_table_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VCONN_OCP] = GET_VCONN_OCP_TABLE(context->ptrUsbPdContext)->retryCount;
#endif /* VCONN_OCP_ENABLE */

#if VCONN_SCP_ENABLE
    gl_app_fault_retry_limit[port][FAULT_TYPE_VCONN_SCP] = VCONN_SCP_RETRY_COUNT;
#endif /* VCONN_SCP_ENABLE */

#if BB_ILIM_DET_ENABLE
    /* Set Infinite retry for iLim fault */
    gl_app_fault_retry_limit[port][FAULT_TYPE_ILIM_DET] = FAULT_COUNTER_SKIP_VALUE;
#endif /* BB_ILIM_DET_ENABLE */

#if VREG_INRUSH_DET_ENABLE
    /* Set Infinite retry for Inrush Detect */
    gl_app_fault_retry_limit[port][FAULT_TYPE_VREG_INRSH_DET] = FAULT_COUNTER_SKIP_VALUE;
#endif /* VREG_INRUSH_DET_ENABLE */

#if VREG_BROWN_OUT_DET_ENABLE
    /* Set Infinite retry for Vddd brown out fault */
    gl_app_fault_retry_limit[port][FAULT_TYPE_VREG_BOD] = FAULT_COUNTER_SKIP_VALUE;
#endif /* BB_ILIM_DET_ENABLE */

#if (CCG_CC_SBU_OVP_RETRY_LIMIT != 0)
    /* Set the retry limit where it is non-zero. */
    gl_app_fault_retry_limit[port][FAULT_TYPE_CC_OVP]  = CCG_CC_SBU_OVP_RETRY_LIMIT;
    gl_app_fault_retry_limit[port][FAULT_TYPE_SBU_OVP] = CCG_CC_SBU_OVP_RETRY_LIMIT;
#endif /* (CCG_CC_SBU_OVP_RETRY_LIMIT != 0) */

    return true;
}

void fault_handler_register_cbks (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_usbpd_context_t *usbpd_ctx = ptrPdStackContext->ptrUsbPdContext;

    usbpd_ctx->vbatGndScpCb = pd_vbat_gnd_scp_cbk;
    usbpd_ctx->bbIlimCbk = pd_bb_ilim_fault_handler;
    usbpd_ctx->vregInrushCbk = pd_vreg_inrush_det_fault_handler;
    usbpd_ctx->bodCbk = pd_brown_out_fault_handler;
}

void fault_handler_task(cy_stc_pdstack_context_t * context)
{
#if FAULT_HANDLER_ENABLE
    uint8_t port = context->port;
    app_status_t *app_stat = app_get_status(port);

    /*
     * If SINK fault handling is pending, queue a port disable command.
     */
    if((app_stat->fault_status & ((uint8_t)APP_PORT_SINK_FAULT_ACTIVE)) != 0u)
    {
        if (Cy_PdStack_Dpm_SendTypecCommand (context, CY_PDSTACK_DPM_CMD_PORT_DISABLE, app_port_disable_cb) != CY_PDSTACK_STAT_BUSY)
        {
            app_stat->fault_status &= (~((uint8_t)APP_PORT_SINK_FAULT_ACTIVE));
            app_stat->fault_status |= ((uint8_t)APP_PORT_DISABLE_IN_PROGRESS);
        }
    }
#else
    CY_UNUSED_PARAMETER(context);
#endif /* FAULT_HANDLER_ENABLE */
}

#if VBUS_OVP_ENABLE

#define MAX_OVP_DEBOUNCE_CYCLES         (0x20u)

/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(cy_stc_pdstack_context_t * context, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t ovp_cb)
{
#if VBUS_OVP_ADC_MODE_SUPPORTED
    uint8_t level;
    uint8_t threshold;
#endif /* VBUS_OVP_ADC_MODE_SUPPORTED */

    uint32_t intr_state;

    if (GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->enable)
    {
        intr_state = Cy_SysLib_EnterCriticalSection();

        if (GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->mode == CY_USBPD_VBUS_OVP_MODE_ADC)
        {
#if VBUS_OVP_ADC_MODE_SUPPORTED
            threshold = GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->threshold;

#if ADC_FALSE_OVP_FIX
            /* Make sure threshold is set to a suitable value at low voltages to avoid false OVP trips. */
            if (apply_threshold(volt_mV, threshold) < ADC_VBUS_MIN_OVP_LEVEL)
            {
                volt_mV   = ADC_VBUS_MIN_OVP_LEVEL;
                threshold = 0;
            }
#endif /* ADC_FALSE_OVP_FIX */

            /* Set OVP threshold. */
            level = Cy_USBPD_Adc_GetVbusLevel(context->ptrUsbPdContext, APP_OVP_ADC_ID, volt_mV, threshold);
            Cy_USBPD_Adc_CompCtrl(context->ptrUsbPdContext, APP_OVP_ADC_ID, APP_OVP_ADC_INPUT, level, CY_USBPD_ADC_INT_FALLING, ovp_cb);
#endif /* VBUS_OVP_ADC_MODE_SUPPORTED */
        }
        else /* (VBUS_OVP_MODE != VBUS_OVP_MODE_ADC) */
        {
#if (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined (CY_DEVICE_CCG5) || \
        defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            Cy_USBPD_Fault_Vbus_OvpEnable(context->ptrUsbPdContext, volt_mV, ovp_cb, pfet);
#endif /* CCGx */
        }

        Cy_SysLib_ExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(cy_stc_pdstack_context_t * context, bool pfet)
{
    if (GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->enable)
    {
        /* Disable OVP. */
        if (GET_VBUS_OVP_TABLE(context->ptrUsbPdContext)->mode == CY_USBPD_VBUS_OVP_MODE_ADC)
        {
#if VBUS_OVP_ADC_MODE_SUPPORTED
            Cy_USBPD_Adc_CompCtrl(context->ptrUsbPdContext, APP_OVP_ADC_ID, 0, 0, 0, NULL);
#endif /* VBUS_OVP_ADC_MODE_SUPPORTED */
        }
        else
        {
#if (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined (CCG5) ||\
        defined(CY_DEVICE_CCG6) || defined(CCG5C) || defined(PAG1S) || defined(CCG6DF) || defined(CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            Cy_USBPD_Fault_Vbus_OvpDisable(context->ptrUsbPdContext, pfet);
#endif /* CCGx */
        }
    }
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
#if CCG4_DOCK
extern void dock_uvp_check (uint8_t port, cy_timer_id_t id);
#endif /* CCG4_DOCK */

/* Configure Under-Voltage Protection checks based on parameters in config table. */
void app_uvp_enable(cy_stc_pdstack_context_t * context, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t uvp_cb)
{
    uint32_t intr_state;
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);

    if (GET_VBUS_UVP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
        intr_state = Cy_SysLib_EnterCriticalSection ();

#if CY_PD_PPS_SRC_ENABLE
        if (dpm_stat->srcSelPdo.fixed_src.supplyType == CY_PDSTACK_PDO_AUGMENTED)
        {
            /*
             * In PPS mode operation, UVP is not an unrecoverable event. It
             * needs to be dealt with a simple hardreset. Configure for non-
             * hardware cutoff operation. NOTE: The threshold for operation
             * can be overridden to set the cut-off based on system requirement.
             * Currently using the lowest UVP point for this.
             */
            volt_mV = ((uint16_t)dpm_stat->srcSelPdo.pps_src.minVolt) * CY_PD_VOLT_PER_UNIT_PPS;
            Cy_USBPD_Fault_Vbus_UvpEnable(context->ptrUsbPdContext, volt_mV,uvp_cb, pfet);
        }
        else
#endif /* CY_PD_PPS_SRC_ENABLE */
        {
#if CCG4_DOCK
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext,context,VBUS_UVP_TIMER_ID, VBUS_UVP_TIMER_PERIOD, dock_uvp_check);
#else /* CCG4_DOCK */
            Cy_USBPD_Fault_Vbus_UvpEnable(context->ptrUsbPdContext, volt_mV, uvp_cb, pfet);
#endif /* CCG4_DOCK */
        }

        Cy_SysLib_ExitCriticalSection (intr_state);
    }
}

void app_uvp_disable(cy_stc_pdstack_context_t * context, bool pfet)
{

    /* Disable UVP. */
    if (GET_VBUS_UVP_TABLE(context->ptrUsbPdContext)->enable != 0u)
    {
#if CCG4_DOCK
        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, VBUS_UVP_TIMER_ID);
#else /* CCG4_DOCK */
        Cy_USBPD_Fault_Vbus_UvpDisable(context->ptrUsbPdContext, pfet);
#endif /* CCG4_DOCK */
    }
}

#endif /* VBUS_UVP_ENABLE */

#if OTP_ENABLE

#if !INTERNAL_BJT_BASED_OTP

/* Globals to keep track of OTP condition. */
static uint8_t gl_otp_therm_type[NO_OF_TYPEC_PORTS][THERMISTOR_COUNT] = {{APP_THERMISTOR_TYPE_ERROR}};
static bool gl_otp_port_disable[NO_OF_TYPEC_PORTS] = {false};
/* Global to keep track of thermistors in OTP condition */
static uint8_t gl_therm_in_otp = 0x00;
static uint16_t gl_otp_sys_debounce_count = 0;
static uint16_t gl_therm_debounce_count[THERMISTOR_COUNT] = {0};

static void app_otp_init(cy_stc_pdstack_context_t * context)
{
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    uint8_t port=context->port;
    /* Initialization for port specific instance. */
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + PD_GET_PTR_OTP_TBL(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */

    uint8_t therm_type = PD_GET_PTR_OTP_TBL(port)->therm_type;
    gl_otp_port_disable[port] = false;
    gl_otp_sys_debounce_count = 0;
    gl_therm_in_otp = 0x00;
    for(index = 0; index < active_thermistors; index++)
    {
        gl_therm_debounce_count[index] = 0;
    }

    if(0x00 == therm_type)
    {
        gl_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_NTC;
    }
    else if(0x01 == (therm_type & 0x01))
    {
        gl_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_PTC;
    }
    else if(0x02 == (therm_type & 0x02))
    {
        gl_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_INTRNL;
    }
    else
    {
        gl_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_ERROR;
    }
#if (THERMISTOR_COUNT > 1)

    if(PD_GET_PTR_OTP_TBL(port)->therm_1_enable)
    {
        therm_type = PD_GET_PTR_OTP_TBL(port)->therm_type_1;
        if(0x00 == therm_type)
        {
            gl_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_NTC;
        }
        else if(0x01 == (therm_type & 0x01))
        {
            gl_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_PTC;
        }
        else if(0x02 == (therm_type & 0x02))
        {
            gl_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_INTRNL;
        }
        else
        {
            gl_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_ERROR;
        }
    }
#endif /* (THERMISTOR_COUNT > 1) */
}

/* This function will return the current temperature of the system */
static uint16_t app_otp_get_sys_temp(cy_stc_pdstack_context_t * context, uint8_t therm_id)
{  
    uint8_t level;
    uint32_t intr_state;
    uint16_t therm_volt;
    intr_state = Cy_SysLib_EnterCriticalSection();
    /* We need to drive GPIO1 high in order to read the thermistor value */
#ifdef CY_DEVICE_PAG1S
    uint8_t gpio = OTP_THERMISTOR_GPIO_0_P1;
#ifdef CY_PINS_THERM_DRIVE_H
    gpio_set_value(GPIO_PORT_0_PIN_1, true);
    if(therm_id == 1)
    {    
        gpio = OTP_THERMISTOR_GPIO_1_P1;
    }
#endif /* CY_PINS_THERM_DRIVE_H */

    /*
     * Configure GPIO. As per PAG1S device limitation, only AMUX_B can be used when EA
     * operational to avoid any noise on EA. Since AMUX_B for ADC is internally connected
     * to VBUS_MON, we need to first change to AMUX and then measure. Once done, this
     * has to be reverted.
     */
    pd_disconnect_vbus_div_from_amux(port);
#ifdef CCG3PA_NFET
    hsiom_set_config(gpio, THERMISTOR_GPIO_HSIOM_MODE);
    CyDelayUs(20);
    /* Take ADC sample. */
    pd_adc_select_vref(port,APP_THERMISTOR_POLL_ADC_ID,PD_ADC_VREF_VDDD);
    level = pd_adc_sample(port, APP_THERMISTOR_POLL_ADC_ID, PD_ADC_INPUT_AMUX_B);
    /*
     * Using custom level-voltage conversion logic - The entry to the config table for cut-off
     * and restart may have been calculated using a different nominal value for Vddd. We need to scale
     * the Vddd to match the voltage values mentioned via the configuration utility
     */
    therm_volt = ((level * OTP_VDDD_REFERENCE)/PD_ADC_NUM_LEVELS);
    pd_adc_select_vref(port,APP_THERMISTOR_POLL_ADC_ID,PD_ADC_VREF_PROG);
    hsiom_set_config(gpio, HSIOM_MODE_GPIO);
    pd_connect_vbus_div_to_amux(port);
#else
    hsiom_set_config(gpio, HSIOM_MODE_AMUXB);
    CyDelayUs(20);
    /* Take ADC sample. */
    pd_adc_select_vref(port,PD_ADC_ID_0,PD_ADC_VREF_VDDD);
    level = pd_adc_sample(port, PD_ADC_ID_0, PD_ADC_INPUT_AMUX_B);
    /*
     * Using custom level-voltage conversion logic - The entry to the config table for cut-off
     * and restart may have been calculated using a different nominal value for Vddd. We need to scale
     * the Vddd to match the voltage values mentioned via the configuration utility
     */
    therm_volt = ((level * OTP_VDDD_REFERENCE)/PD_ADC_NUM_LEVELS);
    pd_adc_select_vref(port,PD_ADC_ID_0,PD_ADC_VREF_PROG);
    hsiom_set_config(gpio, HSIOM_MODE_GPIO);
    pd_connect_vbus_div_to_amux(port);
#endif /* CCG3PA_NFET */
#ifdef CY_PINS_THERM_DRIVE_H
    /* We need to drive the GPIO 1 low in order to conserve power */    
    gpio_set_value(GPIO_PORT_0_PIN_1, false);
#endif /* CY_PINS_THERM_DRIVE_H */    

#else /* CCG3PA */ 
    hsiom_set_config(OTP_THERM_GPIO, THERMISTOR_GPIO_HSIOM_MODE);
    CyDelayUs(20);
    pd_adc_select_vref(port,APP_THERMISTOR_POLL_ADC_ID,PD_ADC_VREF_VDDD);
    /* Take ADC sample. */
    level = pd_adc_sample (port, APP_THERMISTOR_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);
    therm_volt = ((level * OTP_VDDD_REFERENCE)/PD_ADC_NUM_LEVELS);
    pd_adc_select_vref(port,APP_THERMISTOR_POLL_ADC_ID,PD_ADC_VREF_PROG);
    hsiom_set_config(OTP_THERM_GPIO, HSIOM_MODE_GPIO);
#endif /* PAG1S */   
    CyExitCriticalSection(intr_state);
    return therm_volt;
}    

static bool otp_restart_port(uint8_t port, uint16_t therm_volt, uint8_t therm_id)
{
    uint16_t restart_val = PD_GET_PTR_OTP_TBL(port)->restart_val;
#if (THERMISTOR_COUNT > 1)
    if(therm_id == 0x01)
    {
        restart_val = PD_GET_PTR_OTP_TBL(port)->restart_val_1;
    }
#endif /* (THERMISTOR_COUNT > 1) */

    return (((gl_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_NTC) &&
                (therm_volt >= restart_val)) ||
            (((gl_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_PTC) &&
              (therm_volt <= restart_val))));
}

static bool otp_is_ot_exist(uint8_t port, uint16_t therm_volt, uint8_t therm_id)
{
    uint16_t cutoff_val = PD_GET_PTR_OTP_TBL(port)->cutoff_val;
#if (THERMISTOR_COUNT > 1)
    if(therm_id == 0x01)
    {
        cutoff_val = PD_GET_PTR_OTP_TBL(port)->cutoff_val_1;
    }
#endif /* (THERMISTOR_COUNT > 1) */

    return ((gl_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_NTC) &&
            (therm_volt <= cutoff_val)) ||
        (((gl_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_PTC) &&
          (therm_volt >= PD_GET_PTR_OTP_TBL(port)->cutoff_val)));
}

#if OTP_VSAFE_5V_ENABLE
#if (!CY_PD_SINK_ONLY)
/* Source Rp current value backup */
static uint8_t gl_src_cur_level[NO_OF_TYPEC_PORTS];

/* Source PDO backup */
static pd_do_t gl_src_pdo_backup[NO_OF_TYPEC_PORTS][MAX_NO_OF_PDO];

/* Source PDO count backup */
static uint8_t gl_src_pdo_count_backup[NO_OF_TYPEC_PORTS];

#if (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE)
/* Source Legacy charging configuration backup */
static chg_cfg_params_t gl_src_bc_backup[NO_OF_TYPEC_PORTS];
#endif /* (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE) */

/* 5V@900mA PDO */
static pd_do_t srcPdo[1]= {{0x0A01905A}};
#endif /* !CY_PD_SINK_ONLY */
#endif /* OTP_VSAFE_5V_ENABLE */

static void otp_debounce_cb(uint8_t port, timer_id_t id)
{
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    uint16_t therm_volt;
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + PD_GET_PTR_OTP_TBL(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */
#if OTP_VSAFE_5V_ENABLE
#if (!CY_PD_SINK_ONLY)
    const dpm_status_t *dpm_stat = dpm_get_info (port);
#if (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE)
    chg_cfg_params_t chg_params;
#endif /* (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE) */
    uint8_t i = 0;
#endif /* !CY_PD__SINK_ONLY */
#endif /* OTP_VSAFE_5V_ENABLE */

    if(gl_otp_port_disable[port] == false)
    {
        for(index = 0; index < active_thermistors; index++)
        {
            therm_volt = app_otp_get_sys_temp(port, index);
            if(true == otp_is_ot_exist(port, therm_volt, index))
            {
                gl_therm_debounce_count[index] ++;
            }
        }
        if (pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext)->enable)
        {
            gl_otp_sys_debounce_count++;
            if(gl_otp_sys_debounce_count > PD_GET_PTR_OTP_TBL(port)->debounce)
            {
                for(index = 0; index < active_thermistors; index++)
                {
                    if(gl_therm_debounce_count[index] > PD_GET_PTR_OTP_TBL(port)->debounce)
                    {
                        /* Here all thermistors which show OTP condition after debounce are set */
                        gl_therm_in_otp |= (0x01 << index);
                    }    
                }
                if(gl_therm_in_otp != 0x00)
                {
                    /* Clear all the counts here */
                    for(index=0; index < active_thermistors; index++)
                    {
                        gl_therm_debounce_count[index] = 0;
                    }
                    gl_otp_sys_debounce_count = 0;
                    gl_otp_port_disable[port] = true;

#if OTP_VSAFE_5V_ENABLE
#if (!CY_PD_SINK_ONLY)
                    if(dpm_stat->curPortRole == CY_PD_PRT_ROLE_SOURCE)
                    {
                        /* Report fault to DPM */
                        dpm_set_fault_active(port);

                        /* Set alert message */
                        pd_do_t alert;
                        alert.val = 0;
                        alert.ado_alert.otp = true;
                        dpm_set_alert(port, alert);

                        /* 
                         * Take a backup of current SRC_CAP info.
                         * And, update SRC_CAP to have only 5V@900mA PDO until
                         * OTP condition persists.
                         */
                        gl_src_pdo_count_backup[port] = dpm_stat->srcPdoCount;

                        for (i = 0; i < gl_src_pdo_count_backup[port]; i++)
                        {
                            gl_src_pdo_backup[port][i] = dpm_stat->srcPdo[i];
                        }
                        dpm_update_src_cap(port, 1, srcPdo);
                        dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, NULL, NULL);

                        /* 
                         * Take a backup of source current level (Rp) for Type-C only.
                         * And, update source current level to default Rp (900mA)
                         * OTP condition persists.
                         */
                        gl_src_cur_level[port] = dpm_stat->srcCurLevel;
                        dpm_typec_command(port, DPM_CMD_SET_RP_DFLT, NULL);

#if (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE)
                        /* 
                         * Take a backup of legacy charging config.
                         * And, disable legacy charging.
                         */
                        memcpy((uint8_t *)(&gl_src_bc_backup[port]), (uint8_t *)bc_get_config(port),
                            sizeof (chg_cfg_params_t));
                        chg_params = gl_src_bc_backup[port];
                        chg_params.src_sel = 0;
                        bc_stop(port);
                        bc_set_config(port, &chg_params);
                        bc_start(port, BC_PORT_SOURCE);
#endif /* (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE) */

                        /*
                         * Try a Hard Reset to reset port.
                         * If not successful (not in PD contract), try Type-C error recovery.
                         */
                        dpm_clear_hard_reset_count(port);

                        if (dpm_send_hard_reset (port, PD_HARDRES_REASON_NONE) != CCG_STAT_SUCCESS)
                        {
                            dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
                        }
                    }
                    else
                    {
                        (void)dpm_stop(port);
                    }
#endif /* (!CY_PD_SINK_ONLY) */
#else /* !OTP_VSAFE_5V_ENABLE */
                    (void)dpm_stop(port);
#endif /* OTP_VSAFE_5V_ENABLE */

                    app_event_handler(context, APP_EVT_SYSTEM_OT_FAULT, NULL);
                }
            }
            else
            {
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            } 
        }
    }
    else if(gl_otp_port_disable[port] == true)
    {
        /* We are currently in OTP condition - check if all thermistors are out of OTP */
        for(index = 0; index < active_thermistors; index++)
        {
            therm_volt = app_otp_get_sys_temp(port, index);
            if(true != otp_restart_port(port, therm_volt,index))
            {
                /* If atleast one thermistor still remains in otp - exit */
                break;
            }
            gl_therm_debounce_count[index]++;
        }
        if(index == active_thermistors)
        {
            if (pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext)->enable)
            {
                gl_otp_sys_debounce_count++;
                if(gl_otp_sys_debounce_count > PD_GET_PTR_OTP_TBL(port)->debounce)
                {
                    for(index = 0; index < active_thermistors; index++)
                    {
                        if(gl_therm_debounce_count[index] > PD_GET_PTR_OTP_TBL(port)->debounce)
                        {
                            gl_therm_in_otp &= ~(index+1);
                        }
                    }
                }
                else
                {
                    timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
                }

                if(gl_therm_in_otp == 0x00)
                {
#if OTP_VSAFE_5V_ENABLE
#if (!CY_PD_SINK_ONLY)
                    if(dpm_stat->curPortRole == CY_PD_PRT_ROLE_SOURCE)
                    {
                        /* If no other fault is active, clear fault to DPM */
                        if (!app_port_fault_count_exceeded(port))
                        {
                            if ((app_get_status(port)->fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                            {
                                dpm_clear_fault_active(port);
                            }
                        }

                        /* Restore SRC_CAP. */
                        dpm_update_src_cap(port, gl_src_pdo_count_backup[port], gl_src_pdo_backup[port]);
                        dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, NULL, NULL);

                        /* Restore Rp current level */
                        dpm_typec_command(port, gl_src_cur_level[port], NULL);

#if (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE)
                        /* Restore legacy charging */
                        bc_stop(context);
                        bc_set_config(context, (cy_stc_legacy_charging_cfg_t*)&gl_src_bc_backup);
                        bc_start(context, BC_PORT_SOURCE);
#endif /* (BATTERY_CHARGING_ENABLE && LEGACY_DYN_CFG_ENABLE) */
                    }
                    else
                    {
                        (void)dpm_start(port);
                    }
#endif /* !CY_PD_SINK_ONLY */
#else /* !OTP_VSAFE_5V_ENABLE */
                    (void)dpm_start(port);
#endif /* OTP_VSAFE_5V_ENABLE */
                    gl_otp_port_disable[port] = false;
                    app_otp_enable(port);
                }
            }
            else
            {
                /* OTP feature can get disabled while debouncing is in progress.
                 * There is no point continuing OTP checking in this case.
                 * This is equivalent to no OTP condition.
                 */
                app_otp_init(port);
            }
        }
    }
}

void app_otp_check_temp(cy_stc_pdstack_context_t * context)
{
    uint8_t ot_exist = 0;
    uint8_t port=context->port;
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    uint16_t therm_volt;
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + PD_GET_PTR_OTP_TBL(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */

    /* Check if OTP has been enabled in the config table */
    if(pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext)->enable)
    {
        if(gl_otp_port_disable[port] == false)
        {
            /* Check thermistors to see whether any are in OTP */
            for(index = 0; index < active_thermistors; index++)
            {
                therm_volt = app_otp_get_sys_temp(port, index);
                ot_exist |= otp_is_ot_exist(port, therm_volt, index);    
            }
            if(ot_exist != 0)
            {
                /* Initialize all counters to 0 and start debounce */
                for(index=0; index < active_thermistors; index++)
                {
                    gl_therm_debounce_count[index] = 0;
                }
                gl_otp_sys_debounce_count = 0;
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
        else if (gl_otp_port_disable[port] == true)    
        {
            for(index = 0; index < active_thermistors; index++)
            {
                therm_volt = app_otp_get_sys_temp(port, index);
                if(true != otp_restart_port(port, therm_volt, index))
                {
                    break;
                }
            }
            if(index == active_thermistors)
            {
                /* All thermistors are out of OTP condition - start debounce */
                for(index=0; index < active_thermistors; index++)
                {
                    gl_therm_debounce_count[index] = 0;
                }
                gl_otp_sys_debounce_count = 0;
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
    }
}

#else /* INTERNAL_BJT_BASED_OTP */

static void ccg6f_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    /* Nothing to do here. */
}

static void ccgx_internal_bjt_debounce_cb(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);    
    uint8_t temperature = pd_hal_measure_internal_temp(port);

    if(app_stat->is_hot_shutdown == false)
    {
        /* If debounce task was initiated on detecting temperature going above cut-off 
         * Sample the temperature reading again to prevent shutdown on faulty readings.
         */
        if(temperature >= app_stat->turn_off_temp_limit)
        {
            gl_otp_debounce_count[port]++;
            if(pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext)->enable != 0u)
            {
                if(gl_otp_debounce_count[port] > PD_GET_PTR_OTP_TBL(port)->debounce)
                {
                    /* Turn off the FETs. */
                    psrc_disable (context, NULL);
#if !CY_PD_SOURCE_ONLY
                    psnk_disable (context, NULL);
#endif /* !CCG_SOURCE_ONLY */    

                    /* Send OT notifications to the solution. */
                    app_event_handler (port, APP_EVT_TEMPERATURE_FAULT, 0);

                    /* Set fault active flag and disable the port. */
                    gl_otp_debounce_count[port] = 0;
                    app_stat->is_hot_shutdown = true;
                    dpm_set_fault_active (port);
                    dpm_stop(port);
                }
                else
                {
                    timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, 
                                                        ccgx_internal_bjt_debounce_cb);                    
                }
            }
        }
    }
    else
    {
        if(temperature < app_stat->turn_on_temp_limit)
        {
            gl_otp_debounce_count[port]++;
            if(pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext)->enable != 0u)
            {
                if(gl_otp_debounce_count[port] > PD_GET_PTR_OTP_TBL(port)->debounce)
                {
                    /* Clear the fault condition and re-enable the port. */
                    gl_otp_debounce_count[port] = 0;
                    app_stat->is_hot_shutdown = false;
                    dpm_clear_fault_active (port);
                    dpm_start (port);
                }
                else
                {
                    timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, 
                                                        ccgx_internal_bjt_debounce_cb);                    
                }                
            }        
        }
    }
}

/* OT monitor callback function. */
static void ccgx_internal_bjt_monitor_cbk(uint8_t port, timer_id_t id)
{
    uint8_t temperature = 0;
    app_status_t* app_stat = app_get_status(port);
    temperature = pd_hal_measure_internal_temp(port);
    /* Check if over-temperature condition. */
    if (
            (temperature >= app_stat->turn_off_temp_limit) &&
            (app_stat->is_hot_shutdown == false)
       )
    {
        gl_otp_debounce_count[port] = 0;        
        timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, ccgx_internal_bjt_debounce_cb);
    }
    
    if (
            (app_stat->is_hot_shutdown == true) &&
            (temperature < app_stat->turn_on_temp_limit)
       )
    {
        gl_otp_debounce_count[port] = 0;
        timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, ccgx_internal_bjt_debounce_cb);
    }
    /* Re-start timer to keep monitoring temperature of IC */
    timer_start(port, APP_OT_DETECTION_TIMER, APP_OT_DETECTION_TIMER_PERIOD, ccgx_internal_bjt_monitor_cbk);    
}
#endif

void app_otp_enable(uint8_t port)
{
#if !INTERNAL_BJT_BASED_OTP
    /*
     * If port is < NO_OF_TYPEC_PORTS, individual port specific data structure
     * instances are updated. Port = NO_OF_TYPEC_PORTS is used for updating
     * data structures for all ports at 1 shot. This is used during
     * initialization.
     */
    if (NO_OF_TYPEC_PORTS == port)
    {
        /* This has been called from initialization. We need to do the data
           structure initialization for all port instances. */
        for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            app_otp_init(port);
        }
    }
    else
    {
        /* Initialization for port specific instance. */
        app_otp_init(port);
    }
#else /* INTERNAL_BJT_BASED_OTP */

    otp_settings_t *ot_settings = pd_get_ptr_otp_tbl(ptrPdStackContext->ptrUsbPdContext);
    app_status_t *app_stat = app_get_status(port);
    uint8_t idx;

    /* Check OTP parameters in the config table. */
    if (((otp_settings->enable) != 0) &&
                (ot_settings->therm_type == APP_THERMISTOR_TYPE_INTRNL ))
    {   
        app_stat->turn_off_temp_limit = ot_settings->cutoff_val;
        app_stat->turn_on_temp_limit = ot_settings->restart_val;

        /* Start timer to monitor temperature of IC */
        app_status[port].is_hot_shutdown = false;
        for(idx = 0; idx < NO_OF_TYPEC_PORTS; idx++)
        {
            /* Clear all the OTP global variables */
            gl_otp_debounce_count[idx] = 0;
        }
        timer_start(port, APP_OT_DETECTION_TIMER, APP_OT_DETECTION_TIMER_PERIOD, ccgx_internal_bjt_monitor_cbk);        
    }
#endif /* INTERNAL_BJT_BASED_OTP */
}

bool app_otp_status(uint8_t port)
{
#if !INTERNAL_BJT_BASED_OTP
    return gl_otp_port_disable[port];
#else /* INTERNAL_BJT_BASED_OTP */
    return false;
#endif /* !INTERNAL_BJT_BASED_OTP */
}

#endif /* OTP_ENABLE */

/* Callback to handle VBAT_GND SCP event */
void pd_vbat_gnd_scp_cbk(void *callbackContext, bool state)
{
#if VBAT_GND_SCP_ENABLE
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpd_ctx->pdStackContext;
    CY_UNUSED_PARAMETER(state);

    /* Disable the port */
    if (context->ptrAppCbk != NULL)
    {
        context->ptrAppCbk->psrc_disable(context, NULL);
    }

    /*
     * Fault is persistent and hence fault GPIO shall
     * keep the GND FET off until the fault enable is invoked again.
     */

    Cy_USBPD_Fault_VbatGndScpDis(context->ptrUsbPdContext, CCG_SRC_FET);

    /* Enqueue HPI SCP fault event. */
    app_event_handler(context, APP_EVT_VBAT_GND_SCP_FAULT, NULL);
#else /* VBAT_GND_SCP_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
#endif /* VBAT_GND_SCP_ENABLE */
}

void pd_bb_ilim_fault_handler(void *callbackContext, bool state)
{
#if BB_ILIM_DET_ENABLE
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackContext;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)(usbpd_ctx->pdStackContext);
    CY_UNUSED_PARAMETER(state);

    /* Enqueue fault event. */
    app_event_handler(context, APP_EVT_ILIM_FAULT, NULL);
#else /* BB_ILIM_DET_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
#endif /* BB_ILIM_DET_ENABLE */
}

/* Function to handle buck-boost iLim fault */
void pd_brown_out_fault_handler(void *callbackContext, bool state)
{
#if VREG_BROWN_OUT_DET_ENABLE
    uint8_t i;
    cy_stc_pdstack_context_t *context_0;
#if defined(CY_DEVICE_CCG7D)
    cy_stc_pdstack_context_t *context_1;
#endif /* defined(CY_DEVICE_CCG7D) */
    cy_stc_pdstack_context_t *context;

    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        context = solution_fn_handler->Get_PdStack_Context(i);
        /* Disable the port. */
        if (context->ptrAppCbk != NULL)
        {
            context->ptrAppCbk->psrc_disable(context, NULL);
        }

#if VREG_INRUSH_DET_ENABLE
        /* Disable inrush fault timer */
        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_VREG_TIMER));
#endif /* VREG_INRUSH_DET_ENABLE */
    }

    context_0 = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_0_IDX);
#if defined(CY_DEVICE_CCG7D)
    context_1 = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_1_IDX);
#endif /* defined(CY_DEVICE_CCG7D) */

#if defined(CY_DEVICE_CCG7D)
    /* Disable interrupt handling until recovery is tried */
    Cy_USBPD_Fault_BrownOutDetDis(context_1->ptrUsbPdContext);
#if VREG_INRUSH_DET_ENABLE
    /* Disable inrush fault also as both are related to vreg */
    Cy_USBPD_Fault_VregInrushDetDis(context_1->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#else
    
    /* Disable interrupt handling until recovery is tried */
    Cy_USBPD_Fault_BrownOutDetDis(context_0->ptrUsbPdContext);
#if VREG_INRUSH_DET_ENABLE
    /* Disable inrush fault also as both are related to vreg */
    Cy_USBPD_Fault_VregInrushDetDis(context_0->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#endif /* defined(CY_DEVICE_CCG7D) */

    /* Enqueue fault event. */
    app_event_handler(context_0, APP_EVT_VREG_BOD_FAULT, NULL);
#else /* VREG_BROWN_OUT_DET_ENABLE */
    /* No statement */
#endif /* VREG_BROWN_OUT_DET_ENABLE */
    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
}

/* Function to handle Vreg inrush detection fault */
void pd_vreg_inrush_det_fault_handler(void *callbackContext, bool state)
{
#if VREG_INRUSH_DET_ENABLE
    uint8_t i;
    cy_stc_pdstack_context_t *context_0;
#if defined(CY_DEVICE_CCG7D)
    cy_stc_pdstack_context_t *context_1;
#endif /* defined(CY_DEVICE_CCG7D) */
    cy_stc_pdstack_context_t *context;
#if defined(CY_DEVICE_CCG7S)
    if(false == app_is_typec_attached())
#endif /* defined(CY_DEVICE_CCG7S) */
    {
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
        {
            context = solution_fn_handler->Get_PdStack_Context(i);
            /* Disable the port. */
            if (context->ptrAppCbk != NULL)
            {
                context->ptrAppCbk->psrc_disable(context, NULL);
            }

#if VREG_INRUSH_DET_ENABLE
            /* Disable inrush fault timer */
            CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_VREG_TIMER));
#endif /* VREG_INRUSH_DET_ENABLE */
        }
    }

    context_0 = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_0_IDX);
#if defined(CY_DEVICE_CCG7D)
    context_1 = solution_fn_handler->Get_PdStack_Context(TYPEC_PORT_1_IDX);
    /* Disable interrupt handling until recovery is tried */
    Cy_USBPD_Fault_VregInrushDetDis(context_1->ptrUsbPdContext);
#if VREG_BROWN_OUT_DET_ENABLE
    /* Disable brownout fault also as both are related to vreg */
    Cy_USBPD_Fault_BrownOutDetDis(context_1->ptrUsbPdContext);
#endif /* VREG_BROWN_OUT_DET_ENABLE */

#else
   /* Disable interrupt handling until recovery is tried */
    Cy_USBPD_Fault_VregInrushDetDis(context_0->ptrUsbPdContext);
#if VREG_BROWN_OUT_DET_ENABLE
    /* Disable brownout fault also as both are related to vreg */
    Cy_USBPD_Fault_BrownOutDetDis(context_0->ptrUsbPdContext);
#endif /* VREG_BROWN_OUT_DET_ENABLE */
#endif /* defined(CY_DEVICE_CCG7D) */

    /* Enqueue fault event. */
    app_event_handler(context_0, APP_EVT_VREG_INRUSH_FAULT, NULL);
#else /* VREG_INRUSH_DET_ENABLE */
    /* No statement */
#endif /* VREG_INRUSH_DET_ENABLE */

    CY_UNUSED_PARAMETER(callbackContext);
    CY_UNUSED_PARAMETER(state);
}

/* End of File */
