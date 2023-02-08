/******************************************************************************
* File Name:   cy_pdaltmode_hw.c
* \version 2.0
*
* Description: Hardware control for Alternate Mode implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdaltmode_defines.h"
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_mngr.h"

#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include <cy_pdutils_sw_timer.h>
#include "app_timer_id.h"
#include "app.h"

#if DP_UFP_SUPP
#include "cy_usbpd_hpd.h"
#endif

#if RIDGE_SLAVE_ENABLE
#include "cy_pdaltmode_intel_ridge.h"
#include "cy_pdaltmode_ridge_slave.h"
#endif /* (RIDGE_SLAVE_ENABLE) */

#if BB_RETIMER_ENABLE
#include "bb_retimer.h"
#endif /* BB_RETIMER_ENABLE */

#if AMD_SUPP_ENABLE
#include "amd.h"
#endif /* AMD_SUPP_ENABLE */

#if AMD_RETIMER_ENABLE
#include "amd_retimer.h"
#endif /* AMD_RETIMER_ENABLE */

#if DEBUG_LOG
#include "debug.h"
#endif

#if DP_DFP_SUPP
/* DFP HPD callback */
static void Cy_PdAltMode_HW_DpSrcHpdCbk(void * context, cy_en_usbpd_hpd_events_t event);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* UFP HPD callback */
static void Cy_PdAltMode_HW_DpSnkHpdCbk(void * context, cy_en_usbpd_hpd_events_t event);
#endif /* DP_UFP_SUPP */

#if DPM_DEBUG_SUPPORT

/* Global buffer to store Type C FSM history */
static uint8_t mux_state_buf[NO_OF_TYPEC_PORTS][64];

/* This function pushes state to the state history buffer */
void mux_push_to_buf(uint8_t port, uint8_t state)
{
    static uint8_t indx[NO_OF_TYPEC_PORTS];

    /* Check if state is already present */
    if(indx[port] == 0)
    {
        if((state+1) == mux_state_buf[port][63])
            return;
    }
    else
    {
        if((state+1) == mux_state_buf[port][indx[port]-1])
            return;
    }
    /* If not present add to the buffer */
    mux_state_buf[port][indx[port]] = state + 1;
    indx[port]++;
    if(indx[port] > 63)
        indx[port] = 0;
    mux_state_buf[port][indx[port]] = 0xFF;
}

uint8_t* get_mux_state_buf(uint8_t port)
{
    return mux_state_buf[port];
}
#endif /* DPM_DEBUG_SUPPORT */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)

/********************** Function definitions **********************/
bool Cy_PdAltMode_HW_EvalAppAltHwCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *cmd_param)
{
    bool stat = false;
#if (!ICL_ALT_MODE_HPI_DISABLED)
#if (!CCG_BACKUP_FIRMWARE)
    uint8_t             hw_type, data_role;
    cy_stc_pdaltmode_hw_evt_t     cmd_info;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* Convert received cmd bytes as info and data */
    cmd_info.val  = CY_PDUTILS_MAKE_DWORD(cmd_param[3], cmd_param[2], cmd_param[1], cmd_param[0]);
    hw_type   = cmd_info.hw_evt.hw_type;
    data_role = cmd_info.hw_evt.data_role;

    if(data_role == ptrPdStackContext->dpmConfig.curPortType)
    {
        switch (hw_type)
        {
            case ALT_MODE_MUX:
                stat = Cy_PdAltMode_HW_EvalMuxCmd(ptrAltModeContext, cmd_info.hw_evt.evt_data);
                break;

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
            case ALT_MODE_HPD:
                stat = Cy_PdAltMode_HW_EvalHpdCmd(ptrAltModeContext, cmd_info.hw_evt.evt_data);
                break;
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

            default:
                /* Intentionally left empty */
                break;
        }
    }

#endif /* (!CCG_BACKUP_FIRMWARE) */
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

    return stat;
}

#if (!CCG_BACKUP_FIRMWARE)
bool Cy_PdAltMode_HW_EvalMuxCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t cmd)
{
    if (cmd < (uint32_t)MUX_CONFIG_RIDGE_CUSTOM)
    {
        return Cy_PdAltMode_HW_SetMux(ptrAltModeContext, (cy_en_pdaltmode_mux_select_t)cmd, CY_PDALTMODE_NO_DATA);
    }

    return false;
}
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
void Cy_PdAltMode_HW_SetCbk(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_cbk_pdaltmode_hw_cmd_t cbk)
{
    /* Register the callback if the port is valid. */
    if (ptrAltModeContext->pdStackContext->port < NO_OF_TYPEC_PORTS)
    {
        ptrAltModeContext->hwDetails.hw_cmd_cbk = cbk;
    }
}

bool Cy_PdAltMode_HW_EvalHpdCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t cmd)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
#if ((VIRTUAL_HPD_ENABLE) && (!ICL_ENABLE))
    bool virtual_hpd = Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext);
#endif /* ((VIRTUAL_HPD_ENABLE) && (!ICL_ENABLE)) */

    if (cmd == CY_PDALTMODE_HPD_DISABLE_CMD)
    {
        ptrAltModeContext->hwDetails.alt_mode_cmd_pending = false;
        
#if AMD_RETIMER_ENABLE
        amd_retimer_set_hpd (port, HPD_EVENT_UNPLUG);
#endif /* AMD_RETIMER_ENABLE */

#if ICL_ENABLE

        /* Only Virtual HPD supported in case of ICL/TGL projects. */
        tr_hpd_deinit (port);

#else /* ICL_ENABLE */

#if VIRTUAL_HPD_ENABLE
        if (virtual_hpd)
        {
            Cy_PdAltMode_Ridge_HpdDeInit(ptrAltModeContext);
        }
#endif /* VIRTUAL_HPD_ENABLE */
        {
            Cy_USBPD_Hpd_Deinit(ptrPdStackContext->ptrUsbPdContext);
        }
#endif /* ICL_ENABLE */

        return true;
    }

#if DP_DFP_SUPP
    if(ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
    {
        if (cmd == CY_PDALTMODE_HPD_ENABLE_CMD)
        {
            ptrAltModeContext->hwDetails.alt_mode_cmd_pending = false;
            
#if AMD_RETIMER_ENABLE
            amd_retimer_set_hpd (port, HPD_EVENT_NONE);
#endif /* AMD_RETIMER_ENABLE */
#if ICL_ENABLE
            tr_hpd_init(port, Cy_PdAltMode_HW_DpSrcHpdCbk);

#else /* ICL_ENABLE */

#if VIRTUAL_HPD_ENABLE
            if (virtual_hpd)
            {
                Cy_PdAltMode_Ridge_HpdInit(ptrAltModeContext, Cy_PdAltMode_HW_DpSrcHpdCbk);
            }
            else
#endif /* VIRTUAL_HPD_ENABLE */
            {
                Cy_USBPD_Hpd_TransmitInit(ptrPdStackContext->ptrUsbPdContext , Cy_PdAltMode_HW_DpSrcHpdCbk);
            }

#endif /* ICL_ENABLE */

            return true;
        }
        else
        {
            if (cmd < CY_PDALTMODE_HPD_DISABLE_CMD)
            {
                ptrAltModeContext->hwDetails.alt_mode_cmd_pending = true;
                ptrAltModeContext->hwDetails.alt_mode_hpd_state = (cy_en_usbpd_hpd_events_t)cmd;

#if ICL_ENABLE

                Cy_PdAltMode_Ridge_HpdSendEvt(port, (hpd_event_type_t)cmd);

#else /* ICL_ENABLE */

#if VIRTUAL_HPD_ENABLE
                if (virtual_hpd)
                {
                    Cy_PdAltMode_Ridge_HpdSendEvt(ptrAltModeContext, (cy_en_usbpd_hpd_events_t)cmd);
                }
                else
#endif /* VIRTUAL_HPD_ENABLE */
                {
                    Cy_USBPD_Hpd_TransmitSendevt(ptrPdStackContext->ptrUsbPdContext, (cy_en_usbpd_hpd_events_t) cmd);
                }

#endif /* ICL_ENABLE */

                return true;
            }
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if(ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        if (cmd == CY_PDALTMODE_HPD_ENABLE_CMD)
        {
#if ICL_ENABLE

            tr_hpd_init(port, Cy_PdAltMode_HW_DpSnkHpdCbk);

#else /* ICL_ENABLE */

#if VIRTUAL_HPD_ENABLE
            if (virtual_hpd)
            {
                Cy_PdAltMode_Ridge_HpdInit(ptrAltModeContext, Cy_PdAltMode_HW_DpSnkHpdCbk);
            }
            else
#endif /* VIRTUAL_HPD_ENABLE */
            {
                ptrAltModeContext->hwDetails.alt_mode_hpd_state = CY_USBPD_HPD_EVENT_UNPLUG;
                Cy_USBPD_Hpd_ReceiveInit(ptrPdStackContext->ptrUsbPdContext, Cy_PdAltMode_HW_DpSnkHpdCbk);
            }

#endif /* ICL_ENABLE */

            return true;
        }

#if ICL_ENABLE

        if (cmd == HPD_COMMAND_DONE)
        {
            tr_hpd_sendevt(port, (hpd_event_type_t)cmd);
        }

#else /* ICL_ENABLE */

#if VIRTUAL_HPD_ENABLE
        if ((virtual_hpd) && (cmd == CY_USBPD_HPD_COMMAND_DONE))
        {
            Cy_PdAltMode_Ridge_HpdSendEvt(ptrAltModeContext, (cy_en_usbpd_hpd_events_t)cmd);
        }
#endif /* VIRTUAL_HPD_ENABLE */

#endif /* ICL_ENABLE */
    }
#endif /* DP_UFP_SUPP */

    return false;
}

#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

#if DP_UFP_SUPP
static void Cy_PdAltMode_HW_DpSnkHpdCbk(void *context, cy_en_usbpd_hpd_events_t event)
{
    cy_stc_usbpd_context_t *ptrUsbpdContext = (cy_stc_usbpd_context_t *) context;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrUsbpdContext->pdStackContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    cy_stc_pdaltmode_hw_evt_t alt_mode_hw_data;

    if (((cy_en_usbpd_hpd_events_t)event > CY_PDALTMODE_HPD_ENABLE_CMD) && ((cy_en_usbpd_hpd_events_t)event < CY_PDALTMODE_HPD_DISABLE_CMD))
    {
        alt_mode_hw_data.hw_evt.data_role = CY_PD_PRT_TYPE_UFP;
        alt_mode_hw_data.hw_evt.hw_type   = ALT_MODE_HPD;

        /* Save first 4 bytes of event */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        ptrAltModeContext->hwDetails.hw_sln_data = (uint32_t)alt_mode_hw_data.val;

        /* Store current HPD event. */
        ptrAltModeContext->hwDetails.alt_mode_hpd_state = (cy_en_usbpd_hpd_events_t)event;

        /* Call the event callback, if it exists. */
        if (ptrAltModeContext->hwDetails.hw_cmd_cbk != NULL)
        {
            ptrAltModeContext->hwDetails.hw_cmd_cbk(ptrAltModeContext, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        sln_pd_event_handler (ptrPdStackContext, APP_EVT_APP_HW, &(ptrAltModeContext->hwDetails.hw_sln_data));
    }
    if(event == CY_USBPD_HPD_INPUT_CHANGE)
    {
        /*
         * Start HPD ACTIVITY TIMER to prevent re-entry into deepsleep.
         * HPD RX hardware block can't detect HPD events if device enters
         * deepsleep while HPD state is changing (or if HPD is HIGH).
         * If HPD is not connected, timer period shall be 100ms
         * (this is the default minimum HPD Connect debounce time).
         * Otherwise, timer period is 5ms (this is large enough
         * to capture HPD LOW and IRQ events.
         */
        if (Cy_PdAltMode_HW_DpSnkGetHpdState(ptrAltModeContext) == false)
        {
            /* Start a timer of 100ms to prevent deep sleep entry. */
            Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, HPD_RX_ACTIVITY_TIMER_ID, CY_PD_HPD_RX_ACTIVITY_TIMER_PERIOD_MAX);
        }
        else
        {
            Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, HPD_RX_ACTIVITY_TIMER_ID, CY_PD_HPD_RX_ACTIVITY_TIMER_PERIOD_MIN);
        }
    }
}

bool Cy_PdAltMode_HW_DpSnkGetHpdState(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /*
     * Return HPD state based on last HPD event from HAL.
     * If last event was UNPLUG, HPD is not connected. If it was
     * PLUG or IRQ, HPD is connected.
     */
    if (
           (ptrAltModeContext->hwDetails.alt_mode_hpd_state == CY_USBPD_HPD_EVENT_UNPLUG) ||
           (ptrAltModeContext->hwDetails.alt_mode_hpd_state == CY_USBPD_HPD_EVENT_NONE)
       )
    {
        return false;
    }
    else
    {
        return true;
    }
}
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
static void Cy_PdAltMode_HW_DpSrcHpdCbk(void *context, cy_en_usbpd_hpd_events_t event)
{
    cy_stc_usbpd_context_t *ptrUsbpdContext = (cy_stc_usbpd_context_t *) context;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrUsbpdContext->pdStackContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    if ((cy_en_usbpd_hpd_events_t)event == CY_USBPD_HPD_COMMAND_DONE)
    {
#if (!ICL_ALT_MODE_HPI_DISABLED)            
        cy_stc_pdaltmode_hw_evt_t alt_mode_hw_data;
    
#if AMD_RETIMER_ENABLE
        /* Process HPD signal to Retimer */
        if (
               (gl_alt_mode_hpd_state[port] == HPD_EVENT_PLUG) ||
               (gl_alt_mode_hpd_state[port] == HPD_EVENT_UNPLUG)
           )
        {
            amd_retimer_set_hpd (port, gl_alt_mode_hpd_state[port]);
        }
#endif /* AMD_RETIMER_ENABLE */

        /* ALT. MODE command completed. */
        ptrAltModeContext->hwDetails.alt_mode_cmd_pending = false;

        /* Set data role and HW type */
        alt_mode_hw_data.hw_evt.data_role = CY_PD_PRT_TYPE_DFP;
        alt_mode_hw_data.hw_evt.hw_type   = ALT_MODE_HPD;

        /* If HPD command done */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        ptrAltModeContext->hwDetails.hw_sln_data = (uint32_t)alt_mode_hw_data.val;

        /* Call the event callback, if it exists. */
        if (ptrAltModeContext->hwDetails.hw_cmd_cbk != NULL)
        {
            ptrAltModeContext->hwDetails.hw_cmd_cbk (ptrAltModeContext, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        sln_pd_event_handler(ptrPdStackContext, APP_EVT_APP_HW, &(ptrAltModeContext->hwDetails.hw_sln_data));
#else
        /* ALT. MODE command completed. */
        gl_alt_mode_cmd_pending[port] = false;
        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (ptrAltModeContext, (uint32_t)event);
        }   
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    }
}
#endif /* DP_DFP_SUPP */

void Cy_PdAltMode_HW_DeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdaltmode_mux_select_t cfg = MUX_CONFIG_SAFE;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* If we still have a device connected, switch MUX to USB mode. */
    if(ptrPdStackContext->dpmConfig.attach)
    {
        if(ptrPdStackContext->dpmStat.faultActive == false)
        {
            cfg = MUX_CONFIG_SS_ONLY;
        }
        else
        {
#if ICL_ENABLE
            if (PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
            {
                /* CDT 277908: in fault condition switch MUX to isolate state */
                cfg = MUX_CONFIG_ISOLATE;
            }
#endif /* ICL_ENABLE */
        }
    }

    /* Enable the desired connection for the high speed data lines. */
    Cy_PdAltMode_HW_SetMux(ptrAltModeContext, cfg, CY_PDALTMODE_NO_DATA);
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    
#if ICL_ENABLE
    Cy_PdAltMode_Ridge_HpdDeInit(ptrAltModeContext);
#else /* ICL_ENABLE */
#if VIRTUAL_HPD_ENABLE
    if (Cy_PdAltMode_HW_IsHostHpdVirtual (ptrAltModeContext))
    {
        Cy_PdAltMode_Ridge_HpdDeInit(ptrAltModeContext);
    }
    else
#endif /* VIRTUAL_HPD_ENABLE */
    {
        Cy_USBPD_Hpd_Deinit(ptrPdStackContext->ptrUsbPdContext);
    }
#endif /* ICL_ENABLE */

    /* Clear state variables. */
    ptrAltModeContext->hwDetails.alt_mode_cmd_pending = false;
    ptrAltModeContext->hwDetails.alt_mode_hpd_state   = CY_USBPD_HPD_EVENT_NONE;
    ptrAltModeContext->hwDetails.hw_cmd_cbk           = NULL;
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */
}

cy_en_pdaltmode_mux_select_t Cy_PdAltMode_HW_GetMuxState(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    if (ptrAltModeContext->pdStackContext->port < NO_OF_TYPEC_PORTS)
    {
        return (ptrAltModeContext->hwDetails.app_mux_state);
    }
    return (MUX_CONFIG_ISOLATE);
}

#if MUX_DELAY_EN && !GATKEX_CREEK
void Cy_PdAltMode_HW_MuxDelayCbk (cy_timer_id_t id, void * ptrContext)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

#if MUX_POLL_EN
    /* Stop Delay timer: There can be cases where this function is called manually. */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, APP_MUX_DELAY_TIMER);
#endif /* MUX_POLL_EN */

    /* Clear the MUX busy flag. */
    ptrAltModeContext->appStatus.is_mux_busy = false;

    /* If VDM response has been delayed, send it. */
    if (ptrAltModeContext->appStatus.is_vdm_pending != false)
    {
        ptrAltModeContext->appStatus.vdm_resp_cbk(ptrPdStackContext, &ptrAltModeContext->appStatus.vdmResp);
        ptrAltModeContext->appStatus.is_vdm_pending = false;
    }
    ptrAltModeContext->appStatus.is_mux_busy = false;

#if 0 /* Jira CCG_H3_4-350 */
    /* Update mux with saved config if required */
    if (app_mux_update_req[port] != false)
    {
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, app_mux_saved_state[port], app_mux_saved_custom_data[port]);
    }
#endif /* Jira CCG_H3_4-350 */

#if MUX_POLL_EN
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, APP_MUX_POLL_TIMER);
#if (CCG_HPI_ENABLE) && (CCG_HPI_PD_ENABLE)
    /* Send MUX Error notification if MUX failed */
    if (
            (ptrAltModeContext->appStatus.vdmResp.resp_buf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd_type == CMD_TYPE_RESP_NAK) ||
            (ptrAltModeContext->appStatus.vdmResp.resp_buf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_BUSY)
       )
    {
        /* Notify the EC if there is a MUX access error. */
        hpi_send_hw_error_event (port, SYS_HW_ERROR_MUX_ACCESS);
    }
#endif /* (CCG_HPI_ENABLE) && (CCG_HPI_PD_ENABLE) */
#endif /* MUX_POLL_EN */
}

#if MUX_POLL_EN
void mux_poll_cbk (cy_timer_id_t id, void * ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    mux_poll_status_t mux_stat;

    if (ptrAltModeContext->appStatus.mux_poll_cbk != NULL)
    {
        /* Run and analyse MUX polling function */
        mux_stat = ptrAltModeContext->appStatus.mux_poll_cbk(ptrAltModeContext);
        switch(mux_stat)
        {
            case MUX_STATE_FAIL:
                /* Save status and goto MUX handler */
                app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                Cy_PdAltMode_HW_MuxDelayCbk(APP_MUX_DELAY_TIMER, ptrPdStackContext);
                break;
            case MUX_STATE_SUCCESS:
                /* Save status and goto MUX handler */
                app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                Cy_PdAltMode_HW_MuxDelayCbk(APP_MUX_DELAY_TIMER, ptrPdStackContext);
                break;
            default:
                /* Run polling again */
                app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_BUSY;
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,  APP_MUX_POLL_TIMER, APP_MUX_POLL_TIMER_PERIOD, mux_poll_cbk);
        }
    }
}
#endif /* MUX_POLL_EN */
#endif /* MUX_DELAY_EN */

#if ICL_ENABLE
static bool gl_ignore_mux_changes[NO_OF_TYPEC_PORTS] = { false
#if PMG1_PD_DUALPORT_ENABLE    
    , 
    false
#endif /* PMG1_PD_DUALPORT_ENABLE */    
};

void ignore_mux_changes(uint8_t port, bool ignore)
{
    gl_ignore_mux_changes[port] = ignore;
}
#endif /* ICL_ENABLE */

bool Cy_PdAltMode_HW_SetMux(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t cfg, uint32_t custom_data)
{
    (void)custom_data;
    bool retval = true;

    if (ptrAltModeContext->appStatus.skip_mux_config != false)
    {
        return retval;
    }

    if (cfg == MUX_CONFIG_DEINIT)
    {
#if MUX_DELAY_EN
#if !GATKEX_CREEK
        /* Stop Delay timer */
        Cy_PdUtils_SwTimer_Stop(ptrAltModeContext->pdStackContext->ptrTimerContext, APP_MUX_DELAY_TIMER);
#endif

        /* Clear the MUX busy flag. */
        ptrAltModeContext->appStatus.is_mux_busy = false;
        ptrAltModeContext->hwDetails.app_mux_update_req = false;
#endif /* MUX_DELAY_EN */

        ptrAltModeContext->hwDetails.app_mux_state = cfg;
        return retval;
    }
    
#if ((!VIRTUAL_HPD_ENABLE) && (DP_DFP_SUPP))
    if (Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext) == false)
    {
        if (
               (ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP) &&
                (ptrAltModeContext->appStatus.alt_mode_entered != false) &&
               (ptrAltModeContext->hwDetails.alt_mode_cmd_pending == true)
            )
        {
            ptrAltModeContext->hwDetails.app_mux_saved_custom_data = custom_data;
            ptrAltModeContext->hwDetails.app_mux_saved_state       = cfg;
            ptrAltModeContext->hwDetails.app_mux_update_req        = true;
            return retval;
        }
        if (ptrAltModeContext->appStatus.is_mux_busy == false)
        {
            ptrAltModeContext->hwDetails.app_mux_update_req = false;
        }
    }
    
#endif /* (!VIRTUAL_HPD_ENABLE) && (DP_DFP_SUPP) */

#if BB_RETIMER_ENABLE && ICL_ENABLE
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        /* If application layer requested to ignore set_mux calls, do so.
         * This is only used during retimer firmware updates */
        if (gl_ignore_mux_changes[port])
            return retval;
    }
#endif /* BB_RETIMER_ENABLE && ICL_ENABLE  */

#if MUX_DELAY_EN && !GATKEX_CREEK
    if (ptrAltModeContext->appStatus.is_mux_busy == false)
    {
        /* Run MUX delay timer */
        ptrAltModeContext->appStatus.is_mux_busy = true;
#if ICL_SLAVE_ENABLE
        timer_start(port, APP_MUX_DELAY_TIMER, PD_GET_PTR_ICL_TGL_CFG_TBL(0)->soc_mux_config_delay, Cy_PdAltMode_HW_MuxDelayCbk);
#else
        Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext,
                ptrAltModeContext->pdStackContext, APP_MUX_DELAY_TIMER, APP_MUX_VDM_DELAY_TIMER_PERIOD, Cy_PdAltMode_HW_MuxDelayCbk);
#endif /* ICL_SLAVE_ENABLE */
        ptrAltModeContext->hwDetails.app_mux_update_req = false;

#if MUX_POLL_EN
        /* Run MUX polling timer */
        Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext,
                        ptrAltModeContext->pdStackContext, APP_MUX_POLL_TIMER, APP_MUX_POLL_TIMER_PERIOD, mux_poll_cbk);
#endif /* MUX_POLL_EN */
    }
    else
    {
        ptrAltModeContext->hwDetails.app_mux_saved_custom_data = custom_data;
        ptrAltModeContext->hwDetails.app_mux_saved_state       = cfg;
        ptrAltModeContext->hwDetails.app_mux_update_req        = true;
        return retval;
    }
#endif /* MUX_DELAY_EN */

    /* Store the current MUX configuration. */
    ptrAltModeContext->hwDetails.app_mux_state = cfg;

#if DPM_DEBUG_SUPPORT
    mux_push_to_buf(port, cfg);
#endif /* DPM_DEBUG_SUPPORT */

#if (MUX_TYPE == DP_MUX)
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
        retval = mux_ctrl_set_cfg (ptrAltModeContext->pdStackContext, cfg,  ptrAltModeContext->pdStackContext->dpmConfig.polarity);
#if (CCG_HPI_ENABLE) && (CCG_HPI_PD_ENABLE)
        if (!retval)
        {
            /* Notify the EC if there is a MUX access error. */
            hpi_send_hw_error_event (port, SYS_HW_ERROR_MUX_ACCESS);
        }
#endif
    }
    else
    {
        retval = false;
    }
#elif (MUX_TYPE == RIDGE_MUX)
    /* In TBT use cases, this call is used to configure the SBU Mux.
     * This has to be configured before notifying Alpine/Titan Ridge. */
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
#if ICL_SLAVE_ENABLE
        /* For RKL platform, set the mux to isolate if disconnect is happened.*/
        if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_tgl_selection == 3)
        {
            if ((MUX_CONFIG_RIDGE_CUSTOM == cfg) && (CY_PDALTMODE_NO_DATA == custom_data))
            {
                retval = Cy_PdAltMode_HW_MuxCtrlSetCfg (ptrAltModeContext, MUX_CONFIG_ISOLATE);
            }
            else
            {
                retval = Cy_PdAltMode_HW_MuxCtrlSetCfg (ptrAltModeContext, cfg);
            }
        }
        else
#endif /* ICL_SLAVE_ENABLE */
        {
            retval = Cy_PdAltMode_HW_MuxCtrlSetCfg (ptrAltModeContext, cfg);
        }
    }

#if RIDGE_SLAVE_ENABLE
    /*
       Update the Alpine/Titan Ridge status register. Do this even if the above call failed.
       This function is not expected to fail as it is an internal operation.
     */
    Cy_PdAltMode_Ridge_SetMux(ptrAltModeContext, cfg, ptrPdStackContext->dpmConfig.polarity, custom_data);
#endif /* RIDGE_SLAVE_ENABLE */

#elif (MUX_TYPE == AMD_MUX)
#if AMD_SUPP_ENABLE
    /* In AMD use cases, this call is used to configure the SBU Mux.
     * This has to be configured before notifying the SoC. */
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
        retval = Cy_PdAltMode_HW_MuxCtrlSetCfg (ptrAltModeContext, cfg);
    }
    /* Update AMD Renoir SoC */
    retval = amd_set_mux(port, cfg, dpm_get_polarity(port), custom_data);
#endif /* AMD_SUPP_ENABLE */

#if MUX_DELAY_EN && !GATKEX_CREEK
    if (retval != false)
    {
        /* Run MUX delay timer */
        app_get_status(port)->is_mux_busy = true;
        timer_start(port, APP_MUX_DELAY_TIMER, APP_MUX_VDM_DELAY_TIMER_PERIOD, Cy_PdAltMode_HW_MuxDelayCbk);
#if MUX_POLL_EN
        /* Run MUX polling timer */
        timer_start(port, APP_MUX_POLL_TIMER, APP_MUX_POLL_TIMER_PERIOD, mux_poll_cbk);
#endif /* MUX_POLL_EN */
    }
#endif /* MUX_DELAY_EN */
#endif

    return retval;
}

bool Cy_PdAltMode_HW_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return (!ptrAltModeContext->hwDetails.alt_mode_cmd_pending);
}

void Cy_PdAltMode_HW_Sleep(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    (void)ptrAltModeContext;
#if (!ICL_ENABLE)
#if VIRTUAL_HPD_ENABLE
    if (!Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext))
#endif /* VIRTUAL_HPD_ENABLE */
    {
#if (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF)))
#if DP_DFP_SUPP
        uint8_t port = ptrAltModeContext->pdStackContext->port;
        /* We can use the presence of a callback as indication that the HPD block is active. */
        if (ptrAltModeContext->hwDetails.hw_cmd_cbk != NULL)
        {
            /* Set the value of the HPD GPIO based on the last event. */
            if (port == 0)
            {
                Cy_GPIO_Write(Cy_GPIO_PortToAddr(HPD_P0_PORT_PIN >> 4), (HPD_P0_PORT_PIN & 0x0Fu),
                        (ptrAltModeContext->hwDetails.alt_mode_hpd_state > CY_USBPD_HPD_EVENT_UNPLUG));
            }
#if PMG1_PD_DUALPORT_ENABLE
            else
            {
                Cy_GPIO_Write(Cy_GPIO_PortToAddr(HPD_P1_PORT_PIN >> 4), (HPD_P1_PORT_PIN & 0x0Fu),
                        (ptrAltModeContext->hwDetails.alt_mode_hpd_state > CY_USBPD_HPD_EVENT_UNPLUG));
            }
#endif /* PMG1_PD_DUALPORT_ENABLE */
            /* Move the HPD pin from HPD IO mode to GPIO mode. */
            Cy_USBPD_Hpd_SleepEntry (ptrAltModeContext->pdStackContext->ptrUsbPdContext);
        }
#endif /* DP_DFP_SUPP */
#endif /* (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF))) */

#if (DP_UFP_SUPP) && (PMG1_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Prepare to enter deep sleep. */
        Cy_USBPD_Hpd_RxSleepEntry (ptrAltModeContext->pdStackContext->ptrUsbPdContext,
                Cy_USBPD_Hpd_ReceiveGetStatus(ptrAltModeContext->pdStackContext->ptrUsbPdContext));
#endif /* DP_UFP_SUPP && PMG1_HPD_RX_ENABLE */
    }
#endif /* (ICL_ENABLE == 0) */
}

void Cy_PdAltMode_HW_Wakeup(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    (void) ptrAltModeContext;
#if (!ICL_ENABLE)
#if VIRTUAL_HPD_ENABLE
        if (!Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext))
#endif /* VIRTUAL_HPD_ENABLE */
        {
#if (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF)))
#if DP_DFP_SUPP
        /* We can use the presence of a callback as indication that the HPD block is active. */
        if (ptrAltModeContext->hwDetails.hw_cmd_cbk != NULL)
        {
            /* Move the HPD pin back to HPD IO mode. */
            Cy_USBPD_Hpd_Wakeup (ptrAltModeContext->pdStackContext->ptrUsbPdContext,
                    (ptrAltModeContext->hwDetails.alt_mode_hpd_state > CY_USBPD_HPD_EVENT_UNPLUG));
        }
#endif /* DP_DFP_SUPP */
#endif /* (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF))) */

#if (DP_UFP_SUPP) && (PMG1_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Wakeup and revert HPD RX configurations. */
        Cy_USBPD_Hpd_RxWakeup (ptrAltModeContext->pdStackContext->ptrUsbPdContext);
#endif /* DP_UFP_SUPP  && PMG1_HPD_RX_ENABLE */
    }
#endif /* (!ICL_ENABLE) */
}

#define MAKE_MUX_STATE(polarity,cfg)    ((uint8_t)(polarity << 7) | (cfg & 0x7F))

bool Cy_PdAltMode_HW_MuxCtrlInit (cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* Leave the MUX in isolate mode at start. */
    return (Cy_PdAltMode_HW_MuxCtrlSetCfg (ptrAltModeContext, MUX_CONFIG_ISOLATE));
}


/* This function is used to configure the SBU Mux based on the active Alt. Modes. */
bool Cy_PdAltMode_HW_MuxCtrlSetCfg (cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t cfg)
{
    cy_en_usbpd_dpdm_mux_cfg_t dpdm_conf;

    cy_en_usbpd_sbu_switch_state_t sbu1_conf = CY_USBPD_SBU_NOT_CONNECTED;
    cy_en_usbpd_sbu_switch_state_t sbu2_conf = CY_USBPD_SBU_NOT_CONNECTED;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* We need to configure the SBU and DP/DM MUX blocks. */
    if ((ptrPdStackContext->port < NO_OF_TYPEC_PORTS) &&
            (MAKE_MUX_STATE(ptrPdStackContext->dpmConfig.polarity, cfg) != ptrAltModeContext->hwDetails.mux_cur_state))
    {

        /* Enable USB 2.0 connections through the appropriate signals. */
        if (ptrPdStackContext->dpmConfig.polarity)
        {
            dpdm_conf = CY_USBPD_DPDM_MUX_CONN_USB_BOT;
        }
        else
        {
            dpdm_conf = CY_USBPD_DPDM_MUX_CONN_USB_TOP;
        }

#if (GATKEX_CREEK != 1)
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP) || (TBT_DFP_SUPP) || (TBT_UFP_SUPP))
        uint8_t app_sbu_conf =  ptrAltModeContext->tbtCfg->sbuConfig;
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP) || (TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

        switch(cfg)
        {
            case MUX_CONFIG_SAFE:
            case MUX_CONFIG_SS_ONLY:
                /* Ensure that DP/DM connection is still enabled. */
                break;

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
            case MUX_CONFIG_DP_4_LANE:
            case MUX_CONFIG_DP_2_LANE:
                if ((app_sbu_conf == CY_PD_HOST_SBU_CFG_FULL) && (ptrPdStackContext->dpmConfig.polarity))
                {
                    /* Flipped connection: SBU1 <-> AUXN, SBU2 <-> AUXP. */
                    sbu1_conf = CY_USBPD_SBU_CONNECT_AUX2;
                    sbu2_conf = CY_USBPD_SBU_CONNECT_AUX1;
                }
                else
                {
                    /* Straight connection: SBU1 <-> AUXP, SBU2 <-> AUXN */
                    sbu1_conf = CY_USBPD_SBU_CONNECT_AUX1;
                    sbu2_conf = CY_USBPD_SBU_CONNECT_AUX2;
                }
                break;
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

#if ((TBT_DFP_SUPP) || (TBT_UFP_SUPP))
            case MUX_CONFIG_RIDGE_CUSTOM:
            case MUX_CONFIG_USB4_CUSTOM:
#if DELAYED_TBT_LSXX_CONNECT
                sbu1_conf = SBU_NOT_CONNECTED;
                sbu2_conf = SBU_NOT_CONNECTED;
#else /* DELAYED_TBT_LSXX_CONNECT */
                if (app_sbu_conf == CY_PD_HOST_SBU_CFG_PASS_THROUGH)
                {
                    /* Always connect SBUx to AUX side. */
                    sbu1_conf = CY_USBPD_SBU_CONNECT_AUX1;
                    sbu2_conf = CY_USBPD_SBU_CONNECT_AUX2;
                }
                else
                {
                    if ((app_sbu_conf == CY_PD_HOST_SBU_CFG_FULL) && (ptrPdStackContext->dpmConfig.polarity))
                    {
                        /* Flipped connection: SBU1 <-> LSRX, SBU2 <-> LSTX. */
                        sbu1_conf = CY_USBPD_SBU_CONNECT_LSRX;
                        sbu2_conf = CY_USBPD_SBU_CONNECT_LSTX;
                    }
                    else
                    {
                        /* Straight connection: SBU1 <-> LSTX, SBU2 <-> LSRX */
                        sbu1_conf = CY_USBPD_SBU_CONNECT_LSTX;
                        sbu2_conf = CY_USBPD_SBU_CONNECT_LSRX;
                    }
                }
#endif /* DELAYED_TBT_LSXX_CONNECT */
                break;
#endif /* ((TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

            default:
                /* No USB 2.0 connections required. */
                dpdm_conf = CY_USBPD_DPDM_MUX_CONN_NONE;
                break;
        }

#else
        sbu1_conf = CY_USBPD_SBU_CONNECT_LSTX;
        sbu2_conf = CY_USBPD_SBU_CONNECT_LSRX;
#endif

        /* Configure SBU connections as required. */
        Cy_USBPD_Mux_SbuSwitchConfigure(ptrPdStackContext->ptrUsbPdContext, sbu1_conf, sbu2_conf);

        /* Configure D+/D- MUX as required. */
        Cy_USBPD_Mux_ConfigDpDm(ptrPdStackContext->ptrUsbPdContext, dpdm_conf);
    }

    /* Store the current MUX configuration and polarity. */
    ptrAltModeContext->hwDetails.mux_cur_state = MAKE_MUX_STATE (ptrPdStackContext->dpmConfig.polarity, cfg);
    return true;
}

bool Cy_PdAltMode_HW_IsHostHpdVirtual(cy_stc_pdaltmode_context_t * ptrAltModeContext)
{
    bool ret = false;

#if VIRTUAL_HPD_ENABLE
    if(ptrAltModeContext->tbtCfg->hpdHandling != 0)
    {
        ret = true;
    }
#else
    (void)ptrAltModeContext;
#endif /* VIRTUAL_HPD_ENABLE */

    return ret;
}

#endif

/* [] END OF FILE */
