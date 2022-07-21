/***************************************************************************//**
* \file alt_mode_hw.c
* \version 1.1.0 
*
* Hardware control for Alternate mode source file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <config.h>
#include <cy_pdstack_dpm.h>
#include <alt_mode_hw.h>
#include <app.h>
#include <cy_pdstack_common.h>
#include <cy_usbpd_defines.h>
#include <cy_pdstack_utils.h>
#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */
#include <cy_sw_timer.h>

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
#include <gpio.h>
#include <hpd.h>
#include <srom.h>
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

#if (RIDGE_SLAVE_ENABLE)
#include <intel_ridge.h>
#include <ridge_slave.h>
#endif /* (RIDGE_SLAVE_ENABLE) */

#if BB_RETIMER_ENABLE
#include <bb_retimer.h>
#endif /* BB_RETIMER_ENABLE */

#if AMD_SUPP_ENABLE
#include <amd.h>
#endif /* AMD_SUPP_ENABLE */

#if AMD_RETIMER_ENABLE
#include <amd_retimer.h>
#endif /* AMD_RETIMER_ENABLE */

extern app_sln_handler_t *solution_fn_handler;
/* Holds hw solution event/command data */
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
#if (!ICL_ALT_MODE_HPI_DISABLED)
static uint32_t hw_sln_data[NO_OF_TYPEC_PORTS];
#endif /* !ICL_ALT_MODE_HPI_DISABLED */
/* Holds command callback information. */
static alt_mode_hw_cmd_cbk_t gl_hw_cmd_cbk[NO_OF_TYPEC_PORTS];
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

/* Holds current HPD command status. */
static volatile bool gl_alt_mode_cmd_pending[NO_OF_TYPEC_PORTS];
/* Holds current HPD pin status */
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static volatile hpd_event_type_t gl_alt_mode_hpd_state[NO_OF_TYPEC_PORTS];
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

#if DP_DFP_SUPP
/* DFP HPD callback */
static void
dp_src_hpd_cbk(uint8_t port, hpd_event_type_t event);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* UFP HPD callback */
static void dp_snk_hpd_cbk(uint8_t port, hpd_event_type_t event);
#endif /* DP_UFP_SUPP */

static mux_select_t app_mux_state[NO_OF_TYPEC_PORTS];

#if ((MUX_DELAY_EN) || (((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))))
uint32_t     gl_app_mux_saved_custom_data[NO_OF_TYPEC_PORTS];
mux_select_t gl_app_mux_saved_state[NO_OF_TYPEC_PORTS];
bool         gl_app_mux_update_req[NO_OF_TYPEC_PORTS];
#endif /* (MUX_DELAY_EN) || (((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))) */

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

/********************** Function definitions **********************/
bool eval_app_alt_hw_cmd(uint8_t port, uint8_t *cmd_param)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    bool stat = false;
#if (!ICL_ALT_MODE_HPI_DISABLED)
#if (!CCG_BACKUP_FIRMWARE)
    uint8_t             hw_type, data_role;
    alt_mode_hw_evt_t     cmd_info;

    /* Convert received cmd bytes as info and data */
    cmd_info.val  = MAKE_DWORD(cmd_param[3], cmd_param[2], cmd_param[1], cmd_param[0]);
    hw_type   = cmd_info.hw_evt.hw_type;
    data_role = cmd_info.hw_evt.data_role;

    if (data_role == (uint8_t)context->dpmConfig.curPortType)
    {
        switch ((alt_mode_hw_t)hw_type)
        {
            case ALT_MODE_MUX:
                stat = eval_mux_cmd(port, cmd_info.hw_evt.evt_data);
                break;
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
            case ALT_MODE_HPD:
                stat = eval_hpd_cmd(port, cmd_info.hw_evt.evt_data);
                break;
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */
            default:
                /* Intenionally left empty */
                break;
        }
    }
#endif /* (!CCG_BACKUP_FIRMWARE) */
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

    return stat;
}

#if (!CCG_BACKUP_FIRMWARE)
bool eval_mux_cmd(uint8_t port, uint32_t cmd)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    if (cmd < (uint32_t)MUX_CONFIG_RIDGE_CUSTOM)
    {
        return set_mux(context, (mux_select_t)cmd, NO_DATA);
    }

    return false;
}
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
void alt_mode_hw_set_cbk(uint8_t port, alt_mode_hw_cmd_cbk_t cbk)
{
    /* Register the callback if the port is valid. */
    if (port < NO_OF_TYPEC_PORTS)
    {
        gl_hw_cmd_cbk[port] = cbk;
    }
}

bool eval_hpd_cmd(uint8_t  port, uint32_t cmd)
{
#if ((RIDGE_I2C_HPD_ENABLE) && (!ICL_ENABLE))
    bool virtual_hpd = app_is_host_hpd_virtual(port);
#endif /* ((RIDGE_I2C_HPD_ENABLE) && (!ICL_ENABLE)) */

    if (cmd == HPD_DISABLE_CMD)
    {
        gl_alt_mode_cmd_pending[port] = false;
        
#if AMD_RETIMER_ENABLE
        amd_retimer_set_hpd (port, HPD_EVENT_UNPLUG);
#endif /* AMD_RETIMER_ENABLE */

#if ICL_ENABLE

        /* Only Virtual HPD supported in case of ICL/TGL projects. */
        tr_hpd_deinit (port);

#else /* ICL_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
        if (virtual_hpd)
        {
            tr_hpd_deinit(port);
        }
        else
#endif /* RIDGE_I2C_HPD_ENABLE */
        {
            (void)hpd_deinit(port);
        }

#endif /* ICL_ENABLE */

        return true;
    }

#if DP_DFP_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        if (cmd == HPD_ENABLE_CMD)
        {
            gl_alt_mode_cmd_pending[port] = false;
            
#if AMD_RETIMER_ENABLE
            amd_retimer_set_hpd (port, HPD_EVENT_NONE);
#endif /* AMD_RETIMER_ENABLE */
#if ICL_ENABLE
            tr_hpd_init(port, dp_src_hpd_cbk);

#else /* ICL_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
            if (virtual_hpd)
            {
                tr_hpd_init(port, dp_src_hpd_cbk);
            }
            else
#endif /* RIDGE_I2C_HPD_ENABLE */
            {
                hpd_transmit_init(port, dp_src_hpd_cbk);
            }

#endif /* ICL_ENABLE */

            return true;
        }
        else
        {
            if (cmd < HPD_DISABLE_CMD)
            {
                gl_alt_mode_cmd_pending[port] = true;
                gl_alt_mode_hpd_state[port]   = (hpd_event_type_t)cmd;

#if ICL_ENABLE

                tr_hpd_sendevt(port, (hpd_event_type_t)cmd);

#else /* ICL_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
                if (virtual_hpd)
                {
                    tr_hpd_sendevt(port, (hpd_event_type_t)cmd);
                }
                else
#endif /* RIDGE_I2C_HPD_ENABLE */
                {
                    hpd_transmit_sendevt(port, (hpd_event_type_t)cmd , false);
                }

#endif /* ICL_ENABLE */

                return true;
            }
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        if (cmd == HPD_ENABLE_CMD)
        {
#if ICL_ENABLE

            tr_hpd_init(port, dp_snk_hpd_cbk);

#else /* ICL_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
            if (virtual_hpd)
            {
                tr_hpd_init(port, dp_snk_hpd_cbk);
            }
            else
#endif /* RIDGE_I2C_HPD_ENABLE */
            {
                gl_alt_mode_hpd_state[port] = HPD_EVENT_UNPLUG;
                (void)hpd_receive_init(port, dp_snk_hpd_cbk);
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

#if RIDGE_I2C_HPD_ENABLE
        if ((virtual_hpd) && (cmd == HPD_COMMAND_DONE))
        {
            tr_hpd_sendevt(port, (hpd_event_type_t)cmd);
        }
#endif /* RIDGE_I2C_HPD_ENABLE */

#endif /* ICL_ENABLE */
    }
#endif /* DP_UFP_SUPP */

    return false;
}

#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */


#if DP_UFP_SUPP
static void dp_snk_hpd_cbk(uint8_t port, hpd_event_type_t event)
{
    alt_mode_hw_evt_t alt_mode_hw_data;

    if (((uint8_t)event > HPD_ENABLE_CMD) && ((uint8_t)event < HPD_DISABLE_CMD))
    {
        alt_mode_hw_data.hw_evt.data_role = (uint8_t)PRT_TYPE_UFP;
        alt_mode_hw_data.hw_evt.hw_type   = (uint8_t)ALT_MODE_HPD;

        /* Save first 4 bytes of event */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        hw_sln_data[port] = (uint32_t)alt_mode_hw_data.val;

        /* Store current HPD event. */
        gl_alt_mode_hpd_state[port] = event;

        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (port, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        /* QAC suppression 0315: This is a generic function accepting various types of data pointer
         * as arguments. In this case the data pointer points to 4 byte aligned data which does not 
         * violate alignment requirements of internal function calls. */
        sln_pd_event_handler (port, APP_EVT_APP_HW, &(hw_sln_data[port])); /* PRQA S 0315 */
    }
}

bool dp_snk_get_hpd_state(uint8_t port)
{
    /*
     * Return HPD state based on last HPD event from HAL.
     * If last event was UNPLUG, HPD is not connected. If it was
     * PLUG or IRQ, HPD is connected.
     */
#if CCG4_DOCK
    return  hpd_receive_get_status(port);
#else
    if (
           (gl_alt_mode_hpd_state[port] == HPD_EVENT_UNPLUG) ||
           (gl_alt_mode_hpd_state[port] == HPD_EVENT_NONE)
       )
    {
        return false;
    }
    else
    {
        return true;
    }
#endif /* CCG4_DOCK */
}
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
static void dp_src_hpd_cbk(uint8_t port, hpd_event_type_t event)
{
    if (event == HPD_COMMAND_DONE)
    {
#if (!ICL_ALT_MODE_HPI_DISABLED)            
        alt_mode_hw_evt_t alt_mode_hw_data;
    
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
        gl_alt_mode_cmd_pending[port] = false;

        /* Set data role and HW type */
        alt_mode_hw_data.hw_evt.data_role = PRT_TYPE_DFP;
        alt_mode_hw_data.hw_evt.hw_type   = ALT_MODE_HPD;

        /* If HPD command done */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        hw_sln_data[port] = (uint32_t)alt_mode_hw_data.val;

        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (port, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        sln_pd_event_handler(port, APP_EVT_APP_HW, &(hw_sln_data[port]));
#else
        /* ALT. MODE command completed. */
        gl_alt_mode_cmd_pending[port] = false;
        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (port, (uint32_t)event);
        }   
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    }
}
#endif /* DP_DFP_SUPP */

void alt_mode_hw_deinit(uint8_t port)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    mux_select_t cfg = MUX_CONFIG_SAFE;
    
    /* If we still have a device connected, switch MUX to USB mode. */
    if (ptrDpmConfig->attach)
    {
        if (context->dpmStat.faultActive == false)
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
    (void)set_mux(context, cfg, NO_DATA);

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))

#if ICL_ENABLE

    tr_hpd_deinit(port);

#else /* ICL_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
    if (app_is_host_hpd_virtual (port))
    {
        tr_hpd_deinit(port);
    }
    else
#endif /* RIDGE_I2C_HPD_ENABLE */
    {
        (void)hpd_deinit(port);
    }

#endif /* ICL_ENABLE */

    /* Clear state variables. */
    gl_alt_mode_cmd_pending[port] = false;
    gl_alt_mode_hpd_state[port]   = HPD_EVENT_NONE;
    gl_hw_cmd_cbk[port]           = NULL;
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */
}

mux_select_t get_mux_state(uint8_t port)
{
    if (port < NO_OF_TYPEC_PORTS)
    {
        return (app_mux_state[port]);
    }
    return (MUX_CONFIG_ISOLATE);
}

#if MUX_DELAY_EN
void mux_cbk (uint8_t port, timer_id_t id)
{
    (void)id;
    app_status_t* app_stat = app_get_status(port);

#if MUX_POLL_EN
    /* Stop Delay timer: There can be cases where this function is called manually. */
    timer_stop(port, APP_MUX_DELAY_TIMER);
#endif /* MUX_POLL_EN */

    /* Clear the MUX busy flag. */
    app_stat->is_mux_busy = false;

    /* If VDM response has been delayed, send it. */
    if (app_stat->is_vdm_pending != false)    
    {
        app_stat->vdm_resp_cbk(port, &app_stat->vdm_resp);
        app_stat->is_vdm_pending = false;
    }
    app_stat->is_mux_busy = false;

#if 0 /* Jira CCG_H3_4-350 */
    /* Update mux with saved config if required */
    if (app_mux_update_req[port] != false)
    {
        set_mux(port, app_mux_saved_state[port], app_mux_saved_custom_data[port]);  
    }
#endif /* Jira CCG_H3_4-350 */

#if MUX_POLL_EN
    timer_stop(port, APP_MUX_POLL_TIMER);

#if (CCG_HPI_ENABLE) && (CCG_HPI_PD_ENABLE)
    /* Send MUX Error notification if MUX failed */
    if (
            (app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type == CMD_TYPE_RESP_NAK) ||
            (app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_BUSY)
       )
    {
        /* Notify the EC if there is a MUX access error. */
        hpi_send_hw_error_event (port, SYS_HW_ERROR_MUX_ACCESS);
    }
#endif /* (CCG_HPI_ENABLE) && (CCG_HPI_PD_ENABLE) */
#endif /* MUX_POLL_EN */
}

#if MUX_POLL_EN
void mux_poll_cbk (uint8_t port, timer_id_t id)
{
    (void)id;
    mux_status_t mux_stat;
    app_status_t*     app_stat = app_get_status(port);

    if (app_stat->mux_poll_cbk != NULL)
    {
        /* Run and analyse MUX polling function */
        mux_stat = app_stat->mux_poll_cbk(port);
        switch(mux_stat)
        {
            case MUX_STATE_FAIL:
                /* Save status and goto MUX handler */
                app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_NAK;
                mux_cbk(port, APP_MUX_DELAY_TIMER);
                break;
            case MUX_STATE_SUCCESS:
                /* Save status and goto MUX handler */
                app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                mux_cbk(port, APP_MUX_DELAY_TIMER);
                break;
            default:
                /* Run polling again */
                app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_BUSY;
                timer_start(port, APP_MUX_POLL_TIMER, APP_MUX_POLL_TIMER_PERIOD, mux_poll_cbk);
        }
    }
}
#endif /* MUX_POLL_EN */
#endif /* MUX_DELAY_EN */

#if ICL_ENABLE
static bool gl_ignore_mux_changes[NO_OF_TYPEC_PORTS] = { false
#if CCG_PD_DUALPORT_ENABLE    
    , 
    false
#endif /* CCG_PD_DUALPORT_ENABLE */    
};

void ignore_mux_changes(uint8_t port, bool ignore)
{
    gl_ignore_mux_changes[port] = ignore;
}
#endif /* ICL_ENABLE */

#if 0
bool set_mux(uint8_t port, mux_select_t cfg, uint32_t custom_data)
{
    bool retval      = true;

    (void)custom_data;

    if (app_get_status(port)->skip_mux_config != false)
    {
        return retval;
    }

    if (cfg == MUX_CONFIG_DEINIT)
    {
#if MUX_DELAY_EN
        /* Stop Delay timer */
        timer_stop(port, APP_MUX_DELAY_TIMER);

        /* Clear the MUX busy flag. */
        app_get_status(port)->is_mux_busy = false;
        gl_app_mux_update_req[port] = false;
#endif /* MUX_DELAY_EN */

        app_mux_state[port] = cfg;
        return retval;
    }
    
#if ((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))
    if (app_is_host_hpd_virtual(port) == false)
    {
        if (
               (gl_dpm_port_type[port] != PRT_TYPE_UFP)          &&
               (app_get_status(port)->alt_mode_entered != false) &&
               (gl_alt_mode_cmd_pending[port] == true)
            )
        {
            gl_app_mux_saved_custom_data[port] = custom_data;
            gl_app_mux_saved_state[port]       = cfg;
            gl_app_mux_update_req[port]        = true;
            return retval;
        }
        if (app_get_status(port)->is_mux_busy == false)
        {
            gl_app_mux_update_req[port] = false;
        }
    }
    
#endif /* (!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP) */
#if BB_RETIMER_ENABLE && ICL_ENABLE
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        /* If application layer requested to ignore set_mux calls, do so.
         * This is only used during retimer firmware updates */
        if (gl_ignore_mux_changes[port])
            return retval;
    }
#endif /* BB_RETIMER_ENABLE && ICL_ENABLE  */

#if MUX_DELAY_EN
    if (app_get_status(port)->is_mux_busy == false)
    {
        /* Run MUX delay timer */
        app_get_status(port)->is_mux_busy = true;
#if ICL_SLAVE_ENABLE
        timer_start(port, APP_MUX_DELAY_TIMER, PD_GET_PTR_ICL_TGL_CFG_TBL(0)->soc_mux_config_delay, mux_cbk);
#else
        timer_start(port, APP_MUX_DELAY_TIMER, APP_MUX_VDM_DELAY_TIMER_PERIOD, mux_cbk);
#endif /* ICL_SLAVE_ENABLE */
        gl_app_mux_update_req[port] = false;

#if MUX_POLL_EN
        /* Run MUX polling timer */
        timer_start(port, APP_MUX_POLL_TIMER, APP_MUX_POLL_TIMER_PERIOD, mux_poll_cbk);
#endif /* MUX_POLL_EN */
    }
    else
    {
        gl_app_mux_saved_custom_data[port] = custom_data;
        gl_app_mux_saved_state[port]       = cfg;
        gl_app_mux_update_req[port]        = true;
        return retval;
    }
#endif /* MUX_DELAY_EN */

    /* Store the current MUX configuration. */
    app_mux_state[port] = cfg;

#if DPM_DEBUG_SUPPORT
    mux_push_to_buf(port, cfg);
#endif /* DPM_DEBUG_SUPPORT */

#if (MUX_TYPE == DP_MUX)
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
        retval = mux_ctrl_set_cfg (port, cfg,  dpm_get_polarity(port));
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
     * This has to be configured before notifying the SoC. */
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
#if ICL_SLAVE_ENABLE
        /* For RKL platform, set the mux to isolate if disconnect is happened.*/
        if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_tgl_selection == 3)
        {
            if ((MUX_CONFIG_RIDGE_CUSTOM == cfg) && (NO_DATA == custom_data))
            {
                retval = mux_ctrl_set_cfg (port, MUX_CONFIG_ISOLATE, dpm_get_polarity(port));
            }
            else
            {
                retval = mux_ctrl_set_cfg (port, cfg, dpm_get_polarity(port));
            }
        }
        else
#endif /* ICL_SLAVE_ENABLE */
        {
            retval = mux_ctrl_set_cfg (port, cfg, dpm_get_polarity(port));
        }
    }

#if RIDGE_SLAVE_ENABLE
    /*
       Update the Ridge/SoC data status register. Do this even if the above call failed.
       This function is not expected to fail as it is an internal operation.
     */
    ridge_set_mux (port, cfg, dpm_get_polarity(port), custom_data);
#endif /* RIDGE_SLAVE_ENABLE */

#elif (MUX_TYPE == AMD_MUX)
#if AMD_SUPP_ENABLE
    /* In AMD use cases, this call is used to configure the SBU Mux.
     * This has to be configured before notifying the SoC. */
    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
        retval = mux_ctrl_set_cfg (port, cfg, dpm_get_polarity(port));
    }
    /* Update AMD Renoir SoC */
    retval = amd_set_mux(port, cfg, dpm_get_polarity(port), custom_data);
#endif /* AMD_SUPP_ENABLE */
#endif /* MUX_TYPE */

    return retval;
}
#endif /* 0 */

bool alt_mode_hw_is_idle(uint8_t port)
{
    return (!gl_alt_mode_cmd_pending[port]);
}

void alt_mode_hw_sleep(uint8_t port)
{
#if (!ICL_ENABLE)
#if RIDGE_I2C_HPD_ENABLE
    if (!app_is_host_hpd_virtual(port))
#endif /* RIDGE_I2C_HPD_ENABLE */
    {
#if (!(defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF)))
#if DP_DFP_SUPP
        /* We can use the presence of a callback as indication that the HPD block is active. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            /* Set the value of the HPD GPIO based on the last event. */
            if (port == 0)
            {
                CALL_MAP(gpio_set_value)(HPD_P0_PORT_PIN, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
            }
#if CCG_PD_DUALPORT_ENABLE
            else
            {
                CALL_MAP(gpio_set_value)(HPD_P1_PORT_PIN, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
            }
#endif /* CCG_PD_DUALPORT_ENABLE */
            /* Move the HPD pin from HPD IO mode to GPIO mode. */
            hpd_sleep_entry (port);
        }
#else
    CY_UNUSED_PARAMETER(port);
#endif /* DP_DFP_SUPP */
#endif /* (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF))) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Prepare to enter deep sleep. */
        hpd_rx_sleep_entry (port, dp_snk_get_hpd_state(port));
#endif /* DP_UFP_SUPP && CCG_HPD_RX_ENABLE */
    }
#else
    CY_UNUSED_PARAMETER(port);
#endif /* (!ICL_ENABLE) */
}

void alt_mode_hw_wakeup(uint8_t port)
{
#if (!ICL_ENABLE)
#if RIDGE_I2C_HPD_ENABLE
    if (!app_is_host_hpd_virtual(port))
#endif /* RIDGE_I2C_HPD_ENABLE */
    {
#if (!((defined CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF)))
#if DP_DFP_SUPP
        /* We can use the presence of a callback as indication that the HPD block is active. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            /* Move the HPD pin back to HPD IO mode. */
            hpd_wakeup (port, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
        }
#else
    CY_UNUSED_PARAMETER(port);
#endif /* DP_DFP_SUPP */
#endif /* (!((defined CCG5) || defined(CCG5C) || defined(CCG6) || defined(CCG6DF) || defined(CCG6SF))) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Wakeup and revert HPD RX configurations. */
        hpd_rx_wakeup (port);
#endif /* DP_UFP_SUPP  && CCG_HPD_RX_ENABLE */
    }
#else
    CY_UNUSED_PARAMETER(port);
#endif /* (!ICL_ENABLE) */
}

/* [] END OF FILE */
