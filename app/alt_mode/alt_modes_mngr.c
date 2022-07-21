/***************************************************************************//**
* \file alt_mode_mngr.c
* \version 1.1.0 
*
* Alternate Mode Manager source file.
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
#include <alt_modes_mngr.h>
#include <alt_mode_hw.h>
#include <cy_pdstack_common.h>
#include <cy_pdstack_dpm.h>
#include <app.h>
#include <vdm.h>
#include <cy_sw_timer.h>
#include <vdm_task_mngr.h>
#include <vdm.h>
#include <cy_usbpd_defines.h>
#include "cy_pdstack_utils.h"

#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if (HPI_AM_SUPP != 0)
#include <custom_hpi_vid.h>
#endif /* (HPI_AM_SUPP != 0) */

#if ((MAX_SUPP_ALT_MODES < MAX_SVID_SUPP) && ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)))  
#error "The number of user alternate modes exceed maximum supported alt modes. Please increase MAX_SUPP_ALT_MODES."
#endif

extern app_sln_handler_t *solution_fn_handler;
/**
 * @typedef alt_mode_status_t
 * @brief struct to hold alt modes manager status
 */
typedef struct
{
    /* Holds info when register alt mode */
    alt_mode_reg_info_t   reg_info;
    /* Supported alternate modes. */
    uint32_t              am_supported_modes;
    /* Exited alternate modes. */
    uint32_t              am_exited_modes;
    /* Active alternate modes. */
    uint32_t              am_active_modes;
    /* Pointers to each alt mode info structure */
    alt_mode_info_t*      alt_mode_info[MAX_SUPP_ALT_MODES];
    /* Number of existed alt modes */
    uint8_t               alt_modes_numb;
    /* Buffer to hold VDM */
    cy_stc_pdstack_dpm_pd_cmd_buf_t      vdm_buf;
    /* Holds application event data */
    uint32_t              app_evt_data[ALT_MODE_EVT_SIZE];
    /* Current alt modes mngr status */
    alt_mode_mngr_state_t state;
    /* Pointer to vdm_msg_info_t struct in vdm task mngr */
    vdm_msg_info_t       *vdm_info;
    /* Hold current SVID index for discovery mode command */
    uint8_t               svid_idx;
    /* Check whether the device is a PD 3.0 supporting UFP. */
    uint8_t               pd3_ufp;
    /* Exit all alt modes procedure callback */
    cy_pdstack_pd_cbk_t              exit_all_cbk;
    /* Flag to indicate that exit all alt modes is required */
    bool                  exit_all_flag;
#if HPI_AM_SUPP
    /* Flag to indicate is custom alt mode reset is required */
    bool                  reset_custom_mode;
#endif /* HPI_AM_SUPP */
#if ATTENTION_QUEUE_SUPP
    pd_do_t               att_header;
    /* Holds unprocessed attention VDM header */
    pd_do_t               att_vdo;
    /* Holds unprocessed attention VDO */
    alt_mode_info_t      *att_alt_mode;
    /* Holds pointer to alt mode which saved attention is related to */
#endif /* ATTENTION_QUEUE_SUPP */
}alt_mode_mngr_status_t;

/*Main structure to hold alt modes manager status */
static alt_mode_mngr_status_t gl_alt_mode[NO_OF_TYPEC_PORTS];

#if (!CCG_BACKUP_FIRMWARE)
/* Handle received VDM for specific alt mode */
static void alt_mode_vdm_handle(uint8_t port, alt_mode_info_t *am_info, const cy_stc_pdstack_pd_packet_t *vdm);
#endif /* (!CCG_BACKUP_FIRMWARE) */ 

#if DFP_ALT_MODE_SUPP
/* Find next available alt mode for processing if previous exited */
static uint8_t get_next_alt_mode(uint8_t port);
#endif /* DFP_ALT_MODE_SUPP   */

#if UFP_ALT_MODE_SUPP
/* This function verifies possibility of entering of corresponding UFP alt mode */
static bool is_mode_activated(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm);
/* UFP function for alt modes processing */
static bool ufp_reg_alt_mode(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm);
/* Handles UFP enter mode processing */
static bool ufp_enter_alt_mode(uint8_t port, alt_mode_info_t *am_info_p, const cy_stc_pdstack_pd_packet_t *vdm, uint8_t am_idx);
#endif /* UFP_ALT_MODE_SUPP */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
/* Returns alt mode index for given svid */
static uint8_t get_base_alt_mode_svid_idx(uint8_t port, uint16_t svid);    
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */    

/* Composes alt mode info to vdm_msg_info_t struct before sending */
static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t* info, cy_en_pd_sop_t sop_type);

/* Parses received VDM info and moves it to alt mode info struct */
static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* info, cy_en_pd_sop_t sop_type);

/* Alt modes mngr AMS Prototypes */
static vdm_task_t run_disc_mode(uint8_t port);
static vdm_task_t eval_disc_mode(uint8_t port);
static vdm_task_t disc_mode_fail(uint8_t port);
static vdm_task_t monitor_alt_modes(uint8_t port);
static vdm_task_t eval_alt_modes(uint8_t port);
static vdm_task_t alt_mode_mngr_deinit(uint8_t port);
static vdm_task_t fail_alt_modes(uint8_t port);

uint16_t* get_alt_modes_vdo_info(uint8_t port, cy_en_pd_port_type_t type, uint8_t idx);

/*State Table*/
static vdm_task_t (*const alt_mode_ams_table [(uint8_t)ALT_MODE_MNGR_STATE_PROCESS + 1u]
        [(uint8_t)VDM_EVT_EXIT + 1u]) (uint8_t port) = {
    {
        /* Send next discovery svid */
        run_disc_mode,
        /* Evaluate disc svid response */
        eval_disc_mode,
        /* Process failed disc svid response */
        disc_mode_fail,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    },
    {
        /* Monitor if any changes appears in modes  */
        monitor_alt_modes,
        /* Evaluate alt mode response */
        eval_alt_modes,
        /* Process failed alt modes response */
        fail_alt_modes,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    }
};

/************************* DFP Related Function definitions *******************/

vdm_task_t reg_alt_mode_mngr(uint8_t port, atch_tgt_info_t* atch_tgt_info, vdm_msg_info_t* vdm_msg_info)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
#if ((CY_PD_REV3_ENABLE && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)))
    cy_stc_pd_dpm_config_t * dpm_stat = &(context->dpmConfig);
    bool pd3_live = (bool)(dpm_stat->specRevSopLive >= PD_REV3);
#endif

    gl_alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);
    gl_alt_mode[port].vdm_info       = vdm_msg_info;

    /* Check device role to start with. */
    if (context->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
    {
        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
#if CCG_USB4_SUPPORT_ENABLE        
            if (app_get_status(port)->usb4_active != false)
            {
                return VDM_TASK_ALT_MODE;
            }
#endif /* CCG_USB4_SUPPORT_ENABLE */
            /* UFP does not support any of the SVIDs of interest. Exit VDM manager. */
            return VDM_TASK_EXIT;
        }
        else
        {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            if (get_alt_mode_numb(port) != 0u)
            {
                /* Register pointers to VDM mngr info */
                gl_alt_mode[port].reg_info.atch_tgt_info  = atch_tgt_info;
                /* Set alt modes mngr state to Discovery Mode process */
                gl_alt_mode[port].reg_info.data_role      = (uint8_t)PRT_TYPE_DFP;
                gl_alt_mode[port].state                   = ALT_MODE_MNGR_STATE_DISC_MODE;

                /* Set alt mode trigger based on config */
                app_get_status(port)->alt_mode_trig_mask = get_pd_port_config(port)->alt_mode_trigger;                
                return VDM_TASK_ALT_MODE;
            }
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
        }
    }
    else
    {
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        gl_alt_mode[port].pd3_ufp = pd3_live;
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
            /* If we have no SVIDs to evaluate by UFP then go to regular monitoring */
            gl_alt_mode[port].state    = ALT_MODE_MNGR_STATE_PROCESS;
            return VDM_TASK_ALT_MODE;
        }
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        else
        {
            if (pd3_live)
            {
                /* If PD spec revision is 3.0, we can start with discover mode process. */
                return VDM_TASK_ALT_MODE;
            }
        }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
    }
    
    return VDM_TASK_EXIT;
}

bool is_alt_mode_mngr_idle(uint8_t port)
{
    bool    is_idle = true;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    alt_mode_info_t*  am_info_p;
    uint8_t           alt_mode_idx;

    /* QAC suppression 3415: Calling this function on the right hand side of '||'
     * is not a side effect as it is not intended to call this function if the 
     * left hand operand of '||' evaluates to true. */
    if ((gl_alt_mode[port].state == ALT_MODE_MNGR_STATE_DISC_MODE) ||
        (!alt_mode_hw_is_idle(port))) /* PRQA S 3415 */
    {
        return false;
    }

    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        am_info_p = get_mode_info(port, alt_mode_idx);
        /* If mode is active */
        if ((IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx) != 0u))
        {
            /* If alt mode not Idle then return current alt mode state */
            if ((am_info_p->mode_state != ALT_MODE_STATE_IDLE) ||
                (am_info_p->app_evt_needed != false))
            {
                is_idle = false;
            }
        }
    }
#else
    CY_UNUSED_PARAMETER(port);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

    return (is_idle);
}

void alt_mode_mngr_sleep(uint8_t port)
{
    alt_mode_hw_sleep (port);
}

void alt_mode_mngr_wakeup(uint8_t port)
{
    alt_mode_hw_wakeup (port);
}

vdm_task_t vdm_task_mngr_alt_mode_process(uint8_t port, vdm_evt_t vdm_evt)
{
    /* Run alt modes manager ams table */
    return alt_mode_ams_table[gl_alt_mode[port].state][vdm_evt](port);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static void set_disc_mode_params(uint8_t port, sop_t sop)
{
    vdm_msg_info_t *vdm_p = gl_alt_mode[port].vdm_info;

    vdm_p->vdm_header.val   = NO_DATA;
    vdm_p->VDM_HDR.svid     = gl_alt_mode[port].reg_info.atch_tgt_info->tgt_svid[gl_alt_mode[port].svid_idx];
    vdm_p->VDM_HDR.cmd      = CY_PDSTACK_VDM_CMD_DSC_MODES;
    vdm_p->VDM_HDR.objPos  = NO_DATA;
    vdm_p->sop_type         = sop;
    vdm_p->vdo_numb         = NO_DATA;
    vdm_p->VDM_HDR.vdmType = CY_PDSTACK_VDM_TYPE_STRUCTURED;
}

static void send_sln_event_nodata(uint8_t port, uint16_t svid, uint8_t am_id, alt_mode_app_evt_t evtype)
{
    app_event_handler (port, APP_EVT_ALT_MODE,
            form_alt_mode_event (port, svid, am_id, evtype, NO_DATA)
            );
}

static void send_sln_app_evt(uint8_t port, uint32_t data)
{
    app_event_handler (port, APP_EVT_ALT_MODE,
            form_alt_mode_event (port, gl_alt_mode[port].vdm_info->VDM_HDR.svid,
                gl_alt_mode[port].reg_info.alt_mode_id,
                    (data == NO_DATA) ? gl_alt_mode[port].reg_info.app_evt : AM_EVT_DATA_EVT,
                        data)
            );
}

static void send_app_evt_wrapper(uint8_t port, alt_mode_reg_info_t *reg)
{
    if (reg->app_evt != AM_NO_EVT)
    {
        /* Send notifications to the solution. */
        send_sln_app_evt(port, NO_DATA);
        reg->app_evt = AM_NO_EVT;
    }
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t run_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    uint16_t cur_svid;

    /* Set cable sop flag not needed at start*/
    gl_alt_mode[port].reg_info.cbl_sop_flag = SOP_INVALID;

    /* Search for next SVID until svid array is not empty */
    while ((cur_svid = gl_alt_mode[port].reg_info.atch_tgt_info->tgt_svid[gl_alt_mode[port].svid_idx]) != 0)
    {
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        /* Check is current port date role UFP */
        if (gl_alt_mode[port].pd3_ufp)
        {
            /* Send Disc Mode cmd */
            set_disc_mode_params (port, CY_PD_SOP);
            return VDM_TASK_SEND_MSG;
        }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

#if DFP_ALT_MODE_SUPP
        if (is_svid_supported(cur_svid, port) != MODE_NOT_SUPPORTED)
        {
            /* Send notifications to the solution. */
            send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_SUPP);

            /* If SVID is supported send Disc Mode cmd */
            set_disc_mode_params (port, CY_PD_SOP);
            return VDM_TASK_SEND_MSG;
        }

#if (SAVE_SUPP_SVID_ONLY == 0)
        /* Send notifications to the solution. */
        send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_NOT_SUPP);
#endif /* SAVE_SUPP_SVID_ONLY */

        /* If svid not supported */
        gl_alt_mode[port].svid_idx++;
#endif /* DFP_ALT_MODE_SUPP */
    }

    if ( 
           (gl_alt_mode[port].am_supported_modes == NONE_MODE_MASK)
#if CCG_USB4_SUPPORT_ENABLE        
        && (app_get_status(port)->usb4_active == false)
#endif /* CCG_USB4_SUPPORT_ENABLE */        
        )
    {
        /* No supp modes */
        return VDM_TASK_EXIT;
    }

#if (!ICL_ALT_MODE_HPI_DISABLED)
    /* Send SVID discovery finished notification to the solution. Enabled only if HPI commands are enabled. */
    send_sln_event_nodata (port, 0, 0, AM_EVT_DISC_FINISHED);
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */    

#if DFP_ALT_MODE_SUPP
    /* Goto alt mode process */
    get_next_alt_mode(port);
#endif /* DFP_ALT_MODE_SUPP */
    gl_alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
#else
    CY_UNUSED_PARAMETER(port);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
static void handle_cbl_disc_mode(uint8_t port, bool failed)
{
    uint8_t               tbl_svid_idx, vdo_idx;
    alt_mode_info_t      *am_info_p;
    vdm_msg_info_t       *vdm_p = gl_alt_mode[port].vdm_info;
    alt_mode_reg_info_t  *reg   = &(gl_alt_mode[port].reg_info);
    /*
       Get index of function in alt_mode_config related to receive SVID.
       This is expected to be valid as we would already have gone through
       Discover MODE for the UFP.
     */

    tbl_svid_idx = get_base_alt_mode_svid_idx(port, vdm_p->VDM_HDR.svid);
    reg->atch_type = CABLE;

    /* Analyse all received VDOs. */
    for (vdo_idx = 0; ((failed) || (vdo_idx < vdm_p->vdo_numb)); vdo_idx++)
    {
        if (failed)
        {
            reg->cbl_sop_flag = SOP_INVALID;
        }
        else
        {
            /* Save current VDO and its position in svid structure */
            reg->svid_emca_vdo = vdm_p->vdo[vdo_idx];
        }

        /* Check if DFP support attached target alt mode */
        am_info_p = gl_reg_alt_mode[tbl_svid_idx].reg_am_ptr(port, reg);
        if (am_info_p == NULL)
        {
            /* Get index of SVID related configuration from config.c */
            uint8_t cfg_svid_idx = is_svid_supported(vdm_p->VDM_HDR.svid, port);
            if(cfg_svid_idx != MODE_NOT_SUPPORTED)
            {
                /* Remove pointer to alt mode info struct */
                gl_alt_mode[port].alt_mode_info[cfg_svid_idx] = NULL;
                /* Remove flag that alt mode could be runned */
                REMOVE_FLAG(gl_alt_mode[port].am_supported_modes, cfg_svid_idx);
            }
            reg->cbl_sop_flag = SOP_INVALID;
        }
        else
        {
            if (!failed)
                am_info_p->cbl_obj_pos = (vdo_idx + VDO_START_IDX);
        }

        send_app_evt_wrapper(port, reg);

        if (failed)
            break;
    }
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t eval_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    volatile uint8_t               vdo_idx, tbl_svid_idx, cfg_svid_idx;
    alt_mode_info_t      *am_info_p;
    vdm_msg_info_t       *vdm_p = gl_alt_mode[port].vdm_info;
    alt_mode_reg_info_t  *reg   = &(gl_alt_mode[port].reg_info);
    
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (gl_alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        gl_alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    /* Evaluate CY_PD_SOP response */
    if (vdm_p->sop_type == (uint8_t)CY_PD_SOP)
    {
        /* Get index of function in alt_mode_config related to receive SVID */
        tbl_svid_idx = get_base_alt_mode_svid_idx(port, vdm_p->VDM_HDR.svid);
        /* Get index of SVID related configuration from config.c */
        cfg_svid_idx = is_svid_supported(vdm_p->VDM_HDR.svid, port);
        /* Additional check if we support current SVID operations */
        if (cfg_svid_idx != MODE_NOT_SUPPORTED)
        {
            reg->atch_type = ATCH_TGT;
            /* Analyse all rec VDOs */
            for (vdo_idx = 0; vdo_idx < vdm_p->vdo_numb; vdo_idx++)
            {
                /* Save current VDO and its position in svid structure */
                reg->svid_vdo = vdm_p->vdo[vdo_idx];
                /* Check if DFP support attached target alt mode */
                am_info_p = gl_reg_alt_mode[tbl_svid_idx].reg_am_ptr(port, reg);
                /* If VDO relates to any of supported alt modes */
                if (reg->alt_mode_id != MODE_NOT_SUPPORTED)
                {
                    send_app_evt_wrapper(port, reg);
                    /* If alternate modes discovered and could be runned */
                    if (am_info_p != NULL)
                    {
                        /* Save alt mode ID and object position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->objPos     = (vdo_idx + VDO_START_IDX);
#if (!ICL_ALT_MODE_HPI_DISABLED)                        
                        if (am_info_p->app_evt_needed != false)
                        {
                            /* Send notifications to the solution. */
                            send_sln_app_evt (port, am_info_p->app_evt_data.val);
                            am_info_p->app_evt_needed = false;
                        }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */                        
                        /* Save pointer to alt mode info struct */
                        gl_alt_mode[port].alt_mode_info[cfg_svid_idx] = am_info_p;
                        /* Set flag that alt mode could be runned */
                        SET_FLAG(gl_alt_mode[port].am_supported_modes, cfg_svid_idx);
                    }
                }
            }

            /* If cable DISC Mode is needed - send VDM */
            if (reg->cbl_sop_flag != SOP_INVALID)
            {
                set_disc_mode_params (port, SOP_PRIME);
                return VDM_TASK_SEND_MSG;
            }
        }
    }
    /* Evaluate cable response: Packet type will be CY_PD_SOP' or CY_PD_SOP'' here. */
    else
    {
        handle_cbl_disc_mode(port, false);

        /* If cable DISC Mode is needed - send VDM */
        if (reg->cbl_sop_flag != SOP_INVALID)
        {
            set_disc_mode_params (port, SOP_DPRIME);
            return VDM_TASK_SEND_MSG;
        }
    }

    /* If no any result goto next SVID */
    gl_alt_mode[port].svid_idx++;

#else
    CY_UNUSED_PARAMETER(port);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    return VDM_TASK_ALT_MODE;
}

static vdm_task_t disc_mode_fail(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (gl_alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        gl_alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    if (gl_alt_mode[port].vdm_info->sop_type == (uint8_t)SOP_PRIME)
    {
        handle_cbl_disc_mode(port, true);
    }
    /* If Disc Mode cmd fails goto next SVID */
    gl_alt_mode[port].svid_idx++;
#else
    CY_UNUSED_PARAMETER(port);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

static vdm_task_t monitor_alt_modes(uint8_t port)
{
    uint8_t          alt_mode_idx;
    alt_mode_state_t alt_mode_state;
    vdm_task_t       stat = VDM_TASK_ALT_MODE;
    cy_en_pd_sop_t            sop_state;
    alt_mode_info_t  *am_info_p;
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    
#if MUX_UPDATE_PAUSE_FSM
    /* Check if MUX is not busy */
    if (
          (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
#if ((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))
       || (gl_app_mux_update_req[port] != false)
#endif /* (!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP) */
       )
    {
        return stat;
    }
#endif /* MUX_UPDATE_PAUSE_FSM */
    
    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = get_mode_info(port, alt_mode_idx);
            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            switch (alt_mode_state)
            {
            case ALT_MODE_STATE_SEND:
                /* This case activates when VDM sequence for given alt mode */
                /* was interrupted by alt mode with higher priority */
            case ALT_MODE_STATE_WAIT_FOR_RESP:
                /*
                 * Check if CY_PD_SOP' or CY_PD_SOP'' messages are required.
                 * We do not send cable messages if VConn fault is present.
                 */
                if ((app_get_status(port)->fault_status & ((uint8_t)APP_PORT_VCONN_FAULT_ACTIVE | (uint8_t)APP_PORT_V5V_SUPPLY_LOST)) != 0u)
                {
                    am_info_p->sop_state[CY_PD_SOP_PRIME]  = ALT_MODE_STATE_IDLE;
                    am_info_p->sop_state[CY_PD_SOP_DPRIME] = ALT_MODE_STATE_IDLE;
                }
                
                /* CDT 300965: Set Exit mode sequence CY_PD_SOP -> CY_PD_SOP''-> CY_PD_SOP' */
                if (
                       am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE
#if DP_DFP_SUPP                    
                       || (
                           (am_info_p->VDM_HDR.svid == DP_SVID) &&
                           (am_info_p->vdo[CY_PD_SOP]->val == DP_USB_SS_CONFIG) &&
                           (am_info_p->VDM_HDR.cmd == DP_STATE_CONFIG)
                           )
#endif /* DP_DFP_SUPP */                    
                   )
                {
                    if (am_info_p->sop_state[CY_PD_SOP] == ALT_MODE_STATE_SEND)
                    {
                        sop_state = CY_PD_SOP;
                    }
                    else if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                    {
                        sop_state = CY_PD_SOP_DPRIME;
                    }
                    else
                    {
                        sop_state = CY_PD_SOP_PRIME;
                    }
                }
                else
                {
                    if (am_info_p->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_SEND)
                    {
                        sop_state = CY_PD_SOP_PRIME;
                    }
                    else if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                    {
                        sop_state = CY_PD_SOP_DPRIME;
                    }
                    else
                    {
                        sop_state = CY_PD_SOP;
                    }
                }

                /* Check if MUX should and could be set*/
                if (
                        ((am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ENTER_MODE) ||
                         (am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE)) &&
                        (am_info_p->set_mux_isolate != false)          &&
                        ((gl_alt_mode[port].am_active_modes & (uint32_t)(~((uint32_t)1u << alt_mode_idx))) == (uint32_t)false)
                   )
                {
                    (void)set_mux(context, MUX_CONFIG_SAFE, NO_DATA);
                }

                if (
                        (am_info_p->is_active)                         || 
                        (am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ENTER_MODE) ||
                        (am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE)
                    )
                {
                    /* Copy to vdm info and send vdm */
                    (void)move_to_vdm_info(port, am_info_p, sop_state);
                    am_info_p->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
                }

                stat = VDM_TASK_SEND_MSG;
                break;

            case ALT_MODE_STATE_EXIT:
#if HPI_AM_SUPP
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
                if (gl_alt_mode[port].reset_custom_mode != false)
                {
                    /* Send enter mode with new custom svid alt mode with new SVID */
                    if (am_info_p->cbk != NULL)
                    {
                        alt_mode_evt_t tmp_data;
                        /* Send command to change custom SVID */
                        am_info_p->eval_app_cmd(port, tmp_data);
                        am_info_p->mode_state = ALT_MODE_STATE_INIT;
                        am_info_p->cbk(port);
                        /* Set flag that reset is required */
                        gl_alt_mode[port].reset_custom_mode = false;
                        break;
                    }                   
                }
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */
#endif /* HPI_AM_SUPP */

#if DFP_ALT_MODE_SUPP
                if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
                {
                    /* Remove flag from active mode */
                    REMOVE_FLAG(gl_alt_mode[port].am_active_modes, alt_mode_idx);
                    /* Set alt mode as disabled */
                    am_info_p->mode_state = ALT_MODE_STATE_DISABLE;
                    /* Set flag as exited mode */
                    SET_FLAG(gl_alt_mode[port].am_exited_modes, alt_mode_idx);

                    /* If any active modes are present */
                    if (gl_alt_mode[port].am_active_modes == NONE_MODE_MASK)
                    {
                        /* Notify APP layer that ALT mode has been exited. */
                        app_get_status(port)->alt_mode_entered = false;
                        /* Set MUX to SS config */
                        set_mux(port, MUX_CONFIG_SS_ONLY, NO_DATA);
                        if (gl_alt_mode[port].exit_all_flag == false)
                        {
                            /* Get next alt mode if avaliable */
                            get_next_alt_mode(port);
                        }
                        else
                        {
                            /* Run callback command after all alt modes were exited */
                            gl_alt_mode[port].exit_all_cbk(port, true);
                            gl_alt_mode[port].exit_all_flag = false;
                            stat = VDM_TASK_WAIT;
                        }

                    }
                }
#endif /* DFP_ALT_MODE_SUPP */

#if UFP_ALT_MODE_SUPP
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    if (gl_alt_mode[port].am_active_modes != NONE_MODE_MASK)
                    {
                        /* Notify APP layer that ALT mode has been exited. */
                        app_get_status(port)->alt_mode_entered = false;
                    }

#if (!ICL_ALT_MODE_HPI_DISABLED)
                    /* Send notifications to the solution if alt mode was exited. */
                    /* QAC suppression 0315: app_event_handler is designed to accept various types of data
                     * for processing and hence the arguments are implicitly converted to void pointer. The 
                     * argument passed here is not dangerous as it is 4 byte aligned. */
                    app_event_handler (port, APP_EVT_ALT_MODE,
                            form_alt_mode_event (port, /* PRQA S 0315 */
                                am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id,
                                AM_EVT_ALT_MODE_EXITED, NO_DATA));
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

                    /* Set alt mode not active */
                    am_info_p->is_active = false;
                    /* Remove flag that alt mode could be processed */
                    REMOVE_FLAG(gl_alt_mode[port].am_active_modes, alt_mode_idx);
                }
#endif /* UFP_ALT_MODE_SUPP   */
                break;

            case ALT_MODE_STATE_IDLE:
#if (!ICL_ALT_MODE_HPI_DISABLED)
                /* If alt modes need to send app event data */
                if (am_info_p->app_evt_needed != false)
                {
                    /* Send notifications to the solution. */
                    /* QAC suppression 0315: app_event_handler is designed to accept various types of data
                     * for processing and hence the arguments are implicitly converted to void pointer. The 
                     * argument passed here is not dangerous as it is 4 byte aligned. */
                    app_event_handler (solution_fn_handler->Get_PdStack_Context(port), APP_EVT_ALT_MODE,
                            form_alt_mode_event (port, /* PRQA S 0315 */
                                am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id,
                                AM_EVT_DATA_EVT, 
                                am_info_p->app_evt_data.val));
                    am_info_p->app_evt_needed = false;
                }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */                
                break;

            case ALT_MODE_STATE_RUN:
                /* Run ufp evaluation function */
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                }
                break;

            default:
                /* Intentionally left empty */
                break;

            }
            
            if(stat == VDM_TASK_SEND_MSG)
            {
                break;
            }
        }
    }

    return stat;
}

static vdm_task_t eval_alt_modes(uint8_t port)
{
    uint8_t            alt_mode_idx;
    bool               eval_flag   = false;
    bool               skip_handle = false;
    alt_mode_state_t   alt_mode_state;
    vdm_task_t         stat = VDM_TASK_ALT_MODE;
    alt_mode_info_t    *am_info_p;
    vdm_msg_info_t     *vdm_p = gl_alt_mode[port].vdm_info;

#if (!ICL_ALT_MODE_EVTS_DISABLED)    
    alt_mode_app_evt_t appevt_type = AM_NO_EVT;
#if (!ICL_ALT_MODE_HPI_DISABLED)
    uint32_t           appevt_data = NO_DATA;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif /*  !ICL_ALT_MODE_EVTS_DISABLED */   
    
    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = get_mode_info(port, alt_mode_idx);

            /* If pointer to cbk is NULL then return */
            if (am_info_p->cbk == NULL)
            {
                break;
            }

            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            cy_en_pd_sop_t next_sop = CY_PD_SOP_INVALID;
            switch (alt_mode_state)
            {
                case ALT_MODE_STATE_WAIT_FOR_RESP:
                    /* Set flag that send transaction passed successful */
                    am_info_p->sop_state[vdm_p->sopType] = ALT_MODE_STATE_IDLE;
                    /* Copy received resp to alt mode info struct */
                    get_vdm_info_vdo(port, am_info_p, (cy_en_pd_sop_t)vdm_p->sopType);
                    
                    /* CDT 300965: Set Exit mode sequence CY_PD_SOP -> CY_PD_SOP''-> CY_PD_SOP' */
                    if (am_info_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE)
                    {
                        if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed send CY_PD_SOP'' VDM */
                            next_sop = CY_PD_SOP_DPRIME;
                        }
                        else if (am_info_p->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If CY_PD_SOP'' not needed - send CY_PD_SOP VDM */
                            next_sop = CY_PD_SOP_PRIME;
                        }
                        else
                        {
                            eval_flag = true;
                        }
                    }
#if DP_DFP_SUPP                     
                    /* CDT 300965: Set DP USB SS command sequence CY_PD_SOP -> CY_PD_SOP'-> CY_PD_SOP'' */
                    if ((am_info_p->VDM_HDR.svid == DP_SVID) && (am_info_p->vdo[CY_PD_SOP]->val == DP_USB_SS_CONFIG) && (am_info_p->VDM_HDR.cmd == DP_STATE_CONFIG))
                    {
                        if (am_info_p->sop_state[SOP_PRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed, send CY_PD_SOP' VDM */
                            next_sop = SOP_PRIME;
                        }
                        else if (am_info_p->sop_state[SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed, send CY_PD_SOP'' VDM */
                            next_sop = SOP_DPRIME;
                        }
                        else
                        {
                            eval_flag = true;
                        }
                    }
 #endif /* DP_DFP_SUPP */
                    if (next_sop != CY_PD_SOP_INVALID)
                    {
                        skip_handle = true;
                    }
                    
                    if (skip_handle == false)
                    {
                        /* If received VDM is CY_PD_SOP */
                        if ((vdm_p->sopType == (uint8_t)CY_PD_SOP) || (eval_flag != false))
                        {
                            /* Run alt mode analysis function */
                            /* NULL pointer check for cbk is not required as the loop breaks if the pointer is NULL,
                             * before reaching here. */
                            am_info_p->cbk(port);
                            /* If UVDM command then break */
                            if (vdm_p->VDM_HDR.vdmType != (uint32_t)CY_PDSTACK_VDM_TYPE_STRUCTURED)
                            {
                                break;
                            }
                            /* If alt mode entered */
                            if (vdm_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ENTER_MODE)
                            {
                                /* Notify APP layer that ALT mode has been entered. */
                                app_get_status(port)->alt_mode_entered = true;
                                /* Set mode as active */
                                am_info_p->is_active = true;

#if (!ICL_ALT_MODE_EVTS_DISABLED)
                                /* Queue notifications to the solution. */
                                appevt_type = AM_EVT_ALT_MODE_ENTERED;
#endif /* (!ICL_ALT_MODE_EVTS_DISABLED) */
                            }

                            if (vdm_p->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE)
                            {
                                /* Set mode as not active */
                                am_info_p->is_active = false;
#if (!ICL_ALT_MODE_EVTS_DISABLED)
                                /* Queue notifications to the solution. */
                                appevt_type = AM_EVT_ALT_MODE_EXITED;
#endif /* (!ICL_ALT_MODE_EVTS_DISABLED) */
                            }

#if (!ICL_ALT_MODE_EVTS_DISABLED)
#if (!ICL_ALT_MODE_HPI_DISABLED)
                            /* If alt modes need to send app event data */
                            if (am_info_p->app_evt_needed != false)
                            {
                                /* Queue notifications to the solution. */
                                appevt_type = AM_EVT_DATA_EVT;
                                appevt_data = am_info_p->app_evt_data.val;
                                am_info_p->app_evt_needed = false;
                            }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

                            /* Send event to solution space, if required. */
                            if (appevt_type != AM_NO_EVT)
                            {
#if (!ICL_ALT_MODE_HPI_DISABLED)
                                /* QAC suppression 0315: app_event_handler is designed to accept various types of data
                                 * for processing and hence the arguments are implicitly converted to void pointer. The 
                                 * argument passed here is not dangerous as it is 4 byte aligned. */
                                app_event_handler (solution_fn_handler->Get_PdStack_Context(port), APP_EVT_ALT_MODE,
                                        form_alt_mode_event (port, /* PRQA S 0315 */
                                            am_info_p->VDM_HDR.svid,
                                                am_info_p->alt_mode_id,
                                                    appevt_type, appevt_data));
#else
                                send_sln_event_nodata (port, 
                                        am_info_p->VDM_HDR.svid,
                                        am_info_p->alt_mode_id,
                                        appevt_type);
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
                            }
#endif /* (!ICL_ALT_MODE_EVTS_DISABLED) */
                        }
                        else
                        {
                            /* If received VDM is CY_PD_SOP' type, check if CY_PD_SOP'' is needed.
                               If received VDM is CY_PD_SOP'', the check on SOP_DPRIME will fail trivially. */
                            if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                            {
                                /* If needed send CY_PD_SOP'' VDM */
                                next_sop = CY_PD_SOP_DPRIME;
                            }
                            else if (am_info_p->sop_state[CY_PD_SOP] == ALT_MODE_STATE_SEND)
                            {
                                /* If CY_PD_SOP'' not needed - send CY_PD_SOP VDM */
                                next_sop = CY_PD_SOP;
                            }
                            else
                            {
                                /* NULL pointer check for cbk is not required as the loop breaks if the pointer is NULL,
                                 * before reaching here. */
                                am_info_p->cbk(port);
                            }
                        }
                    }
                    
                    if (next_sop != CY_PD_SOP_INVALID)
                    {
                        (void)move_to_vdm_info (port, am_info_p, next_sop);
                        stat = VDM_TASK_SEND_MSG;
                    }
                    break;

                default:
                    /* Intentionally left empty */
                    break;
            }
        }
    }

    return stat;
}

static vdm_task_t fail_alt_modes(uint8_t port)
{
    cy_stc_pdstack_context_t *ptrPdStackContext=solution_fn_handler->Get_PdStack_Context(port);
    uint8_t             alt_mode_idx;
    alt_mode_state_t    alt_mode_state;
    alt_mode_app_evt_t  appevt_type = AM_EVT_CBL_RESP_FAILED;
    alt_mode_info_t     *am_info_p;
    vdm_msg_info_t      *vdm_p = gl_alt_mode[port].vdm_info;
    
    /* Look through all alt modes */
    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = get_mode_info(port, alt_mode_idx);
            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            /* If mode waits for response */
            if (alt_mode_state == ALT_MODE_STATE_WAIT_FOR_RESP)
            {
                /* Change status to fail */
                am_info_p->sop_state[vdm_p->sopType] = ALT_MODE_STATE_FAIL;
                /* Set Failure code at the object pos field */
                am_info_p->VDM_HDR.objPos = vdm_p->VDM_HDR.objPos;

                if (vdm_p->sopType == (uint8_t)CY_PD_SOP)
                {
                    appevt_type = AM_EVT_SOP_RESP_FAILED;
                }

                /* Send notifications to the solution. */
                /* QAC suppression 0315: app_event_handler is designed to accept various types of data
                 * for processing and hence the arguments are implicitly converted to void pointer. The 
                 * argument passed here is not dangerous as it is 4 byte aligned. */
                app_event_handler (ptrPdStackContext, APP_EVT_ALT_MODE,
                        form_alt_mode_event (port, /* PRQA S 0315 */
                            am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id, appevt_type, NO_DATA));

                /* Run alt mode analysis function. */
                am_info_p->mode_state = ALT_MODE_STATE_FAIL;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                }
            }
        }
    }

    return VDM_TASK_ALT_MODE;
}

#if DFP_ALT_MODE_SUPP
static uint8_t get_next_alt_mode(uint8_t port)
{

    uint16_t* am_vdo_ptr = NULL;
    uint8_t am_idx = MODE_NOT_SUPPORTED;
    uint32_t idx1, idx2, am_config_svid, am_numb = gl_alt_mode[port].alt_modes_numb;
    uint32_t aval_modes = (
            (gl_alt_mode[port].am_supported_modes) &
            (~gl_alt_mode[port].am_exited_modes) &
            (~app_get_status(port)->alt_mode_trig_mask)
            );
    
    /* Start looking for next supported alt modes */
    for (idx1 = 0; idx1 < am_numb; idx1++)
    {
        /* If mode supported by CCG and not processed yet */
        if (IS_FLAG_CHECKED((aval_modes), idx1))
        {
            /* Look for the alt modes which could be runned */
            am_vdo_ptr = get_alt_modes_vdo_info(port, PRT_TYPE_DFP, idx1);
            am_config_svid = NO_DATA;
            for (idx2 = 0; idx2 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx2++)
            {
                /* Check which alternate modes could be run simulateneously */
                am_idx = get_alt_modes_config_svid_idx(port, PRT_TYPE_DFP,
                        am_vdo_ptr[idx2 + AM_SVID_CONFIG_OFFSET_IDX]);    
                if (am_idx != MODE_NOT_SUPPORTED)
                {
                    SET_FLAG(am_config_svid, am_idx);     
                }
            }

#if HPI_AM_SUPP
            /* Check if HPI SVID is supported */
            if (app_get_status(port)->custom_hpi_svid != NO_DATA)
            {
                uint8_t hpi_svid_idx = is_svid_supported(app_get_status(port)->custom_hpi_svid, port);

                /* Set bit as active if it's HPI alt mode */
                if (hpi_svid_idx != MODE_NOT_SUPPORTED)
                {
                    SET_FLAG(am_config_svid, hpi_svid_idx);
                }
            }
#endif /* #if HPI_AM_SUPP */    

            /* Set alternate mode Enter mask */
            gl_alt_mode[port].am_active_modes = am_config_svid & aval_modes;
            
            return true;
        }
    }

    return false;
}

#endif /* DFP_ALT_MODE_SUPP   */

#if (!UCSI_ALT_MODE_ENABLED)
static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t *info, cy_en_pd_sop_t sop_type)
#else
uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t *info, cy_en_pd_sop_t sop_type)
#endif /*UCSI_ALT_MODE_ENABLED*/
{
    vdm_msg_info_t *vdm_p = gl_alt_mode[port].vdm_info;
    
    vdm_p->sopType   = (uint8_t)sop_type;
    vdm_p->vdo_numb   = NO_DATA;
    vdm_p->vdm_header = info->vdm_header;    
    if ((info->vdo[sop_type] != NULL) && (info->vdo_numb[sop_type] != NO_DATA))
    {
        vdm_p->vdo_numb = info->vdo_numb[sop_type];
        /* Save received VDO */
        MEM_COPY((uint8_t *)vdm_p->vdo, (const uint8_t *)info->vdo[sop_type],((uint32_t)(info->vdo_numb[sop_type]) << 2u));
    }
    if (
            info->uvdm_supp == false
    
    /* If SVDM */
#if UVDM_SUPP        
        ||  ((info->VDM_HDR.vdmType == (uint32_t)VDM_TYPE_STRUCTURED)
        &&   (info->uvdm_supp))
#endif /* UVDM_SUPP */        
        )
    {
        vdm_p->VDM_HDR.vdmType = (uint32_t)CY_PDSTACK_VDM_TYPE_STRUCTURED;
        vdm_p->VDM_HDR.objPos = info->obj_pos;
        /* Set object position if custom Attention should be send */
        if ((info->VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ATTENTION) && (info->custom_att_obj_pos))
        {
            vdm_p->VDM_HDR.objPos = info->VDM_HDR.objPos;
        }
        if (sop_type != CY_PD_SOP)
        {
            vdm_p->VDM_HDR.objPos  = info->cbl_obj_pos;
        }
    }
    
    return (uint8_t)true;
}

#if (!UCSI_ALT_MODE_ENABLED)
static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* info, cy_en_pd_sop_t sop_type)
#else
void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* info, cy_en_pd_sop_t  sop_type)
#endif/*UCSI_ALT_MODE_ENABLED*/
{
    vdm_msg_info_t *vdm_p = gl_alt_mode[port].vdm_info;
    uint8_t         vdo_numb;

#if UVDM_SUPP        
    if (info->VDM_HDR.vdmType == (uint32_t)CY_PDSTACK_VDM_TYPE_STRUCTURED)
#endif /* UVDM_SUPP */        
    {
        /* Copy object position to the VDM Header */
        info->VDM_HDR.objPos = vdm_p->VDM_HDR.objPos;
    }
    vdo_numb = vdm_p->vdo_numb;
    /* Copy received VDO to alt mode info */
    if ((vdo_numb != 0u) && (info->vdo[sop_type] != NULL))
    {
        info->vdo_numb[sop_type] = vdm_p->vdo_numb;
        /* Save Rec VDO */
        if (vdm_p->vdo_numb <= info->vdo_max_numb)
        {
            /* Save received VDO */
            mem_copy((uint8_t *)info->vdo[sop_type], (const uint8_t *)vdm_p->vdo,((uint32_t)(vdm_p->vdo_numb) << 2u));
        }
    }
}

static vdm_task_t alt_mode_mngr_deinit(uint8_t port)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    uint8_t             alt_mode_idx;
    alt_mode_info_t     *am_info_p;
    bool                wait_for_exit = false;

    /* Find and reset alt modes info structures */
    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        am_info_p = get_mode_info(port, alt_mode_idx);
        if (am_info_p != NULL)
        {
            /* If current data role is UFP - set mode to idle */
            if (context->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
                /* CDT 232310 fix */
                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                am_info_p->VDM_HDR.cmd = (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE;
            }
            else
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (
                        (context->dpmConfig.contractExist != false) &&
                        (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx) != 0u)
                   )
                {
                    /* DFP: If PD contract is still present, make sure that EXIT_MODE request
                     * is sent to the UFP.
                     */
                    wait_for_exit = true;
                }
            }

            /* Exit from alt mode */
            if (am_info_p->cbk != NULL)
            {
                am_info_p->cbk(port);
            }

            if (!wait_for_exit)
            {
                /* Reset alt mode info */
                reset_alt_mode_info(am_info_p);
            }
        }
    }

    if (wait_for_exit)
    {
        return VDM_TASK_ALT_MODE;
    }

    /* Reset alt mode mngr info */
    alt_mode_mngr_reset_info(port);

#if (CCG_BB_ENABLE != 0)
    (void)bb_disable(port, true);
    (void)bb_update_all_status(port, BB_ALT_MODE_STATUS_INIT_VAL);
#endif /* (CCG_BB_ENABLE != 0) */

    return VDM_TASK_EXIT;
}

/******************** Common ALT Mode DFP and UFP functions *******************/

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))

uint8_t is_svid_supported(uint16_t svid, uint8_t port)
{

    uint8_t ret_idx = MODE_NOT_SUPPORTED;

#if (NON_TBT_MUX_SUPPORT == 1)    
    if((PD_GET_PTR_TBTHOST_CFG_TBL(port)->non_tbt_mux != 0) && (svid == INTEL_VID))
    {
        return ret_idx;
    }
#endif /* NON_TBT_MUX_SUPPORT*/    

    if (
            (svid != NO_DATA)          &&
            (port < NO_OF_TYPEC_PORTS) &&
            (get_base_alt_mode_svid_idx(port, svid) != MODE_NOT_SUPPORTED)
        )
    {
        ret_idx = get_alt_modes_config_svid_idx(port, gl_dpm_port_type[port], svid);
    }
    
    return ret_idx;    
}    

bool set_custom_svid(uint8_t port, uint16_t svid)
{
    bool                 ret = false;
#if HPI_AM_SUPP    
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
    uint8_t              cfg_idx;
    alt_mode_info_t     *am_info_p;
    alt_mode_reg_info_t *reg = &gl_alt_mode[port].reg_info;
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */
    volatile uint16_t prev_svid = app_get_status(port)->custom_hpi_svid;

    /* Proceed only if custom SVID has been changed. */
    if (svid != prev_svid)
    {
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
        /* Check custom alt mode state */
        if (gl_alt_mode[port].state == ALT_MODE_MNGR_STATE_PROCESS)
        {
            cfg_idx   = get_alt_mode_numb(port) - 1;
            am_info_p = gl_alt_mode[port].alt_mode_info[cfg_idx];
            /* Save new custom SVID */
            app_get_status(port)->custom_hpi_svid = svid;

            /* Check if alt mode is active */
            if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, cfg_idx) != false)
            {
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                    am_info_p->cbk(port);
                    if (svid != NO_DATA)
                    {
                        /* Set flag that reset is required */
                        gl_alt_mode[port].reset_custom_mode = true;
                    }
                }
            }
            else if (svid == NO_DATA)
            {
                /* Empty handler to ignore empty SVID */
            }
            /* Check if mode is supported then enter it */
            else if (IS_FLAG_CHECKED(gl_alt_mode[port].am_supported_modes, cfg_idx) != false)
            {
                if (am_info_p->cbk != NULL)
                {
                    alt_mode_evt_t tmp_data;
                    /* Send command to change custom SVID */
                    am_info_p->eval_app_cmd(port, tmp_data);
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(port);
                    SET_FLAG(gl_alt_mode[port].am_active_modes, cfg_idx);
                }                
            }
            /* If no custom SVID was registered then register it */
            else 
            {
                reg->atch_type = ATCH_TGT;
                am_info_p = gl_reg_alt_mode[get_base_alt_mode_svid_idx(port, HPI_AM_SVID)].reg_am_ptr(port, reg);
                if ((reg->alt_mode_id != MODE_NOT_SUPPORTED) && (am_info_p != NULL))
                {
                    am_info_p->objPos = 1;
                    /* Save pointer to alt mode info struct */
                    gl_alt_mode[port].alt_mode_info[cfg_idx] = am_info_p;
                    /* Set flag that alt mode could be runned */
                    SET_FLAG(gl_alt_mode[port].am_supported_modes, cfg_idx);
                    SET_FLAG(gl_alt_mode[port].am_active_modes, cfg_idx);
                }
            }
        }
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */
        
        /* Save new custom SVID */
        app_get_status(port)->custom_hpi_svid = svid;
        ret = true;
    }
#endif /* HPI_AM_SUPP */    
    
    return ret;
}


uint16_t get_custom_svid(uint8_t port)
{    
    uint16_t ret = NO_DATA;

#if HPI_AM_SUPP
    custom_alt_cfg_settings_t *ptr = PD_GET_PTR_CUSTOM_ALT_MODE_TBL(port);

    /* Check custom alt mode reg at first */
    if (app_get_status(port)->custom_hpi_svid != NO_DATA)
    {
        ret = app_get_status(port)->custom_hpi_svid;
    }  
    else if ((gl_dpm_port_type[port] == PRT_TYPE_DFP) && (ptr->custom_dfp_mask != NO_DATA))
    {
        /* Check config table custom alt mode*/
        ret = ptr->custom_alt_mode;
    }
#endif /* HPI_AM_SUPP */    
    
    return ret;
}

void set_alt_mode_mask(uint8_t port, uint16_t mask)
{
#if HPI_AM_SUPP    
    app_get_status(port)->dfp_alt_mode_mask = (uint8_t)(mask >> DFP_ALT_MODE_HPI_OFFSET);
    app_get_status(port)->ufp_alt_mode_mask = (uint8_t)(mask & UFP_ALT_MODE_HPI_MASK);  

    /* Check if we need to disable some alt modes and restart alt mode layer */
    if (gl_alt_mode[port].am_active_modes != (gl_alt_mode[port].am_active_modes & app_get_status(port)->dfp_alt_mode_mask))
    {
        alt_mode_layer_reset(port);
    }
#endif /* HPI_AM_SUPP */    
}

uint16_t get_svid_from_idx(uint8_t port, uint8_t idx)
{
    uint16_t* am_vdo_ptr;
    uint16_t  svid       = MODE_NOT_SUPPORTED;

    /* Look for the SVID with index position */
    am_vdo_ptr = get_alt_modes_vdo_info(port, gl_dpm_port_type[port], idx);
    if (am_vdo_ptr != NULL)
    {
        svid = am_vdo_ptr[AM_SVID_CONFIG_OFFSET_IDX];
    }
#if HPI_AM_SUPP
    /* Return custom alt mode idx */
    else if ( 
                (app_get_status(port)->custom_hpi_svid != NO_DATA) &&
                (idx == (get_alt_mode_numb(port) - 1))
            )
    {
        svid = app_get_status(port)->custom_hpi_svid;
    }
#endif /* HPI_AM_SUPP */    
    return svid;
}

uint16_t* get_alt_modes_vdo_info(uint8_t port, port_type_t type, uint8_t idx)
{
    /* QAC suppression 0310: Correct alignment of this pointer is ensured by the configuration table
     * generated by EZ-PD configuration utility. */
    uint16_t *ptr = (uint16_t *)(PD_GET_PTR_ALT_MODE_TBL(port)); /* PRQA S 0310 */
    uint8_t mask  = (type != PRT_TYPE_UFP) ? app_get_status(port)->dfp_alt_mode_mask : app_get_status(port)->ufp_alt_mode_mask;
    uint8_t len   = (uint8_t)ptr[0] >> 1;
    uint8_t loc_idx = 0;

    for (uint16_t i = 2; i < len; i += ((ptr[i] / 2u) + 1u))
    {
        if ((loc_idx == idx) && ((mask & (1u << idx)) != 0u))
        {
            return (ptr + i);
        }

        loc_idx++;
    }
    
    return NULL;
}

uint8_t get_alt_modes_config_svid_idx(uint8_t port, port_type_t type, uint16_t svid)
{
    uint16_t loc_len;
    uint8_t  idx = 0;

#if HPI_AM_SUPP
    if (app_get_status(port)->custom_hpi_svid == svid)
    {
        /* Return last possible index */
        return (get_alt_mode_numb(port) - 1u);
    }
#endif /* HPI_AM_SUPP */

    /* QAC suppression 0310: Correct alignment of this pointer is ensured by the configuration table
     * generated by EZ-PD configuration utility. */
    uint16_t *ptr = (uint16_t *)(PD_GET_PTR_ALT_MODE_TBL(port)); /* PRQA S 0310 */
    uint8_t mask  = (type != PRT_TYPE_UFP) ? app_get_status(port)->dfp_alt_mode_mask : app_get_status(port)->ufp_alt_mode_mask;
    uint8_t len   = (uint8_t)ptr[0] / 2u;

    for (uint8_t i = 2; i < len; i += (uint8_t)loc_len + 1u)
    {
        if ((svid == ptr[i + 1u]) && ((mask & (1u << idx)) != 0u))
        {
            return idx;
        }

        loc_len = ptr[i] >> 1;
        idx++;
    } 
    
    return MODE_NOT_SUPPORTED;
}

static uint8_t get_base_alt_mode_svid_idx(uint8_t port, uint16_t svid)
{
    uint8_t idx, base_am_numb;

    base_am_numb = sizeof(gl_reg_alt_mode)/sizeof(reg_am_t);
    /* Look through all alt modes */
    /* QAC suppression 2877: The for loop may get executed more than once based on
     * the number of alternate modes programmed in configuration table. */
    for (idx = 0; idx < base_am_numb; idx++) /* PRQA S 2877 */
    {
        /* if alt mode with given index is supported by CCG */
        if (gl_reg_alt_mode[idx].svid == svid)
        {
            return idx;
        }
    }   
#if HPI_AM_SUPP
    /* Check if custom HPI alt mode is avaliable */
    if (
           (get_base_alt_mode_svid_idx(port, HPI_AM_SVID) != MODE_NOT_SUPPORTED) &&
           (app_get_status(port)->custom_hpi_svid == svid)
        )
    {
        return get_base_alt_mode_svid_idx(port, HPI_AM_SVID);
    }
#endif /* HPI_AM_SUPP */
    
    return MODE_NOT_SUPPORTED;
}

#else
void set_alt_mode_mask(uint8_t port, uint16_t mask)
{
    (void)port;
    (void)mask;
}

bool set_custom_svid(uint8_t port, uint16_t svid)
{
    (void)port;
    (void)svid;

    return false;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

uint8_t get_alt_mode_numb(uint8_t port)
{
    uint8_t count   = 0;
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if (!CCG_BACKUP_FIRMWARE)

    if(get_pd_port_config(port)->alt_mode_tbl_offset != 0u)
    {
        /* QAC suppression 0310: The correct alignment of this pointer is ensured by 
         * the configuration table generated by EZ-PD configuration utility. */
        uint16_t* ptr   = (uint16_t *)(PD_GET_PTR_ALT_MODE_TBL(port)); /* PRQA S 0310 */
        uint8_t len     = (uint8_t)(ptr[0] / 2u);
        uint16_t loc_len;

        for (uint8_t i = 2; i < len; i += (uint8_t)loc_len + 1u)
        {
            loc_len = ptr[i] >> 1;
            count++;
        }
    }

#if HPI_AM_SUPP
    if (get_custom_svid(port) != NO_DATA)
    {
        count++;
    }
#endif /* HPI_AM_SUPP */    
#endif /* (!CCG_BACKUP_FIRMWARE) */
#else
    CY_UNUSED_PARAMETER(port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
    return count;
}

void reset_alt_mode_info(alt_mode_info_t *info)
{
    memset((uint8_t *)info, 0u, (uint32_t)sizeof(*info));
}

void alt_mode_mngr_reset_info(uint8_t port)
{
    memset((uint8_t *)&gl_alt_mode[port], 0u, (uint32_t)sizeof(gl_alt_mode[port]));
}

cy_stc_pdstack_dpm_pd_cmd_buf_t* get_vdm_buff(uint8_t port)
{
    return &(gl_alt_mode[port].vdm_buf);
}

/******************* ALT MODE Solution Related Functions ****************************/

/* Function to reset alt mode layer */
void alt_mode_layer_reset(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    app_get_status(port)->skip_mux_config = true;
    vdm_task_mngr_deinit (port);
    enable_vdm_task_mngr (port);
    app_get_status(port)->skip_mux_config = false;
#else
    CY_UNUSED_PARAMETER(port);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
}

#if ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE))

#if DFP_ALT_MODE_SUPP
bool is_enter_allowed(uint8_t port, uint16_t svid)
{
    bool    am_allowed         = false;
    uint8_t am_numb            = gl_alt_mode[port].alt_modes_numb;
    uint8_t idx, idx2;
    uint16_t* am_vdo_ptr       = NULL;
        
    /* Find out if any alt mode already entered */
    if (gl_alt_mode[port].am_active_modes != NO_DATA)
    {
        for (idx = 0; idx < am_numb; idx++)
        {
            /* Find active alt modes */
            if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, idx))
            {
                am_allowed = false;
                /* Get pointer to SVID configuration */
                am_vdo_ptr = get_alt_modes_vdo_info(port, PRT_TYPE_DFP, idx);
                if (am_vdo_ptr != NULL)
                {
                    /* Find out if selected alt mode could be processed simultaneously  with active alt modes */
                    for (idx2 = 0; idx2 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx2++)
                    {
                        /* Check which alternate modes could be run simulateneously */
                        if (am_vdo_ptr[idx2 + AM_SVID_CONFIG_OFFSET_IDX] == svid)
                        {
                            am_allowed = true;
                            break;
                        }
                    }
                    /* If selected alt mode couldn't run simultaneously  with active alt modes return false */
                    if (am_allowed == false)
                    {
                        return false;
                    }
                }
            }
        }
    }
    
    return true;
}

/* Function to exit all active alternate modes. */
void alt_mode_mngr_exit_all(uint8_t port, bool send_vdm_exit, pd_cbk_t exit_all_cbk)
{
    uint8_t                  alt_mode_idx;
    alt_mode_info_t         *am_info_p = NULL;
    alt_mode_mngr_status_t*  mngr_ptr  = &gl_alt_mode[port];

    if (gl_dpm_port_type[port] != PRT_TYPE_DFP)
    {
        /* Configure the MUX to get out of alt modes*/
        if(app_get_status(port)->alt_mode_entered)
        {
            set_mux (port, MUX_CONFIG_ISOLATE, 0);
        }
    }

    if ((send_vdm_exit != false) && (app_get_status(port)->alt_mode_entered))
    {
        mngr_ptr->exit_all_cbk  = exit_all_cbk;
        mngr_ptr->exit_all_flag = true;
    }

    /* For each alternate mode, initiate mode exit. */
    for (alt_mode_idx = 0; alt_mode_idx < mngr_ptr->alt_modes_numb; alt_mode_idx++)
    {
        if (IS_FLAG_CHECKED(mngr_ptr->am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info (port, alt_mode_idx);
            if (am_info_p != NULL)
            {
                
                /* Logicaly exit all alt modes */
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk (port);
                }
            }
        }
    }
}

#endif /* DFP_ALT_MODE_SUPP */

bool eval_app_alt_mode_cmd(uint8_t port, uint8_t *cmd, uint8_t *data)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
#if (!ICL_ALT_MODE_HPI_DISABLED)
#if (!CCG_BACKUP_FIRMWARE)
    alt_mode_evt_t  cmd_info, cmd_data;
    alt_mode_info_t *am_info_p = NULL;
    uint8_t         alt_mode_idx;
    bool            found = false;

    /* Convert received cmd bytes as info and data */
    cmd_info.val = MAKE_DWORD(cmd[3], cmd[2], cmd[1], cmd[0]);
    cmd_data.val = MAKE_DWORD(data[3], data[2], data[1], data[0]);

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check if received app command is start discover process for UFP when PD 3.0 supported */
    if (
            (gl_alt_mode[port].pd3_ufp) &&
            (cmd_info.alt_mode_event.data_role == PRT_TYPE_UFP) &&
            (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_RUN_UFP_DISC)
       )
    {
        /* Try to start Discovery process if VDM manager is not busy  */
        return is_ufp_disc_started(port);
    }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    /* Look for the alternate mode entry which matches the SVID and alt mode id. */
    for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is supported. */
        if (IS_FLAG_CHECKED(gl_alt_mode[port].am_supported_modes, alt_mode_idx) != 0u)
        {
            am_info_p = get_mode_info (port, alt_mode_idx);
            if ((am_info_p->VDM_HDR.svid == cmd_info.alt_mode_event.svid) &&
                (am_info_p->alt_mode_id  == cmd_info.alt_mode_event.alt_mode))
            {
                found = true;
                break;
            }
        }
    }

#if DFP_ALT_MODE_SUPP
    if ((gl_dpm_port_type[port] != PRT_TYPE_UFP) && (cmd_info.alt_mode_event.data_role != PRT_TYPE_UFP))
    {
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_SET_TRIGGER_MASK)
        {
            app_get_status(port)->alt_mode_trig_mask = (uint8_t)cmd_data.alt_mode_event_data.evt_data;
            return true;
        }
        /* Check if Enter command and trigger is set */
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_ENTER)
        {
            /* If we found an alternate mode entry for this {SVID, ID} pair. */
            if (found)
            {
                if (
                        (am_info_p->cbk != NULL) && 
                        (am_info_p->is_active == false) && 
                        (is_enter_allowed(port, am_info_p->VDM_HDR.svid) != false)
                    )
                {
                    /* Set flag as active mode */
                    SET_FLAG(gl_alt_mode[port].am_active_modes, alt_mode_idx);
                    REMOVE_FLAG(gl_alt_mode[port].am_exited_modes, alt_mode_idx);

                    /* Inits alt mode */
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(port);

                    /* Goto alt mode processing */
                    gl_alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
                    return true;
                }
            }
            return false;
        }
    }
#endif /* DFP_ALT_MODE_SUPP */

    if ((found) && (am_info_p->is_active == true) && (am_info_p->mode_state == ALT_MODE_STATE_IDLE) &&
         (cmd_info.alt_mode_event.data_role == (uint8_t)context->dpmConfig.curPortType))
    {
        /* If received cmd is specific alt mode command */
        if (cmd_info.alt_mode_event.alt_mode_evt == (uint32_t)AM_CMD_SPEC)
        {
            return (am_info_p->eval_app_cmd(port, cmd_data));
        }
#if DFP_ALT_MODE_SUPP
        else if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_EXIT)
        {
            if (context->dpmConfig.curPortType != PRT_TYPE_UFP)
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                    return true;
                }
            }
        }
#endif /* DFP_ALT_MODE_SUPP */
    }
#endif /* (!CCG_BACKUP_FIRMWARE) */
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    return false;
}

#endif /* ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE)) */

const uint32_t* form_alt_mode_event(uint8_t port, uint16_t svid, uint8_t am_idx, alt_mode_app_evt_t evt, uint32_t data)
{
    cy_stc_pdstack_context_t * context=solution_fn_handler->Get_PdStack_Context(port);
    alt_mode_evt_t temp;

    temp.alt_mode_event.svid                           = (uint32_t)svid;
    temp.alt_mode_event.alt_mode                       = (uint32_t)am_idx;
    temp.alt_mode_event.alt_mode_evt                   = (uint32_t)evt;
    temp.alt_mode_event.data_role                      = (uint32_t)context->dpmConfig.curPortType;
    gl_alt_mode[port].app_evt_data[ALT_MODE_EVT_IDX]      = temp.val;
    gl_alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = NO_DATA;

    if (data != NO_DATA)
    {
        gl_alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = data;
    }

    return gl_alt_mode[port].app_evt_data;
}

alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx)
{
    return gl_alt_mode[port].alt_mode_info[alt_mode_idx];
}

#if (!CCG_BACKUP_FIRMWARE)
#if ATTENTION_QUEUE_SUPP
static void alt_mode_att_cb(uint8_t port, timer_id_t id);
#endif /* ATTENTION_QUEUE_SUPP */
static void alt_mode_vdm_handle(uint8_t port, alt_mode_info_t *am_info, const cy_stc_pdstack_pd_packet_t *vdm)
{
#if ATTENTION_QUEUE_SUPP
    /* Check if it's attention message which comes while alt mode state machine is busy */
    if (
            (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_ATTENTION) && 
            ( 
                (am_info->mode_state != ALT_MODE_STATE_IDLE)
#if MUX_UPDATE_PAUSE_FSM
             || (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
#endif /* MUX_UPDATE_PAUSE_FSM */
            )
        )
    {
        /* Save attention for the further processing */
        gl_alt_mode[port].att_header = vdm->dat[VDM_HEADER_IDX];
        gl_alt_mode[port].att_vdo = vdm->dat[VDO_START_IDX];
        gl_alt_mode[port].att_alt_mode = am_info;
        timer_start(port, ALT_MODE_ATT_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, alt_mode_att_cb);
        return;
    }
#endif /* ATTENTION_QUEUE_SUPP */
    
    /* Save Header */
    am_info->vdm_header.val = vdm->dat[CY_PD_VDM_HEADER_IDX].val;
    am_info->vdo_numb[CY_PD_SOP]  = vdm->len - VDO_START_IDX;

    if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info->vdo_max_numb + VDO_START_IDX)))
    {
        /* Save received VDO */
        mem_copy((uint8_t *)am_info->vdo[CY_PD_SOP], (const uint8_t *)&(vdm->dat[VDO_START_IDX]),
              ((uint32_t)(am_info->vdo_numb[CY_PD_SOP]) * CY_PD_WORD_SIZE));
    }

    /* Run ufp alt mode cbk */
    am_info->mode_state = ALT_MODE_STATE_IDLE;
    if(NULL != am_info->cbk)
    {
        am_info->cbk(port);
    }
}

#if ATTENTION_QUEUE_SUPP
static void alt_mode_att_cb(uint8_t port, timer_id_t id)
{
    (void)id;
    cy_stc_pdstack_pd_packet_t vdm;
    alt_mode_mngr_status_t *am = &gl_alt_mode[port];
    
    if ((am->att_alt_mode != NULL) && (am->att_alt_mode->is_active))
    {
        /* Form VDM packet from the saved Attention and try to re-call attention handler in alt mode */
        vdm.dat[VDM_HEADER_IDX] = gl_alt_mode[port].att_header;
        vdm.dat[VDO_START_IDX]  = gl_alt_mode[port].att_vdo;
        vdm.len                 = VDO_START_IDX + 1;
        /* Run alt mode VDM handler */
        alt_mode_vdm_handle(port, am->att_alt_mode, &vdm);
    }    
}
#endif /* ATTENTION_QUEUE_SUPP */
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )
        
uint32_t get_active_alt_mode_mask(uint8_t port)
{
    return gl_alt_mode[port].am_active_modes;
}

uint32_t get_supp_alt_modes(uint8_t port)
{
    return gl_alt_mode[port].am_supported_modes;
}

void set_alt_mode_state(uint8_t port_idx, uint8_t alt_mode_idx)
{
    /* Set flag as active mode */
    SET_FLAG(gl_alt_mode[port_idx].am_active_modes, alt_mode_idx);
    REMOVE_FLAG(gl_alt_mode[port_idx].am_exited_modes, alt_mode_idx);
}
#endif /*CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED*/

#if UFP_ALT_MODE_SUPP

#if (CCG_BB_ENABLE != 0)    
static void bb_update(uint8_t port, uint8_t am_idx, bool bb_stat)
{
    if (bb_is_present(port) != false)
    {
        /* Disable AME timeout timer. */
        timer_stop(port, (timer_id_t)APP_AME_TIMEOUT_TIMER);
        if (bb_stat == false)
        {
            /* Enable BB controller */
            (void)bb_enable(port, BB_CAUSE_AME_FAILURE);
            /* Update BB alt mode status register as Error */
            (void)bb_update_alt_status(port, am_idx, BB_ALT_MODE_STAT_UNSUCCESSFUL);
        }
        else
        {
            /* Enable BB controller */
            (void)bb_enable(port, BB_CAUSE_AME_SUCCESS);
            /* Update BB alt mode status register as successful config */
            (void)bb_update_alt_status(port, am_idx, BB_ALT_MODE_STAT_SUCCESSFUL);
        }
    }
}
#endif /* (CCG_BB_ENABLE != 0) */

static bool ufp_reg_alt_mode(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm)
{
    uint8_t              svid_idx, vdo_numb, alt_mode_idx;
    pd_do_t             *dobj;
    alt_mode_info_t     *am_info_p;
    alt_mode_reg_info_t *reg       = &(gl_alt_mode[port].reg_info);
    uint32_t             svid      = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid;
    uint32_t             objPos   = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.objPos;

    /* Check if any alt mode is supported by DFP */
    if (get_alt_mode_numb(port) != 0u)
    {
        /* Get index of related svid register function */
        svid_idx = get_base_alt_mode_svid_idx(port, (uint16_t)svid);
        /* If SVID processing supported by CCG */
        if (svid_idx != MODE_NOT_SUPPORTED)
        {
            reg->data_role = (uint8_t)PRT_TYPE_UFP;
            /* Get SVID related VDOs and number of VDOs */
            if (get_modes_vdo_info (port, (uint16_t)svid, &dobj, &vdo_numb) == false)
            {
                return false;
            }
            /* Check if object position is not greather than VDO numb */
            if (objPos < vdo_numb)
            {
                /* Save Disc mode VDO and object position */
                reg->svid_vdo = dobj[objPos];
                /* Check if UFP support attached target alt mode */
                am_info_p = gl_reg_alt_mode[svid_idx].reg_am_ptr(port, reg);
                /* If VDO relates to any of supported alt modes */
                if ((gl_alt_mode[port].reg_info.alt_mode_id != MODE_NOT_SUPPORTED) && (am_info_p != NULL))
                {
                    /* If alternate modes discovered and could be runned */
                    /* Get index of alt mode in the compatibility table */
                    alt_mode_idx = get_alt_modes_config_svid_idx(port, PRT_TYPE_UFP, (uint16_t)svid);
                    if (alt_mode_idx != MODE_NOT_SUPPORTED)
                    {
                        /* Save alt mode ID and obj position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->objPos = (uint8_t)objPos;
                        /* Save pointer to alt mode info struct */
                        gl_alt_mode[port].alt_mode_info[alt_mode_idx] = am_info_p;
                        /* Set flag that alt mode could be runned */
                        SET_FLAG(gl_alt_mode[port].am_supported_modes, alt_mode_idx);
                        return true;
                    }
                }
            }
#if (CCG_BB_ENABLE != 0)               
            else if (bb_is_present(port) != false)
            {
                /* Go throught all alt modes */
                for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
                {
                    am_info_p = get_mode_info(port, alt_mode_idx);
                    /* If Alt mode with corresponded SVID already active then don't notify BB device */
                    if (
                            (am_info_p != NULL) && 
                            (am_info_p->VDM_HDR.svid == vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                            (am_info_p->is_active == true)
                       )
                    {
                        return false;
                    }                     
                    /*
                     * If Alt mode with corresponded SVID not activated and 
                     * Enter Mode command has object position which not supported 
                     * by CCG then enumerate BB
                     */
                      bb_update(port, svid_idx, false);
                }
            }
            else
            {
                /* No statement */
            }
#endif /* (CCG_BB_ENABLE != 0) */            
        }
    }

    return false;
}

static bool ufp_enter_alt_mode(uint8_t port, alt_mode_info_t *am_info_p, const cy_stc_pdstack_pd_packet_t *vdm, uint8_t am_idx)
{
    /* Process VDM */
    alt_mode_vdm_handle(port, am_info_p, vdm);
    
    /* QAC suppression 2991, 2995: This redundant check is retained for applications / future implementations
     * of alt_mode_vdm_handle that returns status other than ALT_MODE_STATE_IDLE. */
    if (am_info_p->mode_state != ALT_MODE_STATE_FAIL) /* PRQA S 2991, 2995 */
    {
        /* Set mode as active */
        am_info_p->is_active = true;

        /* Notify APP layer that ALT mode has been entered. */
        app_get_status(port)->alt_mode_entered = true;

        /* Send notifications to the solution if alt mode entered. */
        /* QAC suppression 0315: app_event_handler is designed to accept various types of data
         * for processing and hence the arguments are implicitly converted to void pointer. The 
         * argument passed here is not dangerous as it is 4 byte aligned. */
        app_event_handler (port, APP_EVT_ALT_MODE,
                form_alt_mode_event (port, /* PRQA S 0315 */
                    am_info_p->VDM_HDR.svid,
                        am_info_p->alt_mode_id,
                            AM_EVT_ALT_MODE_ENTERED, NO_DATA));

#if (CCG_BB_ENABLE != 0)
        /* Update BB status success */
        bb_update(port, am_idx, true);
#endif /* (CCG_BB_ENABLE != 0) */

        /* Set flag that alt mode could be processed */
        SET_FLAG(gl_alt_mode[port].am_active_modes, am_idx);
        return true;
    }

#if (CCG_BB_ENABLE != 0)
    /* Update BB status not success */
    bb_update(port, am_idx, false);
#endif /* (CCG_BB_ENABLE != 0) */
    return false;
}

static bool is_mode_activated(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm)
{
    uint8_t      idx;

    /* If any alt mode already registered */
    if (gl_alt_mode[port].am_supported_modes != NONE_MODE_MASK)
    {
        for (idx = 0; idx < gl_alt_mode[port].alt_modes_numb; idx++)
        {
            /* Try to find alt mode among supported alt modes */
            if (
                    /* QAC suppression 3415: Calling this function on the right hand side of &&'
                     * is not a side effect as it is not intended to call this function if the 
                     * left hand operand of '&&' evaluates to true. */
                    (gl_alt_mode[port].alt_mode_info[idx] != NULL) &&
                    (get_mode_info(port, idx)->VDM_HDR.svid == /* PRQA S 3415 */
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                    (get_mode_info(port, idx)->objPos == /* PRQA S 3415 */
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.objPos)
               )
            {
                /* return true if ufp alt mode already registered in alt mode mngr */
                return true;
            }
        }
    }

    /* Register alt mode and check possibility of alt mode entering
     * CDT 237168 */
    return ufp_reg_alt_mode(port, vdm);
}

#endif /* UFP_ALT_MODE_SUPP */

bool eval_rec_vdm(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm)
{
#if (!CCG_BACKUP_FIRMWARE)
    uint8_t         idx;
#if UFP_ALT_MODE_SUPP
    uint8_t         idx2;
    bool            enter_flag   = false;
#endif /* UFP_ALT_MODE_SUPP */
    alt_mode_info_t *am_info_p;
    cy_pd_pd_do_t         vdm_header   = vdm->dat[CY_PD_VDM_HEADER_IDX];
    vdm_resp_t*     vdm_response = &(app_get_status(port)->vdmResp);


    if (VDM_HDR.vdmType == (uint32_t)CY_PDSTACK_VDM_TYPE_STRUCTURED)
    {
        /* Discovery commands not processed by alt modes manager */
        if (VDM_HDR.cmd < (uint32_t)CY_PDSTACK_VDM_CMD_ENTER_MODE)
        {
            return false;
        }
        /* Save number of available alt modes */
        gl_alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);
#if UFP_ALT_MODE_SUPP
        /* If Enter mode cmd */
        if (
                (VDM_HDR.cmd == (uint8_t)CY_PDSTACK_VDM_CMD_ENTER_MODE) &&
                (gl_dpm_port_type[port] == PRT_TYPE_UFP)
           )
        {
            if (is_mode_activated(port, vdm) == true)
            {
                enter_flag = true;
            }
        }
#endif /* UFP_ALT_MODE_SUPP */
    }

    /* Go throught all alt modes */
    for (idx = 0; idx < gl_alt_mode[port].alt_modes_numb; idx++)
    {
        am_info_p = get_mode_info(port, idx);

        /* Check if received command processing allowed */
        if (
                (am_info_p != NULL) && (am_info_p->VDM_HDR.svid == VDM_HDR.svid) &&
                (
                    (am_info_p->obj_pos == VDM_HDR.objPos) ||
                    (am_info_p->custom_att_obj_pos == true) || 
                    ((am_info_p->uvdm_supp == true) && 
                    (VDM_HDR.vdmType == (uint32_t)CY_PDSTACK_VDM_TYPE_UNSTRUCTURED))
                )
            )
        {
#if UFP_ALT_MODE_SUPP
            /* If Enter mode cmd */
            if (enter_flag == true)
            {
                /* If alt mode already active then ACK */
                if (am_info_p->is_active == true)
                {
                    return true;
                }

                /* If all alt modes not active and just entered  */
                if (gl_alt_mode[port].am_active_modes == NONE_MODE_MASK)
                {
                    return ufp_enter_alt_mode(port, am_info_p, vdm, idx);
                }
                else
                {
                    /* Check alt mode in ufp consistent table  */
                    for (idx2 = 0; idx2 < gl_alt_mode[port].alt_modes_numb; idx2++)
                    {
                        /* Find table index of any active alt mode */
                        if ((IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, idx2) != 0u))
                        {
                            uint16_t* am_vdo_ptr = get_alt_modes_vdo_info(port, PRT_TYPE_DFP, idx2);
                            if (am_vdo_ptr != NULL)
                            {
                                uint8_t idx3;
                                /* Find out if selected alt mode could be processed simultaneously  with active alt modes */
                                for (idx3 = 0; idx3 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx3++)
                                {
                                    /* Check which alternate modes could be run simulateneously */
                                    if (am_vdo_ptr[idx3 + AM_SVID_CONFIG_OFFSET_IDX] == VDM_HDR.svid)
                                    {
                                        break;
                                    }
                                    /* If selected alt mode couldn't run simultaneously  with active alt modes return false */
                                    return false;
                                }
                            }
                        }             
                    }
                    /* Try to enter alt mode */
                    return ufp_enter_alt_mode(port,am_info_p, vdm, idx);
                 }
            }
            /* Any other received command */
            else
#endif /* UFP_ALT_MODE_SUPP */
            {
                /* Check if alt mode is active */
                if (
                       (am_info_p->is_active == false) ||
                       (
                           (am_info_p->mode_state != ALT_MODE_STATE_IDLE) &&
                            /* 
                             * In case a VDM is received as UFP, honour the request if you are waiting for
                             * a response or attention message to be sent out. Do not do this in case you 
                             * are DFP as a DFP state machine is driven from the DUT.
                             */
                            (
#if DFP_ALT_MODE_SUPP
                                (gl_dpm_port_type[port] == PRT_TYPE_DFP) ||
#endif /* DFP_ALT_MODE_SUPP */
                                (am_info_p->mode_state != ALT_MODE_STATE_WAIT_FOR_RESP)
                            )

#if ATTENTION_QUEUE_SUPP
                        && (VDM_HDR.cmd != VDM_CMD_ATTENTION)
#endif /* ATTENTION_QUEUE_SUPP */
                        )
                   )
                {
                    return false;
                }

                /* Save custom attention object position if needed */
                if (
                        (VDM_HDR.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ATTENTION) &&
                        (am_info_p->custom_att_obj_pos)    &&
                        (VDM_HDR.vdmType == (uint32_t)CY_PDSTACK_VDM_TYPE_STRUCTURED)
                    )
                {
                    /* QAC suppression 2982: Required for applications with UFP_ALT_MODE_ENABLE = 1 */
                    am_info_p->VDM_HDR.objPos = VDM_HDR.objPos; /* PRQA S 2982 */
                }

                /* Process VDM */
                alt_mode_vdm_handle(port, am_info_p, vdm);

                /* If command processed successful */
                /* QAC suppression 2991, 2995: This redundant check is retained for applications / future implementations
                 * of alt_mode_vdm_handle that returns status other than ALT_MODE_STATE_IDLE. */
                if (am_info_p->mode_state != ALT_MODE_STATE_FAIL) /* PRQA S 2991, 2995 */
                {
                    /* Copy VDM header to respond buffer if Unstructured*/
                    if (am_info_p->VDM_HDR.vdmType == (uint32_t)CY_PDSTACK_VDM_TYPE_UNSTRUCTURED)
                    {
                        vdm_response->respBuf[CY_PD_VDM_HEADER_IDX] = am_info_p->vdm_header;
                    }
                    /* Set number of data objects */
                    vdm_response->doCount = am_info_p->vdo_numb[CY_PD_SOP] + VDO_START_IDX;
                    if (am_info_p->vdo_numb[CY_PD_SOP] != NO_DATA)
                    {
                        /* If VDO resp is needed */
                        mem_copy((uint8_t *)&(vdm_response->respBuf[VDO_START_IDX]),
                        (const uint8_t *)am_info_p->vdo[CY_PD_SOP],
                                ((uint32_t)(am_info_p->vdo_numb[CY_PD_SOP]) * CY_PD_WORD_SIZE));

                    }
                    return true;
                }
                else
                {
                    /* Set alt mode state as idle */
                    am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                    return false;
                }
            }
        }

#if UFP_ALT_MODE_SUPP
        /* If Exit all modes */
        /* QAC suppressions 2813: Suspicious detection by the tool is invalid as the pointer
         * is always initialized with the correct address. */
        if (
                (VDM_HDR.vdmType == (uint8_t)CY_PDSTACK_VDM_TYPE_STRUCTURED)   &&
                (VDM_HDR.objPos == EXIT_ALL_MODES)         &&
                (VDM_HDR.cmd == (uint8_t)VDM_CMD_EXIT_MODE)          &&
                (am_info_p->VDM_HDR.svid == VDM_HDR.svid)   && /* PRQA S 2813 */
                (am_info_p->is_active)
            )
        {
            /* Save cmd */
            am_info_p->VDM_HDR.cmd = EXIT_ALL_MODES; /* PRQA S 2813 */
            if (am_info_p->cbk != NULL)
            {
                /* Run ufp alt mode cbk */
                am_info_p->cbk(port);
            }
            return true;
        }
#endif /* UFP_ALT_MODE_SUPP */
    }
#endif /* (!CCG_BACKUP_FIRMWARE) */
    return false;
}

uint8_t alt_mode_get_status(uint8_t port)
{
    uint8_t ret = NO_DATA;
#if DFP_ALT_MODE_SUPP
    alt_mode_info_t *am_info_p = NULL;
    uint8_t          alt_mode_idx;

    if (gl_alt_mode[port].state == ALT_MODE_MNGR_STATE_PROCESS)
    {
        /* Check each alternate mode */
        for (alt_mode_idx = 0; alt_mode_idx < gl_alt_mode[port].alt_modes_numb; alt_mode_idx++)
        {
            if (IS_FLAG_CHECKED(gl_alt_mode[port].am_active_modes, alt_mode_idx))
            {
                am_info_p = get_mode_info (port, alt_mode_idx);
                if (am_info_p != NULL)
                {
                    if (am_info_p->is_active != false)
                    {
                        /* Set alt modes which were entered */
                        SET_FLAG(ret, alt_mode_idx);
                    }
                }
            }
        }
        /* Mode discovery complete. */
        ret |= APP_DISC_COMPLETE_MASK;
    }
    /* Check if USB4 mode is entered */
    if (app_get_status(port)->usb4_active != false)
        ret |= APP_USB4_ACTIVE_MASK;
#else
    CY_UNUSED_PARAMETER(port);
#endif /* DFP_ALT_MODE_SUPP */

    return ret;
}


/* [] END OF FILE */

