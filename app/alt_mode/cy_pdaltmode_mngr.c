/******************************************************************************
* File Name:   cy_pdaltmode_mngr.c
* \version 2.0
*
* Description: Alternate Mode Source implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdaltmode_defines.h"

#include "cy_pdutils.h"
#include "cy_pdstack_dpm.h"
#include <cy_pdutils_sw_timer.h>
#include "app_timer_id.h"

#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_config.h"
#include "cy_pdaltmode_vdm_task.h"

#if GATKEX_CREEK
#include "cy_pdaltmode_intel_ridge.h"
#endif

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_host_details.h"
#endif

#if (CCG_BB_ENABLE != 0)
#include "cy_pdaltmode_billboard.h"
#endif /* (CCG_BB_ENABLE != 0) */

#if ((MAX_SUPP_ALT_MODES < MAX_SVID_SUPP) && ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)))  
#error "The number of user alternate modes exceed maximum supported alt modes. \
        Please increase MAX_SUPP_ALT_MODES"
#endif

#if (!CCG_BACKUP_FIRMWARE)
/* Handle received VDM for specific alt mode */
static void Cy_PdAltMode_Mngr_VdmHandle(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t *am_info, const cy_stc_pdstack_pd_packet_t *vdm);
#endif /* (!CCG_BACKUP_FIRMWARE) */ 

#if DFP_ALT_MODE_SUPP
/* Find next available alt mode for processing if previous exited */
static uint8_t Cy_PdAltMode_Mngr_GetNextAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /* DFP_ALT_MODE_SUPP   */

#if UFP_ALT_MODE_SUPP
/* This function verifies possibility of entering of corresponding UFP alt mode */
static bool Cy_PdAltMode_Mngr_IsModeActivated(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm);
/* UFP function for alt modes processing */
static bool Cy_PdAltMode_Mngr_UfpRegAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm);
/* Handles UFP enter mode processing */
static bool Cy_PdAltMode_Mngr_UfpEnterAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t *am_info_p, const cy_stc_pdstack_pd_packet_t *vdm, uint8_t am_idx);
#endif /* UFP_ALT_MODE_SUPP */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
/* Returns alt mode index for given svid */
static uint8_t Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */    

/* Composes alt mode info to vdm_msg_info_t struct before sending */
static uint8_t Cy_PdAltMode_Mngr_MoveToVdmInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t* info, cy_en_pd_sop_t sop_type);

/* Parses received VDM info and moves it to alt mode info struct */
static void Cy_PdAltMode_Mngr_GetVdmInfoVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t* info, cy_en_pd_sop_t sop_type);

/* Alt modes mngr AMS Prototypes */
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_RunDiscMode(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_EvalDiscMode(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_DiscModeFail(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_MonitorAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_EvalAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_FailAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext);
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_AltModeMngrDeinit(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static uint16_t* Cy_PdAltMode_Mngr_GetAltModesVdoInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_port_type_t type, uint8_t idx);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */


/*State Table*/
static cy_en_pdaltmode_vdm_task_t (*const alt_mode_ams_table [ALT_MODE_MNGR_STATE_PROCESS + 1u]
        [VDM_EVT_EXIT + 1u]) (cy_stc_pdaltmode_context_t *ptrAltModeContext) = {
    {
        /* Send next discovery svid */
        Cy_PdAltMode_Mngr_RunDiscMode,
        /* Evaluate disc svid response */
        Cy_PdAltMode_Mngr_EvalDiscMode,
        /* Process failed disc svid response */
        Cy_PdAltMode_Mngr_DiscModeFail,
        /* Exit from alt mode manager */
        Cy_PdAltMode_Mngr_AltModeMngrDeinit
    },
    {
        /* Monitor if any changes appears in modes  */
        Cy_PdAltMode_Mngr_MonitorAltModes,
        /* Evaluate alt mode response */
        Cy_PdAltMode_Mngr_EvalAltModes,
        /* Process failed alt modes response */
        Cy_PdAltMode_Mngr_FailAltModes,
        /* Exit from alt mode manager */
        Cy_PdAltMode_Mngr_AltModeMngrDeinit
    }
};

void Cy_PdAltMode_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if STORE_DETAILS_OF_HOST
    Cy_PdAltMode_HostDetails_Task(ptrAltModeContext);
#else
    (void) ptrAltModeContext;
#endif /* STORE_DETAILS_OF_HOST */
}

/************************* DFP Related Function definitions *******************/

cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_RegAltModeMngr(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info, cy_stc_pdaltmode_vdm_msg_info_t* vdm_msg_info)
{
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_live = (bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3);
#endif

    ptrAltModeContext->altModeStatus.alt_modes_numb = Cy_PdAltMode_Mngr_GetAltModeNumb(ptrAltModeContext);
    ptrAltModeContext->altModeStatus.vdm_info       = vdm_msg_info;

    /* Check device role to start with. */
    if(ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
    {
        if (atch_tgt_info->tgt_svid[0] == CY_PDALTMODE_NO_DATA)
        {
#if CY_PD_USB4_SUPPORT_ENABLE
            if(ptrAltModeContext->appStatus.usb4_active != false)
            {
                return VDM_TASK_ALT_MODE;
            }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
            /* UFP does not support any of the SVIDs of interest. Exit VDM manager. */
            return VDM_TASK_EXIT;
        }
        else
        {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            if (Cy_PdAltMode_Mngr_GetAltModeNumb(ptrAltModeContext) != 0)
            {
                /* Register pointers to VDM mngr info */
                ptrAltModeContext->altModeStatus.reg_info.atch_tgt_info  = atch_tgt_info;
                /* Set alt modes mngr state to Discovery Mode process */
                ptrAltModeContext->altModeStatus.reg_info.data_role      = CY_PD_PRT_TYPE_DFP;
                ptrAltModeContext->altModeStatus.state                   = ALT_MODE_MNGR_STATE_DISC_MODE;

                /* Set alt mode trigger based on config */
                ptrAltModeContext->appStatus.alt_mode_trig_mask = 0;
                return VDM_TASK_ALT_MODE;
            }
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
        }
    }
    else
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        ptrAltModeContext->altModeStatus.pd3_ufp = pd3_live;
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        if (atch_tgt_info->tgt_svid[0] == CY_PDALTMODE_NO_DATA)
        {
            /* If we have no SVIDs to evaluate by UFP then go to regular monitoring */
            ptrAltModeContext->altModeStatus.state    = ALT_MODE_MNGR_STATE_PROCESS;
            return VDM_TASK_ALT_MODE;
        }
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        else
        {
            if (pd3_live)
            {
                /* If PD spec revision is 3.0, we can start with discover mode process. */
                return VDM_TASK_ALT_MODE;
            }
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
    }

    return VDM_TASK_EXIT;
}

bool Cy_PdAltMode_Mngr_IsAltModeMngrIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    bool    is_idle = true;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    cy_stc_pdaltmode_mngr_info_t*  am_info_p = NULL;
    uint8_t           alt_mode_idx;

    if ((ptrAltModeContext->altModeStatus.state == ALT_MODE_MNGR_STATE_DISC_MODE) ||
            (!Cy_PdAltMode_HW_IsIdle(ptrAltModeContext)))
    {
        return false;
    }

    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);
        /* If mode is active */
        if ((IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx) != 0u))
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
    (void)ptrAltModeContext;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

    return (is_idle);
}

void Cy_PdAltMode_Mngr_Sleep(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    Cy_PdAltMode_HW_Sleep (ptrAltModeContext);
#else
    CY_UNUSED_PARAMETER(ptrAltModeContext);
#endif
}

void Cy_PdAltMode_Mngr_Wakeup(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    Cy_PdAltMode_HW_Wakeup (ptrAltModeContext);
#else
    CY_UNUSED_PARAMETER(ptrAltModeContext);
#endif
}

cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_AltModeProcess(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt)
{
    /* Run alt modes manager ams table */
    return alt_mode_ams_table[ptrAltModeContext->altModeStatus.state][vdm_evt](ptrAltModeContext);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static void Cy_PdAltMode_Mngr_SetDiscModeParams(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_sop_t sop)
{
    cy_stc_pdaltmode_vdm_msg_info_t *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;

    vdm_p->vdm_header.val   = CY_PDALTMODE_NO_DATA;
    vdm_p->vdm_header.std_vdm_hdr.svid     = ptrAltModeContext->altModeStatus.reg_info.atch_tgt_info->tgt_svid[ptrAltModeContext->altModeStatus.svid_idx];
    vdm_p->vdm_header.std_vdm_hdr.cmd      = CY_PDSTACK_VDM_CMD_DSC_MODES;
    vdm_p->vdm_header.std_vdm_hdr.objPos   = CY_PDALTMODE_NO_DATA;
    vdm_p->sopType         = sop;
    vdm_p->vdo_numb         = CY_PDALTMODE_NO_DATA;
    vdm_p->vdm_header.std_vdm_hdr.vdmType = CY_PDSTACK_VDM_TYPE_STRUCTURED;
}

static void Cy_PdAltMode_Mngr_SendSlnEventNoData(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid, uint8_t am_id, cy_en_pdaltmode_app_evt_t evtype)
{
    ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
            Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext, svid, am_id, evtype, CY_PDALTMODE_NO_DATA));
}

static void Cy_PdAltMode_Mngr_SendSlnAppEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t data)
{
    ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
            Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext, ptrAltModeContext->altModeStatus.vdm_info->vdm_header.std_vdm_hdr.svid,
                ptrAltModeContext->altModeStatus.reg_info.alt_mode_id,
                (data == CY_PDALTMODE_NO_DATA) ? ptrAltModeContext->altModeStatus.reg_info.app_evt : AM_EVT_DATA_EVT, data));
}

static void Cy_PdAltMode_Mngr_SendAppEvtWrapper(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg)
{
    if (reg->app_evt != AM_NO_EVT)
    {
        /* Send notifications to the solution. */
        Cy_PdAltMode_Mngr_SendSlnAppEvt(ptrAltModeContext, CY_PDALTMODE_NO_DATA);
        reg->app_evt = AM_NO_EVT;
    }
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_RunDiscMode(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    (void)ptrAltModeContext;
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

    uint16_t cur_svid;

    /* Set cable sop flag not needed at start*/
    ptrAltModeContext->altModeStatus.reg_info.cbl_sop_flag = CY_PD_SOP_INVALID;

    /* Search for next SVID until svid array is not empty */
    while ((cur_svid = ptrAltModeContext->altModeStatus.reg_info.atch_tgt_info->tgt_svid[ptrAltModeContext->altModeStatus.svid_idx]) != 0)
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        /* Check is current port date role UFP */
        if (ptrAltModeContext->altModeStatus.pd3_ufp)
        {
            /* Send Disc Mode cmd */
            Cy_PdAltMode_Mngr_SetDiscModeParams (port, SOP);
            return VDM_TASK_SEND_MSG;
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

#if DFP_ALT_MODE_SUPP
        if (Cy_PdAltMode_Mngr_IsSvidSupported(ptrAltModeContext, cur_svid) != MODE_NOT_SUPPORTED)
        {
            /* Send notifications to the solution. */
            Cy_PdAltMode_Mngr_SendSlnEventNoData (ptrAltModeContext, cur_svid, 0, AM_EVT_SVID_SUPP);

            /* If SVID is supported send Disc Mode cmd */
            Cy_PdAltMode_Mngr_SetDiscModeParams (ptrAltModeContext, CY_PD_SOP);
            return VDM_TASK_SEND_MSG;
        }

#if (SAVE_SUPP_SVID_ONLY == 0)
        /* Send notifications to the solution. */
        Cy_PdAltMode_Mngr_SendSlnEventNoData (ptrAltModeContext, cur_svid, 0, AM_EVT_SVID_NOT_SUPP);
#endif /* SAVE_SUPP_SVID_ONLY */

        /* If svid not supported */
        ptrAltModeContext->altModeStatus.svid_idx++;
#endif /* DFP_ALT_MODE_SUPP */
    }

    if ( 
            (ptrAltModeContext->altModeStatus.am_supported_modes == NONE_MODE_MASK)
#if CY_PD_USB4_SUPPORT_ENABLE
            && (ptrAltModeContext->appStatus.usb4_active == false)
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
       )
    {
        /* No supp modes */
        return VDM_TASK_EXIT;
    }

#if (!ICL_ALT_MODE_HPI_DISABLED)
    /* Send SVID discovery finished notification to the solution. Enabled only if HPI commands are enabled. */
    Cy_PdAltMode_Mngr_SendSlnEventNoData (ptrAltModeContext, 0, 0, AM_EVT_DISC_FINISHED);
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */    

#if DFP_ALT_MODE_SUPP
    /* Goto alt mode process */
    Cy_PdAltMode_Mngr_GetNextAltMode(ptrAltModeContext);
#endif /* DFP_ALT_MODE_SUPP */
    ptrAltModeContext->altModeStatus.state = ALT_MODE_MNGR_STATE_PROCESS;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
static void Cy_PdAltMode_Mngr_HandleCblDiscMode(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool failed)
{
    uint8_t tbl_svid_idx, vdo_idx;
    cy_stc_pdaltmode_mngr_info_t *am_info_p;
    cy_stc_pdaltmode_vdm_msg_info_t    *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;
    cy_stc_pdaltmode_alt_mode_reg_info_t *reg   = &(ptrAltModeContext->altModeStatus.reg_info);

    /*
       Get index of function in alt_mode_config related to receive SVID.
       This is expected to be valid as we would already have gone through
       Discover MODE for the UFP.
       */
    tbl_svid_idx = Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(ptrAltModeContext, vdm_p->vdm_header.std_vdm_hdr.svid);
    reg->atch_type = CABLE;

    /* Analyse all received VDOs. */
    for (vdo_idx = 0; ((failed) || (vdo_idx < vdm_p->vdo_numb)); vdo_idx++)
    {
        if (failed)
        {
            reg->cbl_sop_flag = CY_PD_SOP_INVALID;
        }
        else
        {
            /* Save current VDO and its position in svid structure */
            reg->svid_emca_vdo = vdm_p->vdo[vdo_idx];
        }

        /* Check if DFP support attached target alt mode */
        am_info_p = gl_reg_alt_mode[tbl_svid_idx].reg_am_ptr (ptrAltModeContext, reg);
        if (am_info_p == NULL)
        {
            /* Get index of alt mode in the compatibility table */
            uint8_t cfg_svid_idx = Cy_PdAltMode_Mngr_IsSvidSupported(ptrAltModeContext, vdm_p->vdm_header.std_vdm_hdr.svid);
            if (cfg_svid_idx != MODE_NOT_SUPPORTED)
            {
                /* Remove pointer to alt mode info struct */
                ptrAltModeContext->altModeStatus.alt_mode_info[cfg_svid_idx] = NULL;

                /* Remove flag that alt mode could be run */
                REMOVE_FLAG(ptrAltModeContext->altModeStatus.am_supported_modes, cfg_svid_idx);
            }
            reg->cbl_sop_flag = CY_PD_SOP_INVALID;
        }
        else
        {
            if (!failed)
                am_info_p->cbl_obj_pos = (vdo_idx + VDO_START_IDX);
        }

        Cy_PdAltMode_Mngr_SendAppEvtWrapper(ptrAltModeContext, reg);

        if (failed)
            break;
    }
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_EvalDiscMode(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    volatile uint8_t               vdo_idx, tbl_svid_idx, cfg_svid_idx;
    cy_stc_pdaltmode_mngr_info_t      *am_info_p;
    cy_stc_pdaltmode_vdm_msg_info_t       *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;
    cy_stc_pdaltmode_alt_mode_reg_info_t  *reg   = &(ptrAltModeContext->altModeStatus.reg_info);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (ptrAltModeContext->altModeStatus.pd3_ufp)
    {
        /* Goto next SVID */
        ptrAltModeContext->altModeStatus.svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    /* Evaluate SOP response */
    if (vdm_p->sopType == (uint8_t)CY_PD_SOP)
    {
        /* Get index of function in alt_mode_config related to receive SVID */
        tbl_svid_idx = Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(ptrAltModeContext, vdm_p->vdm_header.std_vdm_hdr.svid);

        /* Get index of SVID related configuration from config.c */
        cfg_svid_idx = Cy_PdAltMode_Mngr_IsSvidSupported(ptrAltModeContext, vdm_p->vdm_header.std_vdm_hdr.svid);

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
                am_info_p =  gl_reg_alt_mode[tbl_svid_idx].reg_am_ptr(ptrAltModeContext, reg);

                /* If VDO relates to any of supported alt modes */
                if (reg->alt_mode_id != MODE_NOT_SUPPORTED)
                {
                    Cy_PdAltMode_Mngr_SendAppEvtWrapper(ptrAltModeContext, reg);

                    /* If alternate modes discovered and could be run */
                    if (am_info_p != NULL)
                    {
                        /* Save alt mode ID and object position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->obj_pos     = (vdo_idx + VDO_START_IDX);
#if (!ICL_ALT_MODE_HPI_DISABLED)                        
                        if (am_info_p->app_evt_needed != false)
                        {
                            /* Send notifications to the solution. */
                            Cy_PdAltMode_Mngr_SendSlnAppEvt (ptrAltModeContext, am_info_p->app_evt_data.val);
                            am_info_p->app_evt_needed = false;
                        }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */                        
                        /* Save pointer to alt mode info struct */
                        ptrAltModeContext->altModeStatus.alt_mode_info[cfg_svid_idx] = am_info_p;

                        /* Set flag that alt mode could be run */
                        SET_FLAG(ptrAltModeContext->altModeStatus.am_supported_modes, cfg_svid_idx);
                    }
                }
            }

            /* If cable DISC Mode is needed - send VDM */
            if (reg->cbl_sop_flag != CY_PD_SOP_INVALID)
            {
                Cy_PdAltMode_Mngr_SetDiscModeParams (ptrAltModeContext, CY_PD_SOP_PRIME);
                return VDM_TASK_SEND_MSG;
            }
        }
    }
    /* Evaluate cable response: Packet type will be SOP' or SOP'' here. */
    else
    {
        Cy_PdAltMode_Mngr_HandleCblDiscMode(ptrAltModeContext, false);

        /* If cable DISC Mode is needed - send VDM */
        if (reg->cbl_sop_flag != CY_PD_SOP_INVALID)
        {
            Cy_PdAltMode_Mngr_SetDiscModeParams (ptrAltModeContext, CY_PD_SOP_DPRIME);
            return VDM_TASK_SEND_MSG;
        }
    }

    /* If no any result goto next SVID */
    ptrAltModeContext->altModeStatus.svid_idx++;
#else
    (void)ptrAltModeContext;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    return VDM_TASK_ALT_MODE;
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_DiscModeFail(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (ptrAltModeContext->altModeStatus.pd3_ufp)
    {
        /* Goto next SVID */
        ptrAltModeContext->altModeStatus.svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    if (ptrAltModeContext->altModeStatus.vdm_info->sopType == (uint8_t)CY_PD_SOP_PRIME)
    {
        Cy_PdAltMode_Mngr_HandleCblDiscMode(ptrAltModeContext, true);
    }
    /* If Disc Mode cmd fails goto next SVID */
    ptrAltModeContext->altModeStatus.svid_idx++;
#else
    (void)ptrAltModeContext;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_MonitorAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t alt_mode_idx;
    cy_en_pdaltmode_state_t alt_mode_state;
    cy_en_pdaltmode_vdm_task_t stat = VDM_TASK_ALT_MODE;
    cy_en_pd_sop_t sop_state;
    cy_stc_pdaltmode_mngr_info_t  *am_info_p  = NULL;

    uint8_t fault_status = *(ptrAltModeContext->fault_status_p);

#if ATTENTION_QUEUE_SUPP
    /* Check if MUX is not busy */
    if (
            (ptrAltModeContext->appStatus.mux_stat == MUX_STATE_BUSY)
#if ((!VIRTUAL_HPD_ENABLE) && (DP_DFP_SUPP))
            || (ptrAltModeContext->hwDetails.app_mux_update_req != false)
#endif /* (!VIRTUAL_HPD_ENABLE) && (DP_DFP_SUPP) */
       )
    {
        return stat;
    }
#endif /* ATTENTION_QUEUE_SUPP */

    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);
            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            switch (alt_mode_state)
            {
                case ALT_MODE_STATE_SEND:
                    /* This case activates when VDM sequence for given alt mode */
                    /* was interrupted by alt mode with higher priority */
                case ALT_MODE_STATE_WAIT_FOR_RESP:
                    /*
                     * Check if SOP' or SOP'' messages are required.
                     * We do not send cable messages if VConn fault is present.
                     */
                    if((fault_status & (FAULT_APP_PORT_VCONN_FAULT_ACTIVE | FAULT_APP_PORT_V5V_SUPPLY_LOST)) != 0)
                    {
                        am_info_p->sop_state[CY_PD_SOP_PRIME]  = ALT_MODE_STATE_IDLE;
                        am_info_p->sop_state[CY_PD_SOP_DPRIME] = ALT_MODE_STATE_IDLE;
                    }

                    /* CDT 300965: Set Exit mode sequence SOP -> SOP''-> SOP' */
                    if
#if DP_DFP_SUPP
                        (
#endif /* DP_DFP_SUPP */
                            (am_info_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE)
#if DP_DFP_SUPP
                            || (
                                (am_info_p->vdm_header.std_vdm_hdr.svid == CY_PDALTMODE_DP_SVID) && 
                                (am_info_p->vdo[CY_PD_SOP]->val == CY_PDALTMODE_DP_USB_SS_CONFIG) &&
                                (am_info_p->vdm_header.std_vdm_hdr.cmd == DP_STATE_CONFIG)
                               )                    
                       )
#endif /* DP_DFP_SUPP */
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
                            ((am_info_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ENTER_MODE) ||
                             (am_info_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE)) &&
                            (am_info_p->set_mux_isolate != false)          &&
                            ((ptrAltModeContext->altModeStatus.am_active_modes & (~(1u << alt_mode_idx))) == false)
                       )
                    {
#if GATKEX_CREEK
                        /* To set the ridge to disconnect between USB and TBT states */
                        Cy_PdAltMode_Ridge_SetDisconnect(ptrAltModeContext);
#else
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
                        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SAFE, CY_PDALTMODE_NO_DATA);
#endif /* (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP) */
#endif
                    }

                    if (
                            (am_info_p->is_active) ||
                            (am_info_p->vdm_header.std_vdm_hdr.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_ENTER_MODE) || 
                            (am_info_p->vdm_header.std_vdm_hdr.cmd == (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE)
                       )
                    {
                        /* Copy to vdm info and send vdm */
                        Cy_PdAltMode_Mngr_MoveToVdmInfo(ptrAltModeContext, am_info_p, sop_state);
                        am_info_p->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
                    }

                    stat = VDM_TASK_SEND_MSG;
                    break;

                case ALT_MODE_STATE_EXIT:
#if HPI_AM_SUPP
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
                    if (ptrAltModeContext->altModeStatus.reset_custom_mode != false)
                    {
                        /* Send enter mode with new custom svid alt mode with new SVID */
                        if (am_info_p->cbk != NULL)
                        {
                            cy_stc_pdaltmode_alt_mode_evt_t tmp_data;
                            /* Send command to change custom SVID */
                            am_info_p->eval_app_cmd(port, tmp_data);
                            am_info_p->mode_state = ALT_MODE_STATE_INIT;
                            am_info_p->cbk(ptrAltModeContext);
                            /* Set flag that reset is required */
                            ptrAltModeContext->altModeStatus.reset_custom_mode = false;
                            break;
                        }                   
                    }
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */
#endif /* HPI_AM_SUPP */

#if DFP_ALT_MODE_SUPP
                    if(ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
                    {
                        /* Remove flag from active mode */
                        REMOVE_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx);

                        /* Set alt mode as disabled */
                        am_info_p->mode_state = ALT_MODE_STATE_DISABLE;

                        /* Set flag as exited mode */
                        SET_FLAG(ptrAltModeContext->altModeStatus.am_exited_modes, alt_mode_idx);

                        /* If any active modes are present */
                        if (ptrAltModeContext->altModeStatus.am_active_modes == NONE_MODE_MASK)
                        {
                            /* Notify APP layer that ALT mode has been exited. */
                            ptrAltModeContext->appStatus.alt_mode_entered = false;

                            /* Set MUX to SS config */
                            Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SS_ONLY, CY_PDALTMODE_NO_DATA);
                            if (ptrAltModeContext->altModeStatus.exit_all_flag == false)
                            {
                                /* Get next alt mode if available */
                                Cy_PdAltMode_Mngr_GetNextAltMode(ptrAltModeContext);
                            }
                            else
                            {
                                /* Run callback command after all alt modes were exited */
                                ptrAltModeContext->altModeStatus.exit_all_cbk(ptrAltModeContext->pdStackContext, true);
                                ptrAltModeContext->altModeStatus.exit_all_flag = false;
                                stat = VDM_TASK_WAIT;
                            }

                        }
                    }
#endif /* DFP_ALT_MODE_SUPP */

#if UFP_ALT_MODE_SUPP
                    if(ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
                    {
                        if (ptrAltModeContext->altModeStatus.am_active_modes != NONE_MODE_MASK)
                        {
                            /* Notify APP layer that ALT mode has been exited. */
                            ptrAltModeContext->appStatus.alt_mode_entered = false;
                        }

#if (!ICL_ALT_MODE_HPI_DISABLED)
                        /* Send notifications to the solution if alt mode was entered. */
                        ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
                                Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext,am_info_p->vdm_header.std_vdm_hdr.svid,
                                    am_info_p->alt_mode_id, AM_EVT_ALT_MODE_EXITED, CY_PDALTMODE_NO_DATA));
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

                        /* Set alt mode not active */
                        am_info_p->is_active = false;

                        /* Remove flag that alt mode could be processed */
                        REMOVE_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx);
                    }
#endif /* UFP_ALT_MODE_SUPP   */
                    break;

                case ALT_MODE_STATE_IDLE:
#if (!ICL_ALT_MODE_HPI_DISABLED)
                    /* If alt modes need to send app event data */
                    if (am_info_p->app_evt_needed != false)
                    {
                        /* Send notifications to the solution. */
                        ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
                                Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext,am_info_p->vdm_header.std_vdm_hdr.svid,
                                    am_info_p->alt_mode_id, AM_EVT_DATA_EVT, am_info_p->app_evt_data.val));

                        am_info_p->app_evt_needed = false;
                    }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */                
                    break;

                case ALT_MODE_STATE_RUN:
                    /* Run ufp evaluation function */
                    if (am_info_p->cbk != NULL)
                    {
                        am_info_p->cbk(ptrAltModeContext);
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

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_EvalAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t         alt_mode_idx;
    bool            eval_flag   = false;
    bool            skip_handle = false;
    cy_en_pdaltmode_state_t   alt_mode_state;
    cy_en_pdaltmode_vdm_task_t         stat = VDM_TASK_ALT_MODE;
    cy_stc_pdaltmode_mngr_info_t    *am_info_p;
    cy_stc_pdaltmode_vdm_msg_info_t     *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;

#if (!ICL_ALT_MODE_EVTS_DISABLED)    
    cy_en_pdaltmode_app_evt_t appevt_type = AM_NO_EVT;
#if (!ICL_ALT_MODE_HPI_DISABLED)
    uint32_t           appevt_data = CY_PDALTMODE_NO_DATA;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif /*  !ICL_ALT_MODE_EVTS_DISABLED */   

    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);

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
                    Cy_PdAltMode_Mngr_GetVdmInfoVdo(ptrAltModeContext, am_info_p, (cy_en_pd_sop_t)vdm_p->sopType);

                    /* CDT 300965: Set Exit mode sequence SOP -> SOP''-> SOP' */
                    if (am_info_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE)
                    {
                        if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed send SOP'' VDM */
                            next_sop = CY_PD_SOP_DPRIME;
                        }
                        else if (am_info_p->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If SOP'' not needed - send SOP VDM */
                            next_sop = CY_PD_SOP_PRIME;
                        }
                        else
                        {
                            eval_flag = true;
                        }
                    }
#if DP_DFP_SUPP
                    /* CDT 300965: Set DP USB SS command sequence SOP -> SOP'-> SOP'' */
                    if ((am_info_p->vdm_header.std_vdm_hdr.svid == CY_PDALTMODE_DP_SVID) &&
                            (am_info_p->vdo[CY_PD_SOP]->val == CY_PDALTMODE_DP_USB_SS_CONFIG) &&
                            (am_info_p->vdm_header.std_vdm_hdr.cmd == DP_STATE_CONFIG))
                    {
                        if (am_info_p->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed send SOP'' VDM */
                            next_sop = CY_PD_SOP_PRIME;
                        }
                        else if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed, send SOP'' VDM */
                            next_sop = CY_PD_SOP_DPRIME;
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
                        /* If received VDM is SOP */
                        if ((vdm_p->sopType == CY_PD_SOP) || (eval_flag != false))
                        {
                            /* Run alt mode analysis function */
                            am_info_p->cbk(ptrAltModeContext);
                            /* If UVDM command then break */
                            if (vdm_p->vdm_header.std_vdm_hdr.vdmType != CY_PDSTACK_VDM_TYPE_STRUCTURED)
                            {
                                break;
                            }
                            /* If alt mode entered */
                            if (vdm_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ENTER_MODE)
                            {
                                /* Notify APP layer that ALT mode has been entered. */
                                ptrAltModeContext->appStatus.alt_mode_entered = true;

                                /* Set mode as active */
                                am_info_p->is_active = true;

#if (!ICL_ALT_MODE_EVTS_DISABLED)
                                /* Queue notifications to the solution. */
                                appevt_type = AM_EVT_ALT_MODE_ENTERED;
#endif /* (!ICL_ALT_MODE_EVTS_DISABLED) */
                            }

                            if (vdm_p->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE)
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
                                ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
                                        Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext, am_info_p->vdm_header.std_vdm_hdr.svid,
                                            am_info_p->alt_mode_id, appevt_type, appevt_data));
#else
                                Cy_PdAltMode_Mngr_SendSlnEventNoData (port, 
                                        am_info_p->VDM_HDR.svid,
                                        am_info_p->alt_mode_id,
                                        appevt_type);
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
                            }
#endif /* (!ICL_ALT_MODE_EVTS_DISABLED) */
                        }
                        else
                        {
                            /* If received VDM is SOP' type, check if SOP'' is needed.
                               If received VDM is SOP'', the check on SOP_DPRIME will fail trivially. */
                            if (am_info_p->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_SEND)
                            {
                                /* If needed send SOP'' VDM */
                                next_sop = CY_PD_SOP_DPRIME;
                            }
                            else if (am_info_p->sop_state[CY_PD_SOP] == ALT_MODE_STATE_SEND)
                            {
                                /* If SOP'' not needed - send SOP VDM */
                                next_sop = CY_PD_SOP;
                            }
                            else
                            {
                                am_info_p->cbk(ptrAltModeContext);
                            }
                        }
                    }

                    if (next_sop != CY_PD_SOP_INVALID)
                    {
                        (void)Cy_PdAltMode_Mngr_MoveToVdmInfo (ptrAltModeContext, am_info_p, next_sop);
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

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_FailAltModes(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t             alt_mode_idx;
    cy_en_pdaltmode_state_t    alt_mode_state;
    cy_en_pdaltmode_app_evt_t  appevt_type = AM_EVT_CBL_RESP_FAILED;
    cy_stc_pdaltmode_mngr_info_t     *am_info_p = NULL;
    cy_stc_pdaltmode_vdm_msg_info_t      *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;

    /* Look through all alt modes */
    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx) != 0u)
        {
            am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);

            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;

            /* If mode waits for response */
            if (alt_mode_state == ALT_MODE_STATE_WAIT_FOR_RESP)
            {
                /* Change status to fail */
                am_info_p->sop_state[vdm_p->sopType] = ALT_MODE_STATE_FAIL;

                /* Set Failure code at the object pos field */
                am_info_p->vdm_header.std_vdm_hdr.objPos = vdm_p->vdm_header.std_vdm_hdr.objPos;

                if (vdm_p->sopType == CY_PD_SOP)
                {
                    appevt_type = AM_EVT_SOP_RESP_FAILED;
                }

                /* Send notifications to the solution. */
                ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
                        Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext, am_info_p->vdm_header.std_vdm_hdr.svid,
                            am_info_p->alt_mode_id, appevt_type, CY_PDALTMODE_NO_DATA));

                /* Run alt mode analysis function. */
                am_info_p->mode_state = ALT_MODE_STATE_FAIL;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(ptrAltModeContext);
                }
            }
        }
    }

    return VDM_TASK_ALT_MODE;
}

#if DFP_ALT_MODE_SUPP
static uint8_t Cy_PdAltMode_Mngr_GetNextAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if LEGACY_CFG_TABLE_SUPPORT
    uint16_t* am_vdo_ptr = NULL;
#endif /* LEGACY_CFG_TABLE_SUPPORT */
    uint8_t am_idx = MODE_NOT_SUPPORTED;
    uint32_t idx1, idx2, am_config_svid, am_numb = ptrAltModeContext->altModeStatus.alt_modes_numb;
    uint32_t aval_modes = (
            (ptrAltModeContext->altModeStatus.am_supported_modes) &
            (~ptrAltModeContext->altModeStatus.am_exited_modes) &
            (~ptrAltModeContext->appStatus.alt_mode_trig_mask)
            );

    /* Start looking for next supported alt modes */
    for (idx1 = 0; idx1 < am_numb; idx1++)
    {
        /* If mode supported by CCG and not processed yet */
        if (IS_FLAG_CHECKED((aval_modes), idx1))
        {
#if LEGACY_CFG_TABLE_SUPPORT
            /* Look for the alt modes which could be run */
            am_vdo_ptr = Cy_PdAltMode_Mngr_GetAltModesVdoInfo(ptrPdStackContext, CY_PD_PRT_TYPE_DFP, idx1);
            am_config_svid = CY_PDALTMODE_NO_DATA;
            for (idx2 = 0; idx2 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx2++)
            {
                /* Check which alternate modes could be run simultaneously */
                am_idx = Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(ptrPdStackContext, CY_PD_PRT_TYPE_DFP,
                        am_vdo_ptr[idx2 + AM_SVID_CONFIG_OFFSET_IDX]);
                if (am_idx != MODE_NOT_SUPPORTED)
                {
                    SET_FLAG(am_config_svid, am_idx);
                }
            }
#else
            /* Look for the alt modes which could be run */
            am_config_svid = CY_PDALTMODE_NO_DATA;
            for (idx2 = 0; idx2 < 4u; idx2++)
            {
                /* Check which alternate modes could be run simultaneously */
                am_idx = Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(ptrAltModeContext, CY_PD_PRT_TYPE_DFP, ptrAltModeContext->altModeCfg->supported_alt_mode[idx2]);
                if (am_idx != MODE_NOT_SUPPORTED)
                {
                    SET_FLAG(am_config_svid, am_idx);     
                }
            }
#endif /* LEGACY_CFG_TABLE_SUPPORT */

#if HPI_AM_SUPP
            /* Check if HPI SVID is supported */
            if (app_get_status(port)->custom_hpi_svid != CY_PDALTMODE_NO_DATA)
            {
                uint8_t hpi_svid_idx = Cy_PdAltMode_Mngr_IsSvidSupported(app_get_status(port)->custom_hpi_svid, port);

                /* Set bit as active if it's HPI alt mode */
                if (hpi_svid_idx != MODE_NOT_SUPPORTED)
                {
                    SET_FLAG(am_config_svid, hpi_svid_idx);
                }
            }
#endif /* #if HPI_AM_SUPP */    

            /* Set alternate mode Enter mask */
            ptrAltModeContext->altModeStatus.am_active_modes = am_config_svid & aval_modes;

            return true;
        }
    }

    return false;
}

#endif /* DFP_ALT_MODE_SUPP   */

#if (!UCSI_ALT_MODE_ENABLED)
static uint8_t Cy_PdAltMode_Mngr_MoveToVdmInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t *info, cy_en_pd_sop_t sop_type)
#else
uint8_t Cy_PdAltMode_Mngr_MoveToVdmInfo(uint8_t port, cy_stc_pdaltmode_mngr_info_t *info, sop_t sop_type)
#endif /*UCSI_ALT_MODE_ENABLED*/
{
    cy_stc_pdaltmode_vdm_msg_info_t *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;

    vdm_p->sopType   = sop_type;
    vdm_p->vdo_numb   = CY_PDALTMODE_NO_DATA;
    vdm_p->vdm_header = info->vdm_header;    
    if ((info->vdo[sop_type] != NULL) && (info->vdo_numb[sop_type] != CY_PDALTMODE_NO_DATA))
    {
        vdm_p->vdo_numb = info->vdo_numb[sop_type];
        /* Save received VDO */
        CY_PDUTILS_MEM_COPY((uint8_t *)vdm_p->vdo, (const uint8_t *)info->vdo[sop_type],((uint32_t)(info->vdo_numb[sop_type]) << 2u));
    }
    if (
            info->uvdm_supp == false
            /* If SVDM */
#if UVDM_SUPP
            ||  ((info->vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
                &&   (info->uvdm_supp))
#endif /* UVDM_SUPP */        
       )
    {
        vdm_p->vdm_header.std_vdm_hdr.vdmType = CY_PDSTACK_VDM_TYPE_STRUCTURED;
        vdm_p->vdm_header.std_vdm_hdr.objPos = info->obj_pos;
        /* Set object position if custom Attention should be send */
        if ((info->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION) && (info->custom_att_obj_pos))
        {
            vdm_p->vdm_header.std_vdm_hdr.objPos = info->vdm_header.std_vdm_hdr.objPos;
        }
        if (sop_type != CY_PD_SOP)
        {
            vdm_p->vdm_header.std_vdm_hdr.objPos  = info->cbl_obj_pos;
        }
    }

    return true;
}

#if (!UCSI_ALT_MODE_ENABLED)
static void Cy_PdAltMode_Mngr_GetVdmInfoVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t* info, cy_en_pd_sop_t sop_type)
#else
void Cy_PdAltMode_Mngr_GetVdmInfoVdo(uint8_t port, cy_stc_pdaltmode_mngr_info_t* info, sop_t sop_type)
#endif/*UCSI_ALT_MODE_ENABLED*/
{
    cy_stc_pdaltmode_vdm_msg_info_t *vdm_p = ptrAltModeContext->altModeStatus.vdm_info;
    uint8_t         vdo_numb;

#if UVDM_SUPP        
    if (info->vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
#endif /* UVDM_SUPP */        
    {
        /* Copy object position to the VDM Header */
        info->vdm_header.std_vdm_hdr.objPos = vdm_p->vdm_header.std_vdm_hdr.objPos;
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
            CY_PDUTILS_MEM_COPY((uint8_t *)info->vdo[sop_type], (const uint8_t *)vdm_p->vdo,((uint32_t)(vdm_p->vdo_numb) << 2u));
        }
    }
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_AltModeMngrDeinit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t             alt_mode_idx;
    cy_stc_pdaltmode_mngr_info_t     *am_info_p = NULL;
    bool                wait_for_exit = false;

    /* Find and reset alt modes info structures */
    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);
        if (am_info_p != NULL)
        {
            /* If current data role is UFP - set mode to idle */
            if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
                /* CDT 232310 fix */
                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                am_info_p->vdm_header.std_vdm_hdr.cmd = (uint32_t)CY_PDSTACK_VDM_CMD_EXIT_MODE;
            }
            else
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (
                        (ptrAltModeContext->pdStackContext->dpmConfig.contractExist != false) &&
                        (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx) != 0u)
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
                am_info_p->cbk(ptrAltModeContext);
            }

            if (!wait_for_exit)
            {
                /* Reset alt mode info */
                Cy_PdAltMode_Mngr_ResetAltModeInfo(am_info_p);
            }
        }
    }

    if (wait_for_exit)
    {
        return VDM_TASK_ALT_MODE;
    }

    /* Reset alt mode mngr info */
    Cy_PdAltMode_Mngr_ResetInfo(ptrAltModeContext);

#if (CCG_BB_ENABLE != 0)
    Cy_PdAltMode_Billboard_Disable(ptrAltModeContext, true);
    Cy_PdAltMode_Billboard_UpdateAllStatus(ptrAltModeContext, BB_ALT_MODE_STATUS_INIT_VAL);
#endif /* (CCG_BB_ENABLE != 0) */

    return VDM_TASK_EXIT;
}

/******************** Common ALT Mode DFP and UFP functions *******************/

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))

uint8_t Cy_PdAltMode_Mngr_IsSvidSupported(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid)
{
    uint8_t ret_idx = MODE_NOT_SUPPORTED;

#if (NON_TBT_MUX_SUPPORT == 1)    
    if((PD_GET_PTR_TBTHOST_CFG_TBL(port)->non_tbt_mux != 0) && (svid == INTEL_VID))
    {
        return ret_idx;
    }
#endif /* NON_TBT_MUX_SUPPORT*/    

    if (
            (svid != CY_PDALTMODE_NO_DATA)          &&
            (ptrAltModeContext->pdStackContext->port < NO_OF_TYPEC_PORTS) &&
            (Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(ptrAltModeContext, svid) != MODE_NOT_SUPPORTED)
       )
    {
        ret_idx = Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(ptrAltModeContext, ptrAltModeContext->pdStackContext->dpmConfig.curPortType, svid);
    }

    return ret_idx;    
}    

bool Cy_PdAltMode_Mngr_SetCustomSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid)
{
    bool                 ret = false;
#if HPI_AM_SUPP    
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
    uint8_t              cfg_idx;
    cy_stc_pdaltmode_mngr_info_t     *am_info_p;
    cy_stc_pdaltmode_alt_mode_reg_info_t *reg = &ptrAltModeContext->altModeStatus.reg_info;
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */
    volatile uint16_t prev_svid = app_get_status(port)->custom_hpi_svid;

    /* Proceed only if custom SVID has been changed. */
    if (svid != prev_svid)
    {
#if DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE
        /* Check custom alt mode state */
        if (ptrAltModeContext->altModeStatus.state == ALT_MODE_MNGR_STATE_PROCESS)
        {
            cfg_idx   = Cy_PdAltMode_Mngr_GetAltModeNumb(port) - 1;
            am_info_p = ptrAltModeContext->altModeStatus.alt_mode_info[cfg_idx];
            /* Save new custom SVID */
            app_get_status(port)->custom_hpi_svid = svid;

            /* Check if alt mode is active */
            if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, cfg_idx) != false)
            {
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                    am_info_p->cbk(ptrAltModeContext);
                    if (svid != CY_PDALTMODE_NO_DATA)
                    {
                        /* Set flag that reset is required */
                        ptrAltModeContext->altModeStatus.reset_custom_mode = true;
                    }
                }
            }
            else if (svid == CY_PDALTMODE_NO_DATA)
            {
                /* Empty handler to ignore empty SVID */
            }
            /* Check if mode is supported then enter it */
            else if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_supported_modes, cfg_idx) != false)
            {
                if (am_info_p->cbk != NULL)
                {
                    cy_stc_pdaltmode_alt_mode_evt_t tmp_data;

                    /* Send command to change custom SVID */
                    am_info_p->eval_app_cmd(port, tmp_data);
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(ptrAltModeContext);
                    SET_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, cfg_idx);
                }                
            }
            /* If no custom SVID was registered then register it */
            else 
            {
                reg->atch_type = ATCH_TGT;
                am_info_p = gl_reg_alt_mode[Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(port, HPI_AM_SVID)].reg_am_ptr(ptrAltModeContext, reg);
                if ((reg->alt_mode_id != MODE_NOT_SUPPORTED) && (am_info_p != NULL))
                {
                    am_info_p->obj_pos = 1;

                    /* Save pointer to alt mode info struct */
                    ptrAltModeContext->altModeStatus.alt_mode_info[cfg_idx] = am_info_p;

                    /* Set flag that alt mode could be run */
                    SET_FLAG(ptrAltModeContext->altModeStatus.am_supported_modes, cfg_idx);
                    SET_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, cfg_idx);
                }
            }
        }
#endif /* DYNAMIC_CUSTOM_SVID_CHANGE_ENABLE */

        /* Save new custom SVID */
        app_get_status(port)->custom_hpi_svid = svid;
        ret = true;
    }
#endif /* HPI_AM_SUPP */    

    (void) ptrAltModeContext;
    (void) svid;

    return ret;
}

uint16_t Cy_PdAltMode_Mngr_GetCustomSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{    
    uint16_t ret = CY_PDALTMODE_NO_DATA;

#if HPI_AM_SUPP
    custom_alt_cfg_settings_t *ptr = PD_GET_PTR_CUSTOM_ALT_MODE_TBL(port);

    /* Check custom alt mode reg at first */
    if (app_get_status(port)->custom_hpi_svid != CY_PDALTMODE_NO_DATA)
    {
        ret = app_get_status(port)->custom_hpi_svid;
    }  
    else if ((ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP) && (ptr->custom_dfp_mask != CY_PDALTMODE_NO_DATA))
    {
        /* Check config table custom alt mode*/
        ret = ptr->custom_alt_mode;
    }
#else
    (void) ptrAltModeContext;
#endif /* HPI_AM_SUPP */    

    return ret;
}

void Cy_PdAltMode_Mngr_SetAltModeMask(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t mask)
{
#if HPI_AM_SUPP    
    app_get_status(port)->dfp_alt_mode_mask = (uint8_t)(mask >> DFP_ALT_MODE_HPI_OFFSET);
    app_get_status(port)->ufp_alt_mode_mask = (uint8_t)(mask & UFP_ALT_MODE_HPI_MASK);  

    /* Check if we need to disable some alt modes and restart alt mode layer */
    if (ptrAltModeContext->altModeStatus.am_active_modes != (ptrAltModeContext->altModeStatus.am_active_modes & app_get_status(port)->dfp_alt_mode_mask))
    {
        Cy_PdAltMode_Mngr_LayerReset(port);
    }
#else
    (void) ptrAltModeContext;
    (void) mask;
#endif /* HPI_AM_SUPP */    
}

uint16_t Cy_PdAltMode_Mngr_GetSvidFromIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t idx)
{
    uint16_t* am_vdo_ptr;
    uint16_t  svid       = MODE_NOT_SUPPORTED;

    /* Look for the SVID with index position */
    am_vdo_ptr = Cy_PdAltMode_Mngr_GetAltModesVdoInfo(ptrAltModeContext, ptrAltModeContext->pdStackContext->dpmConfig.curPortType, idx);
    if (am_vdo_ptr != NULL)
    {
        svid = am_vdo_ptr[AM_SVID_CONFIG_OFFSET_IDX];
    }
#if HPI_AM_SUPP
    /* Return custom alt mode idx */
    else if ( 
            (app_get_status(port)->custom_hpi_svid != CY_PDALTMODE_NO_DATA) &&
            (idx == (Cy_PdAltMode_Mngr_GetAltModeNumb(port) - 1))
            )
    {
        svid = app_get_status(port)->custom_hpi_svid;
    }
#endif /* HPI_AM_SUPP */    
    return svid;
}

uint16_t* Cy_PdAltMode_Mngr_GetAltModesVdoInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_port_type_t type, uint8_t idx)
{
    cy_stc_pdaltmode_app_status_t* app = &ptrAltModeContext->appStatus;

    uint8_t mask  = (type != CY_PD_PRT_TYPE_UFP) ? app->dfp_alt_mode_mask : app->ufp_alt_mode_mask;

#if LEGACY_CFG_TABLE_SUPPORT
    uint16_t *ptr = (uint16_t *)ptrAltModeContext->altModeCfg;
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
#else
    for (uint8_t i = 0; i < 4u; i++)
    {
        if ((i == idx) && ((mask & (1u << idx)) != 0u))
        {
            return (uint16_t* )ptrAltModeContext->altModeCfg->supported_alt_mode;
        }
    }
#endif /* LEGACY_CFG_TABLE_SUPPORT */

    return NULL;
}

uint8_t Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_port_type_t type, uint16_t svid)
{
    cy_stc_pdaltmode_app_status_t* app = &ptrAltModeContext->appStatus;

#if HPI_AM_SUPP
    if (app_get_status(port)->custom_hpi_svid == svid)
    {
        /* Return last possible index */
        return (Cy_PdAltMode_Mngr_GetAltModeNumb(port) - 1u);
    }
#endif /* HPI_AM_SUPP */

    uint8_t mask  = (type != CY_PD_PRT_TYPE_UFP) ? app->dfp_alt_mode_mask : app->ufp_alt_mode_mask;

#if LEGACY_CFG_TABLE_SUPPORT
    for (uint8_t idx = 0; idx < MAX_SUPP_ALT_MODES; idx++ )
    {
        if((ptrAltModeContext->vdmInfoConfig->discMode[idx].dataObjsVid == svid) && (mask & (1u << idx)))
        {
            return idx;
        }
    }
#else
    for (uint8_t idx = 0; idx < MAX_SUPP_ALT_MODES; idx++ )
    {
        if((ptrAltModeContext->altModeCfg->supported_alt_mode[idx] == svid) && (mask & (1u << idx)))
        {
            return idx;
        }
    }
#endif /* LEGACY_CFG_TABLE_SUPPORT */

    return MODE_NOT_SUPPORTED;
}

static uint8_t Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid)
{
    uint8_t idx, base_am_numb;

    (void)ptrAltModeContext;

    base_am_numb = sizeof(gl_reg_alt_mode)/sizeof(cy_stc_pdaltmode_reg_am_t);

    /* Look through all alt modes */
    for (idx = 0; idx < base_am_numb; idx++) 
    {
        /* if alt mode with given index is supported by CCG */
        if (gl_reg_alt_mode[idx].svid == svid)
        {
            return idx;
        }
    }   
#if HPI_AM_SUPP
    /* Check if custom HPI alt mode is available */
    if (
            (Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(port, HPI_AM_SVID) != MODE_NOT_SUPPORTED) &&
            (app_get_status(port)->custom_hpi_svid == svid)
       )
    {
        return Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(port, HPI_AM_SVID);
    }
#endif /* HPI_AM_SUPP */

    return MODE_NOT_SUPPORTED;
}

#else
void Cy_PdAltMode_Mngr_SetAltModeMask(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t mask)
{
    (void)ptrAltModeContext;
    (void)mask;
}

bool Cy_PdAltMode_Mngr_SetCustomSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid)
{
    (void)ptrAltModeContext;
    (void)svid;

    return false;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

uint8_t Cy_PdAltMode_Mngr_GetAltModeNumb(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t count   = 0;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if (!CCG_BACKUP_FIRMWARE)

#if LEGACY_CFG_TABLE_SUPPORT
   // if(ptrAltModeContext->altModeCfg->table_len != 0)
    {
        uint16_t* ptr   = (uint16_t *)&ptrAltModeContext->altModeCfg; /* PRQA S 0310 */
        uint8_t len     = (uint8_t)(ptr[0] / 2u);
        uint16_t loc_len;

        for (uint8_t i = 2; i < len; i += (uint8_t)loc_len + 1u)
        {
            loc_len = ptr[i] >> 1;
            count++;
        }
    }

#if HPI_AM_SUPP
    if (Cy_PdAltMode_Mngr_GetCustomSvid(port) != CY_PDALTMODE_NO_DATA)
    {
        count++;
    }
#endif /* HPI_AM_SUPP */    
#else
    uint8_t mask  = (ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP) ?
        ptrAltModeContext->altModeCfg->dfp_mask : ptrAltModeContext->altModeCfg->ufp_mask;

    for (uint8_t idx = 0; idx < MAX_SUPP_ALT_MODES; idx++ )
    {
        if((ptrAltModeContext->altModeCfg->supported_alt_mode[idx] != 0) && (mask & (1u << idx)))
        {
            count++;
        }
    }
#endif /* LEGACY_CFG_TABLE_SUPPORT */
#endif /* (!CCG_BACKUP_FIRMWARE) */
#else
    (void)ptrAltModeContext;
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return count;
}

void Cy_PdAltMode_Mngr_ResetAltModeInfo(cy_stc_pdaltmode_mngr_info_t *info)
{
    CY_PDUTILS_MEM_SET((uint8_t *)info, 0u, (uint32_t)sizeof(*info));
}

void Cy_PdAltMode_Mngr_ResetInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    CY_PDUTILS_MEM_SET((uint8_t *)&ptrAltModeContext->altModeStatus, 0u, (uint32_t)sizeof(ptrAltModeContext->altModeStatus));
}

cy_stc_pdstack_dpm_pd_cmd_buf_t* Cy_PdAltMode_Mngr_GetVdmBuff(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &(ptrAltModeContext->altModeStatus.vdm_buf);
}

/******************* ALT MODE Solution Related Functions ****************************/

/* Function to reset alt mode layer */
void Cy_PdAltMode_Mngr_LayerReset(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    ptrAltModeContext->appStatus.skip_mux_config = true;
    Cy_PdAltMode_VdmTask_MngrDeInit (ptrAltModeContext);
    Cy_PdAltMode_VdmTask_Enable (ptrAltModeContext);
    ptrAltModeContext->appStatus.skip_mux_config = false;
#else
    (void)ptrAltModeContext;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
}

#if ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE))

#if DFP_ALT_MODE_SUPP
bool is_enter_allowed(uint8_t port, uint16_t svid)
{
    bool    am_allowed         = false;
    uint8_t am_numb            = ptrAltModeContext->altModeStatus.alt_modes_numb;
    uint8_t idx, idx2;
    uint16_t* am_vdo_ptr       = NULL;

    /* Find out if any alt mode already entered */
    if (ptrAltModeContext->altModeStatus.am_active_modes != CY_PDALTMODE_NO_DATA)
    {
        for (idx = 0; idx < am_numb; idx++)
        {
            /* Find active alt modes */
            if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, idx))
            {
                am_allowed = false;
                /* Get pointer to SVID configuration */
                am_vdo_ptr = Cy_PdAltMode_Mngr_GetAltModesVdoInfo(port, PRT_TYPE_DFP, idx);
                if (am_vdo_ptr != NULL)
                {
                    /* Find out if selected alt mode could be processed simultaneously  with active alt modes */
                    for (idx2 = 0; idx2 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx2++)
                    {
                        /* Check which alternate modes could be run simultaneously */
                        if (am_vdo_ptr[idx2 + AM_SVID_CONFIG_OFFSET_IDX] == svid)
                        {
                            am_allowed = true;
                            break;
                        }
                    }
                    /* If selected alt mode could not run simultaneously with active alt modes return false */
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
void alt_mode_mngr_exit_all(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool send_vdm_exit, pd_cbk_t exit_all_cbk)
{
    uint8_t                  alt_mode_idx;
    cy_stc_pdaltmode_mngr_info_t         *am_info_p = NULL;
    alt_mode_mngr_status_t*  mngr_ptr  = &ptrAltModeContext->altModeStatus;

    if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_DFP)
    {
        /* Configure the MUX to get out of alt modes*/
        if(app_get_status(port)->alt_mode_entered)
        {
            Cy_PdAltMode_HW_SetMux (ptrAltModeContext, MUX_CONFIG_ISOLATE, 0);
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
            am_info_p = Cy_PdAltMode_Mngr_GetModeInfo (ptrAltModeContext, alt_mode_idx);
            if (am_info_p != NULL)
            {

                /* Logically exit all alt modes */
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk (ptrAltModeContext);
                }
            }
        }
    }
}

#endif /* DFP_ALT_MODE_SUPP */

bool Cy_PdAltMode_Mngr_EvalAppAltModeCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *cmd, uint8_t *data)
{
#if (!ICL_ALT_MODE_HPI_DISABLED)
#if (!CCG_BACKUP_FIRMWARE)
    cy_stc_pdaltmode_alt_mode_evt_t  cmd_info, cmd_data;
    cy_stc_pdaltmode_mngr_info_t *am_info_p = NULL;
    uint8_t         alt_mode_idx;
    bool            found = false;

    /* Convert received cmd bytes as info and data */
    cmd_info.val = CY_PDUTILS_MAKE_DWORD(cmd[3], cmd[2], cmd[1], cmd[0]);
    cmd_data.val = CY_PDUTILS_MAKE_DWORD(data[3], data[2], data[1], data[0]);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check if received app command is start discover process for UFP when PD 3.0 supported */
    if (
            (ptrAltModeContext->altModeStatus.pd3_ufp) &&
            (cmd_info.alt_mode_event.data_role == PRT_TYPE_UFP) &&
            (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_RUN_UFP_DISC)
       )
    {
        /* Try to start Discovery process if VDM manager is not busy  */
        return Cy_PdAltMode_VdmTask_IsUfpDiscStarted(port);
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    /* Look for the alternate mode entry which matches the SVID and alt mode id. */
    for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is supported. */
        if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_supported_modes, alt_mode_idx) != 0u)
        {
            am_info_p = Cy_PdAltMode_Mngr_GetModeInfo (ptrAltModeContext, alt_mode_idx);
            if ((am_info_p->vdm_header.std_vdm_hdr.svid == cmd_info.alt_mode_event.svid) &&
                    (am_info_p->alt_mode_id  == cmd_info.alt_mode_event.alt_mode))
            {
                found = true;
                break;
            }
        }
    }

#if DFP_ALT_MODE_SUPP
    if ((ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP) && (cmd_info.alt_mode_event.data_role != CY_PD_PRT_TYPE_UFP))
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
                    SET_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx);
                    REMOVE_FLAG(ptrAltModeContext->altModeStatus.am_exited_modes, alt_mode_idx);

                    /* Initializes alt mode */
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(ptrAltModeContext);

                    /* Goto alt mode processing */
                    ptrAltModeContext->altModeStatus.state = ALT_MODE_MNGR_STATE_PROCESS;
                    return true;
                }
            }
            return false;
        }
    }
#endif /* DFP_ALT_MODE_SUPP */

    if ((found) && (am_info_p->is_active == true) && (am_info_p->mode_state == ALT_MODE_STATE_IDLE) &&
            (cmd_info.alt_mode_event.data_role == ptrAltModeContext->pdStackContext->dpmConfig.curPortType))
    {
        /* If received cmd is specific alt mode command */
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_SPEC)
        {
            return (am_info_p->eval_app_cmd(ptrAltModeContext, cmd_data));
        }
#if DFP_ALT_MODE_SUPP
        else if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_EXIT)
        {
            if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(ptrAltModeContext);
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

const uint32_t* Cy_PdAltMode_Mngr_FormAltModeEvent(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid, uint8_t am_idx, cy_en_pdaltmode_app_evt_t evt, uint32_t data)
{
    cy_stc_pdaltmode_alt_mode_evt_t temp;

    temp.alt_mode_event.svid                           = (uint32_t)svid;
    temp.alt_mode_event.alt_mode                       = (uint32_t)am_idx;
    temp.alt_mode_event.alt_mode_evt                   = (uint32_t)evt;
    temp.alt_mode_event.data_role = ptrAltModeContext->pdStackContext->dpmConfig.curPortType;
    ptrAltModeContext->altModeStatus.app_evt_data[ALT_MODE_EVT_IDX]      = temp.val;
    ptrAltModeContext->altModeStatus.app_evt_data[ALT_MODE_EVT_DATA_IDX] = CY_PDALTMODE_NO_DATA;

    if (data != CY_PDALTMODE_NO_DATA)
    {
        ptrAltModeContext->altModeStatus.app_evt_data[ALT_MODE_EVT_DATA_IDX] = data;
    }

    return ptrAltModeContext->altModeStatus.app_evt_data;
}

cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_Mngr_GetModeInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t alt_mode_idx)
{
    return ptrAltModeContext->altModeStatus.alt_mode_info[alt_mode_idx];
}

#if (!CCG_BACKUP_FIRMWARE)
#if ATTENTION_QUEUE_SUPP
static void Cy_PdAltMode_AttentionCbk(cy_timer_id_t id, void* context);
#endif /* ATTENTION_QUEUE_SUPP */
static void Cy_PdAltMode_Mngr_VdmHandle(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t *am_info, const cy_stc_pdstack_pd_packet_t *vdm)
{
#if ATTENTION_QUEUE_SUPP
    /* Check if it's attention message which comes while alt mode state machine is busy */
    if (
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION) &&
            ( 
             (am_info->mode_state != ALT_MODE_STATE_IDLE)
#if MUX_UPDATE_PAUSE_FSM
             || (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
#endif /* MUX_UPDATE_PAUSE_FSM */
            )
       )
    {
        /* Save attention for the further processing */
        ptrAltModeContext->altModeStatus.att_header = vdm->dat[CY_PD_VDM_HEADER_IDX];
        ptrAltModeContext->altModeStatus.att_vdo = vdm->dat[VDO_START_IDX];
        ptrAltModeContext->altModeStatus.att_alt_mode = am_info;
        Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                ALT_MODE_ATT_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, Cy_PdAltMode_AttentionCbk);
        return;
    }
#endif /* ATTENTION_QUEUE_SUPP */
    /* Save Header */
    am_info->vdm_header.val = vdm->dat[CY_PD_VDM_HEADER_IDX].val;
    am_info->vdo_numb[CY_PD_SOP]  = vdm->len - VDO_START_IDX;

    if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info->vdo_max_numb + VDO_START_IDX)))
    {
        /* Save received VDO */
        CY_PDUTILS_MEM_COPY((uint8_t*)am_info->vdo[CY_PD_SOP], (uint8_t*)&(vdm->dat[VDO_START_IDX]),
                (am_info->vdo_numb[CY_PD_SOP]) * CY_PD_WORD_SIZE);
    }

    /* Run ufp alt mode cbk */
    am_info->mode_state = ALT_MODE_STATE_IDLE;
    if(NULL != am_info->cbk)
    {
        am_info->cbk(ptrAltModeContext);
    }
}
#if ATTENTION_QUEUE_SUPP
static void Cy_PdAltMode_AttentionCbk(cy_timer_id_t id, void* context)
{
    (void)id;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) context;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *) ptrPdStackContext->ptrAltModeContext;

    cy_stc_pdstack_pd_packet_t vdm;
    cy_stc_pdaltmode_alt_mode_mngr_status_t *am = &ptrAltModeContext->altModeStatus;
    
    if ((am->att_alt_mode != NULL) && (am->att_alt_mode->is_active))
    {
        /* Form VDM packet from the saved Attention and try to re-call attention handler in alt mode */
        vdm.dat[CY_PD_VDM_HEADER_IDX] = ptrAltModeContext->altModeStatus.att_header;
        vdm.dat[CY_PD_ID_HEADER_IDX]  = ptrAltModeContext->altModeStatus.att_vdo;
        vdm.len                 = CY_PD_ID_HEADER_IDX + 1;
        /* Run alt mode VDM handler */
        Cy_PdAltMode_Mngr_VdmHandle(ptrAltModeContext, am->att_alt_mode, &vdm);
    }    
}
#endif /* ATTENTION_QUEUE_SUPP */
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )
        
uint32_t get_active_alt_mode_mask(uint8_t port)
{
    return ptrAltModeContext->altModeStatus.am_active_modes;
}

uint32_t get_supp_alt_modes(uint8_t port)
{
    return ptrAltModeContext->altModeStatus.am_supported_modes;
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
static void Cy_PdAltMode_Billboard_Update(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t am_idx, bool bb_stat)
{
    if (Cy_PdAltMode_Billboard_IsPresent(ptrAltModeContext) != false)
    {
        /* Disable AME timeout timer. */
        Cy_PdUtils_SwTimer_Stop(ptrAltModeContext->pdStackContext->ptrTimerContext,
                GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_AME_TIMEOUT_TIMER));
        if (bb_stat == false)
        {
            /* Enable BB controller */
            Cy_PdAltMode_Billboard_Enable(ptrAltModeContext, BB_CAUSE_AME_FAILURE);

            /* Update BB alt mode status register as Error */
            Cy_PdAltMode_Billboard_UpdateAltStatus(ptrAltModeContext, am_idx, BB_ALT_MODE_STAT_UNSUCCESSFUL);
        }
        else
        {
            /* Enable BB controller */
            Cy_PdAltMode_Billboard_Enable(ptrAltModeContext, BB_CAUSE_AME_SUCCESS);

            /* Update BB alt mode status register as successful config */
            Cy_PdAltMode_Billboard_UpdateAltStatus(ptrAltModeContext, am_idx, BB_ALT_MODE_STAT_SUCCESSFUL);
        }
    }
}
#endif /* (CCG_BB_ENABLE != 0) */

static bool Cy_PdAltMode_Mngr_GetModesVdoInfo(cy_stc_pdaltmode_context_t * ptrAltModeContext, uint16_t svid, cy_pd_pd_do_t **temp_p, uint8_t *no_of_vdo)
{
    for (uint8_t i = 0; i < 12u; i++)
    {
        /* If size is less than or equal to 4, return NACK. */
        if (ptrAltModeContext->vdmInfoConfig->discMode[i].disModeLength <= 4u)
        {
            return false;
        }

        if(ptrAltModeContext->vdmInfoConfig->discMode[i].dataObjsVid == svid)
        {
            *no_of_vdo = ((ptrAltModeContext->vdmInfoConfig->discMode[i].disModeLength) >> 2u);
            *temp_p = (cy_pd_pd_do_t *) ptrAltModeContext->vdmInfoConfig->discMode[i].modeDataObj;
            return true;
        }
    }
    return false;
}

static bool Cy_PdAltMode_Mngr_UfpRegAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm)
{
    uint8_t              svid_idx, vdo_numb, alt_mode_idx = 0;
    cy_pd_pd_do_t             *dobj;
    cy_stc_pdaltmode_mngr_info_t     *am_info_p = NULL;
    cy_stc_pdaltmode_alt_mode_reg_info_t *reg       = &(ptrAltModeContext->altModeStatus.reg_info);
    uint32_t             svid      = vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid;
    uint32_t             obj_pos   = vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.objPos;

    /* Check if any alt mode is supported by DFP */
    if (Cy_PdAltMode_Mngr_GetAltModeNumb(ptrAltModeContext) != 0u)
    {
        /* Get index of related svid register function */
        svid_idx = Cy_PdAltMode_Mngr_GetBaseAltModeSvidIdx(ptrAltModeContext, (uint16_t)svid);

        /* If SVID processing supported by CCG */
        if (svid_idx != MODE_NOT_SUPPORTED)
        {
            reg->data_role = CY_PD_PRT_TYPE_UFP;

            /* Get SVID related VDOs and number of VDOs */
            if(Cy_PdAltMode_Mngr_GetModesVdoInfo(ptrAltModeContext, svid, &dobj, &vdo_numb) == false)
            {
                return false;
            }
            /* Check if object position is not greater than VDO number */
            if (obj_pos < vdo_numb)
            {
                /* Save Disc mode VDO and object position */
                reg->svid_vdo = dobj[obj_pos];

                /* Check if UFP support attached target alt mode */
                am_info_p = gl_reg_alt_mode[svid_idx].reg_am_ptr(ptrAltModeContext, reg);

                /* If VDO relates to any of supported alt modes */
                if ((ptrAltModeContext->altModeStatus.reg_info.alt_mode_id != MODE_NOT_SUPPORTED) && (am_info_p != NULL))
                {
                    /* If alternate modes discovered and could be run */
                    /* Get index of alt mode in the compatibility table */
                    alt_mode_idx = Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(ptrAltModeContext, CY_PD_PRT_TYPE_UFP, (uint16_t)svid);
                    if (alt_mode_idx != MODE_NOT_SUPPORTED)
                    {
                        /* Save alt mode ID and obj position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->obj_pos = obj_pos;

                        /* Save pointer to alt mode info struct */
                        ptrAltModeContext->altModeStatus.alt_mode_info[alt_mode_idx] = am_info_p;

                        /* Set flag that alt mode could be run */
                        SET_FLAG(ptrAltModeContext->altModeStatus.am_supported_modes, alt_mode_idx);
                        return true;
                    }
                }
            }
#if (CCG_BB_ENABLE != 0)               
            else if (Cy_PdAltMode_Billboard_IsPresent(ptrAltModeContext) != false)
            {
                /* Go through all alt modes */
                for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
                {
                    am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, alt_mode_idx);
                    /* If Alt mode with corresponded SVID already active then don't notify BB device */
                    if (
                            (am_info_p != NULL) && 
                            (am_info_p->vdm_header.std_vdm_hdr.svid == vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid) &&
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
                    Cy_PdAltMode_Billboard_Update(ptrAltModeContext, svid_idx, false);
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

static bool Cy_PdAltMode_Mngr_UfpEnterAltMode(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_mngr_info_t *am_info_p, const cy_stc_pdstack_pd_packet_t *vdm, uint8_t am_idx)
{
    /* Process VDM */
    Cy_PdAltMode_Mngr_VdmHandle(ptrAltModeContext, am_info_p, vdm);

    if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
    {
        /* Set mode as active */
        am_info_p->is_active = true;

        /* Notify APP layer that ALT mode has been entered. */
        ptrAltModeContext->appStatus.alt_mode_entered = true;

        ptrAltModeContext->pdStackContext->ptrAppCbk->app_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_ALT_MODE,
                Cy_PdAltMode_Mngr_FormAltModeEvent (ptrAltModeContext, am_info_p->vdm_header.std_vdm_hdr.svid,
                    am_info_p->alt_mode_id, AM_EVT_ALT_MODE_ENTERED, CY_PDALTMODE_NO_DATA));

#if (CCG_BB_ENABLE != 0)
        /* Update BB status success */
        Cy_PdAltMode_Billboard_Update(ptrAltModeContext, am_idx, true);
#endif /* (CCG_BB_ENABLE != 0) */

        /* Set flag that alt mode could be processed */
        SET_FLAG(ptrAltModeContext->altModeStatus.am_active_modes, am_idx);
        return true;
    }

#if (CCG_BB_ENABLE != 0)
    /* Update BB status not success */
    Cy_PdAltMode_Billboard_Update(ptrAltModeContext, am_idx, false);
#endif /* (CCG_BB_ENABLE != 0) */
    return false;
}

static bool Cy_PdAltMode_Mngr_IsModeActivated(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm)
{
    uint8_t      idx;

    /* If any alt mode already registered */
    if (ptrAltModeContext->altModeStatus.am_supported_modes != NONE_MODE_MASK)
    {
        for (idx = 0; idx < ptrAltModeContext->altModeStatus.alt_modes_numb; idx++)
        {
            /* Try to find alt mode among supported alt modes */
            if (
                    (ptrAltModeContext->altModeStatus.alt_mode_info[idx] != NULL) &&
                    (Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, idx)->vdm_header.std_vdm_hdr.svid ==
                     vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                    (Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, idx)->obj_pos ==
                     vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.objPos)
               )
            {
                /* return true if ufp alt mode already registered in alt mode mngr */
                return true;
            }
        }
    }

    /* Register alt mode and check possibility of alt mode entering
     * CDT 237168 */
    return Cy_PdAltMode_Mngr_UfpRegAltMode(ptrAltModeContext, vdm);
}

#endif /* UFP_ALT_MODE_SUPP */

#if VPRO_WITH_USB4_MODE
void Cy_PdAltMode_RidgeSlave_VproStatusUpdateEnable(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool vpro_value);
#endif

bool Cy_PdAltMode_Mngr_EvalRecVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm)
{
#if (!CCG_BACKUP_FIRMWARE)
    uint8_t         idx;
#if UFP_ALT_MODE_SUPP
    uint8_t         idx2;
    bool            enter_flag   = false;
#endif /* UFP_ALT_MODE_SUPP */
    cy_stc_pdaltmode_mngr_info_t *am_info_p   = NULL;
    cy_pd_pd_do_t   vdm_header   = vdm->dat[CY_PD_VDM_HEADER_IDX];
    vdm_resp_t*     vdm_response = &(ptrAltModeContext->appStatus.vdmResp);

    if (vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
    {
        /* Discovery commands not processed by alt modes manager */
        if (vdm_header.std_vdm_hdr.cmd < CY_PDSTACK_VDM_CMD_ENTER_MODE)
        {
            return false;
        }
        /* Save number of available alt modes */
        ptrAltModeContext->altModeStatus.alt_modes_numb = Cy_PdAltMode_Mngr_GetAltModeNumb(ptrAltModeContext);

#if UFP_ALT_MODE_SUPP
        /* If Enter mode cmd */
        if (
                (vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ENTER_MODE) &&
                (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
           )
        {
            if (Cy_PdAltMode_Mngr_IsModeActivated(ptrAltModeContext, vdm) == true)
            {
                enter_flag = true;
            }

            /* Send an ACK for ENTER_VPRO mode command if USB4 mode is active */
#if VPRO_WITH_USB4_MODE
            if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
            {
                /**< Index of VDM header data object in a received message. */
                if(
                        (vdm_header.std_vdm_hdr.svid == CY_PDALTMODE_TBT_SVID) &&
                        (vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ENTER_MODE) &&
                        (vdm_header.std_vdm_hdr.objPos == 1) &&
                        /* Enter mode command with Vpro mode */
                        (vdm->dat[CY_PD_ID_HEADER_IDX].tbt_vdo.vproDockHost == 1u) &&
                        (vdm->dat[CY_PD_ID_HEADER_IDX].tbt_vdo.intelMode == 2u)
                  )
                {
#if RIDGE_SLAVE_ENABLE
                    Cy_PdAltMode_RidgeSlave_VproStatusUpdateEnable(ptrAltModeContext, true);
#endif
                    return true;
                }
            }
#endif /* #if VPRO_WITH_USB4_MODE */
        }
#endif /* UFP_ALT_MODE_SUPP */
    }
    /* Go through all alt modes */
    for (idx = 0; idx < ptrAltModeContext->altModeStatus.alt_modes_numb; idx++)
    {
        am_info_p = Cy_PdAltMode_Mngr_GetModeInfo(ptrAltModeContext, idx);

        /* Check if received command processing allowed */
        if (
                (am_info_p != NULL) && (am_info_p->vdm_header.std_vdm_hdr.svid == vdm_header.std_vdm_hdr.svid) &&
                (
                 (am_info_p->obj_pos == vdm_header.std_vdm_hdr.objPos) ||
                 (am_info_p->custom_att_obj_pos == true) || 
                 ((am_info_p->uvdm_supp == true) && 
                  (vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_UNSTRUCTURED))
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
                if (ptrAltModeContext->altModeStatus.am_active_modes == NONE_MODE_MASK)
                {
                    return Cy_PdAltMode_Mngr_UfpEnterAltMode(ptrAltModeContext, am_info_p, vdm, idx);
                }
                else
                {
                    /* Check alt mode in ufp consistent table  */
                    for (idx2 = 0; idx2 < ptrAltModeContext->altModeStatus.alt_modes_numb; idx2++)
                    {
                        /* Find table index of any active alt mode */
                        if ((IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, idx2) != 0u))
                        {
                            uint16_t* am_vdo_ptr = Cy_PdAltMode_Mngr_GetAltModesVdoInfo(ptrAltModeContext, CY_PD_PRT_TYPE_DFP, idx2);
                            if (am_vdo_ptr != NULL)
                            {
                                uint8_t idx3;

                                /* Find out if selected alt mode could be processed simultaneously  with active alt modes */
                                for (idx3 = 0; idx3 < (am_vdo_ptr[AM_SVID_CONFIG_SIZE_IDX] >> 1); idx3++)
                                {
                                    /* Check which alternate modes could be run simultaneously */
                                    if (am_vdo_ptr[idx3 + AM_SVID_CONFIG_OFFSET_IDX] == vdm_header.std_vdm_hdr.svid)
                                    {
                                        break;
                                    }
                                    /* If selected alt mode could not run simultaneously  with active alt modes return false */
                                    return false;
                                }
                            }
                        }             
                    }
                    /* Try to enter alt mode */
                    return Cy_PdAltMode_Mngr_UfpEnterAltMode(ptrAltModeContext, am_info_p, vdm, idx);
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
                          (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP) ||
#endif /* DFP_ALT_MODE_SUPP */
                          (am_info_p->mode_state != ALT_MODE_STATE_WAIT_FOR_RESP)
                         )
#if ATTENTION_QUEUE_SUPP
                         && (vdm_header.std_vdm_hdr.cmd != CY_PDSTACK_VDM_CMD_ATTENTION)
#endif /* ATTENTION_QUEUE_SUPP */
                        )
                   )
                   {
                       return false;
                   }

                /* Save custom attention object position if needed */
                if (
                        (vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION) &&
                        (am_info_p->custom_att_obj_pos)    &&
                        (vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
                   )
                {
                    am_info_p->vdm_header.std_vdm_hdr.objPos = vdm_header.std_vdm_hdr.objPos;
                }
                /* Process VDM */
                Cy_PdAltMode_Mngr_VdmHandle(ptrAltModeContext, am_info_p, vdm);

                /* If command processed successful */
                if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                {
                    /* Copy VDM header to respond buffer if Unstructured*/
                    if (am_info_p->vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_UNSTRUCTURED)
                    {
                        vdm_response->respBuf[CY_PD_VDM_HEADER_IDX] = am_info_p->vdm_header;
                    }
                    /* Set number of data objects */
                    vdm_response->doCount = am_info_p->vdo_numb[CY_PD_SOP] + VDO_START_IDX;
                    if (am_info_p->vdo_numb[CY_PD_SOP] != CY_PDALTMODE_NO_DATA)
                    {
                        /* If VDO resp is needed */
                        CY_PDUTILS_MEM_COPY((uint8_t*) & (vdm_response->respBuf[VDO_START_IDX]),
                                (const uint8_t*) am_info_p->vdo[CY_PD_SOP],
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
        if (
                (vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)   &&
                (vdm_header.std_vdm_hdr.objPos == EXIT_ALL_MODES)                    &&
                (vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE)         &&
                (am_info_p->vdm_header.std_vdm_hdr.svid == vdm_header.std_vdm_hdr.svid)   &&
                (am_info_p->is_active)
           )
        {
            /* Save cmd */
            am_info_p->vdm_header.std_vdm_hdr.cmd = EXIT_ALL_MODES;
            if (am_info_p->cbk != NULL)
            {
                /* Run ufp alt mode cbk */
                am_info_p->cbk(ptrAltModeContext);
            }
            return true;
        }
#endif /* UFP_ALT_MODE_SUPP */
    }
#endif /* (!CCG_BACKUP_FIRMWARE) */
    return false;
}

uint8_t Cy_PdAltMode_Mngr_GetStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t ret = CY_PDALTMODE_NO_DATA;
#if DFP_ALT_MODE_SUPP
    cy_stc_pdaltmode_mngr_info_t *am_info_p = NULL;
    uint8_t          alt_mode_idx;

    if (ptrAltModeContext->altModeStatus.state == ALT_MODE_MNGR_STATE_PROCESS)
    {
        /* Check each alternate mode */
        for (alt_mode_idx = 0; alt_mode_idx < ptrAltModeContext->altModeStatus.alt_modes_numb; alt_mode_idx++)
        {
            if (IS_FLAG_CHECKED(ptrAltModeContext->altModeStatus.am_active_modes, alt_mode_idx))
            {
                am_info_p = Cy_PdAltMode_Mngr_GetModeInfo (ptrAltModeContext, alt_mode_idx);
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
    if (ptrAltModeContext->appStatus.usb4_active != false)
    {
        ret |= APP_USB4_ACTIVE_MASK;
    }
#else
    (void)ptrAltModeContext;
#endif /* DFP_ALT_MODE_SUPP */

    return ret;
}

cy_en_pdstack_usb_data_sig_t Cy_PdAltMode_Mngr_GetCableUsbCap(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_en_pdstack_usb_data_sig_t usbcap = CY_PDSTACK_USB_SIG_UNKNOWN;

#if (!(CCG_CBL_DISC_DISABLE))
    if(ptrPdStackContext->dpmConfig.emcaPresent)
    {
        if ((ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) && (ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo1.vdoVersion == CY_PD_CBL_VDO_VERS_1_2))
        {
            if (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.usb2Supp == (uint8_t)false)
            {
                usbcap = CY_PDSTACK_USB_2_0_SUPP;
            }
            if (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.ssSupp == (uint8_t)false)
            {
                usbcap = (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.usbGen == (uint8_t)false) ? CY_PDSTACK_USB_GEN_1_SUPP : CY_PDSTACK_USB_GEN_2_SUPP;
            }
        }
        else
        {
            /* By default, calculate USB signalling from Passive/Active Cable VDO (#1). */
            usbcap = (cy_en_pdstack_usb_data_sig_t)(ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup);
        }
    }
#endif /* (!(CCG_CBL_DISC_DISABLE)) */

    return (usbcap);
}

/* [] END OF FILE */
