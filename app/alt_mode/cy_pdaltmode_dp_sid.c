/******************************************************************************
* File Name:   dp_sid.c
* \version 2.0
*
* Description: DisplayPort Alternate Mode Source implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdaltmode_defines.h"

#include "cy_pdstack_dpm.h"
#include <cy_pdutils_sw_timer.h>
#include "app_timer_id.h"

#include "cy_usbpd_hpd.h"

#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_dp_sid.h"
#include "cy_pdaltmode_intel_ridge.h"

#if ICL_ENABLE
#include "icl.h"
#endif /* ICL_ENABLE */

#if DP_DFP_SUPP
/* Analyses DP DISC MODE info for DFP */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegDfp(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg);

/* Analyse received DP sink DISC MODE VDO */
static cy_en_pdaltmode_dp_state_t Cy_PdAltMode_DP_AnalyseSinkVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t svid_vdo);

/* Evaluates DP sink Status Update/Attention VDM */
static cy_en_pdaltmode_dp_state_t Cy_PdAltMode_DP_EvalUfpStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Initialize DP DFP alt mode */
static void Cy_PdAltMode_DP_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Checks if cable supports DP handling */
static bool Cy_PdAltMode_DP_IsCableCapable(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);

/* HPD callback function */
static void Cy_PdAltMode_DP_SrcHpdRespCbk(void *context, uint32_t event);

/* Add received DP source HPD status to queue */
static void Cy_PdAltMode_DP_DfpEnqueueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status);

/* Clean HPD queue */
static void Cy_PdAltMode_DP_CleanHpdQueue(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if ICL_ENABLE
/* Dequeue next HPD status */
void Cy_PdAltMode_DP_DfpDequeueHpd(uint8_t port);
#else
static void Cy_PdAltMode_DP_DfpDequeueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /*ICL_ENABLE*/

/* Main DP Source alt mode function */
static void Cy_PdAltMode_DP_DfpRun(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Analyses received status update VDO */
static void Cy_PdAltMode_DP_AnalyseStatusUpdateVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Analyse if DFP and UFP DP modes consistent */
static bool Cy_PdAltMode_DP_IsPrtnrCcgConsistent(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t config);

/* Analyses DP DISC MODE info for the cable */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegCbl(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg);

/* Check if cable contains DP SVID */
static bool Cy_PdAltMode_DP_IsCableSvid(const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* Main DP Sink alt mode function */
static void Cy_PdAltMode_DP_UfpRun(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Updates DP Sink Status */
static bool Cy_PdAltMode_DP_UfpUpdateStatusField (cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_dp_stat_bm_t bit_pos, bool status);

/* Add received DP sink HPD status to queue */
static void Cy_PdAltMode_DP_UfpEnqueueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_usbpd_hpd_events_t status);

/* Dequeue DP Sink HPD status */
static void Cy_PdAltMode_DP_UfpDequeueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Verifies is input configuration valid */
static bool Cy_PdAltMode_DP_IsConfigCorrect(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if CY_PD_CCG5_TO_PMG1S3_FEATURE
/* Vconn Swap initiate callback function */
static void Cy_PdAltMode_DP_VconnSwapInitCbk (cy_timer_id_t id, void *ptrContext);

/* Vconn Swap callback function */
static void Cy_PdAltMode_DP_VconnSwapCbk(cy_stc_pdstack_context_t * ptrPdStackContext, cy_en_pdstack_resp_status_t resp, const cy_stc_pdstack_pd_packet_t* pkt_ptr);
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
#endif /* DP_UFP_SUPP */

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static void Cy_PdAltMode_DP_AddHpdEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_usbpd_hpd_events_t evt);

/* Composes VDM for sending by alt mode manager */
static void Cy_PdAltMode_DP_SendCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Returns pointer to DP alt mode cmd info structure */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_Info(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Evaluates DP sink Configuration command response */
static cy_en_pdaltmode_mux_select_t Cy_PdAltMode_DP_EvalConfig(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Exits DP alt mode */
static void Cy_PdAltMode_DP_Exit(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Initialize HPD functionality */
static void Cy_PdAltMode_DP_InitHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t port_role);

/* Returns pointer to DP VDO buffer */
static cy_pd_pd_do_t* Cy_PdAltMode_DP_GetVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Resets DP variables when exits DP alt mode */
static void Cy_PdAltMode_DP_ResetVar(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Stores given VDO in DP VDO buffer for sending */
static void Cy_PdAltMode_DP_AssignVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t vdo);

#if (!ICL_ALT_MODE_HPI_DISABLED)
/* Evaluates received command from application */
static bool Cy_PdAltMode_DP_EvalAppCmd(void *context, cy_stc_pdaltmode_alt_mode_evt_t cmd_data);
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

#if DP_DFP_SUPP
#if (!ICL_ALT_MODE_HPI_DISABLED)    
static void Cy_PdAltMode_DP_SetAppEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t evtype, uint32_t evdata)
{
    ptrAltModeContext->dpStatus.info.app_evt_needed = true;
    ptrAltModeContext->dpStatus.info.app_evt_data.alt_mode_event_data.evt_type = evtype;
    ptrAltModeContext->dpStatus.info.app_evt_data.alt_mode_event_data.evt_data = evdata;
}
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif

/********************* Alt modes manager register function ********************/

cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegModes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info)
{
#if (DP_DFP_SUPP || CY_PD_USB4_SUPPORT_ENABLE || DP_UFP_SUPP)
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;
#endif /* DP_DFP_SUPP || CY_PD_USB4_SUPPORT_ENABLE  || DP_UFP_SUPP */
    cy_stc_pdaltmode_mngr_info_t* ret_info = NULL;
    reg_info->alt_mode_id = MODE_NOT_SUPPORTED;

#if CY_PD_USB4_SUPPORT_ENABLE
    if(ptrAltModeContext->appStatus.usb4_active != false)
    {
        return ret_info;
    }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    /* Check if DP SVID discover mode received VDO relates to DP alt mode */
    if (reg_info->svid_vdo.std_dp_vdo.rsvd == CY_PDALTMODE_DP_ALT_MODE_ID)
    {
#if DP_UFP_SUPP
        /* If DP sink */
        if (reg_info->data_role == (uint8_t)CY_PD_PRT_TYPE_UFP)
        {
            /* Reset DP info struct */
            Cy_PdAltMode_Mngr_ResetAltModeInfo(&(ptrAltModeContext->dpStatus.info));
            Cy_PdAltMode_DP_ResetVar(ptrAltModeContext);
            reg_info->alt_mode_id = CY_PDALTMODE_DP_ALT_MODE_ID;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->cbk = (alt_mode_cbk_t) Cy_PdAltMode_DP_UfpRun;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdm_header.std_vdm_hdr.svid = CY_PDALTMODE_DP_SVID;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo[CY_PD_SOP] = ptrAltModeContext->dpStatus.vdo;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_max_numb = MAX_DP_VDO_NUMB;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->eval_app_cmd = (alt_mode_app_cbk_t) Cy_PdAltMode_DP_EvalAppCmd;

            /* Set application event */
            reg_info->app_evt = AM_EVT_ALT_MODE_SUPP;

            /* Get supported DP sink pin assignment. */
            if ((reg_info->svid_vdo.std_dp_vdo.recep) != 0u)
            {
                ptrAltModeContext->dpStatus.ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.ufpDPin;
            }
            else
            {
                ptrAltModeContext->dpStatus.ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.dfpDPin;
            }
           return &(ptrAltModeContext->dpStatus.info);
        }
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
        /* If DP source */
        if (reg_info->atch_type == ATCH_TGT)
        {
#if STORE_DETAILS_OF_HOST
            cy_stc_pdaltmode_context_t *dsContext =
                    (cy_stc_pdaltmode_context_t *)ptrAltModeContext->hostDetails.dsAltModeContext;
            if((dsContext->hostDetails.ds_mode_mask  & DP_MODE_DFP) == 0)
            {
                return NULL;
            }
#endif /* STORE_DETAILS_OF_HOST */

            /* Analyze DFP DP Disc mode response */
            ret_info = Cy_PdAltMode_DP_RegDfp(ptrAltModeContext, reg_info);
        }
        else if (reg_info->atch_type == CABLE)
        {
            /* Analyze DP cable Disc mode response */
           ret_info = Cy_PdAltMode_DP_RegCbl(ptrAltModeContext, reg_info);
        }
#endif /* DP_DFP_SUPP */
    }

    CY_UNUSED_PARAMETER(context);
    return ret_info;
}

#if DP_DFP_SUPP
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegDfp(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* Reset DP info struct */
    Cy_PdAltMode_Mngr_ResetAltModeInfo(&ptrAltModeContext->dpStatus.info);
    reg->alt_mode_id = CY_PDALTMODE_DP_ALT_MODE_ID;
    Cy_PdAltMode_DP_ResetVar(ptrAltModeContext);
#if (!ICL_ALT_MODE_HPI_DISABLED)  
    /* Set config ctrl as false at the start */
    ptrAltModeContext->dpStatus.dp_cfg_ctrl = false;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

    /* Check if cable discovered and if active cable has no fixed SS lines */
    if (Cy_PdAltMode_DP_IsCableCapable(ptrAltModeContext, reg->atch_tgt_info) == false)
    {
        /* Set application event */
        reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
        return NULL;
    }
    /* Analyse sink VDO */
    ptrAltModeContext->dpStatus.state = Cy_PdAltMode_DP_AnalyseSinkVdo(ptrAltModeContext, reg->svid_vdo);
    if (ptrAltModeContext->dpStatus.state == DP_STATE_EXIT)
    {
        /* Set application event */
        reg->app_evt = AM_EVT_NOT_SUPP_PARTNER_CAP;
        return NULL;
    }

    /* Check if cable is active and supports PD 3.0 */
    if (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
    {
        /* Check if cable has DP SVID */
        if (Cy_PdAltMode_DP_IsCableSvid(reg->atch_tgt_info) == false)
        {
            /* SOP' disc svid is not needed */
            reg->cbl_sop_flag = CY_PD_SOP_INVALID;

            /* Set application event */
            reg->app_evt = AM_EVT_ALT_MODE_SUPP;
        }
        else
        {
            /* SOP' disc svid  needed */
            reg->cbl_sop_flag = CY_PD_SOP_PRIME;
        }
    }
    /* Copy cable, device/AMA info pointer */
    ptrAltModeContext->dpStatus.tgt_info_ptr       = reg->atch_tgt_info;
    ptrAltModeContext->dpStatus.max_sop_supp       = CY_PD_SOP;

#if CCG_UCSI_ENABLE
#if (!ICL_ALT_MODE_HPI_DISABLED)    
    /* New event to notify the UCSI module */
    reg->app_evt = AM_EVT_SUPP_ALT_MODE_CHNG;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif /*CCG_UCSI_ENABLE*/

#if (!ICL_ALT_MODE_HPI_DISABLED)
    /* Notify application which DP MUX config is supported by both DFP and UFP */
    Cy_PdAltMode_DP_SetAppEvt(ptrAltModeContext, CY_PDALTMODE_DP_ALLOWED_MUX_CONFIG_EVT,
            (ptrAltModeContext->dpStatus.partner_dp_pins_supp & ptrAltModeContext->dpStatus.ccg_dp_pins_supp));
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

    Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_max_numb = MAX_DP_VDO_NUMB;

    /* Prepare DP Enter command */
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state   = ALT_MODE_STATE_INIT;
    Cy_PdAltMode_DP_DfpRun(ptrAltModeContext);

    return &ptrAltModeContext->dpStatus.info;
}

static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegCbl(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    reg->alt_mode_id          = CY_PDALTMODE_DP_ALT_MODE_ID;
    uint8_t cbl_supp_pin_asgn = reg->svid_emca_vdo.std_dp_vdo.dfpDPin |
                reg->svid_emca_vdo.std_dp_vdo.ufpDPin;

    if (reg->cbl_sop_flag != CY_PD_SOP_INVALID)
    {
        if (
                (Cy_PdAltMode_DP_IsPrtnrCcgConsistent(ptrAltModeContext, cbl_supp_pin_asgn) &&
                (cbl_supp_pin_asgn & (CY_PDALTMODE_DP_DFP_D_CONFIG_C | CY_PDALTMODE_DP_DFP_D_CONFIG_D)))
            )
        {
            /* SOP' VDM needed */
            ptrAltModeContext->dpStatus.max_sop_supp               = CY_PD_SOP_PRIME;

            /* Allow send SOP' packets */
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->sop_state[CY_PD_SOP_PRIME] = ALT_MODE_STATE_SEND;

            /* Allow SOP'/SOP" DP handling */
            ptrAltModeContext->dpStatus.dp_act_cbl_supp = true;

            /* Save supported cable pin assignment */
            ptrAltModeContext->dpStatus.cable_config_supp = cbl_supp_pin_asgn;

            /* If SOP'' controller present (active cables only) allow SOP'' VDM */
            if ((ptrAltModeContext->dpStatus.tgt_info_ptr->cblVdo->val != CY_PDALTMODE_NO_DATA) &&
                    (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) &&
                    (ptrAltModeContext->dpStatus.tgt_info_ptr->cblVdo->std_cbl_vdo.sopDp == 1))
            {
                Cy_PdAltMode_DP_Info(ptrAltModeContext)->sop_state[CY_PD_SOP_DPRIME]   = ALT_MODE_STATE_SEND;
                ptrAltModeContext->dpStatus.max_sop_supp                  = CY_PD_SOP_DPRIME;
            }

            /* Set application event */
            reg->app_evt = AM_EVT_ALT_MODE_SUPP;
        }

        reg->cbl_sop_flag = CY_PD_SOP_INVALID;
    }

    return &(ptrAltModeContext->dpStatus.info);
}
#endif /* DP_DFP_SUPP */

/************************* DP Source Functions definitions ********************/

#if DP_DFP_SUPP
 
#if MUX_UPDATE_PAUSE_FSM
static void Cy_PdAltMode_DP_DfpRunCbk(cy_timer_id_t id, void * ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;


    if (
            (ptrAltModeContext->appStatus.mux_stat == MUX_STATE_BUSY) ||
           ((Cy_PdAltMode_DP_Info(port)->mode_state != ALT_MODE_STATE_IDLE) || 
            (Cy_PdAltMode_DP_Info(port)->mode_state != ALT_MODE_STATE_PAUSE))
       )
    {
        /* Run timer if MUX is busy */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                ALT_MODE_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, Cy_PdAltMode_DP_DfpRunCbk);
    }
    else
    {
        Cy_PdAltMode_DP_Info(port)->mode_state = ALT_MODE_STATE_IDLE;

        /* If Status Update or Config command is saved */
        if (ptrAltModeContext->dpStatus.prev_state != DP_STATE_IDLE)
        {
            /* Process saved DP state */
            ptrAltModeContext->dpStatus.state            = ptrAltModeContext->dpStatus.prev_state;
            Cy_PdAltMode_DP_Info(port)->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
            ptrAltModeContext->dpStatus.prev_state       = DP_STATE_IDLE;

            /* Run DP DFP handler */
            Cy_PdAltMode_DP_DfpRun(ptrAltModeContext);
        }
    }
}
#endif /* MUX_UPDATE_PAUSE_FSM */
    
static void Cy_PdAltMode_DP_DfpRun(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdaltmode_state_t dp_mode_state = Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state;

    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_INIT:
            /* Enable DP functionality */
            Cy_PdAltMode_DP_Init(ptrAltModeContext);

            /* Send Enter Mode command */
            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            ptrAltModeContext->dpStatus.state = (cy_en_pdaltmode_dp_state_t) ptrAltModeContext->dpStatus.info.vdm_header.std_vdm_hdr.cmd;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
            switch (ptrAltModeContext->dpStatus.state)
            {
                case DP_STATE_ENTER:
                    /* Init HPD */
                    Cy_PdAltMode_DP_InitHpd(ptrAltModeContext, CY_PD_PRT_TYPE_DFP);

                    /* Go to Status update */
                    ptrAltModeContext->dpStatus.state = DP_STATE_STATUS_UPDATE;
                    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, CY_PDALTMODE_STATUS_UPDATE_VDO);
                    break;

                case DP_STATE_STATUS_UPDATE:
#if MUX_UPDATE_PAUSE_FSM
                    if (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
                    {
                        /* Run timer if MUX is busy */
                        timer_start(port, ALT_MODE_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, Cy_PdAltMode_DP_DfpRunCbk);
                        ptrAltModeContext->dpStatus.prev_state = DP_STATE_STATUS_UPDATE;
                        Cy_PdAltMode_DP_Info(port)->mode_state = ALT_MODE_STATE_PAUSE;
                        return;
                    }
#endif /* MUX_UPDATE_PAUSE_FSM */
                    /* Analyse received VDO */
                    Cy_PdAltMode_DP_AnalyseStatusUpdateVdo(ptrAltModeContext);
                    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
#if (!ICL_ALT_MODE_HPI_DISABLED)
                    if (ptrAltModeContext->dpStatus.dp_cfg_ctrl == CY_PDALTMODE_DP_MUX_CTRL_CMD)
                    {
                        return;
                    }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
                    break;

                case DP_STATE_CONFIG:
#if MUX_UPDATE_PAUSE_FSM
                    if (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
                    {
                        /* Run timer if MUX is busy */
                        timer_start(port, ALT_MODE_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, Cy_PdAltMode_DP_DfpRunCbk);
                        ptrAltModeContext->dpStatus.prev_state = DP_STATE_CONFIG;
                        Cy_PdAltMode_DP_Info(port)->mode_state = ALT_MODE_STATE_PAUSE;
                        return;
                    }
#endif /* MUX_UPDATE_PAUSE_FSM */

#if (!ICL_ALT_MODE_HPI_DISABLED)                    
                    /* If App controls DP MUX Config - send ACK to App */
                    if (ptrAltModeContext->dpStatus.dp_cfg_ctrl == CY_PDALTMODE_DP_MUX_CTRL_CMD)
                    {
                        Cy_PdAltMode_DP_SetAppEvt(ptrAltModeContext, CY_PDALTMODE_DP_STATUS_UPDATE_EVT, CY_PDALTMODE_DP_CFG_CMD_ACK_MASK);
                    }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

                    /* Set MUX */
                    ptrAltModeContext->dpStatus.dp_mux_cfg = Cy_PdAltMode_DP_EvalConfig(ptrAltModeContext);

                    if (ptrAltModeContext->dpStatus.dp_exit == false)
                    {
#if ICL_ENABLE
                        /* Custom_data field is updated for dfp_asgmnt, hpd_state & hpd_irq fields 
                           to avoid BB_Retimer duplicate write*/
                        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, ptrAltModeContext->dpStatus.dp_mux_cfg,
                                (ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfp_asgmnt |
                                 (ptrAltModeContext->dpStatus.status_vdo.dp_stat_vdo.hpd_state << HPD_STATE_BIT_POS) |
                                 (ptrAltModeContext->dpStatus.status_vdo.dp_stat_vdo.hpd_irq << HPD_IRQ_BIT_POS))
                               );

                        /* Clear HPD queue if USB config is set */
                        if (ptrAltModeContext->dpStatus.dp_mux_cfg == MUX_CONFIG_SS_ONLY)
                        {
                            Cy_PdAltMode_DP_CleanHpdQueue(port);
                        }
#else
                        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, ptrAltModeContext->dpStatus.dp_mux_cfg, ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt
#if AMD_RETIMER_ENABLE
                                | ptrAltModeContext->dpStatus.status_vdo.dp_stat_vdo.hpd_state << HPD_STATE_BIT_POS
#endif /* AMD_RETIMER_ENABLE */
                            );
                        /* Handle HPD queue */
                        if (ptrAltModeContext->dpStatus.config_vdo.val == EMPTY_VDO)
                        {
                            Cy_PdAltMode_DP_DfpEnqueueHpd(ptrAltModeContext, EMPTY_VDO);
                        }
                        else
                        {
                            /* Add HPD to queue*/
                            Cy_PdAltMode_DP_DfpEnqueueHpd(ptrAltModeContext, ptrAltModeContext->dpStatus.status_vdo.val);
                        }
#endif /* ICL_ENABLE */       
                    }

                    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
                    
#if AM_PD3_FLOW_CTRL_EN
                    /* Make sure Rp is set back to SinkTxOK here. */
                    Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, false);
#endif /* AM_PD3_FLOW_CTRL_EN */

                    /* CDT 251518 fix - set HPD to LOW when USB config is set */
                    /* Exit DP required */
                    if (ptrAltModeContext->dpStatus.dp_exit != false)
                    {
                        ptrAltModeContext->dpStatus.state = DP_STATE_EXIT;
                        ptrAltModeContext->dpStatus.dp_exit = false;
                        break;
                    }
                    /* Return to avoid send_cmd here. */
                    return;

                case DP_STATE_EXIT:
                    Cy_PdAltMode_DP_Exit(ptrAltModeContext);
                    /* Return to avoid send_cmd here. */
                    return;

                default:
                    /* Return to avoid send_cmd here. */
                    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
                    return;
            }

            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            break;

        case ALT_MODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((cy_en_pdaltmode_dp_state_t)Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdm_header.std_vdm_hdr.cmd == DP_STATE_ATT)
            {   
                /* Analyse received VDO */
                Cy_PdAltMode_DP_AnalyseStatusUpdateVdo(ptrAltModeContext);

                /* Handle HPD status */
                if (
                       (ptrAltModeContext->dpStatus.dp_active_flag != false)            &&
                       (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_stat_vdo.exit == false)    &&
                       (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_stat_vdo.usbCfg == false) &&
                       (ptrAltModeContext->dpStatus.state != DP_STATE_CONFIG)
                    )
                {
                    /* Add HPD to queue*/
                    Cy_PdAltMode_DP_DfpEnqueueHpd(ptrAltModeContext, (*Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)).val);
                }
#if (!ICL_ALT_MODE_HPI_DISABLED)
                if (ptrAltModeContext->dpStatus.dp_cfg_ctrl == CY_PDALTMODE_DP_MUX_CTRL_CMD)
                {
                   return;
                }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
                Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            }
            break;

        case ALT_MODE_STATE_FAIL:
            if ((ptrAltModeContext->dpStatus.state == DP_STATE_ENTER) || (ptrAltModeContext->dpStatus.state == DP_STATE_EXIT))
            {
                Cy_PdAltMode_DP_Exit(ptrAltModeContext);
            }
            else
            {
                /* CDT 311655: Stay in current configuration if UFP NACKs USB configuration */
                if (
                        (ptrAltModeContext->dpStatus.state == DP_STATE_STATUS_UPDATE) ||
                        (
                         (ptrAltModeContext->dpStatus.state == DP_STATE_CONFIG) &&
                         (
                          (ptrAltModeContext->dpStatus.config_vdo.val != CY_PDALTMODE_DP_USB_SS_CONFIG) ||
                          ((cy_en_pdaltmode_fail_status_t)Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->std_vdm_hdr.objPos != NACK)
                         )
                        )
                   )
                {
                    ptrAltModeContext->dpStatus.state = DP_STATE_EXIT;
                    Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
                }
                else
                {
                    ptrAltModeContext->dpStatus.state = DP_STATE_IDLE;
                    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
                }
            }

#if AM_PD3_FLOW_CTRL_EN
            /* Make sure Rp is set back to SinkTxOK here. */
            Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, false);
#endif /* AM_PD3_FLOW_CTRL_EN */
            break;

        case ALT_MODE_STATE_EXIT:
            ptrAltModeContext->dpStatus.state = DP_STATE_CONFIG;
            ptrAltModeContext->dpStatus.dp_exit = true;

            Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, CY_PDALTMODE_DP_USB_SS_CONFIG);
            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            break;

        default:
            break;
    }
}

static void Cy_PdAltMode_DP_AnalyseStatusUpdateVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if (!ICL_ALT_MODE_HPI_DISABLED)
    /* If App layer controls DP MUX Config */
    if (ptrAltModeContext->dpStatus.dp_cfg_ctrl == CY_PDALTMODE_DP_MUX_CTRL_CMD)
    {
        Cy_PdAltMode_DP_SetAppEvt(ptrAltModeContext, CY_PDALTMODE_DP_STATUS_UPDATE_EVT, (*Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)).val);

        /* Set DP to idle and wait for config cmd from App */
        Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
        return;
    }
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    /* CCG handles Status Update in auto mode. */
    ptrAltModeContext->dpStatus.state = Cy_PdAltMode_DP_EvalUfpStatus(ptrAltModeContext);

    if (ptrAltModeContext->dpStatus.state == DP_STATE_CONFIG)
    {
#if AM_PD3_FLOW_CTRL_EN
        /* If we are a PD 3.0 source, change Rp to SinkTxNG to prevent sink from initiating any AMS. */
        Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, true);
#endif /* AM_PD3_FLOW_CTRL_EN */
        /* Clear HPD queue in case of USB SS configuration */
        Cy_PdAltMode_DP_CleanHpdQueue(ptrAltModeContext);
#if (AMD_RETIMER_ENABLE || !VIRTUAL_HPD_ENABLE)
        Cy_PdAltMode_HW_EvalHpdCmd (ptrAltModeContext, CY_USBPD_HPD_EVENT_UNPLUG);
#endif /* AMD_RETIMER_ENABLE || !VIRTUAL_HPD_ENABLE */
        /* Move MUX into SAFE state while going through DP configuration. */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SAFE, CY_PDALTMODE_NO_DATA);

        if (ptrAltModeContext->dpStatus.config_vdo.val != CY_PDALTMODE_DP_USB_SS_CONFIG)
        {
            ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.sign = CY_PDALTMODE_DP_1_3_SIGNALING;
            ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.selConf = CY_PDALTMODE_DP_CONFIG_SELECT;
            if (ptrAltModeContext->dpStatus.dp_act_cbl_supp != false)
            {
                /* Check if DP active cable could provide config pin assignment */
                if ((ptrAltModeContext->dpStatus.cable_config_supp & ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt) == CY_PDALTMODE_NO_DATA)
                {
#if AM_PD3_FLOW_CTRL_EN
                    /* Let Rp change to SinkTxOK. */
                    Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, false);
#endif /* AM_PD3_FLOW_CTRL_EN */
                    ptrAltModeContext->dpStatus.state =  DP_STATE_EXIT;
                    return;
                }
            }
        }

        Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.config_vdo.val);
    }
}

static cy_en_pdaltmode_dp_state_t Cy_PdAltMode_DP_AnalyseSinkVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t svid_vdo)
{
    uint8_t    common_conf = 0;
    cy_en_pdaltmode_dp_state_t ret = DP_STATE_EXIT;
    ptrAltModeContext->dpStatus.dp_4_lane = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_NO_DATA;

#if STORE_DETAILS_OF_HOST
    cy_stc_pdaltmode_context_t *deviceContext =
                                        (cy_stc_pdaltmode_context_t *)ptrAltModeContext->hostDetails.dsAltModeContext;
    uint8_t ds_det_dp_mode = deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl;
#endif

    /* Check the UFP_D-Capable bit. */
    if ((svid_vdo.std_dp_vdo.portCap & 0x01) != 0)
    {
        /* Read DP_MODES_SUPPORTED in DP structure. */
        ptrAltModeContext->dpStatus.ccg_dp_pins_supp = ptrAltModeContext->dpCfg->dp_config_supported;

        /* If DP Interface is presented on a USB TYPE C receptacle,
         * compare with UFP_D Pin Assignments advertised by UFP. */
        if (svid_vdo.std_dp_vdo.recep)
        {
            ptrAltModeContext->dpStatus.partner_dp_pins_supp = svid_vdo.std_dp_vdo.ufpDPin;
        }
        else
        {
            ptrAltModeContext->dpStatus.partner_dp_pins_supp = svid_vdo.std_dp_vdo.dfpDPin;
        }

        common_conf = ptrAltModeContext->dpStatus.partner_dp_pins_supp & ptrAltModeContext->dpStatus.ccg_dp_pins_supp;

        /* There is at least one shared pin configuration between CCG and Port-partner. */
        if ((common_conf & (CY_PDALTMODE_DP_DFP_D_CONFIG_E | CY_PDALTMODE_DP_DFP_D_CONFIG_C | CY_PDALTMODE_DP_DFP_D_CONFIG_D)) != 0)
        {
            /* If one of pin configurations C, E or D is supported by both DFP & UFP, we can go ahead. */
            ret = DP_STATE_ENTER;

            /* Assume that only 4-lane configurations are supported. */
            ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_INVALID_CFG;

            if ((common_conf & CY_PDALTMODE_DP_DFP_D_CONFIG_E) != 0)
            {
                ptrAltModeContext->dpStatus.dp_4_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_E;
            }

            /* Configuration C has higher priority than E. */
            if ((common_conf & CY_PDALTMODE_DP_DFP_D_CONFIG_C) != 0)
            {
                ptrAltModeContext->dpStatus.dp_4_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_C;
            }

            if (
                    ((common_conf & CY_PDALTMODE_DP_DFP_D_CONFIG_D) != 0)
#if STORE_DETAILS_OF_HOST
                    && (ds_det_dp_mode == 1u)
#endif
                )
            {
                ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_D;
            }
            else
            {
                if ((common_conf & CY_PDALTMODE_DP_DFP_D_CONFIG_F) != 0)
                {
                    ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_F;
                }
            }
        }
    }

#if STORE_DETAILS_OF_HOST
    /* See if 2 lane Pin Configuration is supported by both devices.
    * If yes, store the preference. */
    if ((ptrAltModeContext->dpStatus.partner_dp_pins_supp & CY_PDALTMODE_DP_DFP_D_CONFIG_D) && (ds_det_dp_mode == 1u))
    {
        ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_D;
    }
    else if ((Cy_PdAltMode_DP_IsPrtnrCcgConsistent(ptrAltModeContext, CY_PDALTMODE_DP_DFP_D_CONFIG_F)) && (ds_det_dp_mode == 1u))
    {
        ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_DFP_D_CONFIG_F;
    }
    else
    {
        /* Multi Function not possible. */
        ptrAltModeContext->dpStatus.dp_2_lane = CY_PDALTMODE_DP_INVALID_CFG;
    }
#endif

    return ret;
}

static bool Cy_PdAltMode_DP_CheckDp2LaneSupport(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t status)
{
    /*
     * Return true if UFP is requesting multi-function mode and we have a valid 2-lane configuration.
     * Please note that pin configuration F is not valid for a UFP-D.
     */
    return (
            (status.dp_stat_vdo.multFun != false) &&
            (ptrAltModeContext->dpStatus.dp_2_lane != CY_PDALTMODE_DP_INVALID_CFG) &&
            ((status.dp_stat_vdo.dfpUfpConn != DP_CONN_UFP_D) || (ptrAltModeContext->dpStatus.dp_2_lane != CY_PDALTMODE_DP_DFP_D_CONFIG_F))
           );
}

static cy_en_pdaltmode_dp_state_t Cy_PdAltMode_DP_EvalUfpStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdaltmode_dp_state_t ret = DP_STATE_CONFIG;
    bool       dp_mode = false;
    uint32_t   dp_asgn = CY_PDALTMODE_DP_INVALID_CFG;

    cy_pd_pd_do_t status = *Cy_PdAltMode_DP_GetVdo(ptrAltModeContext);

#if ICL_ENABLE
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
    {
        /* Workaround for Dell Type-C dock bug which sends ATTENTION with no Status byte.
         * We just remain in the same state */
        if (Cy_PdAltMode_DP_Info(port)->vdo_numb[SOP] == 0)
            return DP_STATE_IDLE;
    }
#endif /* ICL_ENABLE */

    ptrAltModeContext->dpStatus.status_vdo = status;
    ptrAltModeContext->dpStatus.config_vdo.val = 0;

    /* If UFP sends ATTENTION with no Status byte we just remain in the same state */
    if (Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_numb[CY_PD_SOP] == CY_PDALTMODE_NO_DATA)
        return DP_STATE_IDLE;

    /* Check whether the "USB Configuration Request" or "Exit DisplayPort Mode Request" bits are set. */
    if ((status.val & 0x00000060) != 0)
    {
        /* Ignore re-configuration to USB if MUX is already in USB configuration */
        if ((status.dp_stat_vdo.usbCfg != 0) && (ptrAltModeContext->dpStatus.dp_mux_cfg == MUX_CONFIG_SS_ONLY))
             return DP_STATE_IDLE;
        
        ptrAltModeContext->dpStatus.config_vdo.val = CY_PDALTMODE_DP_USB_SS_CONFIG;

        /* If Exit request bit is set then send Exit cmd */
        if (status.dp_stat_vdo.exit != 0)
        {
            ptrAltModeContext->dpStatus.dp_exit = true;
        }
    }
    else if (ptrAltModeContext->dpStatus.dp_active_flag == false)
    {
        /* Previously in USB configuration. Check if we can switch to DP. */
        if (status.dp_stat_vdo.dfpUfpConn > DP_CONN_DFP_D)
        {
            /* Check if Multi Function requested and CCG can support 2 lane. */
            if (
                    (Cy_PdAltMode_DP_CheckDp2LaneSupport (ptrAltModeContext, status)) ||
                    (ptrAltModeContext->dpStatus.dp_4_lane == CY_PDALTMODE_NO_DATA)
               )
            {
                dp_asgn = ptrAltModeContext->dpStatus.dp_2_lane;
            }
            else
            {
                dp_asgn = ptrAltModeContext->dpStatus.dp_4_lane;
            }

            dp_mode = true;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }
    else
    {
        /* Previously in DP Configuration. */
        /* Check if UFP_D no longer connected. */
        if (status.dp_stat_vdo.dfpUfpConn < DP_CONN_UFP_D)
        {
            /* Move to USB Config. */
            ptrAltModeContext->dpStatus.config_vdo.val = CY_PDALTMODE_DP_USB_SS_CONFIG;
        }
        /* Check if DP sink is requesting for 2 lane and current
         * config is not 2 DP lane. */
        else if (
                    (Cy_PdAltMode_DP_CheckDp2LaneSupport (ptrAltModeContext, status)) &&
                    (ptrAltModeContext->dpStatus.dp_2_lane_active == false)
                )
        {
            /* Move to DP 2 lane. */
            dp_mode = true;
            dp_asgn = ptrAltModeContext->dpStatus.dp_2_lane;
        }
        /* Check if DP Sink is requesting for NON-MULTI Function Mode
         * and current mode is MULTI FUNCTION. In this case switch to
         * 4 lane DP. */
        else if ((ptrAltModeContext->dpStatus.dp_2_lane_active != false) && (status.dp_stat_vdo.multFun == false))
        {
            /* Move to DP 4 lane. */
            dp_mode = true;
            dp_asgn = ptrAltModeContext->dpStatus.dp_4_lane;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }

    if (dp_mode)
    {
        ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt = dp_asgn;
    }

    return ret;
}

static bool Cy_PdAltMode_DP_IsCableCapable(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    if ((atch_tgt_info->cblVdo != NULL) && (atch_tgt_info->cblVdo->val != CY_PDALTMODE_NO_DATA))
    {
        /*
         * If the cable supports DP mode explicitly, we can go ahead with it.
         * Otherwise, the cable has to support USB 3.1 Gen1 or higher data rates.
         */
        if (
               ((Cy_PdAltMode_Mngr_GetCableUsbCap(ptrPdStackContext) == CY_PDSTACK_USB_2_0_SUPP)        ||
                (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)) &&
                (!Cy_PdAltMode_DP_IsCableSvid(atch_tgt_info))
           )
        {
            return false;
        }
    }

    /* Allow DP mode to go through if no cable marker is found. */
    return true;
}

static bool Cy_PdAltMode_DP_IsCableSvid(const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    uint8_t idx = 0;

    while (atch_tgt_info->cbl_svid[idx] != 0)
    {
        /* If Cable Intel SVID found */
        if (atch_tgt_info->cbl_svid[idx] == CY_PDALTMODE_DP_SVID)
        {
            return true;
        }
        idx++;
    }

    return false;
}

static void Cy_PdAltMode_DP_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_DP_Info(ptrAltModeContext);
    
#if (!ICL_ALT_MODE_HPI_DISABLED)
    info->eval_app_cmd = Cy_PdAltMode_DP_EvalAppCmd;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    info->vdo_max_numb    = MAX_DP_VDO_NUMB;
    info->vdo[CY_PD_SOP]        = ptrAltModeContext->dpStatus.vdo;
    info->vdo[CY_PD_SOP_PRIME]  = ptrAltModeContext->dpStatus.cable_vdo;
    info->vdo[CY_PD_SOP_DPRIME] = ptrAltModeContext->dpStatus.cable_vdo;

    /* No need to put MUX into SAFE state at mode entry as this is managed in CONFIG state. */
    info->set_mux_isolate = false;
    info->cbk = (alt_mode_cbk_t)Cy_PdAltMode_DP_DfpRun;

    /* Goto enter state */
    ptrAltModeContext->dpStatus.state = DP_STATE_ENTER;
}

#if MUX_DELAY_EN && ICL_ENABLE
static void Cy_PdAltMode_DP_HpdDelayCbk(uint8_t port, timer_id_t id)
{
    (void)id;
    
    /* Run dequeue just if DP configuration is active */
    if (ptrAltModeContext->dpStatus.dp_active_flag != false)
    {
        if(app_get_status(port)->is_mux_busy)
        {
            timer_start(port, APP_HPD_DELAY_TIMER, PD_GET_PTR_ICL_TGL_CFG_TBL(0)->soc_mux_config_delay, (void *) Cy_PdAltMode_DP_HpdDelayCbk);
        }
        else
        {
            icl_set_evt(port, ICL_EVT_DEQUEUE_HPD);
        }
    }
}
#else
#if !VIRTUAL_HPD_ENABLE
static void Cy_PdAltMode_DP_HpdDelayCbk(cy_timer_id_t id, void *context)
{
    (void)id;

    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *) context;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Run dequeue just if DP configuration is active */
    if ((ptrAltModeContext->dpStatus.dp_active_flag != false) && (Cy_USBPD_Hpdt_IsCommandActive(ptrPdStackContext->ptrUsbPdContext) == true))
    {
        Cy_PdAltMode_DP_DfpDequeueHpd(ptrAltModeContext);
    }
    else
    {
        Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_TIMER_HPD_DELAY_TIMER),
                APP_HPD_DEQUE_POLL_PERIOD, Cy_PdAltMode_DP_HpdDelayCbk);
    }
}
#endif /* VIRTUAL_HPD_ENABLE */
#endif /* MUX_DELAY_EN && ICL_ENABLE */

static void Cy_PdAltMode_DP_CleanHpdQueue(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    ptrAltModeContext->dpStatus.hpd_state = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->dpStatus.queue_read_index = CY_PDALTMODE_NO_DATA;
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_TIMER_HPD_DELAY_TIMER));
}

void Cy_PdAltMode_DP_DfpEnqueueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    uint8_t  prev_queue_idx  = ptrAltModeContext->dpStatus.queue_read_index;
    uint16_t *hpd_state      = &ptrAltModeContext->dpStatus.hpd_state;
    uint8_t  *hpd_idx        = &ptrAltModeContext->dpStatus.queue_read_index;

    /* Check whether IRQ queue is full */
    if (ptrAltModeContext->dpStatus.queue_read_index < CY_PDALTMODE_DP_QUEUE_FULL_INDEX)
    {
        switch(GET_HPD_IRQ_STAT(status))
        {
            case CY_PDALTMODE_HPD_LOW_IRQ_LOW:
                /* Empty queue */
                *hpd_idx = CY_PDALTMODE_DP_QUEUE_STATE_SIZE;
                *hpd_state = (uint16_t)CY_USBPD_HPD_EVENT_UNPLUG;
                prev_queue_idx = CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX;
                break;

            case CY_PDALTMODE_HPD_HIGH_IRQ_LOW:
                /* Add to queue High HPD state */
                Cy_PdAltMode_DP_AddHpdEvt(ptrAltModeContext, CY_USBPD_HPD_EVENT_PLUG);
                break;
            case CY_PDALTMODE_HPD_LOW_IRQ_HIGH:
            case CY_PDALTMODE_HPD_HIGH_IRQ_HIGH:
                /*
                 * Add to queue HPD HIGH if previous HPD level was LOW or it's
                 * a first HPD transaction. In other cases add only HPD IRQ
                 * event to queue.
                 */
                if
                (
                    (prev_queue_idx == CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX) ||
                    ((cy_en_usbpd_hpd_events_t)(CY_PDALTMODE_DP_HPD_STATE_MASK &
                        (*hpd_state >> (*hpd_idx - CY_PDALTMODE_DP_QUEUE_STATE_SIZE))) == CY_USBPD_HPD_EVENT_UNPLUG)
                )
                {
                    Cy_PdAltMode_DP_AddHpdEvt(ptrAltModeContext, CY_USBPD_HPD_EVENT_PLUG);
                }
                Cy_PdAltMode_DP_AddHpdEvt(ptrAltModeContext, CY_USBPD_HPD_EVENT_IRQ);
                break;

            default:
                break;
        }
        /* Allow dequeue only if no HPD Dequeuing in the process */
        if (prev_queue_idx == CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)
        {
#if MUX_DELAY_EN && ICL_ENABLE
            /* Prevent quick back-to-back HPD changes */
            if (app_get_status(port)->is_mux_busy)
            {
                timer_start(port, APP_HPD_DELAY_TIMER, APP_MUX_VDM_DELAY_TIMER_PERIOD, Cy_PdAltMode_DP_HpdDelayCbk);
            }
            else
#endif /* MUX_DELAY_EN && ICL_ENABLE */
            if (Cy_USBPD_Hpdt_IsCommandActive(ptrPdStackContext->ptrUsbPdContext) == true)
            {
                Cy_PdAltMode_DP_DfpDequeueHpd(ptrAltModeContext);
            }
        }
    }
}

void Cy_PdAltMode_DP_DfpDequeueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if !VIRTUAL_HPD_ENABLE
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
#endif /* !VIRTUAL_HPD_ENABLE */
    cy_en_usbpd_hpd_events_t hpd_evt = CY_USBPD_HPD_EVENT_NONE;
    uint16_t hpd_state       = ptrAltModeContext->dpStatus.hpd_state;
    uint8_t  hpd_idx         = ptrAltModeContext->dpStatus.queue_read_index;

#if AMD_RETIMER_ENABLE
    if (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, APP_HPD_DELAY_TIMER, APP_HPD_DEQUE_POLL_PERIOD, (void *) Cy_PdAltMode_DP_HpdDelayCbk);
        return;
    }
#endif /* AMD_RETIMER_ENABLE */
    
#if !VIRTUAL_HPD_ENABLE
    /* If current MUX state is not DP - HPD processing is not allowed */
    if (ptrAltModeContext->dpStatus.dp_active_flag == false)
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_TIMER_HPD_DELAY_TIMER),
                APP_HPD_DEQUE_POLL_PERIOD, Cy_PdAltMode_DP_HpdDelayCbk);
    }
#endif /* !VIRTUAL_HPD_ENABLE */
    
    if (hpd_idx != CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)
    {
        hpd_evt = (cy_en_usbpd_hpd_events_t)(CY_PDALTMODE_DP_HPD_STATE_MASK & (hpd_state >> (hpd_idx - CY_PDALTMODE_DP_QUEUE_STATE_SIZE)));
        Cy_PdAltMode_HW_EvalHpdCmd (ptrAltModeContext, (uint32_t)hpd_evt);
    }
#if !VIRTUAL_HPD_ENABLE
    /* Save current HPD state */
    ptrAltModeContext->dpStatus.saved_hpd_state = hpd_evt;
#endif /* !VIRTUAL_HPD_ENABLE */
}

static void Cy_PdAltMode_DP_SrcHpdRespCbk(void *context, uint32_t event)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;

    if (((event & 0xFFFF) == CY_USBPD_HPD_COMMAND_DONE) && (Cy_PdAltMode_DP_Info(ptrAltModeContext)->is_active == true))
    {
        if (ptrAltModeContext->dpStatus.queue_read_index != CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)
        {
            ptrAltModeContext->dpStatus.queue_read_index -= CY_PDALTMODE_DP_QUEUE_STATE_SIZE;

#if MUX_DELAY_EN && ICL_ENABLE
            /* Prevent quick back-to-back HPD changes */
            if(app_get_status(port)->is_mux_busy)
            {
                Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext, APP_HPD_DELAY_TIMER,
                        PD_GET_PTR_ICL_TGL_CFG_TBL(0)->soc_mux_config_delay, Cy_PdAltMode_DP_HpdDelayCbk);
            }
            else
#endif /* MUX_DELAY_EN && ICL_ENABLE */
            {
                Cy_PdAltMode_DP_DfpDequeueHpd(ptrAltModeContext);
            }
        }
    }
}
#endif /* DP_DFP_SUPP */

/************************* DP Sink Functions definitions **********************/

#if DP_UFP_SUPP

#if DP_GPIO_CONFIG_SELECT

/* Global to hold DP Pin configuration when GPIO based selection is enabled. */
uint8_t gl_dp_sink_config = 0;

void dp_sink_set_pin_config(uint8_t dp_config)
{
    /* Store DP Configuration supported as DP Sink. */
    gl_dp_sink_config = dp_config;
}

uint8_t dp_sink_get_pin_config(void)
{
    return gl_dp_sink_config;
}
#endif /* DP_GPIO_CONFIG_SELECT */

#if GATKEX_CREEK
/*Added a timer callback to add a delay between ridge disconnect and DP alt mode entry*/
void Cy_PdAltMode_Ridge_DpMuxDelayCbk (cy_timer_id_t id, void* ptrContext)
{
    (void)id;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Stop Delay timer */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GR_MUX_DELAY_TIMER);

    /* Set MUX */
    Cy_PdAltMode_HW_SetMux(ptrAltModeContext, ptrAltModeContext->dpStatus.dp_mux_cfg, ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt);
    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, NONE_VDO);
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;

    /* If UFP should respond to VDM then send VDM response */
    if ((ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) && (ptrAltModeContext->appStatus.is_vdm_pending != false))
    {
        ptrAltModeContext->appStatus.vdmResp.doCount = 1u;

        ptrAltModeContext->appStatus.vdm_resp_cbk(ptrPdStackContext, &ptrAltModeContext->appStatus.vdmResp);
        ptrAltModeContext->appStatus.is_vdm_pending = false;
    }
    ptrAltModeContext->appStatus.is_mux_busy = false;
}
#endif /* GATKEX_CREEK */

static void Cy_PdAltMode_DP_UfpRun(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t dp_config;

    /* Get alt modes state and VDM command */
    cy_en_pdaltmode_state_t dp_mode_state = Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state;
    ptrAltModeContext->dpStatus.state = (cy_en_pdaltmode_dp_state_t)Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdm_header.std_vdm_hdr.cmd;

    /* Set idle as default */
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;

    /* If exit all modes cmd received */
    if (Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdm_header.std_vdm_hdr.cmd == EXIT_ALL_MODES)
    {
        ptrAltModeContext->dpStatus.state = DP_STATE_EXIT;
    }
    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_IDLE:

            switch(ptrAltModeContext->dpStatus.state)
            {
                case DP_STATE_ENTER:
                    /* Initialize HPD if we are not using HPD via I2C. */
#if !VIRTUAL_HPD_ENABLE
                    /* Enable HPD receiver */
                    Cy_PdAltMode_DP_InitHpd (ptrAltModeContext, (uint8_t)CY_PD_PRT_TYPE_UFP);
#endif /* !VIRTUAL_HPD_ENABLE */

                    /* Fill Status Update with appropriate bits */
                    ptrAltModeContext->dpStatus.status_vdo.val = EMPTY_VDO;

                    /* Update UFP Enabled status field */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_EN, true);
#if !DP_UFP_DONGLE
                    /* Update UFP connection status */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_UFP_CONN, true);
#endif /* DP_UFP_MONITOR */

                    /* Check if Multi-function allowed */
                    if ((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & (CY_PDALTMODE_DP_DFP_D_CONFIG_D | CY_PDALTMODE_DP_DFP_D_CONFIG_F)) != CY_PDALTMODE_NO_DATA)
                    {
                        (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_MF, true);
                    }
                    
                    /* No VDO is needed in response */
                    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, NONE_VDO);
                    return;

                    /* QAC suppression 2023: Intentional return from function
                     * instead of breaking the switch case clause. */
                case DP_STATE_STATUS_UPDATE: /* PRQA S 2023 */
#if VIRTUAL_HPD_ENABLE
                    /* 
                     * Report HPD low in case of DP not enabled. This is because, HPD block itself may not
                     * be enabled at this stage.
                     */
                    if (ptrAltModeContext->dpStatus.dp_active_flag == false)
                    {
                        /* Not update hpd state as hpd not started yet */
                        Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_HPD, false);
                    }
#else /* !VIRTUAL_HPD_ENABLE */
                    /* Update current HPD status */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_HPD,
                            Cy_PdAltMode_HW_DpSnkGetHpdState(ptrAltModeContext));
#endif /* VIRTUAL_HPD_ENABLE */
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (dp_sink_get_pin_config () == CY_PDALTMODE_DP_DFP_D_CONFIG_C)
                    {
                        Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, Cy_PdAltMode_HW_DpSnkGetHpdState(port));
                    }
#else
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & CY_PDALTMODE_DP_DFP_D_CONFIG_C) != CY_PDALTMODE_NO_DATA) ||
                            ((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & CY_PDALTMODE_DP_DFP_D_CONFIG_D) != CY_PDALTMODE_NO_DATA))
                    {
                        /* Always report UFP connected. */
                        Cy_PdAltMode_DP_UfpUpdateStatusField (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        Cy_PdAltMode_DP_UfpUpdateStatusField (port, DP_STAT_UFP_CONN,
                                Cy_PdAltMode_HW_DpSnkGetHpdState(port));
                    }
#endif /* DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */

                    /* CDT 257059 fix - IRQ bit should be set to zero in Status Update response */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField (ptrAltModeContext, DP_STAT_IRQ, false);

                    /* Respond with current DP sink status */
                    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.status_vdo.val);
                    return;

                    /* QAC suppression 2023: Intentional return from function
                     * instead of breaking the switch case clause. */
                case DP_STATE_CONFIG: /* PRQA S 2023 */
                    /* Check if Config VDO is correct */
                    if (
                            /* QAC suppression 3415: These are read only checks and hence not side effects */
                            /* IF DP configuration */
                            ((Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.selConf == CY_PDALTMODE_DP_CONFIG_SELECT) &&
                             (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.sign == CY_PDALTMODE_DP_1_3_SIGNALING)) || /* PRQA S 3415 */

                             /* If USB configuration requested */
                            (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.selConf == CY_PDALTMODE_USB_CONFIG_SELECT) /* PRQA S 3415 */
                       )
                    {
                        /* Save port partner pin config */
                        ptrAltModeContext->dpStatus.partner_dp_pins_supp = Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.dfpAsgmnt;

#if DP_GPIO_CONFIG_SELECT
                        /* Pin configuration supported is based on GPIO status. */
                        dp_config = dp_sink_get_pin_config ();
#else
                        /* Get DP Pin configuration supported from config table. */
                        dp_config = ptrAltModeContext->dpCfg->dp_config_supported;
#endif /* DP_GPIO_CONFIG_SELECT */

                        /* Check if both UFP and DFP support selected pin configuration */
                        /* QAC suppression 3415: These are read only checks and hence not side effects */
                        if (
                                ((Cy_PdAltMode_DP_IsConfigCorrect(ptrAltModeContext)) &&
                                 ((dp_config & ptrAltModeContext->dpStatus.partner_dp_pins_supp) != 0u)) ||
                                 (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.selConf == CY_PDALTMODE_USB_CONFIG_SELECT) /* PRQA S 3415 */
                           )
                        {
                            /* Save config VDO */
                            ptrAltModeContext->dpStatus.config_vdo = *(Cy_PdAltMode_DP_GetVdo(ptrAltModeContext));

                            /* Get DP MUX configuration */
                            ptrAltModeContext->dpStatus.dp_mux_cfg = Cy_PdAltMode_DP_EvalConfig(ptrAltModeContext);

#if GATKEX_CREEK
                            /*
                             * To set the ridge to disconnect between USB and DP states when device is UFP.
                             * For DP alt mode, GR is updated in DP_CONFIG command instead of ENTER_MODE command.
                             * Before entering DP mode, GR disconnect is done and a 10ms delay is added using a timer.
                             */
                            if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
                            {
                                Cy_PdAltMode_Ridge_SetDisconnect(ptrAltModeContext);
                                ptrAltModeContext->appStatus.is_mux_busy = true;
                                Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                                        GR_MUX_DELAY_TIMER, GR_MUX_VDM_DELAY_TIMER_PERIOD, Cy_PdAltMode_Ridge_DpMuxDelayCbk);
                            }

                            if(ptrAltModeContext->appStatus.is_mux_busy == false)
#endif /* GATKEX_CREEK */
                            {
                                /* Set DP MUX */
                                (void)Cy_PdAltMode_HW_SetMux(ptrAltModeContext, ptrAltModeContext->dpStatus.dp_mux_cfg, ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt);
                                Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, NONE_VDO);
                                Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
                            }
#if CY_PD_CCG5_TO_PMG1S3_FEATURE
                            /* Stop timer to avoid hard reset */
                            Cy_PdUtils_SwTimer_Stop(ptrAltModeContext->pdStackContext->ptrTimerContext,
                                    GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_VCONN_RECOVERY_TIMER));

                            /* Check if USB config received as part of Vconn Swap flow */
                            if ((Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_cfg_vdo.selConf == CY_PDALTMODE_USB_CONFIG_SELECT) && (ptrAltModeContext->dpStatus.vconn_swap_req))
                            {
                                /* Start timer to run Vconn Swap */
                                Cy_PdUtils_SwTimer_Start (ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                                        GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_VCONN_RECOVERY_TIMER),
                                            APP_VDM_BUSY_TIMER_PERIOD, Cy_PdAltMode_DP_VconnSwapInitCbk);
                                return;
                            }
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */

#if VIRTUAL_HPD_ENABLE
                            /* Init HPD - Do deferred HPD initialization when using ridge MUX */
                            Cy_PdAltMode_DP_InitHpd(ptrAltModeContext, CY_PD_PRT_TYPE_UFP);
#endif /* VIRTUAL_HPD_ENABLE */

                            /* If HPD is HIGH then send Attention */
                            if  (
#if (!ICL_ENABLE)
                                    (
#if VIRTUAL_HPD_ENABLE
                                     (Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext)) ||
#endif /* VIRTUAL_HPD_ENABLE */
                                     (Cy_PdAltMode_HW_DpSnkGetHpdState(ptrAltModeContext) != false)
                                    ) &&
#endif /* (!ICL_ENABLE) */
                                    (ptrAltModeContext->dpStatus.queue_read_index == CY_PDALTMODE_NO_DATA)
#if VIRTUAL_HPD_DOCK
                                    && (ptrAltModeContext->ridge.ridge_hpd_state != false)
#endif
                                )
                            {
                                /* If HPD already HIGH then add it to queue */
                                Cy_PdAltMode_DP_AddHpdEvt(ptrAltModeContext, CY_USBPD_HPD_EVENT_PLUG);
                                Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_RUN;
                            }
                            return;
                        }
                    }
                    break;

                case DP_STATE_EXIT:
                    /* Deinit alt mode and reset gl_dp variables */
                    Cy_PdAltMode_DP_Exit(ptrAltModeContext);
                    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, NONE_VDO);
                    return;

                    /* QAC suppression 2023: Intentional return from function
                     * instead of breaking the switch case clause. */
                default: /* PRQA S 2023 */
                    break;
            }
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            /* Analyse Attention callback */
            if (ptrAltModeContext->dpStatus.state == DP_STATE_ATT)
            {
                /* Clear Request and IRQ status bits */
                (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_USB_CNFG, false);
                (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_EXIT, false);
                (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_IRQ, false);

#if VIRTUAL_HPD_ENABLE
                if (
#if (!ICL_ENABLE)
                        (!Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext)) ||
#endif /* (!ICL_ENABLE) */
                        (Cy_PdAltMode_DP_GetVdo(ptrAltModeContext)->dp_stat_vdo.hpdIrq == 1u)
                   )
#endif /* VIRTUAL_HPD_ENABLE */
                {
                    (void)Cy_PdAltMode_HW_EvalHpdCmd (ptrAltModeContext, (uint32_t)CY_USBPD_HPD_COMMAND_DONE);
                }

                /* Check if any HPD event in queue */
                Cy_PdAltMode_DP_UfpDequeueHpd(ptrAltModeContext);
                return;
            }
            return;

            /* QAC suppression 2023: Intentional return from function
             * instead of breaking the switch case clause. */
        case ALT_MODE_STATE_RUN: /* PRQA S 2023 */

            /* Send Attention if HPD is High while DP initialization */
            if (ptrAltModeContext->dpStatus.hpd_state != CY_PDALTMODE_NO_DATA)
            {
                Cy_PdAltMode_DP_UfpDequeueHpd(ptrAltModeContext);
            }
            return;

            /* QAC suppression 2023: Intentional return from function
             * instead of breaking the switch case clause. */
        default: /* PRQA S 2023 */
            break;
    }

    /* Send NACK */
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_FAIL;
}

static bool Cy_PdAltMode_DP_IsConfigCorrect(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    bool retval;

    /* If input configuration is valid then return true */
    switch (ptrAltModeContext->dpStatus.partner_dp_pins_supp)
    {
        case CY_PDALTMODE_DP_DFP_D_CONFIG_C:
        case CY_PDALTMODE_DP_DFP_D_CONFIG_D:
        case CY_PDALTMODE_DP_DFP_D_CONFIG_E:
        case CY_PDALTMODE_DP_DFP_D_CONFIG_F:
            retval = true;
            break;
        default:
            retval = false;
            break;
    }

    return retval;
}

static bool Cy_PdAltMode_DP_UfpUpdateStatusField(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_dp_stat_bm_t bit_pos, bool status)
{
    uint32_t    prev_status;

    /* Save current DP sink status */
    prev_status = ptrAltModeContext->dpStatus.status_vdo.val;

    /* Update status field*/
    if (status != false)
    {
        SET_FLAG(ptrAltModeContext->dpStatus.status_vdo.val, (uint8_t)bit_pos);
    }
    else
    {
        REMOVE_FLAG(ptrAltModeContext->dpStatus.status_vdo.val, (uint8_t)bit_pos);
    }

    /* Check if status changed */
    return (prev_status != ptrAltModeContext->dpStatus.status_vdo.val);
}

void Cy_PdAltMode_DP_SnkHpdRespCbk(void *context, uint32_t cmd)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;
    cy_en_usbpd_hpd_events_t event = (cy_en_usbpd_hpd_events_t)cmd;

    /* Handle hpd only when DP sink was entered */
    if (
            /* QAC suppression 3415: These are read only checks and hence not side effects */
            (event > CY_USBPD_HPD_EVENT_NONE)   &&
            (event < CY_USBPD_HPD_COMMAND_DONE) &&
            (Cy_PdAltMode_DP_Info(ptrAltModeContext)->is_active != false)
       )
    {
        /* If HPD received after Status update command was sent then add to HPD queue */
        Cy_PdAltMode_DP_UfpEnqueueHpd(ptrAltModeContext, event);
    }
}

static void Cy_PdAltMode_DP_UfpEnqueueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_usbpd_hpd_events_t status)
{
    uint16_t *hpd_state       = &ptrAltModeContext->dpStatus.hpd_state;
    uint8_t  *hpd_idx         = &ptrAltModeContext->dpStatus.queue_read_index;

    /* Check if queue is not full */
    if ((*hpd_idx) <= (CY_PDALTMODE_DP_UFP_MAX_QUEUE_SIZE * CY_PDALTMODE_DP_QUEUE_STATE_SIZE))
    {
        switch (status)
        {
            case CY_USBPD_HPD_EVENT_UNPLUG:
                /* Empty queue */
                *hpd_idx = CY_PDALTMODE_DP_QUEUE_STATE_SIZE;
                *hpd_state = (uint16_t) status;
                break;
            case CY_USBPD_HPD_EVENT_PLUG:
            case CY_USBPD_HPD_EVENT_IRQ:
                /* Add to queue High HPD state or IRQ */
                Cy_PdAltMode_DP_AddHpdEvt(ptrAltModeContext, status);
                break;

            /* QAC suppression 2024: Intentional return from function 
             * instead of breaking the switch case clause. */
            default: /* PRQA S 2024 */
                return;
        }

        /* If any Attention already is in sending process then halt dequeue procedure */
        if (Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state == ALT_MODE_STATE_IDLE)
        {
            /* Dequeue HPD events */
            Cy_PdAltMode_DP_UfpDequeueHpd(ptrAltModeContext);
        }
    }
}

static void Cy_PdAltMode_DP_UfpDequeueHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_usbpd_hpd_events_t hpd_evt;
    uint16_t hpd_evt_u16;
    bool    is_att_needed    = false;
    uint16_t hpd_state       = ptrAltModeContext->dpStatus.hpd_state;
    uint8_t  hpd_idx         = ptrAltModeContext->dpStatus.queue_read_index;

    if (hpd_idx != (uint8_t)CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)
    {
        /* Get queued HPD event */
        hpd_evt_u16 = (CY_PDALTMODE_DP_HPD_STATE_MASK & (hpd_state >>
                    (hpd_idx - CY_PDALTMODE_DP_QUEUE_STATE_SIZE)));
        hpd_evt = (cy_en_usbpd_hpd_events_t)hpd_evt_u16;

        /* Decrease queue size */
        hpd_idx -= CY_PDALTMODE_DP_QUEUE_STATE_SIZE;

        /* Get queued HPD event */
        switch(hpd_evt)
        {
            case CY_USBPD_HPD_EVENT_UNPLUG:
                /* Update DP sink status */
                if (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_HPD, false))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == CY_PDALTMODE_DP_DFP_D_CONFIG_E)
                    {
                        Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, false);
                    }
#else
                    if (((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & CY_PDALTMODE_DP_DFP_D_CONFIG_C) != CY_PDALTMODE_NO_DATA) ||
                            ((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & CY_PDALTMODE_DP_DFP_D_CONFIG_D) != CY_PDALTMODE_NO_DATA))
                    {
                        /* Always report UFP connected. */
                        Cy_PdAltMode_DP_UfpUpdateStatusField (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        /* Update UFP connection status */
                        Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, false);
                    }
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* Zero IRQ status if Unattached */
                (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_IRQ, false);
                break;
            case CY_USBPD_HPD_EVENT_PLUG:
                /* Update DP sink status */
                if (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_HPD, true))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == CY_PDALTMODE_DP_DFP_D_CONFIG_E)
                    {
                        Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, true);
                    }
#else
                    /* Update UFP connection status */
                    Cy_PdAltMode_DP_UfpUpdateStatusField(port, DP_STAT_UFP_CONN, true);
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* If next queued event is IRQ we can combine in one Attention */
                if ((((CY_PDALTMODE_DP_HPD_STATE_MASK & (hpd_state >>
                                        (hpd_idx - CY_PDALTMODE_DP_QUEUE_STATE_SIZE)))) == (uint16_t)CY_USBPD_HPD_EVENT_IRQ) &&

                        /* QAC suppression 2995: This function is always called when the events are 
                         * present in queue and hence the hpd_idx is never expected to be DP_QUEUE_EMPTY_INDEX.
                         * Still this check is redundantly kept for protection as it is decrementing the hpd_idx
                         * to ensure that it is never negative. */
                        (hpd_idx != (uint8_t)CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)) /* PRQA S 2995 */
                {
                    /* Decrease queue size */
                    hpd_idx -= CY_PDALTMODE_DP_QUEUE_STATE_SIZE;

                    /* Update IRQ field in status */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_IRQ, true);
                    is_att_needed = true;
                }
                else
                {
                    /* Zero IRQ status */
                    (void)Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_IRQ, false);
                }
                break;
            case CY_USBPD_HPD_EVENT_IRQ:
                /* Update DP sink status */
                if (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_IRQ, true))
                {
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                break;
            default:
                /* No statement */
                break;
        }
        ptrAltModeContext->dpStatus.hpd_state = hpd_state;
        ptrAltModeContext->dpStatus.queue_read_index = hpd_idx;

        /* If status changed then send attention */
        if (is_att_needed != false)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            ptrAltModeContext->dpStatus.state = DP_STATE_ATT;
            Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.status_vdo.val);
            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            return;
        }
        /*
         * If Attention for current event not needed, but there are some events
         * left in queue then run dequeue
         */
        else if (hpd_idx != (uint8_t)CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX)
        {
            /* QAC suppression 3670: The queue is finite sized hence there can be maximum 5 
             * recursive calls, which will not result in a stack overflow */
            Cy_PdAltMode_DP_UfpDequeueHpd(ptrAltModeContext);
        }
        else
        {
            /* No statement */
        }
    }
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
}

#if CY_PD_CCG5_TO_PMG1S3_FEATURE
static void Cy_PdAltMode_DP_DpmCmdFailHdlr (cy_stc_pdstack_context_t * ptrPdStackContext, uint8_t *cnt_ptr)
{
    (*cnt_ptr)++;
    if ((*cnt_ptr) < MAX_RETRY_CNT)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VCONN_RECOVERY_TIMER),
                    APP_VDM_BUSY_TIMER_PERIOD, Cy_PdAltMode_DP_VconnSwapInitCbk);
    }
    else
    {
        (*cnt_ptr) = CY_PDALTMODE_NO_DATA;
        /* Init Hard reset if Vconn Swap failed */
        Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL);
    }
}

static void Cy_PdAltMode_DP_VconnSwapInitCbk (cy_timer_id_t id, void *ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Send Vconn Swap command */
    if(Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_VCONN_SWAP, NULL, true, Cy_PdAltMode_DP_VconnSwapCbk) != CY_PDSTACK_STAT_SUCCESS)
    {
        Cy_PdAltMode_DP_DpmCmdFailHdlr(ptrPdStackContext, &(ptrAltModeContext->dpStatus.vconn_init_retry_cnt));
    }
}

static void Cy_PdAltMode_DP_ConfigAttFailedCbk(cy_timer_id_t id, void *ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;

    /* Init Hard reset if configure command not received */
    Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL);
}

static void Cy_PdAltMode_DP_ConfigAttInitCbk (cy_timer_id_t id, void *ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Send Attention VDM with DP config request */
    ptrAltModeContext->dpStatus.state = DP_STATE_ATT;
    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.status_vdo.val);
    Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);

    /* Start timer to make sure that DFP sent configuration command */
    Cy_PdUtils_SwTimer_Start (ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
            APP_VCONN_RECOVERY_TIMER, APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, Cy_PdAltMode_DP_ConfigAttFailedCbk);
}

static void Cy_PdAltMode_DP_VconnSwapCbk(cy_stc_pdstack_context_t * ptrPdStackContext, cy_en_pdstack_resp_status_t resp, const cy_stc_pdstack_pd_packet_t* pkt_ptr)
{
    (void)pkt_ptr;

    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Stop current Vconn timer */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, APP_VCONN_RECOVERY_TIMER);

    ptrAltModeContext->dpStatus.vconn_init_retry_cnt = CY_PDALTMODE_NO_DATA;

    if (resp == CY_PDSTACK_RES_RCVD)
    {
        ptrAltModeContext->dpStatus.vconn_cbk_retry_cnt = CY_PDALTMODE_NO_DATA;

        /* Start timer to run DP config Attention */
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VCONN_RECOVERY_TIMER),
                    APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, Cy_PdAltMode_DP_ConfigAttInitCbk);
        ptrAltModeContext->dpStatus.vconn_swap_req = false;
    }
    else if (resp < CY_PDSTACK_CMD_SENT)
    {
        Cy_PdAltMode_DP_DpmCmdFailHdlr(ptrPdStackContext, &(ptrAltModeContext->dpStatus.vconn_cbk_retry_cnt));
    }
}

#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
#endif /* DP_UFP_SUPP */

/************************* Common DP Functions definitions ********************/

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
 
#if MUX_UPDATE_PAUSE_FSM
static void dp_send_exit_cmd_cb(uint8_t port, timer_id_t id)
{
    (void)id;

    if (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
    {
        /* Run timer if MUX is busy */
        timer_start(port, ALT_MODE_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, dp_send_exit_cmd_cb);
    }
    else
    {
        /* Set exit state and send Exit VDM */
        ptrAltModeContext->dpStatus.state = DP_STATE_EXIT;
        Cy_PdAltMode_DP_SendCmd(port);
    }
}
#endif /* MUX_UPDATE_PAUSE_FSM */
    
static void Cy_PdAltMode_DP_SendCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_mngr_info_t *info;
    cy_en_pd_sop_t sop_idx = CY_PD_SOP;

    if (ptrAltModeContext->dpStatus.state != DP_STATE_IDLE)
    {
        info = Cy_PdAltMode_DP_Info(ptrAltModeContext);
        
        /* QAC suppression 2982: val is a union object that holds the complete data of this union.
         * It is used here to clear the data of this union. Application uses other members of this 
         * union to specifically access the data required. */
        info->vdm_header.val = CY_PDALTMODE_NO_DATA; /* PRQA S 2982 */
        info->vdm_header.std_vdm_hdr.cmd = (uint32_t)ptrAltModeContext->dpStatus.state;
        info->vdm_header.std_vdm_hdr.svid = CY_PDALTMODE_DP_SVID;

        /* Set USB SS in case of Exit mode */
        if (ptrAltModeContext->dpStatus.state == DP_STATE_EXIT)
        {
#if MUX_UPDATE_PAUSE_FSM
            if (app_get_status(port)->mux_stat == MUX_STATE_BUSY)
            {
                /* Run timer if MUX is busy */
                timer_start(port, ALT_MODE_CBK_TIMER, APP_ALT_MODE_POLL_PERIOD, dp_send_exit_cmd_cb);
            }
            else
#endif /* MUX_UPDATE_PAUSE_FSM */
            {
                (void)Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SS_ONLY, CY_PDALTMODE_NO_DATA);
            }
        }
#if DP_DFP_SUPP
        for (sop_idx = CY_PD_SOP; sop_idx <= ptrAltModeContext->dpStatus.max_sop_supp; sop_idx++)
#endif /* DP_DFP_SUPP */
        {
            info->sop_state[sop_idx] = ALT_MODE_STATE_SEND;

            if ((ptrAltModeContext->dpStatus.state == DP_STATE_ENTER) || (ptrAltModeContext->dpStatus.state == DP_STATE_EXIT))
            {
                /* No parameters for the ENTER_MODE and EXIT_MODE VDMs. */
                info->vdo_numb[sop_idx] = CY_PDALTMODE_NO_DATA;
            }
            else
            {
                /* Status Update, Configure and Attention messages take one parameter Data Object. */
                info->vdo_numb[sop_idx] = MAX_DP_VDO_NUMB;

#if DP_DFP_SUPP
                if ((ptrAltModeContext->dpStatus.state == DP_STATE_STATUS_UPDATE) && (sop_idx > CY_PD_SOP))
                {
                    /* Status Update VDMs are only sent to the Port Partner. */
                    info->sop_state[sop_idx] = ALT_MODE_STATE_IDLE;
                }
#endif /* DP_DFP_SUPP */
            }
        }

        /* Start sending the VDMs out. */
        info->mode_state = ALT_MODE_STATE_SEND;
    }
}

static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_Info(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &ptrAltModeContext->dpStatus.info;
}

static void Cy_PdAltMode_DP_AddHpdEvt(cy_stc_pdaltmode_context_t *ptrAltModeContex, cy_en_usbpd_hpd_events_t evt)
{
    ptrAltModeContex->dpStatus.hpd_state <<= CY_PDALTMODE_DP_QUEUE_STATE_SIZE;
    ptrAltModeContex->dpStatus.hpd_state |= (uint16_t) evt;
    ptrAltModeContex->dpStatus.queue_read_index += CY_PDALTMODE_DP_QUEUE_STATE_SIZE;
}

#if DP_DFP_SUPP
static bool Cy_PdAltMode_DP_IsPrtnrCcgConsistent(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t config)
{
    return (
            /* DP MUX pin */
            (ptrAltModeContext->dpStatus.ccg_dp_pins_supp & ptrAltModeContext->dpStatus.partner_dp_pins_supp & config) ||
            /* USB config */
            (config == CY_PDALTMODE_NO_DATA)
       );
}
#endif /* DP_DFP_SUPP */

static cy_en_pdaltmode_mux_select_t Cy_PdAltMode_DP_EvalConfig(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdaltmode_mux_select_t ret = MUX_CONFIG_SS_ONLY;
    
    switch (ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt)
    {
        case CY_PDALTMODE_DP_DFP_D_CONFIG_C:
        case CY_PDALTMODE_DP_DFP_D_CONFIG_E:
            ptrAltModeContext->dpStatus.dp_active_flag = true;
            ptrAltModeContext->dpStatus.dp_2_lane_active = false;
            ret = MUX_CONFIG_DP_4_LANE;
            break;

        case CY_PDALTMODE_DP_DFP_D_CONFIG_D:
        case CY_PDALTMODE_DP_DFP_D_CONFIG_F:
            ptrAltModeContext->dpStatus.dp_active_flag = true;
            ptrAltModeContext->dpStatus.dp_2_lane_active = true;
            ret = MUX_CONFIG_DP_2_LANE;
            break;

        case CY_PDALTMODE_DP_USB_SS_CONFIG:
            ptrAltModeContext->dpStatus.dp_active_flag = false;
            ptrAltModeContext->dpStatus.dp_2_lane_active = false;
            break;

        default:
            /* No statement */
            break;
    }

    return ret;
}

static void Cy_PdAltMode_DP_ResetVar(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* Zeros DP flags */
    ptrAltModeContext->dpStatus.dp_active_flag = false;
    ptrAltModeContext->dpStatus.dp_2_lane_active = false;
    ptrAltModeContext->dpStatus.config_vdo.val = EMPTY_VDO;
    ptrAltModeContext->dpStatus.status_vdo.val = EMPTY_VDO;
    ptrAltModeContext->dpStatus.hpd_state = (uint16_t)false;
    ptrAltModeContext->dpStatus.queue_read_index = (uint8_t)false;
    ptrAltModeContext->dpStatus.state = DP_STATE_IDLE;
    Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, NONE_VDO);
#if CY_PD_CCG5_TO_PMG1S3_FEATURE
    ptrAltModeContext->dpStatus.vconn_swap_req = false;
    ptrAltModeContext->dpStatus.vconn_init_retry_cnt = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->dpStatus.vconn_cbk_retry_cnt = CY_PDALTMODE_NO_DATA;
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
#if DP_DFP_SUPP
    ptrAltModeContext->dpStatus.dp_exit = false;
    ptrAltModeContext->dpStatus.max_sop_supp = CY_PD_SOP;
    ptrAltModeContext->dpStatus.tgt_info_ptr = NULL;
    ptrAltModeContext->dpStatus.dp_act_cbl_supp = false;
#endif /* DP_DFP_SUPP */
#if MUX_UPDATE_PAUSE_FSM
    ptrAltModeContext->dpStatus.prev_state    = DP_STATE_IDLE;
    Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, ALT_MODE_CBK_TIMER);
#endif /* MUX_UPDATE_PAUSE_FSM */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_TIMER_HPD_DELAY_TIMER));
}

static void Cy_PdAltMode_DP_AssignVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t vdo)
{
    /* No DP VDOs needed while send VDM */
    if (vdo == NONE_VDO)
    {
        ptrAltModeContext->dpStatus.info.vdo_numb[CY_PD_SOP] = 0;
#if DP_DFP_SUPP
        ptrAltModeContext->dpStatus.info.vdo_numb[CY_PD_SOP_PRIME]  = 0;
        ptrAltModeContext->dpStatus.info.vdo_numb[CY_PD_SOP_DPRIME] = 0;
#else
        Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_numb[CY_PD_SOP] = 0;
#endif /* DP_DFP_SUPP */
    }
    else
    {
        /* Include given VDO as part of VDM */
        ptrAltModeContext->dpStatus.vdo[CY_PDALTMODE_DP_VDO_IDX].val = vdo;
        Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_numb[CY_PD_SOP] = MAX_DP_VDO_NUMB;

#if DP_DFP_SUPP
        if (ptrAltModeContext->dpStatus.dp_act_cbl_supp != false)
        {
            ptrAltModeContext->dpStatus.cable_vdo[CY_PDALTMODE_DP_VDO_IDX].val = vdo;
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_numb[CY_PD_SOP_PRIME] = MAX_DP_VDO_NUMB;
            if (ptrAltModeContext->dpStatus.max_sop_supp == CY_PD_SOP_DPRIME)
            {
                Cy_PdAltMode_DP_Info(ptrAltModeContext)->vdo_numb[CY_PD_SOP_DPRIME] = MAX_DP_VDO_NUMB;
            }
        }
#endif /* DP_DFP_SUPP */
    }
}

static cy_pd_pd_do_t* Cy_PdAltMode_DP_GetVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &(ptrAltModeContext->dpStatus.vdo[CY_PDALTMODE_DP_VDO_IDX]);
}

static void Cy_PdAltMode_DP_Exit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    Cy_PdAltMode_DP_ResetVar(ptrAltModeContext);
    (void)Cy_PdAltMode_HW_EvalHpdCmd(ptrAltModeContext, (uint32_t)CY_PDALTMODE_HPD_DISABLE_CMD);
    Cy_PdAltMode_HW_SetCbk(ptrAltModeContext, NULL);
    Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_EXIT;
}

static void Cy_PdAltMode_DP_InitHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t port_role)
{
    (void)Cy_PdAltMode_HW_EvalHpdCmd (ptrAltModeContext, (uint32_t)CY_PDALTMODE_HPD_ENABLE_CMD);
    ptrAltModeContext->dpStatus.hpd_state = 0;
    ptrAltModeContext->dpStatus.queue_read_index = 0;

#if DP_DFP_SUPP
    if (port_role != CY_PD_PRT_TYPE_UFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        Cy_PdAltMode_HW_SetCbk (ptrAltModeContext, Cy_PdAltMode_DP_SrcHpdRespCbk);
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if (port_role == (uint8_t)CY_PD_PRT_TYPE_UFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        Cy_PdAltMode_HW_SetCbk (ptrAltModeContext, Cy_PdAltMode_DP_SnkHpdRespCbk);
    }
#endif /* DP_UFP_SUPP */
}

#if (!ICL_ALT_MODE_HPI_DISABLED)
/****************************HPI command evaluation***************************/

static bool Cy_PdAltMode_DP_EvalAppCmd(void *context, cy_stc_pdaltmode_alt_mode_evt_t cmd_data)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;
#if !ICL_DP_DISABLE_APP_CHANGES
#if ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0))
    uint32_t data = cmd_data.alt_mode_event_data.evt_data;
#endif /* ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0)) */

#if DP_DFP_SUPP
    uint32_t config = CY_PDALTMODE_NO_DATA;
#endif /* DP_DFP_SUPP */
#if DP_UFP_SUPP
    bool is_att_needed = false;
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
    /* MUX configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == CY_PDALTMODE_DP_MUX_CTRL_CMD)
    {
        ptrAltModeContext->dpStatus.dp_cfg_ctrl = (data == CY_PDALTMODE_DP_MUX_CTRL_CMD);
        return true;
    }
    /* DP config command received */
    else if ((cmd_data.alt_mode_event_data.evt_type == CY_PDALTMODE_DP_APP_CFG_CMD) &&
            (ptrAltModeContext->dpStatus.dp_cfg_ctrl != false))
    {
        if (data == CY_PDALTMODE_DP_APP_CFG_USB_IDX)
        {
            ptrAltModeContext->dpStatus.state = DP_STATE_CONFIG;

            ptrAltModeContext->dpStatus.config_vdo.val = CY_PDALTMODE_DP_USB_SS_CONFIG;
            Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.config_vdo.val);
            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
        }
        else if(data <= CY_PDALTMODE_DP_APP_CFG_CMD_MAX_NUMB)
        {
            /* Check if received DP config is possible */
            SET_FLAG(config, data);
            /* Check pin assignment compatibility */
            if ((Cy_PdAltMode_DP_IsPrtnrCcgConsistent(ptrAltModeContext, config)) &&
                    (ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt != config))
            {
                ptrAltModeContext->dpStatus.state = DP_STATE_CONFIG;

                /* Prepare Config cmd and send VDM */
                ptrAltModeContext->dpStatus.config_vdo.val = 0;
                ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.dfpAsgmnt = config;
                ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.sign = CY_PDALTMODE_DP_1_3_SIGNALING;
                ptrAltModeContext->dpStatus.config_vdo.dp_cfg_vdo.selConf = CY_PDALTMODE_DP_CONFIG_SELECT;
                Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.config_vdo.val);
                Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            }
        }
        else
        {
            return false;
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    /* MUX configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == CY_PDALTMODE_DP_SINK_CTRL_CMD)
    {
        /* Check if received command is Exit/USB Request */
        if ((data < (uint32_t)DP_STAT_HPD) && (data > (uint32_t)DP_STAT_MF))
        {
            /* Update DP sink status */
            if (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, (cy_en_pdaltmode_dp_stat_bm_t)data, true))
            {
                /* Set send Attention flag */
                is_att_needed = true;
            }
        }

        /* Check if received command is enabling of Multi-function */
        if (data == (uint32_t)DP_STAT_MF)
        {
            /* Check if MF is supported by CCG */
            if (
                    /* QAC suppression 3415: The second check is not required if the first check is incorrect
                     * and hence it is not a side effect. */
                    ((ptrAltModeContext->dpStatus.ccg_dp_pins_supp & (CY_PDALTMODE_DP_DFP_D_CONFIG_D | CY_PDALTMODE_DP_DFP_D_CONFIG_F)) != 0u) &&

                    /* If status changed from previous */
                    (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_MF, true) != false) /* PRQA S 3415 */
               )
            {
                is_att_needed = true;
            }
        }

        /* Check if received command is disabling of Multi-function */
        if (
                /* QAC suppression 3415: The second check is not required if the first check is incorrect
                 * and hence it is not a side effect. */
                (data == ((uint32_t)DP_STAT_MF - 1u)) &&
                (Cy_PdAltMode_DP_UfpUpdateStatusField(ptrAltModeContext, DP_STAT_MF, false) != false) /* PRQA S 3415 */
           )
        {
            is_att_needed = true;
        }

        /* If status changed then send attention */
        if (is_att_needed != false)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            ptrAltModeContext->dpStatus.state = DP_STATE_ATT;
            Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, ptrAltModeContext->dpStatus.status_vdo.val);
            Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
            return true;
        }
        else
        {
            Cy_PdAltMode_DP_Info(ptrAltModeContext)->mode_state = ALT_MODE_STATE_IDLE;
        }
    }
#if CY_PD_CCG5_TO_PMG1S3_FEATURE
    else if (cmd_data.alt_mode_event_data.evt_type == CY_PDALTMODE_DP_APP_VCONN_SWAP_CFG_CMD)
    {
        /* Init Vconn Swap request */
        ptrAltModeContext->dpStatus.vconn_swap_req = true;
        /* Send Attention VDM with USB request */
        ptrAltModeContext->dpStatus.state = DP_STATE_ATT;
        Cy_PdAltMode_DP_AssignVdo(ptrAltModeContext, CY_PDALTMODE_DP_USB_SS_CONFIG);
        Cy_PdAltMode_DP_SendCmd(ptrAltModeContext);
        /* Start timer to make sure that DFP sent configuration command */
        Cy_PdUtils_SwTimer_Start (ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_VCONN_RECOVERY_TIMER),
                    APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, Cy_PdAltMode_DP_ConfigAttFailedCbk);
        return true;
    }
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
#endif /* DP_UFP_SUPP */
#endif /* ICL_DP_DISABLE_APP_CHANGES */
    return false;
}
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

/* [] END OF FILE */
