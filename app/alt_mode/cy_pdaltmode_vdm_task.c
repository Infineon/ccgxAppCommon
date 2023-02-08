/******************************************************************************
* File Name:   cy_pdaltmode_vdm_task.c
* \version 2.0
*
* Description: VDM Task Manager Source implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdaltmode_defines.h"

#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include <cy_pdutils_sw_timer.h>
#include "app_timer_id.h"

#if (CCG_HPI_ENABLE)
#include "hpi.h"
#endif /* (CCG_HPI_ENABLE) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_vdm_task.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_usb4.h"

#if CY_PD_USB4_SUPPORT_ENABLE
#if RIDGE_SLAVE_ENABLE
#include "cy_pdaltmode_intel_ridge_internal.h"
#include "cy_pdaltmode_intel_vid.h"

#elif AMD_SUPP_ENABLE
#include "amd.h"
#if AMD_RETIMER_ENABLE
#include "amd_retimer.h"
#endif /* AMD_RETIMER_ENABLE */
#endif /* RIDGE_SLAVE_ENABLE */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_intel_ridge.h"
#endif /* STORE_DETAILS_OF_HOST */

#if VDM_RESP_QUERY_SUPPORTED

#define MAX_DISC_ID_RESP_LEN    (CY_PD_MAX_NO_OF_DO)
#define MAX_DISC_SVID_RESP_LEN  (18u)

/* Array to store the D_ID response received from the port partner. */
static uint32_t vdm_disc_id_resp[NO_OF_TYPEC_PORTS][MAX_DISC_ID_RESP_LEN];
static uint8_t  vdm_disc_id_resp_len[NO_OF_TYPEC_PORTS];

/* Array to store the D_SVID response received from the port partner. */
static uint32_t vdm_disc_svid_resp[NO_OF_TYPEC_PORTS][MAX_DISC_SVID_RESP_LEN];
static uint8_t  vdm_disc_svid_resp_len[NO_OF_TYPEC_PORTS];

#endif /* VDM_RESP_QUERY_SUPPORTED */


#if SYS_DEEPSLEEP_ENABLE
/**
 * @brief Check whether the VDM task for the port is idle.
 *
 * @param ptrAltModeContext
 * PD Stack Library Context Pointer.
 *
 * @return None.
 */
bool Cy_PdAltMode_VdmTask_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP))
/**
 * @brief Starts Discover process when CCG is UFP due to PD 3.0 spec.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return true if Discover process has started, false if VDM manager id busy.
 */
bool Cy_PdAltMode_VdmTask_IsUfpDiscStarted(uint8_t port);
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) */

/* Init VDM task mngr */
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
/* Discover ID cmd handler */
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_MngDiscId(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt);

/* Discovery SVID cmd handler */
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_MngDiscSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

/* Sets received VDM response command as failed */
static void Cy_PdAltMode_VdmTask_SetVdmFailed(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_fail_status_t fail_stat);

/* Check VDM retries */
static bool Cy_PdAltMode_VdmTask_RetryCheck(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_fail_status_t fail_stat);

/* Parses received VDM resp to vdm_msg_info_t struct */
static uint8_t Cy_PdAltMode_VdmTask_ParseVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t* rec_vdm);

/* Callback function for sent VDM */
static void Cy_PdAltMode_VdmTask_RecCbk(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t status, const cy_stc_pdstack_pd_packet_t* rec_vdm);

/* Function to send VDM */
static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_SendVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Checks if VDM mngr is enabled */
static bool Cy_PdAltMode_VdmTask_IsVdmMngrEnabled(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Returns pointer to vdm_msg_info_t struct */
static cy_stc_pdaltmode_vdm_msg_info_t* Cy_PdAltMode_VdmTask_GetMsg(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Composes vdm_msg_info_t data to dpm_pd_cmd_buf_t struct */
static uint8_t Cy_PdAltMode_VdmTask_ComposeVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Reset vdm mngr info */
static void Cy_PdAltMode_VdmTask_ResetMngr(cy_stc_pdaltmode_context_t *ptrAltModeContext);

void Cy_PdAltMode_VdmTask_Enable(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* If the VDM Task is already running, do nothing. */
    if (ptrAltModeContext->appStatus.vdm_task_en == (uint8_t)false)
    {
        Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_INIT);
        ptrAltModeContext->appStatus.vdm_task_en = (uint8_t)true;
        ptrAltModeContext->appStatus.vdm_prcs_failed = false;

        /* Make sure all alternate mode manager state has been cleared at start. */
        Cy_PdAltMode_Mngr_ResetInfo (ptrAltModeContext);

#if HPI_AM_SUPP
        /* Check if there any custom alt mode available and save it. */
        app_get_status(port)->custom_hpi_svid = get_custom_svid(port);
#endif /* HPI_AM_SUPP */
    }
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_en_pdaltmode_vdm_task_t ret = VDM_TASK_EXIT;

    /* Reset vdm mngr info */
    Cy_PdAltMode_VdmTask_ResetMngr(ptrAltModeContext);
    Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Set alt mode trigger based on config. */
    ptrAltModeContext->appStatus.alt_mode_trig_mask = ptrAltModeContext->dpCfg->dp_mode_trigger;
    ptrAltModeContext->appStatus.dfp_alt_mode_mask = ptrAltModeContext->altModeCfg->dfp_alt_mode_mask;
    ptrAltModeContext->appStatus.ufp_alt_mode_mask = ptrAltModeContext->altModeCfg->ufp_alt_mode_mask;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

    ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_IDLE;
    ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_INACTIVE;
    ptrAltModeContext->hwDetails.mux_cur_state = MUX_CONFIG_ISOLATE;

    if (Cy_PdAltMode_Mngr_GetAltModeNumb(ptrAltModeContext) != (uint8_t)false)
    {
        /* Check if current data role is DFP */
        if(ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
        {
            ret = VDM_TASK_DISC_ID;
        }
        else
        {
            ret = VDM_TASK_REG_ATCH_TGT_INFO;
        }
    }

    return ret;

}

bool Cy_PdAltMode_VdmTask_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /*
       If VDM manager is enabled, check
       1. Whether BUSY timer is running.
       2. Whether the ALT mode tasks are idle.
     */
    return !(Cy_PdAltMode_VdmTask_IsVdmMngrEnabled (ptrAltModeContext) &&
            ((Cy_PdUtils_SwTimer_IsRunning (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER))) ||
             (Cy_PdAltMode_VdmTask_GetTask(ptrAltModeContext) != VDM_TASK_ALT_MODE) ||
             (Cy_PdAltMode_Mngr_IsAltModeMngrIdle(ptrAltModeContext) == false))
            );
}

void Cy_PdAltMode_VdmTask_Manager(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* The VDM task enabled check is performed by the caller. */
    if (
            (Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext,
                    GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER)) == true)
#if MUX_DELAY_EN
              || (ptrAltModeContext->appStatus.is_mux_busy != false)
#endif /* MUX_DELAY_EN */
#if CY_PD_USB4_SUPPORT_ENABLE
            || (ptrAltModeContext->vdmStat.usb4_flag == USB4_PENDING)
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
       )
    {
        return;
    }

    /* Check if cable reset is required */
    if (ptrAltModeContext->appStatus.trig_cbl_rst != false)
    {
        if(Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_CABLE_RESET, NULL, false, NULL) == CY_PDSTACK_STAT_SUCCESS)
        {
            Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                    GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), APP_CABLE_VDM_START_DELAY, NULL);
            ptrAltModeContext->appStatus.trig_cbl_rst = false;
        }
        return;
    }

    /* Get current vdm task */
    switch (Cy_PdAltMode_VdmTask_GetTask(ptrAltModeContext))
    {
        case VDM_TASK_WAIT:
            break;

        case VDM_TASK_INIT:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_VdmTask_Init(ptrAltModeContext));
            break;

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        case VDM_TASK_DISC_ID:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_VdmTask_MngDiscId(ptrAltModeContext, Cy_PdAltMode_VdmTask_GetEvt(ptrAltModeContext)));
            break;

        case VDM_TASK_DISC_SVID:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_VdmTask_MngDiscSvid(ptrAltModeContext, Cy_PdAltMode_VdmTask_GetEvt(ptrAltModeContext)));
            break;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

        case VDM_TASK_SEND_MSG:
            Cy_PdAltMode_VdmTask_ComposeVdm(ptrAltModeContext);
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_VdmTask_SendVdm(ptrAltModeContext));
            break;

#if CY_PD_USB4_SUPPORT_ENABLE
        case VDM_TASK_USB4_TBT:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_Usb4_TbtHandler(ptrAltModeContext, Cy_PdAltMode_VdmTask_GetEvt(ptrAltModeContext)));
            break;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

        case VDM_TASK_REG_ATCH_TGT_INFO:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_Mngr_RegAltModeMngr(ptrAltModeContext, &ptrAltModeContext->vdmStat.atch_tgt, Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext)));
            Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
            break;

        case VDM_TASK_ALT_MODE:
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, Cy_PdAltMode_Mngr_AltModeProcess(ptrAltModeContext, Cy_PdAltMode_VdmTask_GetEvt(ptrAltModeContext)));
            if ((Cy_PdAltMode_VdmTask_GetTask(ptrAltModeContext) == VDM_TASK_SEND_MSG) || (Cy_PdAltMode_VdmTask_GetTask(ptrAltModeContext) == VDM_TASK_ALT_MODE))
            {
                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
            }
            break;

        case VDM_TASK_EXIT:
            Cy_PdAltMode_VdmTask_MngrDeInit(ptrAltModeContext);
            ptrAltModeContext->appStatus.vdm_prcs_failed = true;
            break;

        default:
            /* No statement */
            break;
    }
}

static bool Cy_PdAltMode_VdmTask_RetryCheck(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_fail_status_t fail_stat)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    (ptrAltModeContext->vdmStat.rec_retry_cnt)++;

    /* If we don't receive response more than rec_retry_cnt notify that fail */
    if (ptrAltModeContext->vdmStat.rec_retry_cnt > MAX_RETRY_CNT)
    {
        Cy_PdAltMode_VdmTask_SetVdmFailed(ptrAltModeContext, fail_stat);
        return false;
    }

    /* Try to send msg again */
    Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_SEND_MSG);

    if (fail_stat == BUSY)
    {
        /* Start a BUSY timer to delay the retry attempt if port partner is busy. */
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), APP_VDM_FAIL_RETRY_PERIOD, NULL);
    }
    else
    {
        /* Fix for Ellisys VDMD tests: Delay the retry by 10 ms. */
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), 10u, NULL);
    }

    return true;
}

static void Cy_PdAltMode_VdmTask_SetVdmFailed(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_fail_status_t fail_stat)
{
    /* Reset retry counters */
    ptrAltModeContext->vdmStat.rec_retry_cnt = 0;
    Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_FAIL);

    /* Set failed and save failed command */
    Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext)->vdo->std_vdm_hdr.cmd = Cy_PdAltMode_Mngr_GetVdmBuff(ptrAltModeContext)->cmdDo[0].std_vdm_hdr.cmd;

    /* Save Failure code in the object position field */
    Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext)->vdm_header.std_vdm_hdr.objPos = (uint32_t)fail_stat;
}

/* Received VDM callback */
static void Cy_PdAltMode_VdmTask_RecCbk(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t status, const cy_stc_pdstack_pd_packet_t* rec_vdm)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    uint32_t response;
    bool     run_task_flag = false;

    /* If response ACK */
    if (status == CY_PDSTACK_RES_RCVD)
    {
        /* Check whether received message is a VDM and treat as NAK otherwise. */
        if ((rec_vdm->msg != CY_PDSTACK_DATA_MSG_VDM) || (rec_vdm->len == 0))
        {
            /* Notify manager with failed event. */
            Cy_PdAltMode_VdmTask_SetVdmFailed(ptrAltModeContext, NACK);
            run_task_flag = true;
        }
        else
        {
            /* Handle Standard VDM */
            if (rec_vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
            {
                response = rec_vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType;

#if ((CCG_PD_REV3_ENABLE) && (APP_PD_REV3_ENABLE))
                /* Update VDM version based on the value received from UFP. */
                if (rec_vdm->sop == SOP)
                {
                    app_get_status(port)->vdm_version =
                        GET_MIN (app_get_status(port)->vdm_version, rec_vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.st_ver);
                }
#endif /* ((CCG_PD_REV3_ENABLE) && (APP_PD_REV3_ENABLE)) */

                /* Actual response received. */
                switch (response)
                {
                    case CY_PDSTACK_CMD_TYPE_RESP_ACK:
                        {
#if CY_PD_USB4_SUPPORT_ENABLE
                            if (
                                   (ptrAltModeContext->appStatus.usb4_active == false)   &&
                                   (ptrAltModeContext->vdmStat.usb4_flag > USB4_TBT_CBL_FIND) &&
                                   ((rec_vdm->sop == CY_PD_SOP_PRIME) || (rec_vdm->sop == CY_PD_SOP_DPRIME))
                                )
                            {
                                Cy_PdAltMode_VdmTask_ParseVdm(ptrAltModeContext, rec_vdm);
                                Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_USB4_TBT);
                                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_EVAL);
                                return;
                            }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
                            /* Check if received response is expected. */
                            if ((rec_vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd) ==
                                    Cy_PdAltMode_Mngr_GetVdmBuff(ptrAltModeContext)->cmdDo[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd)
                            {
                                Cy_PdAltMode_VdmTask_ParseVdm(ptrAltModeContext, rec_vdm);
                                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_EVAL);

                                /* Reset timer counter when ACK. */
                                ptrAltModeContext->vdmStat.rec_retry_cnt = 0;

                                /* Continue the state machine operation. */
                                run_task_flag = true;
                            }
                            else if (Cy_PdAltMode_VdmTask_RetryCheck (ptrAltModeContext, BUSY) == false)
                                /* Check for retries. If failure persists after all retries, go to exit. */
                            {
                                run_task_flag = true;
                            }
                            else
                            {
                                /* No statement */
                            }
                        }
                        break;

                    case CY_PDSTACK_CMD_TYPE_INITIATOR:
                    case CY_PDSTACK_CMD_TYPE_RESP_BUSY:
                        /* Target is BUSY. */
                        {
                            /* Check for retries. If failure persists after all retries, go to exit. */
                            if (Cy_PdAltMode_VdmTask_RetryCheck (ptrAltModeContext, BUSY) == false)
                            {
                                run_task_flag = true;
                            }
                        }
                        break;

                    default:
                        /* Target NAK-ed the message. */
                        {
                            /* Notify manager with failed event */
                            Cy_PdAltMode_VdmTask_SetVdmFailed(ptrAltModeContext, NACK);
                            run_task_flag = true;
                        }
                        break;
                }
            }
            /* Handle UVDM */
            else
            {
                Cy_PdAltMode_VdmTask_ParseVdm(ptrAltModeContext, rec_vdm);
                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_EVAL);
                /* Reset timer counter when ACK */
                ptrAltModeContext->vdmStat.rec_retry_cnt = 0;
                run_task_flag = true;
            }
        }
    }
    /* Attention related handler */
    else if (Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext)->vdm_header.std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION)
    {
        /* This statement need to notify alt mode that Attention VDM was successfully sent */
        if ((status == CY_PDSTACK_CMD_SENT) && (Cy_PdAltMode_VdmTask_GetTask(ptrAltModeContext) == VDM_TASK_WAIT))
        {
            /* Start a timer. Command will be retried when timer expires. */
            Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER));
            Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_EVAL);

            /* Reset retry counter */
            ptrAltModeContext->vdmStat.rec_retry_cnt = 0;
            /* Continue the state machine operation. */
            run_task_flag = true;
        }
        else if (status == CY_PDSTACK_SEQ_ABORTED)
        {
            /* Try to send msg again */
            run_task_flag = true;
        }
        else
        {
            /* No statement */
        }
    }
    else
    {
        /* Good CRC not received or no response (maybe corrupted packet). */
        if ((status == CY_PDSTACK_CMD_FAILED) || (status == CY_PDSTACK_RES_TIMEOUT))
        {
            /* Check for retries. If failure persists after all retries, go to exit. */
            if (Cy_PdAltMode_VdmTask_RetryCheck (ptrAltModeContext, (cy_en_pdaltmode_fail_status_t)status) == false)
            {
                run_task_flag = true;
            }
        }
        else
        {
            if (status == CY_PDSTACK_SEQ_ABORTED)
            {
                /* Try to send msg again */
                Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_SEND_MSG);
            }
        }
    }

    /* Check if we need run any task */
    if (run_task_flag == true)
    {
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if CY_PD_USB4_SUPPORT_ENABLE
        if (
                (ptrAltModeContext->appStatus.usb4_active == false) &&
                (ptrAltModeContext->vdmStat.usb4_flag > USB4_TBT_CBL_FIND) &&
                ((rec_vdm->sop == CY_PD_SOP_PRIME) || (rec_vdm->sop == CY_PD_SOP_DPRIME))
           )
        {
            /* Set USB4 Task */
            Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_USB4_TBT);
            Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_FAIL);
            return;
        }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

        switch (ptrAltModeContext->vdmStat.vdm_msg.vdm_header.std_vdm_hdr.cmd)
        {
            case CY_PDSTACK_VDM_CMD_DSC_IDENTITY:
                Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_DISC_ID);
                break;
            case CY_PDSTACK_VDM_CMD_DSC_SVIDS:
                Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_DISC_SVID);
                break;
            default:
                Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_ALT_MODE);
                break;
        }
#else /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
        Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_ALT_MODE);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    }
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

void Cy_PdAltMode_VdmTask_SetDiscParam(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t sop, cy_en_pdstack_std_vdm_cmd_t cmd)
{
    cy_stc_pdaltmode_vdm_msg_info_t *msg_p = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);

    /* Form Discover ID VDM packet. */
    msg_p->vdm_header.val                       = CY_PDALTMODE_NO_DATA;
    msg_p->vdm_header.std_vdm_hdr.svid             = CY_PDATMODE_STD_SVID;
    msg_p->vdm_header.std_vdm_hdr.cmd              = cmd;
    msg_p->vdo_numb                             = CY_PDALTMODE_NO_DATA;
    msg_p->sopType                                 = sop;
    msg_p->vdm_header.std_vdm_hdr.vdmType         = CY_PDSTACK_VDM_TYPE_STRUCTURED;

}

#if (( RIDGE_SLAVE_ENABLE) || (AMD_SUPP_ENABLE))
static void Cy_PdAltMode_VdmTask_UpdateUsbSupportVars(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool ufp_ack)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    const cy_stc_pdstack_dpm_status_t   *dpm_stat = &ptrPdStackContext->dpmStat;
    cy_stc_pdaltmode_app_status_t       *app      = &ptrAltModeContext->appStatus;
    cy_stc_pdaltmode_vdm_msg_info_t     *msg_p    = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);

    /* First set USB support flags based on cable marker properties. */
    if (dpm_stat->cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
    {
        if (dpm_stat->cblVdo2.act_cbl_vdo2.usb2Supp)
        {
            app->usb2Supp = true;
        }
        if (dpm_stat->cblVdo2.act_cbl_vdo2.ssSupp)
        {
            app->usb3Supp = true;
        }
    }
    else if (dpm_stat->cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL)
    {
        app->usb2Supp = true;

        if (dpm_stat->cblVdo.pas_cbl_vdo.usbSsSup > CY_PDSTACK_USB_GEN_2)
        {
            app->usb3Supp = true;
        }
    }

    /* If UFP response is available, confirm USB support in its D_ID response. */
    if (ufp_ack)
    {
        uint32_t dev_cap = msg_p->vdo[CY_PD_PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.devCap;
        
        if ((dev_cap & CY_PDSTACK_DEV_CAP_USB_2_0) != 0)
        {
            app->usb2Supp = true;
        }

        if ((dev_cap & CY_PDSTACK_DEV_CAP_USB_3_2) != 0)
        {
            app->usb3Supp = true;
        }
    }
}
#endif /* RIDGE_SLAVE_ENABLE || AMD_SUPP_ENABLE */

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_MngDiscId(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
#if CY_PD_USB4_SUPPORT_ENABLE
    cy_pd_pd_do_t* eudo_p = &ptrAltModeContext->vdmStat.eudo_buf.cmdDo[0];
    cy_pd_pd_do_t* vdo_do_p = (cy_pd_pd_do_t *)&ptrAltModeContext->vdmInfoConfig->discId;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
    cy_stc_pdaltmode_vdm_msg_info_t *msg_p  = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);
    cy_en_pdaltmode_vdm_task_t ret         = VDM_TASK_EXIT;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_stat->spec_rev_sop_live >= PD_REV3) && 
            (ptrPdStackContext->dpmStat->cur_port_type == CY_PD_PRT_TYPE_UFP));
#endif /* ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover ID VDM packet */
            Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP, CY_PDSTACK_VDM_CMD_DSC_IDENTITY);
            ret  = VDM_TASK_SEND_MSG;
            break;

        /* Evaluate received response */
        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if(ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
            {
#if CY_PD_USB4_SUPPORT_ENABLE
                uint8_t dfp_vdo_idx;
                /* Get DFP VDO index based on product type */
                if(vdo_do_p[CY_PD_ID_HEADER_IDX].std_id_hdr.prodTypeDfp != CY_PDSTACK_PROD_TYPE_HUB)
                {
                    dfp_vdo_idx = (vdo_do_p[CY_PD_ID_HEADER_IDX].std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_PSD) ?
                        CY_PD_PRODUCT_TYPE_VDO_1_IDX : CY_PD_PRODUCT_TYPE_VDO_3_IDX;

                }
                else
                {
                    dfp_vdo_idx = (ptrPdStackContext->ptrPortCfg->portRole == CY_PD_PRT_ROLE_SOURCE)?
                            CY_PD_PRODUCT_TYPE_VDO_1_IDX : CY_PD_PRODUCT_TYPE_VDO_3_IDX;
                }

                /* Check if USB4 mode already entered */
                if (ptrAltModeContext->appStatus.usb4_active != false)
                {
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_FAILED;
                    if ((msg_p->vdo[CY_PD_PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.altModes & CY_PD_UFP_NON_PH_ALT_MODE_SUPP_MASK) == false)
                    {
                        ret = VDM_TASK_WAIT;
                        break;
                    }
                }
                /* Check if CCG and port partner supports USB 4.0 */
                else if (
                         (ptrPdStackContext->dpmConfig.specRevSopLive < CY_PD_REV3) ||
                         ((msg_p->vdo[CY_PD_PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.devCap & CY_PDSTACK_DEV_CAP_USB_4_0) == 0) ||
                         ((vdo_do_p[dfp_vdo_idx].dfp_vdo.hostCap & CY_PDSTACK_HOST_CAP_USB_4_0) == 0) ||
                         (ptrAltModeContext->appStatus.usb4_data_rst_cnt > DATA_RST_RETRY_NUMB) ||
                         ((ptrAltModeContext->appStatus.custom_hpi_host_cap_control  &  USB4_EN_HOST_PARAM_MASK) == 0)
                   )
                {
                    /*
                     * Cannot proceed with USB4 if we do not have USB4 host support, the UFP does not have USB4
                     * device support, we are in PD 2.0 contract or if there have been too many data resets.
                     */
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_FAILED;
                }

#if STORE_DETAILS_OF_HOST
                cy_stc_pdaltmode_context_t *deviceContext =
                                    (cy_stc_pdaltmode_context_t *)ptrAltModeContext->hostDetails.dsAltModeContext;

                /* If USB4 mode is NOT enabled - then do not issue USB4 mode enter command */
                if(
                       ((deviceContext->hostDetails.ds_mode_mask & USB4_MODE_DFP ) != USB4_MODE_DFP) &&
                       (ptrPdStackContext->port == TYPEC_PORT_1_IDX)
                  )
                {
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_FAILED;
                }
#endif /* STORE_DETAILS_OF_HOST */

                /* Check if cable is USB 4.0 capable */
                if (
                        (ptrAltModeContext->vdmStat.usb4_flag != USB4_FAILED)  &&
                        (ptrPdStackContext->dpmConfig.emcaPresent != false)
                   )
                {
                    if ((msg_p->vdo[CY_PD_PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.altModes  & 0x02 ) == false)
                    {
                        /*
                         * Discovery process could not be run if Alternate Modes field in
                         * UFP VDO 1 response is set to zero.
                         */
                        ptrAltModeContext->vdmStat.alt_modes_not_supp = true;
                    }

                    /* Fill EUDO */
                    eudo_p->enterusb_vdo.usbMode    = CY_PD_USB_MODE_USB4;
                    eudo_p->enterusb_vdo.cableType  = false;
                    eudo_p->enterusb_vdo.cableSpeed = ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup;
                  
                    /* Set active cable type */
                    if(ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
                    {
                        eudo_p->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_ACTIVE_REDRIVER;

                        /* If VDO version is 1.3 and higher */
                        if ((ptrPdStackContext->dpmStat.cblVdo2.val != CY_PDALTMODE_NO_DATA) &&
                                (ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo1.vdoVersion >= CY_PD_CBL_VDO_VERS_1_3))
                        {
                            if(ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.optIsolated !=false)
                            {
                                eudo_p->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_OPTICAL;
                            }
                            else if ((ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.phyConn != false)  ||
                                        (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.activeEl == false ))
                            {
                                eudo_p->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_ACTIVE_REDRIVER;
                            }
                            else
                            {
                                eudo_p->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_ACTIVE_RETIMER;
                            }
                        }
                    }

                    /* Set EUDO cable current: Only honor 3A and 5A capable cables. */
                    eudo_p->enterusb_vdo.cableCurrent = 0;
                    if (
                            (
                                (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL) ||
                                (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.vbusThruCbl != 0)

                            ) &&
                            (
                             (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.vbusCur == CY_PDSTACK_CBL_VBUS_CUR_3A) ||
                              (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.vbusCur == CY_PDSTACK_CBL_VBUS_CUR_5A)
                            )
                       )
                    {
                        eudo_p->enterusb_vdo.cableCurrent = ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.vbusCur + 1;
                    }

                    /* Set USB4 not supported as default */
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_FAILED;

                    /* Active cable handler */
                    if (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
                    {
                        if (
                                (ptrPdStackContext->dpmStat.cblVdmVersion == CY_PDSTACK_STD_VDM_VER2) &&
                                (ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo1.vdoVersion >= CY_PD_CBL_VDO_VERS_1_3)
                            )
                        {
                            /* Active cable supports USB4 */
                            if (
                                    (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.usb4Supp == false)   &&
                                    (ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo.usbSsSup >= CY_PDSTACK_USB_GEN_1)
                                )
                            {
#if RIDGE_SLAVE_ENABLE
                                /* Set TBT_Active_link_training in case of USB4 support and VDO version 1.3 */
                                ptrAltModeContext->vdmStat.intel_reg.ridge_stat.act_link_train = true;
#endif /* RIDGE_SLAVE_ENABLE */
                                /* Set ridge register cable speed */
                                eudo_p->enterusb_vdo.cableSpeed = (ptrPdStackContext->dpmStat.cblVdo.pas_cbl_vdo.usbSsSup >= CY_PDSTACK_USB_GEN_3) ?
                                        CY_PDSTACK_USB_GEN_3 : ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup;

                                /* Enter USB4 mode */
                                Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP_PRIME, false);
                            }
                        }
                        else if (ptrPdStackContext->dpmStat.cblModeEn != false)
                        {
                            /* Go through TBT discovery */
                            ptrAltModeContext->vdmStat.usb4_flag = USB4_TBT_CBL_FIND;
                        }
                    }
                    else
                    {
                        /* Since we know cable marker is present; a non-active cable has to be passive. */

                        /* Set cable speed in EUDO */
                        eudo_p->enterusb_vdo.cableSpeed = (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup > CY_PDSTACK_USB_GEN_3) ?
                                CY_PDSTACK_USB_GEN_3 : ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup;

                        if ((ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup == CY_PDSTACK_USB_GEN_2) &&
                                 (ptrPdStackContext->dpmStat.cblModeEn != false))
                        {
                            /* Run TBT cable discovery */
                            ptrAltModeContext->vdmStat.usb4_flag = USB4_TBT_CBL_FIND;
                        }
                        else if (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup != CY_PDSTACK_USB_2_0)
                        {
#if RIDGE_SLAVE_ENABLE
                            if(ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup == CY_PDSTACK_USB_GEN_3)
                            {
                                /* Set TBT_Active_link_training in case of USB4 support and VDO version 1.3 */
                                ptrAltModeContext->vdmStat.intel_reg.ridge_stat.act_link_train = true;
                            }
#endif /* RIDGE_SLAVE_ENABLE */
                            /* Enter USB4 mode */
                            Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                        }
                    }
                }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
#if VDM_RESP_QUERY_SUPPORTED
                /* Store the D_ID response received. */
                vdm_disc_id_resp[port][0] = msg_p->vdm_header.val;
                memcpy ((uint8_t *)&vdm_disc_id_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                vdm_disc_id_resp_len[port] = msg_p->vdo_numb + 1;
#endif /* VDM_RESP_QUERY_SUPPORTED */

#if ((RIDGE_SLAVE_ENABLE) || (AMD_SUPP_ENABLE))
                /* Analyze Disc ID SOP & SOP' responses for USB2/USB3 compatibility */
                Cy_PdAltMode_VdmTask_UpdateUsbSupportVars (ptrAltModeContext, true);
#endif /* RIDGE_SLAVE_ENABLE || AMD_SUPP_ENABLE */

                /* Set svid idx to zero before start DISC SVID */
                ptrAltModeContext->vdmStat.svid_idx  = CY_PDALTMODE_NO_DATA;
                ptrAltModeContext->vdmStat.dsvid_cnt = 0;

                /* Copy ID header to info struct */
                ptrAltModeContext->vdmStat.atch_tgt.ama_vdo.val = msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].val;

                /* Copy AMA VDO to info struct */
                ptrAltModeContext->vdmStat.atch_tgt.tgt_id_header.val = msg_p->vdo[VDO_START_IDX - 1].val;

                /* If AMA and cable (if present) do not need Vconn, disable VConn. */
                if (
                        (ptrPdStackContext->dpmStat.vconnRetain == 0) &&
                        ((msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.prodType != CY_PDSTACK_PROD_TYPE_AMA) ||
                         (msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].std_ama_vdo.vconReq == 0)) &&
                        ((ptrPdStackContext->dpmConfig.emcaPresent == false) ||
                        (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.cblTerm == CY_PDSTACK_CBL_TERM_BOTH_PAS_VCONN_NOT_REQ))
                   )
                {
                    ptrPdStackContext->ptrAppCbk->vconn_disable(ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol);
                }

                /* If alt modes not supported. */
                if (msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.modSupport == false)
                {
#if CCG_UCSI_ENABLE
                    modal_op_support = false;
#endif /*CCG_UCSI_ENABLE*/
#if CY_PD_USB4_SUPPORT_ENABLE
                    ptrAltModeContext->vdmStat.alt_modes_not_supp = true;
                    if (ptrAltModeContext->vdmStat.usb4_flag == USB4_FAILED)
                    {
                        break;
                    }
#else
                    break;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

                }
#if CCG_UCSI_ENABLE
                modal_op_support = true;
#endif /*CCG_UCSI_ENABLE*/

                /* Send Disc SVID cmd */
                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                Cy_PdAltMode_VdmTask_SetEvt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                Cy_PdAltMode_VdmTask_SetEvt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
#if ((RIDGE_SLAVE_ENABLE) || (AMD_SUPP_ENABLE))
            /* Update USB support variables based on cable capabilities alone. */
            Cy_PdAltMode_VdmTask_UpdateUsbSupportVars (ptrAltModeContext, false);
#endif /* ((RIDGE_SLAVE_ENABLE) || (AMD_SUPP_ENABLE)) */
            break;
        default:
            break;
    }

    return ret;
}

/* Checks if input SVID already saved. */
static bool Cy_PdAltMode_VdmTasks_IsSvidStored(uint16_t *svid_arr, uint16_t svid)
{
    uint8_t  idx;

    /* Go through all SVIDs and compare with input SVID */
    for (idx = 0; idx < MAX_SVID_VDO_SUPP; idx++)
    {
        /* If input SVID already saved */
        if (svid_arr[idx] == svid)
        {
            return true;
        }
    }

    return false;
}

/*
   This function saves received Discover SVID resp,
   returns true if a NULL SVID was received.
 */
static bool Cy_PdAltMode_VdmTask_SaveSvids(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t *svid_arr, uint8_t max_svid)
{
    uint8_t         idx, vdo_count;
    uint16_t        svid;
    uint8_t         svid_idx = ptrAltModeContext->vdmStat.svid_idx;
    bool            retval   = false;
    cy_stc_pdaltmode_vdm_msg_info_t  *msg_p   = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);

    /* Compare received SVIDs with supported SVIDs */
    vdo_count = msg_p->vdo_numb;

    /* Stop further discovery if this response does not have the maximum no. of DOs. */
    if (vdo_count < (CY_PD_MAX_NO_OF_DO - 1))
    {
        retval = true;
    }

    for (idx = 0; idx < (vdo_count * 2); idx++)
    {
        if ((idx & 1) == 0)
        {
            /* Upper half of the DO. */
            svid = msg_p->vdo[idx >> 1].val >> 16u;
        }
        else
        {
            /* Lower half of the DO. */
            svid = msg_p->vdo[idx >> 1].val & 0xFFFFu;
        }

        if (svid_idx < (max_svid - 1))
        {
            /* Stop on NULL SVID. */
            if (svid == CY_PDALTMODE_NO_DATA)
            {
                retval = true;
            }
            else
            {
                /* If SVID not saved previously then save */
                if (Cy_PdAltMode_VdmTasks_IsSvidStored(svid_arr, svid) == false)
                {
#if SAVE_SUPP_SVID_ONLY
                    if (get_alt_modes_config_svid_idx(port, ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType, svid) != MODE_NOT_SUPPORTED)
#endif /* SAVE_SUPP_SVID_ONLY */
                    {
                        svid_arr[svid_idx] = svid;
                        svid_idx++;
                    }
                }
#if CY_PD_USB4_SUPPORT_ENABLE
                /* Check if TBT svid is supported by cable if USB4 related
                 * TBT cable Disc Mode procedure should be provided
                 */
                if (
                       (Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext)->sopType == CY_PD_SOP_PRIME) &&
                       (ptrAltModeContext->vdmStat.usb4_flag == USB4_TBT_CBL_FIND) &&
                       (svid == CY_PDALTMODE_TBT_SVID)
                    )
                {
                    /* Start TBT cable Discover mode process */
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_TBT_CBL_DISC_RUN;
                }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

            }
        }
        else
        {
            /* Cannot continue as we have no more space. */
            retval = true;
            break;
        }
    }

    /* Set zero after last SVID in info */
    svid_arr[svid_idx] = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->vdmStat.svid_idx = svid_idx;

    /* Terminate discovery after MAX_DISC_SVID_COUNT attempts. */
    ptrAltModeContext->vdmStat.dsvid_cnt++;
    if (ptrAltModeContext->vdmStat.dsvid_cnt >= MAX_DISC_SVID_COUNT)
        retval = true;

    return retval;
}

static bool Cy_PdAltMode_VdmTask_IsCblSvidReq(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
#if VCONN_OCP_ENABLE
    uint8_t fault_status = *(ptrAltModeContext->fault_status_p);
#endif /* VCONN_OCP_ENABLE */

    bool ret = false;

    /* If cable supports alternate modes, send SOP' disc svid */
    if(ptrPdStackContext->dpmStat.cblModeEn != false)
    {
        if (
                (ptrPdStackContext->dpmConfig.vconnLogical)
#if VCONN_OCP_ENABLE
                && ((fault_status & FAULT_APP_PORT_VCONN_FAULT_ACTIVE) == 0)
#endif /* VCONN_OCP_ENABLE */
           )
        {
            if(Cy_USBPD_Vconn_IsPresent(ptrPdStackContext->ptrUsbPdContext, ptrPdStackContext->dpmConfig.revPol) == false)
            {
                /*
                 * We are VConn source and VConn is off. Enable and apply a delay to let
                 * the EMCA power up.
                 */
                if (ptrAltModeContext->pdStackContext->ptrAppCbk->vconn_enable (ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol))
                {
                    /* Set a flag to indicate that we need to do Cable SVID checks. */
                    ret = true;

                    /* Start a timer to delay the retry attempt. */
                    Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER),
                                APP_CABLE_POWER_UP_DELAY, NULL);
                }
            }
            else
            {
                /* Set a flag to indicate that we need to do Cable SVID checks. */
                ret = true;
            }
        }
    }    
    
    return ret;
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_MngDiscSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    cy_en_pdaltmode_vdm_task_t          ret        = VDM_TASK_EXIT;
    cy_stc_pdaltmode_vdm_msg_info_t     *msg_p      = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);

#if VDM_RESP_QUERY_SUPPORTED
    uint32_t tmp;
#endif /* VDM_RESP_QUERY_SUPPORTED */

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_stat->spec_rev_sop_live >= PD_REV3) &&
            (ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
#if CY_PD_USB4_SUPPORT_ENABLE
            if (ptrAltModeContext->vdmStat.usb4_flag == USB4_TBT_CBL_FIND)
            {
                /* Enable Vconn if required */
                Cy_PdAltMode_VdmTask_IsCblSvidReq(ptrAltModeContext);

                /* Form Discover SVID VDM packet to find out TBT cable */
                Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_PRIME, CY_PDSTACK_VDM_CMD_DSC_SVIDS);
                ret  = VDM_TASK_SEND_MSG;
            }
            else
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
            {
                /* Form Discover SVID VDM packet */
                Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP, CY_PDSTACK_VDM_CMD_DSC_SVIDS);
                ret  = VDM_TASK_SEND_MSG;
            }
            break;

        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if(ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
            {
                /* For attached target response */
                if (msg_p->sopType == (uint8_t)CY_PD_SOP)
                {
#if VDM_RESP_QUERY_SUPPORTED
                    if (ptrAltModeContext->vdmStat.svid_idx == 0)
                    {
                        /* Save the DISC_SVID response. */
                        vdm_disc_svid_resp[port][0] = msg_p->vdm_header.val;
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] = msg_p->vdo_numb + 1;
                    }
                    else
                    {
                        /* Save the incremental DISC_SVID response. */
                        tmp = GET_MIN (msg_p->vdo_numb, MAX_DISC_SVID_RESP_LEN - vdm_disc_svid_resp_len[port]);
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][vdm_disc_svid_resp_len[port]], (uint8_t *)msg_p->vdo,
                                tmp * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] += tmp;
                    }
#endif /* VDM_RESP_QUERY_SUPPORTED */

                    if (msg_p->vdo[0].val == NONE_VDO)
                    {
                        /* No SVID supported */
                        break;
                    }

                    /* Save received SVIDs and check if a NULL SVID was received. */
                    if (Cy_PdAltMode_VdmTask_SaveSvids (ptrAltModeContext, ptrAltModeContext->vdmStat.atch_tgt.tgt_svid, MAX_SVID_VDO_SUPP) != false)
                    {
                        /* Check if cable Disc SVID should be done */
                        if (Cy_PdAltMode_VdmTask_IsCblSvidReq(ptrAltModeContext) != false)
                        {
                            ptrAltModeContext->vdmStat.svid_idx  = 0;
                            ptrAltModeContext->vdmStat.dsvid_cnt = 0;
                            Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_PRIME, CY_PDSTACK_VDM_CMD_DSC_SVIDS);
                            ret = VDM_TASK_SEND_MSG;
                        }
                        else
                        {
                            /*
                             * We are either not VConn source or failed to turn VConn ON.
                             * Skip SOP' checks in the unlikely case where this happens.
                             */
                            Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
                            ret = VDM_TASK_REG_ATCH_TGT_INFO;
                        }
                    }
                    else
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP, CY_PDSTACK_VDM_CMD_DSC_SVIDS);
                        ret = VDM_TASK_SEND_MSG;
                    }
                }
                /* For cable response */
                else
                {
                    /* If the EMCA returned any DOs, save the content. */
                    if ((msg_p->vdo[VDO_START_IDX - 1].val != NONE_VDO) &&
                            (Cy_PdAltMode_VdmTask_SaveSvids(ptrAltModeContext, ptrAltModeContext->vdmStat.atch_tgt.cbl_svid, MAX_CABLE_SVID_SUPP) == false))
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_PRIME, CY_PDSTACK_VDM_CMD_DSC_SVIDS);
                        ret = VDM_TASK_SEND_MSG;
                    }
                    else
                    {
                        /*
                           If EMCA did not support any SVIDs of interest and does not require VConn for operation,
                           we can disable VConn.
                         */
                        if (ptrAltModeContext->vdmStat.atch_tgt.cbl_svid[0] == CY_PDALTMODE_NO_DATA)
                        {
                            if (
                                    (ptrPdStackContext->dpmStat.vconnRetain == 0) &&
                                    (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.cblTerm == CY_PDSTACK_CBL_TERM_BOTH_PAS_VCONN_NOT_REQ)
                                    )
                            {
                                ptrPdStackContext->ptrAppCbk->vconn_disable(ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol);
                            }
                        }

                        /* Move to the next step. */
                        Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
                        ret = VDM_TASK_REG_ATCH_TGT_INFO;
#if CY_PD_USB4_SUPPORT_ENABLE
                        if (ptrAltModeContext->vdmStat.usb4_flag == USB4_TBT_CBL_DISC_RUN)
                        {
                            /* Run Enter USB4 related Disc mode process */
                            ret = VDM_TASK_USB4_TBT;
                        }
                        else if (
                                    (ptrAltModeContext->vdmStat.usb4_flag == USB4_TBT_CBL_FIND) &&
                                    (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL)
                                )
                        {
                            /* Enter USB4 mode */
                            Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                        }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
                    }
                }
            }
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                Cy_PdAltMode_VdmTask_SetEvt(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
#if CY_PD_USB4_SUPPORT_ENABLE
            if (ptrAltModeContext->appStatus.usb4_active == false)
            {
                if (ptrAltModeContext->vdmStat.usb4_flag != USB4_FAILED)
                {
                    if (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL)
                    {
                        /* Enter USB4 mode */
                        Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                    }
                    else
                    {
                        /* Resume VDM handling */
                        ret = Cy_PdAltMode_VdmTask_ResumeHandler(ptrAltModeContext);
                    }
                    break;
                }
            }
            else
            {
                ret = VDM_TASK_WAIT;
                break;
            }
#endif /* (CY_PD_USB4_SUPPORT_ENABLE) */
            /* If cable SVID fails */
            if (msg_p->sopType == (uint8_t)CY_PD_SOP_PRIME)
            {
                Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
            break;

        default:
            break;
    }

    return ret;
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static void Cy_PdAltMode_VdmTask_ResetMngr(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
#if VDM_RESP_QUERY_SUPPORTED
    uint8_t port = ptrPdStackContext->port;
#endif /* VDM_RESP_QUERY_SUPPORTED */
    ptrAltModeContext->vdmStat.rec_retry_cnt      = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->vdmStat.svid_idx           = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->vdmStat.dsvid_cnt          = CY_PDALTMODE_NO_DATA;

#if CY_PD_USB4_SUPPORT_ENABLE
    ptrAltModeContext->vdmStat.alt_modes_not_supp = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->vdmStat.usb4_flag          = USB4_NONE;
    memset(&ptrAltModeContext->vdmStat.eudo_buf, CY_PDALTMODE_NO_DATA, sizeof(cy_stc_pdstack_dpm_pd_cmd_buf_t));
#if RIDGE_SLAVE_ENABLE
    ptrAltModeContext->vdmStat.intel_reg.val      = CY_PDALTMODE_NO_DATA;
#endif /* RIDGE_SLAVE_ENABLE */
#if AMD_SUPP_ENABLE
    ptrAltModeContext->vdmStat.amd_status         = CY_PDALTMODE_NO_DATA;
#endif /* AMD_SUPP_ENABLE */  
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    ptrAltModeContext->vdmStat.vdm_vcs_rqt_state        = VCONN_RQT_INACTIVE;
    ptrAltModeContext->vdmStat.vdm_vcs_rqt_count        = 0;

    /* Clear arrays which hold SVIDs */
    CY_PDUTILS_MEM_SET((uint8_t *)&ptrAltModeContext->vdmStat.atch_tgt, CY_PDALTMODE_NO_DATA, sizeof(cy_stc_pdaltmode_atch_tgt_info_t));

    /* Store the pointer to the cable VDO discovered by PD stack. */
    ptrAltModeContext->vdmStat.atch_tgt.cblVdo = (cy_pd_pd_do_t*)&ptrPdStackContext->dpmStat.cblVdo;

    /* Clear the SOP'' reset state. */
    ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_IDLE;
    ptrAltModeContext->vdmStat.vdm_emca_rst_count = 0;

    ptrAltModeContext->appStatus.tbtCblVdo.val = CY_PDALTMODE_NO_DATA;

    /* Stop the VDM task delay timer. */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER));

#if VDM_RESP_QUERY_SUPPORTED
    vdm_disc_id_resp_len[port] = 0;
    CY_PDUTILS_MEM_SET ((uint8_t *)&vdm_disc_id_resp[port], 0u, CY_PD_MAX_NO_OF_DO * sizeof (uint32_t));
    vdm_disc_svid_resp_len[port] = 0;
    CY_PDUTILS_MEM_SET ((uint8_t *)&vdm_disc_svid_resp[port], 0u, MAX_DISC_SVID_RESP_LEN * sizeof (uint32_t));
#endif /* VDM_RESP_QUERY_SUPPORTED */
}

cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_ResumeHandler(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if CY_PD_USB4_SUPPORT_ENABLE
    /* Remove USB4 pending flag to allow VDM manager handling */
    ptrAltModeContext->vdmStat.usb4_flag = USB4_FAILED;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
    ptrAltModeContext->vdmStat.svid_idx  = CY_PDALTMODE_NO_DATA;
    memset(ptrAltModeContext->vdmStat.atch_tgt.cbl_svid, CY_PDALTMODE_NO_DATA, sizeof(ptrAltModeContext->vdmStat.atch_tgt.cbl_svid));
    memset(ptrAltModeContext->vdmStat.atch_tgt.tgt_svid, CY_PDALTMODE_NO_DATA, sizeof(ptrAltModeContext->vdmStat.atch_tgt.tgt_svid));
    Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
#if CY_PD_USB4_SUPPORT_ENABLE
    /* Resume VDM flow ir exit VDM handler based if alt mode support flag */
    ptrAltModeContext->vdmStat.vdm_task = (ptrAltModeContext->vdmStat.alt_modes_not_supp == false) ? VDM_TASK_DISC_SVID : VDM_TASK_EXIT;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
    return ptrAltModeContext->vdmStat.vdm_task;
}


void Cy_PdAltMode_VdmTask_MngrDeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* CDT 247011 re-fix */
    ptrAltModeContext->appStatus.vdm_prcs_failed = false;

    /* If VDM task is not active, no need to go through the rest of the steps. */
    if (ptrAltModeContext->appStatus.vdm_task_en != false)
    {
        Cy_PdAltMode_VdmTask_ResetMngr(ptrAltModeContext);

        /* Clear all application status flags. */
        ptrAltModeContext->appStatus.vdm_task_en      = false;
        ptrAltModeContext->appStatus.alt_mode_entered = false;
        ptrAltModeContext->appStatus.cbl_rst_done     = false;
        ptrAltModeContext->appStatus.trig_cbl_rst     = false;

        /* Exit from alt mode manager */
        (void)Cy_PdAltMode_Mngr_AltModeProcess(ptrAltModeContext, VDM_EVT_EXIT);

        /* Skip MUX update if port was de-attached */
        if (ptrPdStackContext->dpmConfig.attach == false)
        {
            ptrAltModeContext->appStatus.skip_mux_config = true;
        }

        /* Deinit App HW */
        Cy_PdAltMode_HW_DeInit(ptrAltModeContext);
        
        ptrAltModeContext->appStatus.skip_mux_config    = false;
        ptrAltModeContext->appStatus.cable_retimer_supp = false;
    }
}

void Cy_PdAltMode_VdmTask_MngrExitModes(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    if (
            (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP) &&
            (ptrAltModeContext->appStatus.alt_mode_entered != false)
       )
    {
        /* Exit from alt mode manager. */
        Cy_PdAltMode_Mngr_AltModeProcess(ptrAltModeContext, VDM_EVT_EXIT);
    }
}

static void Cy_PdAltMode_VdmTask_SopDpSoftResetCb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp, const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    switch (resp)
    {
        case CY_PDSTACK_CMD_SENT:
            /* Do nothing. */
            break;

        case CY_PDSTACK_RES_RCVD:
            /* Proceed with rest of alternate mode state machine. */
            ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_DONE;
            break;

        default:
            /* Retry the cable SOFT_RESET. */
            ptrAltModeContext->vdmStat.vdm_emca_rst_count++;
            if (ptrAltModeContext->vdmStat.vdm_emca_rst_count >= MAX_EMCA_DP_RESET_COUNT)
            {
                ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_DONE;
            }
            else
            {
                ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_RETRY;
            }
            break;
    }

    (void) pkt_ptr;
}

static void Cy_PdAltMode_VdmTask_SendSopDpSoftReset(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_stc_pdstack_dpm_pd_cmd_buf_t dpm_cmd_param;
    cy_en_pdstack_status_t     api_stat;

    dpm_cmd_param.cmdSop      = CY_PD_SOP_DPRIME;
    dpm_cmd_param.noOfCmdDo   = 0;
    dpm_cmd_param.datPtr      = NULL;
    dpm_cmd_param.timeout     = 15;

    api_stat = Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_SOFT_RESET_EMCA, &dpm_cmd_param, true, Cy_PdAltMode_VdmTask_SopDpSoftResetCb);
    switch (api_stat)
    {
        case CY_PDSTACK_STAT_SUCCESS:
            /* Wait for the cable SOFT_RESET response. */
            ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_WAIT;
            break;

        case CY_PDSTACK_STAT_BUSY:
        case CY_PDSTACK_STAT_NOT_READY:
            /* Need to retry the command. */
            ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_RETRY;
            break;

        default:
            /* Connection state changed. Abort the Cable SOFT_RESET process. */
            ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_DONE;
            break;
    }
}

static void Cy_PdAltMode_VdmTask_VconnSwapCb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp, const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    switch (resp)
    {
        case CY_PDSTACK_CMD_SENT:
            /* Do nothing. */
            break;

        case CY_PDSTACK_RES_RCVD:
            /* Proceed with rest of alternate mode state machine. */
            switch ((cy_en_pd_ctrl_msg_t)pkt_ptr->msg)
            {
                case CY_PD_CTRL_MSG_ACCEPT:
                    /*
                     * If EMCA has already been detected, apply a delay and proceed with next steps.
                     * Otherwise, keep waiting for cable discovery related events.
                     */
                    if(ptrPdStackContext->dpmConfig.emcaPresent)
                    {
                        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                                GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), APP_CABLE_POWER_UP_DELAY, NULL);
                        ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_INACTIVE;
                    }
                    break;

                case CY_PD_CTRL_MSG_WAIT:
                    ptrAltModeContext->vdmStat.vdm_vcs_rqt_count++;
                    if (ptrAltModeContext->vdmStat.vdm_vcs_rqt_count >= 10u)
                    {
                        ptrAltModeContext->vdmStat.vdm_vcs_rqt_count = (uint8_t)VCONN_RQT_FAILED;
                    }
                    else
                    {
                        /* Start a timer to repeat the VConn Swap request after a delay. */
                        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), 10u, NULL);
                    }
                    break;

                default:
                    ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_FAILED;
                    break;
            }
            break;

        default:
            /* Keep retrying the VConn Swap request. */
            ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_PENDING;
            break;
    }
}

/* Function to initiate VConn Swap and Cable discovery before we can go ahead with the next SOP'/SOP'' message. */
bool Cy_PdAltMode_VdmTask_InitiateVcsCblDiscovery(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_stc_pdstack_dpm_pd_cmd_buf_t  dpm_cmd_param;
    cy_en_pdstack_status_t           api_stat;
    bool                stat = false;
    cy_stc_pd_dpm_config_t* ptrDpmConfig = &(ptrPdStackContext->dpmConfig);

    if(ptrPdStackContext->dpmConfig.vconnLogical == false)
    {
        /* Disable automatic VConn Swap from the Policy Engine at this stage. */
        CALL_MAP(Cy_PdStack_Dpm_UpdateAutoVcsEnable)(ptrPdStackContext, false);
        CALL_MAP(Cy_PdStack_Dpm_UpdateVconnRetain)(ptrPdStackContext, 1u);

        if (ptrAltModeContext->vdmStat.vdm_vcs_rqt_state <= VCONN_RQT_ONGOING)
        {
            /* VDM sequence needs to be delayed because we need to do a VConn Swap. */
            stat = true;

            if (ptrAltModeContext->vdmStat.vdm_vcs_rqt_state != VCONN_RQT_ONGOING)
            {
#if CY_PD_CCG5_TO_PMG1S3_FEATURE
                uint8_t fault_status = *(ptrAltModeContext->fault_status_p);

                if (
                        (!Cy_USBPD_V5V_IsSupplyOn(ptrPdStackContext->ptrUsbPdContext)) ||
                        ((fault_status & FAULT_APP_PORT_VCONN_FAULT_ACTIVE) != 0)
                   )
                {
                    /* Skip the VConn Swap sequence if V5V is not up or if we have a VConn OCP fault status. */
                    ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_FAILED;
                    stat = false;
                }
                else
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
                {
                    dpm_cmd_param.cmdSop      = CY_PD_SOP;
                    dpm_cmd_param.noOfCmdDo   = 0u;
                    dpm_cmd_param.datPtr      = NULL;
                    dpm_cmd_param.timeout     = ptrPdStackContext->senderRspTimeout;

                    api_stat = Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_VCONN_SWAP, &dpm_cmd_param, false, Cy_PdAltMode_VdmTask_VconnSwapCb);
                    if (api_stat != CY_PDSTACK_STAT_SUCCESS)
                    {
                        ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_PENDING;
                    }
                    else
                    {
                        /* Clear the emca_present flag for the port so that cable state machine will start
                         * from the beginning.
                         */
                        ptrPdStackContext->dpmConfig.emcaPresent = false;

                        ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_ONGOING;
                    }
                }
            }
        }
    }
    else
    {
        if (ptrAltModeContext->vdmStat.vdm_vcs_rqt_state != VCONN_RQT_ONGOING)
        {
            ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_INACTIVE;
            ptrAltModeContext->vdmStat.vdm_vcs_rqt_count = 0;

            if(!ptrPdStackContext->ptrAppCbk->vconn_is_present(ptrPdStackContext))
            {
                /* Make sure VConn is enabled and sufficient delay provided for the cable to power up. */
                if (ptrPdStackContext->ptrAppCbk->vconn_enable (ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol))
                {
                        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER), APP_CABLE_POWER_UP_DELAY, NULL);
                        stat = true;
                }
            }
        }
        else
        {
            /* In case cable discovery is pending, wait for it. */
            stat = true;
        }
    }

    return stat;
}

void Cy_PdAltMode_VdmTask_UpdateVcsStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    if (ptrAltModeContext->vdmStat.vdm_vcs_rqt_state == VCONN_RQT_ONGOING)
    {
        ptrAltModeContext->vdmStat.vdm_vcs_rqt_state = VCONN_RQT_INACTIVE;
    }
}

/* Fill DPM cmd buffer with properly VDM info */
static uint8_t Cy_PdAltMode_VdmTask_ComposeVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    uint8_t           idx;
    cy_stc_pdstack_dpm_pd_cmd_buf_t  *vdm_buf = Cy_PdAltMode_Mngr_GetVdmBuff(ptrAltModeContext);
    cy_stc_pdaltmode_vdm_msg_info_t    *msg_p   = Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext);

    vdm_buf->cmdSop = (cy_en_pd_sop_t) msg_p->sopType;
    vdm_buf->noOfCmdDo = msg_p->vdo_numb + VDO_START_IDX;
    vdm_buf->cmdDo[CY_PD_VDM_HEADER_IDX] = msg_p->vdm_header;

    if (msg_p->vdm_header.std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED)
    {
        msg_p->vdm_header.std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_INITIATOR;

        if (vdm_buf->cmdSop == CY_PD_SOP)
        {
            vdm_buf->cmdDo[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer = ptrAltModeContext->appStatus.vdm_version;
        }
        else
        {
            vdm_buf->cmdDo[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer = ptrPdStackContext->dpmStat.cblVdmVersion;
        }

        /* Set exceptions for Enter/Exit mode cmd period */
        switch (msg_p->vdm_header.std_vdm_hdr.cmd)
        {
            case CY_PDSTACK_VDM_CMD_ENTER_MODE:
                vdm_buf->timeout = CY_PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case CY_PDSTACK_VDM_CMD_EXIT_MODE:
                vdm_buf->timeout = CY_PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case CY_PDSTACK_VDM_CMD_ATTENTION:
                /* No timeout required for attention messages. */
                vdm_buf->timeout = CY_PDALTMODE_NO_DATA;
                break;
            default:
                vdm_buf->timeout = CY_PD_VDM_RESPONSE_TIMER_PERIOD;
                break;
        }
    }
    /* Handle UVDM */
    else
    {
        vdm_buf->timeout = CY_PD_VDM_RESPONSE_TIMER_PERIOD;
    }

    /* Copy VDOs to send buffer */
    if (msg_p->vdo_numb > CY_PDALTMODE_NO_DATA)
    {
        for (idx = 0; idx < msg_p->vdo_numb; idx++)
        {
            vdm_buf->cmdDo[idx + VDO_START_IDX].val = msg_p->vdo[idx].val;
        }
    }

    return true;
}

/* Parse received VDM and save info in  */
static uint8_t Cy_PdAltMode_VdmTask_ParseVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t* rec_vdm)
{
    uint8_t vdo_idx;
    cy_stc_pdaltmode_vdm_msg_info_t *msg_p  = Cy_PdAltMode_VdmTask_GetMsg (ptrAltModeContext);

    msg_p->vdm_header = rec_vdm->dat[CY_PD_VDM_HEADER_IDX];
    msg_p->vdo_numb = (rec_vdm->len - VDO_START_IDX);
    msg_p->sopType = rec_vdm->sop;

    /* If VDO is present in received VDM */
    if (rec_vdm->len > VDO_START_IDX)
    {
        for (vdo_idx = 0; vdo_idx < rec_vdm->len; vdo_idx++)
        {
            msg_p->vdo[vdo_idx].val = rec_vdm->dat[vdo_idx + VDO_START_IDX].val;
        }
    }

    return true;
}

static cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_SendVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_en_pd_sop_t pkt_type = Cy_PdAltMode_Mngr_GetVdmBuff(ptrAltModeContext)->cmdSop;

    if((ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP) && (ptrPdStackContext->dpmConfig.attach != false))
    {
        if (pkt_type != CY_PD_SOP)
        {
            /* If VConn Swap and cable discovery are pending, wait for those steps. */
            if (Cy_PdAltMode_VdmTask_InitiateVcsCblDiscovery (ptrAltModeContext))
            {
                return VDM_TASK_SEND_MSG;
            }
        }

        if (pkt_type == CY_PD_SOP_DPRIME)
        {
            switch (ptrAltModeContext->vdmStat.vdm_emca_rst_state)
            {
                /* If SOP'' SOFT RESET has not been done or a retry is pending, try to send it. */
                case CABLE_DP_RESET_IDLE:
                case CABLE_DP_RESET_RETRY:
                    Cy_PdAltMode_VdmTask_SendSopDpSoftReset(ptrAltModeContext);
                    return VDM_TASK_SEND_MSG;

                case CABLE_DP_RESET_WAIT:
                    return VDM_TASK_SEND_MSG;

                case CABLE_DP_RESET_DONE:
                    if (ptrAltModeContext->appStatus.cbl_rst_done != false)
                    {
                        /* Send SOP Prime VDM again after cable reset */
                        Cy_PdAltMode_VdmTask_GetMsg(ptrAltModeContext)->sopType = CY_PD_SOP_PRIME;
                        ptrAltModeContext->vdmStat.vdm_emca_rst_state = CABLE_DP_RESET_SEND_SPRIME;
                        return VDM_TASK_SEND_MSG;
                    }
                    else
                    {
                        break;
                    }
                default:
                    /* EMCA SOFT_RESET done. We can proceed. */
                    break;
            }
        }
    }

    ptrAltModeContext->appStatus.vdm_retry_pending = (ptrAltModeContext->vdmStat.rec_retry_cnt != MAX_RETRY_CNT);

    if(Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_VDM,
            Cy_PdAltMode_Mngr_GetVdmBuff(ptrAltModeContext), false, Cy_PdAltMode_VdmTask_RecCbk) == CY_PDSTACK_STAT_SUCCESS)
    {
        return VDM_TASK_WAIT;
    }

    /* If fails - try to send VDM again */
    return VDM_TASK_SEND_MSG;
}

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
bool Cy_PdAltMode_VdmTask_IsUfpDiscStarted(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    if (Cy_PdAltMode_VdmTask_IsIdle(ptrAltModeContext))
    {
        /* Set VDM Task to send DISC ID VDM */
        Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_DISC_ID);
        Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
        return true;
    }

    return false;
}
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

static bool Cy_PdAltMode_VdmTask_IsVdmMngrEnabled(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return (bool)ptrAltModeContext->appStatus.vdm_task_en;
}

cy_en_pdaltmode_vdm_evt_t Cy_PdAltMode_VdmTask_GetEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return ptrAltModeContext->vdmStat.vdm_evt;
}

cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_GetTask(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return ptrAltModeContext->vdmStat.vdm_task;
}

void Cy_PdAltMode_VdmTask_SetTask(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_task_t task)
{
    ptrAltModeContext->vdmStat.vdm_task = task;
}

void Cy_PdAltMode_VdmTask_SetEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t evt)
{
    ptrAltModeContext->vdmStat.vdm_evt = evt;
}

static cy_stc_pdaltmode_vdm_msg_info_t* Cy_PdAltMode_VdmTask_GetMsg(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &ptrAltModeContext->vdmStat.vdm_msg;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
static uint8_t *vdm_get_stored_disc_resp(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool is_disc_id, uint8_t *resp_len_p)
{
    uint8_t *ptr = NULL;

    /* Check for bad pointer argument. */
    if (resp_len_p == NULL)
        return NULL;

    /* Set response length to zero by default. */
    *resp_len_p = 0;

    if ((port < NO_OF_TYPEC_PORTS) && (ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP))
    {
        if (is_disc_id)
        {
            *resp_len_p = vdm_disc_id_resp_len[port];
            ptr         = (uint8_t *)vdm_disc_id_resp[port];
        }
        else
        {
            *resp_len_p = vdm_disc_svid_resp_len[port];
            ptr         = (uint8_t *)vdm_disc_svid_resp[port];
        }
    }

    return (ptr);
}
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */

uint8_t *Cy_PdAltMode_VdmTask_GetDiscIdResp(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *resp_len_p) /* PRQA S 3408 */
{
#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    /* SIMN: Port needs to be replaced with Context. */
    return vdm_get_stored_disc_resp(port, true, resp_len_p);
#else
    (void) resp_len_p;
    (void) ptrAltModeContext;
    return NULL;
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */
}

uint8_t *Cy_PdAltMode_VdmTask_GetDiscSvidResp(uint8_t port, uint8_t *resp_len_p)
{
#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    return vdm_get_stored_disc_resp(port, false, resp_len_p);
#else
    (void) port;
    (void) resp_len_p;
    return NULL;
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */
}




/* [] END OF FILE */
