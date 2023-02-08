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

#if RIDGE_SLAVE_ENABLE
#include "cy_pdaltmode_intel_ridge_internal.h"
#include "cy_pdaltmode_intel_vid.h"

#elif AMD_SUPP_ENABLE
#include "amd.h"
#if AMD_RETIMER_ENABLE
#include "amd_retimer.h"
#endif /* AMD_RETIMER_ENABLE */
#endif /* RIDGE_SLAVE_ENABLE */

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_host_details.h"
#endif /* STORE_DETAILS_OF_HOST */

#if CY_PD_USB4_SUPPORT_ENABLE

static cy_pd_pd_do_t* get_eudo(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &ptrAltModeContext->vdmStat.eudo_buf.cmdDo[0];
}

cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Usb4_TbtHandler(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_en_pdaltmode_vdm_task_t ret               = VDM_TASK_REG_ATCH_TGT_INFO;
    cy_stc_pdaltmode_vdm_msg_info_t *msg_p        = &ptrAltModeContext->vdmStat.vdm_msg;

    Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);

    if (vdm_evt == VDM_EVT_RUN)
    {
        ret = VDM_TASK_SEND_MSG;
        switch (ptrAltModeContext->vdmStat.usb4_flag)
        {
            case USB4_TBT_CBL_DISC_RUN:
                /* Send Disc Mode to TBT cable */
                Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_PRIME, CY_PDSTACK_VDM_CMD_DSC_MODES);
                break;
            case USB4_TBT_CBL_ENTER_SOP_P:
                /* Set MUX to safe state as CCG enters to TBT alt mode */
                Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SAFE, CY_PDALTMODE_NO_DATA);

                /* Send Disc Mode to TBT cable */
                Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_PRIME, CY_PDSTACK_VDM_CMD_ENTER_MODE);
                msg_p->vdm_header.std_vdm_hdr.objPos = 1u;
                break;
            case USB4_TBT_CBL_ENTER_SOP_DP:
                /* Send Disc Mode to TBT cable */
                Cy_PdAltMode_VdmTask_SetDiscParam(ptrAltModeContext, CY_PD_SOP_DPRIME, CY_PDSTACK_VDM_CMD_ENTER_MODE);
                msg_p->vdm_header.std_vdm_hdr.objPos = 1u;
                break;
            default:
                break;
        }
        msg_p->vdm_header.std_vdm_hdr.svid = CY_PDALTMODE_TBT_SVID;
    }
    else if (vdm_evt == VDM_EVT_EVAL)
    {
        msg_p->vdm_header.std_vdm_hdr.svid = CY_PDALTMODE_TBT_SVID;
        switch (ptrAltModeContext->vdmStat.usb4_flag)
        {
            case USB4_TBT_CBL_DISC_RUN:
                /* Save TBT cable response */
                ptrAltModeContext->appStatus.tbtCblVdo = msg_p->vdo[0];
                /* Set common active/passive cable parameters */
                get_eudo(ptrAltModeContext)->enterusb_vdo.cableSpeed           = msg_p->vdo[0].tbt_vdo.cblSpeed;
#if RIDGE_SLAVE_ENABLE
                ptrAltModeContext->vdmStat.intel_reg.ridge_stat.act_link_train = msg_p->vdo[0].tbt_vdo.linkTraining;
#endif /* RIDGE_SLAVE_ENABLE */
                /* Handle active/passive cable */
                if  (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL)
                {
                    /*
                     * Special handling for LRD cable - bit#24 in TBT disc mode will be set to 1
                     * by this cable & bit#22 (retimer/redriver) will be set to redriver
                     */
                    if((msg_p->vdo[0].tbt_vdo.cableActive != false) && (msg_p->vdo[0].tbt_vdo.b22RetimerCbl == false))
                        get_eudo(ptrAltModeContext)->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_ACTIVE_REDRIVER;
#if RIDGE_SLAVE_ENABLE
                    /* Save TBT cable gen */
                    ptrAltModeContext->vdmStat.intel_reg.ridge_stat.cbl_gen = msg_p->vdo[0].tbt_vdo.cblGen;
#endif /* RIDGE_SLAVE_ENABLE */

                    /* Enter USB4 mode */
                    Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                }
                else if(ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
                {
#if RIDGE_SLAVE_ENABLE
                    /* Save TBT cable gen */
                    ptrAltModeContext->vdmStat.intel_reg.ridge_stat.cbl_gen = msg_p->vdo[0].tbt_vdo.cblGen;
#endif /* RIDGE_SLAVE_ENABLE */
                    get_eudo(ptrAltModeContext)->enterusb_vdo.cableType     = CY_PDSTACK_CBL_TYPE_ACTIVE_REDRIVER;

                    /* Set active cable bit */
                    if(msg_p->vdo[0].tbt_vdo.b22RetimerCbl != false)
                    {
                        get_eudo(ptrAltModeContext)->enterusb_vdo.cableType = CY_PDSTACK_CBL_TYPE_ACTIVE_RETIMER;
                        ptrAltModeContext->appStatus.cable_retimer_supp = true;
                    }

                    if (msg_p->vdo[0].tbt_vdo.cblGen != false)
                    {
                        ptrAltModeContext->vdmStat.usb4_flag = USB4_TBT_CBL_ENTER_SOP_P;
                        ret = VDM_TASK_USB4_TBT;
                    }
                    else
                    {
                        /* Resume VDM handling */
                        ret = Cy_PdAltMode_VdmTask_ResumeHandler(ptrAltModeContext);
                    }
                }
                break;
            case USB4_TBT_CBL_ENTER_SOP_P:
                /* Check if SOP" is present */
                if (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.sopDp != false)
                {
                    /* Send Enter TBT cable SOP" command */
                    ptrAltModeContext->vdmStat.usb4_flag = USB4_TBT_CBL_ENTER_SOP_DP;
                    ret = VDM_TASK_USB4_TBT;
                }
                else
                {
                    /* Enter USB4 mode */
                    Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                }
                break;
            case USB4_TBT_CBL_ENTER_SOP_DP:
                /* Enter USB4 mode */
                Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
                break;
            default:
                break;
        }
    }
    else if (vdm_evt == VDM_EVT_FAIL)
    {
        if (
                (ptrAltModeContext->vdmStat.usb4_flag >= USB4_TBT_CBL_DISC_RUN)    &&
                (ptrAltModeContext->vdmStat.usb4_flag <= USB4_TBT_CBL_ENTER_SOP_DP)
            )
        {
            if  (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_PAS_CBL)
            {
                /* Enter USB4 mode with passive cable Disc ID response parameters */
                Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, CY_PD_SOP, false);
            }
             else if (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
            {
                /* Resume VDM handling */
                ret = Cy_PdAltMode_VdmTask_ResumeHandler(ptrAltModeContext);
            }
        }
    }

    return ret;
}

static void Cy_PdAltMode_Usb4_DataRstRecCbk(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t status, const cy_stc_pdstack_pd_packet_t* rec_vdm)
{
    /* Stack would already do error recovery if the Data Reset response from port partner is not valid. */
    if (status <= CY_PDSTACK_CMD_FAILED)
    {
        /* Send Data Reset message */
        if (Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_DATA_RESET, NULL, false, Cy_PdAltMode_Usb4_DataRstRecCbk) != CY_PDSTACK_STAT_SUCCESS)
        {
            Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                    GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER),
                        APP_VDM_BUSY_TIMER_PERIOD, Cy_PdAltMode_Usb4_DataRstRetryCbk);
        }
    }

    (void) rec_vdm;
}

void Cy_PdAltMode_Usb4_DataRstRetryCbk (cy_timer_id_t id, void *context)
{
    (void)id;

    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)context;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    /* Increment Data Reset counter */
    ptrAltModeContext->appStatus.usb4_data_rst_cnt++;

    /* Send Data Reset command */
    if (Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_DATA_RESET, NULL, false, Cy_PdAltMode_Usb4_DataRstRecCbk) != CY_PDSTACK_STAT_SUCCESS)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER),
                    APP_VDM_BUSY_TIMER_PERIOD, Cy_PdAltMode_Usb4_DataRstRetryCbk);
    }
}

uint32_t Cy_PdAltMode_Usb4_UpdateDataStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t eudo, uint32_t val)
{
#if RIDGE_SLAVE_ENABLE
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_stc_pdaltmode_ridge_reg_t reg;

    /* Start with the original value. */
    reg.val = val;

    /* Set cable speed. */
    reg.ridge_stat.cbl_spd = eudo.enterusb_vdo.cableSpeed;

    /* Set cable type variables. */
    switch (eudo.enterusb_vdo.cableType)
    {
        case CY_PDSTACK_CBL_TYPE_OPTICAL:
            reg.ridge_stat.cbl_type = true;
            reg.ridge_stat.active_cbl = true;
            break;

        case CY_PDSTACK_CBL_TYPE_ACTIVE_RETIMER:
            reg.ridge_stat.retimer = true;
            reg.ridge_stat.active_cbl = true;
            break;
        case CY_PDSTACK_CBL_TYPE_ACTIVE_REDRIVER:
            reg.ridge_stat.active_cbl = true;
            break;

        case CY_PDSTACK_CBL_TYPE_PASSIVE:
            reg.ridge_stat.active_cbl = false;
            reg.ridge_stat.retimer = false;
            break;

        default:
            break;
    }

    /* Set USB speed */
    if (ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup >= CY_PDSTACK_USB_GEN_2)
    {
        reg.ridge_stat.usb3_speed = true;
    }

    /* Set USB4 related MUX */
    Cy_PdAltMode_HW_SetMux (ptrAltModeContext, MUX_CONFIG_USB4_CUSTOM, reg.val);
    return (reg.val);
#elif AMD_SUPP_ENABLE
    (void) val;

    if (ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        /* Enable Retimer in case of UFP data role */
        amd_retimer_enable(port, false, true);
    }

    /* Set USB4 related MUX */
    Cy_PdAltMode_HW_SetMux (ptrAltModeContext, MUX_CONFIG_USB4_CUSTOM, eudo.val);

    return (eudo.val);
#else
    (void)ptrAltModeContext;
    (void)eudo;
    (void)val;
    return CY_PDALTMODE_NO_DATA;
#endif /* RIDGE_SLAVE_ENABLE */
}

static void Cy_PdAltMode_Usb4_EnterRecCbk(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t status, const cy_stc_pdstack_pd_packet_t* rec_vdm)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;
    cy_en_pd_sop_t sop_type = CY_PD_SOP;

    if (status == CY_PDSTACK_RES_RCVD)
    {
        if (rec_vdm->hdr.hdr.msgType == CY_PDSTACK_REQ_ACCEPT)
        {
            /* Remove USB4 pending flag to allow VDM manager handling */
            ptrAltModeContext->vdmStat.usb4_flag = USB4_NONE;

            /* USB4 entered */
            if (ptrAltModeContext->vdmStat.eudo_buf.cmdSop == CY_PD_SOP)
            {
#if RIDGE_SLAVE_ENABLE
                /* Update cable type field */
                if (
                        (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) &&
                        (ptrPdStackContext->dpmStat.cblVdo2.val != CY_PDALTMODE_NO_DATA) &&
                        (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.activeEl != false) &&
                        (ptrPdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.phyConn != false)
                   )
                {
                    ptrAltModeContext->vdmStat.intel_reg.ridge_stat.cbl_type = true;
                }

                /* Set SoC data status fields based on EUDO content. */
                ptrAltModeContext->vdmStat.intel_reg.val = Cy_PdAltMode_Usb4_UpdateDataStatus (ptrAltModeContext, ptrAltModeContext->vdmStat.eudo_buf.cmdDo[0], ptrAltModeContext->vdmStat.intel_reg.val);
#elif AMD_SUPP_ENABLE
                /* Set USB4 related MUX */
                Cy_PdAltMode_Usb4_UpdateDataStatus (port, ptrAltModeContext->vdmStat.eudo_buf.cmd_do[0], ptrAltModeContext->vdmStat.amd_status);
#endif /* RIDGE_SLAVE_ENABLE */
                /* Check if we need to continue with Discovery */
                if (
                       (ptrAltModeContext->vdmStat.alt_modes_not_supp != false)
#if VDM_RESP_QUERY_SUPPORTED
                    /* If TBT mode only (not vPro) supported then don't resume VDM discovery */
                    || ((vdm_disc_id_resp[port][PRODUCT_TYPE_VDO_1_IDX] & UFP_ALT_MODE_SUPP_MASK) == UFP_TBT_ALT_MODE_SUPP_MASK)
#endif /* VDM_RESP_QUERY_SUPPORTED */
                   )
                {
                    Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_WAIT);
                }
                else
                {
                    /* Restart Discovery flow to discover alternate modes */
                    Cy_PdAltMode_VdmTask_SetEvt(ptrAltModeContext, VDM_EVT_RUN);
                    Cy_PdAltMode_VdmTask_SetTask(ptrAltModeContext, VDM_TASK_DISC_SVID);

                    /* Set svid idx to zero before start DISC SVID */
                    ptrAltModeContext->vdmStat.svid_idx  = CY_PDALTMODE_NO_DATA;
                    ptrAltModeContext->vdmStat.dsvid_cnt = CY_PDALTMODE_NO_DATA;
                }
                return;
            }
            /* USB4 SOP or SOP" should be sent to finish USB4 entry */
            else if (ptrAltModeContext->vdmStat.eudo_buf.cmdSop == CY_PD_SOP_PRIME)
            {
                sop_type = ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo.sopDp ? CY_PD_SOP_DPRIME : CY_PD_SOP;
            }

            Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, sop_type, false);
        }
        else if ((rec_vdm->hdr.hdr.msgType == CY_PDSTACK_REQ_REJECT) || (rec_vdm->hdr.hdr.msgType == CY_PDSTACK_REQ_NOT_SUPPORTED))
        {
            /* Resume VDM handling */
            Cy_PdAltMode_VdmTask_ResumeHandler(ptrAltModeContext);
        }
    }
    else if (status <= CY_PDSTACK_CMD_FAILED)
    {
        /* Send Enter USB4 message */
        Cy_PdAltMode_Usb4_Enter(ptrAltModeContext, sop_type, true);
    }
}

static void Cy_PdAltMode_Usb4_EnterUsb4RetryCbk (cy_timer_id_t id, void *context)
{
    (void)id;

    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t*) context;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    cy_en_pdstack_status_t stat = Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_ENTER_USB,
                &ptrAltModeContext->vdmStat.eudo_buf, false, Cy_PdAltMode_Usb4_EnterRecCbk);

    /* Send Enter USB4 command */
    if (stat != CY_PDSTACK_STAT_SUCCESS)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_VDM_BUSY_TIMER),
                  APP_USB4_ENTRY_RETRY_PERIOD, Cy_PdAltMode_Usb4_EnterUsb4RetryCbk);
    }
}

void Cy_PdAltMode_Usb4_Enter(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_sop_t sop_type, bool retry)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    cy_stc_pdstack_dpm_pd_cmd_buf_t  *cmd_buf = &ptrAltModeContext->vdmStat.eudo_buf;
    cy_en_pdstack_status_t stat = CY_PDSTACK_STAT_FAILURE;
    cy_pd_pd_do_t eudo;

    eudo.val = 0;

    /* Set EUDO USB role based on configuration table */
    if (ptrAltModeContext->tbtCfg->usb4Support == CY_PDSTACK_USB_ROLE_DRD)
    {
        eudo.enterusb_vdo.usb4Drd = true;
    }
    if (ptrAltModeContext->tbtCfg->usb3Support == CY_PDSTACK_USB_ROLE_DRD)
    {
        eudo.enterusb_vdo.usb3Drd = true;
    }

    /* Set USB Host supported parameters based on configuration table */
    eudo.val |= ((ptrAltModeContext->appStatus.custom_hpi_host_cap_control & 0x0F) << USB4_EUDO_HOST_PARAM_SHIFT);

    if(ptrAltModeContext->appStatus.tbtCblVdo.tbt_cbl_vdo.b22RetimerCbl == 1)
    {
        eudo.enterusb_vdo.cableType = 1;
    }

#if STORE_DETAILS_OF_HOST
    if(ptrPdStackContext->port == TYPEC_PORT_1_IDX)
    {
        cy_pd_pd_do_t temp_enter_usb_param;
        cy_stc_pdaltmode_ridge_reg_t temp_host_details;;
        cy_stc_pdaltmode_context_t *hostContext =
                (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.usAltModeContext;

        temp_enter_usb_param.val = hostContext->hostDetails.host_eudo.val;
        temp_host_details.val = hostContext->hostDetails.host_details;

        if(temp_host_details.ridge_stat.usb4_conn == true)
        {
            eudo.enterusb_vdo.hostDpSupp = temp_enter_usb_param.enterusb_vdo.hostDpSupp;
            eudo.enterusb_vdo.hostPcieSupp = temp_enter_usb_param.enterusb_vdo.hostPcieSupp;
            eudo.enterusb_vdo.hostTbtSupp = temp_enter_usb_param.enterusb_vdo.hostTbtSupp;
            eudo.enterusb_vdo.hostPresent = temp_enter_usb_param.enterusb_vdo.hostPresent;
            eudo.enterusb_vdo.usbMode = temp_enter_usb_param.enterusb_vdo.usbMode;
        }
        else
        {
            eudo.enterusb_vdo.hostDpSupp = 1;
            eudo.enterusb_vdo.hostPcieSupp = 1;
            eudo.enterusb_vdo.hostTbtSupp = 1;
            eudo.enterusb_vdo.hostPresent = 0;
            eudo.enterusb_vdo.usbMode = CY_PDALTMODE_USB_MODE_USB4;
        }

        /*eudo.enterusb_vdo.hostDpSupp = 1;
        eudo.enterusb_vdo.hostPcieSupp = 1;
        eudo.enterusb_vdo.hostTbtSupp = 1;
        eudo.enterusb_vdo.hostPresent = 1;*/
    }
#endif /* STORE_DETAILS_OF_HOST */

    get_eudo(ptrAltModeContext)->val |= eudo.val;

    /* Fill dpm buffer */
    cmd_buf->cmdSop      = sop_type;
    cmd_buf->noOfCmdDo   = VDO_START_IDX;
    cmd_buf->timeout     = CY_PD_SENDER_RESPONSE_TIMER_PERIOD;

    if(!retry)
    {
        /* Set MUX to safe state similar to TBT alt mode */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SAFE, CY_PDALTMODE_NO_DATA);

        /* Set flag to stop VDM manager while send Enter USB4 command */
        ptrAltModeContext->vdmStat.usb4_flag = USB4_PENDING;
    }

    /* If sending message to EMCA, make sure we are the VConn Source and have the supply turned ON. */
    if ((sop_type == CY_PD_SOP) || (Cy_PdAltMode_VdmTask_InitiateVcsCblDiscovery (ptrAltModeContext) == false))
    {
        stat = Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_ENTER_USB,
                cmd_buf, false, Cy_PdAltMode_Usb4_EnterRecCbk);
    }

    /* In case of failure, start timer to retry the request. */
    if (stat != CY_PDSTACK_STAT_SUCCESS)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                 GET_APP_TIMER_ID(ptrPdStackContext,  APP_VDM_BUSY_TIMER),
                     APP_USB4_ENTRY_RETRY_PERIOD, Cy_PdAltMode_Usb4_EnterUsb4RetryCbk);
    }
}
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
