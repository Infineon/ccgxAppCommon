/******************************************************************************
* File Name:   vdm.c
* \version 2.0
*
* Description: Vendor Defined Message (VDM) handler source file.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/
#include "config.h"
#include <cy_pdutils_sw_timer.h>
#include <vdm.h>
#include "cy_pdstack_common.h"
#include <app.h>
#include <cy_pdaltmode_vdm_task.h>
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
#include <cy_pdaltmode_mngr.h>
#endif /* (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP) */
#if DP_GPIO_CONFIG_SELECT
#include "cy_pdaltmode_dp_sid.h"
#endif /* DP_GPIO_CONFIG_SELECT */

#if ((TBT_DFP_SUPP) || (TBT_UFP_SUPP))
#include <cy_pdaltmode_intel_vid.h>
#endif /* ((TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

#if (AUVDM_SUPPORT != 0)
#include "auvdm.h"
#endif

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S))
#if CCG_LOAD_SHARING_ENABLE
#include <loadsharing.h>
#endif /* CCG_LOAD_SHARING_ENABLE */
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S) */
#include "cy_usbpd_config_table.h"
#include "srom.h"
#include "uvdm.h"

cy_stc_pdaltmode_vdm_info_config_t vdmInfo[NO_OF_TYPEC_PORTS];

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S))
/*
 * In case of multiport CCG3PA systems, stores the dynamically
 * assigned port number that is reported as a part of discover
 * identity response.
 */
static uint8_t gl_vdm_port_num = 0u;

#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S) */


#if (FLASH_ENABLE_NB_MODE == 1) && (FLASHING_MODE_PD_ENABLE == 1)
vdm_resp_cbk_t gl_vdm_resp_cbk = NULL;
/*
 * In Non-Blocking Flash Write Scenario, this is the callback
 * invoked by Flash Module after Flash row write completes.
 * This should be used to send back the VDM response.
 *
 * @param NONE
 * @param NONE
 */
void uvdm_nb_flash_write_cb(flash_write_status_t write_status)
{
    /* Non-Blocking Flashing is supported only for CCG3. Hence the port index is
     * fixed to 0.*/
    app_status_t* app_stat = app_get_status(0);

    /* Response should be sent out only if Flash write was not aborted. */
    if (write_status == FLASH_WRITE_COMPLETE)
    {
        app_stat->vdmResp.respBuf[UVDM_HEADER_INDEX].val =
            ((get_pd_config()->flashing_vid << 16) |
            (CMD_TYPE_RESP_ACK << 11) | (uvdm_get_cur_nb_cmd ()));
        app_stat->vdmResp.respBuf[UVDM_RESPONSE_VDO_INDEX].val =
            (CCG_STAT_SUCCESS + CCG_STATUS_CODE_OFFSET);

        app_stat->vdmResp.doCount = 2;
        /* Ignore Unstructured VDM */
        app_stat->vdmResp.noResp = VDM_AMS_RESP_READY;
        gl_vdm_resp_cbk (0, &app_stat->vdmResp);
    }

    uvdm_reset_nb_cmd_state ();
}
#else
flash_cbk_t uvdm_nb_flash_write_cb = NULL;
#endif /* (FLASH_ENABLE_NB_MODE == 1) && (FLASHING_MODE_PD_ENABLE == 1) */

/* Store VDM information from the config table in the RAM variables. */
void vdm_data_init(cy_stc_pdstack_context_t * context)
{
    cy_stc_pdaltmode_cfg_settings_t *altcfg = pd_get_ptr_base_alt_tbl(context->ptrUsbPdContext);
    cy_stc_pdaltmode_context_t * ptrAltModeContext = context->ptrAltModeContext;
    uint8_t * auto_offset = (uint8_t *)get_auto_config(context->ptrUsbPdContext);
    uint32_t * offset;
    uint32_t temp;
    uint16_t size = 0;
    uint8_t i;

    vdmInfo[context->port].dIdLength = altcfg->disc_id_len;
    if(altcfg->disc_id_len != 0x00u)
    {
        for(i = 0; i < (((altcfg->disc_id_len)/4) - 1); i++)
        {
            offset = (uint32_t *)(auto_offset + (altcfg->disc_id_offset + (4 * (i+1))));
            temp = (uint32_t)(*offset);
            vdmInfo[context->port].discId[i] = temp;
        }
    }

    vdmInfo[context->port].sVidLength = altcfg->disc_svid_len;
    if(altcfg->disc_svid_len != 0x00u)
    {
        for(uint8_t i=0; i < ((altcfg->disc_svid_len/4) - 1); i++)
        {
            offset = (uint32_t *)(auto_offset + (altcfg-> disc_svid_offset + (4 * (i+1))));
            temp = (uint32_t)(*offset);
            vdmInfo[context->port].sVid[i] = temp;
        }
    }

    vdmInfo[context->port].discMode[0].disModeLength = altcfg->disc_mode_len;
    if(altcfg->disc_mode_len != 0x00u)
    {
        for(uint8_t i=0; i < ((altcfg->disc_mode_len/4) - 1); i++)
        {
            offset = (uint32_t *)(auto_offset + (altcfg-> disc_mode_offset + (4 * (i+1))));
            temp = (uint32_t)(*offset);
            vdmInfo[context->port].discMode[0].modeDataObj[i] = temp;
        }

        vdmInfo[context->port].discMode[0].dataObjsVid = ((vdmInfo[context->port].discMode[0].modeDataObj[0] >> 16) & 0xFFFFu);
    }

    ptrAltModeContext->vdmInfoConfig = &vdmInfo[context->port];

    /* Calculate the number of VDOs in the D_ID response. */
    size = ptrAltModeContext->vdmInfoConfig->dIdLength;
    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
    {
        ptrAltModeContext->vdmIdVdoCnt = (uint8_t)((size - 4) >> 2);
    }
    else
    {
        ptrAltModeContext->vdmIdVdoCnt = 0;
    }

    /* Copy the actual Discover Identity response. */
    memcpy ((uint8_t *)ptrAltModeContext->vdmIdVdoResp, (const uint8_t *)ptrAltModeContext->vdmInfoConfig->discId,
            ptrAltModeContext->vdmIdVdoCnt * 4u);

    /* Update the vendor and product IDs from the configuration data. */
    ptrAltModeContext->vdmIdVdoResp[1].std_id_hdr.usbVid = context->ptrPortCfg->mfgVid;
    ptrAltModeContext->vdmIdVdoResp[3].std_prod_vdo.usbPid = context->ptrPortCfg->mfgPid;

    /* Update the D_ID response pointer. */
    ptrAltModeContext->vdmIdVdoP = (cy_pd_pd_do_t *)(ptrAltModeContext->vdmIdVdoResp);

    /* Calculate the number of VDOs in the D_SVID response. */
    size = ptrAltModeContext->vdmInfoConfig->sVidLength;

    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
    {
        ptrAltModeContext->vdmSvidVdoCnt = (uint8_t)((size - 4) >> 2);

        /* Update the D_SVID response pointer. */
        ptrAltModeContext->vdmSvidVdoP = (cy_pd_pd_do_t *)((uint8_t *)(ptrAltModeContext->vdmInfoConfig->sVid));

#if LEGACY_CFG_TABLE_SUPPORT
        /* Store the D_MODE response length from configuration table. */
        ptrAltModeContext->vdmModeDataLen = pd_get_ptr_base_alt_tbl(context->ptrUsbPdContext)->disc_mode_len;

        /* Store pointer to the D_MODE responses. */
        ptrAltModeContext->vdmModeDataP = (uint8_t *)(((const uint8_t *)get_auto_config(context->ptrUsbPdContext)) +
                (pd_get_ptr_base_alt_tbl(context->ptrUsbPdContext)-> disc_mode_offset));

#endif /* LEGACY_CFG_TABLE_SUPPORT */
    }
    else
    {
        ptrAltModeContext->vdmSvidVdoCnt = 0;
#if LEGACY_CFG_TABLE_SUPPORT
        ptrAltModeContext->vdmModeDataLen = 0;
#endif /* LEGACY_CFG_TABLE_SUPPORT */
    }
}

void vdm_update_svid_resp(cy_stc_pdstack_context_t * context, uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p)
{
    uint8_t port = context->port;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = context->ptrAltModeContext;
    if(port < NO_OF_TYPEC_PORTS)
    {
        ptrAltModeContext->vdmSvidVdoCnt  = svid_vdo_cnt;
        ptrAltModeContext->vdmSvidVdoP    = (cy_pd_pd_do_t *)svid_vdo_p;
    }
}

void vdm_update_data(cy_stc_pdstack_context_t * context, uint8_t id_vdo_cnt, uint8_t *id_vdo_p,
        uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p, uint16_t mode_resp_len, uint8_t *mode_resp_p)
{
    uint8_t port = context->port;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = context->ptrAltModeContext;

    if (port < NO_OF_TYPEC_PORTS)
    {
        ptrAltModeContext->vdmIdVdoCnt    = id_vdo_cnt;
        ptrAltModeContext->vdmIdVdoP      = (cy_pd_pd_do_t *)id_vdo_p;
        ptrAltModeContext->vdmSvidVdoCnt  = svid_vdo_cnt;
        ptrAltModeContext->vdmSvidVdoP    = (cy_pd_pd_do_t *)svid_vdo_p;
#if LEGACY_CFG_TABLE_SUPPORT
        ptrAltModeContext->vdmModeDataLen = mode_resp_len;
        ptrAltModeContext->vdmModeDataP   = mode_resp_p;
#endif /* LEGACY_CFG_TABLE_SUPPORT */
    }
}

bool get_modes_vdo_info(cy_stc_pdstack_context_t * context, uint16_t svid, cy_pd_pd_do_t **temp_p, uint8_t *no_of_vdo)
{
    uint8_t d_mode_resp_size_total;
    uint8_t d_mode_resp_size;
    cy_pd_pd_do_t *header;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = context->ptrAltModeContext;
    uint8_t* resp_p = ptrAltModeContext->vdmModeDataP;

    /* Parse all responses based on SVID */
    d_mode_resp_size_total = ptrAltModeContext->vdmModeDataLen;

    /* If size is less than or equal to 4, return NACK. */
    if (d_mode_resp_size_total <= 4u)
    {
        return false;
    }

    while (d_mode_resp_size_total != 0u)
    {
        /* Get the size of packet. */
        d_mode_resp_size = *resp_p;

        /* Read the SVID from header. */
        /* QAC suppression 0310, 3305: The correct alignment of this pointer is ensured by the
         * configuration table that is generated by EZ-PD configuration utility.
         */
        header = (cy_pd_pd_do_t *)(resp_p + 4); /* PRQA S 0310, 3305 */
        if (header->std_vdm_hdr.svid == svid)
        {
            *no_of_vdo = ((d_mode_resp_size-4u) >> 2u);
            *temp_p = header;
            return true;
        }
        /* Move to next packet. */
        resp_p += d_mode_resp_size;
        d_mode_resp_size_total -= d_mode_resp_size;
    }
    return false;
}

cy_pd_pd_do_t* get_gl_vdm_id(cy_stc_pdstack_context_t * context)
{
    cy_stc_pdaltmode_context_t * ptrAltModeContext = context->ptrAltModeContext;
    return (ptrAltModeContext->vdmIdVdoP);
}

void vdm_assign_port_num(cy_stc_pdstack_context_t * context)
{
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S))
    gl_vdm_port_num = context->port;
#else
    CY_UNUSED_PARAMETER(context);
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S) */
}

/* Once the discover identity response is formed, call this function to assign
   port number in multiport CCG3PA/CCG7S systems. */
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S))
static void vdm_disc_id_resp_append_port_num(cy_stc_pdstack_context_t * context)
{

    cy_stc_pdaltmode_app_status_t* app_stat = &(((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus);

    app_stat->vdmResp.respBuf[CY_PD_PRODUCT_TYPE_VDO_1_IDX].dfp_vdo.portNum
        = gl_vdm_port_num;
}
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S) */

void eval_vdm(cy_stc_pdstack_context_t * context, const cy_stc_pdstack_pd_packet_t *vdm, cy_pdstack_vdm_resp_cbk_t vdm_resp_handler)
{
#if (CCG_HPI_PD_ENABLE || FLASHING_MODE_PD_ENABLE)
    cy_stc_pdaltmode_app_status_t* app_stat = &(((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus);
#endif /* CCG_HPI_PD_ENABLE || FLASHING_MODE_PD_ENABLE */
    cy_stc_pdaltmode_context_t* ptrAltModeContext = context->ptrAltModeContext;;

    cy_pd_pd_do_t* dobj = NULL;
    uint8_t i, count = 0u;
    bool pd3_live = false;

#if (!CCG_BACKUP_FIRMWARE)
    bool eval_rslt;
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if (FLASHING_MODE_PD_ENABLE == 1)
#if (FLASH_ENABLE_NB_MODE == 1)
    gl_vdm_resp_cbk = vdm_resp_handler;
#endif /* FLASH_ENABLE_NB_MODE */
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
#endif /* FLASHING_MODE_PD_ENABLE */

#if CY_PD_REV3_ENABLE
    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        pd3_live = true;
    }
#endif /* CY_PD_REV3_ENABLE */

    /* By Default assume we will respond */
    ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_READY;

    if (
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED) &&
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType == CY_PDSTACK_CMD_TYPE_INITIATOR)
       )
    {
        /* Copy received VDM Header data to VDM response Header*/
        ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].val = vdm->dat[CY_PD_VDM_HEADER_IDX].val;
#if CY_PD_REV3_ENABLE
        /* Use the minimum VDM version from among the partner's revision and the live revision. */
        ptrAltModeContext->appStatus.vdm_version = CY_USBPD_GET_MIN (ptrAltModeContext->appStatus.vdm_version,
                vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer);
#endif /* CY_PD_REV3_ENABLE */

        /* Set a NAK response by default. */
        ptrAltModeContext->appStatus.vdmResp.doCount = 1u;
        ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;

#if MUX_DELAY_EN
        /* Save pointer to VDM response handler and set appropriate flag */
        ptrAltModeContext->appStatus.is_vdm_pending = true;
        ptrAltModeContext->appStatus.vdm_resp_cbk = vdm_resp_handler;
#endif /* MUX_DELAY_EN */

        if ((context->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) || (pd3_live))
        {
            /* VDM Commands (D_ID -- EXIT_MODE) should be NAKd if VDO Count in VDM
             * command is more than one. */
            if (vdm->len == 1u)
            {
                switch(vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd)
                {
                    case CY_PDSTACK_VDM_CMD_DSC_IDENTITY:
                        count = ptrAltModeContext->vdmIdVdoCnt;
                        if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == STD_SVID) && (count != 0u))
                        {
                            ptrAltModeContext->appStatus.vdmResp.doCount = count;
                            dobj = ptrAltModeContext->vdmIdVdoP;
                            for(i = 0 ; i < count; i++)
                            {
                                ptrAltModeContext->appStatus.vdmResp.respBuf[i] = dobj[i];
                            }


                            /*
                             * For multiport CCG3PA/CCG7S, which port is master is evaluated at runtime.
                             * Since port number 0 is dedicated for load sharing master,
                             * overwrite the port number in the configuration table.
                             *
                             * Based on current implementation, it is assumed that there is only
                             * one product type VDO and that is DFP VDO.
                             */
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S))
                            vdm_disc_id_resp_append_port_num(context);
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG7S) */

#if CY_PD_REV3_ENABLE
                            /* Mask Product Type (DFP) field when VDM version is 1.0. */
                            if(ptrAltModeContext->appStatus.vdm_version == 0u)
                            {
                                cy_pd_pd_do_t id_hdr = ptrAltModeContext->appStatus.vdmResp.respBuf[VDO_START_IDX];
                                uint8_t max_do_cnt = 4;

                                /* Make sure to clear fields that are reserved under PD 2.0. */
                                id_hdr.std_id_hdr.prodTypeDfp = 0;
                                id_hdr.std_id_hdr.connType = 0;

                                /* Make sure to not use invalid product types. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_PSD)
                                {
                                    id_hdr.std_id_hdr.prodType = CY_PDSTACK_PROD_TYPE_UNDEF;
                                }

                                /* AMAs may be providing one extra VDO. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_AMA)
                                {
                                    max_do_cnt++;
                                }

                                /* Ensure that the size of the response is limited to what is valid based
                                 * on the ID header. */
                                if(ptrAltModeContext->appStatus.vdmResp.doCount > max_do_cnt)
                                {
                                    ptrAltModeContext->appStatus.vdmResp.doCount = max_do_cnt;
                                }
                                ptrAltModeContext->appStatus.vdmResp.respBuf[VDO_START_IDX] = id_hdr;
                            }
#else /* !CY_PD_REV3_ENABLE */
                            app_stat->vdmResp.respBuf[VDO_START_IDX].std_id_hdr.rsvd1 = 0;
#endif /* CY_PD_REV3_ENABLE */

                            /* Set VDM Response ACKed */
                            ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

#if (!CCG_BACKUP_FIRMWARE)
                    case CY_PDSTACK_VDM_CMD_DSC_SVIDS:
                        count = ptrAltModeContext->vdmSvidVdoCnt;
                        if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == STD_SVID) && (count != 0u))
                        {
                            ptrAltModeContext->appStatus.vdmResp.doCount = count;
                            dobj = ptrAltModeContext->vdmSvidVdoP;
                            for(i = 0 ; i < count; i++)
                            {
                                ptrAltModeContext->appStatus.vdmResp.respBuf[i] = dobj[i];
                            }
                            /* Set VDM Response ACKed */
                            ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_DSC_MODES:
                        eval_rslt = get_modes_vdo_info (context, vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid,
                            &dobj, &count);
                        if (eval_rslt == true)
                        {
                            ptrAltModeContext->appStatus.vdmResp.doCount = count;
                            for (i = 0; i < count; i++)
                            {
                                ptrAltModeContext->appStatus.vdmResp.respBuf[i] = dobj[i];
                            }
#if DP_GPIO_CONFIG_SELECT
                            /*
                             * For GPIO based DP Pin configuration selection, update the
                             * response based on GPIO status.
                             */
                            if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid == DP_SVID)
                            {
                                ptrAltModeContext->appStatus.vdmResp.respBuf[1].std_dp_vdo.dfpDPin = dp_sink_get_pin_config ();
                            }
#endif /* DP_GPIO_CONFIG_SELECT */

#if ((TBT_UFP_SUPP) && (VPRO_DOCK_SUPPORT_ENABLE))
                            if (
                                    (PD_GET_PTR_TBTHOST_CFG_TBL(port)->vpro_capable)
                               )
                            {
                                /* Set the VPro available bit in the Mode VDO if enabled in config table. */
                                if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid == INTEL_VID)
                                {
                                    /* Set the VPro enable bit in the Mode VDO. */
                                    ptrAltModeContext->appStatus.vdmResp.respBuf[1].val |= TBT_MODE_VPRO_AVAIL;
                                }
                            }
#endif /* ((TBT_UFP_SUPP) && (VPRO_DOCK_SUPPORT_ENABLE)) */

                            /* Set VDM Response ACKed */
                            ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_ENTER_MODE:
#if CCG_HPI_PD_ENABLE
                        if (hpi_is_vdm_ec_ctrl_enabled (context))
                        {
                            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
                        }
#if (AUVDM_SUPPORT != 0)
                        else
                        {
                            if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == APPLE_SVID)
                                    && (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.objPos == APPLE_MODE_1))
                            {
                                ap_mode_status.ap_mode_pos1_entered = true;
                                app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;

                                /* Event to notify AP Mode status change*/
                                app_event_handler(port, APP_EVT_CUST_ALT_MODE_CHANGED, NULL);
#if CCG_HPI_PD_ENABLE
                                /* Update Alt mode status register */
                                hpi_set_ap_mode_status(port);
#endif /* CCG_HPI_PD_ENABLE */
                            }
                            else
                            {
                                ap_mode_status.ap_mode_pos1_entered = false;
                                app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                            }
                        }
#endif /* (AUVDM_SUPPORT != 0) */
#endif /* CCG_HPI_PD_ENABLE */
                        break;

                    case CY_PDSTACK_VDM_CMD_EXIT_MODE:
#if CCG_HPI_PD_ENABLE
                        if (hpi_is_vdm_ec_ctrl_enabled (context))
                        {
                            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
                        }
#if (AUVDM_SUPPORT != 0)
                        else
                        {

                            if((vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid == APPLE_SVID)
                                && (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.objPos == APPLE_MODE_1))
                            {
                                if(is_ap_mode_entered(port) != false)
                                {
                                    ap_mode_status.ap_mode_pos1_entered = false;

                                    app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
#if CCG_HPI_PD_ENABLE
                                    /* Update Alt mode status register */
                                    hpi_ap_mode_status_exit_mode (port);
#endif
                                    app_event_handler(port, APP_EVT_CUST_ALT_MODE_CHANGED, NULL);

                                    /* Stop apple mode attention timer */
                                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Stop)(port, ATTENTION_TIMER);
                               }
                                else
                                {
                                    app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                                }
                            }
                            else if ((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == APPLE_SVID)
                                    && (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.objPos == ALL_MODE_INDEX))
                            {
                                if (is_ap_mode_entered(port))
                                {
                                    ap_mode_status.ap_mode_pos1_entered = false;
                                    ap_mode_init(port);
#if CCG_HPI_PD_ENABLE
                                    /* Update Apple mode status register */
                                    hpi_clear_ap_mode_status(port);
#endif
                                    app_event_handler(port, APP_EVT_CUST_ALT_MODE_CHANGED, NULL);

                                    /* Updated response for Exit in case of valid exit command. */
                                    app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;

                                    /* Stop apple mode attention timer */
                                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Stop)(port, ATTENTION_TIMER);
                                }
                                else
                                {
                                    app_stat->vdmResp.respBuf[VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                                }
                            }
                        }
#endif /* (AUVDM_SUPPORT != 0) */
#endif /* CCG_HPI_PD_ENABLE */
                        break;
#endif /* (!CCG_BACKUP_FIRMWARE) */

                    case CY_PDSTACK_VDM_CMD_ATTENTION:
                        /* Ignore Attention VDM */
                        ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
                        break;

                    default:
                        break;
                }
            }

            /* Under PD2, handle VDMs based on VDM manager functionality. */
            if (!pd3_live)
            {
                if (
                        (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid != STD_SVID) &&
                        (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd != (uint8_t)CY_PDSTACK_VDM_CMD_ATTENTION)
                   )
                {
#if UFP_ALT_MODE_SUPP
                    /* If DFP cmd processed success */
                    if (Cy_PdAltMode_Mngr_EvalRecVdm((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext), vdm))
                    {
                        /* Set VDM Response ACKed */
                        ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                    }
                    else
#endif /* UFP_ALT_MODE_SUPP */
                    {
#if ((CCG_HPI_PD_ENABLE) && (!CCG_HPI_PD_CMD_DISABLE))
                        if (hpi_is_vdm_ec_ctrl_enabled(context))
                        {
                            /* Let the EC respond if EC VDM response bit is set. */
                            ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
                        }
#endif /* ((CCG_HPI_PD_ENABLE) && (!CCG_HPI_PD_CMD_DISABLE)) */
                    }
                }
            }
#if CY_PD_REV3_ENABLE
            else
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                if (context->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
                {
                    if ((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ENTER_MODE) ||
                            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_EXIT_MODE))
                    {
                        ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_READY;
                    }
                    else
                    {
                        if (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION)
                        {
                            ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
                        }
                        Cy_PdAltMode_Mngr_EvalRecVdm((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext), vdm);
                    }
                }
                else
                {
                    /* If received cmd processed success */
                    if (Cy_PdAltMode_Mngr_EvalRecVdm((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext), vdm) == true)
                    {
                        /* Set VDM Response ACKed */
                        ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                    }
#if CCG_HPI_PD_ENABLE
                    else
                    {
                        /* Let the EC respond to unknown VDMs when enabled. */
                        if (
                                (hpi_is_vdm_ec_ctrl_enabled(context)) &&
                                (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid != STD_SVID)
                           )
                        {
                            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
                        }
                    }
#endif /* CCG_HPI_PD_ENABLE */
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            }
#endif /* CY_PD_REV3_ENABLE */
        }
        else
        {
#if DFP_ALT_MODE_SUPP
            if ((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION) && (app_stat->vdm_task_en == true))
            {
                /* evaluate attention VDM */
                Cy_PdAltMode_Mngr_EvalRecVdm((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext), vdm);
            }
#endif /* DFP_ALT_MODE_SUPP */
            if(vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd!=CY_PDSTACK_VDM_CMD_DSC_IDENTITY)
            {
                /* No response to VDMs received on PD 2.0 connection except Discover Identity command while in DFP state. */
                ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
            }
        }

        /* Set the VDM version for the response. */
        ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer = ptrAltModeContext->appStatus.vdm_version;
    }
    else
    {
#if (FLASHING_MODE_PD_ENABLE == 1)
        /* Pass the VDM to U_VDM handler */
        /* QAC suppression 0310: &vdm->hdr points to uint32_t and is expected to be 4 byte aligned */
        /* QAC suppression 0311: vdm->hdr is used only for readonly purposes inside the function. */
        response_state = uvdm_handle_cmd(context, (uint32_t *)(&vdm->hdr), &dobj, &count,  /* PRQA S 0310, 0311 */
            uvdm_nb_flash_write_cb);
        /* Response is ready. Notify PD. */
        if (response_state == UVDM_HANDLED_RESPONSE_READY)
        {
            app_stat->vdmResp.doCount = count;
            for (i = 0; i < count; i++)
            {
                app_stat->vdmResp.respBuf[i] = dobj[i];
            }
        }
        /* No response to UVDM. */
        else if (response_state == UVDM_HANDLED_NO_RESPONSE)
        {
           /* Ignore Unstructured VDM response. */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
        }
        /* UVDM not recognized by default UVDM handler. */
        else if (response_state == UVDM_NOT_HANDLED)
        {
            /*
             * This is the default implementation to handle UVDM commands which are
             * not recognized by CY UVDM handler. User can update this with custom
             * handlers.
             */

            /* If SVID is Flashing VID, respond with NAK. */
            if (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid ==
                    pd_get_ptr_app_tbl(context->ptrUsbPdContext)->flashing_vid)
            {
                ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].val =
                    vdm->dat[CY_PD_VDM_HEADER_IDX].val;
                ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].ustd_vdm_hdr.cmdType
                    = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.doCount = 1;
            }
            else
            {
                /* Invoke non-CY UVDM handlers here. */
#if QC_PPS_ENABLE
                /* QC 5.0/4.0 UVDM handling. */
                /* QAC suppression 0310: &vdm->hdr points to uint32_t and is expected to be 4 byte aligned */
                /* QAC suppression 0311: vdm->hdr is used only for readonly purposes inside the function. */
                response_state = uvdm_qc_pps_handler(context, (uint32_t *)(&vdm->hdr), /* PRQA S 0310, 0311 */
                    &dobj, &count);
                if (response_state == UVDM_HANDLED_RESPONSE_READY)
                {
                    ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.doCount = count;
                    for (i = 0; i < count; i++)
                    {
                        ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.respBuf[i] = dobj[i];
                    }
                }
                /* No response to UVDM. */
                else if (response_state == UVDM_HANDLED_NO_RESPONSE)
                {
                   /* Ignore Unstructured VDM response. */
                    ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
                }
#endif /* QC_PPS_ENABLE */

                if (response_state == UVDM_NOT_HANDLED)
                {

                    /*
                     * SVID is not Recognized. Ignore if PD contract is PD 2.0.
                     * Send Not_Supported if Contract is PD 3.0.
                     */

                    /* No VDM response required. */
                    ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
#if CY_PD_REV3_ENABLE
                    if (pd3_live)
                    {
                        /* PD 3.0 Contract: Respond with Not_Supported. */
                        ((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_SUPP;
                    }
#endif /* CY_PD_REV3_ENABLE */
                }
            }
        }
        /* UVDM handled but response will be sent later. */
        else
        {
            /* Ignore. */
            return;
        }
#else /* (FLASHING_MODE_PD_ENABLE != 1) */

#if UVDM_SUPP
        if (Cy_PdAltMode_Mngr_EvalRecVdm((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext), vdm) == false)
        {
            /* If UVDM not successful then ignore this UVDM */
            app_stat->vdmResp.noResp = VDM_AMS_RESP_FROM_EC;
        }
#else /* !UVDM_SUPP */
        if (
                (!pd3_live)
#if CCG_HPI_PD_ENABLE
                || (hpi_is_vdm_ec_ctrl_enabled(context))
#endif /* CCG_HPI_PD_ENABLE */
           )
        {
            /* If UVDM not successful then ignore this UVDM */
            ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
        }
        else
        {
            /* PD 3.0 Contract: Respond with Not_Supported. */
            ptrAltModeContext->appStatus.vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_SUPP;
        }
#endif /* UVDM_SUPP */
#endif /* FLASHING_MODE_PD_ENABLE */
    }

#if MUX_DELAY_EN
    /* Send VDM response only when MUX is idle or alt mode response is NACK */
    if (
            (ptrAltModeContext->appStatus.vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType != CY_PDSTACK_CMD_TYPE_RESP_ACK) ||
            (ptrAltModeContext->appStatus.is_mux_busy == false)
       )
    {
        vdm_resp_handler(ptrAltModeContext->pdStackContext, &ptrAltModeContext->appStatus.vdmResp);
        ptrAltModeContext->appStatus.is_vdm_pending = false;
    }
#else
    vdm_resp_handler(context, &(((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.vdmResp));
#endif /* MUX_DELAY_EN */
}

#if CCG_USB4_SUPPORT_ENABLE
/*
 * Follow "Valid Responses to Enter_USB Request" table for PD spec addendum.
 * This function assumes that the UFP VDO1 is always present as the first product type VDO in the
 * Discover_Identity response, for feature matching purposes.
 */
void eval_enter_usb(uint8_t port, const pd_packet_t *eudo_p, app_resp_cbk_t app_resp_handler)
{
    /* Basic validity checks including message length would have been done by the PD stack. */
    pd_do_t eudo    = eudo_p->dat[0];
    pd_do_t ufp_vdo = gl_vdm_id_vdo_p[port][PRODUCT_TYPE_VDO_1_IDX];

    /* Default response: REJECT. */
    app_get_resp_buf(port)->reqStatus = REQ_REJECT;

    /* Match the configuration from the Enter USB DO against the UFP VDO1. */
    if (gl_vdm_id_vdo_cnt[port] > PRODUCT_TYPE_VDO_1_IDX)
    {
        if ((eudo.enterusb_vdo.usbMode == CCG_USB_MODE_USB4) && (ufp_vdo.ufp_vdo_1.devCap & DEV_CAP_USB_4_0))
        {
            app_get_resp_buf(port)->reqStatus = REQ_ACCEPT;
            usb4_update_data_status (port, eudo, 0x00);
        }
    }

    app_resp_handler(port, app_get_resp_buf(port));
}
#endif /* CCG_USB4_SUPPORT_ENABLE */

/* End of File */
