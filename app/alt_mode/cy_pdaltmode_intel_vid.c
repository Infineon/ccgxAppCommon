/******************************************************************************
* File Name:   cy_pdaltmode_intel_vid.c
* \version 2.0
*
* Description: Intel Thunderbolt (VID) control Interface source file.
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

#include "cy_pdaltmode_intel_vid.h"
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_intel_ridge.h"
#include "cy_pdaltmode_intel_ridge_internal.h"
#include "cy_pdaltmode_ridge_slave.h"
#include "cy_pdaltmode_vdm_task.h"

#if (TBT_DFP_SUPP || TBT_UFP_SUPP)

#if TBT_DFP_SUPP
/* Analysis TBT DISC MODE info for DFP */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegDfp(cy_stc_pdaltmode_context_t *ptrPdStackContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg);

/* Analysis TBT DISC MODE info for the cable */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegCbl(cy_stc_pdaltmode_context_t *ptrPdStackContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg);

/* Generate enter mode VDO */
static void Cy_PdAltMode_TBT_GenerateEnterModeVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t svid_vdo, cy_pd_pd_do_t cable_vdo);

/* Composes VDM for sending by alt mode manager */
static void Cy_PdAltMode_TBT_SendCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Initializes TBT DFP alt mode */
static void Cy_PdAltMode_TBT_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Exits TBT alt mode */
static void Cy_PdAltMode_TBT_Exit(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Main TBT source handling functions */
static void Cy_PdAltMode_TBT_DfpRun(void *context);

/* Evaluates received TBT attention VDM */
#if ICL_ENABLE_WAIT_FOR_SOC
static void Cy_PdAltMode_TBT_EvalAttention(uint8_t port);    
#else    
static cy_en_pdaltmode_tbt_state_t Cy_PdAltMode_TBT_EvalAttention(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /* ICL_ENABLE_WAIT_FOR_SOC */

/* Stores given VDO in TBT VDO buffer for sending */
void Cy_PdAltMode_TBT_AssignVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t vdo);

/* Returns pointer to TBT VDO buffer */
cy_pd_pd_do_t* Cy_PdAltMode_TBT_GetVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Checks if cable supports TBT handling */
static bool Cy_PdAltMode_TBT_IsCableCapable(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);

/* Check if cable contains TBT SVID */
static bool Cy_PdAltMode_TBT_IsCableSvid(const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);
#endif /* TBT_DFP_SUPP */

#if TBT_UFP_SUPP
/* Main TBT sink handling functions */
static void Cy_PdAltMode_TBT_UfpRun(void *context);
#endif /* TBT_UFP_SUPP */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
/* Sets Ridge bits depending on current TBT state */
static bool Cy_PdAltMode_TBT_SetRegsEnter(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/* Evaluates command received from application */
static bool Cy_PdAltMode_TBT_EvalAppCmd(void *context, cy_stc_pdaltmode_alt_mode_evt_t cmd_data);

/* Returns pointer to alt mode info structure */
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_Info(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /*  ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

/************************** Function definitions *****************************/

cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegIntelModes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;

    if(ptrAltModeContext->appStatus.usb4_active != false)
    {
        return NULL;
    }

#if TBT_UFP_SUPP
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);
#endif /* (TBT_UFP_SUPP) */

#if CY_PD_USB4_SUPPORT_ENABLE
    ptrAltModeContext->tbtStatus.vpro_supp = false;
    
    if (
             (ptrAltModeContext->appStatus.usb4_active != false) &&
             (ptrAltModeContext->appStatus.custom_hpi_host_cap_control & APP_HPI_VPRO_SUPP_MASK)
       )
    {
        cy_pd_pd_do_t *partner_d_id_resp;
        uint8_t resp_len;

        partner_d_id_resp = (cy_pd_pd_do_t *)Cy_PdAltMode_VdmTask_GetDiscIdResp(ptrAltModeContext, &resp_len);

        if (
                (reg_info->data_role == CY_PD_PRT_TYPE_DFP)               &&
                (reg_info->svid_vdo.tbt_ufp_vdo.vproSupp != false) &&
                (partner_d_id_resp != NULL)                         &&
                ((partner_d_id_resp[CY_PD_PRODUCT_TYPE_VDO_1_IDX].ufp_vdo_1.altModes & CY_PD_UFP_NON_PH_ALT_MODE_SUPP_MASK) != false) &&
                (ptrAltModeContext->appStatus.custom_hpi_host_cap_control & APP_HPI_VPRO_SUPP_MASK)
            )
        {
            /* Vpro supported */
            ptrAltModeContext->tbtStatus.vpro_supp = true;
        }    
        else
        {
            return NULL;
        }
    }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
    
    if (reg_info->svid_vdo.tbt_vdo.intelMode == TBT_ALT_MODE_ID)
    {
#if TBT_UFP_SUPP
        /* If TBT sink */
        if (reg_info->data_role == CY_PD_PRT_TYPE_UFP)
        {
            /* Reset TBT info struct */
            Cy_PdAltMode_Mngr_ResetAltModeInfo(&(ptrAltModeContext->tbtStatus.info));
            reg_info->alt_mode_id    = TBT_ALT_MODE_ID;
            info->cbk                = Cy_PdAltMode_TBT_UfpRun;
            info->vdm_header.std_vdm_hdr.svid = INTEL_VID;
            info->vdo[CY_PD_SOP]     = ptrAltModeContext->tbtStatus.vdo;
            info->vdo_max_numb       = MAX_TBT_VDO_NUMB;
#if (!ICL_ALT_MODE_HPI_DISABLED)
            reg_info->app_evt        = AM_EVT_ALT_MODE_SUPP;
            info->eval_app_cmd       = Cy_PdAltMode_TBT_EvalAppCmd;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

            return info;
        }
#endif /* TBT_UFP_SUPP */

#if TBT_DFP_SUPP
        /* If TBT source */
        if (reg_info->atch_type == ATCH_TGT)
        {
#if STORE_DETAILS_OF_HOST
            cy_stc_pdaltmode_context_t *deviceContext =
                    (cy_stc_pdaltmode_context_t *)ptrAltModeContext->hostDetails.dsAltModeContext;

            if((deviceContext->hostDetails.ds_mode_mask & TBT_MODE_DFP) == 0)
            {
                return NULL;
            }
#endif /* Cy_PdAltMode_HostDetails_GetDsModeMask */
            /* Analyze DFP TBT Disc mode response */
            return Cy_PdAltMode_TBT_RegDfp(ptrAltModeContext, reg_info);
        }
        else if (reg_info->atch_type == CABLE)
        {
            /* Analyze TBT cable Disc mode response */
            return Cy_PdAltMode_TBT_RegCbl(ptrAltModeContext, reg_info);
        }
#endif /* TBT_DFP_SUPP */
    }

    reg_info->alt_mode_id = MODE_NOT_SUPPORTED;
    return NULL;
}

/* Save discover mode response from device */
cy_pd_pd_do_t Cy_PdAltMode_TBT_SvidDiscModefromUfp[NO_OF_TYPEC_PORTS] = {0};

#if TBT_DFP_SUPP
static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegDfp(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg)
{
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    /* Reset TBT info struct */
    Cy_PdAltMode_Mngr_ResetAltModeInfo(&(ptrAltModeContext->tbtStatus.info));

    if (ptrAltModeContext->tbtStatus.vpro_supp == false)
    {
        /* Check if cable discovered and if active cable has no fixed SS lines */
        if (Cy_PdAltMode_TBT_IsCableCapable(ptrAltModeContext, reg->atch_tgt_info) == false)
        {
#if (!ICL_ALT_MODE_HPI_DISABLED)
            /* Set application event */
            reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
            return NULL;
        }

        /* Check if cable has INTEL SVID */
        if (Cy_PdAltMode_TBT_IsCableSvid(reg->atch_tgt_info) == false)
        {
            /* SOP' disc svid is not needed */
            reg->cbl_sop_flag = CY_PD_SOP_INVALID;
#if AMD_SUPP_ENABLE
            /* Set mux to Isolate */
            Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_ISOLATE, CY_PDALTMODE_NO_DATA);
#endif /* AMD_SUPP_ENABLE */
#if (!ICL_ALT_MODE_HPI_DISABLED)
            /* Set application event */
            reg->app_evt = AM_EVT_ALT_MODE_SUPP;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
        }
        else
        {
            /* SOP' disc svid  needed */
            reg->cbl_sop_flag = CY_PD_SOP_PRIME;
        }
        /* Generate TBT enter mode VDO based on disc mode resp */
        Cy_PdAltMode_TBT_GenerateEnterModeVdo(ptrAltModeContext, reg->svid_vdo, reg->svid_emca_vdo);
    }
    else
    {
        ptrAltModeContext->tbtStatus.enter_mode_vdo.val = VPRO_ALT_MODE_ID;
        ptrAltModeContext->tbtStatus.enter_mode_vdo.tbt_vdo.vproDockHost = true;

        /* SOP' disc svid is not needed */
        reg->cbl_sop_flag = CY_PD_SOP_INVALID;
#if (!ICL_ALT_MODE_HPI_DISABLED)
        /* Set application event */
        reg->app_evt = AM_EVT_ALT_MODE_SUPP;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
    }

    reg->alt_mode_id = TBT_ALT_MODE_ID;

    /* Copy cable, device/AMA info pointer */
    ptrAltModeContext->tbtStatus.tgt_info_ptr = reg->atch_tgt_info;
    ptrAltModeContext->tbtStatus.max_sop_supp = CY_PD_SOP;
#if CY_PD_USB4_SUPPORT_ENABLE
    ptrAltModeContext->tbtStatus.dev_vdo      = reg->svid_vdo;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    /* Init TBT */
    info->mode_state   = ALT_MODE_STATE_INIT;
    info->vdo_max_numb = MAX_TBT_VDO_NUMB;
    Cy_PdAltMode_TBT_DfpRun(ptrAltModeContext);

    return info;
}

static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegCbl(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_alt_mode_reg_info_t *reg)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    if (reg->cbl_sop_flag != CY_PD_SOP_INVALID)
    {
        /* Save TBT cable response */
        ptrAltModeContext->appStatus.tbtCblVdo.val = reg->svid_emca_vdo.val;

        /* Generate TBT enter mode VDO based on disc mode resp */
        Cy_PdAltMode_TBT_GenerateEnterModeVdo(ptrAltModeContext, reg->svid_vdo, reg->svid_emca_vdo);

        /* SOP' VDM needed */
        ptrAltModeContext->tbtStatus.max_sop_supp                       = CY_PD_SOP_PRIME;

        /* Allow send SOP' packets */
        info->sop_state[CY_PD_SOP_PRIME]         = ALT_MODE_STATE_SEND;

        /* If SOP'' controller present (active cables only) allow SOP'' VDM */
        if ((ptrAltModeContext->tbtStatus.tgt_info_ptr->cblVdo->val != CY_PDALTMODE_NO_DATA) &&
                (ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) &&
                (ptrAltModeContext->tbtStatus.tgt_info_ptr->cblVdo->std_cbl_vdo.sopDp == 1u))
        {
            info->sop_state[CY_PD_SOP_DPRIME] = ALT_MODE_STATE_SEND;
            ptrAltModeContext->tbtStatus.max_sop_supp                = CY_PD_SOP_DPRIME;
        }

        /* Init TBT */
        info->mode_state = ALT_MODE_STATE_INIT;
        Cy_PdAltMode_TBT_DfpRun(ptrAltModeContext);
    }
    else
    {
        /* If cable is active */
        if ((ptrAltModeContext->tbtStatus.tgt_info_ptr->cblVdo->val != CY_PDALTMODE_NO_DATA) &&
                (ptrAltModeContext->tbtStatus.tgt_info_ptr->cblVdo->std_cbl_vdo.cblTerm >=
                 CY_PDSTACK_CBL_TERM_ONE_ACT_ONE_PAS_VCONN_REQ))
        {
#if (!ICL_ALT_MODE_HPI_DISABLED)
            /* Set application event */
            reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
            /* Disable TBT functionality */
            info->mode_state = ALT_MODE_STATE_DISABLE;

            return NULL;
        }
    }

    /* Return SOP'' disc svid not needed */
    reg->cbl_sop_flag = CY_PD_SOP_INVALID;
#if (!ICL_ALT_MODE_HPI_DISABLED)
    /* Set application event */
    reg->app_evt = AM_EVT_ALT_MODE_SUPP;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */
#if AMD_SUPP_ENABLE
/* Set mux to Isolate */
    Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_ISOLATE, CY_PDALTMODE_NO_DATA);
#endif /* AMD_SUPP_ENABLE */

    return info;
}

static void Cy_PdAltMode_TBT_DfpRun(void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    switch (info->mode_state)
    {
        case ALT_MODE_STATE_INIT:
            Cy_PdAltMode_TBT_Init(ptrAltModeContext);
            Cy_PdAltMode_TBT_SendCmd(ptrAltModeContext);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            ptrAltModeContext->tbtStatus.state = (cy_en_pdaltmode_tbt_state_t)info->vdm_header.std_vdm_hdr.cmd;
            info->mode_state = ALT_MODE_STATE_IDLE;
            switch (ptrAltModeContext->tbtStatus.state)
            {
                case TBT_STATE_ENTER:
                    if (ptrAltModeContext->tbtStatus.vpro_supp != false)
                    {
#if RIDGE_SLAVE_ENABLE
                        /* Set vPro mode register */
                        Cy_PdAltMode_Ridge_SetVpro(ptrAltModeContext, true);
#endif /* RIDGE_SLAVE_ENABLE */
                    }
                    else
                    {
                        Cy_PdAltMode_TBT_SetRegsEnter(ptrAltModeContext);
                    }
                    ptrAltModeContext->tbtStatus.state = TBT_STATE_IDLE;
                    break;

                case TBT_STATE_EXIT:
                    Cy_PdAltMode_TBT_Exit(ptrAltModeContext);
                    break;

                default:
                    break;
            }

#if AM_PD3_FLOW_CTRL_EN
            /* Make sure Rp is changed back to SinkTxOK. */
            Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, false);
#endif /* AM_PD3_FLOW_CTRL_EN */
            return;

        case ALT_MODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((cy_en_pdaltmode_tbt_state_t)info->vdm_header.std_vdm_hdr.cmd == TBT_STATE_ATT)
            {
#if ICL_ENABLE_WAIT_FOR_SOC
                if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
                {
                    Cy_PdAltMode_TBT_EvalAttention(port);
                }
#else
                ptrAltModeContext->tbtStatus.state = Cy_PdAltMode_TBT_EvalAttention(ptrAltModeContext);
#endif /* ICL_ENABLE_WAIT_FOR_SOC */
                Cy_PdAltMode_TBT_SendCmd(ptrAltModeContext);
            }
            break;

        case ALT_MODE_STATE_FAIL:
            switch (ptrAltModeContext->tbtStatus.state)
            {
                case TBT_STATE_ENTER:
                    /* Cbl enter was done */
                    if ((info->sop_state[CY_PD_SOP] == ALT_MODE_STATE_FAIL) &&
                            (ptrAltModeContext->tbtStatus.max_sop_supp != CY_PD_SOP))
                    {
                        ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
                        Cy_PdAltMode_TBT_SendCmd(ptrAltModeContext);
                        info->sop_state[CY_PD_SOP] = ALT_MODE_STATE_DISABLE;
                    }
                    else
                    {
                        /* Check if cable reset is required */
                        if (
                                (ptrAltModeContext->appStatus.cbl_rst_done != false) &&
                                ((info->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_FAIL)  ||
                                 (info->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_FAIL))
                           )
                        {
                            ptrAltModeContext->appStatus.cbl_rst_done = true;
                            ptrAltModeContext->appStatus.trig_cbl_rst = true;
                        }                        
                        Cy_PdAltMode_TBT_Exit(ptrAltModeContext);
                    }
                    break;

                case TBT_STATE_EXIT:
#if 0 /*Refer Jira CCG_H3_4-107*/
                    if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
                    {
                        /* If SOP' exit failed, try SOP'' and SOP exits */
                        if (info->sop_state[CY_PD_SOP_PRIME] == ALT_MODE_STATE_FAIL)
                        {
                            ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
                            Cy_PdAltMode_TBT_SendCmd(port);
                            info->sop_state[CY_PD_SOP_PRIME] = ALT_MODE_STATE_FAIL;
                            return;
                        }
                        
                        /* If SOP'' exit failed, try SOP exit */
                        if (info->sop_state[CY_PD_SOP_DPRIME] == ALT_MODE_STATE_FAIL)
                        {
                            ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
                            Cy_PdAltMode_TBT_SendCmd(port);
                            info->sop_state[CY_PD_SOP_PRIME] = ALT_MODE_STATE_FAIL;
                            info->sop_state[CY_PD_SOP_DPRIME] = ALT_MODE_STATE_FAIL;
                            return;
                        }
                    }
#endif /* 0 */
                    Cy_PdAltMode_TBT_Exit(ptrAltModeContext);
                    break;

                default:
                    break;
            }

#if AM_PD3_FLOW_CTRL_EN
            /* Make sure Rp is changed back to SinkTxOK. */
            Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, false);
#endif /* AM_PD3_FLOW_CTRL_EN */
            break;

        case ALT_MODE_STATE_EXIT:
            ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
            Cy_PdAltMode_TBT_SendCmd(ptrAltModeContext);
            break;

        default:
            break;
    }
}

#if CY_PD_USB4_SUPPORT_ENABLE
static void Cy_PdAltMode_TBT_GenerateEnterModeVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t svid_vdo, cy_pd_pd_do_t cable_vdo)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_pd_pd_do_t emdo;

    emdo.val                = CY_PDALTMODE_NO_DATA;
    emdo.tbt_vdo.intelMode  = TBT_ALT_MODE_ID;
    emdo.tbt_vdo.adapter    = GET_LEGACY_TBT_ADAPTER(svid_vdo.val);
    emdo.tbt_vdo.rsvd2      = svid_vdo.tbt_vdo.rsvd2;


    if (ptrAltModeContext->tbtCfg->vproCapable)
    {
        /* If VPro support is enabled and the port partner supports it, enable the feature. */
        if (GET_VPRO_AVAILABLE_STAT(svid_vdo.val))
        {
            emdo.tbt_vdo.vproDockHost = 1;
        }
    }

    if (cable_vdo.val == CY_PDALTMODE_NO_DATA)
    {
        /*
         * If cable does not provide a MODE VDO, we have to fill the parameters
         * with fields from the cable D_ID response. In case cable discovery is
         * turned off, indicate a simple 3.0 EMCA passive cable.
         */
        if(ptrPdStackContext->dpmConfig.cblDsc != false)
        {
            emdo.tbt_vdo.cblSpeed = ptrPdStackContext->dpmStat.cblVdo.std_cbl_vdo.usbSsSup;
        }
        else
        {
            /*
             * Since the cable discovery is turned off, we have to assume that
             * the product has a type-C plug and the cable supports all the
             * characteristics supported by the product. Indicate highest value.
             */
            emdo.tbt_vdo.cblSpeed = CY_PDSTACK_USB_GEN_2_SUPP;
        }
    }
    else
    {
        
        emdo.tbt_vdo.cblSpeed       = cable_vdo.tbt_vdo.cblSpeed;
        emdo.tbt_vdo.cblGen         = cable_vdo.tbt_vdo.cblGen;     /*  Max allowed cable gen as per spec is 01 */
        emdo.tbt_vdo.cblType        = cable_vdo.tbt_vdo.cblType;
        emdo.tbt_vdo.b22RetimerCbl = cable_vdo.tbt_vdo.b22RetimerCbl;
        emdo.tbt_vdo.linkTraining   = cable_vdo.tbt_vdo.linkTraining;
        emdo.tbt_vdo.cableActive    = cable_vdo.tbt_vdo.cableActive;
#if 0
        /* 
         * Special handling for LRD cable - bit#24 in TBT disc mode will be set to 1 by this cable & bit#22 
         * (retimer/redriver) will be set to redriver
         */
        if((cable_vdo.tbt_vdo.cableActive != false) && (cable_vdo.tbt_vdo.b22RetimerCbl == false))
        {
            emdo.tbt_vdo.cableActive = true;
        }
#endif
    }

    ptrAltModeContext->tbtStatus.enter_mode_vdo = emdo;
}
#else
static void Cy_PdAltMode_TBT_GenerateEnterModeVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t svid_vdo, cy_pd_pd_do_t cable_vdo)
{
    cy_pd_pd_do_t emdo;

    emdo.val                = CY_PDALTMODE_NO_DATA;
    emdo.tbt_vdo.intelMode = TBT_ALT_MODE_ID;
    emdo.tbt_vdo.adapter   = GET_LEGACY_TBT_ADAPTER(svid_vdo.val);

#if VPRO_WITH_USB4_MODE
    if (ptrAltModeContext->tbtCfg->vproCapable)
    {
        /* If VPro support is enabled and the port partner supports it, enable the feature. */
        if (GET_VPRO_AVAILABLE_STAT(svid_vdo.val))
        {
            emdo.tbt_vdo.vproDockHost = 1u;
        }
    }
#endif

    if (cable_vdo.val == CY_PDALTMODE_NO_DATA)
    {
        /*
         * If cable does not provide a MODE VDO, we have to fill the parameters
         * with fields from the cable D_ID response. In case cable discovery is
         * turned off, indicate a simple 3.0 EMCA passive cable.
         */
         if(ptrAltModeContext->pdStackContext->dpmConfig.cblDsc != false)
        {
              emdo.tbt_vdo.cblSpeed = Cy_PdAltMode_Mngr_GetCableUsbCap(ptrAltModeContext->pdStackContext);
              emdo.tbt_vdo.cableActive = (ptrAltModeContext->pdStackContext->dpmStat.cblVdo.std_cbl_vdo.cblTerm >> 1);
        }
        else
        {
            /*
             * Since the cable discovery is turned off, we have to assume that
             * the product has a type-C plug and the cable supports all the
             * characteristics supported by the product. Indicate highest value.
             */
            emdo.tbt_vdo.cblSpeed =  CY_PDSTACK_USB_GEN_2_SUPP;
        }
    }
    else
    {
        emdo.tbt_vdo.cblSpeed     = cable_vdo.tbt_vdo.cblSpeed;
        emdo.tbt_vdo.cblGen       = cable_vdo.tbt_vdo.cblGen;
        emdo.tbt_vdo.cblType      = cable_vdo.tbt_vdo.cblType;
        emdo.tbt_vdo.b22RetimerCbl  = cable_vdo.tbt_vdo.b22RetimerCbl;
        emdo.tbt_vdo.cableActive    = cable_vdo.tbt_vdo.cableActive;
        emdo.tbt_vdo.linkTraining = cable_vdo.tbt_vdo.linkTraining;
    }

    ptrAltModeContext->tbtStatus.enter_mode_vdo = emdo;
}    
#endif /* CY_PD_USB4_SUPPORT_ENABLE */


#if ((ICL_SLAVE_ENABLE) && (ICL_ENABLE_WAIT_FOR_SOC))
static void tbt_try_exit_mode(uint8_t port, timer_id_t id)
{
    /* If PMC updates are pending, then postpone exit until updates are done.
     * This is needed to prevent PMC from seeing out-of-sequence reads leading 
     * to a crash */
        
    if (ridge_update_is_pending())
    {
        timer_start(port, TBT_MODE_EXIT_CHECK_TIMER, TBT_MODE_EXIT_CHECK_PERIOD, tbt_try_exit_mode);
    }
    else
    {
        ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;

#if CCG_SYNC_ENABLE
        /* We exit mode since adapter requested it, most likely due to low-power advertisement.
         * Schedule re-entry of Thunderbolt mode once high power is available */
        ccg_sync_schedule_mode_reentry(port, INTEL_VID, TBT_ALT_MODE_ID);
#endif /* CCG_SYNC_ENABLE */

        Cy_PdAltMode_TBT_SendCmd(port);
    }
}
#endif /* ((ICL_SLAVE_ENABLE) && (ICL_ENABLE_WAIT_FOR_SOC)) */

#if ICL_ENABLE_WAIT_FOR_SOC
static void Cy_PdAltMode_TBT_EvalAttention(uint8_t port)
{
    pd_do_t *att = Cy_PdAltMode_TBT_GetVdo(port);
    cy_stc_pdaltmode_ridge_reg_t  reg  = ptrAltModeContext->tbtStatus.ridge_status;

    /* If Exit is needed */
    if (TBT_EXIT(att->val))
    {
#if ICL_SLAVE_ENABLE
        if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
        {
            tbt_try_exit_mode(port, TBT_MODE_EXIT_CHECK_TIMER);
        }
#else
        ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
        Cy_PdAltMode_TBT_SendCmd(port);
#endif /* ICL_SLAVE_ENABLE */
    }
    /* Check TBT mode data exit request */
    else if ((att->val & TBT_DATA_EXIT_MASK) == TBT_DATA_EXIT_MASK)
    {
        /* Disable TBT mode  */
        reg.ridge_stat.tbt_conn = false;

        /* Set MUX to USB mode */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, reg.val);
    }
    else if ((att->val & TBT_DATA_EXIT_MASK) == 0u)
    {
        /* Set MUX to TBT mode */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.ridge_status.val);
    }
}    
#else    
static cy_en_pdaltmode_tbt_state_t Cy_PdAltMode_TBT_EvalAttention(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdaltmode_tbt_state_t  stat = TBT_STATE_IDLE;
    cy_pd_pd_do_t     *att  = Cy_PdAltMode_TBT_GetVdo(ptrAltModeContext);
    cy_stc_pdaltmode_ridge_reg_t  reg  = ptrAltModeContext->tbtStatus.ridge_status;

    /* If Exit is needed */
    if (TBT_EXIT(att->val))
    {
        stat = TBT_STATE_EXIT;
    }
    /* Check TBT mode data exit request */
    else if ((att->val & TBT_DATA_EXIT_MASK) == TBT_DATA_EXIT_MASK)
    {
        /* Disable TBT mode  */
        reg.ridge_stat.tbt_conn = false;

        /* Set MUX to USB mode */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, reg.val);
    }
    else if ((att->val & TBT_DATA_EXIT_MASK) == 0u)
    {
        /* Set MUX to TBT mode */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.ridge_status.val);
    }

    return stat;
}
#endif /* ICL_ENABLE_WAIT_FOR_SOC */

static void Cy_PdAltMode_TBT_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);
    uint8_t idx;

    for (idx = 0; idx <= ptrAltModeContext->tbtStatus.max_sop_supp; idx++)
    {
        info->sop_state[idx] = ALT_MODE_STATE_SEND;
        info->vdo[idx]       = ptrAltModeContext->tbtStatus.vdo;

        /* TBT not uses VDO for cable while entered */
        if ((cy_en_pd_sop_t)idx > CY_PD_SOP)
        {
            info->vdo_numb[idx] = CY_PDALTMODE_NO_DATA;
        }
    }

    info->cbk = Cy_PdAltMode_TBT_DfpRun;

#if (!ICL_ALT_MODE_HPI_DISABLED)
    info->eval_app_cmd = Cy_PdAltMode_TBT_EvalAppCmd;
#endif /* (!ICL_ALT_MODE_HPI_DISABLED) */

    /* Not setting MUX into safe state for TBT mode as it is causing failures when using Active Cables. */
    info->set_mux_isolate = false;

#if ICL_ENABLE
    if (PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
    {
        if (ptrAltModeContext->tbtStatus.vpro_supp == false)
        {
            /* Ice Lake/Tiger Lake: Change setting to true */
            info->set_mux_isolate = true;
        }
    }
#endif /* ICL_ENABLE */ 

    /* Update the VDO to be sent for enter mode. */
    Cy_PdAltMode_TBT_AssignVdo(ptrAltModeContext, ptrAltModeContext->tbtStatus.enter_mode_vdo.val);

    /* Set TBT state as enter */
    ptrAltModeContext->tbtStatus.state = TBT_STATE_ENTER;

#if AM_PD3_FLOW_CTRL_EN
    /* Make sure Rp is changed to SinkTxNG if we are PD 3.0 source. */
    Cy_PdStack_Dpm_Pd3SrcRpFlowControl (ptrAltModeContext->pdStackContext, true);
#endif /* AM_PD3_FLOW_CTRL_EN */
}

#if STORE_DETAILS_OF_HOST
void Cy_PdAltMode_TBT_SendExitModeCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    Cy_PdAltMode_TBT_Exit(ptrAltModeContext);
    Cy_PdAltMode_TBT_DfpRun(ptrAltModeContext);
}
#endif /* STORE_DETAILS_OF_HOST */

static void Cy_PdAltMode_TBT_Exit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    ptrAltModeContext->tbtStatus.info.mode_state = ALT_MODE_STATE_EXIT;

#if RIDGE_SLAVE_ENABLE    
    if (ptrAltModeContext->tbtStatus.vpro_supp != false)
    {
        /* Disable vPro mode register */
        Cy_PdAltMode_Ridge_SetVpro(ptrAltModeContext, false);
        ptrAltModeContext->tbtStatus.vpro_supp = false;
    }
#endif /* RIDGE_SLAVE_ENABLE */    
}

static bool Cy_PdAltMode_TBT_IsCableCapable(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    /* Check for cable characteristics. */
    if (atch_tgt_info->cblVdo->val != CY_PDALTMODE_NO_DATA)
    {
        /* TBT mode can be entered if an active cable with TBT mode support reports only USB 2.0 support. */
        if (
                ((Cy_PdAltMode_Mngr_GetCableUsbCap(ptrAltModeContext->pdStackContext) == CY_PDSTACK_USB_2_0_SUPP)        ||
                (ptrAltModeContext->pdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)) &&
                (!Cy_PdAltMode_TBT_IsCableSvid(atch_tgt_info))
           )
        {
            return false;
        }

        return true;
    }

    return false;
}

static bool Cy_PdAltMode_TBT_IsCableSvid(const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    uint8_t idx = 0;

    while (atch_tgt_info->cbl_svid[idx] != 0)
    {
        /* If Cable Intel SVID found */
        if (atch_tgt_info->cbl_svid[idx] == INTEL_VID)
        {
            return true;
        }
        idx++;
    }

    return false;
}
#endif /* TBT_DFP_SUPP */

void Cy_PdAltMode_TBT_AssignVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t vdo)
{
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    /* No TBT VDOs needed while send VDM */
    if (vdo == NONE_VDO)
    {
        info->vdo_numb[CY_PD_SOP] = CY_PDALTMODE_NO_DATA;
    }
    else
    {
        /* Include given VDO as part of VDM */
        ptrAltModeContext->tbtStatus.vdo[TBT_VDO_IDX].val = vdo;
        info->vdo_numb[CY_PD_SOP] = MAX_TBT_VDO_NUMB;
    }
}

cy_pd_pd_do_t* Cy_PdAltMode_TBT_GetVdo(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &(ptrAltModeContext->tbtStatus.vdo[TBT_VDO_IDX]);
}

static cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_Info(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return &(ptrAltModeContext->tbtStatus.info);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static void Cy_PdAltMode_TBT_SendCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);
    cy_en_pdaltmode_mux_select_t cfg = MUX_CONFIG_ISOLATE;

    if (ptrAltModeContext->tbtStatus.state != TBT_STATE_IDLE)
    {
        cy_en_pd_sop_t sop_idx;
        info->vdm_header.val = CY_PDALTMODE_NO_DATA;
        info->vdm_header.std_vdm_hdr.cmd    = (uint32_t)ptrAltModeContext->tbtStatus.state;
        info->vdm_header.std_vdm_hdr.svid   = INTEL_VID;
        if (ptrAltModeContext->tbtStatus.state == TBT_STATE_EXIT)
        {
            if(ptrAltModeContext->pdStackContext->dpmConfig.attach != false)
            {
                /* Set Safe Mode before TBT exit. */
                cfg = MUX_CONFIG_SAFE;
            }
            Cy_PdAltMode_HW_SetMux(ptrAltModeContext, cfg, CY_PDALTMODE_NO_DATA);
            Cy_PdAltMode_TBT_AssignVdo(ptrAltModeContext, NONE_VDO);
        }
        for (sop_idx = CY_PD_SOP; sop_idx <= ptrAltModeContext->tbtStatus.max_sop_supp; sop_idx++)
        {
            info->sop_state[sop_idx] = ALT_MODE_STATE_SEND;
        }
        info->mode_state = ALT_MODE_STATE_SEND;
    }
}

#if CY_PD_USB4_SUPPORT_ENABLE
static bool Cy_PdAltMode_TBT_SetRegsEnter(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
#if RIDGE_SLAVE_ENABLE 
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_pd_pd_do_t* tbt_param = &ptrAltModeContext->tbtStatus.enter_mode_vdo;
    cy_stc_pdaltmode_ridge_reg_t* reg = &ptrAltModeContext->tbtStatus.ridge_status;

    cy_en_pdstack_intel_pf_type_t pf_type = ptrAltModeContext->iclCfg->icl_tgl_selection;

    /* Clear all status bits to start with. */
    reg->val = RIDGE_TBT_MODE_MASK;

    /* Set DFP related regs */
    if(ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP)
    {
        reg->ridge_stat.tbt_type  = tbt_param->tbt_vdo.adapter;

        /* Enable USB2 support always. */
        reg->ridge_stat.usb2_conn = false;

        /* If USB Speed is greater than or equal to Gen2 then set the USB3_Speed to Gen2 */
        if (ptrPdStackContext->dpmStat.cblVdo.act_cbl_vdo.usbSsSup >= CY_PDSTACK_USB_GEN_2)
        {
            reg->ridge_stat.usb3_speed = false;
        }
        
        if (pf_type >= CY_PDSTACK_PF_TIGER_LAKE)
        {
            reg->ridge_stat.retimer = tbt_param->tbt_vdo.b22RetimerCbl;
        }
        
        if(ptrPdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL)
        {
            if (pf_type >= CY_PDSTACK_PF_TIGER_LAKE)
            {
                reg->ridge_stat.active_cbl = true;
            }
            else
            {
                /* Active cable for ICL */
                reg->ridge_stat.retimer = true;
            }
        }
        
        if (ptrAltModeContext->tbtStatus.dev_vdo.tbt_ufp_vdo.adapter != false)
        {
            reg->ridge_stat.tbt_type = true;
        }
    }
    else
    {
        if (pf_type >= CY_PDSTACK_PF_TIGER_LAKE)
        {
            if (
                    (tbt_param->tbt_vdo.cableActive != false)     ||
                    (tbt_param->tbt_vdo.b22RetimerCbl != false)
               )
            {
                reg->ridge_stat.active_cbl = true;
            }

            reg->ridge_stat.retimer = tbt_param->tbt_vdo.b22RetimerCbl;
        }
        else
        {
            if (tbt_param->tbt_vdo.b22RetimerCbl != false)
            {
                /* Active cable for ICL */
                reg->ridge_stat.retimer = true;
            }
        }

        if (tbt_param->tbt_vdo.cblSpeed >= CY_PDSTACK_USB_GEN_2)
        {
            reg->ridge_stat.usb3_speed = true;
        }
    }

    reg->ridge_stat.cbl_type        = tbt_param->tbt_vdo.cblType;
    reg->ridge_stat.act_link_train  = tbt_param->tbt_vdo.linkTraining;
    reg->ridge_stat.cbl_spd         = tbt_param->tbt_vdo.cblSpeed;
    reg->ridge_stat.pro_dock_detect = tbt_param->tbt_vdo.vproDockHost;
    reg->ridge_stat.cbl_gen         = tbt_param->tbt_vdo.cblGen;

    /* 
     * Special handling for LRD cable - bit#24 in TBT disc mode will be set to 1 
     * by this cable & bit#22 (retimer/redriver) will be set to redriver
     */
    if (
           (tbt_param->tbt_vdo.cableActive != false)    &&
           (tbt_param->tbt_vdo.b22RetimerCbl == false) &&
           (pf_type >= CY_PDSTACK_PF_TIGER_LAKE)
        )
    {
        reg->ridge_stat.active_cbl = true;
    }

    /* Set MUX */
    return Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.ridge_status.val);
#elif AMD_SUPP_ENABLE
    if (ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        /* Enable Retimer in case of UFP data role */
        amd_retimer_enable(port, false, true);
    }
    /* Set MUX */
    return Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.enter_mode_vdo.val);
#else
    (void) ptrAltModeContext;
    return false;
#endif /* RIDGE_SLAVE_ENABLE */
}

#else
static bool Cy_PdAltMode_TBT_SetRegsEnter(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_ridge_reg_t* reg   = &ptrAltModeContext->tbtStatus.ridge_status;
    cy_pd_pd_do_t* tbt_param = &ptrAltModeContext->tbtStatus.enter_mode_vdo;

    /* Clear all status bits to start with. */
    reg->val                       = RIDGE_TBT_MODE_MASK;

    /* Update status bits based on data from the Enter Mode VDO. */
    reg->ridge_stat.active_cbl      = tbt_param->tbt_vdo.cableActive;
    reg->ridge_stat.cbl_type        = tbt_param->tbt_vdo.cblType;
    reg->ridge_stat.tbt_type        = tbt_param->tbt_vdo.adapter;
    reg->ridge_stat.act_link_train  = tbt_param->tbt_vdo.linkTraining;
    reg->ridge_stat.cbl_spd         = tbt_param->tbt_vdo.cblSpeed;
    reg->ridge_stat.cbl_gen         = tbt_param->tbt_vdo.cblGen;
    reg->ridge_stat.pro_dock_detect = tbt_param->tbt_vdo.vproDockHost;

    /* Set MUX */
    return Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.ridge_status.val);
}
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

/**********************TBT SINK ALT MODE FUNCTIONS****************************/

#if GATKEX_CREEK
void CY_PdAltMode_Ridge_MuxDelayCbk (cy_timer_id_t id, void * ptrContext)
{
    (void)id;

    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrContext;
    cy_stc_pdaltmode_context_t * ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    /* Stop Delay timer */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GR_MUX_DELAY_TIMER);

    /* Copy Enter VDO to TBT struct */
    ptrAltModeContext->tbtStatus.enter_mode_vdo = *Cy_PdAltMode_TBT_GetVdo(ptrAltModeContext);

    /* Set AR MUX */
    if (Cy_PdAltMode_TBT_SetRegsEnter(ptrAltModeContext))
    {
        info->vdo_numb[CY_PD_SOP] = CY_PDALTMODE_NO_DATA;
        info->mode_state    = ALT_MODE_STATE_IDLE;
    }

    /* If UFP should respond to VDM then send VDM response */
    if ((ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) && (ptrAltModeContext->appStatus.is_vdm_pending != false))
    {
        ptrAltModeContext->appStatus.vdm_resp_cbk(ptrPdStackContext, &ptrAltModeContext->appStatus.vdmResp);
        ptrAltModeContext->appStatus.is_vdm_pending = false;
    }
    ptrAltModeContext->appStatus.is_mux_busy = false;
}
#endif

#if TBT_UFP_SUPP
static void Cy_PdAltMode_TBT_UfpRun(void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;

    cy_stc_pdaltmode_mngr_info_t *info = Cy_PdAltMode_TBT_Info(ptrAltModeContext);

    /* Get VDM command */
    ptrAltModeContext->tbtStatus.state = (cy_en_pdaltmode_tbt_state_t)info->vdm_header.std_vdm_hdr.cmd;

    /* If exit all modes cmd received */
    if (info->vdm_header.std_vdm_hdr.cmd == EXIT_ALL_MODES)
    {
        ptrAltModeContext->tbtStatus.state = TBT_STATE_EXIT;
    }
    switch(info->mode_state)
    {
        case ALT_MODE_STATE_IDLE:
            switch(ptrAltModeContext->tbtStatus.state)
            {
                case TBT_STATE_ENTER:
                    /* Check enter mode VDO */
                    if (
                            (Cy_PdAltMode_TBT_GetVdo(ptrAltModeContext)->tbt_vdo.intelMode == TBT_ALT_MODE_ID) &&
                            (info->vdo_numb[CY_PD_SOP] == MAX_TBT_VDO_NUMB)
                        )
                    {
#if GATKEX_CREEK
                        ptrAltModeContext->appStatus.is_mux_busy = false;

                        if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX )
                        {
                            /* To set the ridge to disconnect between USB and TBT states when device is UFP*/
                            Cy_PdAltMode_Ridge_SetDisconnect(ptrAltModeContext);
                            ptrAltModeContext->appStatus.is_mux_busy = true;
                            Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                                    GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, GR_MUX_DELAY_TIMER),
                                        GR_MUX_VDM_DELAY_TIMER_PERIOD, CY_PdAltMode_Ridge_MuxDelayCbk);
                            info->mode_state    = ALT_MODE_STATE_IDLE;
                            return;
                        }

                        if(ptrAltModeContext->appStatus.is_mux_busy == false)
#endif /* GATKEX_CREEK */
                            /* Copy Enter VDO to TBT struct */
                            ptrAltModeContext->tbtStatus.enter_mode_vdo = *Cy_PdAltMode_TBT_GetVdo(ptrAltModeContext);

                        /* Set AR MUX */
                        if (Cy_PdAltMode_TBT_SetRegsEnter(ptrAltModeContext))
                        {
                            info->vdo_numb[CY_PD_SOP] = CY_PDALTMODE_NO_DATA;
                            info->mode_state = ALT_MODE_STATE_IDLE;
                            return;
                        }
                    }
                    break;
                case TBT_STATE_EXIT:
                    Cy_PdAltMode_TBT_AssignVdo(ptrAltModeContext, NONE_VDO);
                    info->mode_state = ALT_MODE_STATE_EXIT;
                    Cy_PdAltMode_TBT_Exit(ptrAltModeContext);
                    return;
                default:
                    /* Send NACK */
                    info->mode_state = ALT_MODE_STATE_FAIL;
                    break;
            }
            info->mode_state = ALT_MODE_STATE_IDLE;
            return;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            info->mode_state = ALT_MODE_STATE_IDLE;
            return;
        default:
            break;
    }

    /* Send NACK */
    info->mode_state = ALT_MODE_STATE_FAIL;
}
#endif /* TBT_UFP_SUPP */

#if (!ICL_ALT_MODE_HPI_DISABLED)
/******************* TBT application Related Functions ************************/

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))

/* Activates only when application TBT command is received */
static bool Cy_PdAltMode_TBT_EvalAppCmd(void *context, cy_stc_pdaltmode_alt_mode_evt_t cmd_data)
{
#if !ICL_TBT_DISABLE_APP_CHANGES
#if TBT_UFP_SUPP
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;

    /* Save HPI attention cmd and send Attention VDM */
    ptrAltModeContext->tbtStatus.state = TBT_STATE_ATT;
    Cy_PdAltMode_TBT_AssignVdo(ptrAltModeContext, cmd_data.val);
    Cy_PdAltMode_TBT_SendCmd(ptrAltModeContext);

    /* Att HPI command has the same structure as Att VDM in TBT spec */
    /* Restore TBT settings */
    if ((BB_STATUS(cmd_data.val) == false) && (USB2_ENABLE(cmd_data.val) == false))
    {
        /* Set AR MUX */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, ptrAltModeContext->tbtStatus.ridge_status.val);
    }
    if (USB2_ENABLE(cmd_data.val))
    {
        /* Set to USB 2.0 */
        Cy_PdAltMode_HW_SetMux(ptrAltModeContext, MUX_CONFIG_SAFE, CY_PDALTMODE_NO_DATA);
    }
#endif /* TBT_UFP_SUPP */
#endif /* ICL_TBT_DISABLE_APP_CHANGES */
    return true;
}
#endif /* !ICL_ALT_MODE_HPI_DISABLED */
#endif /* DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP */

#endif /* ((TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

/* [] END OF FILE */
