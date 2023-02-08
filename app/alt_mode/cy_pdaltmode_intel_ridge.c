/******************************************************************************
* File Name:   cy_pdaltmode_intel_ridge.c
* \version 2.0
*
* Description: Intel Thunderbolt Controller (ridge) control Interface source
*              file.
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

#include "cy_pdaltmode_intel_ridge.h"
#include "cy_pdaltmode_hw.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_dp_sid.h"
#include "cy_pdaltmode_intel_vid.h"
#include "cy_pdaltmode_intel_ridge_internal.h"
#include "cy_pdaltmode_ridge_slave.h"
#include "cy_pdaltmode_vdm_task.h"
#include "cy_pdaltmode_usb4.h"

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_host_details.h"
#endif

#if DEBUG_LOG
#include "debug.h"
#endif

#if BB_RETIMER_ENABLE
#include "bb_retimer.h"
#endif /* BB_RETIMER_ENABLE */

#if RIDGE_SLAVE_ENABLE
/* Sets AR/TR lines as isolate */
static bool Cy_PdAltMode_Ridge_SetIsolate(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param);

/* Sets AR/TR lines as USB 2.0 */
static bool Cy_PdAltMode_Ridge_SetUsb2(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param);

/* Sets AR/TR lines as USB SS */
static bool Cy_PdAltMode_Ridge_SetUsbSs(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param);

/* Sets AR/TR as custom TBT configuration */
static bool Cy_PdAltMode_Ridge_SetCustom(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t ridge_cfg);

/* Sets custom USB4 configuration */
static bool Cy_PdAltMode_Ridge_SetUsb4(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t usb4_cfg);

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
/* Sets AR/TR lines as DP 2 lanes */
static bool Cy_PdAltMode_Ridge_SetDp2lane(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param);

/* Sets AR/TR lines as DP 4 lanes */
static bool Cy_PdAltMode_Ridge_SetDp4lane(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t pin_assign);
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

#define TR_DP_PIN_ASGMNT_E              (0x00)
#define TR_DP_PIN_ASGMNT_CD             (0x01)

static bool (*const ridge_set_fn[MUX_CONFIG_RIDGE_CUSTOM +1])(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param) =
{
    Cy_PdAltMode_Ridge_SetIsolate,
    Cy_PdAltMode_Ridge_SetUsb2,
    Cy_PdAltMode_Ridge_SetUsbSs,
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    Cy_PdAltMode_Ridge_SetDp2lane,
    Cy_PdAltMode_Ridge_SetDp4lane,
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */
    Cy_PdAltMode_Ridge_SetUsb4,
    Cy_PdAltMode_Ridge_SetCustom
};

/************************** Function definitions *****************************/
#if ICL_ENABLE
static uint8_t gl_ridge_force_update[NO_OF_TYPEC_PORTS] = { 0
#if PMG1_PD_DUALPORT_ENABLE
    ,
    0
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

void Cy_PdAltMode_Ridge_ForceStatusUpdate(uint8_t port, uint8_t force_update)
{
    gl_ridge_force_update[port] = force_update;
}
#endif /* ICL_ENABLE */

static void Cy_PdAltMode_Ridge_UpdateReg(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_ridge_reg_t* reg)
{
#if ICL_ENABLE
#if (BB_RETIMER_ENABLE)
    if (PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        retimer_status_update (port, reg->val, gl_ridge_force_update[port]);
        if (app_get_status(port)->retimer_dis_req != false)
        {
            /* Turn the retimer off after it's been written to. */
            retimer_disable (port, (gl_system_state != NB_SYS_PWR_STATE_S0));
            app_get_status(port)->retimer_dis_req = false;
        }
    }
#endif /* (BB_RETIMER_ENABLE) */

    /* Update the status register content and raise an interrupt to the Alpine/Titan Ridge. */
    Cy_PdAltMode_RidgeSlave_StatusUpdate (port, reg->val, gl_ridge_force_update[port]);

    /* Reset force update state after every write */
    gl_ridge_force_update[port] = false;
#else /* (!ICL_ENABLE) */
#if (BB_RETIMER_ENABLE)
    if (PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        retimer_status_update (port, reg->val, false);
        if (app_get_status(port)->retimer_dis_req != false)
        {
            /* Turn the retimer off after it's been written to. */
            retimer_disable (port, (gl_system_state != NB_SYS_PWR_STATE_S0));
            app_get_status(port)->retimer_dis_req = false;
        }
    }
#endif /* (BB_RETIMER_ENABLE) */

    Cy_PdAltMode_RidgeSlave_StatusUpdate (ptrAltModeContext, reg->val, false);
#endif /* ICL_ENABLE */
}

static bool Cy_PdAltMode_Ridge_SetIsolate(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param)
{
    uint32_t status = CY_PDALTMODE_NO_DATA;

    (void)param;

    /* If data connection present. */
    if(ptrAltModeContext->pdStackContext->dpmConfig.attach)
    {
        status = RIDGE_DATA_CONN_PRESENT;
    }

    return Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, status);
}

void Cy_PdAltMode_Ridge_SetDisconnect(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* No connection active. */
#if RIDGE_SLAVE_ENABLE
    /* To update the register status and raise an interrupt to the ridge */
    Cy_PdAltMode_RidgeSlave_StatusUpdate(ptrAltModeContext, CY_PDALTMODE_NO_DATA, false);
#endif

}

static bool Cy_PdAltMode_Ridge_SetUsb2(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param)
{
    uint32_t status = RIDGE_DISCON_STATE_MASK;

    /* If OCP condition is being flagged, set only the OCP and Data Connection Present bits. */
    if (param & RIDGE_STATUS_OCP_MASK)
        status = RIDGE_STATUS_OCP_MASK | RIDGE_DATA_CONN_PRESENT;
    
#if (!CCG_CBL_DISC_DISABLE)
    /* Check active cable bit set is needed */
    if (param & CY_PDALTMODE_RIDGE_ACT_CBL_MASK)
        status |= CY_PDALTMODE_RIDGE_ACT_CBL_MASK;

    /* Check retimer bit set is needed */
    if (param & CY_PDALTMODE_RIDGE_RETIMER_MASK)
        status |= CY_PDALTMODE_RIDGE_RETIMER_MASK;
#endif /* (!CCG_CBL_DISC_DISABLE) */
    
    return Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, status);
}

/* Update USB Status to Goshen Ridge if No Alt Mode */
void Cy_PdAltMode_Ridge_UpdateUsbStatus(cy_timer_id_t id, void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)context;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;
    cy_stc_pdaltmode_ridge_reg_t *reg = &ptrAltModeContext->ridge.ridge_stat;

    if(ptrPdStackContext->dpmConfig.connect == true)
    {
        if( (reg->ridge_stat.dp_conn == 0) && (reg->ridge_stat.tbt_conn == 0)
#if CY_PD_USB4_SUPPORT_ENABLE
                && (reg->ridge_stat.usb4_conn == 0)
#endif
        )
        {
            cy_pd_pd_do_t   cbl_vdo;

            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP) /* Disable the DFP mode for Port A */
            {
                return;
            }

            reg->val &= ~RIDGE_DATA_CONN_MASK;
            reg->val |= RIDGE_USB_STATE_MASK;

            /* Check if no USB 2.0 cable */
            cbl_vdo.val = ptrPdStackContext->dpmStat.cblVdo.val;
            if ((cbl_vdo.val != CY_PDALTMODE_NO_DATA) && (cbl_vdo.std_cbl_vdo.usbSsSup == CY_PDSTACK_USB_2_0_SUPP))
            {
                Cy_PdAltMode_Ridge_SetUsb2(ptrAltModeContext, reg->val);
            }
            /* Set GEN2 speed only if the cable supports it. */
            if ((cbl_vdo.val == CY_PDALTMODE_NO_DATA) || (cbl_vdo.std_cbl_vdo.usbSsSup == CY_PDSTACK_USB_GEN_2_SUPP))
            {
                reg->ridge_stat.usb3_speed = true;
            }
            /* Set AR/TR */
            Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, reg->val);
        }
    }

    (void) id;
}

static bool Cy_PdAltMode_Ridge_SetUsbSs(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t param)
{
    (void)param;
    cy_stc_pdaltmode_ridge_reg_t *reg = &ptrAltModeContext->ridge.ridge_stat;
    
#if (!CCG_CBL_DISC_DISABLE)
#if CY_PD_USB4_SUPPORT_ENABLE
    cy_en_pdstack_intel_pf_type_t pf_type = ptrAltModeContext->iclCfg->icl_tgl_selection;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
#endif /* (!CCG_CBL_DISC_DISABLE) */

#if (!CCG_CBL_DISC_DISABLE)
    cy_en_pdstack_usb_data_sig_t usb_sig = Cy_PdAltMode_Mngr_GetCableUsbCap(ptrAltModeContext->pdStackContext);
#endif /* (!CCG_CBL_DISC_DISABLE) */

#if (TITAN_RIDGE_ENABLE)
    /* We don't support USB 3.0 connection as UFP (Titan Ridge). */
    if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        return false;
    }
#endif /* (!ICL_ENABLE) */

    /* We don't support USB 3.0 connection as UFP. */
    if ((ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) &&
            (ptrAltModeContext->pdStackContext->port == TYPEC_PORT_1_IDX))
    {
        return false;
    }

    /* Clear all bits which are concerned with data connection status. */
    reg->val &= ~RIDGE_DATA_CONN_MASK;

    /* Set the bits associated with USB 3.x connection. */
    reg->val |= RIDGE_USB_STATE_MASK;

#if DEBUG_ACCESSORY_SNK_ENABLE
    /* If this is a Rd-Rd or Rp-Rp debug accessory indicate it to the PMC and retimer */
    if (ptrAltModeContext->pdStackContext->dpmConfig.attachedDev == CY_PD_DEV_DBG_ACC)
    {
        reg->ridge_stat.dbg_acc_mode = 1;
    }
#endif /* DEBUG_ACCESSORY_SNK_ENABLE */

#if (!CCG_CBL_DISC_DISABLE)
#if CY_PD_USB4_SUPPORT_ENABLE
    /* Set Active Cable bit */
    if (
            (ptrAltModeContext->pdStackContext->dpmConfig.emcaPresent != false)         &&
            (ptrAltModeContext->pdStackContext->dpmStat.cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) &&
            ptrAltModeContext->appStatus.cbl_disc_id_finished &&
            ptrAltModeContext->appStatus.usb2Supp != false
       )
    {
        if (pf_type == CY_PDSTACK_PF_ICE_LAKE)
        {
            /* ICL platform uses retimer bit as active cable bit */
            reg->ridge_stat.retimer = true;
        }
        /* TGL platform selected */
        else if (pf_type >= CY_PDSTACK_PF_TIGER_LAKE)
        {
            reg->ridge_stat.active_cbl = true;
            if (
                   (
                           (ptrAltModeContext->pdStackContext->dpmStat.cblVdo2.val != CY_PDALTMODE_NO_DATA)                             &&
                           (ptrAltModeContext->pdStackContext->dpmStat.cblVdo.act_cbl_vdo1.vdoVersion >= CY_PD_CBL_VDO_VERS_1_3) &&
                           (ptrAltModeContext->pdStackContext->dpmStat.cblVdmVersion > CY_PDSTACK_STD_VDM_VER1)                       &&
                           (ptrAltModeContext->pdStackContext->dpmStat.cblVdo2.act_cbl_vdo2.activeEl != false)
                    ) ||
                    (ptrAltModeContext->appStatus.cable_retimer_supp != false)
                )
            {
                {
                    reg->ridge_stat.retimer = true;
                }
            }
        }
    }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    /* Check if no USB 2.0 cable */
    if (
            (ptrAltModeContext->appStatus.cbl_disc_id_finished == true) &&
            (usb_sig == CY_PDSTACK_USB_2_0_SUPP)
        )
    {
        return Cy_PdAltMode_Ridge_SetUsb2(ptrAltModeContext, reg->val);
    }
    
    /* Set GEN2 speed only if the cable supports it. */
    if (
            (ptrAltModeContext->appStatus.cbl_disc_id_finished == false) ||
            (usb_sig >= CY_PDSTACK_USB_GEN_2_SUPP) ||
            (ptrAltModeContext->pdStackContext->dpmConfig.emcaPresent == false)
        )
    {
        reg->ridge_stat.usb3_speed = true;
    }
    else
    {
        reg->ridge_stat.usb3_speed = false;
        reg->ridge_stat.cbl_spd = 0;
    }
#else
    /* Assume USB 3.2 Gen2 support when we have no cable information available. */
    reg->ridge_stat.usb3_speed = 1;
#endif /* (!CCG_CBL_DISC_DISABLE) */

#if GATKEX_CREEK
    if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
    {
        Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext,
                GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, DELAY_US_PORT_USB_CONNECTION_BYAMETIMOUT_TIMER),
                    DELAY_US_PORT_USB_CONNECTION_BYAMETIMOUT_TIMER_PERIOD, Cy_PdAltMode_Ridge_UpdateUsbStatus);
        return true;
    }
#endif /* GATKEX_CREEK */

    /* Set AR/TR */
    return Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, reg->val);
}

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static bool Cy_PdAltMode_Ridge_SetDp2lane(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t pin_assign)
{
    cy_stc_pdaltmode_ridge_reg_t *reg = &ptrAltModeContext->ridge.ridge_stat;
    cy_pd_pd_do_t   cbl_vdo;

    reg->val &= ~RIDGE_DATA_CONN_MASK;
    reg->val |= RIDGE_DP_2_LANE_MASK;

#if (DP_UFP_SUPP)
    if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        reg->ridge_stat.dp_role = true;
    }
#endif /* DP_UFP_SUPP */

    /* As per DP AltMode Spec v1.0, Table 5-2 */
    if ((pin_assign & 0xFF) == CY_PDALTMODE_DP_DFP_D_CONFIG_D)
    {
        reg->ridge_stat.dp_pin_assign = TR_DP_PIN_ASGMNT_CD;
    }

#if ICL_ENABLE
    if (pin_assign & (1 << HPD_STATE_BIT_POS))
    {
        reg->ridge_stat.hpd_lvl = true;
    }

    if (pin_assign & (1 << HPD_IRQ_BIT_POS))
    {
        reg->ridge_stat.hpd_irq = true;
    }
#endif /* ICL_ENABLE */

    /* Set GEN2 speed only if the cable supports it. */
    cbl_vdo.val = ptrAltModeContext->pdStackContext->dpmStat.cblVdo.val;
    if ((cbl_vdo.val == CY_PDALTMODE_NO_DATA) || (Cy_PdAltMode_Mngr_GetCableUsbCap(ptrAltModeContext->pdStackContext) == CY_PDSTACK_USB_GEN_2_SUPP))
    {
        reg->ridge_stat.usb3_speed = true;
    }

    /* Set AR/TR */
    return Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, reg->val);
}

static bool Cy_PdAltMode_Ridge_SetDp4lane(cy_stc_pdaltmode_context_t *ptrAltModeContext,  uint32_t pin_assign)
{
    cy_stc_pdaltmode_ridge_reg_t*     reg = &ptrAltModeContext->ridge.ridge_stat;

    reg->val &= ~RIDGE_DATA_CONN_MASK;
    reg->val |= RIDGE_DP_4_LANE_MASK;

    /* CDT 286795 - trigger config bit if DP C pin assignment */
    if ((pin_assign & 0xFF) == CY_PDALTMODE_DP_DFP_D_CONFIG_C)
    {
        reg->ridge_stat.dp_pin_assign = TR_DP_PIN_ASGMNT_CD;
    }

#if ICL_ENABLE
    if (pin_assign & (1 << HPD_STATE_BIT_POS))
    {
        reg->ridge_stat.hpd_lvl = true;
    }

    if (pin_assign & (1 << HPD_IRQ_BIT_POS))
    {
        reg->ridge_stat.hpd_irq = true;
    }
#endif /* ICL_ENABLE */

#if (DP_UFP_SUPP)
    if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
    {
        reg->ridge_stat.dp_role = true;
    }
#endif /* DP_UFP_SUPP */

    return Cy_PdAltMode_Ridge_SetCustom(ptrAltModeContext, reg->val);
}
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

static void Cy_PdAltMode_Ridge_UpdateUsbSupp(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_ridge_reg_t* reg)
{
    /* Check if USB2 and USB3 fields allowed */
    if (ptrAltModeContext->appStatus.usb2Supp == false)
    {
        reg->ridge_stat.usb2_conn = false;
    }

    if (ptrAltModeContext->appStatus.usb3Supp == false)
    {
        reg->ridge_stat.usb3_conn  = false;
        reg->ridge_stat.usb3_speed = false;
    } 
}

static bool Cy_PdAltMode_Ridge_SetCustom(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t ridge_cfg)
{
#if STORE_DETAILS_OF_HOST
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#endif

    cy_stc_pdaltmode_ridge_reg_t* reg = &ptrAltModeContext->ridge.ridge_stat;

    /* Copy ar config to AR/TR struct */
    reg->val = ridge_cfg;
    
    /* Update USB supported field */
    Cy_PdAltMode_Ridge_UpdateUsbSupp(ptrAltModeContext, reg);

    /* Update other fields only when DATA connection is present. */
    if ((reg->ridge_stat.data_conn_pres) && (ridge_cfg != CY_PDALTMODE_NO_DATA))
    {
        /* Update polarity field. */
        reg->ridge_stat.conn_orien = ptrAltModeContext->ridge.polarity;

        /* Update data role field. */
        reg->ridge_stat.usb_dr = false;

#if ICL_ENABLE
        if (ptrAltModeContext->ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
#else
        if (
                (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) &&
                (reg->ridge_stat.tbt_conn == CY_PDALTMODE_NO_DATA)
                )
#endif /* ICL_ENABLE */
        {
            reg->ridge_stat.usb_dr = true;
        }
    }

#if GATKEX_CREEK
    if (reg->ridge_stat.data_conn_pres == 0)
    {
        reg->val = 0;

#if STORE_DETAILS_OF_HOST
        if(port == TYPEC_PORT_0_IDX)
        {
            Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect(ptrAltModeContext);
        }
#endif /* STORE_DETAILS_OF_HOST */
    }
#endif

#if STORE_DETAILS_OF_HOST
    if(port == TYPEC_PORT_0_IDX)
    {
        ptrAltModeContext->hostDetails.host_details = reg->val;
        ptrAltModeContext->hostDetails.is_host_details_available = true;

        /* Clear the ENTER_USB command details register - since USB4 host is not connected */
        if(reg->ridge_stat.usb4_conn == false)
        {
            ptrAltModeContext->hostDetails.host_eudo.val = CY_PDALTMODE_NO_DATA;
        }
    }
#endif /* STORE_DETAILS_OF_HOST */

    Cy_PdAltMode_Ridge_UpdateReg (ptrAltModeContext, reg);
    return true;
}

static bool Cy_PdAltMode_Ridge_SetUsb4(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t usb4_cfg)
{
#if CY_PD_USB4_SUPPORT_ENABLE
    cy_stc_pdaltmode_ridge_reg_t* reg = &ptrAltModeContext->ridge.ridge_stat;

    /* Copy ar config to AR/TR struct */
    reg->val = usb4_cfg;

    /* Set USB4 connection mask */
    reg->val |= RIDGE_USB4_CONN_MASK;

    /* Update polarity field. */
    reg->ridge_stat.conn_orien = ptrAltModeContext->ridge.polarity;

    reg->ridge_stat.usb3_speed = false;

    Cy_PdAltMode_Ridge_UpdateDr (ptrAltModeContext);

#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    return true;
}

void Cy_PdAltMode_Ridge_SetVpro(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool en_status)
{
#if CY_PD_USB4_SUPPORT_ENABLE
    cy_stc_pdaltmode_ridge_reg_t* reg = &ptrAltModeContext->ridge.ridge_stat;

    if (reg->ridge_stat.pro_dock_detect != en_status)
    {
        /* Enable/Disable vPro mode */
        reg->ridge_stat.pro_dock_detect = en_status;

        /* Update the SoC and retimer as required. */
        Cy_PdAltMode_Ridge_UpdateReg(ptrAltModeContext, reg);
    }
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
}

void Cy_PdAltMode_Ridge_UpdateDr(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{   
    cy_stc_pdaltmode_ridge_reg_t* reg = &ptrAltModeContext->ridge.ridge_stat;
    reg->ridge_stat.usb_dr = false;

    /* Update USB supported field */
    Cy_PdAltMode_Ridge_UpdateUsbSupp(ptrAltModeContext, reg);
    
    if (
           (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
#if (!ICL_ENABLE)
         && (reg->ridge_stat.tbt_conn == CY_PDALTMODE_NO_DATA)
#endif /* (!ICL_ENABLE) */
       )
    {
        reg->ridge_stat.usb_dr = true;
    }
#if STORE_DETAILS_OF_HOST
#if DO_NOT_UPDATE_US_IF_T_AME_TIMER_IS_RUNNING
    if (
            (ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX) &&
            (reg->ridge_stat.data_conn_pres == true) &&
            (reg->ridge_stat.dp_conn == 0) && (reg->ridge_stat.tbt_conn == 0) &&
#if CY_PD_USB4_SUPPORT_ENABLE
            (reg->ridge_stat.usb4_conn == 0) &&
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
            ((Cy_PdUtils_SwTimer_IsRunning(ptrAltModeContext->pdStackContext->ptrTimerContext, DELAY_US_PORT_USB_CONNECTION_BYAMETIMOUT_TIMER)) == true)
        )
    {
        return;
    }
#endif /*DO_NOT_UPDATE_US_IF_T_AME_TIMER_IS_RUNNING */

    if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
    {
       cy_pd_pd_do_t tempusb4_cmd_rcvd_frm_partner;

       tempusb4_cmd_rcvd_frm_partner.val = ptrAltModeContext->hostDetails.host_eudo.val;

       ptrAltModeContext->hostDetails.host_details = reg->val;
       ptrAltModeContext->hostDetails.is_host_details_available = true;

       /* If HOST is not present then clear the USB4 host details */
       if(
           (reg->ridge_stat.data_conn_pres == false) ||
           (reg->ridge_stat.usb4_conn == false)
       )
       {
           ptrAltModeContext->hostDetails.host_eudo.val = CY_PDALTMODE_NO_DATA;
       }

       /* For US port, don't report the event to GR until HOST present bit is set */
       if(tempusb4_cmd_rcvd_frm_partner.enterusb_vdo.hostPresent == false)
       {
           /* Since host is not connected yet */
           ptrAltModeContext->hostDetails.is_host_details_available = false;
       }
       if(reg->ridge_stat.data_conn_pres == false)
       {
           ptrAltModeContext->hostDetails.host_details = CY_PDALTMODE_NO_DATA;
       }
    }
#endif /* STORE_DETAILS_OF_HOST */

    /* Update the SoC and retimer as required. */
    Cy_PdAltMode_Ridge_UpdateReg(ptrAltModeContext, reg);
}

bool Cy_PdAltMode_Ridge_SetMux(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t cfg, uint8_t polarity, uint32_t ridge_cfg)
{
    bool ret = false;

    ptrAltModeContext->ridge.polarity = polarity;

    if (cfg <= MUX_CONFIG_RIDGE_CUSTOM)
    {
        ret = ridge_set_fn[cfg](ptrAltModeContext, ridge_cfg);
    }

    return ret;
}

#if VIRTUAL_HPD_ENABLE
cy_en_pdstack_status_t Cy_PdAltMode_Ridge_HpdInit(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_cb_usbpd_hpd_events_t cbk)
{
    cy_en_pdstack_status_t stat = CY_PDSTACK_STAT_FAILURE;

    ptrAltModeContext->ridge.hpd_update_req = false;
    if (cbk != NULL)
    {
        ptrAltModeContext->ridge.hpd_cbk = cbk;
        stat = CY_PDSTACK_STAT_SUCCESS;
    }

    return stat;
}

void Cy_PdAltMode_Ridge_HpdDeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    ptrAltModeContext->ridge.hpd_update_req = false;
    ptrAltModeContext->ridge.hpd_cbk = NULL;

    Cy_PdAltMode_RidgeSlave_ResetVirtualHpd(ptrAltModeContext);
}

void Cy_PdAltMode_Ridge_InitiateIntForClearBit(cy_timer_id_t id, void *ptrContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t*) ptrContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;

#if !CY_PD_USB4_SUPPORT_ENABLE || GATKEX_CREEK
    ptrAltModeContext->ridge.ridge_stat.ridge_stat.irq_ack = false;
#endif /* !CY_PD_USB4_SUPPORT_ENABLE || GATKEX_CREEK */

    Cy_PdAltMode_RidgeSlave_StatusUpdate(ptrAltModeContext, ptrAltModeContext->ridge.ridge_stat.val, true);

    (void)id;
}

void Cy_PdAltMode_Ridge_EvalHpdQueue(cy_timer_id_t id, void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *) context;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    ptrAltModeContext->ridge.hpd_cbk(ptrPdStackContext->ptrUsbPdContext, CY_USBPD_HPD_COMMAND_DONE);

    (void) id;
}

cy_en_pdstack_status_t Cy_PdAltMode_Ridge_HpdSendEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_usbpd_hpd_events_t evtype)
{
    cy_en_pdstack_status_t stat = CY_PDSTACK_STAT_SUCCESS;
    cy_stc_pdaltmode_ridge_reg_t* reg  = &ptrAltModeContext->ridge.ridge_stat;

    ptrAltModeContext->ridge.hpd_update_req = true;

    /* Update HPD-out as required. */
    switch (evtype)
    {
        case CY_USBPD_HPD_EVENT_UNPLUG:
            reg->ridge_stat.hpd_lvl = false;
            reg->ridge_stat.hpd_irq = false;
            break;

        case CY_USBPD_HPD_EVENT_PLUG:
            ptrAltModeContext->ridge.hpd_update_req = false;
            if (reg->ridge_stat.hpd_lvl == false)
            {
                reg->ridge_stat.hpd_lvl = true;
                ptrAltModeContext->ridge.hpd_update_req = true;
            }
            break;

        case CY_USBPD_HPD_EVENT_IRQ:
            reg->ridge_stat.hpd_irq = true;
            reg->ridge_stat.hpd_lvl = true;
            break;

        case CY_USBPD_HPD_COMMAND_DONE:
            ptrAltModeContext->ridge.hpd_update_req = false;

            /*
             * Update IRQ_ACKfmPD bit when Sink to notify Ridge that Attention
             * command was sent successfully
             */
            if(ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
#if !CY_PD_USB4_SUPPORT_ENABLE
                reg->ridge_stat.irq_ack = false;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

#if GATKEX_CREEK
                reg->ridge_stat.irq_ack = true;
#endif /* GATKEX_CREEK */

                ptrAltModeContext->ridge.hpd_update_req = true;
            }
            break;

        default:
            return CY_PDSTACK_STAT_BAD_PARAM;
    }

    /* Update Ridge/SoC status */
    if (ptrAltModeContext->ridge.hpd_update_req != false)
    {
#if ICL_ENABLE
        if (PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_tgl_selection != 0)
        {
            /* Force status update even if contents are same */
            Cy_PdAltMode_Ridge_ForceStatusUpdate (port, true);
            Cy_PdAltMode_HW_SetMux(ptrAltModeContext->ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, reg->val);
        }
        else
#endif /* ICL_ENABLE */
        {
            Cy_PdAltMode_RidgeSlave_StatusUpdate (ptrAltModeContext, reg->val, true);
        }
    }
    else
    {
        /* If HPD status update not required then go to next queue */
        if (ptrAltModeContext->ridge.hpd_cbk != NULL)
        {
            Cy_PdUtils_SwTimer_Start(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext->pdStackContext,
                    GET_APP_TIMER_ID(ptrAltModeContext->pdStackContext, APP_RIDGE_INIT_HPD_DEQUEUE_TIMER_ID),
                            RIDGE_INIT_HPD_DEQUEUE_TIMER_PERIOD, Cy_PdAltMode_Ridge_EvalHpdQueue);
        }
    }

    return stat;
}

bool Cy_PdAltMode_Ridge_IsHpdChange(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    return ptrAltModeContext->ridge.hpd_update_req;
}

void Cy_PdAltMode_Ridge_InitiateIntForClearHpdIrqBit(cy_timer_id_t id, void *ptrContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext   = (cy_stc_pdstack_context_t*) ptrContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *) ptrPdStackContext->ptrAltModeContext;
    cy_stc_pdaltmode_ridge_reg_t*     reg         = &ptrAltModeContext->ridge.ridge_stat;

    if (reg->ridge_stat.hpd_irq == true)
    {
        reg->ridge_stat.hpd_irq = false;
        Cy_PdAltMode_RidgeSlave_StatusUpdate(ptrAltModeContext, reg->val, true);
    }

    (void) id;
}
#endif /* VIRTUAL_HPD_ENABLE */

void Cy_PdAltMode_Ridge_EvalCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t stat, uint32_t stat_mask)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;
    cy_stc_pdstack_context_t *ptrPdStackContext = ptrAltModeContext->pdStackContext;

    /* Notify the APP layer about the control register change. */
    if (ptrAltModeContext->ridge.ctrl_change_cb != NULL)
    {
        ptrAltModeContext->ridge.ctrl_change_cb(port);
    }

    /* Go through all bits to find bits which were changed from the last time. */
#if STORE_DETAILS_OF_HOST
    if (port == TYPEC_PORT_0_IDX)
    {
        if( ((stat_mask & CY_PDALTMODE_RIDGE_RDY_BIT_MASK) == CY_PDALTMODE_RIDGE_RDY_BIT_MASK) &&
                ((stat & CY_PDALTMODE_RIDGE_RDY_BIT_MASK) == CY_PDALTMODE_RIDGE_RDY_BIT_MASK))
        {
            ptrAltModeContext->hostDetails.gr_rdy_bit = true;
        }
        if((stat & CY_PDALTMODE_RIDGE_RDY_BIT_MASK) == 0)
        {
            ptrAltModeContext->hostDetails.gr_rdy_bit = false;
        }
    }
#endif /* STORE_DETAILS_OF_HOST */

    /* Check TBT conn bit */
    if (stat_mask & CY_PDALTMODE_TBT_HOST_CONN_MASK)
    {
    }

    /* Check USB conn bit */
    if (stat_mask & CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK)
    {
    }

    /* Check DP conn bit */
    if (stat_mask & CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK)
    {
    }

#if VIRTUAL_HPD_ENABLE
    if (((stat_mask & CY_PDALTMODE_RIDGE_IRQ_ACK_MASK) == CY_PDALTMODE_RIDGE_IRQ_ACK_MASK) && ((stat & CY_PDALTMODE_RIDGE_IRQ_ACK_MASK) == CY_PDALTMODE_RIDGE_IRQ_ACK_MASK))
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SEND_IRQ_CLEAR_ACK),
                    APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD, Cy_PdAltMode_Ridge_InitiateIntForClearHpdIrqBit);
    }
#endif /* VIRTUAL_HPD_ENABLE */

#if STORE_DETAILS_OF_HOST
    cy_stc_pdaltmode_context_t *hostContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.usAltModeContext;
    cy_stc_pdaltmode_context_t *deviceContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.dsAltModeContext;

    if (stat_mask & CY_PDALTMODE_RIDGE_HOST_ALL_CONN_MASK)
    {
        if(((stat & CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK) == 0) &&
                (stat & CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK))
        {
            Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, NONE_MODE);
            Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }

        /*Only DP host is connected - Enable DP only */
        else if((stat & CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK) &&
                ((stat & CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK) == 0))
        {
            Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, DP_MODE_DFP);

            /* Disable the two lane DP mode */
            hostContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

            /* If USB3 device or USB4 device or TBT device is already connected, then issue a hard reset */
            if((ptrAltModeContext->ridge.ridge_stat.ridge_stat.usb3_conn == true) ||
                    (ptrAltModeContext->ridge.ridge_stat.ridge_stat.usb4_conn == true) ||
                    (ptrAltModeContext->ridge.ridge_stat.ridge_stat.tbt_conn == true))
            {
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                        Cy_PdAltMode_HostDetails_SendHardResetCbk);
            }
        }

        /* MFDP host is connected - Enable both USB and DP but disable TBT */
        else if(
                   ((stat & CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK)) &&
                   ((stat & CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK))
               )
        {
            Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, DP_MODE_DFP);
            hostContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

            if((ptrAltModeContext->ridge.ridge_stat.ridge_stat.usb3_conn == true) ||
                    (ptrAltModeContext->ridge.ridge_stat.ridge_stat.usb4_conn == true) ||
                    (ptrAltModeContext->ridge.ridge_stat.ridge_stat.tbt_conn == true))
            {
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                        Cy_PdAltMode_HostDetails_SendHardResetCbk);
            }
        }
    }
#endif /* #if STORE_DETAILS_OF_HOST */

#if ((VIRTUAL_HPD_ENABLE) && (DP_UFP_SUPP))
#if (!ICL_ENABLE)
    if (Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext))
#endif /* (ICL_ENABLE) */
    {
        if (ptrAltModeContext->ridge.hpd_cbk != NULL)
        {
            /* Check HPD level conn bit */
            if (stat_mask & CY_PDALTMODE_RIDGE_HPD_LVL_MASK)
            {
                if (stat & CY_PDALTMODE_RIDGE_HPD_LVL_MASK)
                {
                    /* Set HPD High */
                    ptrAltModeContext->ridge.hpd_cbk(ptrPdStackContext->ptrUsbPdContext, CY_USBPD_HPD_EVENT_PLUG);
                }
                else
                {
                    /* Set HPD Low */
                    ptrAltModeContext->ridge.hpd_cbk(ptrPdStackContext->ptrUsbPdContext, CY_USBPD_HPD_EVENT_UNPLUG);
                }
            }

            /* Check HPD IRQ bit */
            if ((stat_mask & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) && (stat & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK))
            {
                ptrAltModeContext->ridge.hpd_cbk(ptrPdStackContext->ptrUsbPdContext, CY_USBPD_HPD_EVENT_IRQ);
            }

            if (( (stat & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) == 0) && (stat_mask & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK))
            {
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SEND_IRQ_CLEAR_ACK),
                            APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD, Cy_PdAltMode_Ridge_InitiateIntForClearBit);
            }
        }
    }
#endif /* DP_UFP_SUPP */

#if STORE_DETAILS_OF_HOST
    if ( (stat & CY_PDALTMODE_RIDGE_DATA_RESET_MASK) && (stat_mask & CY_PDALTMODE_RIDGE_DATA_RESET_MASK) )
    {
        Cy_PdAltMode_Usb4_DataRstRetryCbk(0, ptrPdStackContext);
    }

    if ( (stat & CY_PDALTMODE_RIDGE_POWER_RESET_MASK) && (stat_mask & CY_PDALTMODE_RIDGE_POWER_RESET_MASK))
    {
        Cy_PdStack_Dpm_Stop(ptrPdStackContext);
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER, RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER_PERIOD,
                    Cy_PdAltMode_HostDetails_RestartDpmState);
    }
#endif /* STORE_DETAILS_OF_HOST*/

}

void Cy_PdAltMode_Ridge_SetCtrlChangeCbk(cy_stc_pdaltmode_context_t *ptrAltModeContext, ridge_ctrl_change_cb_t cb)
{
    ptrAltModeContext->ridge.ctrl_change_cb = cb;
}
#endif /* RIDGE_SLAVE_ENABLE */

/* [] END OF FILE */
