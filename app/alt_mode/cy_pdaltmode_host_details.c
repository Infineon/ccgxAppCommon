/******************************************************************************
* File Name:   cy_pdaltmode_host_details.c
* \version 2.0
*
* Description: Source code of Host Details feature.
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
#include "cy_pdaltmode_intel_vid.h"
#include "cy_pdaltmode_intel_ridge_internal.h"
#include "cy_pdaltmode_ridge_slave.h"
#include "cy_pdaltmode_usb4.h"
#include "cy_pdaltmode_host_details.h"

#if DEBUG_LOG
#include "debug.h"
#endif

#if BB_RETIMER_ENABLE
#include "bb_retimer.h"
#endif /* BB_RETIMER_ENABLE */


/************************** Function definitions *****************************/

#if STORE_DETAILS_OF_HOST
void Cy_PdAltMode_HostDetails_UpdateRidgeCbk(cy_timer_id_t id, void * ptrContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t*) ptrContext;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = (cy_stc_pdaltmode_context_t *)ptrPdStackContext->ptrAltModeContext;
    Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(ptrAltModeContext);

    (void) id;
}

void Cy_PdAltMode_HostDetails_RestartDpmState(cy_timer_id_t id, void * ptrContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t*) ptrContext;

    Cy_PdStack_Dpm_Start(ptrPdStackContext);

    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET, TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET_PERIOD,
                Cy_PdAltMode_HostDetails_UpdateRidgeCbk);

    (void)id;
}

void Cy_PdAltMode_HostDetails_SendHardResetCbk(cy_timer_id_t id, void * ptrContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t*) ptrContext;

    /* If PD capable device is connected, then send HARD Reset */
    if(ptrPdStackContext->dpmStat.pdConnected == true)
    {
        if(Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_HARD_RESET, NULL, false, NULL))
        {
            Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                        Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET, TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET_PERIOD,
                    Cy_PdAltMode_HostDetails_UpdateRidgeCbk);
    }
    /* If non-PD device is connected, then disable Type-C port and re-enable it */
    else if(ptrPdStackContext->dpmConfig.attach == true)
    {
        Cy_PdStack_Dpm_Stop(ptrPdStackContext);
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER, RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER_PERIOD,
                    Cy_PdAltMode_HostDetails_RestartDpmState);
    }

    (void) id;
}

void Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t mode_mask)  /* Only for DS ports */
{
    static cy_pd_pd_do_t vdm_svid_vdo_p[3] = {EMPTY_VDO};
    uint16_t size = 0;

    /* US port - Do not change the modes */
    if( ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
    {
        return;
    }

    vdm_svid_vdo_p[0].val = 0xFF00A042;

    /* DP and TBT are not supported as sink */
    if(
        ((mode_mask & TBT_MODE_UFP) == 0)
        && ((mode_mask & DP_MODE_UFP) == 0)
    )
    {
        size = 0u;
        vdm_svid_vdo_p[0].val = 0xFF00A082;
    }

    /* Only DP is supported as sink */
    else if (
                ((mode_mask & TBT_MODE_UFP) == 0) && (mode_mask & DP_MODE_UFP)
            )
    {
        size = 2u;
        vdm_svid_vdo_p[1].std_svid_res.svidN = CY_PDALTMODE_DP_SVID;
        vdm_svid_vdo_p[1].std_svid_res.svidN1 = EMPTY_VDO;
    }

    /* Only TBT is supported as sink */
    else if (
                (mode_mask & TBT_MODE_UFP) && ((mode_mask & DP_MODE_UFP) == 0)
            )
    {
        size = 2u;
        vdm_svid_vdo_p[1].std_svid_res.svidN = CY_PDALTMODE_TBT_SVID;
        vdm_svid_vdo_p[1].std_svid_res.svidN1 = EMPTY_VDO;
    }

    /* Both TBT and DP are supported as sink */
    else if (
                (mode_mask & TBT_MODE_UFP) && ((mode_mask & DP_MODE_UFP))
            )
    {
        size = 3u;
        vdm_svid_vdo_p[1].std_svid_res.svidN = CY_PDALTMODE_TBT_SVID;
        vdm_svid_vdo_p[1].std_svid_res.svidN1 = CY_PDALTMODE_DP_SVID;
        vdm_svid_vdo_p[2].std_svid_res.svidN = EMPTY_VDO;
        vdm_svid_vdo_p[2].std_svid_res.svidN1 = EMPTY_VDO;
    }

    ptrAltModeContext->vdmSvidVdoCnt = size;
    ptrAltModeContext->vdmSvidVdoP   = (cy_pd_pd_do_t *)vdm_svid_vdo_p;

    /* This function always called with device Context */
    ptrAltModeContext->hostDetails.ds_mode_mask = mode_mask;
}

void Cy_PdAltMode_HostDetails_ChangeDsPortBehaviorBasedOnHostCapability(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* Store previous host capability */
    static cy_stc_pdaltmode_ridge_reg_t host_cap_prev = { .val = 0u, };

    cy_stc_pdaltmode_context_t *hostContext = ptrAltModeContext->hostDetails.usAltModeContext;
    cy_stc_pdaltmode_context_t *deviceContext = ptrAltModeContext->hostDetails.dsAltModeContext;

    cy_stc_pdaltmode_ridge_reg_t *deviceCapability = &deviceContext->ridge.ridge_stat;
    cy_stc_pdaltmode_ridge_reg_t *hostCapability = &hostContext->ridge.ridge_stat;

    cy_pd_pd_do_t temp_usb4_cmd_from_host;

    temp_usb4_cmd_from_host.val = hostContext->hostDetails.host_eudo.val;

    /* Host is NOT connected, & hence go back to default capabilities */
    if((hostCapability->ridge_stat.data_conn_pres == false) &&
            (deviceCapability->ridge_stat.data_conn_pres == true))
    {
        if (hostCapability->val == host_cap_prev.val)
        {
            /* No change in host connection */
            return;
        }

        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, (TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_DFP | USB4_MODE_DFP));
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;
        Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect(deviceContext);

        /* Send hard reset */
        Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                Cy_PdAltMode_HostDetails_SendHardResetCbk);
    }

    /* USB4 host is connected */
    else if
            (
                (
                    (hostCapability->ridge_stat.usb4_conn == 1) &&
                    (hostCapability->ridge_stat.usb4_conn != deviceCapability->ridge_stat.usb4_conn)
                )
                ||
                (
                    (temp_usb4_cmd_from_host.enterusb_vdo.hostPresent == true) &&
                    (deviceContext->vdmStat.eudo_buf.cmdDo[0].enterusb_vdo.hostPresent == false)
                )
            )
    {
        /* Change the TBT role based on TBT Tunneling bit received from US port via ENTER_USB CMD */
        if(temp_usb4_cmd_from_host.enterusb_vdo.hostTbtSupp == true)
        {
            Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext,(TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_DFP | USB4_MODE_DFP));
        }
        else
        {
            Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, (DP_MODE_DFP | USB4_MODE_DFP));
        }

        /* Enable the two lane DP mode */
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

        /* If HOST_PRESENT Bit it set from US port, then send ENTER_USB command to partner */
        if(
            (deviceCapability->ridge_stat.usb4_conn == true) &&
            (temp_usb4_cmd_from_host.enterusb_vdo.hostPresent == true) &&
            (deviceContext->vdmStat.eudo_buf.cmdDo[0].enterusb_vdo.hostPresent == false)
        )
        {
            deviceContext->hostDetails.ds_send_eu_with_host_present_set = true;
        }

        /* If USB3 device is already connected, then issue a hard reset */
        else if((deviceCapability->ridge_stat.usb3_conn == true) || (deviceCapability->ridge_stat.dp_conn == true))
        {
            /* Send hard reset */
            Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }

        /* Update the Ridge status if there is already device connected, and host gets connected later on */
        else if (
            (deviceCapability->ridge_stat.dp_conn == true) ||
            (deviceCapability->ridge_stat.tbt_conn == true) ||
            (deviceCapability->ridge_stat.usb2_conn == true) ||
            (deviceCapability->ridge_stat.usb4_conn == true)
        )
        {
            Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(deviceContext);
        }
    }

    /* Thunderbolt host is connected */
    else if((hostCapability->ridge_stat.tbt_conn == 1) && (hostCapability->ridge_stat.tbt_conn != deviceCapability->ridge_stat.tbt_conn)
    )
    {
        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, (TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_DFP));

        /* Enable the two lane DP mode */
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

        /* Issue Data Reset when TBT host gets connected, and USB4 device is already connected */
        if(deviceCapability->ridge_stat.usb4_conn == true)
        {
            Cy_PdAltMode_Usb4_DataRstRetryCbk(0, deviceContext->pdStackContext);
        }

        /*If USB3 device is already connected, then issue a hard reset */
        else if((deviceCapability->ridge_stat.usb3_conn == true) || (deviceCapability->ridge_stat.dp_conn == true))
        {
            Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }

        /* Update the Ridge status if there is already device connected, and host gets connected later on */
        else if (
            (deviceCapability->ridge_stat.dp_conn == true) ||
            (deviceCapability->ridge_stat.tbt_conn == true) ||
            (deviceCapability->ridge_stat.usb2_conn == true)
        )
        {
            Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(deviceContext);
        }
    }

    /* MFDP host is connected */
    else if((hostCapability->ridge_stat.usb3_conn == 1u) &&
            (hostCapability->ridge_stat.dp_conn == 1u))
    {
        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, DP_MODE_DFP);

        /* Enable the two lane DP mode */
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

        /* Issue Data Reset when MFDP host gets connected, and USB4 device is already connected */
        if(deviceCapability->ridge_stat.usb4_conn == true)
        {
            Cy_PdAltMode_Usb4_DataRstRetryCbk(0u, deviceContext->pdStackContext);
        }

        /* If TBT device is connected, then issue Exit TBT mode command */
        else if(deviceCapability->ridge_stat.tbt_conn == true)
        {
            Cy_PdAltMode_TBT_SendExitModeCmd(deviceContext);
        }

        /* If USB3 device is already connected, then issue a hard reset */
        else if((deviceCapability->ridge_stat.usb3_conn == true) || (deviceCapability->ridge_stat.dp_conn == true))
        {
            Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }

        /* Update the Ridge status if there is already device connected, and host gets connected later on */
        else if (
            (deviceCapability->ridge_stat.dp_conn == true) ||
            (deviceCapability->ridge_stat.usb2_conn == true)
        )
        {
            Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(deviceContext);
        }
    }

    /* DP Only host is connected */
    else if(
                (hostCapability->ridge_stat.dp_conn == 1u) &&
                (hostCapability->ridge_stat.dp_conn != deviceCapability->ridge_stat.dp_conn)
            )
    {
        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, DP_MODE_DFP);

        /* Disable the two lane DP mode */
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 0u;

        /* Issue Data Reset when TBT host gets connected, and USB4 device is already connected*/
        if(deviceCapability->ridge_stat.usb4_conn == true)
        {
            Cy_PdAltMode_Usb4_DataRstRetryCbk(0u, deviceContext->pdStackContext);
        }

        /* If TBT device is connected, then issue Exit TBT mode command */
        else if(deviceCapability->ridge_stat.tbt_conn == true)
        {
            Cy_PdAltMode_TBT_SendExitModeCmd(deviceContext);
        }

        /* If USB3 device is already connected, then issue a data reset */
        else if(
                (deviceCapability->ridge_stat.usb3_conn == true) &&
                (deviceCapability->ridge_stat.dp_conn != 1u) &&
                (deviceContext->pdStackContext->dpmConfig.specRevSopLive == CY_PD_REV3)
                )
        {
               Cy_PdAltMode_Usb4_DataRstRetryCbk(0, deviceContext->pdStackContext);
        }

        /* If DP device is already connected, then issue a hard reset */
        else if ((deviceCapability->ridge_stat.dp_conn == true) || (deviceCapability->ridge_stat.usb3_conn == true))
        {
            Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }
        /* Update the Ridge status if there is already device connected, and host gets connected later on */
        else if (
            (deviceCapability->ridge_stat.dp_conn == true) ||
            (deviceCapability->ridge_stat.usb2_conn == true)
        )
        {
            Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(deviceContext);
        }
    }

    /* USB host is connected */
    else if(
            (hostCapability->ridge_stat.usb3_conn == 1) &&
            (hostCapability->ridge_stat.usb3_conn != deviceCapability->ridge_stat.usb3_conn)
    )
    {
        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, NONE_MODE);

        /* Issue Data Reset when USB host gets connected, and USB4 device is already connected */
        if(deviceCapability->ridge_stat.usb4_conn == true)
        {
            Cy_PdAltMode_Usb4_DataRstRetryCbk(0, deviceContext->pdStackContext);
        }

        /* If TBT device is connected, then issue Exit TBT mode command*/
        else if(deviceCapability->ridge_stat.tbt_conn == true)
        {
            Cy_PdAltMode_TBT_SendExitModeCmd(deviceContext);
        }
        else
        {
            Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                    SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                    Cy_PdAltMode_HostDetails_SendHardResetCbk);
        }
    }

    /* None of the host is connected - Safe state */
    if(
            (hostCapability->ridge_stat.data_conn_pres == true) &&
            (hostCapability->ridge_stat.usb3_conn == 0) &&
            (hostCapability->ridge_stat.usb2_conn == 0) &&
            (hostCapability->ridge_stat.dp_conn == 0) &&
            (hostCapability->ridge_stat.tbt_conn == 0) &&
            (hostCapability->ridge_stat.usb4_conn == 0)
    )
    {
        Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(deviceContext, (TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_DFP | USB4_MODE_DFP));
        deviceContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;

        Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect(deviceContext);

        Cy_PdUtils_SwTimer_Start(deviceContext->pdStackContext->ptrTimerContext, deviceContext->pdStackContext,
                SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER, SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD,
                Cy_PdAltMode_HostDetails_SendHardResetCbk);
    }

    host_cap_prev.val = hostCapability->val;
}

void Cy_PdAltMode_HostDetails_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_stc_pdaltmode_context_t *hostContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.usAltModeContext;
    cy_stc_pdaltmode_context_t *deviceContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.dsAltModeContext;

    if(hostContext->hostDetails.is_host_details_available && hostContext->hostDetails.gr_rdy_bit)
    {
        hostContext->hostDetails.is_host_details_available = false;
        Cy_PdAltMode_HostDetails_ChangeDsPortBehaviorBasedOnHostCapability(ptrAltModeContext);
    }
    else if(hostContext->hostDetails.is_host_details_available == true)
    {
        if((hostContext->hostDetails.host_details & RIDGE_DATA_CONN_PRESENT) == 0x00)
        {
            hostContext->hostDetails.is_host_details_available = false;
            Cy_PdAltMode_HostDetails_ChangeDsPortBehaviorBasedOnHostCapability(ptrAltModeContext);
        }
    }

    if(deviceContext->hostDetails.ds_send_eu_with_host_present_set == true)
    {
        Cy_PdAltMode_Usb4_Enter(deviceContext, CY_PD_SOP, false);
        deviceContext->hostDetails.ds_send_eu_with_host_present_set = false;
        Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(deviceContext);
    }
}

void Cy_PdAltMode_HostDetails_Init(cy_stc_pdaltmode_context_t *hostAltModeContext, cy_stc_pdaltmode_context_t *deviceAltModeContext )
{
    hostAltModeContext->hostDetails.host_dp_2_lane_mode_ctrl = 1u;
    hostAltModeContext->hostDetails.ds_dp_2_lane_mode_ctrl = 1u;
    hostAltModeContext->hostDetails.host_mode_mask = (TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_UFP | USB4_MODE_UFP);
    deviceAltModeContext->hostDetails.ds_mode_mask = (TBT_MODE_DFP | TBT_MODE_UFP | DP_MODE_DFP | USB4_MODE_DFP);

    /* in Galactico Creek port0 is always US and port1 DS */
    hostAltModeContext->hostDetails.usAltModeContext = hostAltModeContext;
    hostAltModeContext->hostDetails.dsAltModeContext = deviceAltModeContext;

    deviceAltModeContext->hostDetails.usAltModeContext = hostAltModeContext;
    deviceAltModeContext->hostDetails.dsAltModeContext = deviceAltModeContext;
}

bool Cy_PdAltMode_HostDetails_CheckIfRidgeNeedsToBeUpdated(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t reg_config)
{
    cy_stc_pdaltmode_context_t *hostContext =
                        (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.usAltModeContext;

    cy_stc_pdaltmode_ridge_reg_t     reg;
    reg.val = reg_config;

    /* Do not update the status to GR if host is NOT connected - For DS ports only
     * Update Ridge if there is a disconnect event even after host is disconnected */
    if(reg.ridge_stat.data_conn_pres == true)
    {
        if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_1_IDX)
        {
            cy_stc_pdaltmode_ridge_reg_t temp_host_cap;
            cy_pd_pd_do_t temp_usb4_cmd_from_host;

            temp_usb4_cmd_from_host.val = hostContext->hostDetails.host_eudo.val;
            temp_host_cap.val = hostContext->hostDetails.host_details;

            /* Host is NOT connected or HOST_Present is NOT set for US port then, then do not update ridge */
            if(
                (temp_host_cap.ridge_stat.data_conn_pres == false) ||
                (
                    (temp_host_cap.ridge_stat.data_conn_pres == true) &&
                    (temp_host_cap.ridge_stat.usb4_conn == true) &&
                    (temp_usb4_cmd_from_host.enterusb_vdo.hostPresent == false)
                )
            )
            {
                return false;
            }
        }
        else /* Do not update US port also until host is connected this condition is for port A */
        {
            cy_pd_pd_do_t temp_usb4_cmd_from_host;

            temp_usb4_cmd_from_host.val = hostContext->hostDetails.host_eudo.val;

            /* Host is NOT connected or HOST_Present is NOT set for US port then, then do not update ridge */
            if(
                (reg.ridge_stat.usb4_conn == true) &&
                (temp_usb4_cmd_from_host.enterusb_vdo.hostPresent == false)
            )
            {
                return false;
            }
        }
    }

    return true;
}

void Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
    ptrAltModeContext->ridge.ridge_slave_stat_reg = ptrAltModeContext->hostDetails.back_up_ridge_status;
    ptrAltModeContext->ridge.soc_intr_write(0u);
}

void Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t  intr_state;
    uint32_t temp_reg_val;

    cy_stc_pdaltmode_context_t *hostContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.usAltModeContext;
    cy_stc_pdaltmode_context_t *deviceContext = (cy_stc_pdaltmode_context_t *) ptrAltModeContext->hostDetails.dsAltModeContext;

    intr_state = Cy_SysLib_EnterCriticalSection();

    temp_reg_val = ptrAltModeContext->ridge.ridge_slave_cmd_reg;
    temp_reg_val &= (~CY_PDALTMODE_RIDGE_RDY_BIT_MASK);
    temp_reg_val &= (~CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK);
    temp_reg_val &= (~CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK);
    ptrAltModeContext->ridge.ridge_slave_cmd_reg = temp_reg_val;

    Cy_SysLib_ExitCriticalSection(intr_state);

    /* Clear the GR RDY flag */
    hostContext->hostDetails.gr_rdy_bit = false;
    deviceContext->hostDetails.gr_rdy_bit = false;
}
#endif /* STORE_DETAILS_OF_HOST */

/* [] END OF FILE */
