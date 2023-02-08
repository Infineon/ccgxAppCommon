/******************************************************************************
* File Name:   app.c
* \version 2.0
*
* Description: This is the source file for PD application
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include <cy_pdstack_common.h>
#include "config.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_idac_ctrl.h"
#if (!CY_PD_SINK_ONLY)
#include <psource.h>
#endif /* CY_PD_SINK_ONLY */
#if (CY_PD_SINK_ONLY)
#include <psink.h>
#endif /* CY_PD_SINK_ONLY */
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <cy_pdaltmode_vdm_task.h>
#include "cy_pdaltmode_defines.h"
#include <cy_pdaltmode_mngr.h>
#include <cy_pdaltmode_hw.h>
#include <cy_pdutils_sw_timer.h>
#include <app_timer_id.h>
#include "cy_usbpd_common.h"
#include "srom.h"
#include "cy_usbpd_config_table.h"

#include <cy_gpio.h>

#if CCG_LOAD_SHARING_ENABLE
#if CCG_LS_INTER_INTRA_ENABLE
#include <loadsharing_inter_intra.h>
#else
#include <loadsharing.h>
#endif /* CCG_LS_INTER_INTRA_ENABLE */
#include <power_throttle.h>
#endif /* CCG_LOAD_SHARING_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
#include <sensor_check.h>
#include <power_throttle.h>
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */

#if CY_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif

#if CCG_LIN_ENABLE
#include <lins.h>
#include "lins_config.h"
#endif /* CCG_LIN_ENABLE */
#if CCG_LPM_GPIO_ENABLE
#include <lpm_gpio_app.h>
#endif /* CCG_LPM_GPIO_ENABLE */
#if (CCG_LPM_GPIO_ENABLE || CCG_LIN_ENABLE)
#include <cy_usbpd_typec.h>
#endif /* (CCG_LPM_GPIO_ENABLE || CCG_LIN_ENABLE) */

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#include <intel_ridge.h>
#endif /* RIDGE_SLAVE_ENABLE */
#if AMD_SUPP_ENABLE
#include <amd.h>
#if AMD_RETIMER_ENABLE
#include <amd_retimer.h>
#endif /* AMD_RETIMER_ENABLE */
#endif /* AMD_SUPP_ENABLE */

#if DP_UFP_SUPP
#include "cy_pdaltmode_dp_sid.h"
#endif /* DP_UFP_SUPP */
#if (CCG_BB_ENABLE != 0)
#include <cy_pdaltmode_billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

#if DP_UFP_SUPP
#include <cy_usbpd_hpd.h>
#endif /* DP_UFP_SUPP */

#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
#include <type_a.h>
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if ICL_ENABLE
#include <icl.h>
#endif /* ICL_ENABLE */

#if BB_RETIMER_ENABLE
#include <bb_retimer.h>
#endif /* BB_RETIMER_ENABLE */

#if CCG_UCSI_ENABLE
#include <ucsi.h>
#endif /* CCG_UCSI_ENABLE */

#if CCG_SYNC_ENABLE
#include <ccg_sync.h>
#endif /* CCG_SYNC_ENABLE */

#if WATCHDOG_OVER_POLL_TIMER
#include <instrumentation.h>
#endif /* WATCHDOG_OVER_POLL_TIMER */

#if (AUVDM_SUPPORT != 0)
#include "auvdm.h"
#endif

#if (BCR != 0)
#include "bcr.h"
#endif /* (BCR != 0) */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#include "cy_usbpd_buck_boost.h"
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

/* Global variable to keep track of solution level functions */
app_sln_handler_t *solution_fn_handler;

#if CCG_LOAD_SHARING_ENABLE
static uint32_t  gl_src_sel_pdo_max_volt[NO_OF_TYPEC_PORTS] = {0};
#endif /* CCG_LOAD_SHARING_ENABLE*/
#if (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK)
static bool gl_bad_device[NO_OF_TYPEC_PORTS] = {0};
#endif /* (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK) */

#if ((BATTERY_CHARGING_ENABLE) || (BB_RETIMER_ENABLE) || (NB_HOST_SUPPORT_ENABLE))
extern uint8_t gl_system_state;
uint8_t gl_system_state = (uint8_t)NB_SYS_PWR_STATE_S0;
#endif /* ((BATTERY_CHARGING_ENABLE) || (BB_RETIMER_ENABLE) || (NB_HOST_SUPPORT_ENABLE)) */

#if ((!CY_PD_SINK_ONLY) && (CY_PD_REV3_ENABLE))

#if (CCG_LOAD_SHARING_ENABLE && CY_PD_BIST_STM_ENABLE)
/* Flag to store whether loadsharing was enabled before entering BIST STM mode */
static bool gl_ls_was_enabled[NO_OF_TYPEC_PORTS];
#endif /* (CCG_LOAD_SHARING_ENABLE && CY_PD_BIST_STM_ENABLE) */

#if CY_PD_BIST_STM_ENABLE
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
/* Flag to store whether power throttling was enabled before entering BIST STM mode */
static bool gl_pt_was_enabled[NO_OF_TYPEC_PORTS];
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
#endif /* CY_PD_BIST_STM_ENABLE */

#endif /* ((!CY_PD_SINK_ONLY) && (CY_PD_REV3_ENABLE)) */

/* Update: Review this enum placement */
/**
 * @typedef usb_data_sig_t
 * @brief Enumeration of USB signalling supported by a device or cable.
 */
typedef enum
{
    USB_2_0_SUPP = 0,                           /**< Only USB 2.0 support. */
    USB_GEN_1_SUPP,                             /**< USB 3.1 Gen1 (5 Gbps) support. */
    USB_GEN_2_SUPP,                             /**< USB 3.1 Gen2 (10 Gbps) support. */
    USB_GEN_3_SUPP,                             /**< USB 4 Gen3 (20 Gbps) support. */
    USB_BB_SUPP,                                /**< USB Billboard device support. */
    USB_SIG_UNKNOWN                             /**< USB data signalling support unknown. */
} usb_data_sig_t;

#if (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE)
/* This verifies whether both the ports are detached or not */
bool app_deepsleep_allowed(void);
#endif /* (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE) */

#if (!CY_PD_SINK_ONLY)
/* This flag holds whether there is discharge being applied from invalid VBUS state. */
static bool gl_app_invalid_vbus_dis_on[NO_OF_TYPEC_PORTS];
#endif /* (!CCG_SINK_ONLY) */

#if (HPI_PPS_SINK_SUPPORT != 0)
static void snk_recontract_timer_cb (uint8_t port, timer_id_t id);
void app_pps_sink_disable(uint8_t port);
#endif /* APP_PPS_SINK_SUPPORT */

#if ((!CY_PD_SINK_ONLY) && (CY_PD_REV3_ENABLE))
#if (CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)

#if CY_PD_BIST_STM_ENABLE
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
static bool gl_bist_stm_entry_flag[NO_OF_TYPEC_PORTS];

static void app_bist_stm_oc_cb(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power)
{
    (void)power;

    /* Stop power throttling */
    ccg_power_throttle_set_stop_power_throttle(ptrPdStackContext, true);

    gl_bist_stm_entry_flag[ptrPdStackContext->port] = true;
}
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

static cy_en_pdstack_status_t  app_bist_stm_entry_handler(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_en_pdstack_status_t  stat = CY_PDSTACK_STAT_FAILURE;
    cy_en_pdstack_status_t stat_ls;
    uint8_t port = ptrPdStackContext->port;
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    cy_en_pdstack_status_t  stat_pt;
    gl_bist_stm_entry_flag[port] = false;
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

    /* Disable loadsharing, set min OC and disable power throttling
     * send new SRC_CAP with full capability for each port */
    stat_ls = ccg_ls_ctrl(ptrPdStackContext, true, false, NULL);

    if(CY_PDSTACK_STAT_NO_RESPONSE == stat_ls)
    {
        stat_ls = CY_PDSTACK_STAT_SUCCESS;
    }

#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    gl_pt_was_enabled[port] = ccg_power_throttle_is_enabled(ptrPdStackContext);
    /* Send SRC_CAP and disable power throttling if it was enabled */
    if(true == gl_pt_was_enabled[port])
    {
        stat_pt = ccg_power_throttle_set_oc(ptrPdStackContext, SYSTEM_OC_1, app_bist_stm_oc_cb);
    }
    else
    {
        /* No need of callback to disable power throttling */
        stat_pt = ccg_power_throttle_set_oc(ptrPdStackContext, SYSTEM_OC_1, NULL);
        if(stat_pt == CY_PDSTACK_STAT_SUCCESS)
        {
            gl_bist_stm_entry_flag[port] = true;
        }
    }
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

    if((CY_PDSTACK_STAT_SUCCESS == stat_ls)
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    && (CY_PDSTACK_STAT_SUCCESS == stat_pt)
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
    )
    {
        stat = CY_PDSTACK_STAT_SUCCESS;
    }

#if (NO_OF_TYPEC_PORTS == 1)
    /* QAC suppression 3415: The check on the right hand side of the logical operator is read only
     * and hence it does not have any side effects. */
    if((CY_PDSTACK_STAT_SUCCESS == stat) && (true == ccg_ls_is_master(ptrPdStackContext))) /* PRQA S 3415 */
    {
        stat = ccg_ls_trigger_bist_inter_silicon(ptrPdStackContext, APP_EVT_BIST_STM_ENTRY);
    }
#endif /* (NO_OF_TYPEC_PORTS == 1) */

    return stat;
}

static cy_en_pdstack_status_t  app_bist_stm_exit_handler(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_en_pdstack_status_t  stat = CY_PDSTACK_STAT_SUCCESS;

    if(true == gl_ls_was_enabled[ptrPdStackContext->port])
    {
        /* Re-enable loadsharing */
        stat = ccg_ls_ctrl(ptrPdStackContext, false, true, NULL);
    }

    if(CY_PDSTACK_STAT_NO_RESPONSE == stat)
    {
        stat = CY_PDSTACK_STAT_SUCCESS;
    }

#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    if(true == gl_pt_was_enabled[ptrPdStackContext->port])
    {
        /* Re-enable power throttling */
        ccg_power_throttle_set_stop_power_throttle(ptrPdStackContext, false);
    }
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if (NO_OF_TYPEC_PORTS == 1)
    /* QAC suppression 3415: The check on the right hand side of the logical operator is read only
     * and hence it does not have any side effects. */
    if((CY_PDSTACK_STAT_SUCCESS == stat) && (true == ccg_ls_is_master(ptrPdStackContext))) /* PRQA S 3415 */
    {
        stat = ccg_ls_trigger_bist_inter_silicon(ptrPdStackContext, APP_EVT_BIST_STM_EXIT);
    }
#endif /* (NO_OF_TYPEC_PORTS == 1) */

    if(CY_PDSTACK_STAT_SUCCESS == stat)
    {
        /* Perform Type-C error recovery */
        stat = Cy_PdStack_Dpm_SendTypecCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
    }

    return stat;
}

static void app_bist_stm_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_bist_mode_t mode, bool intended_for_master)
{
    app_status_t *app_stat;
    uint8_t i;

    auto_cfg_settings_t* ptr[NO_OF_TYPEC_PORTS];
    bool policy_mgr_en_flag[NO_OF_TYPEC_PORTS];

    bool port_valid = ((true == ccg_ls_is_master(ptrPdStackContext)) && (true == intended_for_master)) ||
                        (false == intended_for_master);

    /* Only master port is allowed to handle BIST STM entry and exit */
    if(true == port_valid)
    {
        /* Handle BIST STM entry */
        if(CY_PDSTACK_BIST_STM_ENTRY == mode)
        {
#if (NO_OF_TYPEC_PORTS > 1)
            for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
            i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
            {
                app_stat = app_get_status(i);

                /* Do not handle the command if already in BIST STM mode */
                if(false == Cy_PdStack_Dpm_GetBistStmEn(ptrPdStackContext))
                {
                    ptr[i] = pd_get_ptr_auto_cfg_tbl(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext);
                    policy_mgr_en_flag[i] = ptr[i]->policy_manager_enable;
                    gl_ls_was_enabled[i] = ccg_ls_is_enabled(solution_fn_handler->Get_PdStack_Context(i));
                    gl_ls_was_enabled[i] = gl_ls_was_enabled[i] && policy_mgr_en_flag[i];

                    if(true == gl_ls_was_enabled[i])
                    {
                        /* Set the BIST STM enabled flag */
                        Cy_PdStack_Dpm_SetBistStmEn(ptrPdStackContext, true);
                        app_stat->bist_stm_entry_pending = true;
                    }
                }
            }
        }
        /* Handle BIST STM exit */
        else if(CY_PDSTACK_BIST_STM_EXIT == mode)
        {
#if (NO_OF_TYPEC_PORTS > 1)
            for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
            i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
            {
                app_stat = app_get_status(i);

                /* Handle the command only in BIST STM mode */
                if(true == Cy_PdStack_Dpm_GetBistStmEn(ptrPdStackContext))
                {
                    /* Reset the BIST STM enabled flag */
                    Cy_PdStack_Dpm_SetBistStmEn(ptrPdStackContext, false);
                    app_stat->bist_stm_exit_pending = true;
                }
            }
        }
        else
        {
            /* No Action */
        }
    }
}
#endif /* CY_PD_BIST_STM_ENABLE */

#endif /* (CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
#endif /* ((!CY_PD_SINK_ONLY) && (CY_PD_REV3_ENABLE)) */

#if APP_PPS_SINK_SUPPORT
uint8_t hpi_user_reg_handler(uint16_t addr, uint8_t size, uint8_t *data);
void app_pps_sink_disable(uint8_t port);
#endif /* APP_PPS_SINK_SUPPORT */


#if (CCG_TYPE_A_PORT_ENABLE)
static app_status_t gl_app_status[2];
#else
static app_status_t gl_app_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || (WATCHDOG_OVER_POLL_TIMER == 1) || CCG_HPI_AUTO_CMD_ENABLE)
/* Flag to indicate that activity timer timed out. */
static volatile bool ccg_activity_timer_timeout = false;
#endif

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timeout status */
static volatile bool gl_bad_sink_timeout_status[NO_OF_TYPEC_PORTS];
static volatile bool gl_bad_sink_apdo_sel[NO_OF_TYPEC_PORTS];
/* Bad PD sink connected status */
static volatile bool gl_bad_sink_pd_status[NO_OF_TYPEC_PORTS];
static volatile bool gl_bad_sink_recovery_handled[NO_OF_TYPEC_PORTS];
#endif /* CCG_REV3_HANDLE_BAD_SINK */
bool app_validate_configtable_offsets(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint8_t  port=ptrPdStackContext->port;
    (void)port;

#if CCG_REG_SEC_CTRL
    uint32_t sil_id = 0u;
    sys_get_silicon_id(&sil_id);
#endif /* CCG_REG_SEC_CTRL */

#if (NO_OF_TYPEC_PORTS > 1)
    for (port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
#else
    port = TYPEC_PORT_0_IDX;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
    {
#if (!CY_PD_SINK_ONLY)
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_PAG1S))
        if ((get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_power_table_offset == 0u) ||
            /* QAC suppression 3415: The check on the right hand side of the logical
             * operator is read only and is only required if the previous check is not
             * sufficient to decide the result of complete expression. */
            (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_power_table_len < sizeof(pwr_params_t)) /* PRQA S 3415 */
#if CCG_REG_SEC_CTRL
            /* If the part is PAG1S secondary controlled, also check the silicon ID. */
        || ((sil_id != PAG1S_SSC_20W_SID) && (sil_id != PAG1S_SSC_33W_SID) && (sil_id != PAG1S_SSC_45W_SID))
#endif /* CCG_REG_SEC_CTRL */
)
        {
            return false;
        }

        if((uint8_t)VBUS_CTRL_TYPE_P1 != pd_get_ptr_pwr_tbl(ptrPdStackContext->ptrUsbPdContext)->fb_type)
        {
            return false;
        }
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_PAG1S)) */
#endif /* (!CY_PD_SINK_ONLY) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
        if ((get_auto_config(ptrPdStackContext->ptrUsbPdContext)->auto_config_offset == 0u) ||
            (get_auto_config(ptrPdStackContext->ptrUsbPdContext)->auto_config_size < sizeof(auto_cfg_settings_t)))
        {
            /* PORT_CCG7D. For porting only, revert once completed. */
            /* return false; */
        }
        /* No retries for Auto Table. */
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

        /* Initialize the fault-handler variables. */
        if (fault_handler_init_vars (ptrPdStackContext) == false)
        {
            return false;
        }

#if VCONN_OCP_ENABLE
        if ((get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_vconn_ocp_table_offset == 0u) ||
            (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_vconn_ocp_table_len < sizeof(vconn_ocp_settings_t)))
        {
            return false;
        }

        /* No retries for VCONN OCP. */
#endif /* VCONN_OCP_ENABLE */

#if OTP_ENABLE
        if (
                (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_otp_table_offset == 0u)
#ifndef CCG6
                ||
                (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_otp_table_len < sizeof(otp_settings_t))
#endif /* CCG6 */
           )
        {
            return false;
        }

        /* No retries for OTP. */
#endif /* OTP_ENABLE */

#if (BATTERY_CHARGING_ENABLE) || (QC_PPS_ENABLE)
#if CCG_TYPE_A_PORT_ENABLE
        if (port == TYPE_A_PORT_ID)
        {
            typeA_chg_cfg_params_t *type_a_chg_cfg_tbl = pd_get_ptr_type_a_chg_cfg_tbl(0u);
            if ((get_pd_port_config(0)->type_a_chg_tbl_offset == 0u) ||
                (type_a_chg_cfg_tbl->table_len < sizeof(typeA_chg_cfg_params_t)))
            {
                return false;
            }
        }
        else
#endif /* CCG_TYPE_A_PORT_ENABLE */
#if(!BCR) && (!QC_AFC_SNK_EN)
        {
            if ((get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_bch_table_offset == 0u)  ||
                (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_bch_table_len < sizeof(cy_stc_legacy_charging_cfg_t)))
            {
                return false;
            }
        }
#endif /* !BCR && !QC_AFC_SNK_EN */
#endif /* (BATTERY_CHARGING_ENABLE) || (QC_PPS_ENABLE) */

#if POWER_BANK
        if ((get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_bch_table_offset == 0u)  ||
            (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_bch_table_len < sizeof(cy_stc_legacy_charging_cfg_t)))
        {
            return false;
        }
#endif /* POWER_BANK */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (portconf->type_a_enable)
        {
            if ((portconf->type_a_pwr_tbl_offset == 0u)  ||
                (pd_get_ptr_type_a_pwr_tbl(port)->table_len < sizeof(pwr_params_t)))
            {
                return false;
            }
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if(BCR)
        if ((portconf->bcr_tbl_offset == 0)  ||
            (pd_get_ptr_bcr_tbl(port)->table_len < sizeof(bcr_cfg_settings_t)))
        {
            return false;
        }
#endif /* BCR */

#if (CCG_BB_ENABLE != 0)
#if defined(CY_DEVICE_CCG7D)
        /* For CCG7D, only Port 0 has billboard support */
        if(port == TYPEC_PORT_0_IDX)
#endif /* defined(CCG7D) */
        {
            /* QAC suppression 3415: The check on the right hand side of logical operator
             * is read only. */
            if ((get_auto_config(ptrPdStackContext->ptrUsbPdContext)->bb_offset == 0u)  ||
                (get_auto_config(ptrPdStackContext->ptrUsbPdContext)->bb_size < 8u)) /* PRQA S 3415 */
            {
                return false;
            }
        }
#endif /* (CCG_BB_ENABLE != 0) */
    }

    return true;
}
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) ||  (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S))
#if (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE)
static bool ccg_app_is_idle(void)
{
    /** if no features are enabled ; System is in IDLE so always return true */
    bool retVal = true;
    /*
     * If activity timer timeout event is not pending, CCG is idle and system can
     * enter low power mode.
     */
#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || (WATCHDOG_OVER_POLL_TIMER == 1) || CCG_HPI_AUTO_CMD_ENABLE)
    retVal = !ccg_activity_timer_timeout;
#endif

    return retVal;
}
#endif /* (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE) */

#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || (CCG_HPI_AUTO_CMD_ENABLE == 1) ||\
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || (WATCHDOG_OVER_POLL_TIMER == 1))
static void ccg_activity_timer_cb(cy_timer_id_t id, void *callbackContext)
{
    (void)id;
    cy_stc_pdstack_context_t * context = callbackContext;
    /*
     * Activity timer expired. Generate an event so that CCG periodic checks
     * can run.
     */
    ccg_activity_timer_timeout = true;
#if WATCHDOG_OVER_POLL_TIMER
    watchdog_timer_cb(id, callbackContext);
#endif /* WATCHDOG_OVER_POLL_TIMER */

#if CCG_APP_TASK_LPM_SLOW_POLL_ENABLE
    if(app_deepsleep_allowed())
    {
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, CY_PDUTILS_CCG_ACTIVITY_TIMER, CCG_ACTIVITY_TIMER_PERIOD_SLEEP,
            ccg_activity_timer_cb);
    }
    else
#endif /* CCG_APP_TASK_LPM_SLOW_POLL_ENABLE */
    {
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (context->ptrTimerContext, context, CY_PDUTILS_CCG_ACTIVITY_TIMER, CCG_ACTIVITY_TIMER_PERIOD,
            ccg_activity_timer_cb);
    }
}
#endif /* ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
          (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
          (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1)) */

static void ccg_app_task(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    /*
     * For multi-port controllers, we need to run this for all ports together
     * when using single activity timer. Otherwise we might encounter
     * scenarios where we will not handle task routines for a specific port
     * instance or handle it late, as the timer expiry always matches the
     * other port index run.
     * Hence one of the task instance does not react to in the specified
     * interval duration that it is supposed to.
     */
    (void)ptrPdStackContext;

#if CY_CABLE_COMP_ENABLE
    ccg_cable_comp_task(ptrPdStackContext);
#endif /* CY_CABLE_COMP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
    ccg_sensor_debounce_task(ptrPdStackContext);
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */

#if CCG_LOAD_SHARING_ENABLE
    ccg_ls_task(ptrPdStackContext);
#endif /* CCG_LOAD_SHARING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
    ccg_power_throttle_task(ptrPdStackContext);
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
        (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) || \
        (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || \
        (CCG_REG_SEC_CTRL) || (CCG_HPI_AUTO_CMD_ENABLE))
    if (ccg_activity_timer_timeout == true)
    {
        uint8_t port_index;

#if (POWER_BANK == 1)
        pb_bat_monitor ();
#endif /* (POWER_BANK == 1) */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(0)->type_a_enable)
        {
            type_a_detect_disconnect ();
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

        /* Do this for both ports as we only have one timer event. */
#if (NO_OF_TYPEC_PORTS > 1)
        for (port_index = TYPEC_PORT_0_IDX; port_index < NO_OF_TYPEC_PORTS; port_index++)
        {
#else
            port_index = TYPEC_PORT_0_IDX;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
#if (OTP_ENABLE == 1)
#if !(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            app_otp_check_temp(port_index);
#endif /* !(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#endif /* OTP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
            ccg_sensor_check(solution_fn_handler->Get_PdStack_Context(port_index));
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if (defined(CY_DEVICE_PAG1S) && CCG_REG_SEC_CTRL)
            pd_pasc_poll_task(port);
#endif /* (defined(PAG1S) && CCG_REG_SEC_CTRL) */

            ccg_activity_timer_timeout = false;
#if (NO_OF_TYPEC_PORTS > 1)        
       }
#endif /* (NO_OF_TYPEC_PORTS > 1) */       
    (void)port_index;
    }
    
#endif /* ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
        (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) || \
        (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || \
        (CCG_REG_SEC_CTRL) || (CCG_HPI_AUTO_CMD_ENABLE)) */    
}

void ccg_app_task_init(void)
{
#if (POWER_BANK == 1)
    pb_task_init ();
#endif /* (POWER_BANK == 1) */

#if OTP_ENABLE
    /* Enable OTP. */
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    uint8_t i ;
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        app_otp_enable (solution_fn_handler->Get_PdStack_Context(i));
    }
#else
        app_otp_enable (NO_OF_TYPEC_PORTS);
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
#endif /* OTP_ENABLE */

    /*
     * Start CCG activity timer. This timer is periodically used to monitor
     * battery voltage (in power bank application), TYPE-A current consumption
     * and OTP.
     */
#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || (CCG_HPI_AUTO_CMD_ENABLE == 1) ||\
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1) || (WATCHDOG_OVER_POLL_TIMER == 1))
    ccg_activity_timer_timeout = false;
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (solution_fn_handler->Get_PdStack_Context(0)->ptrTimerContext, solution_fn_handler->Get_PdStack_Context(0), CY_PDUTILS_CCG_ACTIVITY_TIMER, CCG_ACTIVITY_TIMER_PERIOD,
            ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) */
}
#endif /* ((defined(CY_DEVICE_CCG3PA)) || (defined(CCG3PA2)) || (defined(CY_DEVICE_PAG1S)) ||  (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static bool app_is_vdm_task_ready(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    /* Assume cable discovery finished when device is UFP. */
    bool retval = true;

#if DFP_ALT_MODE_SUPP

    const cy_stc_pdstack_dpm_status_t *dpm_stat = &(ptrPdStackContext->dpmStat);
    app_status_t app_stat = app_get_status(ptrPdStackContext->port);

    /* This check only makes sense for DFP. */
    if (ptrPdStackContext->dpmConfig.curPortType != PRT_TYPE_UFP)
    {
#if (ROLE_PREFERENCE_ENABLE)
        /* Don't proceed with alternate mode if DR_SWAP is pending. */
        if ((app_stat->app_pending_swaps & APP_DR_SWAP_PENDING) != 0)
        {
            return false;
        }
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if (!CY_PD_CBL_DISC_DISABLE)
        /*
         * Set the cable discovered flag if:
         * 1. Cable discovery is disabled.
         * 2. EMCA present flag in DPM is set.
         * 3. Cable discovery process not restarted.
         */
        if (
                (dpm_stat->cblDsc == false) ||
               ((dpm_stat->emcaPresent != false) &&
                (app_stat->disc_cbl_pending == false))
            )
        {
            app_stat->cbl_disc_id_finished = true;
        }

        /* Return the status of Cable discovered flag. */
        retval = app_stat->cbl_disc_id_finished;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */
    }
#else
    (void)ptrPdStackContext;
#endif /* DFP_ALT_MODE_SUPP */

    return retval;
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

cy_en_pdstack_status_t app_disable_pd_port(cy_stc_pdstack_context_t *ptrPdStackContext, cy_pdstack_dpm_typec_cmd_cbk_t cbk)
{
    cy_en_pdstack_status_t retval = CY_PDSTACK_STAT_SUCCESS;
    uint8_t port = ptrPdStackContext->port;

    if (CALL_MAP(Cy_PdUtils_SwTimer_IsRunning ) (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_FAULT_RECOVERY_TIMER)))
    {
        /* If the HPI Master is asking us to disable the port, make sure all fault protection state is cleared. */
        app_get_status(port)->fault_status &= ~(
                APP_PORT_VBUS_DROP_WAIT_ACTIVE | APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS |
                APP_PORT_VCONN_FAULT_ACTIVE | APP_PORT_V5V_SUPPLY_LOST);

        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_FAULT_RECOVERY_TIMER));
    }
    else
    {
        /* Just pass the call on-to the stack. */
        if (ptrPdStackContext->dpmConfig.dpmEnabled)
        {
            retval = CALL_MAP(Cy_PdStack_Dpm_SendTypecCommand)(ptrPdStackContext,  CY_PDSTACK_DPM_CMD_PORT_DISABLE,  cbk);
            return retval;
        }
    }

    Cy_USBPD_TypeC_DisableRd(ptrPdStackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_1);
    Cy_USBPD_TypeC_DisableRd(ptrPdStackContext->ptrUsbPdContext, CY_PD_CC_CHANNEL_2);
    (void)Cy_PdStack_Dpm_UpdateDisCounter(ptrPdStackContext, false);

    cbk(ptrPdStackContext, CY_PDSTACK_DPM_RESP_SUCCESS);

    return CY_PDSTACK_STAT_SUCCESS;
}

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timer callback */
static void app_bad_sink_timeout_cbk(cy_timer_id_t id,  void * callbackCtx)
{
    (void)id;
    cy_stc_pdstack_context_t* context = callbackCtx;
    bool fault_count_exceed = app_port_fault_count_exceeded(context);
    uint8_t port = context->port;

    /*
     * Take bad sink action only if PD connected device and PD is REV3 and
     * it is not a permanent fault condition.
     */
    if ((((context->dpmStat.pdConnected == true) ||
         (gl_bad_sink_pd_status[port] == true)) &&
         (gl_bad_sink_timeout_status[port] == false) &&
         (!fault_count_exceed)) ||
         (gl_bad_sink_apdo_sel[port] == true))
    {
        /* Set bad sink status and retain it until port detach or disable */
        gl_bad_sink_apdo_sel[port] = false;
        gl_bad_sink_timeout_status[port] = true;
#if (CCG_LOAD_SHARING_ENABLE)
        gl_bad_device[port] = true;
        ccg_power_throttle_set_stop_ls(context, true);
        ccg_power_throttle_set_stop_power_throttle(context, true);
#endif /* (CCG_LOAD_SHARING_ENABLE) */
        /* Initiate Type-C error recovery. */
        (void)CALL_MAP(Cy_PdStack_Dpm_SendTypecCommand)(context, CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        gl_bad_sink_recovery_handled[port] = true;
    }
    else if ((true == gl_bad_sink_timeout_status[port]) &&
            (false == context->dpmConfig.attach))
    {
        gl_bad_sink_timeout_status[port] = false;
    }
    else
    {
        /* No statement */
    }
}
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)))
extern void ccg_hal_check_if_vsys_up (void);
#endif

#if (!CY_PD_CBL_DISC_DISABLE)
static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext);
static void app_cbl_dsc_callback (cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == CY_PDSTACK_SEQ_ABORTED)
    {
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (ptrPdStackContext->ptrTimerContext, ptrPdStackContext , GET_APP_TIMER_ID(ptrPdStackContext,APP_CBL_DISC_TRIGGER_TIMER), APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}


static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *pdstack_context = callbackContext;

    if (CALL_MAP(Cy_PdStack_Dpm_SendPdCommand)(pdstack_context, CY_PDSTACK_DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, false, app_cbl_dsc_callback) != CY_PDSTACK_STAT_SUCCESS )
    {
        /* Start timer which will send initiate the DPM command after a delay. */
        (void)app_cbl_dsc_callback(pdstack_context, CY_PDSTACK_SEQ_ABORTED, 0);
    }
}
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

#if (DFP_ALT_MODE_SUPP)
static void app_exit_all_cbk (cy_timer_id_t id, void *callbackContext)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackContext;
    /* Reset VDM layer */
    app_vdm_layer_reset(ptrPdStackContext);
}

static void app_exit_alt_modes_cbk(cy_stc_pdstack_context_t *ptrPdStackContext, uint32_t stat)
{
    (void)stat;
    /* Run timer to run VDM layer restart */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (ptrPdStackContext->ptrTimerContext, ptrPdStackContext , GET_APP_TIMER_ID(ptrPdStackContext,APP_RESET_VDM_LAYER_TIMER), APP_RESET_VDM_TIMER_PERIOD, app_exit_all_cbk);

}
#endif /* DFP_ALT_MODE_SUPP */

bool app_vdm_layer_reset(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    bool stat = true;

    if ((ptrPdStackContext->dpmConfig.contractExist) && (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP))
    {
#if DFP_ALT_MODE_SUPP
        if (app->alt_mode_entered == false)
        {
            /*
             * Reset the alternate mode state machine. The cable discovery complete flag is also cleared so
             * that alternate mode state machine can be started at the end of cable discovery.
             */
            Cy_PdAltMode_Mngr_LayerReset((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext));
        }
        else
        {
            /* Exit all alt modes at first by sending Exit VDM */
            alt_mode_mngr_exit_all(port, true, app_exit_alt_modes_cbk);
            return stat;
        }
#endif /* DFP_ALT_MODE_SUPP */

#if (!CY_PD_CBL_DISC_DISABLE)
        ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = false;
        ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.disc_cbl_pending     = true;

        /* Ask PD stack to trigger cable discovery. */
        if (Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, false, app_cbl_dsc_callback) !=  CY_PDSTACK_STAT_SUCCESS)
        {
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (ptrPdStackContext->ptrTimerContext, ptrPdStackContext , GET_APP_TIMER_ID(ptrPdStackContext,APP_CBL_DISC_TRIGGER_TIMER), APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
        }
#endif /* (!CY_PD_CBL_DISC_DISABLE) */
    }
    else
    {
#if VCONN_OCP_ENABLE
        /* If there is no PD contract in place and we are VConn source, enable VConn and move on. */
        if ((ptrPdStackContext->dpmConfig.attach) && (ptrPdStackContext->dpmConfig.vconnLogical))
        {
            (void)vconn_enable (ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol);
        }
#endif /* VCONN_OCP_ENABLE */

        stat = false;
    }
    return stat;
#else
    (void)ptrPdStackContext;
    return false;
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if CY_PD_BIST_STM_ENABLE
    app_status_t *app_stat = app_get_status(ptrPdStackContext->port);
    cy_en_pdstack_status_t stat;
#endif /* CY_PD_BIST_STM_ENABLE */

    fault_handler_task (ptrPdStackContext);

#if CCG_REV3_HANDLE_BAD_SINK
    /*
     * Check for bad sink timeout.
     * If the PD contract was not completed for APP_BAD_SINK_TIMEOUT_TIMER_PERIOD,
     * a Type-C error recovery was initiated. Downgrade the PD revision from
     * REV3 to REV2 as expected by few bad sinks to complete the contract.
     * Bad sink status will be maintained until port disconnect/disable.
     */
    if((gl_bad_sink_timeout_status[ptrPdStackContext->port] == true) && (ptrPdStackContext->dpmStat.peFsmState == CY_PDSTACK_PE_FSM_SRC_STARTUP))
    {
        (void)Cy_PdStack_Dpm_DowngradePdRev(ptrPdStackContext);
    }
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* If VDM processing is allowed */
    if (((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_task_en != false)
    {
        /* Wait for cable discovery completion before going on Alt. Modes. */
        if (app_is_vdm_task_ready (ptrPdStackContext))
        {
            Cy_PdAltMode_VdmTask_Manager (ptrPdStackContext->ptrAltModeContext);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if NCP_CLIND_ENABLE
    if(port == TYPEC_PORT_0_IDX)
    {
        Clind_OCP_Check(port);
    }
#endif /* NCP_CLIND_ENABLE */

#if (CCG_BB_ENABLE != 0)
    if (Cy_PdAltMode_Billboard_IsPresent(ptrPdStackContext->ptrAltModeContext) != false)
    {
        Cy_PdAltMode_Billboard_Task(ptrPdStackContext->ptrAltModeContext);
    }
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE

    (void)bc_fsm(ptrPdStackContext);

#endif /* BATTERY_CHARGING_ENABLE */

#if RIDGE_SLAVE_ENABLE
    ridge_slave_task();
#endif /* RIDGE_SLAVE_ENABLE */

#if ((MUX_DELAY_EN) || (((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))))
    if (
            (gl_app_mux_update_req[port] != false) &&
            (gl_app_status[port].is_mux_busy == false)
       )
    {
        Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, gl_app_mux_saved_state[port], gl_app_mux_saved_custom_data[port]);
    }
#endif /* (MUX_DELAY_EN) || (((!RIDGE_I2C_HPD_ENABLE) && (DP_DFP_SUPP))) */

#if AMD_SUPP_ENABLE
    /* Update AMD APU */
    amd_task(port);
#endif /* AMD_SUPP_ENABLE */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Run polling tasks of CCG. */
    ccg_app_task(ptrPdStackContext);
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if (!CCG_BACKUP_FIRMWARE)
#if ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)))
    ccg_hal_check_if_vsys_up ();
#endif /* ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF))) */
#endif /* (!CCG_BACKUP_FIRMWARE) */

#ifdef CY_DEVICE_PAG1S
#if CCG_PASC_LP_ENABLE
    pd_pasc_lp_task(port);
#endif /* CCG_PASC_LP_ENABLE */
#endif /* CY_DEVICE_PAG1S */

#if CY_PD_BIST_STM_ENABLE
    uint8_t port = ptrPdStackContext->port;
    if(true == app_stat->bist_stm_entry_pending)
    {
        stat = app_bist_stm_entry_handler(ptrPdStackContext);
        if(CY_PDSTACK_STAT_SUCCESS == stat)
        {
            app_stat->bist_stm_entry_pending = false;
        }
    }
    else if(true == app_stat->bist_stm_exit_pending)
    {
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
        if(gl_bist_stm_entry_flag[port] == true)
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
        {
            stat = app_bist_stm_exit_handler(ptrPdStackContext);
            if(CY_PDSTACK_STAT_SUCCESS == stat)
            {
#if (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
                gl_bist_stm_entry_flag[port] = false;
#endif /* (CCG_HPI_AUTO_CMD_ENABLE  || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
                app_stat->bist_stm_exit_pending = false;
            }
        }
    }
    else
    {
        /* No Action */
    }
#endif /* CY_PD_BIST_STM_ENABLE */

    return 1u;
}

#if (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE)

bool app_deepsleep_allowed(void)
{
    bool out = true;
#if REGULATOR_REQUIRE_STABLE_ON_TIME
    bool reg_is_enabled;
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    uint8_t i;

    /*
     * Deepsleep mode operation can only be supported when un-attached.
     * When attached, the references and the CSA block requires to be
     * active.
     */
#if (NO_OF_TYPEC_PORTS > 1)
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
    i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
    {
#if REGULATOR_REQUIRE_STABLE_ON_TIME
        reg_is_enabled = REGULATOR_IS_ENABLED((solution_fn_handler->Get_PdStack_Context(i))->ptrUsbPdContext);
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

        if (solution_fn_handler->Get_PdStack_Context(i)->dpmConfig.attach == true
#if REGULATOR_REQUIRE_STABLE_ON_TIME
        || reg_is_enabled
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */
        )
        {
            out = false;
#if (NO_OF_TYPEC_PORTS > 1)
            break;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
        }

    }

    /* Check if dpm is idle */
    if (out == true)
    {
        Cy_PdStack_Dpm_IsIdle(solution_fn_handler->Get_PdStack_Context(i), &out);
    }

#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#ifdef CY_DEVICE_PAG1S
    /*
     * In case of secondary regulator control, we will lose regulation
     * if we go into deep sleep. To avoid this, we need to monitor VBUS_IN
     * level and based on the state of it, wake up and get regulation going
     * again. If this is not done, restricting to use of SLEEP mode only.
     */
#if ((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE))
    out = false;
#else /* !((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE)) */
    if (out == true)
    {
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
        {
            /*
             * For PAG1S, there is an additional sleep check required. Instead of adding
             * a separate function, this function is overridden to do the sleep entry
             * request also from this call.
             */
#if CCG_PASC_LP_ENABLE
            if (pd_pasc_lp_is_active(i) == false)
            {
                /* We are idle and detached. We can now start the LP mode operation. */
                pd_pasc_lp_enable(i);
            }
            if (pd_pasc_lp_ds_allowed(i) == false)
            {
                out = false;
            }
#endif /* CCG_PASC_LP_ENABLE */
        }
    }
#endif /* ((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE)) */

#endif /* PAG1S */

    return out;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

#if (NO_OF_TYPEC_PORTS > 1)
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
#else
    port = TYPEC_PORT_0_IDX;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
    {
#if CCG_BB_ENABLE
        if (!Cy_PdAltMode_Billboard_EnterDeepSleep((solution_fn_handler->Get_PdStack_Context(port))->ptrAltModeContext))
        {
            stat = false;
            return stat;
        }
#endif /* CCG_BB_ENABLE */

#if (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
        /*
         * Check if CCG polling tasks are not pending to be serviced and system can enter
         * low power mode.
         */
        if (ccg_app_is_idle () == false)
        {
            stat = false;
            return stat;
        }
#endif /*  (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

        /* Don't go to sleep while CC/SBU fault handling is pending. */
        if ((app_get_status(port)->fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
        {
            stat = false;
            return stat;
        }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        if (!Cy_PdAltMode_VdmTask_IsIdle((solution_fn_handler->Get_PdStack_Context(port))->ptrAltModeContext))
        {
            stat = false;
            return stat;
        }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Check if HPD RX Activity timer is running.
         * If yes, don't enter deep sleep. */
        if (CALL_MAP(Cy_PdUtils_SwTimer_IsRunning ) ((solution_fn_handler->Get_PdStack_Context(port))->ptrTimerContext, HPD_RX_ACTIVITY_TIMER_ID))
        {
            stat = false;
            return stat;
        }
#endif /* DP_UFP_SUPP && CCG_HPD_RX_ENABLE */

   }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        /* Prepare for deep-sleep entry. */
        Cy_PdAltMode_Mngr_Sleep((solution_fn_handler->Get_PdStack_Context(port))->ptrAltModeContext);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return stat;
}

void app_wakeup(void)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        Cy_PdAltMode_Mngr_Wakeup ((solution_fn_handler->Get_PdStack_Context(port))->ptrAltModeContext);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}
#endif /* (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE) */

#if CY_PD_REV3_ENABLE

#if CHUNKING_NOT_SUPPORTED

void app_not_supported_sent_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    if (resp == CY_PDSTACK_CMD_SENT)
    {
        if (ptrPdStackContext->peFsmState == CY_PDSTACK_PE_FSM_READY)
        {
            /* Send a timeout event to the policy engine by calling pe_tmr_cbk. */
            Cy_PdStack_Dpm_SetPeEvt(ptrPdStackContext, CY_PD_PE_EVT_TIMEOUT);
        }
    }
}

static void app_send_not_supported_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackContext;
    Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, app_not_supported_sent_cb);
}

#else /* !CHUNKING_NOT_SUPPORTED */

void extd_msg_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                                    const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    /* Update : amsType */
    static cy_en_pdstack_ams_type_t amsType[NO_OF_TYPEC_PORTS];
    (void)pkt_ptr;
    uint8_t port = ptrPdStackContext->port;

    if(resp == CY_PDSTACK_RES_RCVD)
    {
        (void)Cy_PdStack_Dpm_SetChunkXferRunning(ptrPdStackContext, amsType[port]);
    }

    if(resp == CY_PDSTACK_CMD_SENT)
    {
        amsType[port] = ptrPdStackContext->dpmStat.nonIntrResponse;
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

#endif /* CHUNKING_NOT_SUPPORTED */

bool app_extd_msg_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pd_packet_extd_t *pd_pkt_p)
{
#if CHUNKING_NOT_SUPPORTED
    /*
     * If we receive any message with more than one chunk, start ChunkingNotSupportedTimer and send NOT_SUPPORTED
     * message on expiry.
     */
    if (
            (pd_pkt_p->hdr.hdr.chunked == true) &&
            (
             (pd_pkt_p->hdr.hdr.chunkNum != 0) ||
             (pd_pkt_p->hdr.hdr.dataSize > ((pd_pkt_p->hdr.hdr.chunkNum + 1) * CY_PD_MAX_EXTD_MSG_LEGACY_LEN))
            )
       )
    {
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext,APP_CHUNKED_MSG_RESP_TIMER), 45, app_send_not_supported_cb);

        /* Stop the PD_GENERIC_TIMER to prevent premature return to ready state. */
        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,PD_GENERIC_TIMER));
    }
    else
    {
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        /* If macro is enabled - allow handling the requests from solution space. */
        return false;
#else
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                         CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
#endif /* CCG_HANDLE_EXT_MSG_IN_SOL */
    }

#else /* CHUNKING_NOT_SUPPORTED */
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == 1u) && (pd_pkt_p->hdr.hdr.dataSize >
               ((pd_pkt_p->hdr.hdr.chunkNum + ((uint32_t)1)) * CY_PD_MAX_EXTD_MSG_LEGACY_LEN)))
    {
        cy_stc_pdstack_dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmdSop = pd_pkt_p->sop;
        extd_dpm_buf.extdType = (cy_en_pdstack_extd_msg_t)pd_pkt_p->msg;
        extd_dpm_buf.extdHdr.val = 0u;
        extd_dpm_buf.extdHdr.extd.chunked = 1u;
        extd_dpm_buf.extdHdr.extd.request = 1u;
        extd_dpm_buf.extdHdr.extd.chunkNum = pd_pkt_p->hdr.hdr.chunkNum + 1u;
        extd_dpm_buf.datPtr = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout = ptrPdStackContext->senderRspTimeout;

        /* Send next chunk request */
        (void)Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,CY_PDSTACK_DPM_CMD_SEND_EXTENDED,
                                          &extd_dpm_buf, true, extd_msg_cb);
     }
    else
    {
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        /* If macro is enabled - allow handling the requests from solution space. */
        return false;
#else
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            (void)Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                              CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
#endif /* CCG_HANDLE_EXT_MSG_IN_SOL */
    }
#endif /* CHUNKING_NOT_SUPPORTED */

    return true;
}
#endif /* CY_PD_REV3_ENABLE */

#if (CCG_BB_ENABLE != 0)
/* Alternate mode entry timeout callback function. */
static void ame_tmr_cbk(cy_timer_id_t id,  void * callbackCtx)
{
    (void)id;
    cy_stc_pdstack_context_t* context = callbackCtx;
    /* Alternate modes are reset in vdm_task_mngr_deinit(). */
    Cy_PdAltMode_Billboard_Enable(context->ptrAltModeContext, BB_CAUSE_AME_TIMEOUT);
}
#endif /* (CCG_BB_ENABLE != 0) */

void app_update_bc_src_snk_support(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t enable)
{
#if BATTERY_CHARGING_ENABLE && (defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6))
    gl_app_status[ptrPdStackContext->port].bc_12_src_disabled = (bool)(!(enable & 0x01));
    gl_app_status[ptrPdStackContext->port].bc_12_snk_disabled = (bool)(!(enable & 0x02));
    if (!enable)
    {
        bc_stop(ptrPdStackContext);
    }
#else
    (void)ptrPdStackContext;
    (void)enable;
#endif /* BATTERY_CHARGING_ENABLE && (CY_DEVICE_defined(CCG5C) || defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6)) */
}

void app_update_bc_snk_support(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t disable)
{
#if BATTERY_CHARGING_ENABLE && (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_SERIES_WLC1))
#if (CY_PD_SINK_ONLY)
    gl_app_status[ptrPdStackContext->port].bc_snk_disabled = (bool)(disable);
    if (disable)
    {
        bc_stop(ptrPdStackContext);
    }
#else
    (void)ptrPdStackContext;
    (void)disable;
#endif /* (CY_PD_SINK_ONLY) */
#else
    (void)ptrPdStackContext;
    (void)disable;
#endif /* BATTERY_CHARGING_ENABLE && (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_SERIES_WLC1) */
}

#if ((CY_PD_REV3_ENABLE) && (NB_HOST_SUPPORT_ENABLE))
static const uint8_t SystemPwrStateMap[] = {
    1u,                 /* S0 */
    3u,                 /* S3 */
    4u,                 /* S4 */
    5u,                 /* S5 */
    2u,                 /* Modern Standby */
    6u,                 /* G3 */
    0u,                 /* Not supported. */
    0u                  /* Not supported. */
};
#endif /* ((CY_PD_REV3_ENABLE) && (NB_HOST_SUPPORT_ENABLE)) */

void app_update_sys_pwr_state(uint8_t state)
{
    (void)state;
#if ((CY_PD_REV3_ENABLE) && (NB_HOST_SUPPORT_ENABLE))
    uint8_t status_val = state;
    uint8_t port;

    /* If there is a change in the status value, update the port status and send an ALERT message. */
    if (state != gl_system_state)
    {
        status_val = (status_val & 0xF8u) | SystemPwrStateMap[status_val & 0x07u];

        for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
#if SEND_ALERT_ON_PWR_STATE_CHG_ENABLE
            dpm_pd_cmd_buf_t dpm_params;
            const dpm_status_t *dpm_stat = dpm_get_info(port);
#endif /* SEND_ALERT_ON_PWR_STATE_CHG_ENABLE */

            CALL_MAP(dpm_update_port_status)(port, &status_val, PD_EXTD_STATUS_PWR_STATE_CHG_OFFSET, 1);

#if SEND_ALERT_ON_PWR_STATE_CHG_ENABLE
            if (
                    (dpm_stat->contract_exist) &&
                    (dpm_stat->spec_rev_sop_live >= PD_REV3) &&
                    (dpm_stat->cur_port_type == PRT_TYPE_DFP)
               )
            {
                dpm_params.cmd_sop       = SOP;
                dpm_params.no_of_cmd_do  = 1;
                dpm_params.timeout       = 0;
                dpm_params.cmd_do[0].val = 0x07000001u;

                /* Send an ALERT message if we are DFP in a PD 3.0 contract. Ignore failure. */
                dpm_pd_command(port, DPM_CMD_SEND_ALERT, &dpm_params, NULL);
            }
#endif /* SEND_ALERT_ON_PWR_STATE_CHG_ENABLE */
        }
    }
#endif /* ((CY_PD_REV3_ENABLE) && (NB_HOST_SUPPORT_ENABLE)) */

#if BATTERY_CHARGING_ENABLE && (defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6))
#if (!CCG_SINK_ONLY)
    uint8_t i = 0;
    bool    reconnect = false;

    /*
       If BC 1.2 support is enabled and the system power state changed; proceed with further checks.
       If we are currently operating as CDP, don't make any changes.
       If we are currently operating as DCP and the new power state is still S3/S4/S5, don't make any changes.
       If we are currently operating as DCP and the new power state is S0, trigger error recovery to allow
       device enumeration (only if no alternate modes are active).
     */
    if (
            (gl_app_status[i].bc_12_src_disabled == 0) &&
            (gl_system_state != state)
       )
    {
#if CCG_PD_DUALPORT_ENABLE
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#endif /* CCG_PD_DUALPORT_ENABLE */
        {
            if (
                    (dpm_get_info(i)->attach) &&
                    (dpm_get_info(i)->cur_port_role == PRT_ROLE_SOURCE) &&
                    ((alt_mode_get_status(i) & APP_ALT_MODE_STAT_MASK) == 0)
               )
            {
                switch (gl_system_state)
                {
                    case NB_SYS_PWR_STATE_S0:
                    case NB_SYS_PWR_STATE_S3:
                        {
                            if ((state == NB_SYS_PWR_STATE_S0) || (state == NB_SYS_PWR_STATE_S3))
                            {
                                if (bc_port_is_cdp(i) == false)
                                {
                                    reconnect = true;
                                }
                            }
                            else
                            {
                                if (bc_port_is_cdp(i) != false)
                                {
                                    reconnect = true;
                                }
                            }
                        }
                        break;

                    case NB_SYS_PWR_STATE_S4:
                    case NB_SYS_PWR_STATE_S5:
                        {
                            if (state == NB_SYS_PWR_STATE_S0)
                            {
                                reconnect = true;
                            }
                        }
                        break;

                    default:
                        break;
                }

                if (reconnect)
                {
                    Cy_PdStack_Dpm_SendTypecCommand (i, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
                }
            }
        }
    }
#endif /* (!CCG_SINK_ONLY) */
#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6)) */

#if ((BATTERY_CHARGING_ENABLE) || (BB_RETIMER_ENABLE) || (NB_HOST_SUPPORT_ENABLE))
    gl_system_state = state;
#endif /* ((BATTERY_CHARGING_ENABLE) || (BB_RETIMER_ENABLE) || (NB_HOST_SUPPORT_ENABLE)) */

#if BB_RETIMER_ENABLE
#if 0
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_0_IDX)->icl_dual_retimer_enable != 0)
    {
        /* We're not checking if any other retimer write is pending/about to happen.
         * No code space for any fine tuned checks. So do a raw retimer write */
        retimer_set_evt(TYPEC_PORT_0_IDX, RT_EVT_UPDATE_STATUS);
    }
#if CCG_PD_DUALPORT_ENABLE
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(TYPEC_PORT_1_IDX)->icl_dual_retimer_enable != 0)
    {
        retimer_set_evt(TYPEC_PORT_1_IDX, RT_EVT_UPDATE_STATUS);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
#else
    retimer_sys_pwr_update();
#endif
#endif /* BB_RETIMER_ENABLE */
}

#if (BATTERY_CHARGING_ENABLE && ((defined(CY_DEVICE_CCG5)) || (((defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_SERIES_WLC1) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && defined(CCG_CDP_EN)))))

/* Function to start the CCG3PA/CCG5 BC 1.2 source state machine. */
void app_bc_12_sm_start(cy_stc_pdstack_context_t *ptrPdStackContext)
{

#if (((defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && defined(CCG_CDP_EN)))
    /* Proceed with CCG3PA BC1.2 CDP source state machine */
    Cy_USBPD_Bch_CdpEn(ptrPdStackContext->ptrUsbPdContext);
#else /* !(((defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && defined(CCG_CDP_EN))) */

/* Proceed with CCG5 BC1.2 Source State Machine.
 * Enable CDP is system power state is S0, otherwise enable DCP.
 */
#if CCG_HPI_ENABLE
    if (hpi_get_sys_pwr_state () == NB_SYS_PWR_STATE_S0)
#endif /* CCG_HPI_ENABLE */
    {
        Cy_USBPD_Bch_CdpEn(ptrPdStackContext->ptrUsbPdContext);
    }
#if CCG_HPI_ENABLE
    else
    {
        Cy_USBPD_Bch_DcpEn(ptrPdStackContext->ptrUsbPdContext);
    }
#endif /* CCG_HPI_ENABLE */
#endif /* (((defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && defined(CCG_CDP_EN))) */
}

#endif /* (BATTERY_CHARGING_ENABLE && ((defined(CY_DEVICE_CCG5)) || (((defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_SERIES_WLC1) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && defined(CCG_CDP_EN))))) */

#if HOST_ALERT_MSG_ENABLE
static uint32_t gl_get_bat_status[NO_OF_TYPEC_PORTS];
#endif /* HOST_ALERT_MSG_ENABLE */

#if (ROLE_PREFERENCE_ENABLE)

/* Variable storing current preference for data role. */
volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];

#if (POWER_ROLE_PREFERENCE_ENABLE)
/* Variable storing current preference for power role. */
volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

/* Forward declaration of function to trigger swap operations. */
static void app_initiate_swap (uint8_t port, timer_id_t id);

static void app_role_swap_resp_cb (
        uint8_t            port,
        resp_status_t      resp,
        const pd_packet_t *pkt_ptr)
{
    app_status_t *app_stat = &gl_app_status[port];
#if (POWER_ROLE_PREFERENCE_ENABLE)
    bool next_swap = false;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    if (resp == RES_RCVD)
    {
        if (pkt_ptr->hdr.hdr.msg_type == CTRL_MSG_WAIT)
        {
            app_stat->actv_swap_count++;
            if (app_stat->actv_swap_count < APP_MAX_SWAP_ATTEMPT_COUNT)
            {
                timer_start (port, APP_INITIATE_SWAP_TIMER, app_stat->actv_swap_delay, app_initiate_swap);
            }
            else
            {
#if (POWER_ROLE_PREFERENCE_ENABLE)
                /* Swap attempts timed out. Proceed with next swap. */
                next_swap = true;
#else
                app_stat->app_pending_swaps = 0;
                app_stat->actv_swap_type  = 0;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
            }
        }
        else
        {
#if (POWER_ROLE_PREFERENCE_ENABLE)
            /* Swap succeeded or failed. Proceed with next swap. */
            next_swap = true;
#else
            app_stat->app_pending_swaps = 0;
            app_stat->actv_swap_type  = 0;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        }
    }
    else if ((resp == CMD_FAILED) || (resp == SEQ_ABORTED) || (resp == RES_TIMEOUT))
    {
        timer_start (port, APP_INITIATE_SWAP_TIMER, app_stat->actv_swap_delay, app_initiate_swap);
    }

#if (POWER_ROLE_PREFERENCE_ENABLE)
    if (next_swap)
    {
        /* Clear the LS bit in the app_pending_swaps flag as it has completed or failed. */
        app_stat->app_pending_swaps &= (app_stat->app_pending_swaps - 1);

        app_stat->actv_swap_type  = 0;
        app_stat->actv_swap_count = 0;
        timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
    }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
}

static void app_initiate_swap (uint8_t port, timer_id_t id)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    app_status_t *app_stat_p = app_get_status(port);
    dpm_pd_cmd_buf_t pd_cmd_buf;

    uint8_t actv_swap     = app_stat_p->actv_swap_type;
    uint8_t swaps_pending = app_stat_p->app_pending_swaps;

    /* Stop the timer that triggers swap operation. */
    timer_stop (port, APP_INITIATE_SWAP_TIMER);

    /* Nothing to do if we are not in PD contract. */
    if (!dpm_stat->contract_exist)
        return;

    if (actv_swap == 0)
    {
#if (POWER_ROLE_PREFERENCE_ENABLE)
        /* No ongoing swap operation. Pick the next pending swap from the list. */
        if ((swaps_pending & APP_VCONN_SWAP_PENDING) != 0)
        {
            actv_swap = DPM_CMD_SEND_VCONN_SWAP;
            app_stat_p->actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
        }
        else
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        {
            if ((swaps_pending & APP_DR_SWAP_PENDING) != 0)
            {
                actv_swap = DPM_CMD_SEND_DR_SWAP;
                app_stat_p->actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
            }
#if (POWER_ROLE_PREFERENCE_ENABLE)
            else
            {
                if (swaps_pending != 0)
                {
                    actv_swap = DPM_CMD_SEND_PR_SWAP;
                    app_stat_p->actv_swap_delay = APP_INITIATE_PR_SWAP_TIMER_PERIOD;
                }
            }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        }

        app_stat_p->actv_swap_count = 0;
    }

    if (actv_swap != 0)
    {
        /* Check whether the selected swap is still valid. */
        switch (actv_swap)
        {
#if (POWER_ROLE_PREFERENCE_ENABLE)
            case DPM_CMD_SEND_VCONN_SWAP:
                if (dpm_stat->vconn_logical)
                {
                    app_stat_p->app_pending_swaps &= ~APP_VCONN_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

            case DPM_CMD_SEND_DR_SWAP:
                /* Stop sending DR_SWAP if any alternate mode has been entered. */
                if (
                        (dpm_stat->cur_port_type == app_pref_data_role[port]) ||
                        (app_stat_p->alt_mode_entered != 0)
                   )
                {
                    app_stat_p->app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;

#if (POWER_ROLE_PREFERENCE_ENABLE)
            case DPM_CMD_SEND_PR_SWAP:
                if (dpm_stat->cur_port_role == app_pref_power_role[port])
                {
                    app_stat_p->app_pending_swaps &= ~APP_PR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

            default:
                actv_swap = 0;
                break;
        }

        if (actv_swap == 0)
        {
            /*
             * Currently selected SWAP is no longer relevant. Re-run function to identify the next swap to be
             * performed.
             */
            if (app_stat_p->app_pending_swaps != 0)
            {
                app_stat_p->actv_swap_type = 0;
                timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
        else
        {
            /* Store the swap command for use in the callback. */
            app_stat_p->actv_swap_type = actv_swap;

            /* Only packet type needs to be set when initiating swap operations. */
            pd_cmd_buf.cmd_sop = SOP;

            /* Try to trigger the selected swap operation. */
            if (dpm_pd_command(port, actv_swap, &pd_cmd_buf, app_role_swap_resp_cb) != CCG_STAT_SUCCESS)
            {
                /* Retries in case of AMS failure can always be done with a small delay. */
                timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
    }
}

/* This function is called at the end of a PD contract to check whether any role swaps need to be triggered. */
void app_contract_handler (uint8_t port)
{
    app_status_t *app_stat = &gl_app_status[port];
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint16_t delay_reqd = APP_INITIATE_PR_SWAP_TIMER_PERIOD;

#if (POWER_ROLE_PREFERENCE_ENABLE)
    /* Check if we need to go ahead with PR-SWAP. */
    if (
            (app_pref_power_role[port] == PRT_DUAL) ||
            (dpm_get_info(port)->cur_port_role == app_pref_power_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_PR_SWAP_PENDING;
    }
    else
    {
        /* If we are about to swap to become source, ensure VConn Swap is done as required. */
        if ((app_pref_power_role[port] == PRT_ROLE_SOURCE) && (dpm_stat->vconn_logical == 0))
        {
            app_stat->app_pending_swaps |= APP_VCONN_SWAP_PENDING;
        }
    }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    /* Check if we need to go ahead with DR-SWAP. */
    if (
            (app_pref_data_role[port] == PRT_TYPE_DRP) ||
            (dpm_get_info(port)->cur_port_type == app_pref_data_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_DR_SWAP_PENDING;
    }
    else
    {
        /* DR-SWAPs need to be initiated as soon as possible. VConn swap will be triggered after DR_SWAP as needed. */
        delay_reqd = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
    }

    /* Start a timer that will kick of the Swap state machine. */
    timer_start (port, APP_INITIATE_SWAP_TIMER, delay_reqd, app_initiate_swap);
}

void app_connect_change_handler (uint8_t port)
{
    /* Stop all timers used to trigger swap operations. */
    timer_stop (port, APP_INITIATE_SWAP_TIMER);

#if (POWER_ROLE_PREFERENCE_ENABLE)
    /* Assume that PR_SWAP and DR_SWAP are pending. The actual status will be updated on contract completion. */
    gl_app_status[port].app_pending_swaps = APP_PR_SWAP_PENDING | APP_DR_SWAP_PENDING;
#else
    /* Assume that DR_SWAP is pending. The actual status will be updated on contract completion. */
    gl_app_status[port].app_pending_swaps = APP_DR_SWAP_PENDING;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    gl_app_status[port].actv_swap_type    = 0;
    gl_app_status[port].actv_swap_count   = 0;
}

#endif /* (ROLE_PREFERENCE_ENABLE) */

#if PD_SRC_POWER_SHARE_ENABLE

static bool     app_pwr_share_enable     = false;
static uint32_t app_sgl_port_cur_limit   = I_1P5A;
static uint8_t  app_src_extra_power_port = NO_OF_TYPEC_PORTS;
static uint8_t  app_src_pdp_default[NO_OF_TYPEC_PORTS] = {
    15
#if CCG_PD_DUALPORT_ENABLE
    ,
    15
#endif /* CCG_PD_DUALPORT_ENABLE */
};

static void app_pwr_share_recontract_cb (uint8_t port, timer_id_t id);

static void app_src_cap_change_cb(uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    /* If the attempt to trigger a new contract failed, queue another attempt for later. */
    if ((resp == CMD_FAILED) || (resp == SEQ_ABORTED))
    {
        timer_start(port, APP_POWER_SHARE_RECONTRACT_TIMER_ID, 10, app_pwr_share_recontract_cb);
    }
}

static void app_pwr_share_recontract_cb (uint8_t port, timer_id_t id)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    dpm_pd_cmd_buf_t    cmd_param;

    if ((dpm_stat->contract_exist) && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
    {
        /* Notify the policy engine that source caps have been changed. */
        cmd_param.cmd_sop       = SOP;
        cmd_param.no_of_cmd_do  = 0;
        cmd_param.timeout       = 0;

        if (dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, &cmd_param, app_src_cap_change_cb) !=
                CCG_STAT_SUCCESS)
        {
            timer_start(port, APP_POWER_SHARE_RECONTRACT_TIMER_ID, 10, app_pwr_share_recontract_cb);
        }
    }
}

static void app_update_src_caps(uint8_t port, uint32_t max_cur)
{
    dpm_status_t           *dpm_stat = dpm_get_status(port);
    pd_do_t                 src_cap;
    uint8_t                 cur_lvl = DPM_CMD_SET_RP_1_5A;
    uint8_t                 pdp_val = 7;                        /* 5V@1.5A ==> 7.5 W */
    uint8_t                 pdomask;

    /* Start with previous PDO mask value. */
    pdomask = dpm_stat->src_pdo_mask;

    /* Update Rp to reflect the power reserved for this port. */
    if (max_cur >= I_3A)
    {
        cur_lvl = DPM_CMD_SET_RP_3A;
        pdp_val = 15;                       /* 5V@3A ==> 15 W. */
    }
    if (max_cur < I_1P5A)
    {
        cur_lvl = DPM_CMD_SET_RP_DFLT;
        pdp_val = 4;                        /* 5V@900mA = 4.5 W. */
    }

    pdomask = ((pdomask & 0x80) | 1);
    src_cap = dpm_stat->src_pdo[0];
    src_cap.fixed_src.max_current = max_cur;

    /* Directly update the DPM-STATUS structures. */
    dpm_stat->src_cur_level = cur_lvl;
    dpm_stat->src_pdo[0]    = src_cap;
    dpm_stat->src_pdo_mask  = pdomask;
    dpm_stat->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX] = pdp_val;

    if ((dpm_stat->contract_exist) && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
    {
        /* Start timer to trigger re-contract with changed power allocation. */
        timer_start (port, APP_POWER_SHARE_RECONTRACT_TIMER_ID, 10, app_pwr_share_recontract_cb);
    }
}

static void app_power_share_update(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /* Skip actions when power sharing is disabled. */
    if (!app_pwr_share_enable)
    {
        return;
    }

    /*
     * If we are source, sink is reporting capability mismatch, and spare power is available; increase the
     * power allocation for this port.
     */
    if (
            (dpm_stat->contract_exist) &&
            (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
       )
    {
        if (
                (dpm_stat->src_rdo.rdo_fix_var.cap_mismatch) &&
                (app_src_extra_power_port == NO_OF_TYPEC_PORTS)
           )
        {
            /* Allocate the power reserve to this port. */
            app_src_extra_power_port = port;

            /* Update PDOs and make a new contract. */
            app_update_src_caps (port, I_3A);
        }
    }
    else
    {
        if (app_src_extra_power_port == port)
        {
            /* Free up the extra power that was allocated to this port. */
            app_src_extra_power_port = NO_OF_TYPEC_PORTS;

            /* Update PDOs and make a new contract. */
            app_update_src_caps (port, app_sgl_port_cur_limit);
        }
    }
}

void app_power_share_init(void)
{
    const pd_port_config_t *conf_p = get_pd_port_config(0);
    pd_do_t src_pdo;

    /* Nothing to do on a single port device. */
    if (NO_OF_TYPEC_PORTS == 1)
    {
        return;
    }

    app_pwr_share_enable = true;
    app_sgl_port_cur_limit = I_1P5A;

    /* Pick the default power allocation per port from the config table. */
    src_pdo.val = conf_p->src_pdo_list[0];
    if (src_pdo.fixed_src.max_current < I_1P5A)
    {
        app_sgl_port_cur_limit = I_0P9A;
    }

    /* Store the original PDP values for each port. */
    app_src_pdp_default[0] = dpm_get_info(0)->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX];
    app_src_pdp_default[1] = dpm_get_info(1)->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX];

    if (app_src_extra_power_port >= NO_OF_TYPEC_PORTS)
    {
        /* Initially allocate limited power per port. */
        app_update_src_caps (0, app_sgl_port_cur_limit);
        app_update_src_caps (1, app_sgl_port_cur_limit);
    }
}

#endif /* PD_SRC_POWER_SHARE_ENABLE */

#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE))
static void app_check_usb_supp(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /* Check port partner's USB capabilities */
    if (
#if (!CCG_SINK_ONLY)
         ((dpm_stat->cur_port_role == PRT_ROLE_SOURCE)                 &&
          ((dpm_stat->src_rdo.rdo_gen.usb_comm_cap == false)           ||
          (dpm_stat->cur_src_pdo[0].fixed_src.usb_comm_cap == false))) ||
         ((dpm_stat->cur_port_role == PRT_ROLE_SINK)                   &&
          ((dpm_stat->snk_rdo.rdo_gen.usb_comm_cap == false)           ||
          (dpm_stat->src_cap_p->dat[0].fixed_src.usb_comm_cap == false)))
#else
          ((dpm_stat->snk_rdo.rdo_gen.usb_comm_cap == false)           ||
          (dpm_stat->src_cap_p->dat[0].fixed_src.usb_comm_cap == false))
#endif /* (!CCG_SINK_ONLY) */
       )
    {
        gl_app_status[port].usb2_supp = false;
        gl_app_status[port].usb3_supp = false;
    }
}
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE) */

usb_data_sig_t app_get_cable_usb_cap(uint8_t port)
{
    usb_data_sig_t usbcap = USB_SIG_UNKNOWN;
    (void)port;
#if (!(CY_PD_CBL_DISC_DISABLE))
    cy_stc_pdstack_context_t * context = solution_fn_handler->Get_PdStack_Context(port);
    cy_stc_pdstack_dpm_status_t *dpm_stat = &(context->dpmStat);
    cy_stc_pd_dpm_config_t *dpm_cfg = &(context->dpmConfig);

    if (dpm_cfg->emcaPresent)
    {
        if ((dpm_stat->cblType == CY_PDSTACK_PROD_TYPE_ACT_CBL) && (dpm_stat->cblVdo.act_cbl_vdo1.vdoVersion == CY_PD_CBL_VDO_VERS_1_2))
        {
            if (dpm_stat->cblVdo.act_cbl_vdo2.usb2Supp == (uint8_t)false)
            {
                usbcap = USB_2_0_SUPP;
            }
            if (dpm_stat->cblVdo.act_cbl_vdo2.ssSupp == (uint8_t)false)
            {
                usbcap = (dpm_stat->cblVdo2.act_cbl_vdo2.usbGen == (uint8_t)false) ? USB_GEN_1_SUPP : USB_GEN_2_SUPP;
            }
        }
        else
        {
            /* By default, calculate USB signalling from Passive/Active Cable VDO (#1). */
            usbcap = (usb_data_sig_t)(dpm_stat->cblVdo.std_cbl_vdo.usbSsSup);
        }
    }
#endif /* (!(CY_PD_CBL_DISC_DISABLE)) */

    return (usbcap);
}

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

static bool gl_app_hard_rst_sent_rcvd[NO_OF_TYPEC_PORTS] = {
    false
#if CCG_PD_DUALPORT_ENABLE
    ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

#if (MUX_DELAY_EN)
void app_hard_rst_retimer_cbk (uint8_t port, timer_id_t id)
{
    app_event_handler(port, APP_EVT_HR_SENT_RCVD_DEFERRED, NULL);
}
#endif /* (MUX_DELAY_EN) */

#if (!CY_PD_SINK_ONLY)

static void app_psrc_invalid_vbus_dischg_disable(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    if (gl_app_invalid_vbus_dis_on[ptrPdStackContext->port] == true)
    {
        gl_app_invalid_vbus_dis_on[ptrPdStackContext->port] = false;
        solution_fn_handler->app_get_callback_ptr(ptrPdStackContext)->vbus_discharge_off(ptrPdStackContext);
        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_PSOURCE_DIS_TIMER));
    }
}

/*
 * Timer callback function in case of timeout from VBUS discharge when trying to remove
 * invalid VBUS voltage from the bus.
 */
static void app_psrc_invalid_vbus_tmr_cbk(cy_timer_id_t id,  void * callbackCtx)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackCtx;
    /* Disable the discharge if it is running. */
    app_psrc_invalid_vbus_dischg_disable(ptrPdStackContext);
}

#endif /* (!CY_PD_SINK_ONLY) */

static void app_complete_data_reset(uint8_t port)
{
#if AMD_SUPP_ENABLE
    /* Make sure the tDataResetFail timer is stopped. */
    timer_stop(port, APP_DATA_RESET_FAIL_TIMER);
#else
    (void)port;
#endif /* AMD_SUPP_ENABLE */

}

#if AMD_SUPP_ENABLE
static void app_error_recovery_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    (void) port;
    (void) resp;
    /* Do nothing. */
}

static void app_data_reset_fail_timer_cb(uint8_t port, timer_id_t id)
{
    (void) id;
    /* Initiate error recovery since tDataResetFail timer has elapsed. */
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, app_error_recovery_cb);
}

static void amd_data_rst_cbk(uint8_t port, app_resp_t* resp)
{
    (void) resp;
    app_complete_data_reset(port);
}
#endif /* AMD_SUPP_ENABLE */

static void app_data_reset_timer_cb(cy_timer_id_t id,  void * callbackCtx)
{
    (void) id;
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackCtx;

    /* Send a timeout to the policy engine if it is still in the data reset states. */
    if ((ptrPdStackContext->dpmStat.peFsmState == CY_PDSTACK_PE_FSM_EVAL_DATA_RESET) || (ptrPdStackContext->dpmStat.peFsmState == CY_PDSTACK_PE_FSM_SEND_DATA_RESET))
    {
        /* DFP needs to re-enable USBx connections at completion of Data_Reset. */
        if (ptrPdStackContext->dpmConfig.curPortType  == CY_PD_PRT_TYPE_DFP)
        {
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
            Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0);
#endif
        }

#if AMD_SUPP_ENABLE
        if (dpm_stat->cur_port_type == CY_PD_PRT_TYPE_DFP)
        {
            /*
             * Start the tDataResetFail timer which will trigger error recovery if MUX configuration could not be
             * completed on time.
             */
            timer_start(port, APP_DATA_RESET_FAIL_TIMER, APP_DATA_RESET_FAIL_TIMEOUT, app_data_reset_fail_timer_cb);

            /* Run APU state monitoring to complete sata reset procedure when APU will be configures as USB SS */
            amd_reg_data_rst_cbk(port, amd_data_rst_cbk);
        }
        else
        {
            app_complete_data_reset(port);
        }

#else
        app_complete_data_reset(ptrPdStackContext->port);
#endif /* AMD_SUPP_ENABLE */
    }
}

void app_data_reset_accepted (cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if AMD_SUPP_ENABLE
    uint8_t port = ptrPdStackContext->port;
    if (app_get_status(port)->apu_reset_pending != false)
    {
        /* Allow APU processing */
        app_get_status(port)->mux_stat = MUX_STATE_IDLE;
    }
#endif /* AMD_SUPP_ENABLE */

    /* Make sure any Alternate Mode states have been cleared. */
    Cy_PdAltMode_Mngr_LayerReset ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext));

    /* Mark USB4 not active as Data Reset has been accepted. */
    ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.usb4_active = false;

#if (!CY_PD_CBL_DISC_DISABLE)
    /* Cable discovery needs to be repeated after data reset is complete. */
    ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

    if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP)
    {
        /* Start the tDataReset timer in case of DFP. */
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_DATA_RESET_TIMER), APP_DATA_RESET_TIMER_PERIOD, app_data_reset_timer_cb);
        /* Isolate the USBx connections as soon as DATA_RESET has been accepted. */
#if AMD_SUPP_ENABLE
        Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, AMD_DATA_RESET_B0_MASK);
#else
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
        Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, 0u);
#endif
#endif /* AMD_SUPP_ENABLE */
    }
    else
    {
        /* Switch to USB only configuration once data reset has been accepted. */
#if AMD_SUPP_ENABLE
        Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, AMD_DATA_RESET_B0_MASK);
#else
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
        Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
#endif
#endif /* AMD_SUPP_ENABLE */
    }
}

void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void* dat)
{
    const cy_en_pdstack_app_req_status_t* result;
    const cy_stc_pdstack_pd_contract_info_t* contract_status;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    uint8_t port = ptrPdStackContext->port;
    app_status_t *app_stat = app_get_status(port);

#if CY_PD_BIST_STM_ENABLE
    bool intended_for_master;
#endif /* CY_PD_BIST_STM_ENABLE */

#if CY_PD_REV3_ENABLE
#if HOST_ALERT_MSG_ENABLE
    cy_pd_pd_do_t alert_ado;
#endif /* HOST_ALERT_MSG_ENABLE */
#endif /* CY_PD_REV3_ENABLE */

#if CCG_LOAD_SHARING_ENABLE
    cy_pd_pd_do_t src_sel_pdo;
    uint32_t max_volt;
    bool async_change = false;
#endif /* CCG_LOAD_SHARING_ENABLE */

#if ((CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) && CCG_REV3_HANDLE_BAD_SINK)
    bool pwr_throttle_cmd_pending = ccg_power_throttle_get_power_throttle_cmd_pending(ptrPdStackContext);
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S))
#if (POWER_BANK == 1) || (CY_PD_SINK_ONLY == 1)
            /*
             * In power bank case, if device is powered by external VDDD (i.e not
             * in dead battery), disable internal VBUS regulator.
             */
            if (ptrPdStackContext->dpmStat.deadBat == false)
            {
                Cy_USBPD_DisableVsysReg (ptrPdStackContext->ptrUsbPdContext);
            }
#endif /* (POWER_BANK == 1) || (CY_PD_SINK_ONLY == 1) */
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S)) */

#if (MUX_TYPE != NONE_MUX)
            /* Initialize the MUX to its default settings (isolate). */
            (void)solution_fn_handler->mux_ctrl_init (ptrPdStackContext);
#endif /* (MUX_TYPE != NONE_MUX) */
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_prcs_failed = false;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
#if BB_RETIMER_ENABLE
            if (dpm_stat->dead_bat == true)
            {
                /* Enable the retimer power and release it from reset. */
                retimer_enable(port, false);
            }
            else
            {
                /* Make sure the retimer is disabled by default. */
                retimer_disable(port, false);
            }
#endif /* BB_RETIMER_ENABLE */
            break;

        case APP_EVT_TYPEC_ATTACH:
#if (!CY_PD_SINK_ONLY)
            /*
             * Check and disable VBUS discharge if we had enabled discharge due to invalid VBUS voltage.
             * Since the check is already there inside the function, need not repeat it here.
             */
            app_psrc_invalid_vbus_dischg_disable(ptrPdStackContext);
#endif /* (!CY_PD_SINK_ONLY) */

#if CCG_REV3_HANDLE_BAD_SINK
            /* Start bad sink timer */
            if (!(CALL_MAP(Cy_PdUtils_SwTimer_IsRunning ) (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER)) && (true == gl_bad_sink_apdo_sel[port])))
            {
                CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER));
                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER), APP_BAD_SINK_TIMEOUT_TIMER_PERIOD, app_bad_sink_timeout_cbk);
            }
#if (CCG_LOAD_SHARING_ENABLE)
            if (true == gl_bad_device[port])
            {
                ccg_power_throttle_set_stop_ls(ptrPdStackContext, false);
                ccg_power_throttle_set_stop_power_throttle(ptrPdStackContext, false);
                gl_bad_device[port] = false;
            }
#endif /* (CCG_LOAD_SHARING_ENABLE) */
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE))
            /* Set USB2/USB3 connection bits to 1 by default. */
            gl_app_status[port].usb2_supp = true;
            gl_app_status[port].usb3_supp = true;
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE) */

            /* This will also enable the USB (DP/DM) MUX where required. */
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
            Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
#endif
            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (ptrPdStackContext->dpmConfig.polarity != gl_app_previous_polarity[port])
            {
                fault_handler_clear_counts (ptrPdStackContext);
            }
            gl_app_previous_polarity[port] = ptrPdStackContext->dpmConfig.polarity ;
            break;

        case APP_EVT_CONNECT:
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_prcs_failed = false;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
#if (!CY_PD_CBL_DISC_DISABLE)
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = false;
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.disc_cbl_pending = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

#if (AUVDM_SUPPORT!= 0)
            ap_mode_init(port);
#endif

#if ((DEBUG_ACCESSORY_SNK_ENABLE) || (DEBUG_ACCESSORY_SRC_ENABLE))
            if (
                    (ptrPdStackContext->dpmConfig.attachedDev == DEV_DBG_ACC)
                    &&
#if !DEBUG_ACCESSORY_SRC_ENABLE
                    (ptrPdStackContext->dpmConfig.curPortRole == PRT_ROLE_SINK)
#elif !DEBUG_ACCESSORY_SNK_ENABLE
                    (ptrPdStackContext->dpmConfig.curPortRole == PRT_ROLE_SOURCE)
#else /* DEBUG_ACCESSORY_SRC_ENABLE == 1 && DEBUG_ACCESSORY_SNK_ENABLE == 1 */
                    (1)
#endif /* !DEBUG_ACCESSORY_SRC_ENABLE */
                )
            {
#if DEBUG_ACCESSORY_SRC_ENABLE
                if(ptrPdStackContext->dpmConfig.curPortRole == PRT_ROLE_SOURCE)
                {
                    solution_fn_handler->app_get_callback_ptr(ptrPdStackContext)->psrc_enable(port, NULL);
                }
#endif /*DEBUG_ACCESSORY_SRC_ENABLE*/

#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE))
                /* Enable USB2/3 bits when connected to a Debug Accessory. */
                gl_app_status[port].usb2_supp = true;
                gl_app_status[port].usb3_supp = true;
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE) */

                app_stat->debug_acc_attached = true;
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
            }
#endif /* ((DEBUG_ACCESSORY_SNK_ENABLE) || (DEBUG_ACCESSORY_SRC_ENABLE)) */

#if (CCG_BB_ENABLE != 0)
            /* Enable the AME timer on attach if in sink mode. */
            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
                if (Cy_PdAltMode_Billboard_BindToPort(ptrPdStackContext->ptrAltModeContext) == CY_PDSTACK_STAT_SUCCESS)
                {
                    /* Start the AME timer in any case as the MUX needs to be setup in the callback in some cases. */
                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            (cy_timer_id_t)APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);

                    /* Leave self power status flag cleared as we do not have a PD contract in place. */
                    Cy_PdAltMode_Billboard_UpdateSelfPwrStatus (ptrPdStackContext->ptrAltModeContext, 0);
                }
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if (ROLE_PREFERENCE_ENABLE)
            app_connect_change_handler (port);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            hardreset_cplt = true;
            /* QAC suppression 2003: Intentional fall through */
            /* Intentional fall-through. */
        case APP_EVT_HR_SENT_RCVD_DEFERRED:                 /* PRQA S 2003 */
        case APP_EVT_HARD_RESET_SENT:                       /* Intentional fall through */
        case APP_EVT_PE_DISABLED:                       /* Intentional fall through */
            typec_only = ((ptrPdStackContext->dpmStat.pdConnected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */
            /* QAC suppression 2003: Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:                       /* PRQA S 2003 */
        case APP_EVT_VBUS_PORT_DISABLE:                     /* Intentional fall through */
        case APP_EVT_DISCONNECT:                     /* Intentional fall through */
        case APP_EVT_TYPE_C_ERROR_RECOVERY:                     /* Intentional fall through */
            /* Intentional fall-through. */
#if (AUVDM_SUPPORT != 0)
            exit_all_ap_modes(port);
#endif

#if (!CY_PD_CBL_DISC_DISABLE)
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = false;
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.disc_cbl_pending = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

#if CCG_REV3_HANDLE_BAD_SINK
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Stop bad sink timer and clear bad sink status. */
                if (!(CALL_MAP(Cy_PdUtils_SwTimer_IsRunning ) (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER)) && (true == gl_bad_sink_apdo_sel[port])))
                {
                    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER));
                }
                if ((false != gl_bad_sink_timeout_status[port]) && (false != gl_bad_sink_recovery_handled[port]))
                {
                    /*
                     * We have detected a bad sink and a detach is being detected.
                     * In this case, we need to defer clearing this flag until we
                     * are sure this is a physical detach.
                     */
                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER), 300u, app_bad_sink_timeout_cbk);
                }
                else
                {
                    gl_bad_sink_timeout_status[port] = false;
                    gl_bad_sink_recovery_handled[port] = false;
                }
                gl_bad_sink_pd_status[port] = false;
#if (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK)
                if (false == gl_bad_sink_apdo_sel[port])
                {
                    gl_bad_device[port] = false;
                }
#endif /* (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK) */
            }
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
            if((evt == APP_EVT_DISCONNECT) && (true == pwr_throttle_cmd_pending))
            {
                ccg_power_throttle_set_power_throttle_renegotiation_complete(ptrPdStackContext, true, false);
            }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S))
#if (POWER_BANK == 1) || (CCG_SINK_ONLY)
            /*
             * In power bank case, if device is powered by external VDDD, VDDD gets
             * shorted to VBUS_IN line. This shall result connecting VDDD to the
             * Type-C VBUS line. This also includes cases where we start as dead
             * dead battery device and then get charged. So if any time VBUS has to
             * be removed in course of PD / Type-C state machine, ensure that internal
             * VBUS regulator is disabled. In event of dead battery, this shall lead
             * to device reset. This is the safest recovery path. CDT 276535.
             *
             * This code can be removed if the VBATT monitoring can be done
             * continuously. But this code can still be in place to avoid any
             * corner case handling.
             */

            /* Do this only on disconnect and type-C error recovery. */
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_TYPE_C_ERROR_RECOVERY))
            {
                pd_hal_disable_vreg(port);
            }
#endif /* (POWER_BANK == 1) || (CCG_SINK_ONLY) */
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S)) */

            if (
                    (evt == APP_EVT_HARD_RESET_SENT) ||
                    (evt == APP_EVT_HARD_RESET_RCVD) ||
                    (evt == APP_EVT_HR_SENT_RCVD_DEFERRED)
               )
            {
                gl_app_hard_rst_sent_rcvd[port] = true;

#if AMD_RETIMER_ENABLE
                if (ptrPdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
                {
                    /* Disable and then re-enable the retimer with a delay. */
                    amd_retimer_status_update (port, 0x00);
                    amd_retimer_rst(port);
                     /* Drop all previous MUX status updates and update the status as no data connection. */
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, 0u);
                }
#endif /* AMD_RETIMER_ENABLE */

#if MUX_DELAY_EN
                if (app_stat->is_mux_busy == false)
#endif /* MUX_DELAY_EN */
                {
#if BB_RETIMER_ENABLE
                    /* Disable and then re-enable the retimer with a delay. */
                    retimer_status_update (port, 0x00, 0);
                    retimer_disable(port, false);
                    retimer_enable(port, true);
#endif /* BB_RETIMER_ENABLE */

#if RIDGE_SLAVE_ENABLE
                    /* Drop all previous MUX status updates and update the status as 0 (no data connection). */
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_DEINIT, 0u);
#endif /* RIDGE_SLAVE_ENABLE */

#if ((BB_RETIMER_ENABLE) || (AMD_RETIMER_ENABLE))
                    app_stat->skip_mux_config = true;
#endif /* ((BB_RETIMER_ENABLE) || (AMD_RETIMER_ENABLE)) */
                }
#if MUX_DELAY_EN
                else
                {
                    /* Run MUX delay timer */
                    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContextICL_HARD_RST_TIMER,
                                timer_get_count(port, GET_APP_TIMER_ID(ptrPdStackContext,APP_MUX_DELAY_TIMER)) + 2,
                                app_hard_rst_retimer_cbk);
                    break;
                }
#endif /* MUX_DELAY_EN */
            }

            if (evt == APP_EVT_DISCONNECT)
            {
#if (!CY_PD_SINK_ONLY)
                if(
                    (false != gl_app_status[port].debug_acc_attached)
                    &&
                    (ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
                )
                {
                    ptrPdStackContext->ptrAppCbk->psrc_disable(ptrPdStackContext, NULL);
                }
                /* Mark debug accessory detached. */
                gl_app_status[port].debug_acc_attached = false;
#endif /* (!CY_PD_SINK_ONLY) */
#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE))
                /* Set USB2/USB3 connection bits to 1 by default. */
                gl_app_status[port].usb2_supp = false;
                gl_app_status[port].usb3_supp = false;
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE) */
            }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if (CCG_USB4_SUPPORT_ENABLE)
            gl_app_status[port].usb4_active = false;
            gl_app_status[port].usb4_data_rst_cnt = 0;
            /* Stop any application level data reset timer. */
            timer_stop(port, APP_DATA_RESET_TIMER);
#endif /* CCG_USB4_SUPPORT_ENABLE */

#if (BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE)
            if (!typec_only)
            {
                gl_app_status[port].skip_mux_config = true;
            }
#endif /* (BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE) */

            Cy_PdAltMode_VdmTask_MngrDeInit (ptrPdStackContext->ptrAltModeContext);
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)) || (defined(CCG6DF)) || (defined(CCG6SF)))
            timer_stop (port, APP_CBL_DISC_TRIGGER_TIMER);
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)) || (defined(CCG6DF)) || (defined(CCG6SF))) */
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE)
            gl_app_status[port].skip_mux_config = false;
#endif /* (BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE) */

#if PD_SRC_POWER_SHARE_ENABLE
            /* If this port had the excess power allocated, free it up. */
            app_power_share_update (port);
#endif /* PD_SRC_POWER_SHARE_ENABLE */

            if (hardreset_cplt)
            {
#if BB_RETIMER_ENABLE
                if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
                {
                    set_retimer_status(port, true);
                }
#endif /* BB_RETIMER_ENABLE */
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
#endif
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
#if !RIDGE_SLAVE_ENABLE
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, 0u);
#endif
#endif /* !RIDGE_SLAVE_ENABLE */
                    CALL_MAP(Cy_PdUtils_SwTimer_Stop) (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_AME_TIMEOUT_TIMER));

#if BB_RETIMER_ENABLE
#if (!CY_PD_SINK_ONLY)
                    if (
                            (!gl_app_hard_rst_sent_rcvd[port])
#if (!CCG_BACKUP_FIRMWARE)
                            &&
                            (dpm_stat->typecFsmState != TYPEC_FSM_TRY_WAIT_SNK)
#endif /* (!CCG_BACKUP_FIRMWARE) */
                       )
                    {
                        app_status[port].retimer_dis_req = true;
                    }
#endif /* (!CY_PD_SINK_ONLY) */
#endif /* BB_RETIMER_ENABLE */

#if RIDGE_SLAVE_ENABLE
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, NO_DATA);
#endif /* RIDGE_SLAVE_ENABLE */

#if AMD_SUPP_ENABLE
                    /* Drop all previous MUX status updates and update the status as 0 (no data connection). */
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, 0u);
#endif
                }
            }
#if (UVP_OVP_RECOVERY || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || ICL_OCP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(ptrPdStackContext))
                {
                    if ((app_stat->fault_status & ((uint8_t)APP_PORT_DISABLE_IN_PROGRESS)) == 0u)
                    {
                        (void)Cy_PdStack_Dpm_ClearFaultActive(ptrPdStackContext);
                    }
                }
            }
#endif /* UVP_OVP_RECOVERY || VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE || ICL_OCP_ENABLE */

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
#if (RIDGE_SLAVE_ENABLE && BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP)
                ridge_slave_set_debug(port, 0x00);
#endif /* (RIDGE_SLAVE_ENABLE && BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP) */

#if CCG_BACKUP_FIRMWARE
#if BB_RETIMER_ENABLE
                /* Turn the retimer off after it's been written to */
                retimer_status_update (port, 0x00, 0);
                retimer_disable (port, (gl_system_state != NB_SYS_PWR_STATE_S0));
#endif /* BB_RETIMER_ENABLE */
#endif /* CCG_BACKUP_FIRMWARE */

                /*
                 * Disable the regulator on port disconnect. This is not required for CCG7D as
                 * it is taken care of by the port disable. Doing it here will break the disable
                 * functionality.
                 */
#if ((!(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))) && REGULATOR_REQUIRE_STABLE_ON_TIME)
                pd_bb_disable(ptrPdStackContext->ptrUsbPdContext);
#endif /* (!(defined(CY_DEVICE_CCG7D) || (defined(CY_DEVICE_CCG7S))) && REGULATOR_REQUIRE_STABLE_ON_TIME) */

                /* Cleanup the PD block states on disconnect. */
                /* Need to review */
                Cy_USBPD_Vbus_HalCleanup(ptrPdStackContext->ptrUsbPdContext);

#if SYS_DEEPSLEEP_ENABLE
#if (!PSVP_FPGA_ENABLE)
                /* Disable the PD block clock peripherals if both the ports are disconnected */
                if(app_deepsleep_allowed())
                {
#if (defined(CY_DEVICE_CCG7D) && 0)
                    port_clock_disable();
#endif /* CY_DEVICE_CCG7D */
                }
#endif /* (!PSVP_FPGA_ENABLE) */
#endif


#if VBAT_GND_SCP_ENABLE
                /*
                 * VBAT-GND SCP level is set to 6A by default.
                 * During active attach, it shall be set to 10A to not conflict with
                 * VBUS SCP. And then reset to 6A once detach is detected.
                 */
                Cy_USBPD_Fault_VbatGndScpLevelSet(ptrPdStackContext->ptrUsbPdContext, PD_VBAT_GND_SCP_6A);
#endif /* VBAT_GND_SCP_ENABLE */

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
                /* Clear the VConn fault status. */
                app_stat->fault_status &= ~((uint8_t)APP_PORT_VCONN_FAULT_ACTIVE);
#endif /* VCONN_OCP_ENABLE || VCONN_SCP_ENABLE */

#if CCG_BB_ENABLE
                if (evt == APP_EVT_DISCONNECT)
                {
                    (void)Cy_PdAltMode_Billboard_Disable (ptrPdStackContext->ptrAltModeContext, true);

                    /* Clear self power status flag cleared on disconnect. */
                    Cy_PdAltMode_Billboard_UpdateSelfPwrStatus (ptrPdStackContext->ptrAltModeContext, 0);
                }
#endif /* CCG_BB_ENABLE */
            }

#if ((!CY_PD_SINK_ONLY) || (ROLE_PREFERENCE_ENABLE))
            if (
                    (evt == APP_EVT_HARD_RESET_COMPLETE) ||
                    (evt == APP_EVT_TYPE_C_ERROR_RECOVERY) ||
                    (evt == APP_EVT_DISCONNECT)
               )
            {
#if (!CY_PD_SINK_ONLY)
                gl_app_hard_rst_sent_rcvd[port] = false;
                /* Temporary fix for build issue. To be reviewed */
                gl_app_hard_rst_sent_rcvd[port] = gl_app_hard_rst_sent_rcvd[port];
#endif /* (!CY_PD_SINK_ONLY) */

#if (ROLE_PREFERENCE_ENABLE)
                /* Stop the DR-Swap and PR-Swap trigger timers.  Assume that
                 * PR_SWAP and DR_SWAP are pending. The actual status will be
                 * updated on contract completion.
                 */
                app_connect_change_handler (port);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            }
#endif /* ((!CY_PD_SINK_ONLY) || (ROLE_PREFERENCE_ENABLE)) */

#if (ADVDM_SUPPORT != 0)
        if (
            (evt == APP_EVT_HARD_RESET_COMPLETE) ||
            (evt == APP_EVT_TYPE_C_ERROR_RECOVERY) ||
            (evt == APP_EVT_DISCONNECT)
            )
        {
            /* ADVDM Persistence check and config updates */
            if(get_advdm_status(port).persistense == 0)
            {
                advdm_exit(port);
            }
        }
#endif

#if APP_PPS_SINK_SUPPORT
            /* Make sure the PPS re-negotiation task is stopped. */
            app_pps_sink_disable (port);
#endif /* APP_PPS_SINK_SUPPORT */

#if HPI_PPS_SINK_SUPPORT
        /* Make sure the PPS re-negotiation task is stopped. */
        app_pps_sink_disable (port);
#endif /* HPI_PPS_SINK_SUPPORT */

#if APP_FRS_RX_ENABLE
            gl_frs_in_prog[port] = false;
#endif /* APP_FRS_RX_ENABLE */
            break;

#if (!CY_PD_CBL_DISC_DISABLE)
        case APP_EVT_EMCA_NOT_DETECTED:
        case APP_EVT_EMCA_DETECTED:
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = true;
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.disc_cbl_pending = false;
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_prcs_failed = false;

#if DFP_ALT_MODE_SUPP
            /* Notify VDM discovery layer that it can proceed. */
            vdm_update_vcs_status (port);
#endif /* DFP_ALT_MODE_SUPP */

#if RIDGE_SLAVE_ENABLE
#if !(DFP_ALT_MODE_SUPP)
            /* Update the SoC after cable discovery if DR swap is accepted.*/
            if(dpm_stat->contract_exist)
            {
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
            }
#endif /* !(DFP_ALT_MODE_SUPP) */
#endif /* RIDGE_SLAVE_ENABLE */
            break;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const cy_en_pdstack_app_req_status_t*)dat ;
            if(*result == CY_PDSTACK_REQ_ACCEPT )
            {
#if (ROLE_PREFERENCE_ENABLE)
                gl_app_status[port].app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                if (gl_app_status[port].actv_swap_type == DPM_CMD_SEND_DR_SWAP)
                {
                    timer_stop (port, APP_INITIATE_SWAP_TIMER);
                    app_contract_handler (port);
                }
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE))
                /* Check if source and sink ports supports usb communication */
                app_check_usb_supp(port);
                /* Update Retimer/SoC data role bit */
                ridge_update_dr(port);
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) */

#if AMD_SUPP_ENABLE
                /* Check if source and sink ports supports usb communication */
                app_check_usb_supp(port);

                if (dpm_stat->cur_port_type == PRT_TYPE_UFP)
                {
                    /* Disable retimer handling */
                    app_get_status(port)->retimer_dis_req = true;
                }
                else
                {
                    /* Enable retimer handling */
                    amd_retimer_enable(port, false, true);
                }
#if CCG_USB4_SUPPORT_ENABLE
                if (gl_app_status[port].usb4_active == false)
#endif /* CCG_USB4_SUPPORT_ENABLE */
                {
                    /* Set MUX to USB after DR swap */
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
                }
#endif /* AMD_SUPP_ENABLE */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if CCG_USB4_SUPPORT_ENABLE
                if (gl_app_status[port].usb4_active != false)
                {
                    /* Run VDM manager */
                    gl_app_status[port].vdm_task_en = false;
                    enable_vdm_task_mngr(port);
                }
                else
#endif /* CCG_USB4_SUPPORT_ENABLE */
                {
                    /* Device data role changed. Reset alternate mode layer. */
                    Cy_PdAltMode_Mngr_LayerReset((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext));
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (CCG_BB_ENABLE != 0)
                /* Start tAME Timer to enable BB functionality */
                if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
                {
                    if (Cy_PdAltMode_Billboard_BindToPort(ptrPdStackContext->ptrAltModeContext) == CY_PDSTACK_STAT_SUCCESS)
                    {
                        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                                                    (cy_timer_id_t)APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
                    }
                }
                else
                {
                    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, (cy_timer_id_t)APP_AME_TIMEOUT_TIMER);
                }
#endif /* (CCG_BB_ENABLE != 0) */
            }
            break;

#if (!CCG_BACKUP_FIRMWARE)
        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_stat->vdm_retry_pending)
            {
                skip_soln_cb = true;
            }
            break;
#endif /* (!CCG_BACKUP_FIRMWARE) */
        case APP_EVT_BAD_SINK_APDO_SEL:
#if CCG_REV3_HANDLE_BAD_SINK
            gl_bad_sink_apdo_sel[port] = true;
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            contract_status = (cy_stc_pdstack_pd_contract_info_t*)dat;

#if (AUVDM_SUPPORT && CCG_HPI_PD_ENABLE)
            pd_packet_t num_src_pdo = *dpm_stat->src_cap_p;
            hpi_set_src_pdo_count(0, num_src_pdo.len);
#endif /* (AUVDM_SUPPORT && CCG_HPI_PD_ENABLE) */

            /* Set VDM version based on active PD revision. */
#if CY_PD_REV3_ENABLE
            if (ptrPdStackContext->dpmConfig.specRevSopLive >= CY_PD_REV3)
            {
                ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_version = CY_PD_STD_VDM_VERSION_REV3;
                /* Enable DATA_RESET message handling in PD 3.0 contract. */
                ptrPdStackContext->dpmStat.usb4En = (uint8_t)true;
            }
            else
#endif /* CY_PD_REV3_ENABLE */
            {
                ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_version = CY_PD_STD_VDM_VERSION_REV2;
                /* Clear USB4 enable flag so that data reset will be REJECTed by the stack. */
                ptrPdStackContext->dpmStat.usb4En = (uint8_t)false;
            }

            if ((contract_status->status == CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED) ||
                    (contract_status->status == CY_PDSTACK_CONTRACT_REJECT_CONTRACT_NOT_VALID))
            {
#if CCG_REV3_HANDLE_BAD_SINK
                if ((contract_status->status == CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                        (contract_status->status == CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED))
                {
                    /* The device is in PD contract. On next detach we need not do special handling */
                    gl_bad_sink_recovery_handled[port] = false;
                }
                /*
                 * Stop bad sink timer, but retain bad sink status until port
                 * detach or disable.
                 */
                CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext,  GET_APP_TIMER_ID(ptrPdStackContext,APP_BAD_SINK_TIMEOUT_TIMER));
                if (true == gl_bad_sink_apdo_sel[port])
                {
                    gl_bad_sink_apdo_sel[port] = false;
                }   
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if PD_SRC_POWER_SHARE_ENABLE
                app_power_share_update (port);
#endif /* PD_SRC_POWER_SHARE_ENABLE */

#if (ROLE_PREFERENCE_ENABLE)
                app_contract_handler (port);
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /*
                 * Contract established.  Enable VDM task manager for Alt. Mode support.
                 * This function will have no effect if the Alt. Modes are already running.
                 */
                if (
                        (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) ||
                        (((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.vdm_prcs_failed == false)
                   )
                {
                    Cy_PdAltMode_VdmTask_Enable(ptrPdStackContext->ptrAltModeContext);
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if ((RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE))
                /* Set USB2/USB3 connection bits to 1 by default*/
                gl_app_status[port].usb2_supp = true;
                gl_app_status[port].usb3_supp = true;

                /* Check if source and sink ports supports usb communication */
                app_check_usb_supp(port);

                /* Update SS mux if no alt mode and usb4 mode and USB 2.0 support */
                if (
#if (!CCG_BACKUP_FIRMWARE)
                      (gl_app_status[port].alt_mode_entered == false) &&
                      (gl_app_status[port].usb4_active == false)      &&
#endif /* (!CCG_BACKUP_FIRMWARE) */
                      (gl_app_status[port].usb2_supp == false)
                   )
                {
                    /* Update MUX in case of usb capabilities changes */
                    Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
                }
#endif /* (RIDGE_SLAVE_ENABLE) || (BB_RETIMER_ENABLE) || (AMD_SUPP_ENABLE) */
            }

#if (CCG_BB_ENABLE != 0)
            /* Now that there is a PD contract in place, we can set the self powered status flag. */
            Cy_PdAltMode_Billboard_UpdateSelfPwrStatus (ptrPdStackContext->ptrAltModeContext, 1);

            if (
                    (contract_status->status != CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) &&
                    /* QAC suppression 3415: The check on the right hand side of logical operator
                     * is read only. */
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) && /* PRQA S 3415 */
                    (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
               )
            {
                (void)Cy_PdAltMode_Billboard_Enable(ptrPdStackContext->ptrAltModeContext, BB_CAUSE_PWR_FAILURE);
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_LOAD_SHARING_ENABLE
            if (CY_PDSTACK_STAT_SUCCESS == Cy_PdStack_Dpm_IsPrevContractValid(ptrPdStackContext) &&
                    ((CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL == contract_status->status) ||
                     (CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED == contract_status->status)))
            {
                src_sel_pdo = (ptrPdStackContext->peStat.srcSelPdo);
                if ((uint8_t)CY_PDSTACK_PDO_FIXED_SUPPLY == src_sel_pdo.src_gen.supplyType)
                {
                    max_volt = src_sel_pdo.src_gen.minVoltage;
                }
                else
                {
                    max_volt = ((uint32_t)src_sel_pdo.pps_src.maxVolt * 2u);
                }
                if (gl_src_sel_pdo_max_volt[port] != max_volt)
                {
                    if(gl_src_sel_pdo_max_volt[port] != 0u)
                    {
                        async_change = true;
                    }
                    gl_src_sel_pdo_max_volt[port] = max_volt;
                }
                else
                {
                    /* No statement */
                }
            }
            else
            {
                /* No statement */
            }
#endif /* CCG_LOAD_SHARING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
            if((true == ccg_power_throttle_get_power_throttle_cmd_pending(ptrPdStackContext)) &&
                    (CY_PDSTACK_CONTRACT_REJECT_NO_CONTRACT != contract_status->status))
            {
                if (CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED == contract_status->status)
                {
                    ccg_power_throttle_set_power_throttle_renegotiation_complete(ptrPdStackContext, true, true);
                }
                else
                {
                    ccg_power_throttle_set_power_throttle_renegotiation_complete(ptrPdStackContext, true, false);
                }
            }
#if CCG_LOAD_SHARING_ENABLE
            else if((false == ccg_power_throttle_get_power_throttle_cmd_pending(ptrPdStackContext)) &&
                    ((CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL == contract_status->status) ||
                     (CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED == contract_status->status)))
            {
                if (ccg_ls_is_enabled(ptrPdStackContext) && (true == async_change))
                {
                    /* Inform async power change. 2nd and 3rd parameter is not looked into in this case */
                    (void)ccg_ls_power_change_complete(ptrPdStackContext, 0u, false, true);
                }
                else
                {
                    /* No statement */
                }
            }
#endif /* CCG_LOAD_SHARING_ENABLE */
            else
            {
                /* No statement */
            }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */


#if RIDGE_SLAVE_ENABLE
            /* Set USB2/USB3 connection bits to 1 by default*/
            gl_app_status[port].usb2_supp = true;
            gl_app_status[port].usb3_supp = true;
            /* Check if source and sink ports supports usb communication */
            app_check_usb_supp(port);

            /* Update SS mux if no alt mode and usb4 mode and USB 2.0 support */
            if (
                  (gl_app_status[port].alt_mode_entered == false) &&
                  (gl_app_status[port].usb4_active == false)      &&
                  (gl_app_status[port].usb2_supp == false)
               )
            {
                /* Update MUX in case of usb capabilities changes */
                Cy_PdAltMode_HW_SetMux(ptrPdstackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
            }
#endif /* RIDGE_SLAVE_ENABLE */
            break;

#if CY_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
#if (CCG_HPI_PD_ENABLE)
            /* Handle the extended message locally if forwarding to EC is not enabled. */
            if (hpi_is_extd_msg_ec_ctrl_enabled (ptrPdStackContext) == false)
#endif /* (CCG_HPI_PD_ENABLE) */
            {
                skip_soln_cb = app_extd_msg_handler(ptrPdStackContext, (cy_stc_pd_packet_extd_t *)dat);
            }
            break;
#if HOST_ALERT_MSG_ENABLE
        case APP_EVT_ALERT_RECEIVED:
            /* Respond to ALERT message only if there is the number of object is one. */
            if (((pd_packet_t*)dat)->len == 1)
            {
                alert_ado = ((pd_packet_t*)dat)->dat[0];
                if(alert_ado.ado_alert.bat_status_change == false)
                {
                    dpm_pd_command(port, DPM_CMD_GET_STATUS, NULL, NULL);
                }
                else
                {
                    uint8_t i = alert_ado.ado_alert.fixed_bats |
                        (alert_ado.ado_alert.hot_swap_bats << 4);
                    dpm_pd_cmd_buf_t cmd;

                    /* Identify the first battery for which the change is intended. */
                    gl_get_bat_status[port] = 0;
                    while ((i != 0) && ((i & 0x01) == 0))
                    {
                        gl_get_bat_status[port]++;
                        i >>= 1;
                    }

                    cmd.cmd_sop = SOP;
                    cmd.extd_hdr.val = 0x1;
                    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
                    cmd.extd_type = EXTD_MSG_GET_BAT_STATUS;
                    cmd.dat_ptr = (uint8_t*)&gl_get_bat_status[port];
                    dpm_pd_command(port, DPM_CMD_SEND_EXTENDED, &cmd, NULL);
                }
            }
            break;
#endif /* HOST_ALERT_MSG_ENABLE */
#endif /* CY_PD_REV3_ENABLE */

        case APP_EVT_TYPEC_ATTACH_WAIT:
#if REGULATOR_REQUIRE_STABLE_ON_TIME
            if(false == ptrPdStackContext->dpmStat.faultActive)
            {
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if !PSVP_FPGA_ENABLE
                if(vbus_get_value(ptrPdStackContext) < CY_PD_VSAFE_0V)
#endif /* !PSVP_FPGA_ENABLE */
                {
                    Cy_USBPD_BB_Enable(ptrPdStackContext->ptrUsbPdContext);
                }
#if !PSVP_FPGA_ENABLE
                else
                {
                    /* Do not enable regulator on entering Attach.Wait state if VBUS is not in VSAFE_0V range */
                }
#endif /* !PSVP_FPGA_ENABLE */
#else
                REGULATOR_ENABLE(port);
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
            }
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */
            /* Update: Need to port this function for CCG7D */
#if (defined(CY_DEVICE_CCG7D) && 0)
            port_clock_enable();
#endif /* defined(CY_DEVICE_CCG7D) */
#if VBAT_GND_SCP_ENABLE
            /*
             * VBAT-GND SCP level is set to 6A by default.
             * During active attach, it shall be set to 10A to not conflict with
             * VBUS SCP. And then reset to 6A once detach is detected.
             */
            Cy_USBPD_Fault_VbatGndScpLevelSet(ptrPdStackContext->ptrUsbPdContext, PD_VBAT_GND_SCP_10A);
#endif /* VBAT_GND_SCP_ENABLE */

#if BB_RETIMER_ENABLE
            retimer_enable(port, false);
#endif /* BB_RETIMER_ENABLE */
#if AMD_RETIMER_ENABLE
            /* Enable the retimer power and release it from reset. */
            amd_retimer_enable(port, false, false);
#endif /* AMD_RETIMER_ENABLE */
#if CCG_PASC_LP_ENABLE
            pd_pasc_lp_disable(port);
#endif /* CCG_PASC_LP_ENABLE */
            break;

        case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
#if (!CY_PD_SINK_ONLY)
            /*
             * Check and disable VBUS discharge if we had enabled discharge due to invalid VBUS voltage.
             * Since the check is already there inside the function, need not repeat it here.
             */
            app_psrc_invalid_vbus_dischg_disable(ptrPdStackContext);
#endif /* (!CY_PD_SINK_ONLY) */

#if REGULATOR_REQUIRE_STABLE_ON_TIME
            /* Review whether REGULATOR_DISABLE macro needs to be retained */
            Cy_USBPD_BB_Disable(ptrPdStackContext->ptrUsbPdContext);
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

#if (BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE)
            /* If we are in Unattached states, turn the retimer off after it's been written to */
            if (
                    (dpm_stat->typec_fsm_state == TYPEC_FSM_UNATTACHED_SNK)
#if (!CCG_SINK_ONLY)
                    ||
                    (dpm_stat->typec_fsm_state == TYPEC_FSM_UNATTACHED_SRC)
#endif /* (!CCG_SINK_ONLY) */
               )
            {
                gl_app_status[port].retimer_dis_req = true;
#if BB_RETIMER_ENABLE
                Cy_PdAltMode_HW_SetMux(ptrPdstackContext->ptrAltModeContext, MUX_CONFIG_RIDGE_CUSTOM, NO_DATA);
#elif AMD_RETIMER_ENABLE
                if(get_mux_state(port) != MUX_CONFIG_ISOLATE)
                {
                    Cy_PdAltMode_HW_SetMux(ptrPdstackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, NO_DATA);
                }
#endif /* BB_RETIMER_ENABLE */
            }
#endif /* BB_RETIMER_ENABLE || AMD_RETIMER_ENABLE */
            break;

#if ((POWER_ROLE_PREFERENCE_ENABLE) || (PD_SRC_POWER_SHARE_ENABLE))
        case APP_EVT_PR_SWAP_COMPLETE:
#if (POWER_ROLE_PREFERENCE_ENABLE)
            gl_app_status[port].app_pending_swaps &= ~APP_PR_SWAP_PENDING;
            if (gl_app_status[port].actv_swap_type == DPM_CMD_SEND_PR_SWAP)
            {
                timer_stop (port, APP_INITIATE_SWAP_TIMER);
                app_contract_handler (port);
            }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
#if PD_SRC_POWER_SHARE_ENABLE
            app_power_share_update (port);
#endif /* PD_SRC_POWER_SHARE_ENABLE */
            break;
#endif /* ((POWER_ROLE_PREFERENCE_ENABLE) || (PD_SRC_POWER_SHARE_ENABLE)) */

        case APP_EVT_DATA_RESET_ACCEPTED:
        /* Need to reset alternate modes as part of Data_Reset. No status update is to be provided. */
            Cy_PdAltMode_Mngr_LayerReset ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext));

#if (!CY_PD_CBL_DISC_DISABLE)
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_disc_id_finished = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

#if (CCG_USB4_SUPPORT_ENABLE)
            /* Mark USB4 not active as Data Reset has been accepted. */
            app_stat->usb4_active = false;
#endif /* CCG_USB4_SUPPORT_ENABLE */            
            /* Update : set_mux calls */
            /* Switch to USB only configuration once data reset has been accepted. */
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
            }
            else
            {
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_ISOLATE, 0u);
            }
#endif
            break;

        case APP_EVT_DATA_RESET_CPLT:
            /* DFP needs to re-enable USBx connections at completion of Data_Reset. */
            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP)
            {
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
                Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
#endif
            }
            break;

#if (CCG_USB4_SUPPORT_ENABLE)
        case APP_EVT_USB_ENTRY_CPLT:
            app_stat->usb4_data_rst_cnt = 0;
            app_stat->usb4_active = true;
            break;
#endif /* CCG_USB4_SUPPORT_ENABLE */

        case APP_EVT_PD_SINK_DEVICE_CONNECTED:
#if CCG_REV3_HANDLE_BAD_SINK
            gl_bad_sink_pd_status[port] = true;
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            break;

        case APP_EVT_PKT_RCVD:
#if CHUNKING_NOT_SUPPORTED
            /* Make sure we stop the timer that initiates NOT_SUPPORTED response sending. */
            CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_CHUNKED_MSG_RESP_TIMER));
#endif /* CHUNKING_NOT_SUPPORTED */

#if ((CCG_USB4_SUPPORT_ENABLE) || ((ICL_ENABLE) && (PROCHOT_SUPP)))
            {
                pd_packet_extd_t *pkt = (pd_packet_extd_t *)dat;
                if ((pkt->hdr.hdr.extd == 0) && (pkt->len == 0) && (pkt->msg == CTRL_MSG_ACCEPT))
                {
#if (CCG_USB4_SUPPORT_ENABLE)
                    if (dpm_stat->pe_fsm_state == PE_FSM_SEND_DATA_RESET)
                    {
                        /* Process steps to be completed after Data Reset has been accepted. */
                        app_data_reset_accepted (port);
                    }
#endif /* (CCG_USB4_SUPPORT_ENABLE) */
                }
            }
#endif /* ((CCG_USB4_SUPPORT_ENABLE) || ((ICL_ENABLE) && (PROCHOT_SUPP))) */
            break;

        case APP_EVT_CBL_RESET_SENT:
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.cbl_rst_done = true;
            break;

#if (!CY_PD_SINK_ONLY)
        case APP_EVT_HR_PSRC_ENABLE:
#if BB_RETIMER_ENABLE
            set_retimer_status(port, true);
#endif /* BB_RETIMER_ENABLE */
#if (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP)
            Cy_PdAltMode_HW_SetMux(ptrPdStackContext->ptrAltModeContext, MUX_CONFIG_SS_ONLY, 0u);
#endif /* (UFP_ALT_MODE_SUPP || DFP_ALT_MODE_SUPP) */
            break;
#endif /* (!CY_PD_SINK_ONLY) */

#if ((ICL_ENABLE) && (PROCHOT_SUPP))
        case APP_EVT_TYPEC_RP_DETACH:
            /* Fall-through */
        case APP_EVT_PR_SWAP_ACCEPTED:
#if ((ICL_ENABLE) && (PROCHOT_SUPP))    
            ProcHot_Assert ();
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */            
            break;
#endif /* ((ICL_ENABLE) && (PROCHOT_SUPP)) */

#if (AUVDM_SUPPORT != 0)
        case APP_EVT_CUST_ALT_MODE_CHANGED:
            skip_soln_cb = false;
            break;
#endif /* (AUVDM_SUPPORT != 0) */

#if CY_PD_BIST_STM_ENABLE
        case APP_EVT_BIST_STM_ENTRY:
            /* QAC suppression 0311: The flag is used only for read only purposes */
            intended_for_master = *((bool *)dat); /* PRQA S 0311 */
            app_bist_stm_handler(ptrPdStackContext, CY_PDSTACK_BIST_STM_ENTRY, intended_for_master);
            break;
        case APP_EVT_BIST_STM_EXIT:
            /* QAC suppression 0311: The flag is used only for read only purposes */
            intended_for_master = *((bool *)dat); /* PRQA S 0311 */
            app_bist_stm_handler(ptrPdStackContext, CY_PDSTACK_BIST_STM_EXIT, intended_for_master);
            break;
#endif /* CY_PD_BIST_STM_ENABLE */

        case APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS:
#if (!CY_PD_SINK_ONLY)
            /*
             * There could be stale voltage on VBUS. Try to discharge this voltage. In case
             * this is a quick reconnect, the previous discharge may already be running.
             * In this case, do not try to discharge. If not, then start a discharge cycle.
             * This should only be done for finite duration to avoid any damage in case this
             * was externally driven voltage. Re-use the psource timer implementation.
             *
             * NOTE: This may need to be re-implemented if the APP_PSOURCE_DIS_TIMER timer
             * is being implemented differently.
             */
            if (CALL_MAP(Cy_PdUtils_SwTimer_IsRunning )(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_PSOURCE_DIS_TIMER)) == false)
            {
                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,GET_APP_TIMER_ID(ptrPdStackContext, APP_PSOURCE_DIS_TIMER),
                        APP_PSOURCE_DIS_TIMER_PERIOD, app_psrc_invalid_vbus_tmr_cbk);
                gl_app_invalid_vbus_dis_on[port] = true;
                solution_fn_handler->app_get_callback_ptr(ptrPdStackContext)->vbus_discharge_on(ptrPdStackContext);
            }
#endif /* (!CY_PD_SINK_ONLY) */
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            Cy_USBPD_BB_Disable(ptrPdStackContext->ptrUsbPdContext);
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
            break;

        default:
            /* Nothing to do. */
            break;
    }

    /* Pass the event notification to the fault handler module. */
    if (fault_event_handler (ptrPdStackContext, evt, dat))
    {
        skip_soln_cb = true;
    }

#if BATTERY_CHARGING_ENABLE
    bc_pd_event_handler (ptrPdStackContext, evt);
#endif /* BATTERY_CHARGING_ENABLE */

#if BCR
    bcr_pwr_event_handler(port, evt, dat);
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
    pb_event_handler (port, evt);
#endif /* POWER_BANK */

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
       solution_fn_handler->sln_pd_event_handler(ptrPdStackContext, evt, dat) ;
    }
}

/* Update : Re-structure app variables */
app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &gl_app_status[port].appResp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &gl_app_status[port];
}

bool app_is_port_enabled(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF)|| defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    bool ret = true;
    uint8_t port = ptrPdStackContext->port;

    if (port > NO_OF_TYPEC_PORTS)
    {
        ret = false;
    }
#if (!CCG_BOOT)
#if ((CCG_HPI_ENABLE) && (!CCG_LOAD_SHARING_ENABLE))
    /* Check if the port has been disabled through HPI. */
    if ((hpi_get_port_enable() & (1u << port)) == 0u)
    {
        ret = false;
    }
#endif /* CCG_HPI_ENABLE */
#endif /* (!CCG_BOOT) */

    if (ptrPdStackContext->dpmConfig.dpmEnabled == false)
    {
        ret = false;
    }

    return ret;
#else
    (void)ptrPdStackContext;
    return true;
#endif 
}


/* Callback that will be called when there is any change to the V5V or VSYS supplies. */
void app_ccgx_supply_change_cb(uint8_t port, ccg_supply_t supply_id, bool present)
{  
#if ((defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF)|| defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && CCGX_V5V_CHANGE_DETECT)
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /*
     * Currently we only handle V5V changes:
     * If V5V is removed, we exit active alternate modes if there is a cable which requires VConn.
     * If V5V is re-applied after being removed, we restart the alternate mode state machine.
     */
    if (supply_id == CCG_SUPPLY_V5V)
    {
        if (!present)
        {
            gl_app_status[port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;

#if (!CCG_VCONN_DISABLE)
            if (vconn_is_present (port))
            {
                vconn_change_handler (port, false);
            }
#endif /* (!CCG_VCONN_DISABLE) */
        }
        else
        {
            if ((app_get_status(port)->fault_status & APP_PORT_V5V_SUPPLY_LOST) != 0)
            {
                gl_app_status[port].fault_status &= ~APP_PORT_V5V_SUPPLY_LOST;
                vconn_change_handler (port, true);
            }
        }
    }
#else
    (void)port;
    (void)supply_id;
    (void)present;
#endif /* ((defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF)|| defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) && CCGX_V5V_CHANGE_DETECT)*/
}

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF)|| defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if (!CCG_BACKUP_FIRMWARE)
/* Callback used to receive fault notification from the HAL and to pass it on to the event handler. */
static bool app_cc_sbu_fault_handler(void *context, bool fault_type)
{
    cy_stc_usbpd_context_t *usbpdContext = (cy_stc_usbpd_context_t *)context;
    cy_stc_pdstack_context_t *pdStackContext = (cy_stc_pdstack_context_t *)usbpdContext->pdStackContext;
    app_event_handler(pdStackContext, (fault_type ? APP_EVT_SBU_OVP : APP_EVT_CC_OVP), NULL);
    return true;
}
#endif /* (!CCG_BACKUP_FIRMWARE) */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

void app_vbus_slow_discharge_cbk(void *pdStackContext)
{
#if VBUS_SLOW_DISCHARGE_EN
    cy_stc_pdstack_context_t *context =(cy_stc_pdstack_context_t *)pdStackContext;
    Cy_USBPD_Vbus_Slow_DischargeCbk(GET_APP_TIMER_ID(context,VBUS_DISCHARGE_SCHEDULE_TIMER), context->ptrUsbPdContext);
#else
    CY_UNUSED_PARAMETER(pdStackContext);
#endif /* VBUS_SLOW_DISCHARGE_EN */
}

bool app_timer_start(cy_stc_usbpd_context_t *context, void *callbackContext, 
        uint16_t id, uint16_t period, cy_timer_cbk_t cbk)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    return CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrTimerContext, callbackContext, (cy_timer_id_t)GET_PDL_TIMER_ID(context, id), period, (cy_cb_timer_t)cbk);
}

void app_timer_stop(cy_stc_usbpd_context_t *context, cy_en_usbpd_timer_id_t id)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrTimerContext, (cy_timer_id_t)GET_PDL_TIMER_ID(context, id));
}

bool app_timer_is_running(cy_stc_usbpd_context_t *context, cy_en_usbpd_timer_id_t id)
{
    cy_stc_pdutils_sw_timer_t *ptrTimerContext = ((cy_stc_pdstack_context_t *)context->pdStackContext)->ptrTimerContext;
    return CALL_MAP(Cy_PdUtils_SwTimer_IsRunning )(ptrTimerContext, (cy_timer_id_t)GET_PDL_TIMER_ID(context, id));
}

void timer_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    ptrPdStackContext->ptrUsbPdContext->timerStopcbk=app_timer_stop;
    ptrPdStackContext->ptrUsbPdContext->timerStartcbk=app_timer_start;
    ptrPdStackContext->ptrUsbPdContext->timerIsRunningcbk=app_timer_is_running;

#if VBUS_SLOW_DISCHARGE_EN
    ptrPdStackContext->ptrUsbPdContext->vbusSlowDischargecbk=app_vbus_slow_discharge_cbk;
#endif /*VBUS_SLOW_DISCHARGE_EN */
}

void app_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if CCG_LOAD_SHARING_ENABLE
    gl_src_sel_pdo_max_volt[ptrPdStackContext->port] = 0u;
#endif /* CCG_LOAD_SHARING_ENABLE*/

    /* Initialize the VDM responses from the configuration table. */
    vdm_data_init(ptrPdStackContext);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Set alt mode trigger based on config. */
    ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.alt_mode_trig_mask = pd_get_ptr_dp_tbl(ptrPdStackContext->ptrUsbPdContext)->dp_mode_trigger;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

#if CCG_USB4_SUPPORT_ENABLE
#if RIDGE_SLAVE_ENABLE
    /* Save host capabilities configuration */
    app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control = PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->host_support;
    if (PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->vpro_capable)
    {
        app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control |= APP_HPI_VPRO_SUPP_MASK;
    }
    if (
           (PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->usb4_support == USB_ROLE_HOST) ||
           (PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->usb4_support == USB_ROLE_DRD)
        )
    {
        app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control |= USB4_EN_HOST_PARAM_MASK;
    }
#elif AMD_SUPP_ENABLE
    /* Save host capabilities configuration */
    app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control = PD_GET_PTR_AMD_CFG_TBL(ptrPdStackContext->port)->host_support;
    if (PD_GET_PTR_AMD_CFG_TBL(ptrPdStackContext->port)->usb4_captive)
    {
        app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control |= USB4_EN_HOST_PARAM_MASK;
    }
#endif /* RIDGE_SLAVE_ENABLE */
#if (!CCG_HPI_PD_CMD_DISABLE)
    /* Set default host capabilities status */
    hpi_set_host_cap_ctrl_reg (ptrPdStackContext->port, app_get_status(ptrPdStackContext->port)->custom_hpi_host_cap_control);
#endif /* !CCG_HPI_PD_CMD_DISABLE */
#endif /* CCG_USB4_SUPPORT_ENABLE */


#if ((defined(CY_DEVICE_CCG6)) && OTP_ENABLE)
    /* Init OT temperature tracking */
    app_otp_enable (ptrPdStackContext->port);
#endif /* ((defined(CY_DEVICE_CCG6)) && OTP_ENABLE) */

#if (CCG_BB_ENABLE != 0)
    /*
     * Initialize the billboard interface. The billboard
     * interface shall not get initialized if it is not
     * enabled in configuration table.
     */
    Cy_PdAltMode_Billboard_Init(ptrPdStackContext->ptrAltModeContext);
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
    (void)bc_init(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

#if (ROLE_PREFERENCE_ENABLE)
    if (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->tbthost_cfg_tbl_offset != 0)
    {
#if (POWER_ROLE_PREFERENCE_ENABLE)
        app_pref_power_role[ptrPdStackContext->port] = (PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->pref_pwr_role);
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        app_pref_data_role[ptrPdStackContext->port] = (PD_GET_PTR_TBTHOST_CFG_TBL(ptrPdStackContext->port)->pref_data_role);
    }
#if AMD_SUPP_ENABLE
    else if (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->amd_cfg_tbl_offset != 0)
    {
#if (POWER_ROLE_PREFERENCE_ENABLE)
        app_pref_power_role[ptrPdStackContext->port] = (PD_GET_PTR_AMD_CFG_TBL(ptrPdStackContext->port)->pref_pwr_role);
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        app_pref_data_role[ptrPdStackContext->port] = (PD_GET_PTR_AMD_CFG_TBL(ptrPdStackContext->port)->pref_data_role);
    }
#endif /* AMD_SUPP_ENABLE */
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Update alternate mode mask settings from config table. */
    {
        if (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->port_n_base_alt_mode_table_offset != 0u)
        {
            cy_stc_pdaltmode_cfg_settings_t *altcfg = pd_get_ptr_base_alt_tbl(ptrPdStackContext->ptrUsbPdContext);

            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.dfp_alt_mode_mask = altcfg->dfp_alt_mode_mask;
            ((cy_stc_pdaltmode_context_t *)(ptrPdStackContext->ptrAltModeContext))->appStatus.ufp_alt_mode_mask = altcfg->ufp_alt_mode_mask;
        }
    }
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

    /* Update : Handle config table calls */
    /* Update custom host config settings to the stack. */
    ptrPdStackContext->dpmStat.typecAccessorySuppDisabled = !(ptrPdStackContext->ptrPortCfg->accessoryEn);
    ptrPdStackContext->dpmStat.typecRpDetachDisabled = !(ptrPdStackContext->ptrPortCfg->rpDetachEn);

#if PD_SRC_POWER_SHARE_ENABLE
    /* Excess power is not allocated to any ports. */
    app_src_extra_power_port = NO_OF_TYPEC_PORTS;
#endif /* PD_SRC_POWER_SHARE_ENABLE */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
    /* For systems with TYPEA ptrPdStackContext->port. */
    if (get_pd_port_config(0)->type_a_enable)
    {
        type_a_port_enable();
    }
    else
    {
        /* Ensure that the state machine variables are initialized correctly. */
        type_a_port_disable();
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if (!CCG_BACKUP_FIRMWARE)
#if (CCG_CC_SBU_OVP_RETRY_LIMIT != 0)
    /* Set the retry limit where it is non-zero. */
    gl_app_fault_retry_limit[ptrPdStackContext->port][FAULT_TYPE_CC_OVP]  = CCG_CC_SBU_OVP_RETRY_LIMIT;
    gl_app_fault_retry_limit[ptrPdStackContext->port][FAULT_TYPE_SBU_OVP] = CCG_CC_SBU_OVP_RETRY_LIMIT;
#endif /* (CCG_CC_SBU_OVP_RETRY_LIMIT != 0) */

    /* Register a callback for notification of CC/SBU faults. */
    ptrPdStackContext->ptrUsbPdContext->ccSbuOvpCbk = app_cc_sbu_fault_handler;
#endif /* (!CCG_BACKUP_FIRMWARE) */

#if ((!ICL_ENABLE) && (CCGX_V5V_CHANGE_DETECT))
    /* Register a handler that will be notified when there is any change in V5V or VSYS state. */
    pd_hal_set_supply_change_evt_cb(app_ccgx_supply_change_cb);
#endif /* ((!ICL_ENABLE) && (CCGX_V5V_CHANGE_DETECT)) */

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(CCG6DF) || defined(CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)*/

#if (ADVDM_SUPPORT != 0)
    advdm_init(ptrPdStackContext->port);
#endif

#if APP_PPS_SINK_SUPPORT
    CALL_MAP(hpi_set_userdef_write_handler)(hpi_user_reg_handler);
#endif /* APP_PPS_SINK_SUPPORT */
}

#if ((CCG_APP_TASK_LPM_SLOW_POLL_ENABLE ||SYS_DEEPSLEEP_ENABLE) && (!CCG_BOOT))

extern bool pd_hal_is_cc_start_pending (void);

/* Implements CCG deep sleep functionality for power saving. */
bool system_sleep()
{
    uint8_t intr_state;
    bool dpm_slept[NO_OF_TYPEC_PORTS] = { 
    false
#if CCG_PD_DUALPORT_ENABLE
    ,false
#endif /* CCG_PD_DUALPORT_ENABLE */ 
      };

    bool dpm_resume = false;
    bool app_slept = false;
    bool retval = false;
#if BATTERY_CHARGING_ENABLE
    bool bc_slept = false;
#endif /* BATTERY_CHARGING_ENABLE */

#if (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    uint8_t i;
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if (((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID)) || (RIDGE_SLAVE_ENABLE))
    bool scb_idle = true;
#if ((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID))
    bool hpi_scb_idle;
#endif /* ((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID)) */
#if (RIDGE_SLAVE_ENABLE)
    bool ridge_slave_scb_idle;
#endif /* (RIDGE_SLAVE_ENABLE) */
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    bool sensor_is_idle;
#endif /*(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
    bool ls_is_idle;
#endif /* CCG_LOAD_SHARING_ENABLE */

#if CY_CABLE_COMP_ENABLE
    bool cable_comp_is_idle;
#endif /* CY_CABLE_COMP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
    bool power_throttle_sleep_allowed;
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */


    for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
    /* Do one DPM sleep capability check before locking interrupts out. */
    Cy_PdStack_Dpm_IsIdle(solution_fn_handler->Get_PdStack_Context(i), &dpm_slept[i]);
    if (!dpm_slept[i])
    {
        return retval;
    }
    }

#if CCG_LIN_ENABLE
    bool lins_slept = false;
#endif /*CCG_LIN_ENABLE*/

    intr_state = Cy_SysLib_EnterCriticalSection();

    /*
     * We have to check the application layer, HPI and the Device Policy
     * Manager (DPM) to see if all of these modules are ready for sleep.
     * CCG can only enter deep sleep if all of these blocks are in an idle
     * state.
     *
     * Note: The respective sleep functions might be performing some
     * state updates as part of the idle check function; and therefore
     * the corresponding wakeup function needs to be called if they have
     * returned true to indicate that sleep is allowed.
     */
    if (app_sleep())
    {
        app_slept = true;

#if BATTERY_CHARGING_ENABLE
        if (bc_sleep(solution_fn_handler->Get_PdStack_Context(0)))
        {
            bc_slept = true;
#endif /* BATTERY_CHARGING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
            sensor_is_idle = ccg_sensor_is_idle(solution_fn_handler->Get_PdStack_Context(0));
#endif /*(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
            ls_is_idle = ccg_ls_is_idle(solution_fn_handler->Get_PdStack_Context(0));
#endif /* CCG_LOAD_SHARING_ENABLE */

#if CY_CABLE_COMP_ENABLE
            cable_comp_is_idle = ccg_cable_comp_is_idle(solution_fn_handler->Get_PdStack_Context(0));
#endif /* CY_CABLE_COMP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
            power_throttle_sleep_allowed = ccg_power_throttle_sleep_allowed(solution_fn_handler->Get_PdStack_Context(0));
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

            if (
#if CCG_HPI_ENABLE
                    (CALL_MAP(hpi_sleep_allowed)()) &&
#endif /* CCG_HPI_ENABLE */

#if CCG_UCSI_ENABLE
                    (ucsi_sleep_allowed()) &&
#endif /* CCG_UCSI_ENABLE */

#if ICL_ENABLE
                    (icl_sleep_allowed()) &&
#endif /* ICL_ENABLE */

#if BB_RETIMER_ENABLE
                    (retimer_sleep_allowed()) &&
#endif /* BB_RETIMER_ENABLE */

#if AMD_SUPP_ENABLE
                    (amd_sleep_allowed()) &&
#endif /* BB_RETIMER_ENABLE */

#if CCG_SYNC_ENABLE
                    (ccg_sync_sleep_allowed()) &&
#endif /* CCG_SYNC_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
                    sensor_is_idle &&
#endif /*(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
                    ls_is_idle &&
#endif /* CCG_LOAD_SHARING_ENABLE */
#if CY_CABLE_COMP_ENABLE
                    cable_comp_is_idle &&
#endif /* CY_CABLE_COMP_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
                    power_throttle_sleep_allowed &&
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
                       true)
            {
                for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
                {

                    (void)Cy_PdStack_Dpm_PrepareDeepSleep(solution_fn_handler->Get_PdStack_Context(i), &dpm_slept[i]);
#if CCG_PD_DUALPORT_ENABLE
                    if(false == dpm_slept[i]) 
                    break;
#endif /* CCG_PD_DUALPORT_ENABLE */
                }
                if(true == dpm_slept[0] 
#if CCG_PD_DUALPORT_ENABLE
                   && true == dpm_slept[1]
#endif /* CCG_PD_DUALPORT_ENABLE */
                  )
                {

#if CCG_SYNC_ENABLE
                    CCGComm_Sleep();
#endif /* CCG_SYNC_ENABLE */

#if CCG_LIN_ENABLE
                    if(lins_sleep_allowed() == true)
                    {
                        lins_slept = true;
#endif /* CCG_LIN_ENABLE */

                /*
                 * Check connection status of TYPE-A and TYPE-C ports to determine if deepsleep
                 * entry is allowed. If not,enter sleep mode to save power.
                 */
                if (
#if CCG_TYPE_A_PORT_ENABLE
                    (type_a_is_idle() == true) &&
#endif /* CCG_TYPE_A_PORT_ENABLE */
                    /* Update: Ask for removal */
                    (app_deepsleep_allowed() == true)
                    )
                {
                    Cy_PdUtils_SwTimer_EnterSleep(solution_fn_handler->Get_PdStack_Context(0)->ptrTimerContext);

                    /*
                     * CDT 224642: The I2C IDLE check needs to be done as the last step
                     * before device enters into sleep. Otherwise, the device may fail
                     * to wake up when there is an address match on the I2C interface.
                     */
#if ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE))
                    if (
#if CCG_HPI_ENABLE
                            (CALL_MAP(hpi_sleep)() != false)
#if CCG_HW_DRP_TOGGLE_ENABLE
                            &&
                            (!pd_hal_is_cc_start_pending ())
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */

#else
                            (true)
#endif /* CCG_HPI_ENABLE */
                            &&
#if RIDGE_SLAVE_ENABLE
                            (ridge_slave_sleep())
#else
                            (true)
#endif /* RIDGE_SLAVE_ENABLE */
                       )
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */
                    {
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if (NO_OF_TYPEC_PORTS > 1)
                                for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
                                i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
                                {
#if CCG_VIN_BASED_VOLTAGE_THROTTLING
#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))
                                    ccg_set_sensor_sleep_mode(solution_fn_handler->Get_PdStack_Context(i));
#endif /* (VIN_OVP_ENABLE) || (VIN_UVP_ENABLE) */
#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING */
                                    Cy_USBPD_SetReference(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, true);
#if DISABLE_CC_OVP_OVER_SLEEP
                                    Cy_USBPD_CcOvpControl(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, false);
#endif /* DISABLE_CC_OVP_OVER_SLEEP */
                                }
#if (defined(CY_DEVICE_CCG7D) && (VBAT_GND_SCP_ENABLE))
                                /*
                                 * In CCG7D ** silicon, VBAT_GND_SCP is not supported in
                                 * deep sleep mode.
                                 * Whereas,
                                 * *A silicon supports this.
                                 * Hence, use additional clocks disable for ** silicon
                                 * to save more power in sleep mode.
                                 */
                                if (Cy_USBPD_Get_Silicon_Rev(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext) == 0u)
                                {
                                    Cy_USBPD_Vbus_SystemClockDisable(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext);
                                }
#endif /* (defined(CY_DEVICE_CCG7D) && (VBAT_GND_SCP_ENABLE))*/
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if (((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID)) || (RIDGE_SLAVE_ENABLE))
#if ((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID))

                                hpi_scb_idle = CALL_MAP(i2c_scb_is_idle)(HPI_SCB_INDEX);
                                scb_idle = scb_idle && hpi_scb_idle;
#endif /* ((CCG_HPI_ENABLE) && (HPI_SCB_INDEX != HPI_SCB_INDEX_INVALID)) */
#if (RIDGE_SLAVE_ENABLE)
                                ridge_slave_scb_idle = ((SCB_PRT[RIDGE_SLAVE_SCB]->i2c_status & SCB_I2C_STATUS_BUS_BUSY) == 0);
                                scb_idle &= ridge_slave_scb_idle;
#endif /* (RIDGE_SLAVE_ENABLE) */
                                if (true == scb_idle)
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */
                                {
#if (defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE)
                                    /* CC-PHY Wrapper Disabled */
                                    Cy_USBPD_TypeC_CgndWrapperDisable(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext);
#endif /* defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE */
                            /* Device sleep entry. */
#if defined(CY_DEVICE_CCG7D)
                                    /*
                                     * In CCG7D ** silicon, VBAT_GND_SCP is not supported in
                                     * deep sleep mode.
                                     * Whereas,
                                     * *A silicon supports this.
                                     */
                                    if (Cy_USBPD_Get_Silicon_Rev(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext) != 0u)
                                    {
                                        /* Enter Deep Sleep mode to save power. */
                                        Cy_SysPm_CpuEnterDeepSleep();
                                    }
                                    else
                                    {
#if VBAT_GND_SCP_ENABLE
                                        /* Enter Sleep mode to save power. */
                                        Cy_SysPm_CpuEnterSleep();
#else /* !VBAT_GND_SCP_ENABLE */
                                        /* Enter Deep Sleep mode to save power. */
                                        Cy_SysPm_CpuEnterDeepSleep();
#endif /* VBAT_GND_SCP_ENABLE */
                                    }
#else
                            Cy_SysPm_CpuEnterDeepSleep();
#endif /* defined(CY_DEVICE_CCG7D) */
                        }
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if (defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE)
                                /* CC-PHY Wrapper Enabled */
                                Cy_USBPD_TypeC_CgndWrapperEnable(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext);
#endif /* defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE */
#if (defined(CY_DEVICE_CCG7D) && (VBAT_GND_SCP_ENABLE))
                                if (Cy_USBPD_Get_Silicon_Rev(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext) == 0u)
                                {
                                    Cy_USBPD_Vbus_SystemClockEnable(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext);
                                }
#endif /* defined(CY_DEVICE_CCG7D) && (VBAT_GND_SCP_ENABLE) */
#if (NO_OF_TYPEC_PORTS > 1)
                                for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
                                i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
                                {
                                    Cy_USBPD_SetReference(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, false);
#if DISABLE_CC_OVP_OVER_SLEEP
                                    Cy_USBPD_CcOvpControl(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, true);
#endif /* DISABLE_CC_OVP_OVER_SLEEP */
                                }
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
                        retval = true;
                    }
                }
                else
                {
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
                    /* Enter Sleep mode to save power. */
                    Cy_SysPm_CpuEnterSleep();
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
                }
#if CCG_LIN_ENABLE
                    }
#endif /* CCG_LIN_ENABLE */


#if CCG_SYNC_ENABLE
                CCGComm_Wakeup();
#endif /* CCG_SYNC_ENABLE */
                }
            }
#if BATTERY_CHARGING_ENABLE
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

    Cy_SysLib_ExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if(true == dpm_slept[0] 
#if CCG_PD_DUALPORT_ENABLE
       && true == dpm_slept[1]
#endif /* CCG_PD_DUALPORT_ENABLE */
     )
    {
      for(i = 0; i < NO_OF_TYPEC_PORTS; i++)
      {
      (void)Cy_PdStack_Dpm_Resume(solution_fn_handler->Get_PdStack_Context(i), &dpm_resume);
      }
    }

    /* Call app_wakeup() if app_sleep() had returned true. */
    if(app_slept)
    {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        app_wakeup();
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
    }

#if CCG_LIN_ENABLE
    if(lins_slept)
    {
        lins_wakeup();
    }
#endif /* CCG_LIN_ENABLE */

#if BATTERY_CHARGING_ENABLE
    if(bc_slept)
    {
        (void)bc_wakeup();
    }
#endif /* BATTERY_CHARGING_ENABLE */

    return retval;
}

#endif /* (CCG_APP_TASK_LPM_SLOW_POLL_ENABLE || SYS_DEEPSLEEP_ENABLE) && (!CCG_BOOT) */

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
#if VCONN_OCP_ENABLE
static bool app_vconn_ocp_cbk(void * callbackCtx, bool compOut)
{
    (void)compOut;
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackCtx;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpd_ctx->pdStackContext;

    /* Disable VConn since we hit a fault. */
    (void)Cy_USBPD_Vconn_Disable(context->ptrUsbPdContext, context->dpmConfig.revPol);

    /* Notify application layer about fault. */
    app_event_handler(context, APP_EVT_VCONN_OCP_FAULT, NULL);

    return true;
}
#endif /* VCONN_OCP_ENABLE */

#if VCONN_SCP_ENABLE
static bool app_vconn_scp_cbk(void * callbackCtx, bool compOut)
{
    (void)compOut;
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackCtx;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpd_ctx->pdStackContext;

    /* Disable VConn since we hit a fault. */
    (void)Cy_USBPD_Vconn_Disable(context->ptrUsbPdContext, context->dpmConfig.revPol);

    /* Notify application layer about fault. */
    app_event_handler(context, APP_EVT_VCONN_SCP_FAULT, NULL);

    return true;
}
#endif /* VCONN_SCP_ENABLE */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

/* Delay between Vconn switch enable and Vconn OCP/SCP enable */
#define APP_VCONN_OCP_SCP_EN_TIME_MS            (5u)

static void app_vconn_ocp_scp_en_cb(cy_timer_id_t id,  void * callbackCtx)
{
    (void)callbackCtx;
    (void)id;

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
    cy_stc_pdstack_context_t* context = callbackCtx;
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

    /* Enable VCONN OCP and SCP as VCONN is now stabilized */

#if VCONN_OCP_ENABLE
    (void)system_vconn_ocp_en(context, app_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

#if VCONN_SCP_ENABLE
    (void)system_vconn_scp_en(context, app_vconn_scp_cbk);
#endif /* VCONN_SCP_ENABLE */
}
#endif /* defined(CY_DEVICE_CCG7D) */

#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!CCG_VCONN_DISABLE)
#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))

    /* Do not attempt to enable VConn if fault was detected in present connection. */
    if ((app_get_status(ptrPdStackContext->port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0u)
    {
        return false;
    }

#if CCG_VCONN_MON_WITH_ADC

    /* If Vconn current is being monitored through ADC, configure the associated IOs. */
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P1,APP_VCONN_MON_AMUX_INPUT_P1);
#if CCG_PD_DUALPORT_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P2,APP_VCONN_MON_AMUX_INPUT_P2);
#endif /* CCG_PD_DUALPORT_ENABLE */

    /*
     * 120us delay required as a settling time after HSIOM config to get a stable
     * ADC reading.
     */
    Cy_SysLib_DelayUs(120);

#endif /* CCG_VCONN_MON_WITH_ADC */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /*
     * CCG7D VCONN OCP is <50mA, hence enabled after VCONN is enabled.
     * VCONN SCP shall be enabled by default to protect on surge
     * during VCONN switch enable.
     */
#else /* !(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
    system_vconn_ocp_en(port, app_vconn_ocp_cbk);
#endif /* CCGx */
#endif /* VCONN_OCP_ENABLE */

    /* Reset RX Protocol for cable */
    (void)Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_PRIME);
    (void)Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_DPRIME);

    if (Cy_USBPD_Vconn_Enable(ptrPdStackContext->ptrUsbPdContext, channel) != CY_USBPD_STAT_SUCCESS)
    {
#if ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)) || (defined(CY_DEVICE_CCG7D)) || defined(CY_DEVICE_CCG7S))
        app_get_status(ptrPdStackContext->port)->fault_status |= APP_PORT_V5V_SUPPLY_LOST;
#endif /* ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S))) */
        return false;
    }

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Enable VCONN OCP after the VCONN switch output is stable */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_VCONN_OCP_TIMER), APP_VCONN_OCP_SCP_EN_TIME_MS,
                                app_vconn_ocp_scp_en_cb);
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */
    return true;
#else
    return false;
#endif /* (!CCG_VCONN_DISABLE) */
}

void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!CCG_VCONN_DISABLE)

    (void)Cy_USBPD_Vconn_Disable(ptrPdStackContext->ptrUsbPdContext, channel);

#if VCONN_OCP_ENABLE
    system_vconn_ocp_dis(ptrPdStackContext);
#endif /* VCONN_OCP_ENABLE */

#if VCONN_SCP_ENABLE
    system_vconn_scp_dis(ptrPdStackContext);
#endif /* VCONN_SCP_ENABLE */

#if ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE))
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Disable VCONN OCP timer */
    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext,APP_VCONN_OCP_TIMER));
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#endif /* ((VCONN_OCP_ENABLE) || (VCONN_SCP_ENABLE)) */

#else
    (void)ptrPdStackContext;
    (void)channel;
#endif /* (!CCG_VCONN_DISABLE) */
}

bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (!CCG_VCONN_DISABLE)
    return Cy_USBPD_Vconn_IsPresent(ptrPdStackContext->ptrUsbPdContext, ptrPdStackContext->dpmConfig.revPol);
#else
    return false;
#endif /* (!CCG_VCONN_DISABLE) */
}

bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per)
{
#if PSVP_FPGA_ENABLE
    bool ret;

    uint16_t vbus_volt =
            Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
                                        APP_VBUS_POLL_ADC_ID,
                    APP_VBUS_POLL_ADC_INPUT);
    uint16_t vbus_lim = (volt + ((per * volt) / 100));

    if(vbus_volt >= vbus_lim)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }

    return ret;
#else /* !PSVP_FPGA_ENABLE */
    uint8_t level;
    bool retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling.
     * To avoid false output on OVP Trip pin when VBUS is polled
     * OVP trip pin is disconnected from OVP comp output and last
     * value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4*/

#ifdef CCG3
    /* Ensure internal VBus divider is connected to AMUX where required. */
    pd_connect_vbus_div_to_amux(port);
#endif /* CCG3 */

    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    (void)Cy_USBPD_Adc_Calibrate(ptrPdStackContext->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID);
    level =  Cy_USBPD_Adc_GetVbusLevel(ptrPdStackContext->ptrUsbPdContext, 
                           APP_VBUS_POLL_ADC_ID, 
                           volt, per);

#if CCG_BACKUP_FIRMWARE
    /* Using pd_adc_sample so that pd_adc_comparator_sample is removed from the binary. */
    if (Cy_USBPD_Adc_CompSample (ptrPdStackContext->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT) >= level)
    {
        retVal = true;
    }
#else
    retVal = Cy_USBPD_Adc_CompSample(ptrPdStackContext->ptrUsbPdContext,
                     APP_VBUS_POLL_ADC_ID, 
                     APP_VBUS_POLL_ADC_INPUT, level);
#endif /* CCG_BACKUP_FIRMWARE */

#ifdef CCG3
    /* Disconnect internal VBus divider from AMUX if possible. */
    pd_disconnect_vbus_div_from_amux(port);
#endif /* CCG3 */

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;

#endif /* !PSVP_FPGA_ENABLE */
}

uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint16_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling. To avoid false output
     * on OVP Trip pin when VBUS is polled OVP trip pin is disconnected from
     * OVP comp output and last value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4 */

#if defined(CY_DEVICE_SERIES_WLC1)
    retVal = ptrPdStackContext->ptrAppCbk->vbus_get_value(ptrPdStackContext);
#else /* !defined(CY_DEVICE_SERIES_WLC1) */
    /* Update : Update APP_VBUS in stack_params.h */
    /* Measure the actual VBUS voltage. */
    retVal = Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
                          APP_VBUS_POLL_ADC_ID,
                          APP_VBUS_POLL_ADC_INPUT);
#endif /* defined(CY_DEVICE_SERIES_WLC1) */

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

#if VCONN_OCP_ENABLE
bool system_vconn_ocp_en(cy_stc_pdstack_context_t *context, cy_cb_vbus_fault_t cbk)
{
    if (cbk == NULL)
    {
        return false;
    }

#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

    if (pd_get_ptr_vconn_ocp_tbl(context->ptrUsbPdContext)->enable)
    {
        /* Request the HAL to enable OCP detection with the appropriate debounce period. */
        Cy_USBPD_Fault_Vconn_OcpEnable(context->ptrUsbPdContext, cbk);
    }
    else
    {
        return false;
    }

#else /* (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || (defined(CY_DEVICE_CCG6DF)) ||\
        (defined(CY_DEVICE_CCG6SF)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr_1_cfg |= CYVAL_USBPD_V5V_CFG_POS_EDG_DIS_NEG_EDG_EN << PDSS_INTR_1_CFG_V5V_CFG_POS;
    pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Configure ADC to detect VConn Over-Current condition. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, APP_VCONN_OCP_ADC_INPUT,
            APP_VCONN_TRIGGER_LEVEL, PD_ADC_INT_RISING, cbk);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) || (defined(CCG6DF)) || (defined(CCG6SF)) */
    return true;
}

void system_vconn_ocp_dis(cy_stc_pdstack_context_t *context)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C) || (defined(CCG6DF)) || (defined(CCG6SF)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

    /* Disable VConn OCP detection in the HAL. */
    Cy_USBPD_Fault_Vconn_OcpDisable(context->ptrUsbPdContext);
#else /* !defined (CCG5) || defined(CCG6) || defined(CCG5C) || (defined(CCG6DF)) || (defined(CCG6SF)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)*/
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Disable the ADC used for VConn OCP detection. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, 0, 0, 0, NULL);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) || (defined(CCG6DF)) || (defined(CCG6SF)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)*/
}

#endif /* VCONN_OCP_ENABLE */

bool set_custom_host_cap_control(uint8_t port, uint8_t host_config)
{
    bool                 ret = false;
    (void)host_config;
    (void)port;
#if HPI_HOST_CAP_ENABLE
    const    dpm_status_t*     dpm_stat = dpm_get_info(port);
    volatile uint8_t prev_host_config   = app_get_status(port)->custom_hpi_host_cap_control;
    /* Proceed only if custom host_config has been changed. */
    if (host_config != prev_host_config)
    {
        app_get_status(port)->custom_hpi_host_cap_control = host_config;
        /* Provide data reset if host capabilities changed */
        if (
               (dpm_stat->pd_connected == true)    &&
               (dpm_stat->usb4_en)                 &&
               (app_get_status(port)->usb4_active)
            )
        {
            /* Provide data reset if host capabilities changed */
            dpm_pd_command(port, DPM_CMD_SEND_DATA_RESET, NULL, NULL);
        }
        ret = true;
    }
#endif /*HPI_HOST_CAP_ENABLE*/
    return ret;
}

#if VCONN_SCP_ENABLE
bool system_vconn_scp_en(cy_stc_pdstack_context_t *context, cy_cb_vbus_fault_t cbk)
{
    if (cbk == NULL)
    {
        return false;
    }

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Enable VConn SCP detection in the HAL. */
    Cy_USBPD_Fault_Vconn_ScpEnable(context->ptrUsbPdContext, cbk);
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
    return true;
}

void system_vconn_scp_dis(cy_stc_pdstack_context_t *context)
{
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Disable VConn SCP detection in the HAL. */
    Cy_USBPD_Fault_Vconn_ScpDisable(context->ptrUsbPdContext);
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
}

#endif /* VCONN_SCP_ENABLE */

bool send_src_info (struct cy_stc_pdstack_context *ptrPdStackContext)
{
    bool is_supported = false;
    cy_stc_pdstack_dpm_ext_status_t* ptrDpmExtStat = &(ptrPdStackContext->dpmExtStat);
    bool temp_throttle = false;
    bool vin_throttle = false;
    bool load_sharing = false;
#if CCG_TEMP_BASED_VOLTAGE_THROTTLING
    if((ccg_power_throttle_get_feature_mask(ptrPdStackContext) & CCG_ENABLE_TEMP_BASED_THROTTLING) != 0u)
    {
       temp_throttle = true;
    }
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */  
#if CCG_VIN_BASED_VOLTAGE_THROTTLING
    if((ccg_power_throttle_get_feature_mask(ptrPdStackContext) & CCG_ENABLE_VIN_BASED_THROTTLING) != 0u)
    {
      vin_throttle = true;
    }
#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING */
#if CCG_LOAD_SHARING_ENABLE
    if(ccg_ls_is_enabled(ptrPdStackContext))
    {
      load_sharing = true;
    }
    else
    {
        ptrDpmExtStat->srcInfo.src_info.portType = 1u;
    }
#endif /* CCG_LOAD_SHARING_ENABLE */
    if(temp_throttle || vin_throttle || load_sharing)
    {
        is_supported = true;
    }
    (void)ptrDpmExtStat;
    return is_supported;
}

bool app_is_host_hpd_virtual(uint8_t port)
{
    bool ret = false;

#if RIDGE_I2C_HPD_ENABLE
    if (
            (get_pd_port_config(ptrPdStackContext->ptrUsbPdContext)->tbthost_cfg_tbl_offset != 0) &&
            (PD_GET_PTR_TBTHOST_CFG_TBL(port)->hpd_handling != 0)
       )
#else
    (void)port;
#endif /* RIDGE_I2C_HPD_ENABLE */

    return ret;
}

bool app_sink_rdo_handler(uint8_t port, cy_pd_pd_do_t snk_rdo)
{
    bool resp = false;
    (void)port;
    (void)snk_rdo;

#if (HPI_PPS_SINK_SUPPORT != 0)
    pd_do_t pdo;
    dpm_pd_cmd_buf_t buf;


    dpm_status_t *dpm_stat = dpm_get_status(port);

    /* Perform sanity checks on validity of the RDO. */
    pdo = dpm_stat->src_cap_p->dat[snk_rdo.rdo_gen.obj_pos - 1];

    if (
            (pdo.pps_src.supply_type == PDO_AUGMENTED) &&
            (pdo.pps_src.apdo_type == APDO_PPS) &&
            ((pdo.pps_src.max_volt * 5) >= snk_rdo.rdo_pps.out_volt) &&
            ((pdo.pps_src.min_volt * 5) <= snk_rdo.rdo_pps.out_volt) &&
            (pdo.pps_src.max_cur >= snk_rdo.rdo_pps.op_cur)
       )
    {
        dpm_stat->pps_snk_rdo = snk_rdo;
        dpm_stat->pps_snk_en = true;
    }
    else
    {
        dpm_stat->pps_snk_en = false;
    }

    /* Make request message */
    buf.cmd_do[0].val = snk_rdo.val;

    /* Send Request message */
    if(dpm_pd_command(port, DPM_CMD_SEND_REQUEST, &buf , NULL) == CCG_STAT_SUCCESS)
    {
        if(dpm_stat->pps_snk_en)
        {
            /* Restart the timer to attempt the sequence again after the defined period. */
            timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_PERIOD, snk_recontract_timer_cb);
        }
        resp = true;
    }
#endif
    return resp;
}

#if (HPI_PPS_SINK_SUPPORT != 0)
static void snk_recontract_cb (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    if (resp == SEQ_ABORTED)
    {
        /* Restart the timer so that the command can be retried. */
        timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_RETRY_PERIOD, snk_recontract_timer_cb);
    }
    else
    {
        /* Restart the timer to attempt the sequence again after the defined period. */
        timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_PERIOD, snk_recontract_timer_cb);
    }
}

static void snk_recontract_timer_cb (uint8_t port, timer_id_t id)
{
    dpm_status_t *dpm_stat = dpm_get_status(port);
    dpm_pd_cmd_buf_t    param;
    ccg_status_t        stat;

    if ((dpm_stat->pps_snk_en) && (dpm_stat->contract_exist != 0) && (dpm_stat->cur_port_role == PRT_ROLE_SINK) &&
        (dpm_stat->spec_rev_sop_live >= PD_REV3))
    {
        param.cmd_sop      = SOP;
        param.no_of_cmd_do = 0;
        param.timeout      = 0;
        param.cmd_do[0].val = dpm_stat->pps_snk_rdo.val;

        stat = dpm_pd_command (port, DPM_CMD_SEND_REQUEST, &param, snk_recontract_cb);
        if (stat != CCG_STAT_SUCCESS)
        {
            /* Restart the timer so that the command can be retried. */
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start) (port, id, 5u, snk_recontract_timer_cb);
        }
    }
    else
    {
        dpm_stat->pps_snk_en = false;
        dpm_stat->pps_snk_rdo.val = 0;
    }
}

void app_pps_sink_disable(uint8_t port)
{
    solution_fn_handler->Get_PdStack_Context(port)->dpmStat.ppsSnkEn = 0u;
}

#endif /* (HPI_PPS_SINK_SUPPORT != 0) */

void register_soln_function_handler(cy_stc_pdstack_context_t *ptrPdStackcontext, app_sln_handler_t *handler)
{
    (void)ptrPdStackcontext;
    /* Register the solution level handler functions */
    solution_fn_handler = handler;
}
#if CCG_LIN_ENABLE
/**
 * @brief Used to Get the fault status
 * @param context Pointer to pdstack context
 * @param evt App event to be handled
 * @return None.
 */
void lins_fault_handle(cy_stc_pdstack_context_t * context, cy_en_pdstack_app_evt_t evt)
{
    bool common_fault_stat = false;
    bool port_specific_fault_stat = false;
    uint8_t port;

    port = context->port;

    switch(evt)
    {
        case APP_EVT_POWER_CYCLE:
            common_fault_stat = true;
            break;

        case APP_EVT_VBUS_UVP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_OVP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_OCP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_SCP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_RCP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_IN_UVP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBUS_IN_OVP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VCONN_OCP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_SYSTEM_OT_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_CRC_ERROR:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_CC_OVP:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VCONN_SCP_FAULT:  /* CC SCP Fault */
            port_specific_fault_stat = true;
            break;

        case APP_EVT_SBU_OVP:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_VBAT_GND_SCP_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_ILIM_FAULT:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_CONNECT:
        case APP_EVT_DISCONNECT:
            if(port == TYPEC_PORT_0_IDX)
            {
                (void)lins_wr_signal(PORT0_ATTACH_DETACH_EVT, 0X01);
            }
#if (NO_OF_TYPEC_PORTS >= 2)
            else if(port == TYPEC_PORT_1_IDX)
            {
                (void)lins_wr_signal(PORT1_ATTACH_DETACH_EVT, 0X01);
            }
#endif /* NO_OF_TYPEC_PORTS >= 2 */
            else
            {
                /* No Statement */
            }
            break;

        case APP_EVT_HARD_RESET_RCVD:
            port_specific_fault_stat = true;
            break;

        case APP_EVT_HARD_RESET_SENT:
            port_specific_fault_stat = true;
            break;

        default :
            /* No Action Required */
            break;
    }

    if(common_fault_stat)
    {
        (void)lins_wr_signal(COMMON_FAULT_STAT, 0x01u);
    }

    if(port_specific_fault_stat)
    {
        if(port == TYPEC_PORT_0_IDX)
        {
            (void)lins_wr_signal(PORT0_FAULT_STAT, 0x01u);
        }
#if (NO_OF_TYPEC_PORTS >= 2)
        else if(port == TYPEC_PORT_1_IDX)
        {
            (void)lins_wr_signal(PORT1_FAULT_STAT, 0x01u);
        }
#endif /* NO_OF_TYPEC_PORTS >= 2 */
        else
        {
            /* No Statement */
        }
    }
}
#endif /* CCG_LIN_ENABLE */

#if ((CCG_LPM_GPIO_ENABLE || CCG_LIN_ENABLE) && (!LIN_BOOT))
/*******************************************************************************
* Function Name: CyDisableInts
********************************************************************************
*
* Summary:
*  All interrupts will be disabled
*
* Parameters:
*  void
*
* Returns:
*  interrupt status
*
*******************************************************************************/
static uint32_t CyDisableInts(void)
{
    uint32_t intState;

    /* Get current interrupt state. */
    intState = NVIC->ICER[0U];

    /* Disable all interrupts. */
    NVIC->ICER[0U] = 0xFFFFFFFFu;

    return (intState);
}
/*******************************************************************************
* Function Name: CyEnableInts
********************************************************************************
*
* Summary:
*  Enables the passed interrupts
*
* Parameters:
*  interrupt mask to be enabled
*
* Returns:
*  void
*
*******************************************************************************/
static void CyEnableInts(uint32_t mask)
{
    NVIC->ISER[0U] = mask;
}
/*******************************************************************************
* Function Name: system_lpm_sleep
********************************************************************************
*
* Summary:
*  Low Power Mode Entry will be taken care in this function
*
* Parameters:
*  void
*
* Returns:
*  void
*
*******************************************************************************/
void system_lpm_sleep(void)
{
    uint8_t dpm_enable_stat = 0x00u;
    bool dpm_lpm;
    bool dpm_resume;
    uint32_t intr_state;
    uint32_t intr_en_status;
    cy_stc_pdstack_context_t *context;

#if CCG_LPM_GPIO_ENABLE
    if(lpm_gpio_get_lpm_stat() == true)
#else /* CCG_LIN_ENABLE */
    if(lins_get_lpm_stat() == true)
    {
        if(lins_sleep_allowed() == true)
#endif /* CCG_LPM_GPIO_ENABLE */
        {
            intr_state = Cy_SysLib_EnterCriticalSection();

            /* Disable any other required functions */
            /* Disable the PD port */
            for (uint8_t i = 0u; i < NO_OF_TYPEC_PORTS; i++)
            {
                context = solution_fn_handler->Get_PdStack_Context(i);

                if (context->dpmConfig.dpmEnabled != false)
                {
                    dpm_enable_stat |= (0x01u << i);
                    (void)Cy_PdStack_Dpm_Stop(solution_fn_handler->Get_PdStack_Context(i));
                }
                Cy_PdStack_Dpm_PrepareDeepSleep(solution_fn_handler->Get_PdStack_Context(i), &dpm_lpm);
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if VBAT_GND_SCP_ENABLE
                /* Disable the VBAT_GND_SCP */
                Cy_USBPD_Fault_VbatGndScpDis(context->ptrUsbPdContext, CCG_SRC_FET);
#endif /* VBAT_GND_SCP_ENABLE */
#endif /* defined(CY_DEVICE_CCG7D) */
#if (defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE)
                /* CC-PHY Wrapper Disabled */
                Cy_USBPD_TypeC_CgndWrapperDisable(context->ptrUsbPdContext);
#endif /* defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE */
            }

            /* Put other features into sleep */

            /* Disable hardware watchdog reset. */
#if WATCHDOG_HARDWARE_RESET_ENABLE
            Cy_WDT_Disable();
#endif /* WATCHDOG_HARDWARE_RESET_ENABLE */

            /* Disable all the WakeUp Interrupts */
            intr_en_status = CyDisableInts();

            Cy_PdUtils_SwTimer_EnterSleep(solution_fn_handler->Get_PdStack_Context(0)->ptrTimerContext);

#if CCG_LPM_GPIO_ENABLE
            lpm_gpio_set_intr(GPIO_INTR_RISING);
#else
            /* Enable only the RX_WakeUp Interrupt */
            NVIC_EnableIRQ((IRQn_Type)(((uint8_t)CCG_LIN_SCB_PIN_RX) >> 4u));
#endif /* CCG_LPM_GPIO_ENABLE */

#if (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            for (uint8_t i = 0u; i < NO_OF_TYPEC_PORTS; i++)
            {
                Cy_USBPD_SetReference(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, true);
                /* Power down Reference generator whether vbat_gnd_scp is enabled or disabled */
                solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext->base->refgen_0_ctrl |= PDSS_REFGEN_0_CTRL_REFGEN_PD;
#if DISABLE_CC_OVP_OVER_SLEEP
                Cy_USBPD_CcOvpControl(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, false);
#endif /* DISABLE_CC_OVP_OVER_SLEEP */
            }
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if CCG_LPM_GPIO_ENABLE
            while (lpm_gpio_read() == false)
#endif /* CCG_LPM_GPIO_ENABLE */
            {
                /* Device sleep entry. */
                Cy_SysPm_CpuEnterDeepSleep();
#if CCG_LPM_GPIO_ENABLE
                lpm_gpio_clr_intr();
#else
                /* LIN Interrupt will get cleared in the wakeup handler */
#endif /* CCG_LPM_GPIO_ENABLE */
            }
#if CCG_LPM_GPIO_ENABLE
            lpm_gpio_clr_lpm_stat();
            /* Set the Interrupt Mode for the GPIO */
            lpm_gpio_set_intr(GPIO_INTR_FALLING);
#endif

#if (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
            for (uint8_t i = 0u; i < NO_OF_TYPEC_PORTS; i++)
            {
                Cy_USBPD_SetReference(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, false);
#if DISABLE_CC_OVP_OVER_SLEEP
                Cy_USBPD_CcOvpControl(solution_fn_handler->Get_PdStack_Context(i)->ptrUsbPdContext, true);
#endif /* DISABLE_CC_OVP_OVER_SLEEP */
            }
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
            Cy_SysLib_ExitCriticalSection(intr_state);

            /* Disable hardware watchdog reset. */
#if WATCHDOG_HARDWARE_RESET_ENABLE
            Cy_WDT_Enable();
#endif /* WATCHDOG_HARDWARE_RESET_ENABLE */

            /* Re-Enable all the Disabled Interrupts */
            CyEnableInts(intr_en_status);

#if CCG_LIN_ENABLE
            /* WakeUp all other Functions */
            lins_wakeup();
#endif /* CCG_LIN_ENABLE */

            /* Wake-Up other features */
            /* Enable the PD ports */
            for (uint8_t i = 0u; i < NO_OF_TYPEC_PORTS; i++)
            {
                (void)Cy_PdStack_Dpm_Resume(solution_fn_handler->Get_PdStack_Context(i), &dpm_resume);
                if((dpm_enable_stat & (0x01u << i)) != 0x00u )
                {
                    (void)Cy_PdStack_Dpm_Start(solution_fn_handler->Get_PdStack_Context(i));
                }
                /* VBAT_GND_SCP is enabled from dpm_start() */
#if (defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE)
                /* CC-PHY Wrapper Disabled */
                Cy_USBPD_TypeC_CgndWrapperEnable(context->ptrUsbPdContext);
#endif /* defined(CY_DEVICE_CCG7S) && CY_USBPD_CGND_SHIFT_ENABLE */
            }
        }
#if !CCG_LPM_GPIO_ENABLE
    }
#endif /* !CCG_LPM_GPIO_ENABLE */
    CY_UNUSED_PARAMETER(intr_state);
}
#endif /* (CCG_LPM_GPIO_ENABLE || CCG_LIN_ENABLE) && (!LIN_BOOT) */

bool app_is_typec_attached(void)
{
    bool attached = false;
    uint8_t i;

#if (NO_OF_TYPEC_PORTS > 1)
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#else
    i = 0;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
    {
        if (solution_fn_handler->Get_PdStack_Context(i)->dpmConfig.attach == true)
        {
            attached = true;
#if (NO_OF_TYPEC_PORTS > 1)
            break;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
        }
    }

    return attached;
}
/* End of File */
