/******************************************************************************
* File Name: ccg7d_rom_wrapper.c
* \version 2.0
*
* Description: Source file for CCG7D ROM Wrapper Functions
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/
#include "config.h"
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
#include <power_throttle.h>
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
#include "ccg7d_rom_wrapper.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_common.h"
#include "app.h"
#if (CCG_LOAD_SHARING_ENABLE)
#if CCG_LS_INTER_INTRA_ENABLE
#include <loadsharing_inter_intra.h>
#else
#include <loadsharing.h>
#endif /* CCG_LS_INTER_INTRA_ENABLE */
#endif /* (CCG_LOAD_SHARING_ENABLE) */
#include "sensor_check.h"
#if (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE))
extern cy_stc_pdstack_context_t * get_pdstack_context(uint8_t portIdx);
dpm_status_t gl_legacy_dpm_stat[NO_OF_TYPEC_PORTS];

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
/* Declaring global pointer to register the ROM callback */
srom_ccg_power_contract_complete callback_set_pdp;
srom_ccg_power_contract_complete callback_get_set_oc;
srom_ccg_power_contract_complete callback_ls_ctrl;
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */

dpm_typec_cmd_cbk_t app_disable_callback;

cy_israddress CyIntSetVector(uint8_t number, cy_israddress address)
{
    IRQn_Type IRQn=(IRQn_Type)number;
    cy_israddress userIsr=(cy_israddress)address;
    return((cy_israddress)(Cy_SysInt_SetVector(IRQn,userIsr)));
}

void CyIntEnable(uint8_t number)
{
    IRQn_Type IRQn=(IRQn_Type)number;
    __NVIC_EnableIRQ (IRQn);
}

void CyIntDisable (uint8_t number)
{
    IRQn_Type IRQn=(IRQn_Type)number;
    __NVIC_DisableIRQ (IRQn);
}

bool timer_start(uint8_t instance, cy_timer_id_t id, uint16_t period, cy_cb_timer_t cb)
{
#ifdef CY_DEVICE_SERIES_WLC1
    SYSTICK_VAL = 0u;
    SYSTICK_CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk ) ;
#else
#if 0
    cy_stc_pdstack_context_t *context=get_pdstack_context(instance);
    /* Mapping old timer ids with new ones. CCG7D ROM was creator based */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    return(Cy_PdUtils_SwTimer_Start_legacy_enable(context->ptrTimerContext, context,temp_id,period,cb));
#endif
#endif

    return true;
}

void timer_stop (uint8_t instance, cy_timer_id_t id)
{
#ifdef CY_DEVICE_SERIES_WLC1
    SYSTICK_CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
#else
#if 0
    cy_stc_pdstack_context_t *context=get_pdstack_context(instance);
    /* Mapping old timer ids with new ones. CCG7D ROM was creator based */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    Cy_PdUtils_SwTimer_Stop(context->ptrTimerContext, temp_id);
#endif

#endif
}

void timer_stop_range(uint8_t instance, cy_timer_id_t start, cy_timer_id_t stop)
{
#ifndef CY_DEVICE_SERIES_WLC1
    cy_stc_pdstack_context_t *context=get_pdstack_context(instance);
    /* Mapping old timer ids with new ones. CCG7D ROM was creator based */
    cy_timer_id_t temp_id_start = cy_sw_psoc_modus_id(true, (cy_timer_id_t)start);
    cy_timer_id_t temp_id_stop = cy_sw_psoc_modus_id(true, (cy_timer_id_t)stop);
    Cy_PdUtils_SwTimer_StopRange(context->ptrTimerContext, temp_id_start, temp_id_stop);
#endif
}

bool timer_is_running(uint8_t instance, cy_timer_id_t id)
{
#ifndef CY_DEVICE_SERIES_WLC1
    cy_stc_pdstack_context_t *context=get_pdstack_context(instance);
    /* Mapping old timer ids with new ones. CCG7D ROM was creator based */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    return(Cy_PdUtils_SwTimer_IsRunning(context->ptrTimerContext, temp_id));
#else
    return false;
#endif
}

uint8_t CyEnterCriticalSection(void)
{
    return (uint8_t)Cy_SysLib_EnterCriticalSection();
}

void CyExitCriticalSection(uint8_t savedIntrStatus)
{
    Cy_SysLib_ExitCriticalSection((uint32_t)savedIntrStatus);
}

const dpm_status_t* dpm_get_info (uint8_t port)
{
    uint8_t loop;
    cy_stc_pdstack_context_t *context=get_pdstack_context(port);
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
    cy_stc_pd_dpm_config_t* dpm_cfg = &(context->dpmConfig);
    cy_stc_pdstack_dpm_ext_status_t* dpm_ext= &(context->dpmExtStat);

    memcpy((void *)&(gl_legacy_dpm_stat[port].alert),(void *)&(dpm_stat->alert),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].app_cbk = (app_cbk_t*)(context->ptrAppCbk);
    gl_legacy_dpm_stat[port].attach = dpm_cfg->attach;
    gl_legacy_dpm_stat[port].attached_dev = (pd_devtype_t)(dpm_cfg->attachedDev);
    gl_legacy_dpm_stat[port].bist_cm2_enabled = dpm_stat->bistCm2Enabled;
    gl_legacy_dpm_stat[port].bist_stm_enabled = dpm_stat->bistStmEnabled;
    gl_legacy_dpm_stat[port].bootup = dpm_stat->bootup;
    gl_legacy_dpm_stat[port].cbl_dsc = dpm_cfg->cblDsc;
    gl_legacy_dpm_stat[port].cbl_mode_en = dpm_stat->cblModeEn;
    gl_legacy_dpm_stat[port].cbl_soft_reset_tried = dpm_stat->cblSoftResetTried;
    gl_legacy_dpm_stat[port].cbl_state = (pe_cbl_state_t)(dpm_stat->cblState);
    gl_legacy_dpm_stat[port].cbl_type = (std_vdm_prod_t)(dpm_stat->cblType);
    gl_legacy_dpm_stat[port].cbl_vdm_version = (std_vdm_ver_t)(dpm_stat->cblVdmVersion);
    memcpy((void *)&(gl_legacy_dpm_stat[port].cbl_vdo),(void *)&(dpm_stat->cblVdo),sizeof(pd_do_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].cbl_vdo_2),(void *)&(dpm_stat->cblVdo2),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].cbl_wait = dpm_stat->cblWait;
    memcpy((void *)&(gl_legacy_dpm_stat[port].cc_live),(void *)&(dpm_stat->ccLive),sizeof(cc_state_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].cc_old_status),(void *)&(dpm_cfg->ccOldStatus),sizeof(cc_state_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].cc_rd_status),(void *)&(dpm_stat->ccRdStatus),sizeof(cc_state_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].cc_status),(void *)&(dpm_stat->ccStatus),sizeof(cc_state_t));
    gl_legacy_dpm_stat[port].cmd_p = (dpm_pd_cmd_buf_t *)(dpm_stat->cmdP);
    gl_legacy_dpm_stat[port].connect = dpm_cfg->connect;
    memcpy((void *)&(gl_legacy_dpm_stat[port].contract),(void *)&(dpm_stat->contract),sizeof(contract_t));
    gl_legacy_dpm_stat[port].contract_exist = dpm_cfg->contractExist;
    gl_legacy_dpm_stat[port].cur_fb = dpm_stat->curFb;
    gl_legacy_dpm_stat[port].cur_port_role = (port_role_t)(dpm_cfg->curPortRole);
    gl_legacy_dpm_stat[port].cur_port_type = (port_type_t)(dpm_cfg->curPortType);

    gl_legacy_dpm_stat[port].cur_snk_pdo_count = dpm_stat->curSnkPdocount;
    gl_legacy_dpm_stat[port].cur_src_pdo_count = dpm_stat->curSrcPdocount;
    gl_legacy_dpm_stat[port].db_support = dpm_stat->dbSupport;
    gl_legacy_dpm_stat[port].dead_bat = dpm_stat->deadBat;
    gl_legacy_dpm_stat[port].dflt_port_role = (port_role_t)dpm_stat->dfltPortRole;
    memcpy((void *)&(gl_legacy_dpm_stat[port].dpm_cmd_buf),(void *)&(dpm_stat->dpmCmdBuf),sizeof(dpm_pd_cmd_buf_t));
    gl_legacy_dpm_stat[port].dpm_enabled = dpm_cfg->dpmEnabled;
    gl_legacy_dpm_stat[port].dpm_err_info = dpm_stat->dpmErrInfo;
    gl_legacy_dpm_stat[port].dpm_init = dpm_stat->dpmInit;
    gl_legacy_dpm_stat[port].dpm_pd_cbk = (dpm_pd_cmd_cbk_t)dpm_stat->dpmPdCbk;
    gl_legacy_dpm_stat[port].dpm_pd_cmd = (dpm_pd_cmd_t)dpm_stat->dpmPdCmd;
    gl_legacy_dpm_stat[port].dpm_pd_cmd_active = dpm_stat->dpmPdCmdActive;
    gl_legacy_dpm_stat[port].dpm_safe_disable = dpm_stat->dpmSafeDisable;
    gl_legacy_dpm_stat[port].dpm_typec_cbk = (dpm_typec_cmd_cbk_t)(dpm_stat->dpmTypecCbk);
    gl_legacy_dpm_stat[port].dpm_typec_cmd = (dpm_typec_cmd_t)(dpm_stat->dpmTypecCmd);
    gl_legacy_dpm_stat[port].dpm_typec_cmd_active = dpm_stat->dpmTypecCmdActive;
    gl_legacy_dpm_stat[port].emca_present = dpm_cfg->emcaPresent;
    gl_legacy_dpm_stat[port].err_recov = dpm_stat->errRecov;
    gl_legacy_dpm_stat[port].fault_active = dpm_stat->faultActive;
    gl_legacy_dpm_stat[port].fr_rx_disabled = dpm_stat->frRxDisabled;
    gl_legacy_dpm_stat[port].fr_rx_en_live = dpm_cfg->frRxEnLive;
    gl_legacy_dpm_stat[port].fr_tx_disabled = dpm_stat->frTxDisabled;
    gl_legacy_dpm_stat[port].fr_tx_en_live = dpm_cfg->frTxEnLive;
    gl_legacy_dpm_stat[port].frs_enable = dpm_stat->frsEnable;
    gl_legacy_dpm_stat[port].frs_rx_en = dpm_stat->frsRxEn;
    gl_legacy_dpm_stat[port].frs_tx_en = dpm_stat->frsTxEn;
    gl_legacy_dpm_stat[port].hw_drp_toggle_en = dpm_stat->hwDrpToggleEn;
    gl_legacy_dpm_stat[port].is_snk_bat = dpm_stat->isSnkBat;
    gl_legacy_dpm_stat[port].is_src_bat = dpm_stat->isSrcBat;
    gl_legacy_dpm_stat[port].non_intr_response = (pd_ams_type)dpm_stat->nonIntrResponse;
    gl_legacy_dpm_stat[port].padval = dpm_stat->padval;
    gl_legacy_dpm_stat[port].pd3_src_cc_busy = dpm_stat->pd3SrcCcBusy;
    gl_legacy_dpm_stat[port].pd_connected = dpm_stat->pdConnected;
    gl_legacy_dpm_stat[port].pd_disabled = dpm_stat->pdDisabled;
    gl_legacy_dpm_stat[port].pd_support = dpm_stat->pdSupport;
    gl_legacy_dpm_stat[port].pe_evt = dpm_stat->peEvt;
    gl_legacy_dpm_stat[port].pe_fsm_state = (pe_fsm_state_t)dpm_stat->peFsmState;
    gl_legacy_dpm_stat[port].polarity = dpm_cfg->polarity;
    gl_legacy_dpm_stat[port].port_disable = dpm_stat->portDisable;
    gl_legacy_dpm_stat[port].port_role = (port_role_t)dpm_stat->portRole;
    memcpy((void *)&(gl_legacy_dpm_stat[port].port_status),(void *)&(dpm_stat->portStatus),sizeof(pd_power_status_t));
    gl_legacy_dpm_stat[port].pps_snk_en = dpm_stat->ppsSnkEn;
    memcpy((void *)&(gl_legacy_dpm_stat[port].pps_snk_rdo),(void *)&(dpm_stat->ppsSnkRdo),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].pps_src_en = dpm_stat->ppsSrcEn;
    gl_legacy_dpm_stat[port].pwr_limited = dpm_stat->pwrLimited;
    gl_legacy_dpm_stat[port].ra_present = dpm_stat->raPresent;
    gl_legacy_dpm_stat[port].rand_base = dpm_stat->randBase;
    gl_legacy_dpm_stat[port].rev3_en = dpm_stat->rev3En;
    gl_legacy_dpm_stat[port].rev_pol = dpm_cfg->revPol;
    gl_legacy_dpm_stat[port].role_at_connect = (port_role_t)dpm_stat->roleAtConnect;
    gl_legacy_dpm_stat[port].rp_supported = dpm_stat->rpSupported;
    gl_legacy_dpm_stat[port].skip_scan = dpm_cfg->skipScan;
    gl_legacy_dpm_stat[port].snk_cur_level = dpm_cfg->snkCurLevel;

    for(loop = 0u; loop < CY_PD_MAX_NO_OF_PDO; loop++)
    {
        memcpy((void *)&(gl_legacy_dpm_stat[port].snk_pdo[loop]),(void *)&(dpm_stat->snkPdo[loop]),sizeof(pd_do_t));
        gl_legacy_dpm_stat[port].cur_snk_max_min[loop] = dpm_stat->curSnkMaxMin[loop];
        memcpy((void *)&(gl_legacy_dpm_stat[port].cur_snk_pdo[loop]),(void *)&(dpm_stat->curSnkPdo[loop]),sizeof(pd_do_t));
        memcpy((void *)&(gl_legacy_dpm_stat[port].src_pdo[loop]),(void *)&(dpm_stat->srcPdo[loop]),sizeof(pd_do_t));
        gl_legacy_dpm_stat[port].ext_snk_cap[loop] = dpm_ext->extSnkCap[loop];
        gl_legacy_dpm_stat[port].ext_src_cap[loop] = dpm_ext->extSrcCap[loop];
        gl_legacy_dpm_stat[port].snk_max_min[loop] = dpm_stat->snkMaxMin[loop];
        memcpy((void *)&(gl_legacy_dpm_stat[port].cur_src_pdo[loop]),(void *)&(dpm_stat->curSrcPdo[loop]),sizeof(pd_do_t));
    }

    for(loop = 0u; loop < CY_PD_EXT_PPS_STATUS_SIZE; loop++)
    {
        gl_legacy_dpm_stat[port].pps_status[loop] = dpm_stat->ppsStatus[loop];
    }
    gl_legacy_dpm_stat[port].snk_pdo_count = dpm_stat->snkPdoCount;
    gl_legacy_dpm_stat[port].snk_pdo_mask = dpm_stat->snkPdoMask;
    gl_legacy_dpm_stat[port].snk_period = dpm_stat->snkPeriod;
    memcpy((void *)&(gl_legacy_dpm_stat[port].snk_rdo),(void *)&(dpm_stat->snkRdo),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].snk_rp_detach_en = dpm_stat->snkRpDetachEn;
    memcpy((void *)&(gl_legacy_dpm_stat[port].snk_sel_pdo),(void *)&(dpm_stat->snkSelPdo),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].snk_usb_comm_en = dpm_stat->snkUsbCommEn;
    gl_legacy_dpm_stat[port].snk_usb_susp_en = dpm_stat->snkUsbSuspEn;
    gl_legacy_dpm_stat[port].spec_rev_cbl = (pd_rev_t)dpm_stat->specRevCbl;
    gl_legacy_dpm_stat[port].spec_rev_peer = (pd_rev_t)dpm_stat->specRevPeer;
    gl_legacy_dpm_stat[port].spec_rev_sop_live = (pd_rev_t)dpm_cfg->specRevSopLive;
    gl_legacy_dpm_stat[port].spec_rev_sop_prime_live = (pd_rev_t)dpm_stat->specRevSopPrimeLive;
    gl_legacy_dpm_stat[port].src_cap_p = (pd_packet_t *)dpm_stat->srcCapP;
    gl_legacy_dpm_stat[port].src_cap_start_delay = dpm_stat->srcCapStartDelay;
    gl_legacy_dpm_stat[port].src_cur_level = dpm_stat->srcCurLevel;
    gl_legacy_dpm_stat[port].src_cur_level_live = dpm_cfg->srcCurLevelLive;
    memcpy((void *)&(gl_legacy_dpm_stat[port].src_cur_rdo),(void *)&(dpm_stat->srcCurRdo),sizeof(pd_do_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].src_last_rdo),(void *)&(dpm_stat->srcLastRdo),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].src_pdo_count = dpm_stat->srcPdoCount;
    gl_legacy_dpm_stat[port].src_pdo_mask = dpm_stat->srcPdoMask;
    gl_legacy_dpm_stat[port].src_period = dpm_stat->srcPeriod;
    memcpy((void *)&(gl_legacy_dpm_stat[port].src_rdo),(void *)&(dpm_stat->srcRdo),sizeof(pd_do_t));
    memcpy((void *)&(gl_legacy_dpm_stat[port].src_sel_pdo),(void *)&(dpm_stat->srcSelPdo),sizeof(pd_do_t));
    gl_legacy_dpm_stat[port].toggle = dpm_cfg->toggle;
    gl_legacy_dpm_stat[port].try_src_snk = dpm_stat->trySrcSnk;
    gl_legacy_dpm_stat[port].try_src_snk_dis = dpm_stat->trySrcSnkDis;
    gl_legacy_dpm_stat[port].typec_evt = dpm_cfg->typecEvt;
    gl_legacy_dpm_stat[port].typec_fsm_state = (typec_fsm_state_t)dpm_stat->typecFsmState;
    gl_legacy_dpm_stat[port].unchunk_sup_live = dpm_stat->unchunkSupLive;
    gl_legacy_dpm_stat[port].unchunk_sup_peer = dpm_stat->unchunkSupPeer;
    gl_legacy_dpm_stat[port].usb4_en = dpm_stat->usb4En;
    gl_legacy_dpm_stat[port].vconn_logical = dpm_cfg->vconnLogical;
    gl_legacy_dpm_stat[port].vconn_retain = dpm_stat->vconnRetain;

    return (const dpm_status_t*)&(gl_legacy_dpm_stat[port]);
}

bool vconn_enable_wrapper(uint8_t port, uint8_t channel)
{
    return vconn_enable(get_pdstack_context(port), channel);
}

void vconn_disable_wrapper(uint8_t port, uint8_t channel)
{
    vconn_disable(get_pdstack_context(port), channel);
}

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
operating_condition_t ccg_power_throttle_get_oc_wrapper(uint8_t port)
{
    return(ccg_power_throttle_get_oc(get_pdstack_context(port)));
}
uint8_t ccg_power_throttle_get_feature_mask_wrapper(uint8_t port)
{
    return(ccg_power_throttle_get_feature_mask(get_pdstack_context(port)));
}

void ccg_power_throttle_set_feature_mask_wrapper(uint8_t port, uint8_t mask)
{
    ccg_power_throttle_set_feature_mask(get_pdstack_context(port), mask);
}

cy_en_pdstack_status_t ccg_power_throttle_set_oc_ec_wrapper(uint8_t port,
                         operating_condition_t oc, srom_ccg_power_contract_complete cb)
{
    return(legacy_ccg_power_throttle_set_oc_ec(port, oc,cb));
}
operating_condition_t ccg_power_throttle_get_oc_ec_wrapper(uint8_t port)
{
    return(ccg_power_throttle_get_oc_ec(get_pdstack_context(port)));
}
uint8_t ccg_power_throttle_get_port_budget_wrapper(uint8_t port)
{
    return ccg_power_throttle_get_port_budget(get_pdstack_context(port));
}

/* Defining another function which calls the global callback defined above */
void ccg_power_throttle_set_pdp_callback(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power)
{
    callback_set_pdp(ptrPdStackContext->port,power);
}
cy_en_pdstack_status_t ccg_power_throttle_set_pdp_wrapper(uint8_t port,
                                    uint8_t power, srom_ccg_power_contract_complete cb)
{
    /* Registering the callback */
    callback_set_pdp = cb;
    /* Calling the function with new function as callback */
    return ccg_power_throttle_set_pdp(get_pdstack_context(port),power,ccg_power_throttle_set_pdp_callback);

}
void ccg_power_throttle_get_set_config_oc_callback(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power)
{
    callback_get_set_oc(ptrPdStackContext->port,power);
}
cy_en_pdstack_status_t ccg_power_throttle_get_set_config_oc_wrapper(uint8_t port, uint8_t *oc2, uint8_t *oc3, bool flag, srom_ccg_power_contract_complete cb)
{
    if(cb != NULL)
    {
        /* Registering the callback */
        callback_get_set_oc = cb;
        /* Calling the function with new function as callback */
        return ccg_power_throttle_get_set_config_oc(get_pdstack_context(port), oc2, oc3, flag, ccg_power_throttle_get_set_config_oc_callback);
    }
    else
        return ccg_power_throttle_get_set_config_oc(get_pdstack_context(port), oc2, oc3, flag, NULL);
}

uint8_t  ccg_get_sys_oc_wrapper(uint8_t port, hpi_oc_buffer_t *buffer)
{
    return ccg_get_sys_oc(get_pdstack_context(port), buffer);
}
cy_en_pdstack_status_t ccg_sensor_temp_ec_wrapper(uint8_t port, uint8_t *buffer)
{
    return ccg_sensor_temp_ec(get_pdstack_context(port), buffer);
}

#endif /* #if !(CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
bool vconn_is_present_wrapper(uint8_t port)
{
    return vconn_is_present(get_pdstack_context(port));
}
#if (CCG_LOAD_SHARING_ENABLE)
bool ccg_ls_is_heart_beat_running_wrapper(uint8_t port)
{
    return ccg_ls_is_heart_beat_running(get_pdstack_context(port));
}

bool ccg_ls_is_enabled_wrapper(uint8_t port)
{
    return(ccg_ls_is_enabled(get_pdstack_context(port)));
}
void ccg_ls_ctrl_callback(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power)
{
    callback_ls_ctrl(ptrPdStackContext->port,power);
}
cy_en_pdstack_status_t ccg_ls_ctrl_wrapper(uint8_t port,
                    bool disable_control, bool flag, srom_ccg_power_contract_complete cb)
{
    /* Registering the callback */
    callback_ls_ctrl = cb;
    return ccg_ls_ctrl(get_pdstack_context(port),disable_control,flag, ccg_ls_ctrl_callback);
}
#endif /* (CCG_LOAD_SHARING_ENABLE) */
cy_en_pdstack_status_t dpm_is_prev_contract_valid_wrapper(uint8_t port)
{
    cy_stc_pdstack_context_t *context=get_pdstack_context(port);
    return Cy_PdStack_Dpm_IsRdoValid(context, context->dpmStat.srcRdo);
}
cy_en_pdstack_status_t dpm_stop(uint8_t port)
{
    return Cy_PdStack_Dpm_Stop(get_pdstack_context(port));
}

cy_en_pdstack_status_t dpm_start(uint8_t port)
{
    return Cy_PdStack_Dpm_Start(get_pdstack_context(port));
}

void app_disable_pd_port_callback(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_dpm_typec_cmd_resp_t resp)
{
    app_disable_callback(ptrPdStackContext->port,(dpm_typec_cmd_resp_t)resp);
}
cy_en_pdstack_status_t app_disable_pd_port_wrapper(uint8_t port, dpm_typec_cmd_cbk_t cbk)
{
    /* Registering the callback */
    app_disable_callback = cbk;
    return app_disable_pd_port(get_pdstack_context(port), app_disable_pd_port_callback);
}

const bc_status_t* bc_get_status_wrapper(uint8_t port)
{
     return bc_get_status(get_pdstack_context(port));
}

auto_cfg_settings_t* pd_get_hpi_ptr_auto_cfg_tbl_wrapper(uint8_t port)
{
     return pd_get_ptr_auto_cfg_tbl(get_pdstack_context(port)->ptrUsbPdContext);
}

uint32_t app_retrieve_fault_status_wrapper(uint8_t port)
{
#if CCG_HPI_AUTO_CMD_ENABLE
    return app_retrieve_fault_status(get_pdstack_context(port));
#else
    (void)port;
    return 0u;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
}
uint16_t Cy_USBPD_Adc_MeasureVbus_wrapper(uint8_t port)
{
    return Cy_USBPD_Adc_MeasureVbus(get_pdstack_context(port)->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID,
                        APP_VBUS_POLL_ADC_INPUT);
}

uint16_t Cy_USBPD_Hal_MeasureCur_wrapper(uint8_t port)
{
    return Cy_USBPD_Hal_MeasureCur(get_pdstack_context(port)->ptrUsbPdContext);
}

uint16_t  ccg_get_battery_voltage_wrapper(uint8_t port)
{
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
    return ccg_get_battery_voltage(get_pdstack_context(port));
#else
    (void)port;
    return 0u;
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE || CCG_HPI_AUTO_CMD_ENABLE) */
}
#endif /* (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE)) */
