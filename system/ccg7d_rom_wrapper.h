/******************************************************************************
* File Name: ccg7d_rom_wrapper.h
* \version 2.0
*
* Description: Header file for CCG7D ROM Wrapper Functions
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/


#ifndef CCG7D_ROM_WRAPPER_H_
#define CCG7D_ROM_WRAPPER_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_syslib.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_defines.h"
#if (CCG_LOAD_SHARING_ENABLE)
#if CCG_LS_INTER_INTRA_ENABLE
#include <loadsharing_inter_intra.h>
#else
#include <loadsharing.h>
#endif /* CCG_LS_INTER_INTRA_ENABLE */
#endif /* (CCG_LOAD_SHARING_ENABLE) */
#include "ccg7d_rom_pd.h"
#include "config.h"
#include "battery_charging.h"
#if !CCG_HPI_ENABLE
#include "srom_dependency.h"
#endif /* !CCG_HPI_ENABLE */
#if CCG_HPI_ENABLE
#include "hpi.h"
#endif /* CCG_HPI_ENABLE */
#if (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE))
#if (!defined(CY_DEVICE_SERIES_WLC1))
#include <power_throttle.h>
#else
#include "srom_dependency.h"
#endif /* CY_DEVICE_SERIES_WLC1 */


cy_israddress CyIntSetVector(uint8_t number, cy_israddress address);

void CyIntEnable(uint8_t number);

void CyIntDisable (uint8_t number);

bool timer_start(uint8_t instance, cy_timer_id_t id, uint16_t period, cy_cb_timer_t cb);

void timer_stop (uint8_t instance, cy_timer_id_t id);

void timer_stop_range(uint8_t instance, cy_timer_id_t start, cy_timer_id_t stop);

bool timer_is_running(uint8_t instance, cy_timer_id_t id);

uint8_t CyEnterCriticalSection(void);

void  CyExitCriticalSection(uint8_t savedIntrStatus);

const dpm_status_t* dpm_get_info (uint8_t port);

bool vconn_enable_wrapper(uint8_t port, uint8_t channel);

void vconn_disable_wrapper(uint8_t port, uint8_t channel);

operating_condition_t ccg_power_throttle_get_oc_wrapper(uint8_t port);

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)

cy_en_pdstack_status_t ccg_sensor_temp_ec_wrapper(uint8_t port, uint8_t *buffer);

uint8_t  ccg_get_sys_oc_wrapper(uint8_t port, hpi_oc_buffer_t *buffer);

operating_condition_t ccg_power_throttle_get_oc_wrapper(uint8_t port);

uint8_t ccg_power_throttle_get_feature_mask_wrapper(uint8_t port);

void ccg_power_throttle_set_feature_mask_wrapper(uint8_t port, uint8_t mask);

cy_en_pdstack_status_t ccg_power_throttle_set_oc_ec_wrapper(uint8_t port,
                         operating_condition_t oc, srom_ccg_power_contract_complete cb);
                         
operating_condition_t ccg_power_throttle_get_oc_ec_wrapper(uint8_t port);

uint8_t ccg_power_throttle_get_port_budget_wrapper(uint8_t port);

cy_en_pdstack_status_t ccg_power_throttle_set_pdp_wrapper(uint8_t port,
                                    uint8_t power, srom_ccg_power_contract_complete cb);
                                 
cy_en_pdstack_status_t ccg_power_throttle_get_set_config_oc_wrapper(uint8_t port, uint8_t *oc2, 
                                        uint8_t *oc3, bool flag, srom_ccg_power_contract_complete cb);
#endif /* !(CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
bool vconn_is_present_wrapper(uint8_t port);
#if (CCG_LOAD_SHARING_ENABLE)
bool ccg_ls_is_heart_beat_running_wrapper(uint8_t port);

cy_en_pdstack_status_t ccg_ls_ctrl_wrapper(uint8_t port,
                    bool disable_control, bool flag, srom_ccg_power_contract_complete cb);

bool ccg_ls_is_enabled_wrapper(uint8_t port);
#endif /* (CCG_LOAD_SHARING_ENABLE) */

cy_en_pdstack_status_t dpm_is_prev_contract_valid_wrapper(uint8_t port);

cy_en_pdstack_status_t dpm_stop(uint8_t port);

cy_en_pdstack_status_t dpm_start(uint8_t port);

cy_en_pdstack_status_t app_disable_pd_port_wrapper(uint8_t port, dpm_typec_cmd_cbk_t cbk);

const bc_status_t* bc_get_status_wrapper(uint8_t port);

auto_cfg_settings_t* pd_get_hpi_ptr_auto_cfg_tbl_wrapper(uint8_t port);

uint32_t app_retrieve_fault_status_wrapper(uint8_t port);

uint16_t Cy_USBPD_Adc_MeasureVbus_wrapper(uint8_t port);

uint16_t Cy_USBPD_Hal_MeasureCur_wrapper(uint8_t port);

uint16_t  ccg_get_battery_voltage_wrapper(uint8_t port);
/** @endcond */
#endif /* (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE)) */
#endif /* CCG7D_ROM_WRAPPER_H_ */
