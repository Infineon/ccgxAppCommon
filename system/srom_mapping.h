/******************************************************************************
* File Name: srom_mapping.h
* \version 2.0
*
* Description: SROM Code default mapping header file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef SROM_MAPPING_H_
#define SROM_MAPPING_H_

/************************************************************************
* DEFAULT CALL_MAP MAPPING
************************************************************************/
#ifndef call_gpio_hsiom_set_config
#define call_gpio_hsiom_set_config(func)          (func)
#endif /* call_gpio_hsiom_set_config(func) */

#ifndef call_flash_set_access_limits
#define call_flash_set_access_limits(func)        (func)
#endif /* call_flash_set_access_limits(func) */

#ifndef call_flash_access_get_status
#define call_flash_access_get_status(func)        (func)
#endif /* call_flash_set_access_limits(func) */

#ifndef call_flash_row_clear
#define call_flash_row_clear(func)                (func)
#endif /* call_flash_row_clear(func) */

#ifndef call_div_round_up
#define call_div_round_up(func)                   (func)
#endif /* call_div_round_up(func) */

#ifndef call_apply_threshold
#define call_apply_threshold(func)   (func)
#endif /* call_apply_threshold */
#ifndef call_gl_op_cur_power
#define call_gl_op_cur_power(func)                (func)
#endif /* call_gl_op_cur_power(func) */

#ifndef call_gl_contract_voltage
#define call_gl_contract_voltage(func)            (func)
#endif /* call_gl_contract_voltage(func) */

#ifndef call_gl_contract_power
#define call_gl_contract_power(func)              (func)
#endif /* call_gl_contract_power(func) */

#ifndef call_gl_max_min_cur_pwr
#define call_gl_max_min_cur_pwr(func)             (func)
#endif /* call_gl_max_min_cur_pwr(func) */

#ifndef call_app_get_resp_buf
#define call_app_get_resp_buf(func)               (func)
#endif /* call_app_get_resp_buf(func) */

#ifndef call_Cy_PdStack_Dpm_IsRdoValid
#define call_Cy_PdStack_Dpm_IsRdoValid(func)      (func)
#endif /* call_Cy_PdStack_Dpm_IsRdoValid(func) */

#ifndef call_hsiom_set_config
#define call_hsiom_set_config(func)               (func)
#endif /* call_hsiom_set_config(func) */

#ifndef call_gpio_set_value
#define call_gpio_set_value(func)                 (func)
#endif /* call_gpio_set_value(func) */

#ifndef call_i2c_slave_ack_ctrl
#define call_i2c_slave_ack_ctrl(func)             (func)
#endif /* call_i2c_slave_ack_ctrl(func) */

#ifndef call_i2c_scb_write
#define call_i2c_scb_write(func)                  (func)
#endif /* call_i2c_scb_write(func) */

#ifndef call_i2c_scb_init
#define call_i2c_scb_init(func)                   (func)
#endif /* call_i2c_scb_init(func) */

#ifndef call_i2c_scb_deinit
#define call_i2c_scb_deinit(func)                 (func)
#endif /* call_i2c_scb_deinit(func) */

#ifndef call_i2c_scb_is_idle
#define call_i2c_scb_is_idle(func)                (func)
#endif /* call_i2c_scb_is_idle(func) */

#ifndef call_i2c_scb_enable_wakeup
#define call_i2c_scb_enable_wakeup(func)          (func)
#endif /* call_i2c_scb_enable_wakeup(func) */

#ifndef call_sys_get_silicon_id
#define call_sys_get_silicon_id(func)             (func)
#endif /* call_sys_get_silicon_id(func) */

#ifndef call_sys_get_device_mode
#define call_sys_get_device_mode(func)            (func)
#endif /* call_sys_get_device_mode(func) */

#ifndef call_i2c_reset
#define call_i2c_reset(func)                      (func)
#endif /* call_i2c_reset(func) */

#ifndef call_sys_get_custom_info_addr
#define call_sys_get_custom_info_addr(func)       (func)
#endif /* call_sys_get_custom_info_addr(func) */

#ifndef call_gpio_set_lvttl_mode
#define call_gpio_set_lvttl_mode(func)            (func)
#endif /* call_gpio_set_lvttl_mode(func) */

#ifndef call_SCB_PRT
#define call_SCB_PRT(func)                        (func)
#endif /* call_SCB_PRT(func) */

#ifndef call_CyIntEnable
#define call_CyIntEnable(func)                    (func)
#endif /* call_CyIntEnable(func) */

#ifndef call_CyIntDisable
#define call_CyIntDisable(func)                   (func)
#endif /* call_CyIntDisable(func) */

#ifndef call_i2c_ack_disable
#define call_i2c_ack_disable(func)                (func)
#endif /* call_i2c_ack_disable(func) */

#ifndef call_i2c_ack_pending
#define call_i2c_ack_pending(func)                (func)
#endif /* call_i2c_ack_pending(func) */

#ifndef call_gl_i2c_scb_config
#define call_gl_i2c_scb_config(func)              (func)
#endif /* call_gl_i2c_scb_config(func) */

#ifndef call_timer_stop
#define call_timer_stop(func)                     (func)
#endif /* call_timer_stop(func) */

#ifndef call_CyDelayUs
#define call_CyDelayUs(func)                      (func)
#endif /* call_CyDelayUs(func) */

#ifndef call_timer_start
#define call_timer_start(func)                    (func)
#endif /* call_timer_start(func) */

#ifndef call_i2c_timer_cb
#define call_i2c_timer_cb(func)                   (func)
#endif /* call_i2c_timer_cb(func) */

#ifndef call_CyIntSetVector
#define call_CyIntSetVector(func)                 (func)
#endif /* call_CyIntSetVector(func) */

#ifndef call_pd_prot_frs_rx_disable
#define call_pd_prot_frs_rx_disable(func)         (func)
#endif /* call_pd_prot_frs_rx_disable(func) */

#ifndef call_pd_prot_frs_tx_disable
#define call_pd_prot_frs_tx_disable(func)         (func)
#endif /* call_pd_prot_frs_tx_disable(func) */

#ifndef call_pd_prot_reset
#define call_pd_prot_reset(func)                  (func)
#endif /* call_pd_prot_reset(func) */

#ifndef call_pd_prot_reset_rx
#define call_pd_prot_reset_rx(func)               (func)
#endif /* call_pd_prot_reset_rx(func) */

#ifndef call_gl_pd_status
#define call_gl_pd_status(func)                   (func)
#endif /* call_gl_pd_status(func) */

#ifndef call_pd_prot_init
#define call_pd_prot_init(func)                   (func)
#endif /* call_pd_prot_init(func) */

#ifndef call_pd_prot_start
#define call_pd_prot_start(func)                  (func)
#endif /* call_pd_prot_start(func) */

#ifndef call_pd_prot_is_busy
#define call_pd_prot_is_busy(func)                (func)
#endif /* call_pd_prot_is_busy(func) */

#ifndef call_pd_prot_refresh_roles
#define call_pd_prot_refresh_roles(func)          (func)
#endif /* call_pd_prot_refresh_roles(func) */

#ifndef call_pd_prot_get_rx_packet
#define call_pd_prot_get_rx_packet(func)          (func)
#endif /* call_pd_prot_get_rx_packet(func) */

#ifndef call_pd_prot_dis_bist_test_data
#define call_pd_prot_dis_bist_test_data(func)     (func)
#endif /* call_pd_prot_dis_bist_test_data(func) */

#ifndef call_pd_prot_send_hard_reset
#define call_pd_prot_send_hard_reset(func)        (func)
#endif /* call_pd_prot_send_hard_reset(func) */

#ifndef call_pd_prot_en_bist_test_data
#define call_pd_prot_en_bist_test_data(func)      (func)
#endif /* call_pd_prot_en_bist_test_data(func) */

#ifndef call_pd_prot_en_bist_cm2
#define call_pd_prot_en_bist_cm2(func)            (func)
#endif /* call_pd_prot_en_bist_cm2(func) */

#ifndef call_pd_prot_dis_bist_cm2
#define call_pd_prot_dis_bist_cm2(func)           (func)
#endif /* call_pd_prot_dis_bist_cm2(func) */

#ifndef call_pd_prot_reset_all
#define call_pd_prot_reset_all(func)              (func)
#endif /* call_pd_prot_reset_all(func) */

#ifndef call_eval_src_cap
#define call_eval_src_cap(func)                   (func)
#endif /* call_eval_src_cap(func) */

#ifndef call_pd_prot_send_data_msg
#define call_pd_prot_send_data_msg(func)          (func)
#endif /* call_pd_prot_send_data_msg(func) */

#ifndef call_pd_prot_send_ctrl_msg
#define call_pd_prot_send_ctrl_msg(func)          (func)
#endif /* call_pd_prot_send_ctrl_msg(func) */

#ifndef call_eval_rdo
#define call_eval_rdo(func)                       (func)
#endif /* call_eval_rdo(func) */

#ifndef call_pd_prot_frs_tx_enable
#define call_pd_prot_frs_tx_enable(func)          (func)
#endif /* call_pd_prot_frs_tx_enable(func) */

#ifndef call_pd_prot_send_extd__msg
#define call_pd_prot_send_extd__msg(func)         (func)
#endif /* call_pd_prot_send_extd__msg(func) */

#ifndef call_pd_prot_send_cable_reset
#define call_pd_prot_send_cable_reset(func)       (func)
#endif /* call_pd_prot_send_cable_reset(func) */

#ifndef call_pd_prot_frs_rx_enable
#define call_pd_prot_frs_rx_enable(func)          (func)
#endif /* call_pd_prot_frs_rx_enable(func) */

#ifndef call_pd_phy_init
#define call_pd_phy_init(func)                    (func)
#endif /* call_pd_phy_init(func) */

#ifndef call_pd_phy_refresh_roles
#define call_pd_phy_refresh_roles(func)           (func)
#endif /* call_pd_phy_refresh_roles(func) */

#ifndef call_pd_phy_is_busy
#define call_pd_phy_is_busy(func)                 (func)
#endif /* call_pd_phy_is_busy(func) */

#ifndef call_timer_stop_range
#define call_timer_stop_range(func)               (func)
#endif /* call_timer_stop_range(func) */

#ifndef call_pd_phy_abort_tx_msg
#define call_pd_phy_abort_tx_msg(func)            (func)
#endif /* call_pd_phy_abort_tx_msg(func) */

#ifndef call_dpm_get_status
#define call_dpm_get_status(func)                 (func)
#endif /* call_dpm_get_status(func) */

#ifndef call_CY_PDUTILS_MEM_SET
#define call_CY_PDUTILS_MEM_SET(func)                        (func)
#endif /* call_mem_set(func) */

#ifndef call_pd_phy_load_msg
#define call_pd_phy_load_msg(func)                (func)
#endif /* call_pd_phy_load_msg(func) */

#ifndef call_CyEnterCriticalSection
#define call_CyEnterCriticalSection(func)         (func)
#endif /* call_CyEnterCriticalSection(func) */

#ifndef call_prl_tmr_cbk
#define call_prl_tmr_cbk(func)                    (func)
#endif /* call_prl_tmr_cbk(func) */

#ifndef call_pd_phy_send_msg
#define call_pd_phy_send_msg(func)                (func)
#endif /* call_pd_phy_send_msg(func) */

#ifndef call_CyExitCriticalSection
#define call_CyExitCriticalSection(func)          (func)
#endif /* call_CyExitCriticalSection(func) */

#ifndef call_pd_phy_send_reset
#define call_pd_phy_send_reset(func)              (func)
#endif /* call_pd_phy_send_reset(func) */

#ifndef call_pd_phy_send_bist_cm2
#define call_pd_phy_send_bist_cm2(func)           (func)
#endif /* call_pd_phy_send_bist_cm2(func) */

#ifndef call_pd_phy_abort_bist_cm2
#define call_pd_phy_abort_bist_cm2(func)          (func)
#endif /* call_pd_phy_abort_bist_cm2(func) */

#ifndef call_pd_phy_dis_unchunked_tx
#define call_pd_phy_dis_unchunked_tx(func)        (func)
#endif /* call_pd_phy_dis_unchunked_tx(func) */

#ifndef call_timer_is_running
#define call_timer_is_running(func)               (func)
#endif /* call_timer_is_running(func) */

#ifndef call_phy_cbk
#define call_phy_cbk(func)                       (func)
#endif /* call_phy_cbk(func) */

#ifndef call_pd_phy_get_rx_packet
#define call_pd_phy_get_rx_packet(func)           (func)
#endif /* call_pd_phy_get_rx_packet(func) */

#ifndef call_CY_PDUTILS_MEM_COPY_word
#define call_CY_PDUTILS_MEM_COPY_word(func)                  (func)
#endif /* call_mem_copy_word(func) */

#ifndef call_pd_frs_rx_disable
#define call_pd_frs_rx_disable(func)              (func)
#endif /* call_pd_frs_rx_disable(func) */

#ifndef call_pd_frs_tx_disable
#define call_pd_frs_tx_disable(func)              (func)
#endif /* call_pd_frs_tx_disable(func) */

#ifndef call_pd_is_msg
#define call_pd_is_msg(func)                      (func)
#endif /* call_pd_is_msg(func) */

#ifndef call_pd_frs_rx_enable
#define call_pd_frs_rx_enable(func)               (func)
#endif /* call_pd_frs_rx_enable(func) */

#ifndef call_pd_frs_tx_enable
#define call_pd_frs_tx_enable(func)               (func)
#endif /* call_pd_frs_tx_enable(func) */

#ifndef call_gpio_clear_intr
#define call_gpio_clear_intr(func)                (func)
#endif /* call_gpio_clear_intr(func) */

#ifndef call_gpio_int_set_config
#define call_gpio_int_set_config(func)            (func)
#endif /* call_gpio_int_set_config(func) */

#ifndef call_hsiom_set_config
#define call_hsiom_set_config(func)               (func)
#endif /* call_hsiom_set_config(func) */

#ifndef call_gpio_read_value
#define call_gpio_read_value(func)                (func)
#endif /* call_gpio_read_value(func) */

#ifndef call_gpio_set_drv_mode
#define call_gpio_set_drv_mode(func)              (func)
#endif /* call_gpio_set_drv_mode(func) */

#ifndef call_gpio_hsiom_set_config
#define call_gpio_hsiom_set_config(func)          (func)
#endif /* call_gpio_hsiom_set_config(func) */

#ifndef call_gl_img1_fw_metadata
#define call_gl_img1_fw_metadata(func)            (func)
#endif /* call_gl_img1_fw_metadata(func) */

#ifndef call_gl_img2_fw_metadata
#define call_gl_img2_fw_metadata(func)            (func)
#endif /* call_gl_img2_fw_metadata(func) */

#ifndef call_sys_get_recent_fw_image
#define call_sys_get_recent_fw_image(func)        (func)
#endif /* call_sys_get_recent_fw_image(func) */

#ifndef call_gl_flash_mode_en
#define call_gl_flash_mode_en(func)               (func)
#endif /* call_gl_flash_mode_en(func) */

#ifndef call_gl_flash_write_in_place
#define call_gl_flash_write_in_place(func)        (func)
#endif /* call_gl_flash_write_in_place(func) */

#ifndef call_gl_flash_access_first
#define call_gl_flash_access_first(func)          (func)
#endif /* call_gl_flash_access_first(func) */

#ifndef call_gl_flash_access_last
#define call_gl_flash_access_last(func)           (func)
#endif /* call_gl_flash_access_last(func) */

#ifndef call_gl_flash_metadata_row
#define call_gl_flash_metadata_row(func)          (func)
#endif /* call_gl_flash_metadata_row(func) */

#ifndef call_gl_flash_bl_last_row
#define call_gl_flash_bl_last_row(func)           (func)
#endif /* call_gl_flash_bl_last_row(func) */

#ifndef call_mbedtls_sha256_init
#define call_mbedtls_sha256_init(func)            (func)
#endif /* call_mbedtls_sha256_init(func) */

#ifndef call_mbedtls_sha256_starts
#define call_mbedtls_sha256_starts(func)          (func)
#endif /* call_mbedtls_sha256_starts(func) */

#ifndef call_mbedtls_sha256_update
#define call_mbedtls_sha256_update(func)          (func)
#endif /* call_mbedtls_sha256_update(func) */

#ifndef call_mbedtls_sha256_finish
#define call_mbedtls_sha256_finish(func)          (func)
#endif /* call_mbedtls_sha256_finish(func) */

#ifndef call_mbedtls_sha256_free
#define call_mbedtls_sha256_free(func)            (func)
#endif /* call_mbedtls_sha256_free(func) */

#ifndef call_mbedtls_sha256
#define call_mbedtls_sha256(func)                 (func)
#endif /* call_mbedtls_sha256(func) */

#ifndef call_verify_signature
#define call_verify_signature(func)               (func)
#endif /* call_verify_signature(func) */

#ifndef call_flash_enter_mode
#define call_flash_enter_mode(func)               (func)
#endif /* call_flash_enter_mode(func) */

#ifndef call_flash_row_write
#define call_flash_row_write(func)                (func)
#endif /* call_flash_row_write(func) */

#ifndef call_GPIO
#define call_GPIO(func)                           (func)
#endif /* call_GPIO(func) */

#ifndef call_HSIOM
#define call_HSIOM(func)                          (func)
#endif /* call_HSIOM(func) */

#ifndef call_GPIO_V
#define call_GPIO_V(func)                           (func)
#endif /* call_GPIO(func) */

#ifndef call_HSIOM_V
#define call_HSIOM_V(func)                          (func)
#endif /* call_HSIOM(func) */

#ifndef call_str
#define call_str(func)                            (func)
#endif /* call_str(func) */

#ifndef call_gl_active_fw
#define call_gl_active_fw(func)                   (func)
#endif /* call_gl_active_fw(func) */

#ifndef call_get_boot_mode_reason
#define call_get_boot_mode_reason(func)           (func)
#endif /* call_get_boot_mode_reason(func) */

#ifndef call_gl_invalid_version
#define call_gl_invalid_version(func)             (func)
#endif /* call_gl_invalid_version(func) */

#ifndef call_sys_get_fw_img1_start_addr
#define call_sys_get_fw_img1_start_addr(func)     (func)
#endif /* call_sys_get_fw_img1_start_addr(func) */

#ifndef call_sys_get_fw_img2_start_addr
#define call_sys_get_fw_img2_start_addr(func)     (func)
#endif /* call_sys_get_fw_img2_start_addr(func) */

/* HPI Auto Boot */
#ifndef call_CyEnterCriticalSection
#define call_CyEnterCriticalSection(func)                (func)
#endif /* call_CyEnterCriticalSection(func) */
#ifndef call_CyExitCriticalSection
#define call_CyExitCriticalSection(func)                (func)
#endif /* call_CyExitCriticalSection(func) */
#ifndef call_dpm_get_info
#define call_dpm_get_info(func)                            (func)
#endif /* call_dpm_get_info */
#ifndef call_ccg_ls_is_enabled
#define call_ccg_ls_is_enabled(func)                    (func)
#endif /* call_ccg_ls_is_enabled */
#ifndef call_ccg_ls_heart_beat_ctrl
#define call_ccg_ls_heart_beat_ctrl(func)                (func)
#endif /* call_ccg_ls_heart_beat_ctrl */
#ifndef call_ccg_power_throttle_get_feature_mask
#define call_ccg_power_throttle_get_feature_mask(func)    (func)
#endif /* call_ccg_power_throttle_get_feature_mask */
#ifndef call_ccg_power_throttle_set_feature_mask
#define call_ccg_power_throttle_set_feature_mask(func)    (func)
#endif /* call_ccg_power_throttle_set_feature_mask */
#ifndef call_ccg_power_throttle_set_oc_ec
#define call_ccg_power_throttle_set_oc_ec(func)            (func)
#endif /* call_ccg_power_throttle_set_oc_ec */
#ifndef call_ccg_power_throttle_get_oc_ec
#define call_ccg_power_throttle_get_oc_ec(func)         (func)
#endif /* call_ccg_power_throttle_get_oc_ec */
#ifndef call_ccg_power_throttle_get_oc
#define call_ccg_power_throttle_get_oc(func)            (func)
#endif /* call_ccg_power_throttle_get_oc */
#ifndef call_vconn_enable
#define call_vconn_enable(func)                            (func)
#endif /* call_vconn_enable */
#ifndef call_vconn_disable
#define call_vconn_disable(func)                        (func)
#endif /* call_vconn_disable */
#ifndef call_ccg_power_throttle_get_oc
#define call_ccg_power_throttle_get_oc(func)            (func)
#endif /* call_ccg_power_throttle_get_oc */
#ifndef call_vconn_is_present
#define call_vconn_is_present(func)                        (func)
#endif /* call_vconn_is_present */
#ifndef call_ccg_ls_is_heart_beat_running
#define call_ccg_ls_is_heart_beat_running(func)            (func)
#endif /* call_ccg_ls_is_heart_beat_running */
#ifndef call_ccg_power_throttle_get_port_budget
#define call_ccg_power_throttle_get_port_budget(func)    (func)
#endif /* call_ccg_power_throttle_get_port_budget */
#ifndef call_dpm_is_prev_contract_valid
#define call_dpm_is_prev_contract_valid(func)            (func)
#endif /* call_dpm_is_prev_contract_valid */
#ifndef call_ccg_ls_ctrl
#define call_ccg_ls_ctrl(func)                            (func)
#endif /* call_ccg_ls_ctrl */
#ifndef call_ccg_power_throttle_set_pdp
#define call_ccg_power_throttle_set_pdp(func)            (func)
#endif /* call_ccg_power_throttle_set_pdp */
#ifndef call_ccg_power_throttle_set_pdp
#define call_ccg_power_throttle_set_pdp(func)            (func)
#endif /* call_ccg_power_throttle_set_pdp */
#ifndef call_ccg_power_throttle_get_set_config_oc
#define call_ccg_power_throttle_get_set_config_oc(func) (func)
#endif /* ccg_power_throttle_get_set_config_oc */
#ifndef call_EC_INT_Write
#define call_EC_INT_Write(func)                            (func)
#endif /* call_EC_INT_Write */
#ifndef call_CySoftwareReset
#define call_CySoftwareReset(func)                        (func)
#endif /* call_CySoftwareReset */
#ifndef call_sys_set_device_mode
#define call_sys_set_device_mode(func)                    (func)
#endif /* call_sys_set_device_mode */
#ifndef call_sys_get_device_mode
#define call_sys_get_device_mode(func)                    (func)
#endif /* call_sys_get_device_mode */
#ifndef call_sys_get_boot_version
#define call_sys_get_boot_version(func)                    (func)
#endif /* call_sys_get_boot_version */
#ifndef call_sys_get_img1_fw_version
#define call_sys_get_img1_fw_version(func)                (func)
#endif /* call_sys_get_img1_fw_version */
#ifndef call_sys_get_img2_fw_version
#define call_sys_get_img2_fw_version(func)                (func)
#endif /* call_sys_get_img2_fw_version */
#ifndef call_sys_get_fw_img1_start_addr
#define call_sys_get_fw_img1_start_addr(func)            (func)
#endif /* call_sys_get_fw_img1_start_addr */
#ifndef call_sys_get_fw_img2_start_addr
#define call_sys_get_fw_img2_start_addr(func)            (func)
#endif /* call_sys_get_fw_img2_start_addr */
#ifndef call_sys_get_recent_fw_image
#define call_sys_get_recent_fw_image(func)                (func)
#endif /* call_sys_get_recent_fw_image */
#ifndef call_sys_get_silicon_id
#define call_sys_get_silicon_id(func)                    (func)
#endif /* call_sys_get_silicon_id */
#ifndef call_get_silicon_revision
#define call_get_silicon_revision(func)                    (func)
#endif /* call_get_silicon_revision */
#ifndef call_sys_get_custom_info_addr
#define call_sys_get_custom_info_addr(func)                (func)
#endif /* call_sys_get_custom_info_addr */
#ifndef call_sys_get_bcdDevice_version
#define call_sys_get_bcdDevice_version(func)            (func)
#endif /* call_sys_get_bcdDevice_version */
#ifndef call_Cy_PdStack_Dpm_Stop
#define call_Cy_PdStack_Dpm_Stop(func)                    (func)
#endif /* call_dpm_stop */
#ifndef call_flash_enter_mode
#define call_flash_enter_mode(func)                        (func)
#endif /* call_flash_enter_mode */
#ifndef call_boot_handle_validate_fw_cmd
#define call_boot_handle_validate_fw_cmd(func)            (func)
#endif /* call_boot_handle_validate_fw_cmd */
#ifndef call_flash_row_write
#define call_flash_row_write(func)                        (func)
#endif /* call_flash_row_write */
#ifndef call_flash_row_read
#define call_flash_row_read(func)                        (func)
#endif /* call_flash_row_read */
#ifndef call_Cy_PdStack_Dpm_Start
#define call_Cy_PdStack_Dpm_Start(func)                    (func)
#endif /* call_dpm_start */
#ifndef call_app_disable_pd_port
#define call_app_disable_pd_port(func)                    (func)
#endif /* call_app_disable_pd_port */
#ifndef call_flash_set_app_priority
#define call_flash_set_app_priority(func)                (func)
#endif /* call_flash_set_app_priority */
#ifndef call_lins_hpi_write
#define call_lins_hpi_write(func)                        (func)
#endif /* call_lins_hpi_write */
#ifndef call_lins_scb_hpi_init
#define call_lins_scb_hpi_init(func)                    (func)
#endif /* call_lins_scb_hpi_init */
#ifndef call_lins_scb_deinit
#define call_lins_scb_deinit(func)                        (func)
#endif /* call_lins_scb_deinit */
#ifndef call_lins_scb_enable_wakeup
#define call_lins_scb_enable_wakeup(func)                (func)
#endif /* call_lins_scb_enable_wakeup */
#ifndef call_memcpy
#define call_memcpy(func)                               (func)
#endif /* call_memcpy */
#ifndef call_memset
#define call_memset(func)                               (func)
#endif /* call_memset */
#ifndef call_hpi_is_ec_ready
#define call_hpi_is_ec_ready(func)                        (func)
#endif /* call_hpi_is_ec_ready */
#ifndef call_hpi_pd_port_disable_cb
#define call_hpi_pd_port_disable_cb(func)                (func)
#endif /* call_hpi_pd_port_disable_cb */
#ifndef call_hpi_config_run_time_params
#define call_hpi_config_run_time_params(func)            (func)
#endif /* call_hpi_config_run_time_params */
#ifndef call_hpi_auto_copy_data_to_flash
#define call_hpi_auto_copy_data_to_flash(func)            (func)
#endif /* call_hpi_auto_copy_data_to_flash */
#ifndef call_ccg_hpi_set_contract_change_cb
#define call_ccg_hpi_set_contract_change_cb(func)       (func)
#endif /* ccg_hpi_set_contract_change_cb */
#ifndef call_hpi_auto_cmd_handle
#define call_hpi_auto_cmd_handle(func)                    (func)
#endif /* call_hpi_auto_cmd_handle */
#ifndef call_hpi_reg_handle_queue
#define call_hpi_reg_handle_queue(func)                    (func)
#endif /* call_hpi_reg_handle_queue */
#ifndef call_hpi_pd_event_handler
#define call_hpi_pd_event_handler(func)                    (func)
#endif /* call_hpi_pd_event_handler */
#ifndef call_hpi_set_boot_priority_conf
#define call_hpi_set_boot_priority_conf(func)            (func)
#endif /* call_hpi_set_boot_priority_conf */
#ifndef call_hpi_set_hpi_version
#define call_hpi_set_hpi_version(func)                    (func)
#endif /* call_hpi_set_hpi_version */
#ifndef call_hpi_reset_dev_regs
#define call_hpi_reset_dev_regs(func)                    (func)
#endif /* call_hpi_reset_dev_regs */
#ifndef call_hpi_set_mode_regs
#define call_hpi_set_mode_regs(func)                    (func)
#endif /* call_hpi_set_mode_regs */
#ifndef call_hpi_update_versions
#define call_hpi_update_versions(func)                    (func)
#endif /* call_hpi_update_versions */
#ifndef call_hpi_update_fw_locations
#define call_hpi_update_fw_locations(func)                (func)
#endif /* call_hpi_update_fw_locations */
#ifndef call_hpi_register_i2c_fsm
#define call_hpi_register_i2c_fsm(func)                    (func)
#endif /* call_hpi_register_i2c_fsm */
#ifndef call_hpi_register_command_overload
#define call_hpi_register_command_overload(func)        (func)
#endif /* call_hpi_register_command_overload */
#ifndef call_hpi_reg_enqueue_event
#define call_hpi_reg_enqueue_event(func)                (func)
#endif /* call_hpi_reg_enqueue_event */
#ifndef call_hpi_reg_handle_intr_write
#define call_hpi_reg_handle_intr_write(func)            (func)
#endif /* call_hpi_reg_handle_intr_write */
#ifndef call_hpi_get_port_enable
#define call_hpi_get_port_enable(func)                    (func)
#endif /* call_hpi_get_port_enable */
#ifndef call_hpi_get_sys_pwr_state
#define call_hpi_get_sys_pwr_state(func)                (func)
#endif /* call_hpi_get_sys_pwr_state */
#ifndef call_hpi_reset_dev
#define call_hpi_reset_dev(func)                        (func)
#endif /* call_hpi_reset_dev */
#ifndef call_Delay_Reset_cb
#define call_Delay_Reset_cb(func)                        (func)
#endif /* call_Delay_Reset_cb */
#ifndef call_get_hpi_soft_reset_delay
#define call_get_hpi_soft_reset_delay(func)                (func)
#endif /* call_get_hpi_soft_reset_delay */
#ifndef call_update_hpi_soft_reset_delay
#define call_update_hpi_soft_reset_delay(func)            (func)
#endif /* call_update_hpi_soft_reset_delay */
#ifndef call_get_hpi_sof_reset_timer_id
#define call_get_hpi_sof_reset_timer_id(func)            (func)
#endif /* call_get_hpi_sof_reset_timer_id */
#ifndef call_update_hpi_sof_reset_timer_id
#define call_update_hpi_sof_reset_timer_id(func)        (func)
#endif /* call_update_hpi_sof_reset_timer_id */
#ifndef call_hpi_reg_handle_cmd_dev
#define call_hpi_reg_handle_cmd_dev(func)                (func)
#endif /* call_hpi_reg_handle_cmd_dev */
#ifndef call_hpi_handle_reg_write
#define call_hpi_handle_reg_write(func)                    (func)
#endif /* call_hpi_handle_reg_write */
#ifndef call_hpi_bb_get_version
#define call_hpi_bb_get_version(func)                    (func)
#endif /* call_hpi_bb_get_version */
#ifndef call_hpi_bb_reg_update
#define call_hpi_bb_reg_update(func)                    (func)
#endif /* call_hpi_bb_reg_update */
#ifndef call_hpi_bb_get_reg
#define call_hpi_bb_get_reg(func)                        (func)
#endif /* call_hpi_bb_get_reg */
#ifndef call_hpi_auto_set_soln_cb_handler
#define call_hpi_auto_set_soln_cb_handler(func)            (func)
#endif /* call_hpi_auto_set_soln_cb_handler */
#ifndef call_hpi_set_black_box_handler
#define call_hpi_set_black_box_handler(func)            (func)
#endif /* call_hpi_set_black_box_handler */
#ifndef call_hpi_set_userdef_write_handler
#define call_hpi_set_userdef_write_handler(func)        (func)
#endif /* call_hpi_set_userdef_write_handler */
#ifndef call_hpi_init_userdef_regs
#define call_hpi_init_userdef_regs(func)                (func)
#endif /* call_hpi_init_userdef_regs */
#ifndef call_hpi_task
#define call_hpi_task(func)                                (func)
#endif /* call_hpi_task */
#ifndef call_hpi_i2c_cmd_callback
#define call_hpi_i2c_cmd_callback(func)                    (func)
#endif /* call_hpi_i2c_cmd_callback */
#ifndef call_hpi_lin_cmd_callback
#define call_hpi_lin_cmd_callback(func)                    (func)
#endif /* call_hpi_lin_cmd_callback */
#ifndef call_hpi_set_no_boot_mode
#define call_hpi_set_no_boot_mode(func)                    (func)
#endif /* call_hpi_set_no_boot_mode */
#ifndef call_hpi_get_port_enable
#define call_hpi_get_port_enable(func)                    (func)
#endif /* call_hpi_get_port_enable */
#ifndef call_hpi_set_fixed_slave_address
#define call_hpi_set_fixed_slave_address(func)            (func)
#endif /* call_hpi_set_fixed_slave_address */
#ifndef call_hpi_set_ec_interrupt
#define call_hpi_set_ec_interrupt(func)                    (func)
#endif /* call_hpi_set_ec_interrupt */
#ifndef call_hpi_send_fw_ready_event
#define call_hpi_send_fw_ready_event(func)                (func)
#endif /* call_hpi_send_fw_ready_event */
#ifndef call_hpi_set_mode
#define call_hpi_set_mode(func)                            (func)
#endif /* call_hpi_set_mode */
#ifndef call_hpi_get_mode
#define call_hpi_get_mode(func)                            (func)
#endif /* call_hpi_get_mode */
#ifndef call_hpi_set_flash_params
#define call_hpi_set_flash_params(func)                    (func)
#endif /* call_hpi_set_flash_params */
#ifndef call_hpi_init
#define call_hpi_init(func)                                (func)
#endif /* call_hpi_init */
#ifndef call_hpi_sleep_allowed
#define call_hpi_sleep_allowed(func)                    (func)
#endif /* call_hpi_sleep_allowed */
#ifndef call_hpi_sleep
#define call_hpi_sleep(func)                            (func)
#endif /* call_hpi_sleep */
#ifndef call_hpi_init_globals
#define call_hpi_init_globals(func)                        (func)
#endif /* call_hpi_init_globals */
#ifndef call_boot_validate_fw
#define call_boot_validate_fw(func)                        (func)
#endif /* call_boot_validate_fw */
#ifndef call_glp_cyBtldrRunType
#define call_glp_cyBtldrRunType(func)                   (func)
#endif /* call_glp_cyBtldrRunType */

#ifndef call_Cy_PdStack_Dpm_GetPdPortStatus
#define call_Cy_PdStack_Dpm_GetPdPortStatus(func)       (func)
#endif /* call_dpm_get_pd_port_status(func) */

#ifndef call_dpm_get_vbus_voltage
#define call_dpm_get_vbus_voltage(func)           (func)
#endif /* call_dpm_get_vbus_voltage(func) */

#ifndef call_alt_mode_get_status
#define call_alt_mode_get_status(func)            (func)
#endif /* call_alt_mode_get_status(func) */

#ifndef call_tcc_get_raw_data
#define call_tcc_get_raw_data(func)               (func)
#endif /* call_tcc_get_raw_data(func) */

#ifndef call_tcc_init
#define call_tcc_init(func)                       (func)
#endif /* call_tcc_init(func) */

#ifndef call_tcc_get_percent
#define call_tcc_get_percent(func)                (func)
#endif /* call_tcc_get_percent(func) */

#ifndef call_tcc_get_raw_data_size
#define call_tcc_get_raw_data_size(func)          (func)
#endif /* call_tcc_get_raw_data_size(func) */

#ifndef call_pd_frs_rx_enable
#define call_pd_frs_rx_enable(func)               (func)
#endif /* call_pd_frs_rx_enable(func) */

#ifndef call_pd_frs_tx_enable
#define call_pd_frs_tx_enable(func)               (func)
#endif /* call_pd_frs_tx_enable(func) */

#ifndef call_get_pd_config
#define call_get_pd_config(func)                  (func)
#endif /* call_get_pd_config(func) */

#ifndef call_ccg_sync_update_remote_port
#define call_ccg_sync_update_remote_port(func)    (func)
#endif /* call_ccg_sync_update_remote_port(func) */

#ifndef call_Cy_PdStack_Dpm_UpdatePortStatus
#define call_Cy_PdStack_Dpm_UpdatePortStatus(func)         (func)
#endif /* call_dpm_update_port_status(func) */

#ifndef call_app_update_sys_pwr_state
#define call_app_update_sys_pwr_state(func)       (func)
#endif /* call_app_update_sys_pwr_state(func) */

#ifndef call_ucsi_init
#define call_ucsi_init(func)                      (func)
#endif /* call_ucsi_init(func) */

#ifndef call_ucsi_set_status_bit
#define call_ucsi_set_status_bit(func)            (func)
#endif /* call_ucsi_set_status_bit(func) */

#ifndef call_ucsi_clear_status_bit
#define call_ucsi_clear_status_bit(func)          (func)
#endif /* call_ucsi_clear_status_bit(func) */

#ifndef call_ucsi_notify
#define call_ucsi_notify(func)                    (func)
#endif /* call_ucsi_notify(func) */

#ifndef call_Cy_Pdstack_Dpm_SendPdCommandEc
#define call_Cy_Pdstack_Dpm_SendPdCommandEc(func)              (func)
#endif /* call_dpm_pd_command_ec(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateSrcCap 
#define call_dpm_Cy_PdStack_Dpm_UpdateSrcCap (func)             (func)
#endif /* call_dpm_update_src_cap(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateSnkCap
#define call_Cy_PdStack_Dpm_UpdateSnkCap(func)             (func)
#endif /* call_dpm_update_snk_cap(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateSrcCapMask
#define call_Cy_PdStack_Dpm_UpdateSrcCapMask(func)        (func)
#endif /* call_dpm_update_src_cap_mask(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateSnkCapMask
#define call_Cy_PdStack_Dpm_UpdateSnkCapMask(func)        (func)
#endif /* call_dpm_update_snk_cap_mask(func) */

#ifndef call_Cy_PdStack_Dpm_SendTypecCommand
#define call_Cy_PdStack_Dpm_SendTypecCommand(func)              (func)
#endif /* call_dpm_typec_command(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateSnkMaxMin 
#define call_Cy_PdStack_Dpm_UpdateSnkMaxMin (func)         (func)
#endif /* call_dpm_update_snk_max_min(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateExtSrcCap
#define call_Cy_PdStack_Dpm_UpdateExtSrcCap(func)         (func)
#endif /* call_dpm_update_ext_src_cap(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateExtSnkCap 
#define call_Cy_PdStack_Dpm_UpdateExtSnkCap (func)         (func)
#endif /* call_dpm_update_ext_snk_cap(func) */

#ifndef call_vdm_get_disc_id_resp
#define call_vdm_get_disc_id_resp(func)           (func)
#endif /* call_vdm_get_disc_id_resp(func) */

#ifndef call_vdm_get_disc_svid_resp
#define call_vdm_get_disc_svid_resp(func)         (func)
#endif /* call_vdm_get_disc_svid_resp(func) */

#ifndef call_Cy_PdStack_Dpm_UpdatePortConfig 
#define call_Cy_PdStack_Dpm_UpdatePortConfig (func)         (func)
#endif /* call_dpm_update_port_config(func) */

#ifndef call_pd_hal_switch_vsys_to_vbus
#define call_pd_hal_switch_vsys_to_vbus(func)     (func)
#endif /* call_pd_hal_switch_vsys_to_vbus(func) */

#ifndef call_app_vdm_layer_reset
#define call_app_vdm_layer_reset(func)            (func)
#endif /* call_app_vdm_layer_reset(func) */

#ifndef call_eval_app_alt_mode_cmd
#define call_eval_app_alt_mode_cmd(func)          (func)
#endif /* call_eval_app_alt_mode_cmd(func) */

#ifndef call_eval_app_alt_hw_cmd
#define call_eval_app_alt_hw_cmd(func)            (func)
#endif /* call_eval_app_alt_hw_cmd(func) */

#ifndef call_dpm_update_swap_response
#define call_dpm_update_swap_response(func)       (func)
#endif /* call_dpm_update_swap_response(func) */

#ifndef call_Cy_PdStack_Dpm_UpdateFrsEnable
#define call_Cy_PdStack_Dpm_UpdateFrsEnable(func) (func)
#endif /* call_dpm_update_frs_enable(func) */

#ifndef call_app_update_bc_src_support
#define call_app_update_bc_src_support(func)      (func)
#endif /* call_app_update_bc_src_support(func) */

#ifndef call_set_alt_mode_mask
#define call_set_alt_mode_mask(func)              (func)
#endif /* call_set_alt_mode_mask(func) */

#ifndef call_set_custom_svid
#define call_set_custom_svid(func)                (func)
#endif /* call_set_custom_svid(func) */

#ifndef call_bc_12_get_status
#define call_bc_12_get_status(func)               (func)
#endif /* call_bc_12_get_status(func) */

#ifndef call_pd_hal_switch_vddd_supply
#define call_pd_hal_switch_vddd_supply(func)      (func)
#endif /* call_pd_hal_switch_vddd_supply(func) */

#ifndef call_app_update_bc_src_snk_support
#define call_app_update_bc_src_snk_support(func)  (func)
#endif /* call_app_update_bc_src_snk_support(func) */

#ifndef call_bc_tmr_cbk
#define call_bc_tmr_cbk(func)                     (func)
#endif 

#ifndef call_cy_sw_timer_init
#define call_cy_sw_timer_init(func)              (func)
#endif /* call_cy_sw_timer_init */
#ifndef call_Cy_PdUtils_SwTimer_Start
#define call_Cy_PdUtils_SwTimer_Start(func)              (func)
#endif /* call_Cy_PdUtils_SwTimer_Start */

#ifndef call_Cy_PdUtils_SwTimer_Stop
#define call_Cy_PdUtils_SwTimer_Stop(func)              (func)
#endif /* call_Cy_PdUtils_SwTimer_Stop */

#ifndef call_Cy_PdUtils_SwTimer_Stop_all
#define call_Cy_PdUtils_SwTimer_Stop_all(func)              (func)
#endif /* call_Cy_PdUtils_SwTimer_Stop_all */
#ifndef call_Cy_PdUtils_SwTimer_StopRange
#define call_Cy_PdUtils_SwTimer_StopRange(func)        (func)
#endif /* call_Cy_PdUtils_SwTimer_StopRange */

#ifndef call_Cy_PdUtils_SwTimer_IsRunning
#define call_Cy_PdUtils_SwTimer_IsRunning(func)        (func)
#endif /* call_Cy_PdUtils_SwTimer_IsRunning */

#ifndef call_calloc
#define call_calloc(func)                        (func)
#endif /* call_calloc */

#ifndef call_free
#define call_free(func)                          (func)
#endif /* call_free */

#ifndef call_Cy_SysLib_DelayUs
#define call_Cy_SysLib_DelayUs(func)             (func)
#endif /* call_Cy_SysLib_DelayUs */

#ifndef call___NVIC_SetVector
#define call___NVIC_SetVector(func)              (func)
#endif /* call___NVIC_SetVector */

#ifndef call___NVIC_EnableIRQ
#define call___NVIC_EnableIRQ(func)              (func)
#endif /* call___NVIC_EnableIRQ */

#ifndef call___NVIC_DisableIRQ
#define call___NVIC_DisableIRQ(func)             (func)
#endif /* call___NVIC_DisableIRQ */

#ifndef call_Cy_PdStack_Dpm_IsPrevContractValid
#define call_Cy_PdStack_Dpm_IsPrevContractValid(func)   (func)
#endif /* call_Cy_PdStack_Dpm_IsPrevContractValid */

#ifndef call___NVIC_SystemReset
#define call___NVIC_SystemReset(func)           (func)
#endif /* call___NVIC_SystemReset */

#ifndef call_i2c_scb_0_intr_handler
#define call_i2c_scb_0_intr_handler(func)        (func)
#endif /* call_i2c_scb_0_intr_handler */

#ifndef call_i2c_scb_1_intr_handler
#define call_i2c_scb_1_intr_handler(func)        (func)
#endif /* call_i2c_scb_1_intr_handler */

#ifndef call_i2c_scb_2_intr_handler
#define call_i2c_scb_2_intr_handler(func)        (func)
#endif /* call_i2c_scb_2_intr_handler */

#ifndef call_i2c_scb_3_intr_handler
#define call_i2c_scb_3_intr_handler(func)        (func)
#endif /* call_i2c_scb_3_intr_handler */

#ifndef call_Cy_SysInt_SetVector
#define call_Cy_SysInt_SetVector(func)             (func)
#endif /* call_Cy_SysInt_SetVector */

#ifndef call_Get_PdStack_Context
#define call_Get_PdStack_Context(func)             (func)
#endif /* call_Get_PdStack_Context */

#ifndef call_Cy_SysLib_ExitCriticalSection
#define call_Cy_SysLib_ExitCriticalSection(func)   (func)
#endif /* call_Cy_SysLib_ExitCriticalSection */

#ifndef call_Cy_SysLib_EnterCriticalSection
#define call_Cy_SysLib_EnterCriticalSection(func)   (func)
#endif /* call_Cy_SysLib_EnterCriticalSection */

#ifndef ATTRIBUTES_HPISS_HPI
#define ATTRIBUTES_HPISS_HPI
#endif /* ATTRIBUTES_HPISS_HPI */
#ifndef HPI_GLOBAL_VAR
#define HPI_GLOBAL_VAR
#endif /* HPI_GLOBAL_VAR */
#ifndef HPI_CONST
#define HPI_CONST
#endif /* HPI_CONST */

#endif /* SROM_MAPPING_H_ */
