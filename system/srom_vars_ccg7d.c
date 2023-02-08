/******************************************************************************
* File Name: srom_vars_ccg7d.c
* \version 2.0
*
* Description: Source file for CCG7D SROM code
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "srom.h"
#include "srom_vars_ccg7d.h"
#include "cy_flash.h"
#include "flash.h"
#include "cy_pdstack_common.h"
#include "ccg7d_rom_pd.h"
#include "i2c.h"
#include "config.h"
#include "ccg7d_rom_wrapper.h"


#if (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE) && !defined(CY_DEVICE_SERIES_WLC1))
#if SROM_CODE_CRYPTO
/* QAC suppression 4604: External library functions required for SROM compilation declared again. */
extern void *calloc (size_t count, size_t eltsize); /* PRQA S 4604 */
extern void free (void *ptr); /* PRQA S 4604 */
#endif /* SROM_CODE_CRYPTO */

#if SROM_CODE_SCB_I2C
/* SCB register structures for direct access. */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
const PSCB_PRT_REGS_T SCB_PRT[I2C_BLOCK_COUNT] = /* PRQA S 6004 */
{
    SCB_PRT0,
    SCB_PRT1,
    SCB_PRT2,
    SCB_PRT3 
};
#endif /* SROM_CODE_SCB_I2C */

#if SROM_CODE_SYS_GPIO
/* GPIO block address array */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
const PGPIO_REGS_T GPIO_V[] = /* PRQA S 6004 */
{
    GPIO0,
    GPIO1,
    GPIO2,
    GPIO3,
};

/* HSIOM block address array */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
const PHSIOM_REGS_T HSIOM_V[] = /* PRQA S 6004 */
{
    HSIOM0,
    HSIOM1,
    HSIOM2,
    HSIOM3,
};
#endif  /* SROM_CODE_SYS_GPIO */

#if SROM_CODE_HPISS_HPI
/* QAC suppression 3449, 3451, 6004: External variable required by SROM code */
extern volatile uint32_t cyBtldrRunType; /* PRQA S 3449, 3451, 6004 */
extern void soln_boot_CySoftwareReset(void);
#if !defined(CY_PINS_EC_INT_H)
/* QAC suppression 3408: SROM generation needs definition to this external function to build. Hence,
 * dummy defintion provided. */
extern void EC_INT_Write(uint8_t value);
#endif /* CY_PINS_EC_INT_H */
#if (!CCG_HPI_AUTO_CMD_ENABLE)
void ccg_power_throttle_set_feature_mask_dummy(uint8_t port, uint8_t mask);
void ccg_power_throttle_set_feature_mask_dummy(uint8_t port, uint8_t mask)
{
    (void)port;
    (void)mask;
}
cy_en_pdstack_status_t ccg_power_throttle_set_oc_ec_dummy(uint8_t port, operating_condition_t oc, srom_ccg_power_contract_complete cb);
cy_en_pdstack_status_t ccg_power_throttle_set_oc_ec_dummy(uint8_t port, operating_condition_t oc, srom_ccg_power_contract_complete cb)
{
    (void)port;
    (void)oc;
    return CY_PDSTACK_STAT_BAD_PARAM;
}
cy_en_pdstack_status_t ccg_power_throttle_set_pdp_dummy(uint8_t port, uint8_t power, srom_ccg_power_contract_complete cb);
cy_en_pdstack_status_t ccg_power_throttle_set_pdp_dummy(uint8_t port, uint8_t power, srom_ccg_power_contract_complete cb)
{
    (void)port;
    (void)power;
    return CY_PDSTACK_STAT_INVALID_COMMAND;
}
cy_en_pdstack_status_t ccg_power_throttle_get_set_config_oc_dummy(uint8_t port, uint8_t *oc2, uint8_t *oc3, bool flag, srom_ccg_power_contract_complete cb);
cy_en_pdstack_status_t ccg_power_throttle_get_set_config_oc_dummy(uint8_t port, uint8_t *oc2, uint8_t *oc3, bool flag, srom_ccg_power_contract_complete cb)
{
    (void)port;
    (void)(*oc2);
    (void)(*oc3);
    (void)flag;
    return CY_PDSTACK_STAT_INVALID_COMMAND;
}
#endif /* !CCG_HPI_AUTO_CMD_ENABLE */
#if ((!CCG_BOOT) && (!(CCG_LOAD_SHARING_ENABLE & HPI_AUTO_CMD_ENABLE)))
    
/* QAC suppression 3408: Dummy unction maintained to avoid build failures when certain features are disabled */
bool ccg_ls_generic_dummy(uint8_t port) /* PRQA S 3408 */
{
    (void) port;
    return false;
}

/* QAC suppression 3408: Dummy unction maintained to avoid build failures when certain features are disabled */
cy_en_pdstack_status_t ccg_ls_ctrl_dummy(uint8_t port, bool disable_control, bool flag, srom_ccg_power_contract_complete cb) /* PRQA S 3408 */
{
    (void) port;
    (void) disable_control;
    (void) flag;
    (void) cb;
    return CY_PDSTACK_STAT_INVALID_COMMAND;
}
#endif /* ((!CCG_BOOT) && (!CCG_LOAD_SHARING_ENABLE)) */
#endif /* SROM_CODE_HPISS_HPI */

static const srom_out_cbk_t gl_srom_out_callback =
{
#if SROM_CODE_CRYPTO
    .calloc = calloc,
    .free = free,
#endif /* SROM_CODE_CRYPTO */

    /* System functions. */
    .CyDelayUs = Cy_SysLib_DelayUs,
    .CyIntSetVector = CyIntSetVector,
    .CyIntEnable = CyIntEnable,
    .CyIntDisable = CyIntDisable,
#if SROM_CODE_SCB_I2C
    /* Timer functions. */
    .timer_start = timer_start,
    .timer_stop = timer_stop,
    .timer_stop_range = timer_stop_range,
    .timer_is_running = timer_is_running,
#endif /* SROM_CODE_SCB_I2C */    

#if SROM_CODE_HPISS_HPI
    .CyEnterCriticalSection = CyEnterCriticalSection,
    .CyExitCriticalSection = CyExitCriticalSection,
#if !CCG_BOOT
    .dpm_get_info = dpm_get_info,
#if CCG_LOAD_SHARING_ENABLE
    .ccg_ls_is_enabled_wrapper = ccg_ls_is_enabled_wrapper,
#else
    .ccg_ls_is_enabled_wrapper = ccg_ls_generic_dummy,
#endif /* CCG_LOAD_SHARING_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
     .ccg_power_throttle_get_feature_mask_wrapper =  ccg_power_throttle_get_feature_mask_wrapper,
#else
     .ccg_power_throttle_get_feature_mask_wrapper = NULL,
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
#if CCG_HPI_AUTO_CMD_ENABLE
    .ccg_power_throttle_set_feature_mask_wrapper = ccg_power_throttle_set_feature_mask_wrapper,
    .ccg_power_throttle_set_oc_ec_wrapper = ccg_power_throttle_set_oc_ec_wrapper,
#else
    .ccg_power_throttle_set_feature_mask_wrapper = ccg_power_throttle_set_feature_mask_dummy,
    .ccg_power_throttle_set_oc_ec_wrapper = ccg_power_throttle_set_oc_ec_dummy,
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
    .vconn_enable_wrapper = vconn_enable_wrapper,
    .vconn_disable_wrapper = vconn_disable_wrapper,
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
    .ccg_power_throttle_get_oc_ec_wrapper = ccg_power_throttle_get_oc_ec_wrapper,
    .ccg_power_throttle_get_oc_wrapper = ccg_power_throttle_get_oc_wrapper,
#else
    .ccg_power_throttle_get_oc_ec_wrapper = NULL,
    .ccg_power_throttle_get_oc_wrapper = NULL,
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
    .vconn_is_present_wrapper = vconn_is_present_wrapper,
#if (CCG_LOAD_SHARING_ENABLE & CCG_HPI_AUTO_CMD_ENABLE & (!CCG_LS_INTER_INTRA_ENABLE))
    .ccg_ls_is_heart_beat_running_wrapper = ccg_ls_is_heart_beat_running_wrapper,
#else
    .ccg_ls_is_heart_beat_running_wrapper = ccg_ls_generic_dummy,
#endif /* CCG_LOAD_SHARING_ENABLE & CCG_HPI_AUTO_CMD_ENABLE */
#if CCG_HPI_AUTO_CMD_ENABLE
    .ccg_power_throttle_get_port_budget_wrapper = ccg_power_throttle_get_port_budget_wrapper,
#else
    .ccg_power_throttle_get_port_budget_wrapper = NULL,
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
    .dpm_is_prev_contract_valid_wrapper = dpm_is_prev_contract_valid_wrapper,
#if (CCG_LOAD_SHARING_ENABLE && (!CCG_LS_INTER_INTRA_ENABLE))
    .ccg_ls_ctrl_wrapper = ccg_ls_ctrl_wrapper,
#else
    .ccg_ls_ctrl_wrapper = ccg_ls_ctrl_dummy,
#endif /* CCG_LOAD_SHARING_ENABLE */
#if CCG_HPI_AUTO_CMD_ENABLE
    .ccg_power_throttle_set_pdp_wrapper = ccg_power_throttle_set_pdp_wrapper,
    .ccg_power_throttle_get_set_config_oc_wrapper = ccg_power_throttle_get_set_config_oc_wrapper,
#else
    .ccg_power_throttle_set_pdp_wrapper = ccg_power_throttle_set_pdp_dummy,
    .ccg_power_throttle_get_set_config_oc_wrapper = ccg_power_throttle_get_set_config_oc_dummy,
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
#else 
    .dpm_get_info = NULL,
    .ccg_ls_is_enabled_wrapper = NULL,
    .ccg_power_throttle_get_feature_mask_wrapper = NULL,
    .ccg_power_throttle_set_feature_mask_wrapper = NULL,
    .ccg_power_throttle_set_oc_ec_wrapper = NULL,
    .vconn_enable_wrapper = NULL,
    .vconn_disable_wrapper = NULL,
    .ccg_power_throttle_get_oc_ec_wrapper = NULL,
    .ccg_power_throttle_get_oc_wrapper = NULL,
    .vconn_is_present_wrapper = NULL,
    .ccg_ls_is_heart_beat_running_wrapper = NULL,
    .ccg_power_throttle_get_port_budget_wrapper = NULL,
    .dpm_is_prev_contract_valid_wrapper = NULL,
    .ccg_ls_ctrl_wrapper = NULL,
    .ccg_power_throttle_set_pdp_wrapper = NULL,
    .ccg_power_throttle_get_set_config_oc_wrapper = NULL,
#endif /* CCG_BOOT */
    .EC_INT_Write = EC_INT_Write,
#if !CCG_BOOT
    .CySoftwareReset = NVIC_SystemReset,
#else
    .CySoftwareReset = soln_boot_CySoftwareReset,
#endif /* !CCG_BOOT */
    .sys_get_device_mode = sys_get_device_mode,
    .sys_get_silicon_id = sys_get_silicon_id,
    .sys_get_custom_info_addr = sys_get_custom_info_addr,
#if !CCG_BOOT
    .dpm_stop = dpm_stop,
#else
    .dpm_stop = NULL,
#endif /* CCG_BOOT */
    .flash_enter_mode = flash_enter_mode,
    .flash_access_get_status = flash_access_get_status,
    .boot_handle_validate_fw_cmd = boot_handle_validate_fw_cmd,
    .flash_row_write = flash_row_write,
    .flash_row_read = flash_row_read,
#if !CCG_BOOT
    .dpm_start = dpm_start,
    .app_disable_pd_port_wrapper = app_disable_pd_port_wrapper,
#else
    .dpm_start = NULL,
    .app_disable_pd_port_wrapper= NULL,
#endif /* CCG_BOOT */
#if CCG_LIN_ENABLE
    .lins_hpi_write = lins_hpi_write,
    .lins_scb_hpi_init = lins_scb_hpi_init,
    .lins_scb_deinit = lins_scb_deinit,
    .lins_scb_enable_wakeup = lins_scb_enable_wakeup,
#else
    .lins_hpi_write = NULL,
    .lins_scb_hpi_init = NULL,
    .lins_scb_deinit = NULL,
    .lins_scb_enable_wakeup = NULL,
#endif /* CCG_LIN_ENABLE */
    .boot_validate_fw = boot_validate_fw,
#endif /* SROM_CODE_HPISS_HPI */ 
};

/* 
 * SROM address space.
 * Use initial 12 bytes for SROM version, call_in table information,
 * followed by call_in table.
 * Actual ROM code shall start after this SROM information.
 */
#define SROM_BASE_VERSION_SIZE          (4u)
#define SROM_APP_VERSION_SIZE           (4u)
#define SROM_TABLE_SIZE                 (2u)
#define SROM_RESERVED                   (2u)

#define SROM_BASE_ADDR                  (0x10002000u)
#define SROM_BASE_VERSION_ADDR          (SROM_BASE_ADDR)
#define SROM_APP_VERSION_ADDR           ((SROM_BASE_VERSION_ADDR) + (SROM_BASE_VERSION_SIZE))
#define SROM_TABLE_SIZE_ADDR            ((SROM_APP_VERSION_ADDR) + (SROM_APP_VERSION_SIZE))
#define SROM_TABLE_ADDR                 ((SROM_TABLE_SIZE_ADDR) + (SROM_TABLE_SIZE) + (SROM_RESERVED))

#if GENERATE_SROM_CODE
__attribute__ ((section(".srom_table"), used))
const srom_in_cbk_t srom_in_callback =
{
    /* Crypto functions */
#if SROM_CODE_CRYPTO
    .mbedtls_sha256 = mbedtls_sha256,
    .mbedtls_sha256_init = mbedtls_sha256_init,
    .mbedtls_sha256_free = mbedtls_sha256_free,
    .mbedtls_sha256_starts = mbedtls_sha256_starts,
    .mbedtls_sha256_update = mbedtls_sha256_update,
    .mbedtls_sha256_finish = mbedtls_sha256_finish,
    .uECC_verify = uECC_verify,
    .uECC_secp256r1 = uECC_secp256r1,
    .uECC_sign = uECC_sign,
    .uECC_set_rng = uECC_set_rng,
    .uECC_valid_public_key = uECC_valid_public_key,
    .verify_signature = verify_signature,
    .crypto_init_globals = crypto_init_globals,
#endif /* SROM_CODE_CRYPTO */

    /* I2C Functions */
#if SROM_CODE_SCB_I2C
    .i2c_slave_ack_ctrl = i2c_slave_ack_ctrl,
    .i2c_reset = i2c_reset,
    .i2c_scb_is_idle = i2c_scb_is_idle,
    .i2c_scb_enable_wakeup = i2c_scb_enable_wakeup,
    .i2c_timer_cb = i2c_timer_cb,
    .i2c_scb_0_intr_handler = i2c_scb_0_intr_handler,
#if (I2C_BLOCK_COUNT > 1)
    .i2c_scb_1_intr_handler = i2c_scb_1_intr_handler,
#endif /* (I2C_BLOCK_COUNT > 1) */
#if (I2C_BLOCK_COUNT > 2)
    .i2c_scb_2_intr_handler = i2c_scb_2_intr_handler,
#endif /* (I2C_BLOCK_COUNT > 2) */
#if (I2C_BLOCK_COUNT > 3)
    .i2c_scb_3_intr_handler = i2c_scb_3_intr_handler,
#endif /* (I2C_BLOCK_COUNT > 3) */
    .i2c_scb_init = i2c_scb_init,
    .i2c_scb_deinit = i2c_scb_deinit,
    .i2c_scb_write = i2c_scb_write,
#endif /* SROM_CODE_SCB_I2C */

    /* GPIO Functions. */
#if SROM_CODE_SYS_GPIO
    .gpio_set_value = gpio_set_value,
    .gpio_read_value = gpio_read_value,
    .gpio_set_drv_mode = gpio_set_drv_mode,
    .gpio_int_set_config = gpio_int_set_config,
    .gpio_get_intr = gpio_get_intr,
    .gpio_clear_intr = gpio_clear_intr,
    .hsiom_set_config = hsiom_set_config,
    .gpio_hsiom_set_config = gpio_hsiom_set_config,
    .gpio_set_lvttl_mode = gpio_set_lvttl_mode,
#endif /* SROM_CODE_SYS_GPIO */

    /* HPI Functions */
#if SROM_CODE_HPISS_HPI
    .hpi_is_ec_ready = hpi_is_ec_ready,
    .hpi_config_run_time_params = hpi_config_run_time_params,
    .hpi_auto_copy_data_to_flash = hpi_auto_copy_data_to_flash,
    .hpi_pd_event_handler = hpi_pd_event_handler,
    .hpi_set_boot_priority_conf = hpi_set_boot_priority_conf,
    .hpi_set_hpi_version = hpi_set_hpi_version,
    .hpi_set_mode_regs = hpi_set_mode_regs,
    .hpi_update_versions = hpi_update_versions,
    .hpi_update_fw_locations = hpi_update_fw_locations,
    .hpi_register_i2c_fsm = hpi_register_i2c_fsm,
    .hpi_reg_enqueue_event = hpi_reg_enqueue_event,
    .hpi_bb_get_version = hpi_bb_get_version,
    .hpi_bb_reg_update = hpi_bb_reg_update,
    .hpi_bb_get_reg = hpi_bb_get_reg,
    .hpi_auto_set_soln_cb_handler = hpi_auto_set_soln_cb_handler,
    .hpi_set_black_box_handler = hpi_set_black_box_handler,
    .hpi_set_userdef_write_handler = hpi_set_userdef_write_handler,
    .hpi_init_userdef_regs = hpi_init_userdef_regs,
    .hpi_task = hpi_task,
    .hpi_i2c_cmd_callback = hpi_i2c_cmd_callback,
    .hpi_lin_cmd_callback = hpi_lin_cmd_callback,
    .hpi_set_no_boot_mode = hpi_set_no_boot_mode,
    .hpi_get_port_enable = hpi_get_port_enable,
    .hpi_set_fixed_slave_address = hpi_set_fixed_slave_address,
    .hpi_set_ec_interrupt = hpi_set_ec_interrupt,
    .hpi_send_fw_ready_event = hpi_send_fw_ready_event,
    .hpi_set_mode = hpi_set_mode,
    .hpi_get_mode = hpi_get_mode,
    .hpi_set_flash_params = hpi_set_flash_params,
    .hpi_init = hpi_init,
    .hpi_sleep_allowed = hpi_sleep_allowed,
    .hpi_sleep = hpi_sleep,
    .hpi_init_globals = hpi_init_globals,
    .hpi_register_command_overload = hpi_register_command_overload,
#endif /* SROM_CODE_HPISS_HPI */

};

/* Place the ROM version, call_in table and it's size at start of ROM. */
__attribute__ ((section(".srom_base_version"), used))
const uint32_t srom_base_version = FW_BASE_VERSION;
__attribute__ ((section(".srom_app_version"), used))
const uint32_t srom_app_version  = APP_VERSION;
__attribute__ ((section(".srom_table_size"), used))
const uint16_t srom_table_size = sizeof(srom_in_callback);
__attribute__ ((section(".srom_reserved"), used))
const uint16_t srom_reserved = 0;

#endif /* GENERATE_SROM_CODE */

static srom_vars_t gl_srom_vars;

#if defined(__ARMCC_VERSION)
        __attribute__ ((section(".srom_out_callback"), zero_init)) __attribute__((used))
#elif defined (__GNUC__)
        __attribute__ ((section(".srom_out_callback"))) __attribute__((used))
#elif defined (__ICCARM__)
        #pragma location=".srom_out_callback"
#endif /* (__ARMCC_VERSION) */
srom_out_cbk_t *gl_p_srom_out_cbk;

#if defined(__ARMCC_VERSION)
        __attribute__ ((section(".srom_in_callback"), zero_init)) __attribute__((used))
#elif defined (__GNUC__)
        __attribute__ ((section(".srom_in_callback"))) __attribute__((used))
#elif defined (__ICCARM__)
        #pragma location=".srom_in_callback"
#endif /* (__ARMCC_VERSION) */
srom_in_cbk_t *gl_p_srom_in_cbk;

#if defined(__ARMCC_VERSION)
        __attribute__ ((section(".srom_status_var"), zero_init)) __attribute__((used))
#elif defined (__GNUC__)
        __attribute__ ((section(".srom_status_var"))) __attribute__((used))
#elif defined (__ICCARM__)
        #pragma location=".srom_status_var"
#endif /* (__ARMCC_VERSION) */
srom_vars_t *gl_p_srom_vars;

void srom_vars_init_ccg7d (void)
{
    CY_PDUTILS_MEM_SET ((uint8_t *)&gl_srom_vars, 0, sizeof(srom_vars_t));
    srom_vars_t *vars;
    /* Setup the pointers */
    /* QAC suppression 0311: Assigned pointer is only used for read only purpose */
    gl_p_srom_out_cbk = (srom_out_cbk_t *)&gl_srom_out_callback; /* PRQA S 0311 */
    gl_p_srom_in_cbk = (srom_in_cbk_t *)SROM_TABLE_ADDR;
    gl_p_srom_vars = (srom_vars_t *)&gl_srom_vars;
    vars = &gl_srom_vars;

    /* Crypto Initialization */
#if SROM_CODE_CRYPTO
    vars->g_rng_function = NULL;
    /* Crypto Global Variables Initialization. */
    CALL_MAP(crypto_init_globals)();
#endif /* SROM_CODE_CRYPTO */

    /* I2C Pointer Setup */
#if SROM_CODE_SCB_I2C
    /* QAC suppression 0311: Assigned pointer is only used for read only purpose */
    vars->SCB_PRT = (PSCB_PRT_REGS_T *)SCB_PRT; /* PRQA S 0311 */
#endif /* SROM_CODE_SCB_I2C */

    /* GPIO pointer setup. */
#if SROM_CODE_SYS_GPIO
    /* QAC suppression 0311: Assigned pointer is only used for read only purpose */
    vars->GPIO_V = (PGPIO_REGS_T *)GPIO_V; /* PRQA S 0311 */
    /* QAC suppression 0311: Assigned pointer is only used for read only purpose */
    vars->HSIOM_V = (PHSIOM_REGS_T *)HSIOM_V; /* PRQA S 0311 */
#endif /* SROM_CODE_SYS_GPIO */

#if (SROM_CODE_HPISS_HPI && !CCG_FIRMWARE_APP_ONLY)
    vars->gl_p_cyBtldrRunType = (volatile uint32_t *)&cyBtldrRunType;
#endif /* SROM_CODE_HPISS_HPI */
#if SROM_CODE_HPISS_HPI
    /* HPI Global Variables Initialization. */
    CALL_MAP(hpi_init_globals)();
#endif /* SROM_CODE_HPISS_HPI */
    (void)vars;
}
#endif /* (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE)) */

