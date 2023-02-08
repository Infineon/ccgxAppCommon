/******************************************************************************
* File Name: srom_vars_ccg7d.h
* \version 2.0
*
* Description: Header file for CCG7D SROM code
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef SROM_VARS_CCG7D_H_
#define SROM_VARS_CCG7D_H_


#include "srom.h"
#include "ccg7d_rom_wrapper.h"
#include "srom_config.h"
#include <config.h>
#include <stdio.h>
#include <string.h>    
#include "i2c.h"
#if CCG_HPI_ENABLE || CCG_HPI_OVER_LIN_ENABLE
#include "hpi.h"
#if (!CCG_HPI_OVER_LIN_ENABLE || HPI_AUTO_SROM_ENABLE)
#include "app.h"
#endif /* (!CCG_HPI_OVER_LIN_ENABLE || HPI_AUTO_SROM_ENABLE) */
#endif /* CCG_HPI_ENABLE || CCG_HPI_OVER_LIN_ENABLE */
#if ((!defined(CY_DEVICE_SERIES_WLC1)) && (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_ENABLE || (defined(CY_DEVICE_CCG7D) && CCG_BOOT)))
#include "power_throttle.h"
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_ENABLE || CCG_BOOT) */
#if (CCG_LOAD_SHARING_ENABLE  || (defined(CY_DEVICE_CCG7D) && CCG_BOOT))
#if (!defined(CY_DEVICE_SERIES_WLC1))
#if CCG_LS_INTER_INTRA_ENABLE
#include <loadsharing_inter_intra.h>
#else
#include <loadsharing.h>
#endif
#endif /* CCG_LS_INTER_INTRA_ENABLE */
#endif /* (CCG_LOAD_SHARING_ENABLE  || CCG_BOOT) */
#include "cy_pdstack_dpm.h"
#include "flash.h"
#if CCG_LIN_ENABLE
#include "lins.h"
#endif /* CCG_LIN_ENABLE */
#include "gpio.h"
#include "cy_pdstack_common.h"
#include "cy_pdutils.h"
#include "system.h"
#include <cy_pdutils_sw_timer.h>
#include "boot.h"
#include "mbedtls/sha256.h"
#include "mbedtls/rsa.h"
#include "uECC.h"
#include "srom_dependency.h"

/************************************************************************
* CALL_IN, CALL_OUT  and GET_VAR structures declaration.
************************************************************************/
#if (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE) && !defined(CY_DEVICE_SERIES_WLC1))
/** @cond DOXYGEN_HIDE */
typedef struct
{
    /* uECC variables */
    uECC_RNG_Function g_rng_function;

    /* SCB */
    PSCB_PRT_REGS_T *SCB_PRT;
    i2c_scb_config_t gl_i2c_scb_config[I2C_BLOCK_COUNT];
    volatile bool i2c_ack_disable[I2C_BLOCK_COUNT];
    volatile bool i2c_ack_pending[I2C_BLOCK_COUNT];

    /* GPIO */
    PGPIO_REGS_T *GPIO_V;
    PHSIOM_REGS_T *HSIOM_V;

    /* HPI */
    volatile uint32_t *gl_p_cyBtldrRunType;
} srom_vars_t;

typedef struct
{
   /* Memory Allocation APIs */
    void *(*calloc) (size_t count, size_t eltsize);
    void (*free) (void *ptr);

    /* System Functions. */
    void(*CyDelayUs)(uint16_t microseconds);
    cy_israddress(*CyIntSetVector)(uint8_t number, cy_israddress address);
    void(*CyIntEnable)(uint8_t number);
    void(*CyIntDisable)(uint8_t number);

    /* Timer Functions */
    bool(*timer_start)(uint8_t instance, cy_timer_id_t id, uint16_t period, cy_cb_timer_t cb);
    void(*timer_stop) (uint8_t instance, cy_timer_id_t id);
    void (*timer_stop_range)(uint8_t instance, cy_timer_id_t start, cy_timer_id_t stop);
    bool (*timer_is_running) (uint8_t instance, cy_timer_id_t id);

    /* HPI Functions */
    uint8_t (*CyEnterCriticalSection)(void);
    void  (*CyExitCriticalSection)(uint8_t savedIntrStatus);
    const dpm_status_t* (*dpm_get_info)(uint8_t port);
    bool (*ccg_ls_is_enabled_wrapper)(uint8_t port);
    uint8_t (*ccg_power_throttle_get_feature_mask_wrapper)(uint8_t port);
    void (*ccg_power_throttle_set_feature_mask_wrapper)(uint8_t port, uint8_t mask);
    cy_en_pdstack_status_t (*ccg_power_throttle_set_oc_ec_wrapper)(uint8_t port,
                            operating_condition_t oc, srom_ccg_power_contract_complete cb);
    bool (*vconn_enable_wrapper)(uint8_t port, uint8_t channel);
    void (*vconn_disable_wrapper)(uint8_t port, uint8_t channel);
    operating_condition_t (*ccg_power_throttle_get_oc_wrapper)(uint8_t port);
    operating_condition_t (*ccg_power_throttle_get_oc_ec_wrapper)(uint8_t port);
    bool (*vconn_is_present_wrapper)(uint8_t port);
    bool (*ccg_ls_is_heart_beat_running_wrapper)(uint8_t port);
    uint8_t (*ccg_power_throttle_get_port_budget_wrapper)(uint8_t port);
    cy_en_pdstack_status_t (*dpm_is_prev_contract_valid_wrapper)(uint8_t port);
    cy_en_pdstack_status_t (*ccg_ls_ctrl_wrapper)(uint8_t port,
                    bool disable_control, bool flag, srom_ccg_power_contract_complete cb);
    cy_en_pdstack_status_t (*ccg_power_throttle_set_pdp_wrapper)(uint8_t port,
                                    uint8_t power, srom_ccg_power_contract_complete cb);
    void (*EC_INT_Write)(uint8_t value);
    void (*CySoftwareReset)(void);
    sys_fw_mode_t (*sys_get_device_mode)(void);
    void (*sys_get_silicon_id)(uint32_t *silicon_id);
    uint32_t (*sys_get_custom_info_addr)(void);
    cy_en_pdstack_status_t (*dpm_stop)(uint8_t port);
    void (*flash_enter_mode)(bool is_enable, flash_interface_t mode, bool data_in_place);
    bool (*flash_access_get_status)(uint8_t modes);
    cy_en_pdstack_status_t (*boot_handle_validate_fw_cmd)(sys_fw_mode_t fw_mode);
    cy_en_pdstack_status_t (*flash_row_write)(uint16_t row_num, uint8_t *data, flash_cbk_t cbk);
    cy_en_pdstack_status_t (*flash_row_read)(uint16_t row_num, uint8_t* data);
    cy_en_pdstack_status_t (*dpm_start)(uint8_t port);
    cy_en_pdstack_status_t (*app_disable_pd_port_wrapper)(uint8_t port, dpm_typec_cmd_cbk_t cbk);
    void (*lins_hpi_write)(uint8_t  scb_index, 
                    uint8_t *source_ptr, uint8_t  size, uint8_t *count);
    void (*lins_scb_hpi_init)(uint8_t scb_index,
    i2c_cb_fun_t cb_fun_ptr,
    uint8_t *scratch_buffer, uint16_t scratch_buffer_size);
    void (*lins_scb_deinit)(uint8_t scb_index);
    void (*lins_scb_enable_wakeup)(uint8_t scb_index);
    
    cy_en_pdstack_status_t (*boot_validate_fw)(sys_fw_metadata_t *fw_metadata);
    cy_en_pdstack_status_t (*ccg_power_throttle_get_set_config_oc_wrapper)(uint8_t port, uint8_t *oc2, uint8_t *oc3, bool flag, srom_ccg_power_contract_complete cb);
} srom_out_cbk_t;

typedef struct
{
    /* Crypto functions */
    mbedtls_sha256_fptr mbedtls_sha256;
    mbedtls_sha256_init_fptr mbedtls_sha256_init;
    mbedtls_sha256_free_fptr mbedtls_sha256_free;
    mbedtls_sha256_starts_fptr mbedtls_sha256_starts;
    mbedtls_sha256_update_fptr mbedtls_sha256_update;
    mbedtls_sha256_finish_fptr mbedtls_sha256_finish;
    uECC_verify_fptr uECC_verify;
    uECC_secp256r1_fptr uECC_secp256r1;
    uECC_sign_fptr uECC_sign;
    uECC_set_rng_fptr uECC_set_rng;
    uECC_valid_public_key_fptr uECC_valid_public_key;
    verify_signature_fptr verify_signature;
    crypto_init_globals_fptr crypto_init_globals;

    /* I2C functions */
    void(*i2c_slave_ack_ctrl)(uint8_t scb_index, bool enable);
    void(*i2c_reset)(uint8_t scb_index);
    bool(*i2c_scb_is_idle)(uint8_t scb_index);
    void(*i2c_scb_enable_wakeup)(uint8_t scb_index);
    void(*i2c_timer_cb)(uint8_t instance, cy_timer_id_t id);
    void(*i2c_scb_0_intr_handler)(void);
    void(*i2c_scb_1_intr_handler)(void);
    void(*i2c_scb_2_intr_handler)(void);
    void(*i2c_scb_3_intr_handler)(void);
    void(*i2c_scb_init)(uint8_t scb_index, i2c_scb_mode_t mode,
            i2c_scb_clock_freq_t clock_freq, uint8_t slave_addr,
            uint8_t slave_mask, i2c_cb_fun_t cb_fun_ptr,
            uint8_t *scratch_buffer, uint16_t scratch_buffer_size
            );
    void(*i2c_scb_deinit)(uint8_t scb_index);
    void(*i2c_scb_write)(uint8_t scb_index,
            uint8_t *source_ptr, uint8_t size,
            uint8_t *count );

    /* GPIO Functions. */
    void (*gpio_set_value)(gpio_port_pin_t port_pin, bool value);
    bool (*gpio_read_value)(gpio_port_pin_t port_pin);
    void (*gpio_set_drv_mode)(gpio_port_pin_t port_pin, gpio_dm_t drv_mode);
    void (*gpio_int_set_config)(gpio_port_pin_t port_pin, uint8_t int_mode);
    bool (*gpio_get_intr)(gpio_port_pin_t port_pin);
    void (*gpio_clear_intr)(gpio_port_pin_t port_pin);
    void (*hsiom_set_config)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode);
    void (*gpio_hsiom_set_config)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
            gpio_dm_t drv_mode, bool value);
    void (*gpio_set_lvttl_mode)(uint8_t port); 

    /* HPI Functions */
    bool (*hpi_is_ec_ready)(void);
    void (*hpi_config_run_time_params)(uint8_t port, hpi_config_run_time_t hpi_config);
    void (*hpi_auto_copy_data_to_flash)(uint8_t port, void *data, uint16_t size);
    void (*hpi_pd_event_handler)(uint8_t port, app_evt_t evt, const void *data);
    void (*hpi_set_boot_priority_conf)(uint8_t conf);
    void (*hpi_set_hpi_version)(uint32_t hpi_vers);
    void (*hpi_set_mode_regs)(uint8_t dev_mode, uint8_t mode_reason);
    void (*hpi_update_versions)(uint8_t *bl_version, uint8_t *fw1_version, uint8_t *fw2_version);
    void (*hpi_update_fw_locations)(uint16_t fw1_location, uint16_t fw2_location);
    void (*hpi_register_i2c_fsm)(fp ptr);
    bool (*hpi_reg_enqueue_event)(hpi_reg_section_t section, uint8_t status, uint16_t length,
            uint8_t *data);
    uint8_t *(*hpi_bb_get_version)(void);
    void (*hpi_bb_reg_update)(uint8_t bb_reg_addr, void *data);
    uint32_t (*hpi_bb_get_reg)(uint8_t bb_reg_addr);
    void (*hpi_auto_set_soln_cb_handler)(legacy_hpi_auto_soln_cb_t cb);
    void (*hpi_set_black_box_handler)(hpi_black_box_cb_t hpi_black_box_handler);
    void (*hpi_set_userdef_write_handler)(hpi_write_cb_t wr_handler);
    cy_en_pdstack_status_t (*hpi_init_userdef_regs)(uint16_t reg_addr,uint8_t size, uint8_t *data);
    void (*hpi_task)(void);
    bool (*hpi_i2c_cmd_callback)(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);
    bool (*hpi_lin_cmd_callback)(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);
    void (*hpi_set_no_boot_mode)(bool enable);
    uint8_t (*hpi_get_port_enable)(void);
    void (*hpi_set_fixed_slave_address)(uint8_t slave_addr);
    void (*hpi_set_ec_interrupt)(bool enable);
    void (*hpi_send_fw_ready_event)(void);
    void (*hpi_set_mode)(hpi_mode_t mode);
    hpi_mode_t (*hpi_get_mode)(void);
    void (*hpi_set_flash_params)(uint32_t flash_size, uint16_t row_size, uint16_t row_cnt,
            uint16_t bl_last_row);
    void (*hpi_init)(uint8_t scb_idx);
    bool (*hpi_sleep_allowed)(void);
    bool (*hpi_sleep)(void);
    void (*hpi_init_globals)(void);
    void (*hpi_register_command_overload)(fpoverload ptr);
} srom_in_cbk_t;

void srom_vars_init_ccg7d(void);

extern srom_out_cbk_t *gl_p_srom_out_cbk;
extern srom_in_cbk_t *gl_p_srom_in_cbk;
extern srom_vars_t *gl_p_srom_vars;

#if CCG_SROM_CODE_ENABLE
/************************************************************************
* MAPPING DEFINITION
************************************************************************/
#define CALL_OUT_FUNCTION(func_name) (gl_p_srom_out_cbk->func_name)
#define CALL_IN_FUNCTION(func_name) (gl_p_srom_in_cbk->func_name)
#define GET_IN_VARIABLE(var_name) (gl_p_srom_vars->var_name)

/************************************************************************
* ATTRIBUTE MAPPING
************************************************************************/
#if GENERATE_SROM_CODE

/* General */
#define ATTRIBUTES __attribute__ ((section (".sromCode"))) __attribute__((used))
/* I2C */
#if SROM_CODE_SCB_I2C
#define ATTRIBUTES_SCB_I2C  __attribute__ ((section (".i2cCode"))) __attribute__((used))
#else /* SROM_CODE_SCB_I2C */
#define ATTRIBUTES_SCB_I2C
#endif /* SROM_CODE_SCB_I2C */
/* GPIO */
#if SROM_CODE_SYS_GPIO
#define ATTRIBUTES_SYS_GPIO  __attribute__ ((section (".gpioCode"))) __attribute__((used))
#else /* SROM_CODE_SYS_GPIO */
#define ATTRIBUTES_SYS_GPIO
#endif /* SROM_CODE_SYS_GPIO */
/* SYSTEM */
#if SROM_CODE_SYS_SYS
#define ATTRIBUTES_SYS_SYS  __attribute__ ((section (".sysCode"))) __attribute__((used))
#else /* SROM_CODE_SYS_SYS */
#define ATTRIBUTES_SYS_SYS
#endif /* SROM_CODE_SYS_SYS */
/* Timer */
#if SROM_CODE_SYS_TIMER
#define ATTRIBUTES_SYS_TIMER  __attribute__ ((section (".timerCode"))) __attribute__((used))
#else /* SROM_CODE_SYS_TIMER */
#define ATTRIBUTES_SYS_TIMER
#endif /* SROM_CODE_SYS_TIMER */
/* TCPWM */
#if SROM_CODE_SYS_TCPWM
#define ATTRIBUTES_SYS_TCPWM  __attribute__ ((section (".tcpwmCode"))) __attribute__((used))
#else /* SROM_CODE_SYS_TCPWM */
#define ATTRIBUTES_SYS_TCPWM
#endif /* SROM_CODE_SYS_TCPWM */
/* Flash */
#if SROM_CODE_SYS_FLASH
#define ATTRIBUTES_SYS_FLASH  __attribute__ ((section (".flashCode"))) __attribute__((used))
#else /* SROM_CODE_SYS_FLASH */
#define ATTRIBUTES_SYS_FLASH
#endif /* SROM_CODE_SYS_FLASH */
/* App PDO */
#if SROM_CODE_APP_PDO
#define ATTRIBUTES_APP_PDO   __attribute__ ((section (".pdoCode"))) __attribute__((used))
#else /* SROM_CODE_APP_PDO */
#define ATTRIBUTES_APP_PDO
#endif /* SROM_CODE_APP_PDO */
/* PD Protocol */
#if SROM_CODE_PD_PROT
#define ATTRIBUTES_PD_PROT  __attribute__ ((section (".pdprotCode"))) __attribute__((used))
#else /* SROM_CODE_PD_PROT */
#define ATTRIBUTES_PD_PROT
#endif /* SROM_CODE_PD_PROT */
/* HPI Auto Boot */
#if SROM_CODE_HPISS_HPI
#define ATTRIBUTES_HPISS_HPI    __attribute__ ((section (".hpiCode"))) __attribute__((used))
#define HPI_GLOBAL_VAR          __attribute__ ((section (".hpi_globals"))) __attribute__((used))
#define HPI_CONST               __attribute__ ((section (".hpi_const"))) __attribute__((used))
#else /* SROM_CODE_HPISS_HPI */
#define ATTRIBUTES_HPISS_HPI 
#define HPI_GLOBAL_VAR
#define HPI_CONST 
#endif /* SROM_CODE_HPISS_HPI */
/* Crypto */
#if SROM_CODE_CRYPTO
#define ROM_STATIC_ATTRIBUTE __attribute__ ((section (".sromCode")))
#define CRYPTO_STATIC_ATTRIBUTE __attribute__ ((section (".cryptoCode")))
#define CRYPTO_ATTRIBUTE __attribute__ ((section (".cryptoCode"))) __attribute__((used))
#define CRYPTO_VAR_ATTRIBUTE __attribute__ ((section (".crypto_const"))) __attribute__((used))
#define CRYPTO_RSA_VAR_ATTRIBUTE __attribute__ ((section (".crypto_globals"), used))
#define ROM_CONSTANT __attribute__ ((section (".rom_rodata")))
#else /* !SROM_CODE_CRYPTO */
#define ROM_STATIC_ATTRIBUTE
#define CRYPTO_STATIC_ATTRIBUTE
#define CRYPTO_ATTRIBUTE
#define CRYPTO_VAR_ATTRIBUTE
#define CRYPTO_RSA_VAR_ATTRIBUTE
#define ROM_CONSTANT
#endif /* SROM_CODE_CRYPTO */

#else /* !GENERATE_SROM_CODE */

#define ATTRIBUTES
#define ATTRIBUTES_SCB_I2C
#define ATTRIBUTES_SYS_GPIO
#define ATTRIBUTES_SYS_SYS
#define ATTRIBUTES_SYS_TIMER
#define ATTRIBUTES_SYS_TCPWM
#define ATTRIBUTES_SYS_FLASH
#define ATTRIBUTES_APP_PDO
#define ATTRIBUTES_PD_PROT
#define ATTRIBUTES_HPISS_HPI
#define HPI_GLOBAL_VAR
#define HPI_CONST
#define ROM_STATIC_ATTRIBUTE
#define CRYPTO_STATIC_ATTRIBUTE
#define CRYPTO_ATTRIBUTE
#define CRYPTO_VAR_ATTRIBUTE
#define CRYPTO_RSA_VAR_ATTRIBUTE
#define ROM_CONSTANT

#endif /* GENERATE_SROM_CODE */
#endif /* CCG_SROM_CODE_ENABLE */

/************************************************************************
* CALL_OUT FUNCTIONS MAPPING
************************************************************************/
#if GENERATE_SROM_CODE
/* Crypto functions */
#if SROM_CODE_CRYPTO
#define call_calloc(func)                      CALL_OUT_FUNCTION(func)
#define call_free(func)                        CALL_OUT_FUNCTION(func)
#endif /* SROM_CODE_CRYPTO */

/* System functions. */
#if ((SROM_CODE_SCB_I2C) || (SROM_CODE_SYS_GPIO))
#define call_CyDelayUs(func)                   CALL_OUT_FUNCTION(func)
#define call_CyIntSetVector(func)              CALL_OUT_FUNCTION(func)
#define call_CyIntEnable(func)                 CALL_OUT_FUNCTION(func)
#define call_CyIntDisable(func)                CALL_OUT_FUNCTION(func)
#endif /* (SROM_CODE_SCB_I2C) || (SROM_CODE_SYS_GPIO) */

/* Timer functions */
#if ((SROM_CODE_SCB_I2C))
#define call_timer_start(func)                 CALL_OUT_FUNCTION(func)
#define call_timer_stop(func)                  CALL_OUT_FUNCTION(func)
#define call_timer_stop_range(func)            CALL_OUT_FUNCTION(func)
#define call_timer_is_running(func)            CALL_OUT_FUNCTION(func)
#endif /* (SROM_CODE_SCB_I2C) */ 

/* HPI Functions */

#if SROM_CODE_HPISS_HPI
#define call_CyEnterCriticalSection(func)               CALL_OUT_FUNCTION(func)
#define call_CyExitCriticalSection(func)                CALL_OUT_FUNCTION(func)
#define call_dpm_get_info(func)                         CALL_OUT_FUNCTION(func)
#define call_ccg_ls_is_enabled_wrapper(func)                    CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_feature_mask_wrapper(func)  CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_set_feature_mask_wrapper(func)  CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_set_oc_ec_wrapper(func)         CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_oc_ec_wrapper(func)         CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_oc_wrapper(func)            CALL_OUT_FUNCTION(func)
#define call_vconn_enable_wrapper(func)                         CALL_OUT_FUNCTION(func)
#define call_vconn_disable_wrapper(func)                        CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_oc_wrapper(func)            CALL_OUT_FUNCTION(func)
#define call_vconn_is_present_wrapper(func)                     CALL_OUT_FUNCTION(func)
#define call_ccg_ls_is_heart_beat_running_wrapper(func)         CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_port_budget_wrapper(func)   CALL_OUT_FUNCTION(func)
#define call_dpm_is_prev_contract_valid_wrapper(func)           CALL_OUT_FUNCTION(func)
#define call_ccg_ls_ctrl_wrapper(func)                          CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_set_pdp_wrapper(func)           CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_set_pdp_wrapper(func)           CALL_OUT_FUNCTION(func)
#define call_EC_INT_Write(func)                         CALL_OUT_FUNCTION(func)
#define call_CySoftwareReset(func)                      CALL_OUT_FUNCTION(func)
#define call_sys_get_device_mode(func)                  CALL_OUT_FUNCTION(func)
#define call_sys_get_silicon_id(func)                   CALL_OUT_FUNCTION(func)
#define call_sys_get_custom_info_addr(func)             CALL_OUT_FUNCTION(func)
#define call_dpm_stop(func)                             CALL_OUT_FUNCTION(func)
#define call_flash_enter_mode(func)                     CALL_OUT_FUNCTION(func)
#define call_flash_access_get_status(func)              CALL_OUT_FUNCTION(func)
#define call_boot_handle_validate_fw_cmd(func)          CALL_OUT_FUNCTION(func)
#define call_flash_row_write(func)                      CALL_OUT_FUNCTION(func)
#define call_flash_row_read(func)                       CALL_OUT_FUNCTION(func)
#define call_dpm_start(func)                            CALL_OUT_FUNCTION(func)
#define call_app_disable_pd_port_wrapper(func)                  CALL_OUT_FUNCTION(func)
#define call_lins_hpi_write(func)                       CALL_OUT_FUNCTION(func)
#define call_lins_scb_hpi_init(func)                    CALL_OUT_FUNCTION(func)
#define call_lins_scb_deinit(func)                      CALL_OUT_FUNCTION(func)
#define call_lins_scb_enable_wakeup(func)               CALL_OUT_FUNCTION(func)
#define call_boot_validate_fw(func)                     CALL_OUT_FUNCTION(func)
#define call_ccg_power_throttle_get_set_config_oc_wrapper(func) CALL_OUT_FUNCTION(func)
#endif /* SROM_CODE_HPISS_HPI */

#endif /* GENERATE_SROM_CODE */

/************************************************************************
* CALL_IN FUNCTIONS MAPPING
************************************************************************/
/* Crypto Functions */
#if SROM_CODE_CRYPTO
#define call_mbedtls_sha256(func)               CALL_IN_FUNCTION(func)
#define call_mbedtls_sha256_init(func)          CALL_IN_FUNCTION(func)
#define call_mbedtls_sha256_free(func)          CALL_IN_FUNCTION(func)
#define call_mbedtls_sha256_starts(func)        CALL_IN_FUNCTION(func)
#define call_mbedtls_sha256_update(func)        CALL_IN_FUNCTION(func)
#define call_mbedtls_sha256_finish(func)        CALL_IN_FUNCTION(func)
#define call_uECC_verify(func)                  CALL_IN_FUNCTION(func)
#define call_uECC_secp256r1(func)               CALL_IN_FUNCTION(func)
#define call_uECC_sign(func)                    CALL_IN_FUNCTION(func)
#define call_uECC_set_rng(func)                 CALL_IN_FUNCTION(func)
#define call_uECC_valid_public_key(func)        CALL_IN_FUNCTION(func)
#define call_verify_signature(func)             CALL_IN_FUNCTION(func)
#define call_crypto_init_globals(func)          CALL_IN_FUNCTION(func)
#endif /* SROM_CODE_CRYPTO */

/* I2C Functions */
#if SROM_CODE_SCB_I2C
#define call_i2c_slave_ack_ctrl(func)            CALL_IN_FUNCTION(func)
#define call_i2c_reset(func)                     CALL_IN_FUNCTION(func)
#define call_i2c_scb_is_idle(func)               CALL_IN_FUNCTION(func)
#define call_i2c_scb_enable_wakeup(func)         CALL_IN_FUNCTION(func)
#define call_i2c_timer_cb(func)                  CALL_IN_FUNCTION(func)
#define call_i2c_scb_0_intr_handler(func)        CALL_IN_FUNCTION(func)
#define call_i2c_scb_1_intr_handler(func)        CALL_IN_FUNCTION(func)
#define call_i2c_scb_2_intr_handler(func)        CALL_IN_FUNCTION(func)
#define call_i2c_scb_3_intr_handler(func)        CALL_IN_FUNCTION(func)
#define call_i2c_scb_init(func)                  CALL_IN_FUNCTION(func)
#define call_i2c_scb_deinit(func)                CALL_IN_FUNCTION(func)
#define call_i2c_scb_write(func)                 CALL_IN_FUNCTION(func)
#endif /* SROM_CODE_SCB_I2C */

/* GPIO Functions. */
#if SROM_CODE_SYS_GPIO
#define call_gpio_set_value(func)                CALL_IN_FUNCTION(func)
#define call_gpio_read_value(func)               CALL_IN_FUNCTION(func)
#define call_gpio_set_drv_mode(func)             CALL_IN_FUNCTION(func)
#define call_gpio_int_set_config(func)           CALL_IN_FUNCTION(func)
#define call_gpio_get_intr(func)                 CALL_IN_FUNCTION(func)
#define call_gpio_clear_intr(func)               CALL_IN_FUNCTION(func)
#define call_hsiom_set_config(func)              CALL_IN_FUNCTION(func)
#define call_gpio_hsiom_set_config(func)         CALL_IN_FUNCTION(func)
#define call_gpio_set_lvttl_mode(func)           CALL_IN_FUNCTION(func)
#endif /* SROM_CODE_SYS_GPIO */

/* HPI Functions */

#if SROM_CODE_HPISS_HPI
#define call_hpi_is_ec_ready(func)                      CALL_IN_FUNCTION(func)
#define call_hpi_config_run_time_params(func)           CALL_IN_FUNCTION(func)
#define call_hpi_auto_copy_data_to_flash(func)          CALL_IN_FUNCTION(func)
#define call_hpi_pd_event_handler(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_set_boot_priority_conf(func)           CALL_IN_FUNCTION(func)
#define call_hpi_set_hpi_version(func)                  CALL_IN_FUNCTION(func)
#define call_hpi_set_mode_regs(func)                    CALL_IN_FUNCTION(func)
#define call_hpi_update_versions(func)                  CALL_IN_FUNCTION(func)
#define call_hpi_update_fw_locations(func)              CALL_IN_FUNCTION(func)
#define call_hpi_register_i2c_fsm(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_register_command_overload(func)        CALL_IN_FUNCTION(func)
#define call_hpi_reg_enqueue_event(func)                CALL_IN_FUNCTION(func)
#define call_hpi_bb_get_version(func)                   CALL_IN_FUNCTION(func)
#define call_hpi_bb_reg_update(func)                    CALL_IN_FUNCTION(func)
#define call_hpi_bb_get_reg(func)                       CALL_IN_FUNCTION(func)
#define call_hpi_auto_set_soln_cb_handler(func)         CALL_IN_FUNCTION(func)
#define call_hpi_set_black_box_handler(func)            CALL_IN_FUNCTION(func)
#define call_hpi_set_userdef_write_handler(func)        CALL_IN_FUNCTION(func)
#define call_hpi_init_userdef_regs(func)                CALL_IN_FUNCTION(func)
#define call_hpi_task(func)                             CALL_IN_FUNCTION(func)
#define call_hpi_i2c_cmd_callback(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_lin_cmd_callback(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_set_no_boot_mode(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_get_port_enable(func)                  CALL_IN_FUNCTION(func)
#define call_hpi_set_fixed_slave_address(func)          CALL_IN_FUNCTION(func)
#define call_hpi_set_ec_interrupt(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_send_fw_ready_event(func)              CALL_IN_FUNCTION(func)
#define call_hpi_set_mode(func)                         CALL_IN_FUNCTION(func)
#define call_hpi_get_mode(func)                         CALL_IN_FUNCTION(func)
#define call_hpi_set_flash_params(func)                 CALL_IN_FUNCTION(func)
#define call_hpi_init(func)                             CALL_IN_FUNCTION(func)
#define call_hpi_sleep_allowed(func)                    CALL_IN_FUNCTION(func)
#define call_hpi_sleep(func)                            CALL_IN_FUNCTION(func)
#define call_hpi_init_globals(func)                     CALL_IN_FUNCTION(func)
#endif /* SROM_CODE_HPISS_HPI */
/************************************************************************
* GET_IN_VARIABLE
************************************************************************/
/* Crypto variables */
#if SROM_CODE_CRYPTO
#define call_g_rng_function(func)               GET_IN_VARIABLE(func)
#endif /* SROM_CODE_CRYPTO */

/* I2C variables */
#if SROM_CODE_SCB_I2C
#define call_SCB_PRT(func)                      GET_IN_VARIABLE(func)
#define call_gl_i2c_scb_config(func)            GET_IN_VARIABLE(func)
#define call_i2c_ack_disable(func)              GET_IN_VARIABLE(func)
#define call_i2c_ack_pending(func)              GET_IN_VARIABLE(func)
#endif /* SROM_CODE_SCB_I2C */

/* GPIO variables */
#if SROM_CODE_SYS_GPIO
#define call_GPIO(func)                         GET_IN_VARIABLE(func)
#define call_HSIOM(func)                        GET_IN_VARIABLE(func)
#endif /* SROM_CODE_SYS_GPIO */

/* HPI variables */
#if SROM_CODE_HPISS_HPI
#define call_gl_p_cyBtldrRunType(func)           GET_IN_VARIABLE(func)
#endif /* SROM_CODE_HPISS_HPI */
/** @endcond */
/************************************************************************
* DEFAULT MAPPING.
* Default mapping for any CALL_MAP in the code base.
* This mapping does direct function or variable call.
************************************************************************/
#include "srom_mapping.h"
#endif /* (defined(CY_DEVICE_CCG7D) && (CCG_SROM_CODE_ENABLE)) */
#endif /* SROM_VARS_CCG7D_H_ */
