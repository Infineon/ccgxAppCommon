/******************************************************************************
* File Name: srom_dependency.h
* \version 2.0
*
* Description: Header file for CCG7S SROM dependencies
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef SROM_DEPENDENCY_H_
#define SROM_DEPENDENCY_H_

#include "cy_pdstack_common.h"

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#if (!CCG_HPI_ENABLE && !CCG_HPI_OVER_LIN_ENABLE)
/**
 * @typedef hpi_reg_section_t
 * @brief HPI register section definitions.
 *
 * HPI registers are grouped into sections corresponding to the functions that are supported.
 */
typedef enum
{
    HPI_REG_SECTION_DEV = 0,            /**< Device information registers. */
    HPI_REG_SECTION_PORT_0,             /**< USB-PD Port 0 related registers. */
    HPI_REG_SECTION_PORT_1,             /**< USB-PD Port 1 related registers. */
    HPI_REG_SECTION_LIN_DEV = 0x03,     /**< Device Information section for LIN. */
    HPI_REG_SECTION_DEV_AUTO_P1 = 0x05, /**< HPI Auto Port 1 related registers */
    HPI_REG_SECTION_DEV_AUTO_P0 = 0x06, /**< HPI Auto Port 0 related registers */
#if CCG_UCSI_ENABLE
    HPI_REG_SECTION_UCSI = 0x0F,        /**< UCSI related registers. */
#endif /*CCG_UCSI_ENABLE*/
    HPI_REG_SECTION_ALL                 /**< Special definition to select all register spaces. */
} hpi_reg_section_t;

/**
 * @typedef hpi_reg_part_t
 * @brief Types of HPI register/memory regions.
 */
typedef enum
{
    HPI_REG_PART_REG = 0,               /**< Register region. */
    HPI_REG_PART_DATA = 1,              /**< Data memory for device section. */
    HPI_REG_PART_FLASH = 2,             /**< Flash memory. */
    HPI_REG_PART_PDDATA_READ = 4,       /**< Read Data memory for port section. */
    HPI_REG_PART_PDDATA_READ_H = 5,     /**< Upper fraction of read data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE = 8,      /**< Write Data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE_H = 9     /**< Upper fraction of write data memory for port section. */
} hpi_reg_part_t;

/**
 * @typedef hpi_mode_t
 * @brief Mode definitions for HPI bootloader and firmware mode.
 */
typedef enum
{
    HPI_MODE_BOOTLOADER = 0u,
    HPI_MODE_FIRMWARE,
    HPI_MODE_MAX
} hpi_mode_t;

/**
 * @typedef hpi_write_cb_t
 * @brief Handler for HPI register writes.
 * @return Type of response to be sent to the EC. Only a single byte response
 * can be sent from here. Use hpi_reg_enqueue_event to send longer responses.
 */
typedef uint8_t (*hpi_write_cb_t)(
        uint16_t  reg_addr,             /**< Address of register that got written. */
        uint8_t   wr_size,              /**< Size of write operation. */
        uint8_t  *wr_data               /**< Buffer containing data written. */
        );
#endif /* (!CCG_HPI_ENABLE && !CCG_HPI_OVER_LIN_ENABLE) */

#if !CCG_HPI_AUTO_CMD_ENABLE  || (!CCG_HPI_ENABLE)

#include "battery_charging.h"
#include "cy_usbpd_config_table.h"

/** @cond DOXYGEN_HIDE */
/**
 * @typedef oc_placeholder_t 
 * @brief Structure to hold the OC reported values from solution space.
 */
typedef struct
{
    uint8_t oc_vin;
    uint8_t oc_temp;
} hpi_oc_buffer_t;

/**
 * @typedef legacy_hpi_auto_soln_cb_t
 * @brief Structure to hold the Solution interface for HPI Auto operation.
 * This is only required for CCG7D for rom compatibility.
 * Solution is expected to fill the structure with required pointers to function
 * to accomplish the command specific tasks. All the registered functions
 * should be non-blocking and take minimum execution time.
 */
typedef struct
{
    const bc_status_t* (*legacy_soln_bc_get_status_handler) (
        uint8_t port               /**< PD port index. */
        );

    auto_cfg_settings_t* (*legacy_soln_get_auto_config_table) (
        uint8_t port               /**< PD port index. */
        );

    uint32_t (*legacy_soln_get_fault_status) (
        uint8_t port               /**< PD port index. */
        );

    cy_en_pdstack_status_t (*legacy_soln_get_sensor_temperature) (
        uint8_t port,              /**< PD port index. */
        uint8_t *buffer            /**< Output temperature place holder. */
        );

    uint16_t (*legacy_soln_get_vbus_voltage)  (
        uint8_t port               /**< PD port index. */
        );

    uint16_t (*legacy_soln_get_vbus_current)  (
        uint8_t port               /**< PD port index. */
        );

    uint16_t (*legacy_soln_get_battery_voltage)  (
        uint8_t port               /**< PD port index. */
        );

    uint8_t (*legacy_soln_get_oc_details)  (
        uint8_t port,              /**< PD port index. */
        hpi_oc_buffer_t *buffer    /**<OC placeholder. */
        );
} legacy_hpi_auto_soln_cb_t;

/**
 * @typedef hpi_auto_soln_cb_t
 * @brief Structure to hold the Solution interface for HPI Auto operation.
 * Solution is expected to fill the structure with required pointers to function
 * to accomplish the command specific tasks. All the registered functions
 * should be non-blocking and take minimum execution time.
 */
typedef struct
{
    /**< Battery Charging status. */
    const bc_status_t* (*soln_bc_get_status_handler) (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );
    /**< Get Auto Configuration Table. */
    auto_cfg_settings_t* (*soln_get_auto_config_table) (
        uint8_t port                                                /**< PD port Index. */
        );
    /**< Get Fault status. */    
    uint32_t (*soln_get_fault_status) (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );
    /**< Get Sensor Temperature. */
    cy_en_pdstack_status_t (*soln_get_sensor_temperature) (
        cy_stc_pdstack_context_t *ptrPdStackContext,              /**< PD port Context. */
        uint8_t *buffer                                           /**< Output temperature place holder. */
        );
    /**< Get Vbus Voltage. */
    uint16_t (*soln_get_vbus_voltage) (
        cy_stc_usbpd_context_t *context,                            /**< USBPD port Context. */
        cy_en_usbpd_adc_id_t adcId,                                 /**< ADC ID. */
        cy_en_usbpd_adc_input_t input                               /**< ADC Input. */
        );
    /**< Get Vbus Current. */
    uint16_t (*soln_get_vbus_current)  (
        cy_stc_usbpd_context_t *context                             /**< USBPD port Context. */
        );
    /**< Get Battery Voltage. */
    uint16_t (*soln_get_battery_voltage)  (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );
    /**< Get oc details. */
    uint8_t (*soln_get_oc_details)  (
        cy_stc_pdstack_context_t *ptrPdStackContext,              /**< PD port Context. */
        hpi_oc_buffer_t *buffer    /**<OC placeholder. */
        );
    /**< Get version details. */
    uint32_t (*soln_get_version_details)  (
        cy_stc_pdstack_context_t *ptrPdStackContext              /**< PD port Context. */
        );
} hpi_auto_soln_cb_t;
/** @endcond */

#endif /* !CCG_HPI_AUTO_CMD_ENABLE  || (!CCG_HPI_ENABLE) */

#if (!CCG_HPI_ENABLE) || (!SYS_BLACK_BOX_ENABLE)
/**
 * @typedef hpi_black_box_cb_t
 * @brief Handler for black box access hpi command.
 * @return Address of black box.
 */
typedef uint32_t (*hpi_black_box_cb_t)(void);
#endif /* (!CCG_HPI_ENABLE) || (!SYS_BLACK_BOX_ENABLE) */

#ifndef POWER_THROTTLE_H_
typedef void (*ccg_power_contract_complete) (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power);

/**
 * @typedef srom_ccg_power_contract_complete
 *
 * @brief Function pointer defining the function type to be registered with
 * Power throttle functionality. Registered function will get called post
 * recontract completion. If nothing was attached at the time of power
 * update request, power notified will be 0.
 * Callback will only be notified if ccg_power_throttle_set_oc() function
 * accepted the request successfully.
 *
 * @param port Port index.
 * @param power Current power consumption. Value of 0 signifies no attach.
 *
 * @return None.
 */
typedef void (*srom_ccg_power_contract_complete) (uint8_t port, uint8_t power);



/**
 * @typedef operating_condition_t
 * @brief Define the system operating condition values.
 */
typedef enum
{
    SYSTEM_OC_1 = 1,           /**< System Operating condition 1: 100% of Power budget */
    SYSTEM_OC_2,               /**< System Operating condition 2: 50% of Power budget */
    SYSTEM_OC_3,               /**< System Operating condition 3: 15W Operation */
    SYSTEM_OC_4                /**< System Operating condition 4: Port Shutdown */
} operating_condition_t;
#endif /* POWER_THROTTLE_H_ */

#if (((!CCG_LOAD_SHARING_ENABLE) && (!CCG_HPI_AUTO_CMD_ENABLE)) || (!CCG_HPI_ENABLE))
typedef uint8_t (*hpi_auto_soln_cmd_handler_ptr) (cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t cmd, uint8_t *value, uint8_t cmd_len);
/** @cond DOXYGEN_HIDE */
typedef struct
{
#if CCG_HPI_BB_ENABLE
    bool hpi_bb_operation_enable;
#endif /* CCG_HPI_BB_ENABLE */
#if CCG_HPI_OVER_LIN_ENABLE
    bool hpi_lin_operation_enable;
    bool hpi_lin_multi_master_mode;
#endif /* CCG_HPI_OVER_LIN_ENABLE */
    bool hpi_dual_firmware_enable;
    hpi_auto_soln_cmd_handler_ptr hpi_auto_soln_cmd_handler;
}hpi_config_run_time_t;
/** @endcond */
#endif /* (((!CCG_LOAD_SHARING_ENABLE) && (!CCG_HPI_AUTO_CMD_ENABLE)) || (!CCG_HPI_ENABLE)) */

#if (CCG_BOOT || !HPI_AUTO_SROM_ENABLE || (!CCG_HPI_ENABLE))
#include "hpi_internal.h"
/* QAC suppression 0631: Extra declaration required to avoid SROM generation build failure. */
typedef void (*fp)(uint8_t cmd_opcode, uint8_t *cmd_prm, uint8_t *flash_mem_addr, cy_en_pdstack_status_t *stat, hpi_response_t *code, bool *is_handled); /* PRQA S 0631 */
typedef bool (*fpoverload)(uint8_t cmd_opcode, uint8_t *cmd_param, uint8_t  cmd_length, hpi_response_t *code, uint8_t *hpi_auto_cmd, uint8_t *hpi_auto_port);
#endif /* (CCG_BOOT || !HPI_AUTO_SROM_ENABLE || (!CCG_HPI_ENABLE)) */

#if CCG_BOOT
bool ccg_ls_is_enabled(cy_stc_pdstack_context_t *ptrPdStackContext);
bool ccg_ls_is_heart_beat_running(cy_stc_pdstack_context_t *ptrPdStackContext);
cy_en_pdstack_status_t ccg_ls_ctrl(cy_stc_pdstack_context_t *ptrPdStackContext, bool disable_control, bool flag, ccg_power_contract_complete cb);
void lins_hpi_write(uint8_t  scb_index, uint8_t *source_ptr, uint8_t  size, uint8_t *count);
void lins_scb_hpi_init(uint8_t scb_index,
        i2c_cb_fun_t cb_fun_ptr,
        uint8_t *scratch_buffer, uint16_t scratch_buffer_size);
void lins_scb_deinit(uint8_t scb_index);
void lins_scb_enable_wakeup(uint8_t scb_index);
#endif /* CCG_BOOT */

#if !CCG_LIN_ENABLE && !CCG_BOOT
void lins_hpi_write(uint8_t scb_index, uint8_t *source_ptr, uint8_t  size, uint8_t *count);
void lins_scb_hpi_init(uint8_t scb_index,
        i2c_cb_fun_t cb_fun_ptr,
        uint8_t *scratch_buffer, uint16_t scratch_buffer_size);
void lins_scb_deinit(uint8_t scb_index);
void lins_scb_enable_wakeup(uint8_t scb_index);
#endif /* !CCG_LIN_ENABLE && !CCG_BOOT */

#if ((!(CCG_HPI_ENABLE || CCG_HPI_OVER_LIN_ENABLE)) || (!(!CCG_HPI_OVER_LIN_ENABLE || HPI_AUTO_SROM_ENABLE)))

/* QAC suppression 3408: SROM generation needs definition to this external function to build. Hence,
 * dummy defintion provided. */
void EC_INT_Write(uint8_t value); /* PRQA S 3408 */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext);

cy_en_pdstack_status_t app_disable_pd_port(cy_stc_pdstack_context_t *ptrPdStackContext, cy_pdstack_dpm_typec_cmd_cbk_t cbk);
#endif /* ((!(CCG_HPI_ENABLE || CCG_HPI_OVER_LIN_ENABLE)) || (!(!CCG_HPI_OVER_LIN_ENABLE || HPI_AUTO_SROM_ENABLE))) */

#if (!(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_ENABLE || CCG_BOOT))
uint8_t ccg_power_throttle_get_feature_mask(cy_stc_pdstack_context_t *ptrPdStackContext);
void ccg_power_throttle_set_feature_mask(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t mask);
cy_en_pdstack_status_t ccg_power_throttle_set_oc_ec(cy_stc_pdstack_context_t *ptrPdStackContext, operating_condition_t oc, ccg_power_contract_complete cb);
operating_condition_t ccg_power_throttle_get_oc_ec(cy_stc_pdstack_context_t *ptrPdStackContext);
operating_condition_t ccg_power_throttle_get_oc(cy_stc_pdstack_context_t *ptrPdStackContext);
uint8_t ccg_power_throttle_get_port_budget(cy_stc_pdstack_context_t *ptrPdStackContext);
cy_en_pdstack_status_t ccg_power_throttle_set_pdp(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t power, ccg_power_contract_complete cb);
cy_en_pdstack_status_t ccg_power_throttle_get_set_config_oc(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t *oc2, uint8_t *oc3, bool flag, ccg_power_contract_complete cb);
#endif /* (!(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_HPI_ENABLE || CCG_BOOT)) */

#endif /* defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D) */

#endif /* SROM_DEPENDENCY_H_ */
