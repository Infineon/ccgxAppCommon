/***************************************************************************//**
* \file uvdm.h
* \version 1.1.0 
*
* Unstructured VDM handler header file.
* These definitions correspond to an unstructured VDM based protocol
* implementation that Cypress devices use to facilitate firmware or
* configuration updates over the CC lines.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
* \addtogroup group_ccgxAppCommon Common source files
* \{
*/

#ifndef UVDM_H_
#define UVDM_H_

#include "cy_usbpd_defines.h"
#include "cy_pdstack_common.h"
#include "flash.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

#define UVDM_RESPONSE_MAX_NO_OF_VDO             (0x07u)  /**< Maximum number of VDOs in U_VDM response. */
#define UVDM_HEADER_INDEX                       (0x00u)  /**< VDM Header index in U_VDM Response. */
#define UVDM_SIGNATURE_BYTE_OFFSET              (0x00u)  /**< U_VDM commands signature byte offset. */
#define UVDM_DEVICE_MODE_VDO_INDEX              (0x01u)  /**< Response U_VDM DEVICE_MODE VDO index. */
#define UVDM_BOOT_LAST_ROW_VDO_INDEX            (0x02u)  /**< BOOT LAST ROW VDO Index. */
#define UVDM_BOOT_VERSION_VDO_INDEX             (0x01u)  /**< Response U_VDM BOOT VERSION VDO index. */
#define UVDM_IMG1_VERSION_VDO_INDEX             (0x03u)  /**< Response U_VDM FW Image 1 VERSION VDO index. */
#define UVDM_IMG2_VERSION_VDO_INDEX             (0x05u)  /**< Response U_VDM FW Image 2 VERSION VDO index. */
#define UVDM_VERSION_NUM_SIZE_BYTES             (0x08u)  /**< Version number size in bytes. */
#define UVDM_GET_VERSION_U_VDM_NO_OF_VDO        (0x06u)  /**< Number of VDOs in GET_VERSION U_VDM Response. */
#define UVDM_GET_SILICON_ID_CMD_SIZE            (0x04u)  /**< GET_SILICON_ID Command size in bytes. */
#define UVDM_GET_SILICON_ID_CMD_SIG             (0x53u)  /**< GET_SILICON_ID Command Signature: 'S' */
#define UVDM_FW1_START_ADDR_VDO_INDEX           (0x01u)  /**< Response UVDM_FW1_START_ADDR VDO index. */
#define UVDM_FW2_START_ADDR_VDO_INDEX           (0x02u)  /**< Response UVDM_FW2_START_ADDR VDO index. */
#define UVDM_GET_FW_START_ADDR_UVDM_NO_OF_VDO   (0x02u)  /**< Number of VDOs in GET_FW_START_ADDRESS_UVDM response. */
#define UVDM_SILICON_ID_VDO_INDEX               (0x02u)  /**< SILICON_ID VDO index in U_VDM response. */
#define UVDM_DEVICE_RESET_CMD_SIZE              (0x04u)  /**< DEVICE_RESET Command size in bytes. */
#define UVDM_DEVICE_RESET_CMD_SIG               (0x52u)  /**< Device RESET Command Signature: 'R' */
#define UVDM_JUMP_TO_BOOT_CMD_SIZE              (0x04u)  /**< JUMP_TO_BOOT Command size in bytes. */
#define UVDM_JUMP_TO_BOOT_CMD_SIG               ('J')   /**< JUMP_TO_BOOT Command Signature: 'J' */
#define UVDM_JUMP_TO_ALT_FW_SIG                 ('A')   /**< Signature value used to request JUMP_TO_ALT_FW: 'A' */
#define UVDM_ENTER_FLASHING_MODE_CMD_SIZE       (0x04u)  /**< ENTER_FLASHING_MODE Command size in bytes. */
#define UVDM_ENTER_FLASHING_MODE_CMD_SIG        (0x50u)  /**< ENTER FLASHING MODE Command Signature: 'P' */
#define UVDM_FLASH_READ_WRITE_CMD_SIZE          (0x04u)  /**< Flash write and read command size. */
#define UVDM_FLASH_READ_WRITE_CMD_SIG           (0x46u)  /**< FLASH_READ_WRITE Command Signature: 'F' */
#define UVDM_FLASH_ROW_NUM_LSB_OFFSET           (0x01u)  /**< Flash Write and Read command row num LSB offset. */
#define UVDM_FLASH_ROW_NUM_MSB_OFFSET           (0x02u)  /**< Flash Write and Read command row num MSB offset. */
#define UVDM_READ_DATA_RESPONSE_VDO_INDEX       (0x01u)  /**< READ_DATA Response VDO index. */
#define UVDM_RESPONSE_VDO_INDEX                 (0x01u)  /**< U_VDM Response VDO index. */
#define UVDM_READ_DATA_NO_OF_VDO_ERROR_CASE     (0x02u)  /**< READ_DATA response no of VDOs in error case. */
#define UVDM_VALIDATE_FW_CMD_SIZE               (0x04u)  /**< VALIDATE_FW command size. */
#define UVDM_VALIDATE_FW_MODE_INDEX             (0x00u)  /**< VALIDATE_FW Register FW MODE offset. */
#define UVDM_REASON_FOR_BOOT_MODE_VDO_INDEX     (0x01u)  /**< REASON_FOR_BOOT_MODE VDO index. */
#define UVDM_GET_CHECKSUM_CMD_SIZE              (0x08u)  /**< GET_CHECKSUM command size. */
#define UVDM_FLASH_ADDR_LSB_OFFSET              (0x00u)  /**< GET_CHECKSUM CMD FLASH ADDR LSB offset. */
#define UVDM_FLASH_SIZE_LSB_OFFSET              (0x04u)  /**< GET_CHECKSUM CMD FLASH SIZE LSB offset. */
#define UVDM_CHECKSUM_VDO_INDEX                 (0x01u)  /**< GET_CHECKSUM response checksum VDO index. */
#define UVDM_SET_APP_PRIORITY_CMD_SIZE          (0x04u)  /**< SET_APP_PRIORITY command size. */
#define UVDM_SET_APP_PRIORITY_INDEX             (0x00u)  /**< SET_APP_PRIORITY FW Image priority offset. */
#define UVDM_SEND_SIGN_SEQUENCE_1               (0x01u)  /**< Send Signature UVDM Valid Sequence Number 1. */
#define UVDM_SEND_SIGN_SEQUENCE_2               (0x02u)  /**< Send Signature UVDM Valid Sequence Number 2. */
#define UVDM_SEND_SIGN_SEQUENCE_3               (0x03u)  /**< Send Signature UVDM Valid Sequence Number 3. */
#define UVDM_SEND_SIGN_SEC_1_2_SIZE             (0x18u)  /**< Send Signature UVDM Section 1 and 2 size in Bytes. */
#define UVDM_SEND_SIGN_SEC_3_SIZE               (0x10u)  /**< Send Signature UVDM Section 3 size in Bytes. */
#define UVDM_GET_CUSTOMER_INFO_SEQ_1            (0x01u)  /**< GET_CUSTOMER_INFO VDM Sequence Number 1. */
#define UVDM_GET_CUSTOMER_INFO_SEQ_2            (0x02u)  /**< GET_CUSTOMER_INFO VDM Sequence Number 2. */
#define UVDM_GET_CUSTOMER_INFO_RESPONSE_SIZE    (0x10u)  /**< GET_CUSTOMER_INFO Response Size in bytes. */
#define UVDM_GET_CUSTOMER_INFO_RESPONSE_VDO_NUM (0x04u)  /**< GET_CUSTOMER_INFO number of VDOs in response. */
/** @cond DOXYGEN_HIDE */
#define UVDM_BLACK_BOX_CMD_SIZE                 (0x04u)  /**< BLACK_BOX command size in bytes */
#define UVDM_BLACK_BOX_CHUNK_SIZE               (0x18u)  /**< Maximum chunk of bytes to be loaded in UVDM to send black box data. */
#define UVDM_TEST_CODE_COVERAGE_CMD_SIZE        (0x04u)  /**< TEST_CODE_COVERAGE command size in bytes. */
#define UVDM_TCC_GET_DATA                       (0x01u)  /**< TEST_CODE_COVERAGE command argument to retrieve or reset TCC data. */
#define UVDM_TCC_GET_STAT                       (0x02u)  /**< TEST_CODE_COVERAGE command argument to retrieve TCC statistics.
                                                             This includes total number of active functions and percentage of
                                                             executed functions. */
#define UVDM_TCC_GET_RAW_DATA                   (0x00u)  /**< TEST_CODE_COVERAGE command argument to retrieve TCC raw data. */
#define UVDM_TCC_RESET_RAW_DATA                 (0x01u)  /**< TEST_CODE_COVERAGE command argument to reset TCC raw data. */
#define UVDM_TCC_GET_PERCENTAGE                 (0x00u)  /**< TEST_CODE_COVERAGE command argument to get percentage of executed functions. */
#define UVDM_TCC_GET_RAW_DATA_SIZE              (0x01u)  /**< TEST_CODE_COVERAGE command argument to get the total number of active functions. */
#define UVDM_TCC_RAW_DATA_MAX_CHUNK_SIZE        (0x18u)  /**< Maximum chunk of bytes to be loaded in the UVDM to send TCC raw data. */
#define UVDM_TCC_RAW_DATA_MAX_SIZE              (256u)  /**< Maximum size of TCC raw data in bytes. */
#define UVDM_TCC_GET_PERCENTAGE_NO_OF_VDO       (0x01u)  /**< No of VDOs responded to TEST_CODE_COVERAGE command with get percentage argument. */
#define UVDM_TCC_GET_RAW_DATA_SIZE_NO_OF_VDO    (0x01u)  /**< No of VDOs responded to TEST_CODE_COVERAGE command with get raw data size argument. */
/** @endcond */

#define UVDM_FWCT_SIG_WRITE_CMD_SIZE            (0x04u)  /**< FWCT Signature write command size in bytes */
#define UVDM_FWCT_ROW_WRITE_CMD_SIZE            (0x04u)  /**< FWCT Row write command size in bytes */

#define UVDM_FWCT_SIG_WRITE_CMD_SIG             ('M')   /**< FWCT Signature write command signature */
#define UVDM_FWCT_ROW_WRITE_CMD_SIG             ('N')   /**< FWCT Row write command signature */

#define CCG_STATUS_CODE_OFFSET                  (2u)    /**< CCG Status offset */

#define CCG_STATUS_TO_HPI_RESPONSE(c)   ((c) + CCG_STATUS_CODE_OFFSET) /**< Convert CCG status code to HPI/UVDM response code. */

/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/

/**
 * @typedef uvdm_cmd_opcode_t
 * @brief List of opcodes supported in Cypress Flashing Alternate Mode U_VDMs.
 */
typedef enum
{
    UVDM_CMD_RESERVED,                      /**< Reserved. */
    UVDM_CMD_GET_DEVICE_MODE_OPCODE,        /**< Get active mode of device. */
    UVDM_CMD_GET_DEVICE_VERSION_OPCODE,     /**< Get version information of all images. */
    UVDM_CMD_GET_SILICON_ID_OPCODE,         /**< Get silicon ID of CCG. */
    UVDM_CMD_DEVICE_RESET_OPCODE,           /**< Reset CCG. */
    UVDM_CMD_JUMP_TO_BOOT_OPCDOE,           /**< Jump to boot mode. */
    UVDM_CMD_ENTER_FLASHING_MODE_OPCODE,    /**< Enable flash access mode. */
    UVDM_CMD_SEND_DATA_OPCODE,              /**< Collect flash row data for write. */
    UVDM_CMD_FLASH_WRITE_OPCODE,            /**< Program flash row. */
    UVDM_CMD_READ_DATA_OPCODE,              /**< Collect flash row data for read. */
    UVDM_CMD_FLASH_READ_OPCODE,             /**< Read flash row. */
    UVDM_CMD_VALIDATE_FW_OPCODE,            /**< Validate FW image. */
    UVDM_CMD_REASON_FOR_BOOT_MODE,          /**< Get reason for boot mode. */
    UVDM_CMD_GET_CHECKSUM,                  /**< Get checksum of specified flash section. */
    UVDM_CMD_GET_FW_START_ADDRESS_OPCODE,   /**< Get start address of FW images. */
    UVDM_CMD_SET_APP_PRIORITY_OPCODE,       /**< Set APP priority value. */
    UVDM_CMD_RESERVED_16,                   /**< Reserved. */
    UVDM_CMD_SEND_SIGNATURE_OPCODE,         /**< Program FW image ECDSA signature. */
    UVDM_CMD_RESERVED_18,                   /**< Reserved. */
    UVDM_CMD_GET_BOOT_TYPE,                 /**< Get information related to boot loader type. */
    UVDM_CMD_GET_CUSTOMER_INFO,             /**< Get custom data. */
    UVDM_CMD_RESERVED_21,                   /**< 21: Reserved. */
/** @cond DOXYGEN_HIDE */
    UVDM_CMD_GET_BLACK_BOX,                 /**< 22: Send black box data as UVDM. */
    UVDM_CMD_TEST_CODE_COVERAGE,            /**< 23: Access function level code coverage information */
/** @endcond */
    UVDM_CMD_FWCT_SIG_WRITE,                /**< 24: Obtain the signature for the FWCT */
    UVDM_CMD_FWCT_ROW_WRITE,                /**< 25: Update the FWCT row */
    UVDM_CMD_CUSTOM = 30                    /**< Custom UVDM implementation for users.
                                                 Needs the solution override option to be enabled. */
} uvdm_cmd_opcode_t;

/**
 * @typedef uvdm_response_state_t
 * @brief List of possible states of UVDM Response.
 */
typedef enum
{
    UVDM_NOT_HANDLED,                       /**< UVDM not recognised, can't be handled. */
    UVDM_HANDLED_RESPONSE_READY,            /**< UVDM handled and response is ready to be sent. */
    UVDM_HANDLED_NO_RESPONSE,               /**< UVDM handled but no response required. */
    UVDM_HANDLED_RESPONSE_NOT_READY         /**< UVDM Handled but response will be sent later, potential
                                                 non-blocking command. */
} uvdm_response_state_t;

/**
 * @typedef uvdm_qc_pps_cmd_t
 * @brief List of opcodes of QC 5.0/4.0 UVDM Commands.
 */
typedef enum
{
    UVDM_QC_GET_CASE_TEMP = 0x1003,         /**< QC 5.0/4.0 Get Case Temperature opcode. */
    UVDM_QC_GET_CONNECTOR_TEMP = 0x0B03,    /**< QC 5.0/4.0 Get Connector Temperature opcode. */
    UVDM_QC_GET_CONNECTOR_VOLT = 0x0603,    /**< QC 5.0/4.0 Get Connector Voltage opcode. */
    UVDM_QC_GET_CHARGER_TYPE = 0x0C03,      /**< QC 5.0/4.0 Get Charger Type opcode. */
    UVDM_QC_GET_CHARGER_VERSION = 0x0E03    /**< QC 5.0/4.0 Get Charger Version opcode. */
} uvdm_qc_pps_cmd_t;

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Returns current non blocking command opcode.
 * This function returns the opcode of current Non Blocking UVDM Command
 * which can be used to form the response VDM.
 *
 * @return uvdm_cmd_opcode_t UVDM Command Opcode
 */
uvdm_cmd_opcode_t uvdm_get_cur_nb_cmd(void);

/**
 * @brief Resets internal state and trackers related to Non Blocking flash write
 * operation at the end of write operation.
 *
 * @return None
 */
void uvdm_reset_nb_cmd_state(void);

/**
 * @brief CY Flashing Mode Entry Handler
 * This function is called when CCG enters CY Flashing Alternate mode.
 * @return None
 */
void uvdm_enter_cy_alt_mode(void);

/**
 * @brief CY Flashing Mode Exit Handler
 * This function is called when CCG exits CY Flashing Alternate mode.
 * @return None
 */
void uvdm_exit_cy_alt_mode(void);

/**
 * @brief Return CY Flashing Mode Status
 * This function returns the current state of CY Flashing Alternate Mode.
 * @return true if CY Flashing Mode active, false otherwise.
 */
bool uvdm_get_flashing_mode(void);

/**
 * @brief Handle Device Reset Command
 * This function handles device Reset command. Device Reset handling is
 * application specific and hence this routine shall be implemented at Solution
 * level.
 *
 * @param reset_sig Type of Reset: Device Reset, Jump to bootloader or Jump to Alt image
 * @return cy_en_pdstack_status_t
 */
cy_en_pdstack_status_t uvdm_handle_device_reset(uint32_t reset_sig);

/**
 * @brief CY Flashing Alternate Mode UVDM Command Handler.
 * @param context Pointer to pdstack
 * @param rx_pkt Pointer to VDM Command
 * @param vdm_rspn_pkt Pointer to the VDM response packet
 * @param vdo_count Number of VDOs in VDM response, 0 if no VDM Response
 * @param flash_cb Callback function for Non Blocking Flash Write
 *
 * @return uvdm_response_state_t
 */
uvdm_response_state_t uvdm_handle_cmd(cy_stc_pdstack_context_t * context, uint32_t *rx_pkt, cy_pd_pd_do_t **vdm_rspn_pkt,
    uint8_t *vdo_count, flash_cbk_t flash_cb);

/**
 * @brief Handles all QC 5.0/4.0 Protocol UVDM commands.
 *
 * @param ptrPdStackContext PD stack context
 * @param rx_pkt Pointer to VDM Command
 * @param vdm_rspn_pkt Pointer to the VDM response packet
 * @param vdo_count Number of VDOs in VDM response, 0 if no VDM Response
 *
 * @return uvdm_response_state_t
 */
uvdm_response_state_t uvdm_qc_pps_handler(cy_stc_pdstack_context_t *ptrPdStackContext, uint32_t *rx_pkt,
    cy_pd_pd_do_t **vdm_rspn_pkt, uint8_t *vdo_count);

/**
 * @brief Flashing Alternate Mode UVDM Command solution space extension handler.
 * This function is invoked by the flashing UVDM handler if the 
 * SLN_FLASHING_UVDM_HANDLER_ENABLE is enabled and the standard handler is not
 * able to resolve.
 *
 * NOTE: Since the function is for extending avaiable solution in the standard
 * handler, the data structures are re-used.
 *
 * @param rx_pkt Pointer to VDM Command
 * @param vdm_rspn_pkt Pointer to the VDM response packet (allocation done in uvdm.c)
 * @param vdo_count Number of VDOs in VDM response, 0 if no VDM Response
 * @param response_code Response status code to be returned in the UVDM response
 *
 * @return uvdm_response_state_t
 */
uvdm_response_state_t sln_flashing_uvdm_handler(uint32_t *rx_pkt, cy_pd_pd_do_t *vdm_rspn_pkt,
    uint8_t *vdo_count, cy_en_pdstack_status_t *response_code);

#endif /* UVDM_H_ */

/** \} group_ccgxAppCommon */

/* [] END OF FILE */
