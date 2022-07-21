/******************************************************************************
* File Name: boot.h
*
* Description: Source file for boot functions
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon Common source files
* \{
*/
#ifndef BOOT_H_
#define BOOT_H_

#include "stdint.h"
#include "stdbool.h"
#include "config.h"
#include "cy_pdstack_common.h"
#include "system.h"
#include "cy_flash.h"
#include "cy_pdstack_utils.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

#define CY_PD_IMG1_FW_STATUS_BIT_MASK                   (0x08)
/**< Mask for Image-1 FW status bit in Boot mode reason byte. */

#define CCG_BOOT_MODE_RQT_SIG                           (0x424Cu)
/**< Signature used for firmware to indicate boot mode request. */

#define CCG_FW1_BOOT_RQT_SIG                            (0x4231u)
/**< Signature used to indicate boot FW1 request. */

#define CCG_FW2_BOOT_RQT_SIG                            (0x4232u)
/**< Signature used to indicate boot FW2 request. */

#define CCG_FW_METADATA_BOOTSEQ_OFFSET                  (0x1Cu)
/**< Firmware boot sequence number offset. */

#define CCG_BL_WAIT_DEFAULT                             (50u)
/**< Default boot-wait window for CCGx boot-loader: 50 ms */

#define CCG_BL_WAIT_MINIMUM                             (20u)
/**< Minimum boot-wait window duration supported: 20 ms */

#define CCG_BL_WAIT_MAXIMUM                             (1000u)
/**< Maximum boot-wait window duration supported: 1000 ms */

#define CCG_FWMETA_APPID_WAIT_DEF                       (0xFFFFu)
/**< FW metadata application ID value requesting default boot-wait window. */

#define CCG_FWMETA_APPID_WAIT_0                         (0x4946u)
/**< FW metadata application ID value requesting a zero boot-wait window. */

#define CONFIGTABLE_SIGNATURE           (0x4946u)        /**< Signature used to validate config table content */

#define CONFIGTABLE_SIZE_OFFSET         (6u)             /**< Offset to table size field in config table. */
#define CONFIGTABLE_CHECKSUM_OFFSET     (8u)             /**< Offset to checksum field in config table. */
#define CONFIGTABLE_CHECKSUM_START      (12u)            /**< Offset at which table checksum calculation starts. */

/*****************************************************************************
* Data Struct Definition
*****************************************************************************/

/**
 * @brief Boot mode reason structure.
 *
 * This structure holds status of FW images and boot mode request.
 * If the CCGx device is running in Boot-loader mode, this register can be used
 * to identify the reason for this. The register will report the validity of
 * FW1 and FW2 binaries even in the case where the device is already running
 * in FW1 or FW2 mode.
 */
typedef union
{
    uint8_t val;                            /**< Integer field used for direct manipulation of reason code. */

    struct fw_mode_reason_t                 /**< Structure containing boot reason status bits. */
    {
        uint8_t boot_mode_request : 1;      /**< Boot mode request made by FW. */
        uint8_t reserved          : 1;      /**< Reserved field: Will be zero. */
        uint8_t fw1_invalid       : 1;      /**< FW1 image invalid: 0=Valid, 1=Invalid. */
        uint8_t fw2_invalid       : 1;      /**< FW2 image invalid: 0=Valid, 1=Invalid. */
        uint8_t reserved1         : 4;      /**< Reserved for later use. */
    } status;                               /**< Struct containing the status fields in the boot mode reason value. */

} fw_img_status_t;

/**
 * @brief CCGx Firmware metadata structure
 *
 * This structure defines the format of the firmware metadata that is stored
 * on device flash. The boot-loader uses the metadata to identify the firmware validity.
 * location, size, start address etc. The metadata for the two runtime firmware images
 * (FW1 and FW2) are located at fixed addresses (for each CCGx part), allowing the boot-loader
 * to precisely locate and validate the flash content during boot-up.
 */
#if USE_CYACD2_METADATA_FORMAT
typedef struct __attribute__((__packed__))
{
    uint32_t fw_start;                  /**< Offset 00: App Fw Start Address. */
    uint32_t fw_size;                   /**< Offset 04: App Fw Size */
    uint16_t boot_app_id;               /**< Offset 08: Boot wait time */
    uint16_t reserved1;                 /**< Offset 0A: Reserved */
    uint32_t config_fw_start;           /**< Offset 0C: Verify Start Address. */
    uint32_t config_fw_size;            /**< Offset 10: Verify Size */
    uint32_t reserved2[16];             /**< Offset 14: Reserved. */
    uint16_t metadata_version;          /**< Offset 54: Version of the metadata structure. */
    uint16_t metadata_valid;            /**< Offset 56: Metadata Valid field. Valid if contains "IF". */
    uint32_t fw_crc32;                  /**< Offset 58: App Fw CRC32 checksum */
    uint32_t reserved3[8];              /**< Offset 5C: Reserved. */
    uint32_t md_crc32;                  /**< Offset 7C: Metadata CRC32 checksum. */
} sys_fw_metadata_t;
#else
typedef struct __attribute__((__packed__))
{
    uint8_t fw_checksum;                  /**< Offset 00: Single Byte FW Checksum. */
    uint32_t fw_entry;                    /**< Offset 01: FW Entry Address */
    uint16_t boot_last_row;               /**< Offset 05: Last Flash row of Bootloader or previous firmware. */
    uint8_t reserved1[2];                 /**< Offset 07: Reserved. */
    uint32_t fw_size;                     /**< Offset 09: Size of Firmware. */
    uint8_t reserved2[3];                 /**< Offset 0D: Reserved. */
    uint8_t active_boot_app;              /**< Offset 10: Creator specific field. Not used in this implementation. */
    uint8_t boot_app_ver_status;          /**< Offset 11: Creator specific field. Not used in this implementation. */
    uint16_t boot_app_version;            /**< Offset 12: Creator specific field. Not used in this implementation. */
    uint16_t boot_app_id;                 /**< Offset 14: Creator specific field. Not used in this implementation. */
    uint16_t metadata_valid;              /**< Offset 16: Metadata Valid field. Valid if contains "CY". */
    uint32_t fw_version;                  /**< Offset 18: Creator specific field. Not used in this implementation. */
    uint32_t boot_seq;                    /**< Offset 1C: Boot sequence number field. Boot-loader will load the valid
                                               FW copy that has the higher sequence number associated with it. */
} sys_fw_metadata_t;
#endif /* USE_CYACD2_METADATA_FORMAT */


/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/

extern sys_fw_metadata_t *gl_img1_fw_metadata;  /**< Pointer to metadata associated with the Image-1 FW binary. */

#if (!CCG_DUALAPP_DISABLE)
/**
 *  @brief Pointer to metadata associated with the Image-2 FW binary.
 */
extern sys_fw_metadata_t *gl_img2_fw_metadata;  /**< Pointer to metadata associated with the Image-2 FW binary. */
#endif /* (!CCG_DUALAPP_DISABLE) */

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

extern sys_fw_metadata_t *gl_img1_fw_pseudo_metadata;   /**< Pointer to pseudo metadata associated with the
                                                             Image-1 FW binary. */

extern sys_fw_metadata_t *gl_img2_fw_pseudo_metadata;   /**< Pointer to pseudo metadata associated with the
                                                             Image-2 FW binary. */

#endif /* CCG_PSEUDO_METADATA_DISABLE */

extern fw_img_status_t gl_img_status;                   /**< Current firmware image status. */

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Validate the configuration table specified.
 *
 * Each copy of CCGx firmware on the device flash contains an embedded
 * configuration table that defines the runtime behaviour of the CCGx device. This
 * function checks whether the configuration table located at the specified location
 * is valid (has valid signature and checksum).
 *
 * @param table_p Pointer to the configuration table to be validated.
 *
 * @return CCG_STAT_SUCCESS if the table is valid, CCG_STAT_FAILURE otherwise.
 */
cy_en_pdstack_status_t boot_validate_configtable(uint8_t *table_p);

/**
 * @brief Validate the firmware image associated with the given metadata.
 *
 * This function validates the firmware binary associated with the
 * metadata specified in the fw_metadata parameter. The validity check includes
 * checks for signature, location, size and checksum. This function internally
 * performs validation of the embedded configuration table using the
 * boot_validate_configtable function.
 *
 * @param fw_metadata Pointer to metadata table of the FW which has to be validated.

 * @return CCG_STAT_SUCCESS if the firmware is valid, CCG_STAT_FAILURE otherwise.
 */
cy_en_pdstack_status_t boot_validate_fw(sys_fw_metadata_t *fw_metadata);

/**
 * @brief Handles the VALIDATE_FW command from HPI or UVDM.
 *
 * This API handles the VALIDATE_FW command received through
 * the HPI or UVDM interfaces.
 *
 * @param fw_mode Firmware binary id: 1 for FW1 and 2 for FW2.

 * @return Status code indicating the validity of the firmware.
 */
cy_en_pdstack_status_t boot_handle_validate_fw_cmd(sys_fw_mode_t fw_mode);

/**
 * @brief Returns the boot-wait delay configured for the application.
 *
 * This function identifies the boot-wait delay required by checking
 * the firmware metadata.
 *
 * @return Boot-wait delay in milliseconds.
 */
uint16_t boot_get_wait_time(void);

/**
 * @brief Identify the firmware binary to be loaded.
 *
 * This function is only used in the CCGx boot-loader, and
 * implements the main start-up logic of the boot-loader. The function
 * validates the two firmware binaries in device flash, and identifies
 * the binary to be loaded. If neither binary is valid, the function returns
 * false notifying the caller to continue in boot-loader mode.
 *
 * @return true if firmware load is allowed, false otherwise.
 */
bool boot_start(void);

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

/**
 * @brief Check for the presence of alternate firmware waiting to be validated.
 *
 * This function checks whether the CCGx device flash contains an alternate firmware
 * binary which is waiting to be validated. This can happen if a firmware update happened
 * during the last power up of the device, and the binary is yet to be validated and made
 * active. The active firmware will make use of the pseudo metadata in flash to identify
 * the alternate firmware, validate it and activate it by updating the actual firmware
 * metadata.
 *
 * Please refer to the CCGx boot sequence description in the firmware guide for a more
 * detailed description of the boot procedure.
 *
 * @return NONE
 */
void boot_check_for_valid_fw(void);

#endif /* CCG_PSEUDO_METADATA_DISABLE */

/**
 * @brief Validate a alternate firmware according to IECS SFWv command
 *
 * This function validate alternate firmware binary. EC update flash rows 
 * step by step without the first row and after that send SFWv command 
 * to validate FW signature. if a firmware signature is correct function
 * writes a first row of a image to flash.
 *
 *@param first_row Poineter to the first row of FW image.
 *
 * @return NONE
 */

bool iecs_check_for_valid_fw(uint8_t *first_row);

/**
 * @brief Returns a bitmap containing the reason for boot mode.
 *
 * This function returns the bitmap value that is to be stored in the
 * BOOT_MODE_REASON HPI register, which identifies the validity of the
 * two firmware binaries. The validation of the firmware is expected to
 * have been completed earlier through the boot_start function. This
 * function only retrieves the status stored during the validation procedure.
 *
 * @see fw_img_status_t
 *
 * @return Boot mode reason bitmap.
 */
fw_img_status_t get_boot_mode_reason(void);

/**
 * @brief Transfer control to the firmware binary identified by boot_start.
 *
 * This function is only used by the CCGx boot-loader. This transfers control to
 * the firmware binary selected as the boot target by the boot_start function.
 * This is expected to be called after the boot-wait window has elapsed.
 *
 * @return None
 */
void boot_jump_to_fw(void);

/**
 * @brief Get the boot sequence number value for the specified firmware image.
 *
 * A boot sequence number field stored in the firmware metadata is used by the
 * CCGx boot-loader to identify the firmware binary to be loaded. This function
 * retrieves the sequence number associated with the specified firmware binary.
 *
 * @param fwid Firmware id whose sequence number is to be retrieved. 1 for FW1
 * and 2 for FW2.
 *
 * @return Boot sequence number value if the firmware is valid, 0 otherwise.
 */
uint32_t boot_get_boot_seq(uint8_t fwid);

/**
 * @brief Function to validate firmware images and update image status.
 *
 * This function is used to validate the firmware images in the CCG device flash
 * and update their status in the image status / boot-mode reason field.
 *
 * @return None
 */
void boot_update_fw_status(void);

/**
 * @brief Function to jump from bootloader to application.
 *
 * This function is used to validate the firmware images in the CCG device flash
 * and update their status in the image status / boot-mode reason field.
 *
 * @param appId application Id
 *
 * @return None
 */
void boot_jump_to_app(uint32_t appId);

#endif /* BOOT_H_ */

/** \} group_ccgxAppCommon */

/* [] END OF FILE */
