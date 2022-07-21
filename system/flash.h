/******************************************************************************
* File Name: flash.h
*
* Description: Flash handler header file
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

#ifndef _FLASH_H_
#define _FLASH_H_

#include "stdint.h"
#include "stdbool.h"
#include "config.h"
#include "cy_pdstack_common.h"
#include "cy_flash.h"

/******************************************************************************
 * Constant definitions.
 *****************************************************************************/

#define SROM_API_PARAM_SIZE             (8u)    /**< Size of flash write SROM API parameters in bytes. */

/** CCGx FLASH OPTIONS **/
/* Only the boot-loader last row and the configuration table size is expected
 * to change to match the project requirement. With a fixed boot-loader, none
 * of this fields should be modified.
 */

/* Total size of flash. */
#define CCG_FLASH_SIZE                          (CY_FLASH_SIZE)

/** @cond DOXYGEN_HIDE */
/* Last row number of flash. */
#define CCG_LAST_FLASH_ROW_NUM                  (CY_FLASH_NUMBER_ROWS - 1u)

/* Flash row size. This depends on the device type. */
#define CCG_FLASH_ROW_SIZE                      (CY_FLASH_SIZEOF_ROW)

/* Shift value used for multiplying row number with flash row size to
 * get the actual flash address. This depends on the flash row size. Update this
 * field accordingly. */
#if (CCG_FLASH_ROW_SIZE == 128)
#define CCG_FLASH_ROW_SHIFT_NUM                 (7u)
#elif (CCG_FLASH_ROW_SIZE == 256)
#define CCG_FLASH_ROW_SHIFT_NUM                 (8u)
#else
#error "Selected device has unsupported flash row size."
#endif /* CY_FLASH_SIZEOF_ROW */

#if USE_CYACD2_METADATA_FORMAT
#define CCG_METADATA_TABLE_SIZE                 (0x80u)
#else
/*FW Metadata Table size in bytes. */
#define CCG_METADATA_TABLE_SIZE                 (0x40u)
#endif /* USE_CYACD2_METADATA_FORMAT */


/* FW Image-1 metadata row number and address. */
#define CCG_IMG1_METADATA_ROW_NUM               (CCG_LAST_FLASH_ROW_NUM)
#define CCG_IMG1_FW_METADATA_ADDR               (((CCG_IMG1_METADATA_ROW_NUM + 1u) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* FW Image-2 metadata row number and address. */
#define CCG_IMG2_METADATA_ROW_NUM               (CCG_LAST_FLASH_ROW_NUM - 1u)
#define CCG_IMG2_FW_METADATA_ADDR               (((CCG_IMG2_METADATA_ROW_NUM + 1u) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* FW Image-2 pseudo-metadata row number and address. */
#define CCG_IMG2_PSEUDO_METADATA_ROW_NUM        (CCG_LAST_FLASH_ROW_NUM - 2u)
#define CCG_IMG2_FW_PSEUDO_METADATA_ADDR        (((CCG_IMG2_PSEUDO_METADATA_ROW_NUM + 1u) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* Black box address and row number */
#define CCG_BLACK_BOX_ROW_NUM                   (CCG_IMG2_METADATA_ROW_NUM - 1u)
#define CCG_BLACK_BOX_ADDR                      (CCG_BLACK_BOX_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM)

/* Last Boot loader row. This field should be changed only when there is a
 * project level change. A change to this field should be synchronous with
 * the following fields:
 *   -> boot-loader cm0gcc.ld file configSection location:
 *      = ((CY_PD_BOOT_LOADER_LAST_ROW + 1) << CY_PD_FLASH_ROW_SHIFT_NUM)
 *   -> firmware boot-loadable component firmware start location:
 *      = (CY_PD_CONFIG_TABLE_ADDRESS + CY_PD_CONFIG_TABLE_SIZE) */
#if ((AUTH_BOOT || CY_FW_USES_AUTH_BOOT) && defined(CY_DEVICE_CCG7S))
#define CCG_BOOT_LOADER_LAST_ROW                (0x3Fu)
#else
#define CCG_BOOT_LOADER_LAST_ROW                (0x2Fu)
#endif /* (AUTH_BOOT || CY_FW_USES_AUTH_BOOT) && defined(CY_DEVICE_CCG7S) */

/* Config table address boundary used to distinguish between FW1 and FW2. */
#define CCG_FW1_CONFTABLE_MAX_ADDR              (0x10000u)

/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/

/**
 * @typedef flash_app_priority_t
 * @brief Enumeration of app priority values.
 *
 * App priority is a debug feature that allows the CCG bootloader to prioritize one
 * copy of the firmware over the other. The default behaviour is for both firmware
 * binaries to have the same priority so that the more recently updated firmware
 * binary gets loaded. The priority scheme can be updated to allow FW1 or FW2 to
 * always be loaded for debugging purposes.
 */
typedef enum
{
    FLASH_APP_PRIORITY_DEFAULT = 0,      /**< Default. Latest image gets priority. */
    FLASH_APP_PRIORITY_IMAGE_1,          /**< Image-1 gets priority over image-2. */
    FLASH_APP_PRIORITY_IMAGE_2           /**< Image-2 gets priority over image-1. */
} flash_app_priority_t;

/**
 * @typedef flash_interface_t
 * @brief List of supported flash update interfaces.
 *
 * CCG supports multiple flash update interfaces such as HPI, UVDM, USB etc.
 * depending on the application. This enumerated type lists the supported flash
 * update interfaces.
 */
typedef enum
{
    FLASH_IF_HPI = 0,                   /**< HPI based flash update interface. */
    FLASH_IF_UVDM,                      /**< UVDM based flash update interface. */
    FLASH_IF_USB_HID,                   /**< USB flash update interface. */
    FLASH_IF_IECS_UART,                 /**< IECS (UART) flash update interface. */
    FLASH_IF_IECS_HPI,                 /**< IECS (I2C) flash update interface. */
} flash_interface_t;

/**
 * @typedef flash_write_status_t
 * @brief List of possible status codes for non blocking flash write operation.
 */
typedef enum
{
    FLASH_WRITE_COMPLETE,               /**< Flash Write successfully completed. */
    FLASH_WRITE_ABORTED,                /**< Flash Write aborted. */
    FLASH_WRITE_COMPLETE_AND_ABORTED,   /**< Flash Write completed with an abort request. */
    FLASH_WRITE_IN_PROGRESS             /**< Flash Write is active. */
} flash_write_status_t;

/*****************************************************************************
* Data Struct Definition
*****************************************************************************/

/**
 * @typedef flash_cbk_t
 * @brief Non-blocking flash write row callback function type.
 *
 * The CCG3 device supports non-blocking flash update operations, so that the
 * firmware can perform other operations while a flash row write is in progress.
 * The completion of the flash row write is notified through a callback function.
 * This type represents the function type which can be registered as a callback
 * for notification of non-blocking flash operations.
 */
typedef void (*flash_cbk_t)(flash_write_status_t status);

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Handle ENTER_FLASHING_MODE Command
 *
 * This function notifies the CCG stack that flash read/write is being enabled
 * by the application. By default, CCG firmware disallows all flash read/write
 * operations. Flash access is only allowed after flashing mode has been
 * explicitly enabled through user command.
 *
 * Note: FW update interface is allowed to Enable Flashing mode only once in one
 * session. This is to ensure that multiple flashing interfaces are not active
 * simultaneously. Each FW update interface is expected to take care of this.
 * Once Flash access is complete, this API should be used to exit FW Update
 * interface.
 *
 * @param is_enable Enable/Disable Flashing Mode
 * @param mode Flash update interface to be used.
 * @param data_in_place Specifies whether the flash write data buffer can be
 * used in place as SROM API parameter buffer.
 *
 * @return None
 */
void flash_enter_mode(bool is_enable, flash_interface_t mode, bool data_in_place);

/**
 * @brief Check whether flashing mode has been entered.
 *
 * This function checks whether flashing mode has currently been entered by a
 * different interface.
 *
 * @param modes Bitmap containing flashing interfaces to be checked.
 *
 * @return Returns true if flashing mode has been entered using any of the
 * interfaces listed in modes, false otherwise.
 */
bool flash_access_get_status(uint8_t modes);

/**
 * @brief Set limits to the flash rows that can be accessed.
 *
 * The CCG stack has been designed to support fail-safe firmware upgrades using
 * a pair of firmware binaries that can mutually update each other. This scheme
 * can only be robust if the currently active firmware binary can effectively
 * protect itself by not allowing access to any of its own flash rows.
 *
 * This function is used to specify the list of flash rows that can safely be
 * accessed by the currently active firmware binary. This function should only
 * be used with parameters derived based on the binary locations identified from
 * firmware metadata. Incorrect usage of this API can cause the device to stop
 * responding during a flash update operation.
 *
 * This API must be invoked as part of intialization before the flash_row_write()
 * and flash_row_read() functions can be called. By default, no flash row can be
 * read or written to.
 *
 * @param start_row The lowest row number that can be written.
 * @param last_row The highest row number that can be written.
 * @param md_row Row containing metadata for the alternate firmware.
 * @param bl_last_row Last bootloader row. Rows above this can be read.
 *
 * @return None
 */
void flash_set_access_limits (uint16_t start_row, uint16_t last_row, uint16_t md_row, uint16_t bl_last_row);

/**
 * @brief Extract current flash access limits
 * @param access_limit_0 Start row num for flash write access
 * @param access_limit_1 Last row num for flash write access
 * @param access_limit_2 Metadata row num 
 * @param access_limit_3 Start row num for read access
 * @return None
 */
void flash_get_access_limits (uint16_t *access_limit_0, uint16_t *access_limit_1,
        uint16_t *access_limit_2, uint16_t *access_limit_3);

#if (FLASH_ENABLE_NB_MODE == 1)
/**
 * @brief Handle Flash Write Abort request.
 *
 * This API is used by FW Update interface to request abort of ongoing non blokcing 
 * flash write request. This API sets a flag which is sampled in the next SPCIF interrupt
 * and abort sequence is started.
 *
 * @param None
 *
 * @return None
 */
void flash_non_blocking_write_abort(void);
#endif /* FLASH_ENABLE_NB_MODE */

/**
 * @brief Erase the contents of the specified flash row.
 *
 * This API erases the contents of the specified flash row by filling it with zeroes.
 * Please note that this API will only work if flashing mode is enabled and the selected
 * row is within the range of write enabled rows.
 *
 * @param row_num Row number to be erased.
 *
 * @return Status of row erase operation.
 */
cy_en_pdstack_status_t flash_row_clear(uint16_t row_num);

/**
 * @brief Write the given data to the specified flash row.
 *
 * This API handles the flash write row operation. The contents from the data buffer
 * is written to the row_num flash row. The access rules for the flash row as the
 * same as for the flash_row_clear API.
 *
 * For non-blocking write row operation, the API returns as soon as the row update
 * is started. The stack takes care of executing all of the steps across multiple
 * resume interrupts; and the callback is called at the end of the process.
 *
 * @param row_num Flash row to be updated.
 * @param data Buffer containing data to be written to the flash row.
 * @param cbk Callback function to be called at the end of non-blocking flash write.
 *
 * @return Status of the flash write. CCG_STAT_SUCCESS or appropriate error code.
 */
cy_en_pdstack_status_t flash_row_write(uint16_t row_num, uint8_t *data, flash_cbk_t cbk);

/**
 * @brief Write the given data to the specified User SFLASH row.
 *
 * This API handles writes to the user region in SFLASH of the CCG device. The function
 * ensures that SFLASH rows containing Cypress proprietary configuration and test data
 * is not cleared.
 *
 * @param row_num Flash row to be updated.
 * @param data Buffer containing data to be written to the SFLASH row.
 *
 * @return Status of the flash write. CCG_STAT_SUCCESS or appropriate error code.
 */
cy_en_pdstack_status_t sflash_row_write(uint16_t row_num, uint8_t *data);

/**
 * @brief Read the contents of the specified User SFLASH row.
 *
 * @param row_num Flash row to be read.
 * @param data Buffer to read the SFLASH row content into.
 *
 * @return Status of the flash read. CCG_STAT_SUCCESS or appropriate error code.
 */
cy_en_pdstack_status_t sflash_row_read(uint16_t row_num, uint8_t* data);

/**
 * @brief Read the contents of the specified flash row.
 *
 * This API handles the flash read row operation. The contents of the flash row are
 * copied into the specified data buffer, if flashing mode has been entered and the
 * row_num is part of the readable range of memory.
 *
 * @param row_num Flash row to be read.
 * @param data Buffer to read the flash row content into.
 *
 * @return Status of the flash read. CCG_STAT_SUCCESS or appropriate error code.
 */
cy_en_pdstack_status_t flash_row_read(uint16_t row_num, uint8_t* data);

#if (CCG_BOOT == 0)

/**
 * @brief Updates the app boot priority flag.
 *
 * This function is used to set the app priority field to override the default FW selection
 * algorithm.
 *
 * @param app_priority Desired boot priority setting.
 *
 * @return Status code of the priority update.
 */
cy_en_pdstack_status_t flash_set_app_priority(flash_app_priority_t app_priority);

#endif /*CCG_BOOT*/

/* Function pointer types used to access ROM-ed versions of SYSTEM module APIs.
 * These correspond to the various functions defined above.
 */
typedef cy_en_pdstack_status_t (*flash_row_read_fptr)(uint16_t row_num, uint8_t* data);
typedef cy_en_pdstack_status_t (*flash_row_write_fptr)(uint16_t row_num, uint8_t *data, flash_cbk_t cbk);
typedef cy_en_pdstack_status_t (*flash_row_clear_fptr)(uint16_t row_num);
typedef void (*flash_get_access_limits_fptr)(uint16_t *access_limit_0, uint16_t *access_limit_1, uint16_t *access_limit_2, uint16_t *access_limit_3);
typedef void (*flash_set_access_limits_fptr)(uint16_t start_row, uint16_t last_row, uint16_t md_row, uint16_t bl_last_row);
typedef void (*flash_enter_mode_fptr)(bool is_enable, flash_interface_t mode, bool data_in_place);
typedef bool (*flash_access_get_status_fptr)(uint8_t modes);
/** @endcond */
#endif /* FLASH_H_ */

/** \} group_ccgxAppCommon */

/* [] END OF FILE */
