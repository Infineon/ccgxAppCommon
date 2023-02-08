/******************************************************************************
* File Name:   cy_pdaltmode_billboard.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the USB Billboard.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_BILLBOARD_H
#define CY_PDALTMODE_BILLBOARD_H

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "cy_pdaltmode_defines.h"
#include "cy_pdaltmode_mngr.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

/**
 * @brief Maximum number of alternate modes supported by the billboard module.
 */
#define BB_MAX_ALT_MODES                (8u)

/**
 * @brief Initialization value for mode status register.
*/
#define BB_ALT_MODE_STATUS_INIT_VAL     (0x5555u)

/**
 * @brief Alternate mode status mask for each mode.
 */
#define BB_ALT_MODE_STATUS_MASK         (0x03u)

/**
 * @brief Billboard OFF timer maximum interval. One second is used to ensure
 * that timer module goes through only one interrupt per request.
 */
#define BB_OFF_TIMER_MAX_INTERVAL       (1000u)

/**
 * @brief Timeout special value for not disabling the billboard interface.
 */
#define BB_OFF_TIMER_NO_DISABLE         (0xFFFFu)

/**
 * @brief Minimum timeout value in allowed in seconds = 60s.
 */
#define BB_OFF_TIMER_MIN_VALUE          (60u)

/**
 * @brief Maximum buffering for EP0 transactions inside the billboard module.
 *
 * This definition is valid only for internal billboard implementation.
 */
#define BB_MAX_EP0_XFER_SIZE            (256u)

/**
 * @brief Additional failure information due to lack of power as per the
 * billboard capability descriptor.
 */
#define BB_CAP_ADD_FAILURE_INFO_PWR     (0x01u)

/**
 * @brief Additional failure information due to USBPD communication failure
 * as per billboard capability descriptor.
 */
#define BB_CAP_ADD_FAILURE_INFO_PD      (0x02u)

/**
 * @brief Disconnect BB Status for BB connect event
 */
#define BB_DISCONNECT_STAT              (0xFFu)

/**
 * @brief Connect BB Status for BB connect event
 */
#define BB_CONNECT_STAT                 (0x01u)

/**
 * @brief Maximum number of alternate modes supported.
 */
#define BB_MAX_SVID                     (2u)

/** Bit field in bb_option from configuration table which indicates
 * that the device needs to enable Vendor interface along with billboard.
 */
#define BB_OPTION_VENDOR_ENABLE          (1 << 0)

/** Bit field in bb_option from configuration table which indicates
 * that the device needs to enable Vendor interface along with billboard.
 */
#define BB_OPTION_BILLBOARD_ENABLE       (1 << 1)

/** Bit field in bb_option from configuration table which indicates
 * that the device needs to enable HID interface along with billboard.
 */
#define BB_OPTION_HID_ENABLE            (0x01)

/** Number of alternate modes field offset in the BOS descriptor. */
#define USB_BOS_DSCR_NUM_ALT_MODE_OFFSET (36)

/** Offset for alternate mode status field offset in the BOS descriptor. */
#define USB_BOS_DSCR_ALT_STATUS_OFFSET  (40)

/** Offset for alternate mode bAdditionalFailureInfo field in the BOS descriptor. */
#define USB_BOS_DSCR_ADD_INFO_OFFSET    (74)

/** Offset for alternate mode 0 information offset in the BOS descriptor. */
#define USB_BOS_DSCR_MODE0_INFO_OFFSET  (76)

/** Discover ID response minimum size. */
#define VDM_DID_MIN_SIZE                (20)

/** Discover ID response ID header VDO offset. */
#define VDM_RESP_ID_HEADER_OFFSET       (8)

/** \} group_pdaltmode_macros */


/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
* \addtogroup group_pdaltmode_enums
* \{
*/
/**
 * @brief USB Billboard status enumeration.
 *
 * The enumeration for the billboard status codes.
 */
typedef enum
{
    BB_STAT_SUCCESS = 0,       /**< Success status */
    BB_STAT_FAILURE,           /**< Failure status */
    BB_STAT_BUSY,              /**< Status code indicating Billboard is busy */
    BB_STAT_NOT_READY,         /**< Billboard not ready */
    BB_STAT_INVALID_ARGUMENT,  /**< Invalid parameter */
    BB_STAT_NOT_SUPPORTED,     /**< Operation not supported */
} bb_status_t;


/**
 * @brief USB Billboard cause for enumeration.
 *
 * The enumeration lists all the supported causes for billboard enumeration.
 */
typedef enum
{
    BB_CAUSE_AME_TIMEOUT,       /**< AME timeout event */
    BB_CAUSE_AME_SUCCESS,       /**< Successful alternate mode entry */
    BB_CAUSE_AME_FAILURE,       /**< Failed alternate mode entry */
    BB_CAUSE_PWR_FAILURE        /**< Failed to get sufficient power */

} bb_cause_t;

/**
 * @brief USB Billboard alternate mode status.
 *
 * The enumeration lists the different status values as defined by
 * the billboard class specification.
 */
typedef enum
{
    BB_ALT_MODE_STAT_ERROR,             /**< Undefined error / alternate mode does not exist */
    BB_ALT_MODE_STAT_NOT_ATTEMPTED,     /**< Alternate mode entry not attempted */
    BB_ALT_MODE_STAT_UNSUCCESSFUL,      /**< Alternate mode entry attempted but failed */
    BB_ALT_MODE_STAT_SUCCESSFUL         /**< Alternate mode entry succeeded */

} bb_alt_mode_status_t;

/**
 * @brief USB Billboard string descriptor indices.
 *
 * The enumeration lists the different string indices used in the billboard
 * implementation.
 */
typedef enum
{
    BB_LANG_ID_STRING_INDEX,            /**< Language ID index */
    BB_MFG_STRING_INDEX,                /**< Manufacturer string index */
    BB_PROD_STRING_INDEX,               /**< Product string index */
    BB_SERIAL_STRING_INDEX,             /**< Serial string index */
    BB_CONFIG_STRING_INDEX,             /**< Configuration string index */
    BB_BB_INF_STRING_INDEX,             /**< Billboard interface string index */
    BB_HID_INF_STRING_INDEX,            /**< HID interface string index */
    BB_URL_STRING_INDEX,                /**< Additional info URL string index */
    BB_ALT_MODE1_STRING_INDEX,          /**< Alternate mode 1 string index */
    BB_ALT_MODE2_STRING_INDEX,          /**< Alternate mode 2 string index */
    BB_ALT_MODE3_STRING_INDEX,          /**< Alternate mode 3 string index */
    BB_ALT_MODE4_STRING_INDEX,          /**< Alternate mode 4 string index */
    BB_ALT_MODE5_STRING_INDEX,          /**< Alternate mode 5 string index */
    BB_ALT_MODE6_STRING_INDEX,          /**< Alternate mode 6 string index */
    BB_ALT_MODE7_STRING_INDEX,          /**< Alternate mode 7 string index */
    BB_ALT_MODE8_STRING_INDEX           /**< Alternate mode 8 string index */

} bb_usb_string_index_t;

/**
 * @brief Alternate Mode information to be used in BOS descriptor.
 */
typedef struct
{
    uint16_t wSvid;                     /**< SVID for the alternate mode. */
    uint8_t  bAltMode;                  /**< Index of the alternate mode. */
    uint8_t  iAltModeString;            /**< String index for the alternate mode. */
    uint32_t dwAltModeVDO;              /**< Contents of Mode VDO for the alternate mode. */
} bb_am_info_t;

/**
 * @brief Complete BOS descriptor information to be used by Billboard device.
 */
typedef struct
{
    uint8_t enable;                     /**< Whether Billboard interface is to be enabled. */
    uint8_t nSvid;                      /**< Number of SVIDs supported. */
    uint8_t prefMode;                   /**< Preferred alternate mode index. */
    uint8_t reserved;                   /**< Reserved field. */

    bb_am_info_t modelist[BB_MAX_SVID]; /**< List of alternate mode information. */
} bb_conn_info_t;
/** \} group_pdaltmode_enums */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
* \ addtogroup group_pdaltmode_functions
* \{
*/

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: bb_enable
****************************************************************************//**
*
*  The function queues a billboard enumeration / re-enumeration.
*
* The API queues the billboard enumeration as per the configuration information
* provided. Enumeration details are retrieved from the configuration table.
*
* The function can be called multiple times applications to trigger a
* re-enumeration without explicit disable call.
*
* For internal implementation of billboard, the USB module is controlled from
* the bb_task() and this function only queues the request. It should be noted
* that only one pending request is honored. If more than one request is
* queued, only the latest is handled. This request is failed if the flashing
* mode is in progress.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \param cause
* Cause for billboard enumeration.
*
* \return
* Status of the call.
*
*******************************************************************************/
cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Enable(cy_stc_pdaltmode_context_t *ptrAltModeContext, bb_cause_t cause);

/**
 * @brief The function updates the alternate mode status.
 *
 * The function updates the alternate mode status information for the specified
 * alternate mode index. The alternate mode index should match the DISCOVER_SVID
 * and DISCOVER_MODES response for the device.
 *
 * @param ptrAltModeContext
 * Pointer to the alt mode context
 *
 * @param mode_index
 * Index of the mode as defined by the alternate mode manager.
 *
 * @param alt_status
 * Current alternate mode status.
 *
 * @return Status of the call.
 */
cy_en_pdstack_status_t Cy_PdAltMode_Billboard_UpdateAltStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t mode_index,
        bb_alt_mode_status_t alt_status);
/**
 * @brief The function updates the alternate mode status for all modes.
 *
 * The current billboard implementation supports a maximum of 8 alternate modes
 * and the each modes as defined in the order of BOS descriptor has two bit
 * status. Bit 1:0 indicates status of alt mode 0, Bit 3:2 indicates status of
 * alt mode 1 etc. This function should be only used when the billboard status
 * needs to be re-initialized to a specific value. In individual entry / exit
 * cases, Cy_PdAltMode_Billboard_Enable_UpdateAltStatus() should be used.
 *
 * @param ptrAltModeContext
 * Pointer to the alt mode context
 *
 * @param status
 * Status data for all alternate modes.
 *
 * @return Status of the call.
 */
cy_en_pdstack_status_t Cy_PdAltMode_Billboard_UpdateAllStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status);

/**
 * @brief The function queues a billboard interface disable.
 *
 * The API disables the billboard device and disconnects the terminations.
 * For internal implementation of billboard, the USB module is controlled
 * from the bb_task() and this function only queues the request. It should be
 * noted that only one pending request is honored. If more than one request is
 * queued, only the latest is handled. A disable call clears any pending enable.
 *
 * @param ptrAltModeContext
 * Pointer to the alt mode context
 *
 * @param force
 * Whether to force a disable. false = Interface not disabled when
 * in flashing mode. true = Interface disabled regardless of the operation mode.
 *
 * @return Status of the call.
 */
cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Disable(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool force);

/**
 * @brief Checks whether a billboard interface is present.
 *
 * The function checks the configuration information and identifies if a billboard
 * device exists.
 *
 * @param ptrAltModeContext
 * Pointer to the alt mode context
 *
 * @return Returns true if billboard is present and false if absent.
 */
bool Cy_PdAltMode_Billboard_IsPresent(cy_stc_pdaltmode_context_t *ptrAltModeContext);

bool Cy_PdAltMode_Billboard_IsEnabled(cy_stc_pdaltmode_context_t *ptrAltModeContext);

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_FlashingCtrl(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool enable);

bool Cy_PdAltMode_Billboard_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext);

bool Cy_PdAltMode_Billboard_EnterDeepSleep(cy_stc_pdaltmode_context_t *ptrAltModeContext);

void Cy_PdAltMode_Billboard_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext);

void Cy_PdAltMode_Billboard_UpdateSelfPwrStatus (cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t self_pwrd);

uint8_t *Cy_PdAltMode_Billboard_GetVersion(void);

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_BindToPort (cy_stc_pdaltmode_context_t *ptrAltModeContext);



/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_BILLBOARD_H */

/* [] END OF FILE */
