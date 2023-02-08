/******************************************************************************
* File Name: fw_update.h
* \version 2.0
*
* Description: Firmware Update Header File
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon App Common Middleware
* \{
*/

#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_
#include <config.h>
#include <stdio.h>
#include <string.h>

#include "flash.h"
#include "cy_pdutils.h"
#include "system.h"  
#include "boot.h"

#if CC_BOOT
#include "cy_pdstack_common.h"
#include "uvdm.h"
#endif /* CC_BOOT */
    
#if I2C_BOOT
#include "hpi_internal.h"    
#endif /* I2C_BOOT */   

/*******************************************************************************
 * FWCT Data Structures and Definitions 
 ******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
/** FWCT Table size*/
#define CCG_FWCT_TABLE_SIZE             (104u)

/** GUID size */
#define CCG_GUID_SIZE                   (16u)

/** FWCT Signature */
#define FWCT_SIGNATURE                  (0x54435746u)
    
/** SHA-256 fw hash Size */
#define CCG_SHA256_HASH_SIZE            (32u)

/** FW version size */
#define CCG_FW_VER_SIZE                 (8u)

/** Key number offset */
#define PKEY_NUM_OFFSET                 (57u)

/** Key exponent offset */
#define PKEY_EXP_OFFSET                 (65u)

/** FWCT Signature Size */    
#define CCG_FWCT_SIG_SIZE               (256u)

/** Maximum flash write count */  
#define MAX_FLASH_WRITE_CNT             (99u)

/** Metadata Row Number of the Primary Firmware */    
#define PRIMARY_FW_MD_ROW               (0x1FFu)

/** If single image application, set this field to contain the primary image metadata row */    
#define SECONDARY_FW_MD_ROW             (PRIMARY_FW_MD_ROW)           

/** Public Key-1 Row number */
#define PKEY_1_FLASH_ROW                (0x14u)

/** Public Key-1 Row number */
#define PKEY_2_FLASH_ROW                (0x16u)
    
/** Invalid row numer is used to check against the read row number given to validate it.*/
#define FLASH_INVALID_ROW_NUMBER        (0xFFFFu)
/** \} group_ccgxAppCommon_macros */

/** \addtogroup group_ccgxAppCommon_enums
* \{
*/
/** Enum to index the array for FWCT row number.*/
typedef enum
{
    FWCT_IDX_0 = 0,
    FWCT_IDX_1 = 1,
    MAX_FWCT_IDX
}eFWCT_IDX_TYPE;

/** Structure for row number */
typedef struct
{
    uint16_t active_row_num;   /**< Active Row number */
    uint16_t inactive_row_num;  /**< Inactive row number */
}sFWCT_ROW_NO_T;

/** Enum for image types.*/
typedef enum
{
    PRIMARY_IMAGE = 0,
    MAX_IMAGE_TYPES
}eIMAGE_TYPE;

/** Enum to indicate firmware status */
typedef enum 
{
    FW_UPDATE_NONE = 0,
    FW_UPDATE_FWCT,
    FW_UPDATE_FWCT_SIG,
    FW_UPDATE_ENABLE_FLASH,
    FW_UPDATE_ROW_WRITE,
} fw_update_states_t;

/** Enum to indicate fwct validity */
typedef enum
{
    FWCT_VALID_NONE = 0, /**< No valid FWCT */
    FWCT_IDX_0_VALID,    /**< FWCT 0 is valid */
    FWCT_IDX_1_VALID,    /**< FWCT 1 is valid */
    MAX_VALID_FWCT       /**< All FWCTs are valid */
} valid_fwct_t;

/** Enum for crypto algorithm choice */
typedef enum
{
    SHA256_RSA2048 = 0,
    SHA256_uECC
}crypto_algo_t;
/** \} group_ccgxAppCommon_enums */
/** Macro to indicate type of crypto algorithm used  */
#define CRYPTO_ALGORITHM_USED           (SHA256_RSA2048)

/*******************************************************************************
 * Signed Firmware Update Function Declarations 
 ******************************************************************************/
/** @cond DOXYGEN_HIDE */
/* This function calculates the 32-bit checksum */
uint32_t ccg_calculate_dword_checksum(uint32_t *data, uint32_t size);

/* Function to compute the Firmware Hash */
void ccg_calculate_fw_hash (uint16_t row_num, uint32_t length, uint8_t output[CCG_SHA256_HASH_SIZE],
                                uint8_t *metadata);

/* Function verifies the signature of the firmware */
cy_en_pdstack_status_t secure_boot_validate_fw(sys_fw_metadata_t * fw_md, uint8_t image_t);

#if CC_BOOT
/* Signed Firmware Update State Machine */    
void signed_fw_update_fsm (uint8_t cmd_opcode, uint8_t cmd_length, uint8_t *cmd_param, cy_pd_pd_do_t *response,  cy_en_pdstack_status_t *response_code,
                            uint8_t *data, bool *is_handled);
#endif /* CC_BOOT */

#if I2C_BOOT
/* Signed Firmware Update State Machine */
void update_i2c_fsm (uint8_t cmd_opcode, uint8_t *cmd_param, uint8_t *flash_mem, cy_en_pdstack_status_t *stat,
                                hpi_response_t *code, bool *is_handled);
#endif /* I2C_BOOT */

#if I2C_BOOT   
#define VALIDATE_STATUS_t hpi_response_t
#else
#define VALIDATE_STATUS_t void        
#endif /* I2C_BOOT */
/** @endcond */
/******************************************************************************/

#endif /* FW_UPDATE_H_ */

/** \} group_ccgxAppCommon */