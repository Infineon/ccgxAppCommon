/******************************************************************************
* File Name: boot.c
* \version 2.0
*
* Description: Firmware Update Source File
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include <config.h>
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "cy_pdutils.h"
#include "system.h"
#include "fw_update.h"
#include "flash.h"
#include "boot.h"
#include "srom.h"
#include "cy_usbpd_defines.h"
#include "uvdm.h"
#include "app_timer_id.h"

/* FWCT SIZE in ROWs - 1 */
#if (CCG_FWCT_TABLE_SIZE > CCG_FLASH_ROW_SIZE)
#define FWCT_SIZE_IN_ROWS               (((CCG_FWCT_TABLE_SIZE / CCG_FLASH_ROW_SIZE) + (((CCG_FWCT_TABLE_SIZE % CCG_FLASH_ROW_SIZE)) ? 1u : 0u)) - 1u)
#else
#define FWCT_SIZE_IN_ROWS               (0u)
#endif /* (CCG_FWCT_TABLE_SIZE <= CCG_FLASH_ROW_SIZE) */


/* FWCT SIGNATURE SIZE in ROWs - 1 */
#if (CCG_FWCT_SIG_SIZE > CCG_FLASH_ROW_SIZE)
#define FWCT_SIG_SIZE_IN_ROWS           (((CCG_FWCT_SIG_SIZE / CCG_FLASH_ROW_SIZE) + (((CCG_FWCT_SIG_SIZE % CCG_FLASH_ROW_SIZE)) ? 1u : 0u)) - 1u)
#else
#define FWCT_SIG_SIZE_IN_ROWS           (0u)
#endif /* (CCG_FWCT_SIG_SIZE <= CCG_FLASH_ROW_SIZE) */

#if AUTH_BOOT
#if I2C_BOOT && BOOTWAIT_ENABLE
/* Global variable to keep track of solution level functions */
extern app_sln_handler_t *solution_fn_handler;
#endif /* I2C_BOOT && BOOTWAIT_ENABLE */

/* Global variable to hold the current FW update state */
static fw_update_states_t gl_fw_update_state = FW_UPDATE_NONE;

/* Global variabl to hold the current active update interface */    
static uint8_t gl_active_upd_inf = 0;
    
/* Global variable to indicate if firmware update is initiated or not */
static bool is_fwu_initiated = false;
    
typedef struct fwct_
{
    uint32_t identity;
    uint16_t table_size;
    uint8_t fwct_version;
    uint8_t crypto_algorithm;
    uint8_t guid[16];
    uint8_t base_fw_version[4];
    uint8_t app_fw_version[4];
    uint8_t min_base_fw_version[4];
    uint8_t min_app_fw_version[4];
    uint8_t nonce[32];
    uint8_t img_digest[32];
    uint8_t padding[144];
    uint32_t fwct_num;
    uint32_t checksum;
} __attribute__((packed)) fwct_t;

static uint8_t __attribute__ ((aligned (4))) fwct_sig[256]; 
static uint8_t __attribute__ ((aligned(4))) hash[CCG_SHA256_HASH_SIZE];

/* Global variable to hold the Last Row written */
static uint16_t glLastRowWritten = 0;

static const uint16_t gl_fwct_row_nos[][MAX_FWCT_IDX] = {{PRIM_FWCT_1_FLASH_ROW_NUM, PRIM_FWCT_2_FLASH_ROW_NUM}};
static uint8_t gl_fwct_active_row_idx[MAX_IMAGE_TYPES] = {(uint8_t)MAX_FWCT_IDX};
static uint8_t * gl_image_md_row_add[MAX_IMAGE_TYPES] = {(uint8_t *)(PRIMARY_FW_MD_ROW << CCG_FLASH_ROW_SHIFT_NUM)};

extern const uint8_t *gl_public_key_buffer;
extern const uint8_t *gl_guid_buffer;
const uint8_t *gl_public_key_buffer = (uint8_t *)((uint32_t)CCG_PUBLIC_KEY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM);
const uint8_t *gl_guid_buffer  = (uint8_t *)((((uint32_t)CCG_PUBLIC_KEY_ROW_NUM + 2u) << CCG_FLASH_ROW_SHIFT_NUM) - 16u);

static uint16_t gl_read_row = FLASH_INVALID_ROW_NUMBER;

/* Allign attribute is added because firmware calculates 4 byte checksum on
 * dwrords which needs to be aligned otherwise exception will be hit. 
 * */
static fwct_t __attribute__((aligned(4))) fwct_ram;

/* Function returns the current status of the update interface lock */
static bool get_upd_inf_status(uint8_t modes)
{
    return ((bool)((gl_active_upd_inf & modes) != 0u));
}

/* Function updates the interface lock */
static void set_upd_inf_access(bool is_enable, flash_interface_t mode)
{
    if (is_enable)
    {
        gl_active_upd_inf = (1u << (uint8_t)mode);
    }
    else
    {
        gl_active_upd_inf = 0;
    }
}

uint32_t ccg_calculate_dword_checksum(uint32_t *data, uint32_t size)
{
    uint8_t idx;
    uint32_t checksum = 0;
    
    for (idx = 0; idx < size; idx++)
    {
        checksum += data[idx];
    }   
    /* Return the binary sum. */
    return checksum;
}    

bool is_fw_update_initiated (void);
bool is_fw_update_initiated (void)
{
    return is_fwu_initiated;
}

void ccg_calculate_fw_hash (uint16_t row_num,
        uint32_t length,
        uint8_t output[CCG_SHA256_HASH_SIZE],
        uint8_t *metadata)
{
    mbedtls_sha256_context ctx;
    
    CALL_MAP(mbedtls_sha256_init)(&ctx);
    CALL_MAP(mbedtls_sha256_starts)(&ctx, (int32_t)false);
    
    /* Passing the flash address to SHA_256_update API without copying to temp buffer increases the speed of calculation.*/
    CALL_MAP(mbedtls_sha256_update)(&ctx, (uint8_t *)(row_num << CCG_FLASH_ROW_SHIFT_NUM), length);

    if (metadata != NULL)
    {
        /*assign primary firmware metadata row aadress to pointer. */
        CALL_MAP(mbedtls_sha256_update)(&ctx, metadata, CCG_FLASH_ROW_SIZE);
    }
    else
    {
        /* pointer is not null, hence it points to metadata row address. */
        ;
    }

    CALL_MAP(mbedtls_sha256_finish)(&ctx, output);
    CALL_MAP(mbedtls_sha256_free)(&ctx);
}

static uint8_t ccg_verify_signature (void)
{
    uint8_t sha256_output[CCG_SHA256_HASH_SIZE];
    /* Compute the Hash of the FWCT table. */
    CALL_MAP(mbedtls_sha256)((uint8_t *)&fwct_ram, CCG_FWCT_TABLE_SIZE, sha256_output, 0);
    return (uint8_t)(CALL_MAP(verify_signature) (sha256_output, CCG_SHA256_HASH_SIZE, (uint8_t *)fwct_sig, 256, 
                        (uint8_t *)(((uint32_t)CCG_PUBLIC_KEY_ROW_NUM) << CCG_FLASH_ROW_SHIFT_NUM), 256));
}

/* Function to identify the Valid FWCT Region of the Firmware */
static void find_valid_fwct(uint8_t image_t)
{
    uint32_t checksum;
    const uint16_t *fwct_row_nos = &gl_fwct_row_nos[image_t][0];
    const fwct_t *fwct_1 = (const fwct_t *)(fwct_row_nos[0] << CCG_FLASH_ROW_SHIFT_NUM);
    const fwct_t *fwct_2 = (const fwct_t *)(fwct_row_nos[1] << CCG_FLASH_ROW_SHIFT_NUM);;
    uint8_t valid_fwct = 0;
    uint8_t idx;
    
   /* Allow Flash Write Access only to the public key space */
    CALL_MAP(flash_set_access_limits)(fwct_row_nos[0], fwct_row_nos[1] + 2u, 
        CCG_IMG1_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    CALL_MAP(flash_enter_mode)(true, FLASH_IF_UVDM, false); 
    
    /* verify the validtiy of the both FWCT in flash. */
    for (idx = 0; idx < (uint8_t)MAX_FWCT_IDX; idx ++)
    {
        fwct_t *fwct = (fwct_t *)(fwct_row_nos[idx] << CCG_FLASH_ROW_SHIFT_NUM);
        if(fwct->identity == FWCT_SIGNATURE)
        {
            /* The entire FWCT row except the checksum field will be used for calculating 
             * the FWCT checksum information
             */
            /* QAC suppression 0310: fwct structure is ensured to be 4 byte aligned. */
            checksum = ccg_calculate_dword_checksum ((uint32_t *)fwct, /* PRQA S 0310 */
                (sizeof(fwct_sig) + sizeof(fwct_sig)) / 4u);
            if(0u == checksum)
            {
                valid_fwct |= (0x01u << idx);
            }
        }
    }
    /* valid_fwct contains the validity of both FWCT.
     * 0x00 - both FWCT are invalid
     * 0x01 - FWCT1 is valid
     * 0x02 - FWCT2 is valid
     * 0x03 - both FWCT 1 and 2 are valid.
     * Others are not used.
     */
    switch (valid_fwct)
    {
        case 0x00u:
        {
            gl_fwct_active_row_idx[image_t] = (uint8_t)MAX_FWCT_IDX;
            break;
        }
        case 0x01u:
        {
            gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_0;
            break;
        }
        case 0x02u:
        {
            gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_1;
            break;
        }
        case 0x03u:
        {
            if (fwct_1->fwct_num > fwct_2->fwct_num)
            {
                gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_0;
            }
            else if (fwct_1->fwct_num < fwct_2->fwct_num)
            {
                gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_1;
            }
            else /* Key numbers equal */
            {
                if ((fwct_1->app_fw_version > fwct_2->app_fw_version) || (fwct_1->base_fw_version > fwct_2->base_fw_version))
                {
                    gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_0;
                }
                else
                {
                    gl_fwct_active_row_idx[image_t] = (uint8_t)FWCT_IDX_1;
                }            
            }
            (void)CALL_MAP(flash_row_clear)(fwct_row_nos[(uint32_t)!(gl_fwct_active_row_idx[image_t] != 0u)]);
            (void)CALL_MAP(flash_row_clear)(fwct_row_nos[(uint32_t)!(gl_fwct_active_row_idx[image_t] != 0u)] + 1u);
            break;
        }
        default:
        {
            /* No statement */
            break;
        }
    }
    CALL_MAP(flash_enter_mode)(false,FLASH_IF_UVDM, false);
}
static void ccg_update_fwct_data(uint8_t image_t)
{
    fwct_t *fwct_p;
    uint32_t fwct_num = 0;
    uint32_t checksum;
    uint32_t active_row_idx = gl_fwct_active_row_idx[image_t];
    uint32_t inactive_row_idx;
    const uint16_t *fwct_row_nos = &gl_fwct_row_nos[image_t][0];
    uint8_t data[256] = {0};
    
    /* Check if active row number contains valid FWCT.*/
    if (active_row_idx >= (uint32_t)MAX_FWCT_IDX)
    {
        /* Invalid active FWCT. Assume index zero as defalut.*/
        active_row_idx = (uint8_t)FWCT_IDX_0;
    }
    else
    {
        /* Valid active FWCT is present. Get the FWCT number.*/
        fwct_p = (fwct_t *)(fwct_row_nos[active_row_idx] << CCG_FLASH_ROW_SHIFT_NUM);
        fwct_num = fwct_p->fwct_num;
    }

    inactive_row_idx = (active_row_idx + 1u) & 0x01u;
    
    fwct_ram.fwct_num = fwct_num + 1u;
    /* Calculate the 2's complement checksum for fwct and fwct signature. 
     * Exclude checksum feild in fwct in the calculation.
     */
    /* QAC suppression 0310: fwct structure is ensured to be 4 byte aligned. */
    checksum = ccg_calculate_dword_checksum ((uint32_t *)&fwct_ram, (CCG_FLASH_ROW_SIZE - 4u) / 4u); /* PRQA S 0310 */
    /* QAC suppression 0310, 3305: fwct_sig structure is ensured to be 256 byte aligned. */
    checksum += ccg_calculate_dword_checksum ((uint32_t *)&fwct_sig, sizeof(fwct_sig) / 4u); /* PRQA S 0310, 3305 */
    /* 2's complement is calculated by subtracting from 0.*/
    /* QAC suppression 2986: This is 2's complement and not a redundant operation. */
    fwct_ram.checksum = (uint32_t)(0u - checksum); /* PRQA S 2986 */
    CY_PDUTILS_MEM_COPY(data, (uint8_t *)&fwct_ram, CCG_FLASH_ROW_SIZE);
    
    CALL_MAP(flash_set_access_limits)(PRIM_FWCT_1_FLASH_ROW_NUM, PRIM_FWCT_2_FLASH_ROW_NUM + 2,
                            PRIMARY_FW_MD_ROW, CCG_PUBLIC_KEY_ROW_NUM - 1);
    (void)CALL_MAP(flash_row_write)((uint16_t)fwct_row_nos[inactive_row_idx], (uint8_t *)&data, NULL);
    (void)CALL_MAP(flash_row_write)((uint16_t)fwct_row_nos[inactive_row_idx] + 1u, (uint8_t *)&fwct_sig[0], NULL);

    /* Erase the data from the active FWCT space. */
    (void)CALL_MAP(flash_row_clear)((uint16_t)fwct_row_nos[active_row_idx]);
    
    /* Update the active fwct row index. */
    gl_fwct_active_row_idx[image_t] = (uint8_t)inactive_row_idx;

}

static bool ccg_validate_fwct(cy_en_pdstack_status_t *stat)
{
    uint8_t * guid_p;
    uint16_t row_num;   
    uint32_t fwct_ram_base;
    uint32_t fwct_flash_base;
    uint32_t fwct_ram_app;
    uint32_t fwct_flash_app;
 
    fwct_t *fwct_p;
    
    /*
     * The following are the basic checks done for the identity of the FWCT received
     * Check 1: The identity field should contain the signature 'F''W''C''T'
     * Check 2: The FWCT Version field should be 1.
     * Check 3: The FWCT table size should be 104 bytes.
     * If any of the checks fail, the firmware has received an FWCT with invalid identity
     */
  
    if ((fwct_ram.identity != FWCT_SIGNATURE) || (fwct_ram.fwct_version != 1u) ||
        (fwct_ram.table_size != CCG_FWCT_TABLE_SIZE)|| (fwct_ram.crypto_algorithm != (uint8_t)CRYPTO_ALGORITHM_USED))
    {
            *stat = CY_PDSTACK_STAT_INVALID_ID;
            return false;
    }
    /* Handling for dual image needs to be considered */
    guid_p = (uint8_t *)((uint32_t)(((uint32_t)CCG_PUBLIC_KEY_ROW_NUM + 2u) << CCG_FLASH_ROW_SHIFT_NUM) - (uint32_t)(16));

    /* QAC suppression 0315: Implicite conversion to void pointer is expected by this standard library function. */
    if(memcmp(&fwct_ram.guid[0], (uint8_t *)guid_p, CCG_GUID_SIZE) != 0) /* PRQA S 0315 */
    {  
        *stat = CY_PDSTACK_STAT_INVALID_GUID;
        return false;
    }
        
    /* The following are the version checks performed on the incoming FWCT. 
     * Minimum App version and Minimum Base version are checked against.
     */
    if( gl_fwct_active_row_idx[PRIMARY_IMAGE] >= (uint8_t)MAX_FWCT_IDX)
    {
        /* Invalid FWCT in flash. Clear it to 0. */
        row_num = gl_fwct_row_nos[PRIMARY_IMAGE][0];
    }
    else
    {
         row_num = gl_fwct_row_nos[PRIMARY_IMAGE][gl_fwct_active_row_idx[PRIMARY_IMAGE]];   
    }
 
    fwct_p = (fwct_t *)((row_num) << CCG_FLASH_ROW_SHIFT_NUM);
    
    CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_ram_base, (const uint8_t *)&fwct_ram.base_fw_version, sizeof(uint32_t));
    CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_flash_base, (const uint8_t *)&fwct_p->min_base_fw_version, sizeof(uint32_t));
    CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_ram_app, (const uint8_t *)&fwct_ram.app_fw_version, sizeof(uint32_t));
    CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_flash_app, (const uint8_t *)&fwct_p->min_app_fw_version, sizeof(uint32_t));
    
    /* Check if application name is matched. */
    if (((fwct_ram_app & 0x0000FFFFu) == (fwct_flash_app & 0x0000FFFFu)) || (fwct_flash_app == 0u))
    {
        /* Check if both app and base version shall be greater than 0. */
        if ((fwct_ram_app == 0u) || (fwct_ram_base == 0u))
        {
            /* App / Base version can not be 0. */
            *stat = CY_PDSTACK_STAT_INVALID_VER;
        }
        else
        {
            /* check if both are aleast greater or equal. */
            if ((fwct_ram_base < fwct_flash_base) || (fwct_ram_app < fwct_flash_app))
            {
                /* atleast one of base or app is lesser. then it is invalid version.*/
                *stat = CY_PDSTACK_STAT_INVALID_VER;
            }
            else
            {
                /* Either both are equal or greater.*/
                *stat = CY_PDSTACK_STAT_SUCCESS;
                return true;
            }
        }
    }    
    return false;
}

static bool fwu_set_access_limits (void)
{
    /* Defining variables and functions for extending to different flash architectures in the future */
    uint16_t flash_access_first;
    uint16_t flash_access_last;
    uint16_t flash_metadata_row;
    uint16_t flash_bl_last_row = CCG_BOOT_LOADER_LAST_ROW;    
                        
    flash_access_first = flash_bl_last_row + 1u;
    flash_access_last = CCG_LAST_FLASH_ROW_NUM;
    flash_metadata_row = CCG_IMG1_METADATA_ROW_NUM;
    
    /* Set the legal flash access range. */
    CALL_MAP(flash_set_access_limits)(flash_access_first, flash_access_last,
                flash_metadata_row, flash_bl_last_row);
    return true;
}

/* Function verifies the signature of the firmware */
cy_en_pdstack_status_t secure_boot_validate_fw(sys_fw_metadata_t * fw_md, uint8_t image_t)
{
    uint16_t  fw_loc;
    uint8_t sha256_output[CCG_SHA256_HASH_SIZE];    
    cy_en_pdstack_status_t stat;

    /* Find the valid FWCT present.*/
    find_valid_fwct(image_t);    
        
    /* Check the validity of Firmware metadata */
    stat = boot_validate_fw(fw_md);
    
    if (CY_PDSTACK_STAT_SUCCESS == stat)
    {
#if !USE_CYACD2_METADATA_FORMAT
        fw_loc = fw_md->boot_last_row + 1u;
#else

        fw_loc = ((uint16_t)fw_md->config_fw_start >> CCG_FLASH_ROW_SHIFT_NUM);
#endif /* !USE_CYACD2_METADATA_FORMAT */

        if(gl_fwct_active_row_idx[image_t] >= (uint8_t)MAX_FWCT_IDX)
        {
            stat = CY_PDSTACK_STAT_INVALID_FW;
        }
        else
        {
            /* Get the Active FWCT row */
            uint16_t active_row_num = gl_fwct_row_nos[image_t][gl_fwct_active_row_idx[image_t]];
            /* Read the stored FWCT into RAM */
            CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_ram, (const uint8_t *)(active_row_num << CCG_FLASH_ROW_SHIFT_NUM), CCG_FLASH_ROW_SIZE);
            /* Compute the hash of the Firmware and update the RAM FWCT firmware digest value */
            ccg_calculate_fw_hash (fw_loc, fw_md->config_fw_size, fwct_ram.img_digest, gl_image_md_row_add[image_t]);
            /* Compute the hash of the stored FWCT */
            CALL_MAP(mbedtls_sha256)((uint8_t *)&fwct_ram, CCG_FWCT_TABLE_SIZE, sha256_output, 0);
            /* Perform the signature check of the firmware */
            if(CALL_MAP(verify_signature)(sha256_output, CCG_SHA256_HASH_SIZE, (uint8_t *)((active_row_num + 1u) << CCG_FLASH_ROW_SHIFT_NUM), 256, 
                                        (uint8_t *)((uint32_t)CCG_PUBLIC_KEY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM), 256) == false)
            {
                /* If check fails, then indicate that the firmware is invalid */
                stat = CY_PDSTACK_STAT_INVALID_FW;
            }
        }
    }
    return stat;    
}

static void secure_boot_fwct_write_handler(uint16_t write_row, uint8_t *flash_mem, cy_en_pdstack_status_t *code, flash_interface_t interface)
{
    if (gl_fw_update_state != FW_UPDATE_NONE)
    {
        /* Out of sequence command. Reset the fsm state. */
        *code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
        gl_fw_update_state = FW_UPDATE_NONE;
        set_upd_inf_access(false, interface);
        return;
    }

    if (write_row > FWCT_SIZE_IN_ROWS)
    {
        /* Row number is not expected to be boyond the size of FWCT. Invalid command.*/
        *code = CY_PDSTACK_STAT_INVALID_COMMAND;
    }
    else
    {
        CY_PDUTILS_MEM_COPY((uint8_t *)&fwct_ram + (write_row << CCG_FLASH_ROW_SHIFT_NUM), flash_mem, CCG_MAX_FLASH_ROW_SIZE);
        *code = CY_PDSTACK_STAT_SUCCESS;
#if (CCG_FWCT_TABLE_SIZE > CCG_FLASH_ROW_SIZE)
        if (FWCT_SIZE_IN_ROWS == write_row)
#endif /* (CCG_FWCT_TABLE_SIZE > CCG_FLASH_ROW_SIZE) */
        {
            if (ccg_validate_fwct(code) == true)
            {
                /* Update the fsm state. */
                gl_fw_update_state = FW_UPDATE_FWCT;
                glLastRowWritten = 0;
            }
        }
    }
}
static void secure_boot_sig_write_handler(uint16_t write_row, uint8_t *flash_mem, cy_en_pdstack_status_t *code, flash_interface_t interface)
{
    if ((gl_fw_update_state != FW_UPDATE_FWCT))
    {
        /* Command sent out of sequence. Reset the FSM. */
        *code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
        gl_fw_update_state = FW_UPDATE_NONE;
        set_upd_inf_access(false, interface);
        return;
    }
    
    /* The expected value of write_row is 1 for writing FWCT signature */
    if(write_row > FWCT_SIG_SIZE_IN_ROWS)
    {
        *code = CY_PDSTACK_STAT_INVALID_COMMAND;
    }
    else
    {
        CY_PDUTILS_MEM_COPY(&fwct_sig[write_row << CCG_FLASH_ROW_SHIFT_NUM], flash_mem, CCG_FLASH_ROW_SIZE);
        *code = CY_PDSTACK_STAT_SUCCESS;
#if (CCG_FWCT_SIG_SIZE > CCG_FLASH_ROW_SIZE)
        if(FWCT_SIG_SIZE_IN_ROWS == write_row)
#endif /* (CCG_FWCT_SIG_SIZE > CCG_FLASH_ROW_SIZE) */
        {
            gl_fw_update_state = FW_UPDATE_FWCT_SIG;
        }
    }
}

static void secure_boot_enter_flash_handler(cy_en_pdstack_status_t *code, flash_interface_t interface)
{
    bool ret_val;
    /* Enter Flashing Mode command has been issued. */
    if (gl_fw_update_state != FW_UPDATE_FWCT_SIG)
    {
        /* Out of sequence command has been issued, reset the FSM state. */
        *code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
        gl_fw_update_state = FW_UPDATE_NONE;
        return;
    }

    (void)fwu_set_access_limits();

    /* Authenticate the FWCT with the received signature */
    ret_val = (bool)ccg_verify_signature();
    if (ret_val == true)
    {
        /* Enter Config mode: Writes are done in place. */
        flash_enter_mode(true, FLASH_IF_HPI, false);

        /* Determine the flash row to be erased, SE or Primary. */
        /* Invalidate primary fw metadata by writing 0s */
        (void)flash_row_clear(PRIMARY_FW_MD_ROW);
        *code = CY_PDSTACK_STAT_SUCCESS;
        gl_fw_update_state = FW_UPDATE_ENABLE_FLASH;
        glLastRowWritten = 0;
    }
    else
    {
        /* FWCT authentication failed. Reset the FSM state. */
        *code = CY_PDSTACK_STAT_INVALID_FWCT;
        gl_fw_update_state = FW_UPDATE_NONE;
        set_upd_inf_access(false, interface);
    }
}

static void secure_boot_flash_write_handler(uint16_t write_row, uint8_t *flash_mem, cy_en_pdstack_status_t *code, flash_interface_t interface)
{
    if ((gl_fw_update_state != FW_UPDATE_ENABLE_FLASH) || (glLastRowWritten > write_row))
    {
        /* Out of sequence command. Reset the FSM */
        *code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
        gl_fw_update_state = FW_UPDATE_NONE;
        set_upd_inf_access(false, interface);
        return;
    }
    
    if(write_row == PRIMARY_FW_MD_ROW)
    {
        /* Get the Primary start and size details from the Metadata in RAM. */
        /* QAC suppression 0310, 3305: flash_mem is ensured to be 64/128/256 byte aligned as per
         * the device flash architecture. */
        sys_fw_metadata_t *metadata = 
            (sys_fw_metadata_t *)&flash_mem[CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE]; /* PRQA S 0310, 3305 */
        if (boot_validate_fw(metadata) != CY_PDSTACK_STAT_SUCCESS)
        {
            *code = CY_PDSTACK_STAT_HASH_CMP_FAILED;
            gl_fw_update_state = FW_UPDATE_NONE;
            set_upd_inf_access(false, interface);
            return;
        }

#if !USE_CYACD2_METADATA_FORMAT
        /* Compute the newly flashed image's hash. */
        ccg_calculate_fw_hash (metadata->boot_last_row + 1u, metadata->fw_size, hash, flash_mem);
#else
        ccg_calculate_fw_hash (((uint32_t)metadata->config_fw_start >> CCG_FLASH_ROW_SHIFT_NUM), metadata->config_fw_size, hash, flash_mem);
#endif /* !USE_CYACD2_METADATA_FORMAT */

        /* QAC suppression 0315: Implicite conversion to void pointer is expected by this standard library function. */
        if (memcmp(hash, &fwct_ram.img_digest[0], CCG_SHA256_HASH_SIZE) == 0) /* PRQA S 0315 */
        {
            (void)CALL_MAP(flash_row_write)(write_row, flash_mem, NULL);    

            /* Update the FWCT data on the Flash. */
            ccg_update_fwct_data((uint8_t)PRIMARY_IMAGE);
            gl_read_row = write_row;
            /* Update the HPI registers with the latest version. */
           //hpi_update_regs(0x68u, fwct_ram.base_fw_version, sizeof(fwct_ram.base_fw_version));
            *code = CY_PDSTACK_STAT_SUCCESS;
        }
        else
        {
            *code = CY_PDSTACK_STAT_HASH_CMP_FAILED;
        }   

        /* Firmware has been written to the Flash. Reset the FSM
         * state. Wait for the device to be reset.
         */
        gl_fw_update_state = FW_UPDATE_NONE;
        set_upd_inf_access(false, interface);
    }
    else
    {
        if(CALL_MAP(flash_row_write)(write_row, flash_mem, NULL) == CY_PDSTACK_STAT_SUCCESS)
        {
            gl_read_row = write_row;
            *code = CY_PDSTACK_STAT_SUCCESS;
        }
        else
        {
            *code = CY_PDSTACK_STAT_FLASH_UPDATE_FAILED;
            gl_fw_update_state = FW_UPDATE_NONE;
            set_upd_inf_access(false, interface);
        }
    }
}

#if I2C_BOOT
void update_i2c_fsm(uint8_t cmd_opcode, uint8_t *cmd_param, uint8_t *flash_mem, cy_en_pdstack_status_t *stat, hpi_response_t *code, bool *is_handled)
{
    uint16_t write_row;
    
    switch ((hpi_dev_reg_address_t)cmd_opcode)
    {
        case HPI_DEV_REG_ENTER_FLASH_MODE:
            {
                if (cmd_param[HPI_SIGNATURE_OFFSET] == (uint8_t)HPI_ENTER_FLASHING_CMD_SIG)
                {
                    /* Check if Firmware update interface is already enabled. */
                    if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_HPI)))
                    {
                        *is_handled = true;
                        /* This is an invalid command. */
                        *code = HPI_RESPONSE_INVALID_COMMAND;
                    }
                    else
                    {
                       *is_handled = true;
                        secure_boot_enter_flash_handler(stat, FLASH_IF_HPI);
                        *code = (hpi_response_t)(uint8_t)CCG_STATUS_TO_HPI_RESPONSE((uint8_t)*stat);
                    }
                }
                break;
            }
        case HPI_DEV_REG_FLASH_READ_WRITE:
            {
                /*
                 * Check master wrote four bytes: Signature, FLASH_CMD
                 * and two bytes row_num. And the flash command is
                 * either write or read.
                 */
                if (cmd_param[HPI_SIGNATURE_OFFSET] != (uint8_t)HPI_FLASH_READ_WRITE_CMD_SIG)
                {
                    *is_handled = true;
                    *stat = CY_PDSTACK_STAT_BAD_PARAM;
                    *code = HPI_RESPONSE_FLASH_UPDATE_FAILED;
                    gl_fw_update_state = FW_UPDATE_NONE;
                    set_upd_inf_access(false, FLASH_IF_HPI);
                    break;
                }
                switch (cmd_param[HPI_FLASH_READ_WRITE_CMD_OFFSET])
                {
                    case HPI_FWCT_SIG_WRITE_CMD:
                        {
                            /* Check if Firmware update interface is already enabled. */
                            if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_HPI)))
                            {
                                *is_handled = true;
                                /* This is an invalid command. */
                                *code = HPI_RESPONSE_INVALID_COMMAND;
                            }
                            else
                            {
                                *is_handled = true;

                                /* FWCT Signature Write command. */  
                                *stat = CY_PDSTACK_STAT_SUCCESS;
                                write_row = CY_PDUTILS_MAKE_WORD (cmd_param[HPI_FLASH_READ_WRITE_ROW_MSB], cmd_param[HPI_FLASH_READ_WRITE_ROW_LSB]);
                                secure_boot_sig_write_handler(write_row, flash_mem, stat, FLASH_IF_HPI);
                                *code = (hpi_response_t)(uint8_t)CCG_STATUS_TO_HPI_RESPONSE((uint8_t)*stat);
                            }
                            break;
                        }
                    case HPI_FWCT_ROW_WRITE_CMD:
                        {
                            /* Check if Firmware update interface is already enabled. */
                            if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_HPI)))
                            {
                                *is_handled = true;
                                /* This is an invalid command. */
                                *code = HPI_RESPONSE_INVALID_COMMAND;
                            }
                            else
                            {
                                *is_handled = true;
                                set_upd_inf_access(true, FLASH_IF_HPI);
                                /* 128 bytes FWCT Write command. */
                                *stat = CY_PDSTACK_STAT_SUCCESS;
                                write_row = CY_PDUTILS_MAKE_WORD (cmd_param[HPI_FLASH_READ_WRITE_ROW_MSB], cmd_param[HPI_FLASH_READ_WRITE_ROW_LSB]);
                                secure_boot_fwct_write_handler(write_row, flash_mem, stat, FLASH_IF_HPI);
                                *code = (hpi_response_t)(uint8_t)CCG_STATUS_TO_HPI_RESPONSE((uint8_t)*stat);
                            }
                            break;                    
                        }
                    case HPI_FLASH_ROW_WRITE_CMD:
                        {
                            *is_handled = true;

                            /* Flash write command. */
                            *stat = CY_PDSTACK_STAT_SUCCESS;
                            write_row = CY_PDUTILS_MAKE_WORD (cmd_param[HPI_FLASH_READ_WRITE_ROW_MSB], cmd_param[HPI_FLASH_READ_WRITE_ROW_LSB]);
                            secure_boot_flash_write_handler(write_row, flash_mem, stat, FLASH_IF_UVDM);
                            *code = (hpi_response_t)(uint8_t)CCG_STATUS_TO_HPI_RESPONSE((uint8_t)*stat);
                            glLastRowWritten = write_row;
                            break;
                        }
                    case HPI_FLASH_ROW_READ_CMD:
                        {
                            write_row = CY_PDUTILS_MAKE_WORD (cmd_param[HPI_FLASH_READ_WRITE_ROW_MSB], cmd_param[HPI_FLASH_READ_WRITE_ROW_LSB]);
                            if (gl_read_row != write_row)
                            {
                                *is_handled = true;
                                /* Out of sequence command. Reset the FSM */
                                *code = HPI_RESPONSE_OUT_OF_SEQ_CMD;
                                gl_fw_update_state = FW_UPDATE_NONE;
                                set_upd_inf_access(false, FLASH_IF_HPI);
                            }
                            gl_read_row = FLASH_INVALID_ROW_NUMBER;
                            break;
                        }
                   default:
                        {
                            *is_handled = true;
                            *stat = CY_PDSTACK_STAT_NOT_SUPPORTED;
                            *code = HPI_RESPONSE_NOT_SUPPORTED;
                            gl_fw_update_state = FW_UPDATE_NONE;
                            set_upd_inf_access(false, FLASH_IF_HPI);
                            break;
                        }    
                }
                break;
            }
#if BOOTWAIT_ENABLE
            case HPI_DEV_REG_JUMP_TO_BOOT:
            {
                if(CALL_MAP(Cy_PdUtils_SwTimer_IsRunning)(solution_fn_handler->Get_PdStack_Context(0)->ptrTimerContext, BL_BOOT_WAIT_TIMER_ID) == true)
                {
                    *is_handled = true;
                    if (cmd_param[HPI_SIGNATURE_OFFSET] == (uint8_t)HPI_JUMP_TO_BOOT_CMD_SIG)
                    {
                        *code = HPI_RESPONSE_SUCCESS;
                        CALL_MAP(Cy_PdUtils_SwTimer_Stop)(solution_fn_handler->Get_PdStack_Context(0)->ptrTimerContext, BL_BOOT_WAIT_TIMER_ID);
                    }
                }
                break;
            }
#endif /* BOOTWAIT_ENABLE */
            default:
            {
                /* No statement */
                break;
            }
    }
}
#endif /* I2C_BOOT */

#if CC_BOOT
/* Signed Firmware Update State Machine */
void signed_fw_update_fsm (uint8_t cmd_opcode, uint8_t cmd_length, uint8_t *cmd_param, cy_pd_pd_do_t *response,
        cy_en_pdstack_status_t *response_code, uint8_t *data, bool *is_handled)
{   
    uint16_t write_row;

    switch((uvdm_cmd_opcode_t)cmd_opcode)
    {
        case UVDM_CMD_ENTER_FLASHING_MODE_OPCODE:
            {
                if(cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] != UVDM_ENTER_FLASHING_MODE_CMD_SIG)
                {
                    *is_handled = true;
                    gl_fw_update_state = FW_UPDATE_NONE;
                    *response_code = CY_PDSTACK_STAT_BAD_PARAM;
                    /* Set the NAK field in the header VDO. */
                    response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                    set_upd_inf_access(false, FLASH_IF_UVDM);
                }                
                else
                {
                    /* Check if Firmware update interface is already enabled. */
                    if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_UVDM)))
                    {
                        *is_handled = true;
                        /* This is an invalid command. */
                        *response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                         response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                                           = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                    }
                    else
                    {
                        *is_handled = true;
                        secure_boot_enter_flash_handler(response_code, FLASH_IF_UVDM);
                    }
                }
                break; 
            }     
          
        case UVDM_CMD_FWCT_ROW_WRITE:
            {
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected.
                 */
                if ((cmd_length != UVDM_FWCT_ROW_WRITE_CMD_SIZE) ||
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] !=
                         (uint8_t)UVDM_FWCT_ROW_WRITE_CMD_SIG))
                {
                    *is_handled = true;
                    gl_fw_update_state = FW_UPDATE_NONE;
                    *response_code = CY_PDSTACK_STAT_BAD_PARAM;
                    /* Set the NAK field in the header VDO. */
                    response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                else
                {
                    /* Check if Firmware update interface is already enabled. */
                    if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_UVDM)))
                    {
                        *is_handled = true;
                        /* This is an invalid command. */
                        *response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                         response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                                           = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                    }
                    else
                    {
                        set_upd_inf_access(true, FLASH_IF_UVDM);
                        *is_handled = true;
                        *response_code = CY_PDSTACK_STAT_SUCCESS;
                        
                        /* FWCT write command */
                        write_row = CY_PDUTILS_MAKE_WORD (cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET], cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                        
                        secure_boot_fwct_write_handler(write_row, data, response_code, FLASH_IF_UVDM);
                        if(*response_code == CY_PDSTACK_STAT_OUT_OF_SEQ_CMD)
                        {
                            response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                                    = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                        }
                    }
                }
                break;                
            }
        
        case UVDM_CMD_FWCT_SIG_WRITE:
            {
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected.
                 */
                if ((cmd_length != UVDM_FWCT_SIG_WRITE_CMD_SIZE) ||
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] !=
                         (uint8_t)UVDM_FWCT_SIG_WRITE_CMD_SIG))
                {
                    *is_handled = true;
                    gl_fw_update_state = FW_UPDATE_NONE;
                    *response_code = CY_PDSTACK_STAT_BAD_PARAM;
                    /* Set the NAK field in the header VDO. */
                    response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                else
                {
                    /* Check if Firmware update interface is already enabled. */
                    if (get_upd_inf_status ((uint8_t)~(1u << (uint8_t)FLASH_IF_UVDM)))
                    {
                        *is_handled = true;
                        /* This is an invalid command. */
                        *response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                         response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                                           = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                    }
                    else
                    {                    
                        *is_handled = true;
                        write_row = CY_PDUTILS_MAKE_WORD (cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET], cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                        secure_boot_sig_write_handler(write_row, data, response_code, FLASH_IF_UVDM);
                    }
                }
                break;
            }
        case UVDM_CMD_FLASH_WRITE_OPCODE:
            {    
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected. Also ensure that there is no pending
                 * FLASH WRITE command in the system.
                 */
                if ((cmd_length != UVDM_FLASH_READ_WRITE_CMD_SIZE) ||
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] !=
                         UVDM_FLASH_READ_WRITE_CMD_SIG))
                {
                    /* Set the NAK field in the header VDO. */
                    *is_handled = true;
                    response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                else
                {
                    *is_handled = true;
                    write_row = CY_PDUTILS_MAKE_WORD (cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET], cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                    secure_boot_flash_write_handler(write_row, data, response_code, FLASH_IF_UVDM);
                    glLastRowWritten = write_row;
                    break;
                }
            }
            /* QAC suppression 2003: Intentional fall through. */
        case UVDM_CMD_READ_DATA_OPCODE: /* PRQA S 2003 */
            {
                /* Allow the handling to pass to the UVDM handler */
                break;
            }
        case UVDM_CMD_SEND_DATA_OPCODE:
            {
                /* Allow the handling to pass to the UVDM handler */
                break;
            }    
        case UVDM_CMD_FLASH_READ_OPCODE:
            {
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected. Also ensure that there is no pending
                 * FLASH WRITE command in the system.
                 */
                if ((cmd_length != UVDM_FLASH_READ_WRITE_CMD_SIZE) ||
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] !=
                         UVDM_FLASH_READ_WRITE_CMD_SIG))
                {
                    /* Set the NAK field in the header VDO. */
                    *is_handled = true;
                    response[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                else
                {
                    /* Allow flash read in Signed FSM only for write verification */
                    write_row =  CY_PDUTILS_MAKE_WORD (cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET], cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                    if (gl_read_row != write_row)
                    {
                        *is_handled = true;
                        /* Out of sequence command. Reset the FSM */
                        *response_code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
                        gl_fw_update_state = FW_UPDATE_NONE;
                        set_upd_inf_access(false, FLASH_IF_UVDM);
                    }
                    gl_read_row = FLASH_INVALID_ROW_NUMBER;
                }
                break;
            }
        default:
            {
                if (gl_fw_update_state != FW_UPDATE_NONE)
                {
                    *is_handled = true;
                    *response_code = CY_PDSTACK_STAT_OUT_OF_SEQ_CMD;
                    gl_fw_update_state = FW_UPDATE_NONE;
                    set_upd_inf_access(false, FLASH_IF_UVDM);
                }
                break;
            }            
    }
}
#endif /* CC_BOOT */
#endif /* AUTH_BOOT */
