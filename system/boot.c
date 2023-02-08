/******************************************************************************
* File Name: boot.c
* \version 2.0
*
* Description: Source file for boot functions
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include <config.h>
#include "cy_pdstack_common.h"
#include "cy_flash.h"
#include <system.h>
#include <flash.h>
#include <boot.h>
#include <srom.h>
#if AUTH_BOOT
#include "fw_update.h"
#endif /* AUTH_BOOT */

#if (SECURE_FW_UPDATE == 1)
#include <secure_boot.h>
#include <crypto_hal.h>
#endif /* SECURE_FW_UPDATE */

/* Structure to hold reason for boot mode. */
fw_img_status_t gl_img_status;

#if CCG_DUALAPP_DISABLE
/* If Dual-App bootloading is disabled, provide stub variable to keep the compiler happy. */
volatile uint8_t gl_Bootloader_1_activeApp = 0;
#endif

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

/* Pointer to Image-1 FW pseudo-metadata table. */
sys_fw_metadata_t * gl_img1_fw_pseudo_metadata;

/* Pointer to Image-2 FW pseudo-metadata table. */
sys_fw_metadata_t * gl_img2_fw_pseudo_metadata =
    (sys_fw_metadata_t *)(CCG_IMG2_FW_PSEUDO_METADATA_ADDR);

#endif /* CCG_PSEUDO_METADATA_DISABLE */

#if (CCG_BOOT != 0)
#if (BOOT_WAIT_WINDOW_DISABLE == 0)
/* Boot-wait duration specified by firmware metadata. */
static volatile uint16_t gl_boot_wait_delay = CCG_BL_WAIT_DEFAULT;
#endif /* (BOOT_WAIT_WINDOW_DISABLE == 0) */
#endif /* CCG_BOOT */

#if (!(SROM_CODE_SYS_BOOT)) 
/* Pointer to Image-1 FW metadata table. */
sys_fw_metadata_t * gl_img1_fw_metadata = 
        (sys_fw_metadata_t *)(CCG_IMG1_FW_METADATA_ADDR);

#if (!CCG_DUALAPP_DISABLE)
/* Pointer to Image-2 FW metadata table. */
sys_fw_metadata_t * gl_img2_fw_metadata = 
        (sys_fw_metadata_t *)(CCG_IMG2_FW_METADATA_ADDR);
#endif /* (!CCG_DUALAPP_DISABLE) */
#endif /* (!SROM_CODE_SYS_BOOT) */

fw_img_status_t get_boot_mode_reason(void)
{
    /* Return the reason for boot mode. */
    return gl_img_status;
}

#define CRC_TABLE_SIZE                      (16U)           /* A number of uint32_t elements in the CRC32 table */
#define CRC_INIT                            (0xFFFFFFFFU)
#define NIBBLE_POS                          (4U)
#define NIBBLE_MSK                          (0xFU)

uint32_t calculate_crc32(const uint8_t *address, uint32_t length)
{
    /* Contains generated values to calculate CRC-32C by 4 bits per iteration*/
    static const uint32_t crcTable[CRC_TABLE_SIZE] =
    {
        0x00000000U, 0x105ec76fU, 0x20bd8edeU, 0x30e349b1U,
        0x417b1dbcU, 0x5125dad3U, 0x61c69362U, 0x7198540dU,
        0x82f63b78U, 0x92a8fc17U, 0xa24bb5a6U, 0xb21572c9U,
        0xc38d26c4U, 0xd3d3e1abU, 0xe330a81aU, 0xf36e6f75U,
    };

    uint32_t crc = CRC_INIT;
    if (length != 0U)
    {
        do
        {
            crc = crc ^ *address;
            crc = (crc >> NIBBLE_POS) ^ crcTable[crc & NIBBLE_MSK];
            crc = (crc >> NIBBLE_POS) ^ crcTable[crc & NIBBLE_MSK];
            --length;
            ++address;
        } while (length != 0U);
    }
    return (~crc);
}

/* Check whether configuration table checksum is good. */
cy_en_pdstack_status_t boot_validate_configtable(uint8_t *table_p)
{
    uint16_t size = CY_PDUTILS_MAKE_WORD (table_p[CONFIGTABLE_SIZE_OFFSET + 1u], table_p[CONFIGTABLE_SIZE_OFFSET]);

    if (((uint32_t)table_p >= CY_FLASH_SIZE) ||
            (CY_PDUTILS_MAKE_WORD (table_p[1], table_p[0]) != CONFIGTABLE_SIGNATURE))
    {
        return CY_PDSTACK_STAT_INVALID_FW;
    }

    uint32_t table_crc=CY_PDUTILS_MAKE_DWORD(table_p[CONFIGTABLE_CHECKSUM_OFFSET+3],table_p[CONFIGTABLE_CHECKSUM_OFFSET+2],table_p[CONFIGTABLE_CHECKSUM_OFFSET+1],table_p[CONFIGTABLE_CHECKSUM_OFFSET]);

    if (table_crc != calculate_crc32 (
                    table_p + CONFIGTABLE_CHECKSUM_START, (uint32_t)size - CONFIGTABLE_CHECKSUM_START))
        {
            return CY_PDSTACK_STAT_INVALID_FW;
        }
    return CY_PDSTACK_STAT_SUCCESS;
}

cy_en_pdstack_status_t boot_validate_fw(sys_fw_metadata_t *fw_metadata)
{
    cy_en_pdstack_status_t status;
    /* Pointer to FW image start address. */
    uint32_t fw_start;
    /* Size of FW image. */
    uint32_t fw_size;
   
#if USE_CYACD2_METADATA_FORMAT
    fw_start = fw_metadata->fw_start;
#else
    fw_start = (((uint32_t)fw_metadata->boot_last_row << CCG_FLASH_ROW_SHIFT_NUM)
            + CY_FLASH_SIZEOF_ROW);
#endif /* USE_CYACD2_METADATA_FORMAT */

    fw_size = fw_metadata->fw_size;
    
    /*
     * Validate:
     * 1) FW size.
     * 2) FW checksum.
     * 3) FW entry.
     */

    if (
            (fw_size == 0u) ||
            ((fw_start + fw_size) >= CY_FLASH_SIZE) ||
#if USE_CYACD2_METADATA_FORMAT
             (fw_metadata->fw_crc32 != calculate_crc32((uint8_t *)fw_start, fw_size))
#else
            (fw_metadata->fw_entry < fw_start) ||
            (fw_metadata->fw_entry >= (fw_start + fw_size)) ||
            /* QAC suppression 3415: The side effect is not harmful. If the previous checks 
             * do not pass, the firmware is invalid and evaluating checksum is redundant, */
            (fw_metadata->fw_checksum != CALL_MAP(Cy_PdUtils_MemCalculateByteChecksum) ((uint8_t *)fw_start, fw_size)) /* PRQA S 3415 */
#endif /* USE_CYACD2_METADATA_FORMAT */
       )
    {
        status = CY_PDSTACK_STAT_INVALID_FW;
    }
    else
    {

        status = boot_validate_configtable ((uint8_t *)(fw_metadata->config_fw_start));

    }

    return status;
}

cy_en_pdstack_status_t boot_handle_validate_fw_cmd(sys_fw_mode_t fw_mode)
{
    sys_fw_metadata_t *md_p = NULL;
    cy_en_pdstack_status_t code = CY_PDSTACK_STAT_NO_RESPONSE;

#if (CCG_BOOT != 0)
    switch (fw_mode)
    {
        case SYS_FW_MODE_FWIMAGE_1:
            md_p = CALL_MAP(gl_img1_fw_metadata);
            break;

#if (!CCG_DUALAPP_DISABLE)
        case SYS_FW_MODE_FWIMAGE_2:
            md_p = CALL_MAP(gl_img2_fw_metadata);
            break;
#endif /* (!CCG_DUALAPP_DISABLE) */

        default:
            code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
            break;
    }
#else /* (CCG_BOOT != 0) */
    if (fw_mode == CALL_MAP(sys_get_device_mode)())
    {
        /* There is no need to validate the currently running image. */
        code = CY_PDSTACK_STAT_SUCCESS;
    }
    else
    {
        switch (fw_mode)
        {
            case SYS_FW_MODE_FWIMAGE_1:
#if (CCG_PSEUDO_METADATA_DISABLE == 0)
                /*
                 * CCG is in FW Image 2 and request is to validate FW image 1. 
                 * There can be following cases:
                 * 1) If Image-1 PMD signature is "CP", use Image-1 PMD for validation.
                 * 2) Otherwise, use Image-1 MD for validation.
                 */
                if(gl_img1_fw_pseudo_metadata->metadata_valid == 
                    SYS_PSEUDO_METADATA_VALID_SIG)
                {
                    md_p = gl_img1_fw_pseudo_metadata;
                }
                else
#endif /* (CCG_PSEUDO_METADATA_DISABLE == 0) */
                {
                    md_p = CALL_MAP(gl_img1_fw_metadata);
                }
                break;

#if (!CCG_DUALAPP_DISABLE)
            case SYS_FW_MODE_FWIMAGE_2:
#if (CCG_PSEUDO_METADATA_DISABLE == 0)
                /*
                 * CCG is in FW Image 1 and request is to validate FW image 2. 
                 * There can be following cases:
                 * 1) If Image-2 PMD signature is "CP", use Image-2 PMD for validation.
                 * 2) Otherwise, use Image-1 MD for validation.
                 */
                if(gl_img2_fw_pseudo_metadata->metadata_valid == 
                    SYS_PSEUDO_METADATA_VALID_SIG)
                {
                    md_p = gl_img2_fw_pseudo_metadata;
                }
                else
#endif /* (CCG_PSEUDO_METADATA_DISABLE == 0) */
                {
                    md_p = CALL_MAP(gl_img2_fw_metadata);
                }
                break;
#endif /* (!CCG_DUALAPP_DISABLE) */

            default:
                code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                break;
        }
    }
#endif /* (CCG_BOOT != 0) */

    if (md_p != NULL)
    {
        if (boot_validate_fw (md_p) == CY_PDSTACK_STAT_SUCCESS)
        {
            code = CY_PDSTACK_STAT_SUCCESS;
            /* QAC suppression 2991, 2995: This operation is redundant as this application supports
             * single firmware. This is retained for other applications which have dual firmware. */
            if(fw_mode == SYS_FW_MODE_FWIMAGE_1) /* PRQA S 2991, 2995 */
            {
                gl_img_status.status.fw1_invalid = 0;
            }
#if (!CCG_DUALAPP_DISABLE)
            else if(fw_mode == SYS_FW_MODE_FWIMAGE_2)
            {
                gl_img_status.status.fw2_invalid = 0;
            }
#endif /* !CCG_DUALAPP_DISABLE */
        }
        else
        {
            code = CY_PDSTACK_STAT_INVALID_FW;
            /* QAC suppression 2991, 2995: This operation is redundant as this application supports
             * single firmware. This is retained for other applications which have dual firmware. */
            if(fw_mode == SYS_FW_MODE_FWIMAGE_1) /* PRQA S 2991, 2995 */
            {
                gl_img_status.status.fw1_invalid = 1;
            }
#if (!CCG_DUALAPP_DISABLE)
            else if(fw_mode == SYS_FW_MODE_FWIMAGE_2)
            {
                gl_img_status.status.fw2_invalid = 1;
            }
#endif /* !CCG_DUALAPP_DISABLE */
        }
    }
    return code;
}

extern volatile uint32_t cyBtldrRunType;

#if (CCG_BOOT != 0)
#if (BOOT_WAIT_WINDOW_DISABLE == 0)
/* Return the boot-wait setting to the user code. */
uint16_t boot_get_wait_time(void)
{
    return (gl_boot_wait_delay);
}

static void boot_set_wait_timeout(sys_fw_metadata_t *md_p)
{
    /* Check for boot-wait option. */
    if (md_p->boot_app_id == CCG_FWMETA_APPID_WAIT_0)
    {
        gl_boot_wait_delay = 0;
    }
    else
    {
        if (md_p->boot_app_id != CCG_FWMETA_APPID_WAIT_DEF)
        {
            /* Get the boot-wait delay from metadata, applying the MIN and MAX limits. */
            gl_boot_wait_delay = CY_USBPD_GET_MAX(CCG_BL_WAIT_MAXIMUM, CY_USBPD_GET_MIN (CCG_BL_WAIT_MINIMUM,
                        md_p->boot_app_id));
        }
    }
}
#endif /* (BOOT_WAIT_WINDOW_DISABLE == 0) */

#define MD_ROW                      (0x1FF80)
/*This function schedules recent and valid FW. It sets boot mode reason */
bool boot_start(void)
{
    sys_fw_metadata_t *md_p;
    bool boot_fw1 = false;
    bool boot_fw2 = false;

#if (CCG_DUALAPP_DISABLE)
    (void)boot_fw1;
    (void)boot_fw2;
#else
    uint8_t img = Bootloader_1_MD_BTLDB_ACTIVE_0;
#endif /* CCG_DUALAPP_DISABLE */

    md_p = NULL;
    gl_img_status.val = 0;
   
    /* Check the two firmware binaries for validity. */
#if (AUTH_BOOT && !CCG_DISABLE_SIG_CHECK_ON_BOOT)
    /* If authenticated boot feature is enabled and if signature verification based on 
     * crypto algorithms are enabled, we check the signature of the firmwrare for validity
     * on every boot up.
     */
    if (secure_boot_validate_fw((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR, (uint8_t)PRIMARY_IMAGE) != CY_PDSTACK_STAT_SUCCESS)
#else
    if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CY_PDSTACK_STAT_SUCCESS)
#endif /* (AUTH_BOOT && !CCG_DISABLE_SIG_CHECK_ON_BOOT) */    
    {
        gl_img_status.status.fw1_invalid  = 1;
    }

#if (!CCG_DUALAPP_DISABLE)

#if AUTH_BOOT
    /* If signed firmware feature is enabled, we check the signature of the firmwrare for validity */
    if (secure_boot_validate_fw((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR, PRIMARY_IMAGE) != CY_PDSTACK_STAT_SUCCESS)
#else
    if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CY_PDSTACK_STAT_SUCCESS)
#endif /* AUTH_BOOT */

#endif /* CCG_DUALAPP_DISABLE */
    {
        gl_img_status.status.fw2_invalid = 1;
    }
    
    /* Check for the boot mode request. */
    /*
     * NOTE: cyBtldrRunType is Bootloader component provided variable.
     * It is used to store the jump signature. Check the lower two bytes
     * for signature.
     */
    if ((cyBtldrRunType & 0xFFFFu) == SYS_BOOT_MODE_RQT_SIG)
    {
        /*
         * FW has made a request to stay in boot mode. Return
         * from here after clearing the variable.
         */
        cyBtldrRunType = 0;
        /* Set the reason for boot mode. */
        gl_img_status.status.boot_mode_request = (uint8_t)true;
        return false;
    }
    
    /* Check if we have been asked to boot FW1 or FW2 specifically. */
    if ((cyBtldrRunType & 0xFFFFu) == CCG_FW1_BOOT_RQT_SIG)
    {
        boot_fw1 = true;
    }

#if (!CCG_DUALAPP_DISABLE)
    if ((cyBtldrRunType & 0xFFFF) == CCG_FW2_BOOT_RQT_SIG)
        boot_fw2 = true;
#endif /* CCG_DUALAPP_DISABLE */

#if (!CCG_DUALAPP_DISABLE)
    /*
     * If we have been specifically asked to boot FW2, do that.
     * Otherwise, if we have not been specifically asked to boot FW1; choose the binary with
     * greater sequence number.
     */
    if (!gl_img_status.status.fw2_invalid)
    {
        /* 
         * FW2 is valid.
         * We can boot this if:
         * 1. We have been asked to boot FW2.
         * 2. FW1 is not valid.
         * 3. FW2 is newer than FW1, and we have not been asked to boot FW1.
         */
        if ((boot_fw2) || (gl_img_status.status.fw1_invalid) ||
                ((!boot_fw1) && (CALL_MAP(sys_get_recent_fw_image)() == SYS_FW_MODE_FWIMAGE_2)))
        {
            md_p = CALL_MAP(gl_img2_fw_metadata);
            img  = Bootloader_1_MD_BTLDB_ACTIVE_1;
        }
        else
        {
            md_p = CALL_MAP(gl_img1_fw_metadata);
            img  = Bootloader_1_MD_BTLDB_ACTIVE_0;
        }
    }
    else
#endif /* CCG_DUALAPP_DISABLE */
    {
        /* FW2 is invalid. */
        /* Load FW1 if it is valid. */
        if (!(gl_img_status.status.fw1_invalid != 0u))
        {
            md_p = CALL_MAP(gl_img1_fw_metadata);
        }
    }
    
    if (md_p != NULL)
    {
#if (BOOT_WAIT_WINDOW_DISABLE == 0)
        /*
         * If we are in the middle of a jump-to-alt-fw command, do not provide
         * the boot wait window.
         */
        if ((boot_fw1) || (boot_fw2))
        {
            gl_boot_wait_delay = 0;
        }
        else
        {
            boot_set_wait_timeout (md_p);
        }
#endif /* BOOT_WAIT_WINDOW_DISABLE */

#if (!CCG_DUALAPP_DISABLE)
        Bootloader_1_activeApp = img;
#endif /* (!CCG_DUALAPP_DISABLE) */
        return true;
    }

    /* Stay in bootlaoder. */
    return false;
}

void boot_jump_to_fw(void)
{
    /* Schedule the FW and undergo a reset. */
    /* To be reviewed: Bootloader_1_SET_RUN_TYPE (Bootloader_1_START_APP); */
    cyBtldrRunType = 0x01;
    Cy_SysLib_ClearResetReason();
    NVIC_SystemReset();
}
#else /* !CCG_BOOT */

void boot_update_fw_status(void)
{
#if (!CCG_BOOT)
    gl_img_status.val = 0;

    /* Check the two firmware binaries for validity. */
    if (boot_handle_validate_fw_cmd (SYS_FW_MODE_FWIMAGE_1) != CY_PDSTACK_STAT_SUCCESS)
    {
        gl_img_status.status.fw1_invalid = 1;
    }

#if (!CCG_DUALAPP_DISABLE)
    if (boot_handle_validate_fw_cmd (SYS_FW_MODE_FWIMAGE_2) != CY_PDSTACK_STAT_SUCCESS)
#endif /* !CCG_DUALAPP_DISABLE */
    {
        gl_img_status.status.fw2_invalid = 1;
    }

#if APP_PRIORITY_FEATURE_ENABLE
    /* Update the app-priority field if the feature is enabled. */
    gl_img_status.status.reserved1 = ((*(uint8_t *)(CCG_APP_PRIORITY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM)) << 4);
#endif /* APP_PRIORITY_FEATURE_ENABLE */

#endif /* CCG_BOOT */
}
    
#if (CCG_PSEUDO_METADATA_DISABLE == 0)

void boot_check_for_valid_fw(void)
{
    /* Temporary data buffer (of size one Flash row) to store contents of pseudo metadata. */
    uint8_t temp_pseudo_metadata_buf[CY_FLASH_SIZEOF_ROW] = {0};
    sys_fw_metadata_t *p_md, *alt_p_md, *md, *alt_md;
    uint16_t alt_p_md_row_num, alt_md_row_num, p_md_row_num;
    /* To hold the bit position of alternate image in Image status structure. */
    uint8_t alt_img_bit_pos;
    bool invalidate_md = false;
#if (SECURE_FW_UPDATE == 1)
    /* Buffer to hold FW HASH. */
#if (MBEDTLS_SHA2 != 1)    
    uint32_t fw_hash[CRYPTO_SHA_2_HASH_SIZE_WORDS] = {0};
#else    
    uint8_t fw_hash[CRYPTO_SHA_2_HASH_SIZE_BYTES] = {0};
#endif
#endif /* SECURE_FW_UPDATE */

    /*Pointer to the metadata table's start address  in pseudo metadata row buffer. */
    sys_fw_metadata_t *temp_fw_pmetadata = (sys_fw_metadata_t *)
            (temp_pseudo_metadata_buf + (CY_FLASH_SIZEOF_ROW - CCG_METADATA_TABLE_SIZE));

    /* Determine the pseudo and actual metadata row details based on FW mode. */
    if (CALL_MAP(sys_get_device_mode)() == SYS_FW_MODE_FWIMAGE_1)
    {
        p_md = gl_img1_fw_pseudo_metadata;
        alt_p_md = gl_img2_fw_pseudo_metadata;
        md = CALL_MAP(gl_img1_fw_metadata);
        alt_md = CALL_MAP(gl_img2_fw_metadata);
        p_md_row_num = CCG_IMG1_LAST_FLASH_ROW_NUM;
        alt_p_md_row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        alt_md_row_num = CCG_IMG2_METADATA_ROW_NUM;
        alt_img_bit_pos = 3;
    }
    else
    {
        p_md = gl_img2_fw_pseudo_metadata;
        alt_p_md = gl_img1_fw_pseudo_metadata;
        md = CALL_MAP(gl_img2_fw_metadata);
        alt_md = CALL_MAP(gl_img1_fw_metadata);
        alt_p_md_row_num = CALL_MAP(gl_img2_fw_metadata)->boot_last_row;
        p_md_row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        alt_md_row_num = CCG_IMG1_METADATA_ROW_NUM;
        alt_img_bit_pos = 2;
    }
    
    /* Current FW shall ensure that it's PMD row is cleared if it contains
     * valid signature. This ensures there is no stale PMD information in the flash. */
    if (p_md->metadata_valid == SYS_PSEUDO_METADATA_VALID_SIG)
    {
        flash_row_clear (p_md_row_num);
    }
        
    /*
     * Read pseudo-metadata row of other fw image. If pseudo-metadata has
     * signature "CP", which indicates that other FW Image got updated before
     * re-boot, and FW image is valid then copy this pseudo-metadta in to
     * its actual metadata row after changing its signature and
     * increamenting image pointer.
     */
    if (alt_p_md->metadata_valid == SYS_PSEUDO_METADATA_VALID_SIG)
    {
        if(boot_validate_fw (alt_p_md) == CY_PDSTACK_STAT_SUCCESS)
        {
#if (SECURE_FW_UPDATE == 1)
#if (CUSTOM_SNK_FEATURE_ENABLE)
             if(sboot_authenticate_fw(alt_p_md, alt_p_md_row_num, fw_hash, NULL) == CY_PDSTACK_STAT_SUCCESS)
#else
             if(sboot_authenticate_fw(alt_p_md, alt_p_md_row_num, fw_hash) == CY_PDSTACK_STAT_SUCCESS)
#endif /* (CUSTOM_SNK_FEATURE_ENABLE) */
#endif /* SECURE_FW_UPDATE */
           {
                /* Read pesudo metadata in a temp buffer. */
                CY_PDUTILS_MEM_COPY ((uint8_t *)temp_fw_pmetadata, (uint8_t *)alt_p_md,
                    CCG_METADATA_TABLE_SIZE);
                temp_fw_pmetadata->metadata_valid = SYS_METADATA_VALID_SIG;
                temp_fw_pmetadata->boot_seq = (md->boot_seq) + 1;
#if (SECURE_FW_UPDATE == 1)
                /* Copy FW HASH in first 32 bytes of pseudo metadata row. */
                CY_PDUTILS_MEM_COPY (temp_pseudo_metadata_buf, (uint8_t *)fw_hash,
                    CRYPTO_SHA_2_HASH_SIZE_BYTES);
#endif /* SECURE_FW_UPDATE */
                /* This flash write can always be blocking. */
                if (CYRET_SUCCESS == Cy_Flash_WriteRow (alt_md_row_num,
                        (uint32_t *)temp_pseudo_metadata_buf))
                {
                
#if APP_PRIORITY_FEATURE_ENABLE
                    flash_set_app_priority(FLASH_APP_PRIORITY_DEFAULT);
#endif
                    /*Invalidate Pseudo-metadata of FW Image 2.*/
                    flash_row_clear (alt_p_md_row_num);
                    /* Undergo Software Reset. */
                    __NVIC_SystemReset();
                }
            }
#if (SECURE_FW_UPDATE == 1)
            else
            {
                invalidate_md = true;
            }
#endif /* SECURE_FW_UPDATE */
        }
        else
        {
            invalidate_md = true;
        }
    }
    else
    {
        /* Check if other image is valid. */
        if (boot_validate_fw (alt_md) != CY_PDSTACK_STAT_SUCCESS)
        {
            /* Mark the other image as invalid. */
            gl_img_status.val |= (1 << alt_img_bit_pos);
        }
    }

    if (invalidate_md)
    {
        /*Invalidate Pseudo-metadata and Metadata of other FW Image.*/
        flash_row_clear (alt_md_row_num);
        flash_row_clear (alt_p_md_row_num);
        /* Mark the other image as invalid. */
        gl_img_status.val |= (1 << alt_img_bit_pos);
    }
}

#if (SECURE_FW_UPDATE && CUSTOM_SNK_FEATURE_ENABLE)
bool iecs_check_for_valid_fw(uint8_t *first_row)
{
    /* Temporary data buffer (of size one Flash row) to store contents of pseudo metadata. */
    uint8_t temp_metadata_buf[CY_FLASH_SIZEOF_ROW] = {0};
    
    sys_fw_metadata_t *md;
    sys_fw_metadata_t *alt_md;
    uint16_t alt_md_row_num;
    
#if (SECURE_FW_UPDATE == 1)
    /* Buffer to hold FW HASH. */
#if (MBEDTLS_SHA2 != 1)    
    uint32_t fw_hash[CRYPTO_SHA_2_HASH_SIZE_WORDS] = {0};
#else    
    uint8_t fw_hash[CRYPTO_SHA_2_HASH_SIZE_BYTES] = {0};
#endif /* (MBEDTLS_SHA2 != 1) */
#endif /* SECURE_FW_UPDATE */

    /*Pointer to the metadata table's start address  in pseudo metadata row buffer. */
    sys_fw_metadata_t *temp_fw_pmetadata = (sys_fw_metadata_t *)
            (temp_metadata_buf + (CY_FLASH_SIZEOF_ROW - CCG_METADATA_TABLE_SIZE));

    /* Determine the pseudo and actual metadata row details based on FW mode. */
    sys_fw_mode_t fw_mode = CALL_IN_FUNCTION(sys_get_device_mode)();
    
    if (fw_mode == SYS_FW_MODE_FWIMAGE_1)
    {
        md = GET_IN_VARIABLE(gl_img1_fw_metadata);
        alt_md = GET_IN_VARIABLE(gl_img2_fw_metadata);
        alt_md_row_num = CCG_IMG2_METADATA_ROW_NUM;
    }
    else
    {
        md = GET_IN_VARIABLE(gl_img2_fw_metadata);
        alt_md = GET_IN_VARIABLE(gl_img1_fw_metadata);
        alt_md_row_num = CCG_IMG1_METADATA_ROW_NUM;
    }
        
    if(boot_handle_validate_fw_cmd(fw_mode) == CY_PDSTACK_STAT_SUCCESS)
    {
#if (SECURE_FW_UPDATE == 1)
       if(sboot_authenticate_fw(alt_md, alt_md_row_num, fw_hash, first_row) == CY_PDSTACK_STAT_SUCCESS)
#endif /* SECURE_FW_UPDATE */
       {
            /* Read pesudo metadata in a temp buffer. */
            CY_PDUTILS_MEM_COPY ((uint8_t *)temp_fw_pmetadata, (uint8_t *)alt_md, CCG_METADATA_TABLE_SIZE);
            temp_fw_pmetadata->metadata_valid = SYS_METADATA_VALID_SIG;
            temp_fw_pmetadata->boot_seq = (md->boot_seq) + 1;

#if (SECURE_FW_UPDATE == 1)
            /* Copy FW HASH in first 32 bytes of pseudo metadata row. */
            CY_PDUTILS_MEM_COPY (temp_metadata_buf, (uint8_t *)fw_hash, CRYPTO_SHA_2_HASH_SIZE_BYTES);
#endif /* SECURE_FW_UPDATE */
            /* This flash write can always be blocking. */
            if (CYRET_SUCCESS == CySysFlashWriteRow (alt_md_row_num, temp_metadata_buf))
            {
            
#if APP_PRIORITY_FEATURE_ENABLE
                flash_set_app_priority(FLASH_APP_PRIORITY_DEFAULT);
#endif
                return true;
            }
        }
    }
    return false;
}
#endif /* #if (SECURE_FW_UPDATE && CUSTOM_SNK_FEATURE_ENABLE) */
#endif /* CCG_PSEUDO_METADATA_DISABLE */
#endif /* CCG_BOOT */

#if !USE_CYACD2_METADATA_FORMAT
uint32_t boot_get_boot_seq(uint8_t fwid)
{
    sys_fw_metadata_t *md_p = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
#if (CCG_BOOT == 0u)
    cy_en_pdstack_status_t fw_valid_stat;
#endif /* (CCG_BOOT == 0u) */

#if (!CCG_DUALAPP_DISABLE)
    if (fwid == SYS_FW_MODE_FWIMAGE_2)
    {
        md_p = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
    }
#endif /* (!CCG_DUALAPP_DISABLE) */

#if (CCG_BOOT != 0)
    if (boot_validate_fw (md_p) == CY_PDSTACK_STAT_SUCCESS)
    {
        return (md_p->boot_seq);
    }
#else /* (CCG_BOOT != 0) */
    fw_valid_stat = boot_validate_fw (md_p);
    /* We only need to validate if the target is not the active firmware. */
    if ((CALL_MAP(sys_get_device_mode)() == (sys_fw_mode_t)fwid) || (fw_valid_stat == CY_PDSTACK_STAT_SUCCESS))
    {
        return (md_p->boot_seq);
    }
#endif /* (CCG_BOOT != 0) */
    CY_UNUSED_PARAMETER(fwid);
    return 0;
}
#endif /* ! USE_CYACD2_METADATA_FORMAT */

/* Pointer to function that is used to jump into address */
typedef void (*cy_fn_dfu_jump_ptr_t)(void);

static void SwitchToApp(uint32_t stackPointer, uint32_t address)
{
    __set_MSP(stackPointer);
CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.1','Casting int to a function pointer is safe as it is guaranteed to have a valid address.');
    ((cy_fn_dfu_jump_ptr_t) address) ();

    /* This function does not return */
    for(;;)
    {
    }
}

void boot_jump_to_app(uint32_t appId)
{
    sys_fw_metadata_t *md_p = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
    
    (void)appId;

#if USE_CYACD2_METADATA_FORMAT
    uint32_t fw_start = md_p->fw_start;
#else
    uint32_t fw_start = ((md_p->boot_last_row << CCG_FLASH_ROW_SHIFT_NUM)
                + CCG_FLASH_ROW_SIZE);
#endif /* USE_CYACD2_METADATA_FORMAT */

    uint32_t stackPointer = ((uint32_t *)fw_start)[0]; /* The Stack Pointer of the app to switch to */
    uint32_t resetHandler = ((uint32_t *)fw_start)[1]; /* Reset_Handler() address */

    cyBtldrRunType = 0U;
    SwitchToApp(stackPointer, resetHandler);
}
/* [] END OF FILE */
