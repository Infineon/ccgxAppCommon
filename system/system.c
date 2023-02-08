/******************************************************************************
* File Name: system.c
* \version 2.0
*
* Description: Source file for boot and flash write support
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include "stdbool.h"
#include "stdint.h"
#include "config.h"
#include "cy_pdstack_common.h"
#include "flash.h"
#include "boot.h"
#include "system.h"
#include "srom.h"

#if (!SROM_CODE_SYS_SYS)

/* Invalid FW Version. */
uint8_t gl_invalid_version[8] = {0};

/* Variable representing the current firmware mode. */
sys_fw_mode_t gl_active_fw = SYS_FW_MODE_INVALID;

#endif /* (!SROM_CODE_SYS_SYS) */

ATTRIBUTES_SYS_SYS void sys_set_device_mode(sys_fw_mode_t fw_mode)
{
    CALL_MAP(gl_active_fw) = fw_mode;
}    

ATTRIBUTES_SYS_SYS sys_fw_mode_t sys_get_device_mode(void)
{
#if (CCG_BOOT == 1)
    return SYS_FW_MODE_BOOTLOADER;
#else /* Firmware */
    return (CALL_MAP(gl_active_fw));
#endif /* CCG_BOOT */
}

#if !CCG_FIRMWARE_APP_ONLY
ATTRIBUTES_SYS_SYS static uint8_t* sys_get_fw_version(sys_fw_metadata_t *fw_metadata)
{
    /*
     * FW stores its version at an offset of CY_PD_FW_VERSION_OFFSET
     * from its start address. Read the version from that location.
     * The start address of FW is determined from the boot_last_row
     * field of the FW metadata.
     */
#if USE_CYACD2_METADATA_FORMAT
    uint32_t addr = fw_metadata->fw_start + SYS_FW_VERSION_OFFSET;
#else
    uint32_t addr = ((((uint32_t)fw_metadata->boot_last_row) << CCG_FLASH_ROW_SHIFT_NUM)
            + CY_FLASH_SIZEOF_ROW + SYS_FW_VERSION_OFFSET);
#endif /* USE_CYACD2_METADATA_FORMAT */
    return (uint8_t *)addr;
}
#endif /* CCG_FIRMWARE_APP_ONLY */

ATTRIBUTES_SYS_SYS uint8_t* sys_get_boot_version(void)
{
    return (uint8_t *)SYS_BOOT_VERSION_ADDRESS;
}

ATTRIBUTES_SYS_SYS uint8_t* sys_get_img1_fw_version(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    /* Check if Image 1 is valid. */
    if ((CALL_MAP(get_boot_mode_reason)()).status.fw1_invalid == 0u)
    {
        return sys_get_fw_version (CALL_MAP(gl_img1_fw_metadata));
    }
    else
    {
        return (CALL_MAP(gl_invalid_version));
    }
#else
    return (CALL_MAP(gl_invalid_version));
#endif /* CCG_FIRMWARE_APP_ONLY */
}

#if (!CCG_DUALAPP_DISABLE)    
ATTRIBUTES_SYS_SYS uint8_t* sys_get_img2_fw_version(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    /* Check if Image 2 is valid. */
    if ((CALL_MAP(get_boot_mode_reason)()).status.fw2_invalid == 0)
    {
        return sys_get_fw_version (CALL_MAP(gl_img2_fw_metadata));
    }
    else
    {
    return (CALL_MAP(gl_invalid_version));
    }
#else
    return (CALL_MAP(gl_invalid_version));
#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* (!CCG_DUALAPP_DISABLE) */

ATTRIBUTES_SYS_SYS uint32_t sys_get_fw_img1_start_addr(void)
{
#if !CCG_FIRMWARE_APP_ONLY
#if !USE_CYACD2_METADATA_FORMAT
    uint32_t bl_last;
#endif /* !USE_CYACD2_METADATA_FORMAT */

    /* Check if Image 1 is valid. */
    if (((CALL_MAP(get_boot_mode_reason)()).status.fw1_invalid) != 1u)
    {        
#if USE_CYACD2_METADATA_FORMAT
        return CALL_MAP(gl_img1_fw_metadata)->fw_start;
#else
        bl_last = CALL_MAP(gl_img1_fw_metadata)->boot_last_row;
        return ((bl_last << CCG_FLASH_ROW_SHIFT_NUM) + CY_FLASH_SIZEOF_ROW);
#endif /* USE_CYACD2_METADATA_FORMAT */
    }
    else
    {
        /* Image 1 is invalid. Can't use MD Row info. Use Boot last row info
         * from flash configuration and response. */
        return ((((uint32_t)CCG_BOOT_LOADER_LAST_ROW) << CCG_FLASH_ROW_SHIFT_NUM)
                + CY_FLASH_SIZEOF_ROW);
    }
#else
    return SYS_INVALID_FW_START_ADDR;
#endif /* CCG_FIRMWARE_APP_ONLY */
}

#if (!CCG_DUALAPP_DISABLE) 
ATTRIBUTES_SYS_SYS uint32_t sys_get_fw_img2_start_addr(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    uint32_t bl_last;

    /* Check if Image 2 is valid. */
    if (((CALL_MAP(get_boot_mode_reason)()).status.fw2_invalid) != 1)
    {
        bl_last = CALL_MAP(gl_img2_fw_metadata)->boot_last_row;
        return ((bl_last << CCG_FLASH_ROW_SHIFT_NUM) + CY_FLASH_SIZEOF_ROW);
    }
    else
    {
        /* Image 2 is invalid. Can't use MD Row info. Use last row FW1 info info
         * from flash configuration and response. */
        return ((CCG_IMG1_LAST_FLASH_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM)
                + CY_FLASH_SIZEOF_ROW);
    }
#else
    return SYS_INVALID_FW_START_ADDR;
#endif /* CCG_FIRMWARE_APP_ONLY */
}

ATTRIBUTES_SYS_SYS uint8_t sys_get_recent_fw_image(void)
{
#if (APP_PRIORITY_FEATURE_ENABLE == 1)
    /* Read APP Priority field to determine which image gets the priority. */
    uint8_t app_priority;

    app_priority = *((uint8_t *)(CCG_APP_PRIORITY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM));
    if (app_priority != 0)
    {
        if (app_priority == 0x01)
            return SYS_FW_MODE_FWIMAGE_1;
        else
            return SYS_FW_MODE_FWIMAGE_2;
    }
    else
#endif /* !APP_PRIORITY_FEATURE_ENABLE */
    {
        /* Prioritize FW2 over FW1 if no priority is defined. */
        if ((CALL_MAP(gl_img2_fw_metadata))->boot_seq >= (CALL_MAP(gl_img1_fw_metadata))->boot_seq)
            return SYS_FW_MODE_FWIMAGE_2;
        else
            return SYS_FW_MODE_FWIMAGE_1;
    }
}
#endif /* (!CCG_DUALAPP_DISABLE) */

ATTRIBUTES_SYS_SYS void sys_get_silicon_id(uint32_t *silicon_id)
{
    /* Read Silicon ID. */
    (void)silicon_id;
#if (!PSVP_FPGA_ENABLE)
    *silicon_id = SFLASH_SILICON_ID;
#else
    *silicon_id = 0x3100u;
#endif
}

ATTRIBUTES_SYS_SYS uint8_t get_silicon_revision(void)
{

    uint8_t srev;

    /* Silicon revision is stored in core-sight tables.
     * Major revision = ROMTABLE_PID2(7:4)
     * Minor revision = ROMTABLE_PID3(7:4) */
    srev = (uint8_t)((ROMTABLE_PID2) & 0xF0u);
    srev |= (uint8_t)(((ROMTABLE_PID3) & 0xF0u) >> 4u);
    return srev;
}

ATTRIBUTES_SYS_SYS uint32_t sys_get_custom_info_addr(void)
{
    uint32_t addr;

    /* Get FW start address based on FW image. */
#if (CCG_DUALAPP_DISABLE)
    addr = CALL_MAP(sys_get_fw_img1_start_addr)();
#else /* (!CCG_DUALAPP_DISABLE) */
    if (CALL_MAP(sys_get_device_mode)() == SYS_FW_MODE_FWIMAGE_1)
    {
        addr = CALL_MAP(sys_get_fw_img1_start_addr)();
    }
    else
    {
        addr = CALL_MAP(sys_get_fw_img2_start_addr)();
    }
#endif /* (!CCG_DUALAPP_DISABLE) */

    return (addr + SYS_FW_CUSTOM_INFO_OFFSET);
}

ATTRIBUTES_SYS_SYS uint16_t sys_get_bcdDevice_version(uint32_t ver_addr)
{
    /*     
     * bcdDevice version is defined and derived from FW version as follows:
     * Bit 0:3   = FW_MINOR_VERSION
     * Bit 4:6   = FW_MAJOR_VERSION (lower 3 bits)
     * Bit 7:13  = APP_EXT_CIR_NUM (lower 7 bits)
     * Bit 14:15 = APP_MAJOR_VERSION (lower 2 bits)
     * FW version is stored at a fixed offset in FLASH in this format:
     * 4 Byte BASE_VERSION, 4 Byte APP_VERSION
     */
    
    uint8_t base_major_minor = *((uint8_t *)(ver_addr + 3u));
    uint8_t app_cir_num = *((uint8_t *)(ver_addr + 6u));
    uint8_t app_major_minor = *((uint8_t *)(ver_addr + 7u));
    
    return ((((uint16_t)base_major_minor) & 0x7Fu) | ((((uint16_t)app_cir_num) & 0x7Fu) << 7u)
        | ((((uint16_t)app_major_minor) & 0x30u) << 10u));
}

/* [] END OF FILE */
