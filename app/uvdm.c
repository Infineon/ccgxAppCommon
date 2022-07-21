/***************************************************************************//**
* \file uvdm.c
* \version 1.1.0 
*
* Unstructured VDM handler source file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "cy_pdstack_common.h"
#include "cy_flash.h"
#include "system.h"
#include "cy_pdstack_utils.h"
#include "flash.h"
#include "boot.h"
#include "uvdm.h"
#include "srom.h"
#include "app.h"
#include "cy_usbpd_config_table.h"
#if QC_PPS_ENABLE
#include "thermistor.h"
#endif /* QC_PPS_ENABLE */

/* Global variable to keep track of solution level functions */
extern app_sln_handler_t *solution_fn_handler;

#if AUTH_BOOT
#include "fw_update.h"    
#endif /* AUTH_BOOT */

#if (SECURE_FW_UPDATE == 1)
#include <crypto_hal.h>
#include <secure_boot.h>
#endif /* SECURE_FW_UPDATE */

#if QC_PPS_ENABLE
#include "cy_pdstack_dpm.h"
#include "app.h"
#include "thermistor.h"
#endif /* QC_PPS_ENABLE */

#if SYS_BLACK_BOX_ENABLE
#include "blackbox.h"
#endif /* SYS_BLACK_BOX_ENABLE */

#if TEST_CODE_COVERAGE /* skip_tcc */
#include "tcc.h" /* skip_tcc */
#endif /* TEST_CODE_COVERAGE */ /* skip_tcc */

/*
 * Following macros are used for data memory write and read operations.
 * Note that CCG3 has 128 bytes flash row and CCG4 has 256 bytes flash row
 * size. Hence, Macros are defined based on size of flash row.
 */

/* Index of first section of Data Buffer. */
#define UVDM_DATA_BUF_FIRST_SEC_INDEX                           (0x01u)

/* Size of first five sections of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_SECTION_SIZE_24                        (24u)

#if (CY_FLASH_SIZEOF_ROW == 128u)
/* Index of last section of Data Buffer. */
#define UVDM_DATA_BUF_LAST_SEC_INDEX                            (0x06u)

/* Number of Data Memory Sections. */
#define UVDM_NUM_DATA_MEMORY_SECTIONS                           (0x06u)

/* Size of last section of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_LAST_SECTION_SIZE                      (0x08u)

/*
 * Mask to determine if all sections are written to before FLASH_WRITE
 * command. One bit represents one section in the data buffer.
 */
#define UVDM_SEND_DATA_SECTION_UPDATE_MASK                      (0x003Fu)

#elif (CY_FLASH_SIZEOF_ROW == 256u)
/* Index of last section of Data Buffer. */
#define UVDM_DATA_BUF_LAST_SEC_INDEX                            (0x0Bu)

/* Number of Data Memory Sections. */
#define UVDM_NUM_DATA_MEMORY_SECTIONS                           (0x06u)

/* Size of last section of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_LAST_SECTION_SIZE                      (0x10u)

/*
 * Mask to determine if all sections are written to before FLASH_WRITE
 * command. One bit represents one section in the data buffer.
 */
#define UVDM_SEND_DATA_SECTION_UPDATE_MASK                      (0x07FFu)
#endif /* CY_FLASH_SIZEOF_ROW */

/* Array of VDOs to hold response VDM. */
static cy_pd_pd_do_t gl_response_vdo[UVDM_RESPONSE_MAX_NO_OF_VDO];

void CyGetUniqueId(uint32* uniqueId)
{
    /* Need to review the function */
    (void)uniqueId;
}

/*
 * Data buffer (of size one Flash row) to store data related
 * with flashing commands. Align it at Flash row size bytes boundary.
 */
typedef struct
{
    uint8_t srom_api_param[SROM_API_PARAM_SIZE];
    uint8_t data_buf[CY_FLASH_SIZEOF_ROW];
} flash_mem_data_t;

static flash_mem_data_t gl_data_buf __attribute__((aligned(CY_FLASH_SIZEOF_ROW)));

/* Current write pointer in the data buffer. */
static uint16_t gl_write_data_pointer = 0;

/* Current read pointer in the data buffer. */
static uint8_t gl_read_data_pointer = 0;

/*
 * In Non-Blocking Flash update case, this flag ensures that
 * system handles only one FLASH WRITE request at a time.
 */
static volatile bool gl_flash_write_cmd_pending = false;

#if (CCG_BOOT == 0)
/*
 * This is the variable which is used to keep track of SEND_DATA VDMs.
 * Basically, expectation is that all sections of data buffer are updated
 * through SEND_DATA UVDM Command before a FLASH_WRITE Command.
 * Usage: Each bit in this variable corresponds to one section of data buffer.
 * Clear the correspondig bit when that section is updated. If this variable is
 * zero when FLASH_WRITE request comes in, it ensures that all sections were
 * Restore, the value back after FLASH_WRITE command completes.
 */
static uint16_t gl_send_data_section_tracker = UVDM_SEND_DATA_SECTION_UPDATE_MASK;

/* Current Non blocking VDM Command. */
static uvdm_cmd_opcode_t gl_cur_nb_cmd = UVDM_CMD_RESERVED;

uvdm_cmd_opcode_t uvdm_get_cur_nb_cmd(void)
{
    return gl_cur_nb_cmd;
}

void uvdm_reset_nb_cmd_state(void)
{
    /*
     * This marks the end of FLASH WRITE Command (Successful or Abort).
     * CCG can handle the next FLASH WRITE command from now.
     */
    gl_flash_write_cmd_pending = false;

    if (gl_cur_nb_cmd == UVDM_CMD_FLASH_WRITE_OPCODE)
    {
        /* Revert back SEND_DATA UVDM tracker. */
        gl_send_data_section_tracker = UVDM_SEND_DATA_SECTION_UPDATE_MASK;
    }
}
#endif /* (CCG_BOOT == 0) */

#if (PSVP_FPGA_ENABLE) && (!CCG_BOOT)
/*
 * The function returns the coverage data in encoded form.
 * Each bit in the coverage data holds information about DWORD.
 *
 * @param buffer Buffer to return the encoded return data.
 * @param offset Offset in terms of 8-DWORD count. For example
 *      for address 0x10002020, the offset is 1. Caller should
 *      ensure not to make unaligned access.
 * @param size Size or status read in 8-DWORD count. For 32byte
 *      status, this should be 1. Caller should ensure not to
 *      make unaligned access. Also, the size should not exceed
 *      the available buffering.
 */
void rom_coverage_get_data(uint8_t *buffer, uint16_t offset, uint8_t size)
{
    uint16_t i, j, start_loc, end_loc;
    volatile uint32_t *wr_ptr, *rd_ptr;
    wr_ptr = (volatile uint32_t *)0x401000C8;
    rd_ptr = (volatile uint32_t *)0x401000CC;

    /*
     * Since the start address need to map to DWORD address,
     * we need to multiply the offset and size by 8.
     * The information received is a bit and needs to be shifted into
     * the appropriate byte of the buffer.
     */
    start_loc = offset << 3;
    end_loc = start_loc + (size << 3);

    /*
     * i runs from actual ROM D-WORD offset to required size.
     * j runs from 0 to size. This infomration needs to be loaded
     * into each byte of buffer. Here j / 8 is used as buffer byte
     * offset and j % 8 is used as buffer bit offset in the byte.
     */
    for(i = start_loc, j = 0; i < end_loc; i++, j++)
    {
        *wr_ptr = i;
        Cy_SysLib_DelayUs(50);
        if(*rd_ptr & 0x01)
        {
            buffer[j >> 3] |= 1 << (j & 0x07);
        }
        else
        {
            buffer[j >> 3] &= ~(1 << (j & 0x07));
        }
    }
}
#endif /* (PSVP_FPGA_ENABLE) && (!CCG_BOOT) */

static bool uvdm_exit_flashing_mode(void)
{
#if (CCG_BOOT == 0)
    /* Check if CY Alternate mode was used to enable Flashing mode. If yes, exit Flashing mode. */
    if (flash_access_get_status (1u << (uint8_t)FLASH_IF_UVDM))
#endif /* (CCG_BOOT == 0) */
    {
        flash_enter_mode(false, FLASH_IF_UVDM, true);
#if (FLASH_ENABLE_NB_MODE == 1)
        /* Check if a non-blocking flash write is active. If yes, start an abort operation. */
        if (gl_flash_write_cmd_pending)
        {
            /* Signal Flash module to abort the current flash write operation. */
            flash_non_blocking_write_abort ();
        }
#endif /* FLASH_ENABLE_NB_MODE */
        /* Valid request. */
        return true;
    }
#if (CCG_BOOT == 0)
    else
    {
        /* This is an invalid request. */
        return false;
    }
#endif /* (CCG_BOOT == 0) */
}

#if (CCG_BOOT == 0)
/* Flag to indicate CY Flashing mode entry. */
static volatile bool gl_cy_flashing_mode = false;

void uvdm_enter_cy_alt_mode(void)
{
    gl_cy_flashing_mode = true;
}

void uvdm_exit_cy_alt_mode(void)
{
    /* Exit CY Alternate Mode. */
    gl_cy_flashing_mode = false;
    (void)uvdm_exit_flashing_mode ();
}

bool uvdm_get_flashing_mode(void)
{
    /* Return the state of CY flashing mode. */
    return gl_cy_flashing_mode;
}
#endif /* CCG_BOOT */

/*
 * Handles SEND_DATA command. Copies data into the data buffer.
 * Sequnce number determines which section of data buffer to update.
 *
 * @param data Pointer to data array
 * @param size Size of data in bytes
 * @param seqNum Sequnce number of data
 * @return bool
 */
static bool uvdm_update_data_memory(uint8_t* data, uint8_t size, uint8_t seqNum)
{
    /* Ensure Sequence number is valid. */
    if ((seqNum < UVDM_DATA_BUF_FIRST_SEC_INDEX) ||
            (seqNum > UVDM_DATA_BUF_LAST_SEC_INDEX))
    {
        return false;
    }

    /* Calculate the write data pointer based on sequence number. */
    gl_write_data_pointer = ((uint16_t)seqNum - UVDM_DATA_BUF_FIRST_SEC_INDEX)
                * UVDM_DATA_BUFFER_SECTION_SIZE_24;

    /* Ensure the size of data for section 1 to 5 is 24 bytes. */
    if (((seqNum < UVDM_DATA_BUF_LAST_SEC_INDEX) && (size != UVDM_DATA_BUFFER_SECTION_SIZE_24))
        || ((seqNum == UVDM_DATA_BUF_LAST_SEC_INDEX) && (size != UVDM_DATA_BUFFER_LAST_SECTION_SIZE)))
    {
        return false;
    }

    /* Copy data. */
    mem_copy (&gl_data_buf.data_buf[gl_write_data_pointer], data, size);

#if (CCG_BOOT == 0)
    /* Updated SEND_DATA UVDM tracker. */
    gl_send_data_section_tracker &= (uint16_t)(~((uint16_t)1u << (seqNum - 1u)));
#endif /* (CCG_BOOT == 0) */
    return true;
}

#if (SECURE_FW_UPDATE == 0)
/*
 * Handles READ_DATA command. Reads data from the data buffer based on
 * sequence number.
 *
 * @param data Pointer to data array
 * @param size Size of data in bytes
 * @return NONE
 */
static bool uvdm_read_data_memory(uint8_t* data, uint8_t seqNum)
{
    uint8_t size;

    /* Ensure Sequence number is valid. */
    if ((seqNum < UVDM_DATA_BUF_FIRST_SEC_INDEX) ||
            (seqNum > UVDM_DATA_BUF_LAST_SEC_INDEX))
    {
        return false;
    }

    /* Calculate the read data pointer based on sequence number. */
    gl_read_data_pointer = (seqNum - UVDM_DATA_BUF_FIRST_SEC_INDEX)*
                UVDM_DATA_BUFFER_SECTION_SIZE_24;

    /* Get the size of data to read based on sequence number. */
    if (seqNum < UVDM_DATA_BUF_LAST_SEC_INDEX)
    {
        size = UVDM_DATA_BUFFER_SECTION_SIZE_24;
    }
    else
    {
        size = UVDM_DATA_BUFFER_LAST_SECTION_SIZE;
    }

    /* Copy data. */
    mem_copy (data, &gl_data_buf.data_buf[gl_read_data_pointer], size);
    return true;
}
#endif /* SECURE_FW_UPDATE */

uvdm_response_state_t uvdm_handle_cmd(cy_stc_pdstack_context_t * context, uint32_t *rx_pkt, cy_pd_pd_do_t **vdm_rspn_pkt,
    uint8_t *vdo_count, flash_cbk_t flash_cb)
{
    cy_en_pdstack_status_t response_code = CY_PDSTACK_STAT_NO_RESPONSE;
    uvdm_response_state_t response_state = UVDM_HANDLED_RESPONSE_READY;
    uint8_t *cmd_param;
    uvdm_cmd_opcode_t cmd_opcode;
    uint8_t cmd_length, seqNum;
    uint16_t row_num;
#if (SECURE_FW_UPDATE == 0u)
    bool stat;
#endif /* (SECURE_FW_UPDATE == 0u) */
    cy_pd_pd_do_t* d_obj = (cy_pd_pd_do_t *)((uint32_t)(&(rx_pkt[1])));
#if AUTH_BOOT    
    bool is_handled = false;
#endif
#if (CCG_BOOT == 0)
    if (0u == pd_get_ptr_app_tbl(context->ptrUsbPdContext)->flashing_vid)
    {
        return UVDM_NOT_HANDLED;
    }
    /* Check if SVID is Flashing VID. */
    if (d_obj->ustd_vdm_hdr.svid != pd_get_ptr_app_tbl(context->ptrUsbPdContext)->flashing_vid)
#else
    /* In boot mode, Check if SVID is Cypress VID. */
    if (d_obj->ustd_vdm_hdr.svid != CY_PD_CY_VID)
#endif /* CCG_BOOT */
    {
        /* Can't handle the VDM. */
        return UVDM_NOT_HANDLED;
    }

    /* Set up parameters. */
    cmd_param = (uint8_t *)&rx_pkt[2];
    cmd_opcode = (uvdm_cmd_opcode_t)d_obj->ustd_vdm_hdr.cmd;
    cmd_length = ((uint8_t)CY_PD_GET_PD_HDR_CNT((rx_pkt[0])) - 1u) << 2;
    seqNum = d_obj->ustd_vdm_hdr.seqNum;

    /*
     * Atleast one VDO needs to be sent back as response. Increment
     * this number as required by response.
     */
    uint8_t no_of_vdo = 1;

    /* Fill the header VDO with Cypress VID/Flashing VID and Responder ACK. */
#if (CCG_BOOT == 0)
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.svid = pd_get_ptr_app_tbl(context->ptrUsbPdContext)->flashing_vid;
#else
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.svid = CY_PD_CY_VID;
#endif /* CCG_BOOT */
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_ACK;
    /* Ensure sequence number field is updated in response header. */
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.seqNum = d_obj->ustd_vdm_hdr.seqNum;

#if (AUTH_BOOT && CC_BOOT)
    /* Invoke the signed firmware update state machine */
    signed_fw_update_fsm((uint8_t)cmd_opcode, cmd_length, cmd_param, gl_response_vdo, &response_code,
                                gl_data_buf.data_buf, &is_handled);
#endif /* AUTH_BOOT */

#if (AUTH_BOOT && CC_BOOT)
    if(false == is_handled)
#endif    
    {
        /* Handle command. */
        switch (cmd_opcode)
        {
            case UVDM_CMD_GET_DEVICE_MODE_OPCODE:
                /*
                 * Make sure cmd_length is 0 in this case, as no command specific
                 * Data object is expected.
                 */
                if (cmd_length == 0u)
                {
                    *((uint32_t *)((uint32_t)&(gl_response_vdo[UVDM_DEVICE_MODE_VDO_INDEX])))
                        =  (uint32_t)sys_get_device_mode ();
                    no_of_vdo++;
                    /*
                     * If DEVICE MODE is bootloader, append
                     * bootloader last flash row in the second
                     * VDO.
                     */
                    if (gl_response_vdo[UVDM_DEVICE_MODE_VDO_INDEX].val
                            == (uint32_t)SYS_FW_MODE_BOOTLOADER)
                    {
                        gl_response_vdo[UVDM_BOOT_LAST_ROW_VDO_INDEX].val
                            = CCG_BOOT_LOADER_LAST_ROW;
                        no_of_vdo++;
                    }
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType =
                        (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

                /* Return the version of all three images. */
            case UVDM_CMD_GET_DEVICE_VERSION_OPCODE:
                /*
                 * Make sure cmd_length is 0 in this case, as no command specific
                 * Data object is expected.
                 */
                if (cmd_length == 0u)
                {
                    /* Get boot loader version. */
                    mem_copy ((uint8_t *)(&gl_response_vdo[UVDM_BOOT_VERSION_VDO_INDEX]),
                            sys_get_boot_version(), UVDM_VERSION_NUM_SIZE_BYTES);
                    mem_copy ((uint8_t *)(&gl_response_vdo[UVDM_IMG1_VERSION_VDO_INDEX]),
                            sys_get_img1_fw_version(), UVDM_VERSION_NUM_SIZE_BYTES);
#if (!CCG_DUALAPP_DISABLE)
                    MEM_COPY ((uint8_t *)(&gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX]),
                            sys_get_img2_fw_version(), UVDM_VERSION_NUM_SIZE_BYTES);
#else
                    gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX].val = 0;
                    gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX + 1u].val = 0;
#endif
                    no_of_vdo += UVDM_GET_VERSION_U_VDM_NO_OF_VDO;
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

            case UVDM_CMD_GET_SILICON_ID_OPCODE:
                /* Validate command length and command signature. */
                if ((cmd_length == UVDM_GET_SILICON_ID_CMD_SIZE) &&
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                         UVDM_GET_SILICON_ID_CMD_SIG))
                {
                    /* Read Silicon ID. */
                    sys_get_silicon_id ((uint32_t *)
                            ((uint32_t)&gl_response_vdo[UVDM_SILICON_ID_VDO_INDEX]));
                    /* Read UUID.
                     * Note: Use uint32* instead of uint32_t* as this is what the CyGetUniqueId() API uses.
                     */
                    CyGetUniqueId ((uint32 *)((uint32_t)&gl_response_vdo[UVDM_SILICON_ID_VDO_INDEX + 1u]));
                    /*
                     * Increment the VDO number to send the VDO containing Silicon ID
                     * and 2 VDOs for UUID.
                     */
                    no_of_vdo += 3u;
                    response_code = CY_PDSTACK_STAT_SUCCESS;
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

            case UVDM_CMD_DEVICE_RESET_OPCODE:
                /* Validate command length and command signature. */
                if ((cmd_length == UVDM_DEVICE_RESET_CMD_SIZE) &&
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                         UVDM_DEVICE_RESET_CMD_SIG))
                {
#if CCG_BOOT
                    response_code = uvdm_handle_device_reset (0);
#else
                    response_code = solution_fn_handler->uvdm_handle_device_reset (0);
#endif
                    /* Check if no response. */
                    if (response_code == CY_PDSTACK_STAT_NO_RESPONSE)
                    {
                        /* No VDM response. */
                        response_state = UVDM_HANDLED_NO_RESPONSE;
                    }
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

#if (CCG_BOOT == 0) && (CCG_FIRMWARE_APP_ONLY == 0)
            case UVDM_CMD_JUMP_TO_BOOT_OPCDOE:
#if OTP_VSAFE_5V_ENABLE
                /* If OTP fault is active, do not allow jump to boot */
                if(app_otp_status(0) == true)
                {
                    break;
                }
#endif /* OTP_VSAFE_5V_ENABLE */
                /* Validate Command Length. */
                if (cmd_length == UVDM_JUMP_TO_BOOT_CMD_SIZE)
                {
                    /* Check if JUMP TO BOOT Request. */
                    if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                            (uint8_t)UVDM_JUMP_TO_BOOT_CMD_SIG)
                    {
#if !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D))
                        /*
                         * We can branch to bootloader only if Bootloader has a FW
                         * update interface and does not support secure boot feature.
                         * Bootlaoder stores this info (BOOT_TYPE) at a fixed offset in
                         * flash. Take decision based on that.
                         */
                        if (((*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET)) & (SYS_BOOT_TYPE_SECURE_BOOT_MASK
                                    | SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_MASK)) != 0u)
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                        }
                        else
#endif /* !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D)) */                        
                        {
                            /* This function does not return. */
                            response_code = solution_fn_handler->uvdm_handle_device_reset (CCG_BOOT_MODE_RQT_SIG);
                        }
                    }
#if (!CCG_DUALAPP_DISABLE)
                    /* Check if JUMP TO ALT Image Request. */
                    else if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                            UVDM_JUMP_TO_ALT_FW_SIG)
                    {
#if (FLASH_ENABLE_NB_MODE == 0)
                        sys_fw_metadata_t *addr;
                        uint32_t bootsig;

                        /* If secure boot is supported, don't handle this request. */
                        if ((*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET)) &
                                (SYS_BOOT_TYPE_SECURE_BOOT_MASK))
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                        }
                        else
                        {
                            if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
                            {
                                addr    = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
                                bootsig = CCG_FW2_BOOT_RQT_SIG;
                            }
                            else
                            {
                                addr    = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
                                bootsig = CCG_FW1_BOOT_RQT_SIG;
                            }

                            if (boot_validate_fw (addr) != CY_PDSTACK_STAT_SUCCESS)
                            {
                                response_code = CY_PDSTACK_STAT_INVALID_FW;
                            }
                            else
                            {
                                response_code = CY_PDSTACK_STAT_SUCCESS;
                            }

                            /* If the alternate firmware is valid, we can proceed. */
                            if (response_code == CY_PDSTACK_STAT_SUCCESS)
                            {
                                /* Jump to alternate image. */
                                response_code = solution_fn_handler->uvdm_handle_device_reset (bootsig);
                            }
                        }
#else
                        /*
                         * Don't support JUMP_TO_ALT image command if non blocking
                         * FW update feature is enabled.
                         */
                        response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
#endif /* (FLASH_ENABLE_NB_MODE == 0) */
                    }
#endif /* (!CCG_DUALAPP_DISABLE) */
                    else
                    {
                        response_code = CY_PDSTACK_STAT_INVALID_SIGNATURE;
                    }

                    /* Check if no response. */
                    if (response_code == CY_PDSTACK_STAT_NO_RESPONSE)
                    {
                        /* No VDM response. */
                        response_state = UVDM_HANDLED_NO_RESPONSE;
                    }
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;
#endif /* (CCG_BOOT == 0) && (CCG_FIRMWARE_APP_ONLY == 0) */

            case UVDM_CMD_ENTER_FLASHING_MODE_OPCODE:
                /* Validate Command Length */
                if (cmd_length == UVDM_ENTER_FLASHING_MODE_CMD_SIZE)
                {
                    /* Check if request is to Enable Flashing Mode. */
                    if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                            UVDM_ENTER_FLASHING_MODE_CMD_SIG)
                    {
                        /* Check if Flashing Mode is already enabled. */
                        if (flash_access_get_status (~((uint8_t)1u << (uint8_t)FLASH_IF_UVDM)))
                        {
                            /* This is an invalid command. */
                            response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                        }
                        else
                        {
                            flash_enter_mode(true, FLASH_IF_UVDM, true);
                            response_code = CY_PDSTACK_STAT_SUCCESS;
                        }
                    }
                    /* Request is to Exit Flashing Mode. */
                    else
                    {
                        /*
                         * If flashing mode is enable and CC was the active
                         * interface responsible for it, exit the mode and send
                         * success. Otherwise, send failure.
                         */
#if (CCG_BOOT == 0)
                        /* Since the function always returns true if CCG_BOOT is 1, the function call inside if 
                         * condition is not required. */
                        if (uvdm_exit_flashing_mode ())
#else
                        (void)uvdm_exit_flashing_mode ();
#endif /* (CCG_BOOT == 1) */
                        {
                            response_code = CY_PDSTACK_STAT_SUCCESS;
                        }
#if (CCG_BOOT == 0)
                        else
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
                        }
#endif /* (CCG_BOOT == 0) */
                    }
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

            case UVDM_CMD_SEND_DATA_OPCODE:
                /* Invoke update data memory function and pass the sequence number */
                if (true == uvdm_update_data_memory (cmd_param, cmd_length, seqNum))
                {
                    /* No response to this VDM. */
                    response_state = UVDM_HANDLED_NO_RESPONSE;
                }
                else
                {
                    /*
                     * Set the NAK field in the header VDO as the command
                     * paramters are not correct.
                     */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

            case UVDM_CMD_FLASH_WRITE_OPCODE:
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected. Also ensure that there is no pending
                 * FLASH WRITE command in the system.
                 */
                stat = gl_flash_write_cmd_pending;

                if ((cmd_length == UVDM_FLASH_READ_WRITE_CMD_SIZE) &&
                        (!stat) &&
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                         UVDM_FLASH_READ_WRITE_CMD_SIG))
                {
#if (CCG_BOOT == 0)
                    /*
                     * Check the value of tracker to determine that all sections
                     * of data buffer were updated through SEND_DATA UVDM before sending
                     * FLASH WRITE request.
                     */
                    if (gl_send_data_section_tracker != 0u)
                    {
                        response_code = CY_PDSTACK_STAT_FLASH_UPDATE_FAILED;
                    }
                    else
#endif /* (CCG_BOOT == 0) */
                    {
                        row_num = MAKE_WORD(cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET],
                                    cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                        response_code = flash_row_write(row_num, gl_data_buf.data_buf, flash_cb);
#if (CCG_BOOT == 0)
                        /*
                         * For non blocking case, if response is NO_RESPONSE, don't send
                         * Response VDM. Response VDM will be sent only after Flash Write
                         * completes.
                         */
                        if (response_code == CY_PDSTACK_STAT_NO_RESPONSE)
                        {
                            /* Set the flag to indicate that CCG is handling a FLASH write command currently. */
                            gl_flash_write_cmd_pending = true;
                            /* Response VDM is pending. */
                            response_state = UVDM_HANDLED_RESPONSE_NOT_READY;
                            gl_cur_nb_cmd = UVDM_CMD_FLASH_WRITE_OPCODE;
                        }
                        else

                        {
                            /* Revert back SEND_DATA UVDM tracker. */
                            gl_send_data_section_tracker =
                                UVDM_SEND_DATA_SECTION_UPDATE_MASK;
                        }
#endif /* (CCG_BOOT == 0) */
                        if(row_num == CCG_IMG1_METADATA_ROW_NUM)
                        {
                            gl_img_status.status.fw1_invalid = 1;
                        }
#if (!CCG_DUALAPP_DISABLE)
                        else if(row_num == CCG_IMG2_METADATA_ROW_NUM)
                        {
                            gl_img_status.status.fw2_invalid = 1;
                        }
#endif /* !CCG_DUALAPP_DISABLE */
                    }
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

            case UVDM_CMD_READ_DATA_OPCODE:
#if (SECURE_FW_UPDATE == 0)
                /*
                 *Invoke read data memory functiona and pass the sequence
                 * number. Make sure cmd_length is 0 in this case, as no command specific
                 * Data object is expected.
                 */
                stat = uvdm_read_data_memory ((uint8_t *)
                                &gl_response_vdo[UVDM_READ_DATA_RESPONSE_VDO_INDEX], seqNum);
                if ((cmd_length == 0u) && (true == stat))
                {
                    /* Increment the number of VDOs based on sequence number */
                    if (seqNum < UVDM_DATA_BUF_LAST_SEC_INDEX)
                    {
                        no_of_vdo += UVDM_NUM_DATA_MEMORY_SECTIONS;
                    }
                    else
                    {
                        no_of_vdo += 0x04u; /* UVDM_READ_DATA_NO_OF_VDO_ERROR_CASE; */
                    }
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
    #else
                response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
    #endif /* SECURE_FW_UPDATE */
                break;

    #if SYS_BLACK_BOX_ENABLE
            case UVDM_CMD_GET_BLACK_BOX:
                if(cmd_length == UVDM_BLACK_BOX_CMD_SIZE)
                {
                    uint16_t effective_offset = ((uint16_t)seqNum * UVDM_BLACK_BOX_CHUNK_SIZE);
                    int16_t bytes_left = (int16_t)SYS_BLACK_BOX_SIZE - (int16_t)effective_offset;
                    
                    if((bytes_left <= 0) || (d_obj[1].val >= SYS_BLACK_BOX_NO_OF_FLASH_ROWS))
                    {
                        response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                    }
                    else
                    {
                        if((uint16_t)bytes_left >= UVDM_BLACK_BOX_CHUNK_SIZE)
                        {
                            bytes_left = (int16_t)UVDM_BLACK_BOX_CHUNK_SIZE;
                        }
                        
                        mem_copy((uint8_t *)&gl_response_vdo[UVDM_READ_DATA_RESPONSE_VDO_INDEX], (uint8_t *)(CCG_BLACK_BOX_ADDR + effective_offset), (size_t)bytes_left);
                        no_of_vdo += ((uint8_t)(((uint16_t)bytes_left - 1u) >> 2) + 1u);
                    }
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;
    #endif /* SYS_BLACK_BOX_ENABLE */

            case UVDM_CMD_FLASH_READ_OPCODE:
#if ((SECURE_FW_UPDATE == 0) && (CCG_FLASH_READ_DISABLE == 0))
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected.
                 */
                if ((cmd_length == UVDM_FLASH_READ_WRITE_CMD_SIZE) &&
                        (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                         UVDM_FLASH_READ_WRITE_CMD_SIG))
                {
                    response_code = flash_row_read(
                            MAKE_WORD(cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET],
                                cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]), gl_data_buf.data_buf);

                    if(response_code == CY_PDSTACK_STAT_SUCCESS)
                    {
                        response_code = CY_PDSTACK_STAT_FLASH_DATA_AVAILABLE;
                    }
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
    #else
                response_code = CY_PDSTACK_INVALID_COMMAND;
    #endif /* ((SECURE_FW_UPDATE == 0) && (CCG_FLASH_READ_DISABLE == 0)) */     
                break;

            case UVDM_CMD_VALIDATE_FW_OPCODE:
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected.
                 */
                if (cmd_length == UVDM_VALIDATE_FW_CMD_SIZE)
                {
                    response_code = boot_handle_validate_fw_cmd ((sys_fw_mode_t)cmd_param[UVDM_VALIDATE_FW_MODE_INDEX]);
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;
    #if (CCG_BOOT == 1)
            case UVDM_CMD_REASON_FOR_BOOT_MODE:
                /*
                 * Make sure cmd_length is 0 in this case, as no command specific
                 * Data object is expected.
                 */
                if (!(cmd_length != 0u))
                {
                    /* Get the reason for boot mode. */
                    *((uint8_t *)&gl_response_vdo[UVDM_REASON_FOR_BOOT_MODE_VDO_INDEX])
                        = get_boot_mode_reason().val;
                    /* Increment number of VDOs to be sent by 1. */
                    no_of_vdo++;
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;
    #endif /* CCG_BOOT */

            case UVDM_CMD_GET_CHECKSUM:
    #if (SECURE_FW_UPDATE == 0)
                /* Check for flash address and flash size parameters. */
                if (cmd_length == UVDM_GET_CHECKSUM_CMD_SIZE)
                {
                    uint8_t *addr = (uint8_t *)MAKE_DWORD(cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET + 3u],
                            cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET + 2u], cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET + 1u],
                            cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET]);
                    uint32_t size = MAKE_DWORD(cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET + 3u],
                            cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET + 2u], cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET + 1u],
                            cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET]);
                    if (((uint32_t)addr + size) >= CY_FLASH_SIZE)
                    {
                        /* Set the NAK field in the header VDO. */
                        gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                            = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                    }
                    else
                    {
                        *((uint8_t *)&gl_response_vdo[UVDM_CHECKSUM_VDO_INDEX])
                            = mem_calculate_byte_checksum (addr, size);
                        /* Increment number of VDOs to be sent by 1. */
                        no_of_vdo++;
                    }
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
    #else
                response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
    #endif /* SECURE_FW_UPDATE */
                break;
                /* Don't support GET_FW_START_ADDRESS command in case of single app bootloader  */
    #if (!(CCG_DUALAPP_DISABLE && CCG_BOOT))
                /* Return the start Address of Both FW images. */
            case UVDM_CMD_GET_FW_START_ADDRESS_OPCODE:
                /*
                 * Make sure cmd_length is 0 in this case, as no command specific
                 * Data object is expected.
                 */
                if (cmd_length == 0u)
                {
                    gl_response_vdo[UVDM_FW1_START_ADDR_VDO_INDEX].val =
                        sys_get_fw_img1_start_addr();
    #if (!CCG_DUALAPP_DISABLE)
                    gl_response_vdo[UVDM_FW2_START_ADDR_VDO_INDEX].val =
                        sys_get_fw_img2_start_addr();
    #else
                    gl_response_vdo[UVDM_FW2_START_ADDR_VDO_INDEX].val =
                        SYS_INVALID_FW_START_ADDR;
    #endif /* CCG_DUALAPP_DISABLE */
                    no_of_vdo += UVDM_GET_FW_START_ADDR_UVDM_NO_OF_VDO;
                }
                else
                {
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;
    #endif /* !(CCG_DUALAPP_DISABLE && CCG_BOOT) */

    #if (CCG_BOOT == 0)
                /*
                 * This command is used for setting FW application priority value in App
                 * Priority Row for debugging purpose.
                 */
            case UVDM_CMD_SET_APP_PRIORITY_OPCODE:
    #if (APP_PRIORITY_FEATURE_ENABLE == 1u)
                /*
                 * Make sure cmd_length is 4 in this case, as just 1 command specific
                 * Data object is expected.
                 */
                if (cmd_length == UVDM_SET_APP_PRIORITY_CMD_SIZE)
                {
                    response_code = flash_set_app_priority(
                            cmd_param[UVDM_SET_APP_PRIORITY_INDEX]);
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
    #else
                response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
    #endif /* APP_PRIORITY_FEATURE_ENABLE */
                break;
    #endif /* CCG_BOOT */

    
            case UVDM_CMD_GET_BOOT_TYPE:
                /*
                 * Send back Bootloader type which is stored at a fixed offset in
                 * Bootloader flash region.
                 */
                if (cmd_length == 0u)
                {
                    gl_response_vdo[0x01].val = 0;
                    *((uint8_t *)&gl_response_vdo[0x01]) =
                        (*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET));
                    no_of_vdo++;
                }
                else
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmdType
                        = (uint8_t)CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
                break;

#if (CCG_BOOT == 0)
            case UVDM_CMD_SEND_SIGNATURE_OPCODE:
#if (SECURE_FW_UPDATE == 1)
                /*
                 * This is CY UVDM used to collect FW Image Signature and store it in
                 * pre-defined Flash location.
                 * Sequence number determines the section of Signature.
                 * Seq 1: First 24 bytes
                 * Seq 2: Next 24 bytes
                 * Seq 3: Last 16 bytes
                 */
                switch (seqNum)
                {
                    case UVDM_SEND_SIGN_SEQUENCE_1:
                    case UVDM_SEND_SIGN_SEQUENCE_2:
                        if (cmd_length == UVDM_SEND_SIGN_SEC_1_2_SIZE)
                        {
                            sboot_accumulate_sig (cmd_param, cmd_length,
                                    (seqNum - 1) * UVDM_SEND_SIGN_SEC_1_2_SIZE);
                            response_code = CY_PDSTACK_STAT_SUCCESS;
                        }
                        else
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                        }
                        break;
                    case UVDM_SEND_SIGN_SEQUENCE_3:
                        if ((cmd_length == UVDM_SEND_SIGN_SEC_3_SIZE)
                                && (!gl_flash_write_cmd_pending))
                        {
                            sboot_accumulate_sig (cmd_param, cmd_length,
                                    (seqNum - 1) * UVDM_SEND_SIGN_SEC_1_2_SIZE);
                            response_code = sboot_program_sig (flash_cb, false, 0x00);
                            /* For Non-Blocking Flash Programming. */
                            if (response_code == CY_PDSTACK_STAT_NO_RESPONSE)
                            {
                                /*
                                 * Set the flag to indicate that CCG is handling a FLASH write
                                 * command currently.
                                 */
                                gl_flash_write_cmd_pending = true;
                                /* Response VDM is pending. */
                                response_state = UVDM_HANDLED_RESPONSE_NOT_READY;
                                gl_cur_nb_cmd = UVDM_CMD_SEND_SIGNATURE_OPCODE;
                            }
                        }
                        else
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                        }
                        break;
                    default:
                        response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                        break;
                }
#else
                response_code = CY_PDSTACK_STAT_INVALID_COMMAND;
#endif /* SECURE_FW_UPDATE */
                break;

            case UVDM_CMD_GET_CUSTOMER_INFO:
                /* Respond with Customer specific info based on sequence number. */
                if ((seqNum != UVDM_GET_CUSTOMER_INFO_SEQ_1) &&
                        (seqNum != UVDM_GET_CUSTOMER_INFO_SEQ_2))
                {
                    response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                }
                else
                {
                    /* Get address of customer info. */
                    uint32_t addr = sys_get_custom_info_addr ();
                    /* Send first 16 bytes for seq number 1 and next 16 bytes for
                     * sequence number 2. */
                    addr = addr + (((uint32_t)seqNum - 1u) << 4u);
                    mem_copy ((uint8_t *)&gl_response_vdo[0x01], (uint8_t *)addr,
                            UVDM_GET_CUSTOMER_INFO_RESPONSE_SIZE);
                    no_of_vdo += UVDM_GET_CUSTOMER_INFO_RESPONSE_VDO_NUM;
                }
                break;
#endif /* CCG_BOOT */
#if TEST_CODE_COVERAGE /* skip_tcc */
            case UVDM_CMD_TEST_CODE_COVERAGE:
                if (cmd_length == UVDM_TEST_CODE_COVERAGE_CMD_SIZE)
                {
                    if (cmd_param[0] == UVDM_TCC_GET_DATA)
                    {
                        if (cmd_param[1] == UVDM_TCC_GET_RAW_DATA)
                        {
                            uint32_t bytes_read = tcc_get_raw_data((uint8_t *)&gl_response_vdo[0x01], 
                            (cmd_param[2] * UVDM_TCC_RAW_DATA_MAX_CHUNK_SIZE), UVDM_TCC_RAW_DATA_MAX_CHUNK_SIZE);
                            if (bytes_read > 0)
                            {
                                no_of_vdo += (((bytes_read - 1) >> 2) + 1);
                            }
                            else
                            {
                                response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                            }
                        }
                        else if (cmd_param[1] == UVDM_TCC_RESET_RAW_DATA)
                        {
                            tcc_init();
                            response_code = CY_PDSTACK_STAT_SUCCESS;
                        }
                        else
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                        }
                    }
                    else if (cmd_param[0] == UVDM_TCC_GET_STAT)
                    {
                        if (cmd_param[1] == UVDM_TCC_GET_PERCENTAGE)
                        {
                            uint8_t tcc_percent = tcc_get_percent();
                            MEM_COPY ((uint8_t *)&gl_response_vdo[0x01], (uint8_t *)&tcc_percent, sizeof(uint8_t));
                            no_of_vdo += UVDM_TCC_GET_PERCENTAGE_NO_OF_VDO;
                        }
                        else if (cmd_param[1] == UVDM_TCC_GET_RAW_DATA_SIZE)
                        {
                            uint32_t tcc_raw_data_size = tcc_get_raw_data_size();
                            MEM_COPY ((uint8_t *)&gl_response_vdo[0x01], (uint8_t *)&tcc_raw_data_size, sizeof(uint32_t));
                            no_of_vdo += UVDM_TCC_GET_RAW_DATA_SIZE_NO_OF_VDO;
                        }
                        else
                        {
                            response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                        }
                    }
                    else
                    {
                        response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                    }            
                }
                else
                {
                    response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                } 
                break;
#endif /* TEST_CODE_COVERAGE */ /* skip_tcc */


#if (PSVP_FPGA_ENABLE) && (!CCG_BOOT)
           case UVDM_CMD_CUSTOM:
           {

               uint8_t count = 24;
               uint16_t read_offset;

               if ((cmd_opcode != UVDM_CMD_CUSTOM) && (cmd_length != 4))
               {
                   response_code = CY_PDSTACK_STAT_INVALID_ARGUMENT;
                   return response_state;
               }

               /*
                * The read offset is the encoded address offset in ROM. Each byte of
                * return data holds the bitwise access data for 8 DWORD locations.
                * The offset specified is in terms of number of bytes of data returned.
                * read offset = ((ROM addr offset / 4) / 8).
                * The reduced count is to minimize the CC interface overhead.
                */
               read_offset = MAKE_WORD(cmd_param[1], cmd_param[0]);

               if ((read_offset & 0x3) == 0)
               {


                   /*
                    * If the offset is at the last transfer, it will not have full
                    * data. max read offset is ((ROM_SIZE / 4) / 8) - 24.
                    */
                   if (read_offset > 1000)
                   {
                       count = 1024 - read_offset;
                   }

                   rom_coverage_get_data((uint8_t *)(&gl_response_vdo[1]),
                           read_offset, count);

                   no_of_vdo += (count >> 2);
                   response_state = UVDM_HANDLED_RESPONSE_READY;
                   response_code = CY_PDSTACK_STAT_NO_RESPONSE;
               }
           }
           break;
#endif /* (PSVP_FPGA_ENABLE) && (!CCG_BOOT) */
        default:
            /* Can't recognise VDM */
#if SLN_FLASHING_UVDM_HANDLER_ENABLE
            /* Expecting the solution handler to re-use the response data from UVDM handler. */
            response_state = sln_flashing_uvdm_handler(rx_pkt, gl_response_vdo, &no_of_vdo, &response_code);
#else /* SLN_FLASHING_UVDM_HANDLER_ENABLE */
            response_state = UVDM_NOT_HANDLED;
#endif /* SLN_FLASHING_UVDM_HANDLER_ENABLE */
            break;
        }
    }

    /* Handle response. */
    if (response_state == UVDM_HANDLED_RESPONSE_READY)
    {
        /* Place the cmd opcode in header VDO. */
        gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd = (uint8_t)cmd_opcode;

        /*
         * Check if response is not "NO_RESPONSE". In case of NO_RESPONSE,
         * we have to send out the response code in index 1 VDO.
         */
        if (response_code != CY_PDSTACK_STAT_NO_RESPONSE)
        {
            no_of_vdo += 1u;
            gl_response_vdo[UVDM_RESPONSE_VDO_INDEX].val = ((uint32_t)response_code +
                CCG_STATUS_CODE_OFFSET);
        }

        /* Update the pointer to point to Response VDM. */
        *vdm_rspn_pkt = &gl_response_vdo[0];
        /* Update count of VDOs. */
        *vdo_count = no_of_vdo;
    }
    else
    {
        /* No VDM response required. */
        *vdo_count = 0;
    }

    return response_state;
}

#if QC_PPS_ENABLE

#define QC_PPS_SVID                 (0x05C6u)
    
#define QC_RESPONSE_ACK             (0xA0u)
#define QC_RESPONSE_NACK            (0x50u)

#define QC_CHARGER_TYPE             (QC_PPS_CHARGER_TYPE)
#define QC_CHARGER_VERSION          (QC_PPS_VERSION)

/* Macro defines the index of the sensor to be used for measuring the connector temperature */
#ifndef APP_QC_CONNECTOR_SENSOR_IDX
#define APP_QC_CONNECTOR_SENSOR_IDX                 (0u)
#endif /* APP_QC_CONNECTOR_SENSOR_IDX */

#ifndef APP_QC_CASE_SENSOR_IDX
/* Macro defines the index of the sensor to be used for measuring the case temperature */
#define APP_QC_CASE_SENSOR_IDX                      (0u)
#endif /* APP_QC_CASE_SENSOR_IDX */

uvdm_response_state_t uvdm_qc_pps_handler(cy_stc_pdstack_context_t *ptrPdStackContext, uint32_t *rx_pkt,
    cy_pd_pd_do_t **vdm_rspn_pkt, uint8_t *vdo_count)
{
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
    uint16_t qc_cmd;
    uint16_t volt_connector;   
    uint8_t temperature;
    cy_pd_pd_do_t* d_obj = (cy_pd_pd_do_t *)((uint32_t)(&(rx_pkt[1])));
    *vdo_count = 0;

    /* Review: Handle UVDMs only if in Source mode. */

    /*
     * Handle QC UVDM only if:
     * 1) SVID is Qualcomm VID
     * 2) QC 5.0/4.0 support is enabled in configuration table.
     */
    if (((pd_get_ptr_chg_cfg_tbl(ptrPdStackContext->ptrUsbPdContext)->srcSel & BC_SRC_QC_4_0_MODE_ENABLE_MASK) != 0u) &&
        (d_obj->ustd_qc_pps_hdr.svid == (uint32_t)QC_PPS_SVID))
    {
        /* Copy the header in response VDO. */
        gl_response_vdo[UVDM_HEADER_INDEX].val = d_obj->val;
        /* Set ACK as default response. */
        gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_pps_hdr.cmd0 = QC_RESPONSE_ACK;
        /* Set Default response VDO count to 2: Header + 1 VDO. */
        *vdo_count = 2;

       /* Get QC Command. */
       qc_cmd = (d_obj->ustd_qc_pps_hdr.cmd0 |
            ((uint16_t)d_obj->ustd_qc_pps_hdr.cmd1 << 8));

       /* As per QC 5.0/4.0, we should respond only if this is a valid request. */
       response_state = UVDM_HANDLED_NO_RESPONSE;

        /* Check if UVDM Command has only header VDO and that we are UFP. If not, do not respond. */
        if ((CY_PD_GET_PD_HDR_CNT((rx_pkt[0])) == 1u) && (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP))
        {
            /* Assume this to be a valid request by default to optimize. */
            response_state = UVDM_HANDLED_RESPONSE_READY;

            switch ((uvdm_qc_pps_cmd_t)qc_cmd)
            {
                case UVDM_QC_GET_CASE_TEMP:
#if (TEMPERATURE_SENSOR_COUNT != 0u)
                    temperature = ccg_get_sensor_temperature(ptrPdStackContext, APP_QC_CASE_SENSOR_IDX);
#else
                    temperature = THERMISTOR_FAULT_TEMP;
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u)*/
                    if(temperature == THERMISTOR_FAULT_TEMP)
                    {
                        /* If the temperature sensor is not present/ fault temperature is returned, Send NAK. */
                        gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_pps_hdr.cmd0 = QC_RESPONSE_NACK;
                        *vdo_count = 1;                        
                    }
                    else
                    {
                        /* Since this is a one byte response, copy the data to val */
                        gl_response_vdo[1].val = (uint32_t)temperature;                                    
                    }
                    break;

                case UVDM_QC_GET_CONNECTOR_TEMP:
#if (TEMPERATURE_SENSOR_COUNT != 0u)
                    temperature = ccg_get_sensor_temperature(ptrPdStackContext, APP_QC_CONNECTOR_SENSOR_IDX);
#else
                    temperature = THERMISTOR_FAULT_TEMP;
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u)*/
                    if(temperature == THERMISTOR_FAULT_TEMP)
                    {
                        /* If the temperature sensor is not present/ fault temperature is returned, Send NAK. */
                        gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_pps_hdr.cmd0 = QC_RESPONSE_NACK;
                        *vdo_count = 1;                        
                    }
                    else
                    {
                        /* Since this is a one byte response, copy the data to val */                        
                        gl_response_vdo[1].val = (uint32_t)temperature;                                    
                    }            
                    break;

                case UVDM_QC_GET_CONNECTOR_VOLT:
                    volt_connector = ptrPdStackContext->ptrAppCbk->vbus_get_value(ptrPdStackContext);
                    /* Convert in 10mV units from mV units. */
                    volt_connector /= 10u;
                    gl_response_vdo[1].qc_pps_data_vdo.data0 = 0;
                    gl_response_vdo[1].qc_pps_data_vdo.data1 = (volt_connector & 0xFFu);
                    gl_response_vdo[1].qc_pps_data_vdo.data2 = ((volt_connector & 0x0F00u) >> 8);
                    gl_response_vdo[1].qc_pps_data_vdo.data3 = 0;
                    break;

                case UVDM_QC_GET_CHARGER_TYPE:
                    /* Since this is a one byte response, copy the data to val */                    
                    gl_response_vdo[1].val = (uint32_t)QC_CHARGER_TYPE;
                    break;

                case UVDM_QC_GET_CHARGER_VERSION:
                    /* Since this is a one byte response, copy the data to val */                    
                    gl_response_vdo[1].val = (uint32_t)QC_CHARGER_VERSION;
                    break;

                default:
                    response_state = UVDM_HANDLED_NO_RESPONSE;
                    break;
            }
        }

        if (response_state == UVDM_HANDLED_RESPONSE_READY)
        {
            *vdm_rspn_pkt = &gl_response_vdo[0];
        }
    }

    return response_state;
}
#endif /* QC_PPS_ENABLE */

/* [] END OF FILE */
