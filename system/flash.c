/******************************************************************************
* File Name: flash.h
* \version 2.0
*
* Description: Flash handler source file
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include <config.h>
#include <cy_pdutils.h>
#include <cy_pdstack_common.h>
#include <system.h>
#include <boot.h>
#include "cy_flash.h"
#include <flash.h>
#include "srom.h"
#include "cy_usbpd_regs.h"
#include "cy_device.h"
#include "cyip_srsslt.h"
#include "cy_flash.h"

#ifndef CY_FLASH_CPUSS_REQ_START
#define CY_FLASH_CPUSS_REQ_START                        ((uint32_t) ((uint32_t) 0x1U << 31U))
#endif /* CY_FLASH_CPUSS_REQ_START */

#ifndef CY_FLASH_API_OPCODE_LOAD
#define CY_FLASH_API_OPCODE_LOAD                        (0x04U)
#endif /* CY_FLASH_API_OPCODE_LOAD */

#ifndef CY_FLASH_API_OPCODE_WRITE_SFLASH_ROW
#define CY_FLASH_API_OPCODE_WRITE_SFLASH_ROW            (0x18U)
#endif /* CY_FLASH_API_OPCODE_WRITE_SFLASH_ROW */

#if !(SROM_CODE_SYS_FLASH)
/*
 * Flags to indicate Flashing mode. Flash read and write requests
 * are honoured only when any of these flags are set. Also, the
 * flash_set_access_limits() function must be called before flash
 * access can be done. The default values prevent any write or
 * read.
 */
static uint8_t gl_flash_mode_en = 0;

/* Lowest flash row number that can be accessed. */
static uint16_t gl_flash_access_first = CCG_LAST_FLASH_ROW_NUM + 1u;

/* Highest flash row number that can be accessed. */
static uint16_t gl_flash_access_last = CCG_LAST_FLASH_ROW_NUM + 1u;

/* Flash row containing metadata for the alternate firmware image. */
static uint16_t gl_flash_metadata_row = CCG_LAST_FLASH_ROW_NUM + 1u;

/* Last boot loader flash row. Used for read protection. */
static uint16_t gl_flash_bl_last_row = CCG_LAST_FLASH_ROW_NUM + 1u;

#if (!CCG_BOOT)
/* Whether flash write data can be used in-place as SROM API parameter. */
static volatile bool gl_flash_write_in_place = false;
#endif /* (!CCG_BOOT) */

#endif /* SROM_CODE_SYS_FLASH */

#if (FLASH_ENABLE_NB_MODE == 1)
/* SPC Interrupt Number. */
#if defined(CCG3) || defined(DMC)
#define FLASH_SPC_INTR                  (0x0C)
#elif (defined(CCG3PA2))
#define FLASH_SPC_INTR                  (0x09)
#else /* CCGx */
#error "Not suppported device family. Non blocked flashing supported only on CCG3 and CCG3PA2."
#endif /* CCGx */
#endif /* (FLASH_ENABLE_NB_MODE == 1) */

/* MACROS for SROM APIs. */

#define SROM_API_RETURN_VALUE                   \
    (((CY_FLASH_CPUSS_SYSARG_REG & 0xF0000000u) \
      == 0xA0000000u) ? CYRET_SUCCESS :         \
     (CY_FLASH_CPUSS_SYSARG_REG & 0x0000000Fu))

/* Keys used in SROM APIs. */
#define SROM_FLASH_API_KEY_ONE          (0xB6u)
#define SROM_FLASH_API_KEY_TWO(x)       (uint32_t)(0xD3u + x)
#define SROM_FLASH_KEY_TWO_OFFSET       (0x08u)

/* Offset of Argument 1 and 2 (Words) for SROM APIs in SRAM Buffer. */
#define SROM_API_ARG0_OFFSET            (0x00u)
#define SROM_API_ARG1_OFFSET            (0x01u)

/* SROM LOAD FLASH API. */
#define SROM_LOAD_FLASH_API_OPCODE              (0x04u)
#define SROM_LOAD_FLASH_DATA_OFFSET             (0x02u)
#define SROM_LOAD_FLASH_BYTE_ADDR               (0x00u)
#define SROM_LOAD_FLASH_BYTE_ADDR_OFFSET        (0x10u)
#define SROM_LOAD_FLASH_MACRO_OFFSET            (0x18u)

/* FLASH ROW PROGRAM API. */
#define SROM_FLASH_PROGRAM_API_OPCODE           (0x05u)

/* Non-BLOCKING Write Flash Row API */
#define SROM_NB_FLASH_ROW_API_OPCODE            (0x07u)
#define SROM_NB_FLASH_ROW_NUM_OFFSET            (0x10u)

/* Resume Non-Blocking API */
#define SROM_RESUME_NB_API_OPCODE               (0x09u)

/* Abort Non-Blokcing Flash Row Write API Opcode. */
#define SROM_ABORT_FLASH_WRITE_OPCODE           (0x1Cu)

/* Write User SFLASH Row API Opcode. */
#define SROM_USER_SFLASH_WRITE_OPCODE           (0x18u)

/* CPUSS SYSARG return value mask. */
#define CPUSS_SYSARG_RETURN_VALUE_MASK          (0xF0000000u)

/* CPUSS SYSARG success return value. */
#define CPUSS_SYSARG_PASS_RETURN_VALUE          (0xA0000000u)

/* CPUSS SYSARG error return value. */
#define CPUSS_SYSARG_ERROR_RETURN_VALUE         (0xF0000000u)

#define CPUSS_FLASH_PARAM_SIZE                  (8u)

#if (FLASH_ENABLE_NB_MODE == 1)

/* Buffer used for SROM APIs */
static uint32_t gl_srom_arg_buf[(CY_FLASH_SIZEOF_ROW/ sizeof(uint32_t)) + 2];

/* Callback function registered by user for Non Blokcing Flash Write Row operation. */
static flash_cbk_t flash_notify;

/* Counter to keep track of the SPC Interrupts while Non-Blocking Flash update. */
static uint8_t gl_spc_intr_counter = 0;

/* Flag to indciate Abort request for current Non Blokcing Flash write operation was received. */
static bool gl_flash_nb_write_abort = false;

CY_ISR_PROTO(flash_spc_intr_handler);

/* Refer BROS 001-88589 Section 4.6.2.5 for SROM API's description. */

/*
 * @brief Execute SROM LOAD FLASH API
 *
 * Description
 * This API loads the page latch buffer with the data to be programmed in flash,
 * This is the first API in FLASH ROW operation.
 *
 * @param data Pointer to data to be flashed
 * @param flash_macro_index Flash macro number
 *
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_load_flash_bytes_api(uint8_t *data, uint8_t flash_macro_index)
{
    uint8_t retValue;
    /* Write 128 bytes in a temp buf which is eventually passed to SROM API. */
    memcpy ((void *)&gl_srom_arg_buf[SROM_LOAD_FLASH_DATA_OFFSET], data,
            CY_FLASH_SIZEOF_ROW);

    /* Fill in the arguments for API. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET] = (uint32_t)
                ((flash_macro_index << SROM_LOAD_FLASH_MACRO_OFFSET) |
                (SROM_LOAD_FLASH_BYTE_ADDR << SROM_LOAD_FLASH_BYTE_ADDR_OFFSET) |
                (SROM_FLASH_API_KEY_TWO(SROM_LOAD_FLASH_API_OPCODE) <<
                SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* Number of bytes to flash - 1 */
    gl_srom_arg_buf[SROM_API_ARG1_OFFSET] = CY_FLASH_SIZEOF_ROW-1;

    /* SYSARG */
    CPUSS_SYSARG = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS_SYSREQ = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_LOAD_FLASH_API_OPCODE);

    /* Read the Result. */
    retValue = SROM_API_RETURN_VALUE;
    if (retValue != 0)
    {
        return false;
    }
    return true;
}

/*
 * @brief Execute Non Blocking Write Row
 *
 * Description
 * This API performs the first part of the write row operation, which is
 * the pre-program operation.
 *
 * @param row_num Flash Row Number to be programmed
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_nb_flash_write_api(uint16_t row_num)
{
    /* Arguments */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)
            ((row_num << SROM_NB_FLASH_ROW_NUM_OFFSET) |
            (SROM_FLASH_API_KEY_TWO(SROM_NB_FLASH_ROW_API_OPCODE) <<
            SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);

    /* This command results in three SPC interrupts. Reset the counter. */
    gl_spc_intr_counter = 0;
    /* SYSARG */
    CPUSS_SYSARG = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS_SYSREQ = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_NB_FLASH_ROW_API_OPCODE);

    /*
     * Check if the status in SYSARG is failure. If yes, then return false.
     * Otherwise, request was successful.
     */
    if ((CPUSS_SYSARG & CPUSS_SYSARG_RETURN_VALUE_MASK) ==
        CPUSS_SYSARG_ERROR_RETURN_VALUE)
    {
        return false;
    }
    return true;
}

/*
 * @brief Execute SROM Non Blocking Resume APIs
 *
 * @param None
 * @rerurn None
 */
static void srom_resume_non_blocking(void)
{
    /* Execute Resume Non Blocking API */

    /* Arguments. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)((SROM_FLASH_API_KEY_TWO(SROM_RESUME_NB_API_OPCODE)
        << SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* SYSARG */
    CPUSS_SYSARG = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS_SYSREQ = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_RESUME_NB_API_OPCODE);
}

/*
 * @brief Execute SROM Non Blocking Abort APIs
 *
 * @param None
 * @rerurn None
 */
static void srom_abort_flash_write(void)
{
    /* Execute Abort Non Blaokcing Flash Row Write operation. */

    /* Arguments. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)((SROM_FLASH_API_KEY_TWO(SROM_ABORT_FLASH_WRITE_OPCODE)
        << SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* SYSARG */
    CPUSS_SYSARG = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS_SYSREQ = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_ABORT_FLASH_WRITE_OPCODE);
}

/* SPC Interrupt for Resume Non-Blocking SROM API */
CY_ISR(flash_spc_intr_handler)
{
    flash_write_status_t status = FLASH_WRITE_COMPLETE;

    /* Check if Abort request is pending. */
    if (gl_flash_nb_write_abort)
    {
        /*
         * See if we are already in last phase of Flash update i.e the
         * third interrupt. If yes, there is no point in aborting the flash as there
         * is nothing left to abort.
         */
        if (gl_spc_intr_counter < 2)
        {
           srom_abort_flash_write ();
           status = FLASH_WRITE_ABORTED;
        }
        else
        {
            srom_resume_non_blocking ();
            status = FLASH_WRITE_COMPLETE_AND_ABORTED;
        }
        gl_flash_nb_write_abort = false;
    }
    else
    {
        /*
         * Once Non-Blocking Flash row update process starts, this interrupt
         * will fire three times. FW is expected to call Resume Non-Blocking
         * SROM API from here.
         */
        gl_spc_intr_counter++;

        /* Resume Non-Blokcing Operation. */
        srom_resume_non_blocking ();
    }

    /*
     * See if this is the third interrupt of flash write sequence or if there
     * was an abort request. In both cases, reset the counters/clocks and invoke
     * the registered callback.
     */
    if ((gl_spc_intr_counter == 3) || (status != FLASH_WRITE_COMPLETE))
    {
        /* Disable clock to the charge pump after flash write is complete or aborted. */
        SRSSLT->clk_select = (SRSSLT->clk_select & ~CLK_SELECT_PUMP_SEL_MASK);

        /* Reset counter. */
        gl_spc_intr_counter = 0;
        /* Invoke Callback. */
        if (flash_notify != NULL)
        {
            /* Callback should notify the status of flash write as well. */
            flash_notify (status);
        }
    }
}
#endif /* FLASH_ENABLE_NB_MODE */

#if (!CCG_BOOT)
/*
 * This function invokes the SROM API to do a flash row write.
 * This function is used instead of the CySysFlashWriteRow, so as to avoid
 * the clock trim updates that are done as part of that API.
 *
 * Note: This function expects that the data_p buffer has CPUSS_FLASH_PARAM_SIZE
 * bytes of prefix space which can be used for setting up the SROM write API
 * parameters.
 */
ATTRIBUTES_SYS_FLASH static cy_en_pdstack_status_t flash_trig_row_write_in_place(uint32_t row_num, uint8_t *data_p, bool is_sflash)
{
    volatile uint32_t *params = ((uint32_t *)data_p) - 2;
    cy_en_pdstack_status_t status = CY_PDSTACK_STAT_SUCCESS;

    /* Connect the charge pump to IMO clock for flash write. */
#ifdef PAG1S
    SRSSLT_CLK_SELECT = (SRSSLT_CLK_SELECT & ~SRSSLT_CLK_SELECT_PUMP_SEL_Msk) | (1 << SRSSLT_CLK_SELECT_PUMP_SEL_Pos);
#else /* !PAG1S */
    SRSSLT_CLK_SELECT = (SRSSLT_CLK_SELECT & ~SRSSLT_CLK_SELECT_PUMP_SEL_Msk) | (1u << SRSSLT_CLK_SELECT_PUMP_SEL_Pos);
#endif /* PAG1S */

    /* Set the parameters for load data into latch operation. */
    params[0] = SROM_FLASH_API_KEY_ONE |
        (SROM_FLASH_API_KEY_TWO((SROM_LOAD_FLASH_API_OPCODE)) << SROM_FLASH_KEY_TWO_OFFSET);
    params[1] = CY_FLASH_SIZEOF_ROW - 1u;
#if (CY_IP_FLASH_MACROS > 1)
    if (CY_FLASH_GET_MACRO_FROM_ROW(row_num) != 0)
    {
        params[0] |= (1 << SROM_LOAD_FLASH_MACRO_OFFSET);
    }
#endif /* (CY_IP_FLASH_MACROS > 1) */

    CPUSS_SYSARG = (uint32_t)(&params[0]);
    CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | CY_FLASH_API_OPCODE_LOAD);
    
    __NOP();
    __NOP();
    __NOP();

    /* If load latch is successful. */
    if ((CPUSS_SYSARG & CPUSS_SYSARG_RETURN_VALUE_MASK) == CPUSS_SYSARG_PASS_RETURN_VALUE)
    {
        if (is_sflash)
        {
            /* Perform the sflash write. */
            params[0] = (SROM_FLASH_API_KEY_ONE |
                    (SROM_FLASH_API_KEY_TWO((SROM_USER_SFLASH_WRITE_OPCODE)) << SROM_FLASH_KEY_TWO_OFFSET));
            params[1] = row_num;
            CPUSS_SYSARG = (uint32_t)(&params[0]);
            CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | SROM_USER_SFLASH_WRITE_OPCODE);
        }
        else
        {
            /* Perform the flash write. */
            params[0] = ((row_num << SROM_NB_FLASH_ROW_NUM_OFFSET) | SROM_FLASH_API_KEY_ONE |
                    (SROM_FLASH_API_KEY_TWO((SROM_FLASH_PROGRAM_API_OPCODE)) << SROM_FLASH_KEY_TWO_OFFSET));
            CPUSS_SYSARG = (uint32_t)(&params[0]);
            CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | SROM_FLASH_PROGRAM_API_OPCODE);
        }
        
        __NOP();
        __NOP();
        __NOP();
        
        if ((CPUSS_SYSARG & CPUSS_SYSARG_RETURN_VALUE_MASK) != CPUSS_SYSARG_PASS_RETURN_VALUE)
        {
            status = CY_PDSTACK_STAT_FAILURE;
        }
    }
    else
    {
        status = CY_PDSTACK_STAT_FAILURE;
    }

    /* Disconnect the clock to the charge pump after flash write is complete. */
#ifdef PAG1S
    SRSSULT->clk_select = (SRSSULT->clk_select & ~CLK_SELECT_PUMP_SEL_MASK);
#else /* !PAG1S */
    SRSSLT_CLK_SELECT = (SRSSLT_CLK_SELECT & ~SRSSLT_CLK_SELECT_PUMP_SEL_Msk);
#endif /* PAG1S */

    return status;
}
#endif /* (!CCG_BOOT) */

#if 0
#if !((CCG_HPI_ENABLE) && (!CCG_BOOT) && (!SYS_BLACK_BOX_ENABLE))
/*
 * This function invokes the SROM API to do a flash row write.
 * This function is used instead of the CySysFlashWriteRow, so as to avoid
 * the clock trim updates that are done as part of that API.
 */
ATTRIBUTES_SYS_FLASH static cy_en_pdstack_status_t flash_trig_row_write(uint32_t row_num, uint8_t *data_p, bool is_sflash)
{
    volatile uint32_t params[(CY_FLASH_SIZEOF_ROW+ CPUSS_FLASH_PARAM_SIZE) / sizeof(uint32_t)];
    cy_en_pdstack_status_t status = CY_PDSTACK_STAT_SUCCESS;

    /* Connect the charge pump to IMO clock for flash write. */
#ifdef PAG1S
    SRSSULT->clk_select = (SRSSULT->clk_select & ~CLK_SELECT_PUMP_SEL_MASK) | (1 << CLK_SELECT_PUMP_SEL_POS);
#else /* !PAG1S */
    SRSSLT_CLK_SELECT = (SRSSLT_CLK_SELECT & ~SRSSLT_CLK_SELECT_PUMP_SEL_Msk) | (1u << SRSSLT_CLK_SELECT_SYSCLK_DIV_Pos);
#endif /* PAG1S */

    /* Copy the data into the parameter buffer. */
    /* QAC suppression 0312: volatile qualifier for params[] is not mandatory as
     * mem_copy never reads params[], and it makes sure that each byte is written. */
    mem_copy ((uint8_t *)(&params[2]), (const uint8_t *)data_p, CY_FLASH_SIZEOF_ROW); /* PRQA S 0312 */

    /* Set the parameters for load data into latch operation. */
    params[0] = SROM_FLASH_API_KEY_ONE |
        (SROM_FLASH_API_KEY_TWO((SROM_LOAD_FLASH_API_OPCODE)) << SROM_FLASH_KEY_TWO_OFFSET);
    params[1] = CY_FLASH_SIZEOF_ROW- 1u;
#if (CY_IP_FLASH_MACROS > 1)
    if (CY_FLASH_GET_MACRO_FROM_ROW(row_num) != 0)
    {
        params[0] |= (1 << SROM_LOAD_FLASH_MACRO_OFFSET);
    }
#endif /* (CY_IP_FLASH_MACROS > 1) */

    CPUSS_SYSARG = (uint32_t)(&params[0]);
    CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | CY_FLASH_API_OPCODE_LOAD);

    __NOP();
    __NOP();
    __NOP();

    /* If load latch is successful. */
    if ((CPUSS_SYSARG & CPUSS_SYSARG_RETURN_VALUE_MASK) == CPUSS_SYSARG_PASS_RETURN_VALUE)
    {
        if (is_sflash)
        {
            /* Perform the sflash write. */
            params[0] = (SROM_FLASH_API_KEY_ONE |
                    (SROM_FLASH_API_KEY_TWO((CY_FLASH_API_OPCODE_WRITE_SFLASH_ROW)) << SROM_FLASH_KEY_TWO_OFFSET));
            params[1] = row_num;
            CPUSS_SYSARG = (uint32_t)(&params[0]);
            CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | CY_FLASH_API_OPCODE_WRITE_SFLASH_ROW);
        }
        else
        {
            /* Perform the flash write. */
            params[0] = ((row_num << SROM_NB_FLASH_ROW_NUM_OFFSET) | SROM_FLASH_API_KEY_ONE |
                    (SROM_FLASH_API_KEY_TWO((SROM_FLASH_PROGRAM_API_OPCODE)) << SROM_FLASH_KEY_TWO_OFFSET));
            CPUSS_SYSARG = (uint32_t)(&params[0]);
            CPUSS_SYSREQ = (CY_FLASH_CPUSS_REQ_START | SROM_FLASH_PROGRAM_API_OPCODE);
        }

        __NOP();
        __NOP();
        __NOP();

        if ((CPUSS_SYSARG & CPUSS_SYSARG_RETURN_VALUE_MASK) != CPUSS_SYSARG_PASS_RETURN_VALUE)
        {
            status = CY_PDSTACK_STAT_FAILURE;
        }
    }
    else
    {
        status = CY_PDSTACK_STAT_FAILURE;
    }

    /* Disconnect the clock to the charge pump after flash write is complete. */
#ifdef PAG1S
    SRSSULT->clk_select = (SRSSULT->clk_select & ~CLK_SELECT_PUMP_SEL_MASK);
#else /* !PAG1S */
    SRSSLT_CLK_SELECT = (SRSSLT_CLK_SELECT & ~SRSSLT_CLK_SELECT_PUMP_SEL_Msk);
#endif /* PAG1S */

    return status;
}
#endif /* !((CCG_HPI_ENABLE) && (!CCG_BOOT) && (!SYS_BLACK_BOX_ENABLE)) */
#endif

ATTRIBUTES_SYS_FLASH void flash_enter_mode(bool is_enable, flash_interface_t mode, bool data_in_place)
{
    /* Enter or Exit the Flashing mode. Only one mode will be active at a time. */
    if (is_enable)
    {
        CALL_MAP(gl_flash_mode_en) = (1u << (uint8_t)mode);
#if (!CCG_BOOT)
        CALL_MAP(gl_flash_write_in_place) = data_in_place;
#else
        CY_UNUSED_PARAMETER(data_in_place);
#endif /* (!CCG_BOOT) */
    }
    else
    {
        CALL_MAP(gl_flash_mode_en) = 0;
#if (!CCG_BOOT)
        CALL_MAP(gl_flash_write_in_place) = false;
#endif /* (!CCG_BOOT) */
    }
}

ATTRIBUTES_SYS_FLASH bool flash_access_get_status (uint8_t modes)
{
    return ((bool)((CALL_MAP(gl_flash_mode_en) & modes) != 0u));
}

ATTRIBUTES_SYS_FLASH void flash_set_access_limits (uint16_t start_row, uint16_t last_row, uint16_t md_row,
        uint16_t bl_last_row)
{
    /*
     * Caller is expected to provide valid parameters. No error checking
     * is expected to be done by this function. Store the flash write and
     * flash read access area information.
     */
    CALL_MAP(gl_flash_access_first) = start_row;
    CALL_MAP(gl_flash_access_last)  = last_row;
    CALL_MAP(gl_flash_metadata_row) = md_row;
    CALL_MAP(gl_flash_bl_last_row) = bl_last_row;
}

ATTRIBUTES_SYS_FLASH void flash_get_access_limits (uint16_t *access_limit_0, uint16_t *access_limit_1, uint16_t *access_limit_2, uint16_t *access_limit_3)
{
    *access_limit_0 = CALL_MAP(gl_flash_access_first);
    *access_limit_1 = CALL_MAP(gl_flash_access_last);
    *access_limit_2 = CALL_MAP(gl_flash_metadata_row);
    *access_limit_3 = CALL_MAP(gl_flash_bl_last_row);
}

ATTRIBUTES_SYS_FLASH static cy_en_pdstack_status_t flash_blocking_row_write(uint16_t row_num, uint8_t *data, bool is_sflash)
{
    cy_en_pdstack_status_t stat;

#if (!CCG_BOOT)
    /* Invoke Flash Write API. */
    if (CALL_MAP(gl_flash_write_in_place))
    {
        stat = flash_trig_row_write_in_place (row_num, data, is_sflash);
    }
    else
#else
    CY_UNUSED_PARAMETER(is_sflash);
#endif /* (!CCG_BOOT) */
    {
#if ((CCG_HPI_ENABLE) && (!CCG_BOOT) && (!SYS_BLACK_BOX_ENABLE))
        /* Assume that only in-place writes are enabled in HPI based binaries. */
        stat = CY_PDSTACK_STAT_FAILURE;
#else
        stat = (cy_en_pdstack_status_t)Cy_Flash_WriteRow((uint32_t)(row_num << CCG_FLASH_ROW_SHIFT_NUM), (uint32_t*)data);
#endif /* ((CCG_HPI_ENABLE) && (!CCG_BOOT)) */
    }

    if (stat != CY_PDSTACK_STAT_SUCCESS)
    {
        stat = CY_PDSTACK_STAT_FLASH_UPDATE_FAILED;
    }

    return stat;
}

/*
 * @brief Handle Clear Flash Row operation.
 *
 * Description
 * This function clears spcified flash row
 *
 * @param row_num Flash Row Number
 * @return cy_en_pdstack_status_t Status Code
 */
ATTRIBUTES_SYS_FLASH cy_en_pdstack_status_t flash_row_clear(uint16_t row_num)
{
#if (CCG3PA2_FLASH_WRITE_ISSUE != 0)
    uint8_t clear_buufer[128] = {0};
    cy_en_pdstack_status_t status;
    
    if (CYRET_SUCCESS == CySysFlashWriteRow (row_num, clear_buufer))
    {
        status = CCG_STAT_SUCCESS;
    }
else
    {
        status = CCG_STAT_FLASH_UPDATE_FAILED;
    }
    return status;    
#else
    uint8_t buffer[CY_FLASH_SIZEOF_ROW+ CPUSS_FLASH_PARAM_SIZE] = {0};
    /*
     * SROM needs flash writes at 48MHz, if IMO is already set for 48MHz use
     * optimized fash write implementation otherwise use creator component
     * which takes care of IMO clock configuration internally.
     */
#if ((CYDEV_BCLK__HFCLK__MHZ == 48) || SROM_CODE_SYS_FLASH)
    return flash_blocking_row_write (row_num, buffer + CPUSS_FLASH_PARAM_SIZE, false);
#else
    return ((cy_en_pdstack_status_t)Cy_Flash_WriteRow((row_num << CCG_FLASH_ROW_SHIFT_NUM), (uint32_t*)(buffer + CPUSS_FLASH_PARAM_SIZE)));
#endif /* ((CYDEV_BCLK__HFCLK__MHZ == 48) || SROM_CODE_SYS_FLASH) */
#endif /* (CCG3PA2_FLASH_WRITE_ISSUE != 0) */
}

#if (FLASH_ENABLE_NB_MODE == 1)

void flash_non_blocking_write_abort(void)
{
    /* Set a flag which which will be sampled in next SPCIF interrupt. */
    gl_flash_nb_write_abort = true;
}

static cy_en_pdstack_status_t flash_non_blocking_row_write(uint16_t row_num, uint8_t *data,
        flash_cbk_t cbk)
{
    uint8_t flash_macro_index;

    /* Determine the Flash Macro from Row number. */
    flash_macro_index = CY_FLASH_GET_MACRO_FROM_ROW(row_num);

    /* Load Flash Bytes SROM API. */
    if (!srom_load_flash_bytes_api (data, flash_macro_index))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }

    /* Connect the charge pump to IMO clock for flash write. */
    SRSSLT->clk_select = (SRSSLT->clk_select & ~CLK_SELECT_PUMP_SEL_MASK) | (1 << CLK_SELECT_PUMP_SEL_POS);

    /* Set SPC Interrupt Vector and enable Interrupt. */
    CyIntDisable (FLASH_SPC_INTR);
    CyIntSetVector (FLASH_SPC_INTR, &flash_spc_intr_handler);
    CyIntEnable (FLASH_SPC_INTR);

    /* Flash Callback. */
    flash_notify = cbk;

    /* Non Blocking Write Row API. */
    if (!(srom_nb_flash_write_api (row_num)))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }
    /*
     * Non-Blocking Flash Write has started. Response will
     * go back only after write completes.
     */
    return CCG_STAT_NO_RESPONSE;
}
#endif /* FLASH_ENABLE_NB_MODE */

ATTRIBUTES_SYS_FLASH cy_en_pdstack_status_t flash_row_write(uint16_t row_num, uint8_t *data, flash_cbk_t cbk)
{
    (void)cbk;
    /* Initialize Return Status Value. */
    cy_en_pdstack_status_t  status;
    
#if (!CCG_DUALAPP_DISABLE)
#if ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0))
    uint32_t seq_num;
    uint16_t offset;
#else /* ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0)) */
    sys_fw_metadata_t *fw_metadata;
#endif /* ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0)) */
#endif /* CCG_DUALAPP_DISABLE */

#if PSVP_FPGA_ENABLE
    /* Override the IMO for FPGA to 48MHz. Store original to restore. */
    uint32_t imo_val = SRSSLT_CLK_IMO_SELECT;
    SRSSLT_CLK_IMO_SELECT = 6;
#endif /* PSVP_FPGA_ENABLE */
    /*
     * On actual CCG silicon, we use customized flash update code for reduced stack usage.
     * This version of code can only work when the HFCLK setting for the device is 48 MHz.
     * Return error if flash write is being initiated with HFCLK set to values other than
     * 48 MHz.
     */
#if ((!PSVP_FPGA_ENABLE) && (CYDEV_BCLK__HFCLK__MHZ != 48) && 0)
    return (CY_PDSTACK_STAT_FLASH_UPDATE_FAILED);
#endif /* ((!PSVP_FPGA_ENABLE) && (CYDEV_BCLK__HFCLK__MHZ != 48)) */

    /* Can't handle flash update request if Flashing mode is not active. */
    if (CALL_MAP(gl_flash_mode_en) == 0u)
    {
        return CY_PDSTACK_STAT_NOT_READY;
    }

#ifndef DMC
    if ((data == NULL) || (row_num < CALL_MAP(gl_flash_access_first)) ||
            ((row_num > CALL_MAP(gl_flash_access_last))
#if (!CCG_BUILD_NONPROG_BOOT)
            && (row_num != CALL_MAP(gl_flash_metadata_row))
#endif /* !CCG_BUILD_NONPROG_BOOT */
            ) ||
            (row_num > CCG_LAST_FLASH_ROW_NUM))
#else
    if ((data == NULL) || (row_num < gl_flash_access_first) ||
        ((row_num > gl_flash_access_last) && (row_num != gl_flash_metadata_row) &&
          (
#if HX3_SLAVE_SUPPORT
           (row_num < HX3_FW2_DMC_FLASH_START_ROW_ID) ||
#else
           (row_num < DOCK_METADATA_START_ROW_ID) ||
#endif /* HX3_SLAVE_SUPPORT */
#if (USR_DEFINED_SN_SUPPORT)
           (row_num > DOCK_METADATA_END_ROW_ID_WITH_SN)
#else
           (row_num > DOCK_METADATA_END_ROW_ID)
#endif /* USR_DEFINED_SN_SUPPORT */
        )))
#endif /* DMC */
    {
        return CY_PDSTACK_STAT_INVALID_ARGUMENT;
    }
#if (!CCG_DUALAPP_DISABLE)
#if CCG_BOOT
    /*
     * Ensure boot loader is not allowed to write to reserved rows (if any) in FW2 Image area.
     * Certain applications use FW Image 2 area to store APP priority, Customer info etc.
     * This rows are sandwiched between last row of image 2 and image 2's metadata table row.
     */
    if ((row_num > CCG_IMG2_LAST_FLASH_ROW_NUM) && (row_num < CCG_IMG2_METADATA_ROW_NUM))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }
#endif /* CCG_BOOT */

#if ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0))
    /* Byte offset to the sequence number field in metadata. */
    offset  = (CY_FLASH_SIZEOF_ROW- CCG_METADATA_TABLE_SIZE + CCG_FW_METADATA_BOOTSEQ_OFFSET);
    if (row_num == CCG_IMG1_METADATA_ROW_NUM)
    {
#if CCG_PRIORITIZE_FW2
        /* Set sequence number to 0. */
        seq_num = 0;
#else /* !CCG_PRIORITIZE_FW2 */
        /* Set sequence number to 1 + that of FW2. */
        seq_num = boot_get_boot_seq (SYS_FW_MODE_FWIMAGE_2) + 1;
#endif /* CCG_PRIORITIZE_FW2 */

        ((uint32_t *)data)[offset / 4] = seq_num;
    }
    if (row_num == CCG_IMG2_METADATA_ROW_NUM)
    {
#if CCG_PRIORITIZE_FW1
        /* Set sequence number to 0. */
        seq_num = 0;
#else /* !CCG_PRIORITIZE_FW1 */
        /* Set sequence number to 1 + that of FW1. */
        seq_num = boot_get_boot_seq (SYS_FW_MODE_FWIMAGE_1) + 1;
#endif /* CCG_PRIORITIZE_FW1 */

        ((uint32_t *)data)[offset / 4] = seq_num;
    }
#else /* ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0)) */
    /*
     * Refer JITM#2, In FW mode, For metadata rows write,
     * FW image updates the coresponding pseudo metadata
     * row instead of actual metadata row.
     */
    if (row_num == CCG_IMG1_METADATA_ROW_NUM)
    {
        row_num = gl_img2_fw_metadata->boot_last_row;
        /*
         * Mark the METADATA_VALID as "CP" which indicates that FW flashing is
         * now complete. After RESET, this will signal the current FW to
         * validate the other image and then jump to it.
         */
        fw_metadata= (sys_fw_metadata_t *)(data + (CY_FLASH_SIZEOF_ROW
                - CCG_METADATA_TABLE_SIZE));
        fw_metadata->metadata_valid = SYS_PSEUDO_METADATA_VALID_SIG;
    }
    else if (row_num == CCG_IMG2_METADATA_ROW_NUM)
    {
        row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        /*
         * Mark the METADATA_VALID as "CP" which indicates that FW flashing is
         * now complete. After RESET, this will signal the current FW to validate
         * the other image and then jump to it.
         */
        fw_metadata= (sys_fw_metadata_t *)(data + (CY_FLASH_SIZEOF_ROW
                - CCG_METADATA_TABLE_SIZE));
        fw_metadata->metadata_valid = SYS_PSEUDO_METADATA_VALID_SIG;
    }
#endif /* ((CCG_BOOT != 0) || (CCG_PSEUDO_METADATA_DISABLE != 0)) */
#endif /* (CCG_DUALAPP_DISABLE) */

#if (FLASH_ENABLE_NB_MODE == 1)
   /*
    * Determine mode of flashing: Blocking or Non-Blocking based
    * on the callback function pointer.
    */
    if (cbk == NULL)
    {
        status = flash_blocking_row_write (row_num, data, false);
    }
    else
    {
        status = flash_non_blocking_row_write (row_num, data, cbk);
    }
#else
    /* Blocking Flash Row Write in Bootloader mode. */
    /* Handle only if Flashing mode is active. */
    status = flash_blocking_row_write (row_num, data, false);
#endif /* FLASH_ENABLE_NB_MODE */

#if PSVP_FPGA_ENABLE
    /* Restore the IMO for FPGA to original. */
    SRSSLT_CLK_IMO_SELECT = imo_val;
#endif /* PSVP_FPGA_ENABLE */

    return status;
}

ATTRIBUTES_SYS_FLASH cy_en_pdstack_status_t flash_row_read(uint16_t row_num, uint8_t* data)
{
    /* Can't handle flash update request if Flashing mode is not active. */
    if (CALL_MAP(gl_flash_mode_en) == 0u)
    {
        return CY_PDSTACK_STAT_NOT_READY;
    }

    /* We allow any row outside of the boot-loader to be read. */
    if ((data == NULL) || (row_num <= CALL_MAP(gl_flash_bl_last_row)) ||
            (row_num > CCG_LAST_FLASH_ROW_NUM))
    {
        return CY_PDSTACK_STAT_INVALID_ARGUMENT;
    }

    CY_PDUTILS_MEM_COPY (data, (const uint8_t *)(row_num << CCG_FLASH_ROW_SHIFT_NUM), CY_FLASH_SIZEOF_ROW);
    return CY_PDSTACK_STAT_SUCCESS;
}

#if (CYDEV_BCLK__HFCLK__MHZ == 48)
#if !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D))
bool is_sflash_row_writeable(uint16_t row_num);
bool is_sflash_row_writeable(uint16_t row_num)
{
    bool ret = false;
    (void)row_num;

#if (CY_SFLASH_NUMBER_USERROWS != 0)
    if (row_num < CY_SFLASH_NUMBER_USERROWS)
    {
#ifdef CCG3
        /* User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to. */
        if (row_num != 1)
            ret = true;
#endif /* CCG3 */

#ifdef DMC
        /* User SFLASH Row #1 contains configuration data and should not be written to.
         * User SFLASH Row #2 contains device test status and should not be written to.
         */
        if ((row_num != 1) && (row_num != 2))
            ret = true;
#endif /* DMC */

#ifdef CCG4
        /* User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to. */
        if (row_num != 1)
            ret = true;
#endif /* CCG4 */

#ifdef CCG5
        /* CCG5 does not have any user SFLASH rows. */
#endif /* CCG5 */

#ifdef CCG5C
        /* User SFLASH Row #0 contains OCP calibration data and should not be written to.
         * User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to.
         */
        if (row_num > 1)
            ret = true;
#endif /* CCG5C */

#ifdef CCG6
        /* User SFLASH Row #0 contains OCP calibration data and should not be written to.
         * User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to.
         */
        if (row_num > 1)
            ret = true;
#endif /* CCG6 */

#ifdef CCG3PA
        /* User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to. */
        if (row_num != 1)
            ret = true;
#endif /* CCG3PA */

#ifdef CCG3PA2
        /* User SFLASH Row #1 contains trim data for the CC IDAC and should not be written to. */
        if (row_num != 1)
            ret = true;
#endif /* CCG3PA2 */

#if ((defined(CCG6DF)) || (defined(CCG6SF)))
        /* CCG6DF/CCG6SF do not have any user SFLASH rows. */
#endif /* ((defined(CCG6DF)) || (defined(CCG6SF))) */
    }
#endif /* (CY_SFLASH_NUMBER_USERROWS != 0) */

    return ret;
}
#endif /* !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D)) */
#endif /* CYDEV_BCLK__HFCLK__MHZ == 48 */

#if !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D))
cy_en_pdstack_status_t sflash_row_write(uint16_t row_num, uint8_t *data)
{
    (void)row_num;
    (void)data;

    /*
     * On actual CCG silicon, we use customized flash update code for reduced stack usage.
     * This version of code can only work when the HFCLK setting for the device is 48 MHz.
     * Return error if flash write is being initiated with HFCLK set to values other than
     * 48 MHz.
     */
#if (CYDEV_BCLK__HFCLK__MHZ != 48)
    return (CY_PDSTACK_STAT_FLASH_UPDATE_FAILED);
    
#else
#if (CY_SFLASH_NUMBER_USERROWS != 0)
    /* Initialize Return Status Value. */
    cy_en_pdstack_status_t status;
    /* Check validity of row_num parameter. */
    if (!is_sflash_row_writeable (row_num))
    {
        return CY_PDSTACK_STAT_INVALID_ARGUMENT;
    }

    status = flash_blocking_row_write (row_num, data, true);
    return status;
#else
    return CY_PDSTACK_STAT_INVALID_ARGUMENT;
#endif /* (CY_SFLASH_NUMBER_USERROWS != 0) */
    
#endif /* (CYDEV_BCLK__HFCLK__MHZ != 48) */
}
#endif /* !(defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_CCG7D))*/

#define SFLASH_SYS_ROW_CNT      (4u)

cy_en_pdstack_status_t sflash_row_read(uint16_t row_num, uint8_t* data)
{
    (void)row_num;
    (void)data;
    cy_en_pdstack_status_t ret = CY_PDSTACK_STAT_INVALID_ARGUMENT;
#if (CY_SFLASH_NUMBER_USERROWS != 0)
    const uint8_t *src_addr = (uint8_t *)(SFLASH_BASE_ADDR + ((SFLASH_SYS_ROW_CNT + (uint32_t)row_num) << CCG_FLASH_ROW_SHIFT_NUM));

    if (row_num < CY_SFLASH_NUMBER_USERROWS)
    {
        CY_PDUTILS_MEM_COPY (data, src_addr, CY_FLASH_SIZEOF_ROW);
        ret = CY_PDSTACK_STAT_SUCCESS;
    }
#endif /* (CY_SFLASH_NUMBER_USERROWS != 0) */

    return ret;
}

#if APP_PRIORITY_FEATURE_ENABLE
cy_en_pdstack_status_t flash_set_app_priority(flash_app_priority_t app_priority)
{
    uint8_t temp_buf[CY_FLASH_SIZEOF_ROW+ CPUSS_FLASH_PARAM_SIZE] = {0};

    /* Ensure APP Priority value is valid. */
    if (app_priority > FLASH_APP_PRIORITY_IMAGE_2)
    {
        return CY_PDSTACK_STAT_INVALID_ARGUMENT;
    }
    else
    {
        /* Set APP Priority Field. */
        temp_buf[CPUSS_FLASH_PARAM_SIZE] = app_priority;

#if (CCG3PA2_FLASH_WRITE_ISSUE != 0)

        cy_en_pdstack_status_t status;
    
        if (CYRET_SUCCESS == CySysFlashWriteRow (CCG_APP_PRIORITY_ROW_NUM, temp_buf))
        {
            status = CY_PDSTACK_STAT_SUCCESS;
        }
        else
        {
            status = CY_PDSTACK_STAT_FLASH_UPDATE_FAILED;
        }
        return status;       
#else
            
        return flash_blocking_row_write (CCG_APP_PRIORITY_ROW_NUM, temp_buf + CPUSS_FLASH_PARAM_SIZE, false);
#endif
    }
}
#endif /* APP_PRIORITY_FEATURE_ENABLE */

/* [] END OF FILE */
