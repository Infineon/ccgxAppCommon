/******************************************************************************
* File Name: instrumentation.c
*
* Description: Source file containing application instrumentation definitions
*
* This USB-PD port controller application supports high level instrumentation
* to track the task execution latencies and runtime stack usage. This header
* file contains the definitions associated with these functions.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/

#include "instrumentation.h"
#include "cy_pdl.h"
#include <stddef.h>
#include <cy_pdstack_common.h>
#include "config.h"
#if CCG_HPI_ENABLE
#include "hpi.h"
#endif /* CCG_HPI_ENABLE */

/* Run-time stack lower limit defined in linker script. */
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
extern int __StackLimit;
#elif defined(__ARMCC_VERSION)
extern unsigned long Image$$ARM_LIB_STACK$$ZI$$Base;
#endif /* defined(__GNUC__) */

instrumentation_cb_t gl_instrumentation_cb = NULL;

#if RESET_ON_ERROR_ENABLE

/* RAM based signature and offset used to check whether reset data is valid. */
#define RESET_DATA_VALID_OFFSET         (0)
#define RESET_DATA_VALID_SIG            (0xDEADBEEF)

/* RAM based signature and offset used to check whether watchdog reset has happened. */
#define WATCHDOG_RESET_OFFSET           (1)
#define WATCHDOG_RESET_SIG              (0xC003300C)

/* RAM offset where the watchdog reset count is maintained. */
#define RESET_COUNT_OFFSET              (2)

/* Size of the reset tracking data structure in DWORDs. */
#define RESET_DATA_STRUCT_SIZE          (3)

/* gl_runtime_data_addr initialized here instead of init() to resolve MISRA
 * invalid pointer access issue.
 */
/* Address of the run-time instrumentation data structure. */
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
volatile uint32_t *gl_runtime_data_addr = (uint32_t volatile *)&__StackLimit;
#elif defined(__ARMCC_VERSION)
volatile uint32_t *gl_runtime_data_addr = (uint32_t *)&Image$$ARM_LIB_STACK$$ZI$$Base;
#else
volatile uint32_t *gl_runtime_data_addr = NULL;
#endif /* defined(__GNUC__) */


/* Variable used to identify whether main loop has been run. */
volatile uint32_t gl_main_loop_delay = 0;

/* Margin (in ms) available until watchdog reset. */
volatile uint16_t gl_min_reset_margin = WATCHDOG_RESET_PERIOD_MS;

extern cy_stc_pdstack_context_t pdstack_port0_ctx;

/* Timer callback to reset device if main loop has not been run as expected. */
void watchdog_timer_cb (
    cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
    void *callbackContext)       /**< Timer module Context. */
{
    (void)callbackContext;
    (void)id;

    /*
     * It is possible that this timer is the only reason for the device to wake from sleep.
     * Hence allow three consecutive timer expiry before resetting the device.
     */
    gl_main_loop_delay++;
    if (gl_main_loop_delay >= 2u)
    {
        if(gl_instrumentation_cb != NULL)
        {
            gl_instrumentation_cb(0, INST_EVT_WDT_RESET);
        }
        /* Store the reset signature into RAM. */
        gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
        NVIC_SystemReset ();
    }
#if !WATCHDOG_OVER_POLL_TIMER
    /* Start the timer again. */
    cy_sw_timer_start (pdstack_port0_ctx.ptrTimerContext, NULL, WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* !WATCHDOG_OVER_POLL_TIMER */
}

#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE
    
/*
 * Minimum run-time stack usage value. This many bytes at the top of the stack
 * will not be tracked for usage.
 */
#define MIN_STACK_USAGE     (256u)

/* Signature used to track stack usage. */
#define STACK_UNUSED_SIG    (0x00555500)

/* Address of the bottom location of the run-time stack. */
uint32_t *gStackBottom  = (uint32_t *)CYDEV_SRAM_BASE;
volatile uint16_t gMinStackMargin   = 0;

#endif /* STACK_USAGE_CHECK_ENABLE */

void instrumentation_init(void)
{
    uint32_t wdr_cnt = 0;

    /* Added to avoid compiler warning if all features are disabled. */
    (void)wdr_cnt;

#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p;

    /* Store the stack bottom location. */
#if defined(__GNUC__)
    gStackBottom = (uint32_t *)&__cy_stack_limit;
#else
    gStackBottom = (uint32_t *)&Image$$ARM_LIB_STACK$$ZI$$Base;
#endif /* defined(__GNUC__) */

#if RESET_ON_ERROR_ENABLE
    /* If we have watchdog reset tracking enabled, the lowest twelve bytes of stack cannot be used. */
    gStackBottom += RESET_DATA_STRUCT_SIZE;
#endif /* RESET_ON_ERROR_ENABLE */

    /* Fill the stack memory with unused signature. */
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        *addr_p = STACK_UNUSED_SIG;
    }
    
    /* Initialize the stack margin value. */
    gMinStackMargin = (uint16_t)((uint32_t)addr_p - (uint32_t)gStackBottom);

#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_37 (gMinStackMargin);
#endif /* CCG_HPI_ENABLE */

#endif /* STACK_USAGE_CHECK_ENABLE */
#if RESET_ON_ERROR_ENABLE
    if (gl_runtime_data_addr[RESET_DATA_VALID_OFFSET] == RESET_DATA_VALID_SIG)
    {
        wdr_cnt = gl_runtime_data_addr[RESET_COUNT_OFFSET];
        if (gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] == WATCHDOG_RESET_SIG)
        {
            wdr_cnt++;
        }
    }

    /*
     * Store the reset data valid signature and current reset count.
     * Also clear the reset detected signature.
     */
    gl_runtime_data_addr[RESET_DATA_VALID_OFFSET] = RESET_DATA_VALID_SIG;
    gl_runtime_data_addr[WATCHDOG_RESET_OFFSET]   = 0;
    gl_runtime_data_addr[RESET_COUNT_OFFSET]      = wdr_cnt;
    

#if HPI_WATCHDOG_RESET_ENABLE
    gl_min_reset_margin = 2 * WATCHDOG_RESET_PERIOD_MS;
    hpi_set_reserved_reg_35 (gl_min_reset_margin);
#endif /* HPI_WATCHDOG_RESET_ENABLE */

#endif /* RESET_ON_ERROR_ENABLE */

#if ((CCG_HPI_ENABLE) && (HPI_WATCHDOG_RESET_ENABLE))
    /* Store the reset count into the HPI register. */
    if (wdr_cnt > 0xFFu)
        wdr_cnt = 0xFFu;
    hpi_set_reset_count (wdr_cnt);
#endif /* ((CCG_HPI_ENABLE) && (HPI_WATCHDOG_RESET_ENABLE)) */
}

void instrumentation_start(void)
{
#if RESET_ON_ERROR_ENABLE
#if !WATCHDOG_OVER_POLL_TIMER
    /* Start the timer used for watchdog reset. */
    cy_sw_timer_start (pdstack_port0_ctx.ptrTimerContext, NULL,  WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* !WATCHDOG_OVER_POLL_TIMER */
#endif /* RESET_ON_ERROR_ENABLE */

#if WATCHDOG_HARDWARE_RESET_ENABLE
    /*
     * Enable WDT hardware reset.
     * WDT interrupt flag is expected to be cleared by software timer module
     * (At the least WATCHDOG_TIMER_ID is active always).
     * If WDT interrupt handler is not executed because of CPU lock up and
     * the WDT interrupt flag is not cleared for the three consecutive
     * interrupts, a hardware reset is triggered for the recovery.
     */
#ifdef CY_DEVICE_PAG1S
    SRSSULT->wdt_disable_key = 0;
#else
    Cy_WDT_Enable();
#endif /* CY_DEVICE_PAG1S */
#endif /* WATCHDOG_HARDWARE_RESET_ENABLE */
}

void instrumentation_task(void)
{
#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p = gStackBottom;
#endif /* STACK_USAGE_CHECK_ENABLE */

#if RESET_ON_ERROR_ENABLE

    /* Clear the variable to indicate main loop has been run. */
    gl_main_loop_delay = 0;

#if CALCULATE_TIMER_MARGINS
    /* Calculate how much time is left before device could get reset. */
    gl_min_reset_margin = CY_USBPD_GET_MIN(gl_min_reset_margin,
            ((1 - gl_main_loop_delay) * WATCHDOG_RESET_PERIOD_MS) + timer_get_count(0, WATCHDOG_TIMER_ID));
#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_35 (gl_min_reset_margin);
#endif /* CCG_HPI_ENABLE */

    /* Stop and restart the timer. */
    timer_stop (0, WATCHDOG_TIMER_ID);
    timer_start (0, WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* CALCULATE_TIMER_MARGINS */

#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        if (*addr_p != STACK_UNUSED_SIG)
        {
            break;
        }
    }
    
    /* Calculate the minimum stack availability margin and update debug register. */
    gMinStackMargin = CY_USBPD_GET_MIN(gMinStackMargin, ((uint32_t)addr_p - (uint32_t)gStackBottom));
#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_37(gMinStackMargin);
#endif /* CCG_HPI_ENABLE */

#endif /* STACK_USAGE_CHECK_ENABLE */
}

void HardFault_Handler(void)
{
    if(gl_instrumentation_cb != NULL)
    {
        gl_instrumentation_cb(0, INST_EVT_HARD_FAULT);
    }
        
#if RESET_ON_ERROR_ENABLE
    /* Store the reset signature into RAM. */
    gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
    NVIC_SystemReset ();
#else 
    Cy_SysLib_ProcessingFault(); 
#endif /* RESET_ON_ERROR_ENABLE*/
}

void instrumentation_register_cb(instrumentation_cb_t cb)
{
    if(cb != NULL)
    {
        gl_instrumentation_cb = cb;
    }
}
/* End of file */
