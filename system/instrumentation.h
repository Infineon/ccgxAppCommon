/******************************************************************************
* File Name: instrumentation.h
*
* Description: Header file containing application instrumentation definitions
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

/**
* \addtogroup group_ccgxAppCommon Common source files
* \{
*/
#ifndef _INSTRUMENTATION_H_
#define _INSTRUMENTATION_H_

#include <stdint.h>
#include "cy_sw_timer.h"

/**
 * @brief Enumeration of all instrumentation fault events.
 */
typedef enum instrumentation_events
{
    INST_EVT_WDT_RESET = 0,                 /**< 0x00: Instrumentation fault event for watchdog reset. */
    INST_EVT_HARD_FAULT = 1                 /**< 0x01: Instrumentation fault event for hard fault. */
} inst_evt_t;

/**
 * @brief Callback function to solution level handler for instrumentation faults.
 */
typedef void (*instrumentation_cb_t)(uint8_t port, uint8_t evt);

/**
 * @brief Initialize data structures associated with application instrumentation.
 * @return None
 */
void instrumentation_init(void);

/**
 * @brief Start any timers or tasks associated with application instrumentation.
 * @return None
 */
void instrumentation_start(void);

/**
 * @brief Perform tasks associated with application instrumentation. The specific
 * functionality implemented is user defined and can vary.
 * @return None
 */
void instrumentation_task(void);

/**
 * @brief Register solution level callback function to be executed when instrumentation fault occurs.
 * @param cb Callback function
 * @return None
 */
void instrumentation_register_cb(instrumentation_cb_t cb);

/**
 * @brief Timer callback to reset device if main loop has not been run as expected.
 * @param id Timer Id
 * @param callbackContext Callback pointer
 */
void watchdog_timer_cb (cy_timer_id_t id, void *callbackContext);

#endif /* _INSTRUMENTATION_H_ */

/** \} group_ccgxAppCommon */