/******************************************************************************
* File Name: sensor_check.h
* \version 2.0
*
* Description: Sensor check handler header file.
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


#ifndef SENSOR_CHECK_H_
#define SENSOR_CHECK_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "cy_pdstack_common.h"
#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
#include <power_throttle.h>
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */
#if (CCG_HPI_AUTO_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
#include <hpi.h>
#endif /* (CCG_HPI_AUTO_ENABLE || CCG_HPI_AUTO_CMD_ENABLE) */

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
/**
   @brief Temperature sensor enable
 */
#define THROTTLE_TEMP_SENSOR_EN                     (0x80u)

/**
   @brief Defines the hysteresis value to be used for temperature(deg C) based power throttling
 */
#define THROTTLE_TEMPERATURE_HYSTERESIS             (5u)
/**
   @brief Defines the max temperature value that can be assigned
 */
#define THROTTLE_MAX_TEMP                           (255u)
/**
   @brief Address of the first register of the slave
 */
#define SLAVE_REG_ADD                               (0x01u)
/**
   @brief Define the voltage divider multiplier value
 */
#define THROTTLE_VOLT_DIVIDER                       (18u)
/**
   @brief Defines the hysteresis value to be used for VIN( 100 mV ) based power throttling
 */
#define THROTTLE_VIN_HYSTERESIS                     (5u)

#if CCG_HPI_AUTO_CMD_ENABLE
/**
   @brief Defines data type of OC Buffer
 */
#define CCG_OC_BUFFER_DATE_TYPE hpi_oc_buffer_t
#else
/**
   @brief Defines data type of OC Buffer
 */
#define CCG_OC_BUFFER_DATE_TYPE void
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
/** \} group_ccgxAppCommon_macros */
/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
/**
 * @brief This function initializes the sensor check block. This should be
 * called one time only at system startup.
 * @param ptrPdStackcontext PD stack context
 * @return None
 */
void ccg_sensor_init(cy_stc_pdstack_context_t *ptrPdStackcontext);

/**
 * @brief Facilitates to know if sensor check module is idle. System
 * sleep is allowed in idle state only. In idle state, ccg_ls_task()
 * can run in lower frequency.
 * @param ptrPdStackcontext PD stack context
 * @return False if sensor check module is idle. True otherwise.
 */
bool ccg_sensor_is_idle(cy_stc_pdstack_context_t *ptrPdStackcontext);

/**
 * @brief Facilitates caller to update system operating condition to
 * the operating condition indicated by VIN and temperature of the sensors.
 * @param ptrPdStackcontext PD stack context
 * @return None. 
 */
void ccg_sensor_check(cy_stc_pdstack_context_t *ptrPdStackcontext);

/**
 * @brief This function performs the debounce related checks. This will be
 * called continuously, but will be executed only if the flag is set.
 * @param ptrPdStackcontext PD stack context
 * @return None
 */
void ccg_sensor_debounce_task(cy_stc_pdstack_context_t *ptrPdStackcontext);

/**
 * @brief This function returns the current sensor temperature. This will be
 * called by the EC to know the temperature of the sensors
 * @param ptrPdStackcontext PD stack context
 * @param buffer to which the information will be copied
 * @return Success or Failure depending on whether temperature based throttling
 * is enabled in the firmware
 */
cy_en_pdstack_status_t ccg_sensor_temp_ec(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t *buffer);

/**
 * @brief This function extracts VIN(Battery) voltage.
 * @param ptrPdStackcontext PD stack context
 * @return Voltage in millivolts.
 */
uint16_t ccg_get_battery_voltage(cy_stc_pdstack_context_t *ptrPdStackcontext);

/**
 * @brief This function extracts undebounced OC specific information.
 * @param ptrPdStackcontext PD stack context
 * @param oc_buffer Placeholder for undebounced OC.
 * @return Undebounced system OC at this point of time.
 */
uint8_t ccg_get_sys_oc(cy_stc_pdstack_context_t *ptrPdStackcontext, CCG_OC_BUFFER_DATE_TYPE *oc_buffer);

/**
 * @brief This function configures the VIN OV/UV comparators for sleep mode.
 * @param ptrPdStackcontext PD stack context
 * @return None
 */
void ccg_set_sensor_sleep_mode(cy_stc_pdstack_context_t *ptrPdStackcontext);
/** \} group_ccgxAppCommon_functions */
/** \} group_ccgxAppCommon */

#endif /* SENSOR_CHECK_H_ */
