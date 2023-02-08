/******************************************************************************
* File Name: thermistor.h
* \version 2.0
*
* Description: Thermistor handler header file.
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


#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include <config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <cy_pdstack_common.h>

/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
/**
   @brief Defines the max temperature value that can be assigned for thermistor
 */
#define THERMISTOR_FAULT_TEMP                           (255u)
/** \} group_ccgxAppCommon_macros */

/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/

/**
 * @brief This function maps the thermal voltage read from thermistor to temperature value
 * @param ptrPdStackcontext PD stack context
 * @param therm_volt The thermal voltage that must be mapped to temperature
 * @return The temperature value corresponding to the thermal voltage read
 */
uint8_t ccg_volt_temp_map(cy_stc_pdstack_context_t *ptrPdStackcontext, uint16_t therm_volt);

/**
 * @brief This function reads the temperature of the specified sensor
 * @param ptrPdStackcontext PD stack context
 * @param sensor_id The index of the sensor to be read
 * @return The temperature value corresponding to the thermal voltage read
 */
uint8_t ccg_get_sensor_temperature(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t sensor_id);

/**
 * @brief This function maps the thermal voltage read from thermistor to temperature value
 * @param ptrPdStackcontext PD stack context
 * @param volt_temp_table Pointer to the table to be used for thermal voltage to temperature mapping
 * @return None
 */
void register_thermistor_mapping_table(cy_stc_pdstack_context_t *ptrPdStackcontext, const uint16_t *volt_temp_table);
/** \} group_ccgxAppCommon_functions */
/** \} group_ccgxAppCommon */

#endif /* THERMISTOR_H_ */

