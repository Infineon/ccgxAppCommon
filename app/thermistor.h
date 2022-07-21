/***************************************************************************//**
* \file thermistor.h
* \version 1.1.0 
*
* Thermistor handler header file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
* \addtogroup group_ccgxAppCommon Common source files
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
   @brief Defines the max temperature value that can be assigned for thermistor
 */
#define THERMISTOR_FAULT_TEMP                           (255u)
    
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

/** \} group_ccgxAppCommon */

#endif /* THERMISTOR_H_ */

