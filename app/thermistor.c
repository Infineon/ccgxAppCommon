/******************************************************************************
* File Name: thermistor.c
* \version 2.0
*
* Description: Thermistor handler source file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include <thermistor.h>
#include "srom.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_gpio.h"
#include "gpio.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "app_timer_id.h"
#include "app.h"

#if (TEMPERATURE_SENSOR_COUNT != 0u)
extern app_sln_handler_t *solution_fn_handler;
#if TEMPERATURE_SENSOR_IS_THERMISTOR
/* Array holds the pins to which the thermistors are attached */
static gpio_port_pin_t gl_thermistor_gpio[NO_OF_TYPEC_PORTS][TEMPERATURE_SENSOR_COUNT] = 
{
    {PORT_0_THERMISTOR_0_PIN_CONNECTION
#if TEMPERATURE_SENSOR_COUNT > 1        
        ,PORT_0_THERMISTOR_1_PIN_CONNECTION
#endif /* TEMPERATURE_SENSOR_COUNT > 1 */
    }
#if (NO_OF_TYPEC_PORTS > 1)
    ,{PORT_1_THERMISTOR_0_PIN_CONNECTION
#if TEMPERATURE_SENSOR_COUNT > 1        
        ,PORT_1_THERMISTOR_1_PIN_CONNECTION
#endif /* TEMPERATURE_SENSOR_COUNT > 1 */ 
    }
#endif /* (NO_OF_TYPEC_PORTS >= 2) */
};

/* Pointer to the voltage to temperature mapping table */
static const uint16_t *gl_volt_temp_map;

void register_thermistor_mapping_table(cy_stc_pdstack_context_t *ptrPdStackcontext, const uint16_t *volt_temp_table)
{
    (void)ptrPdStackcontext;
    gl_volt_temp_map = volt_temp_table;    
}

/* Function to map voltage to temperature */
uint8_t ccg_volt_temp_map(cy_stc_pdstack_context_t *ptrPdStackcontext, uint16_t therm_volt)
{
    (void)ptrPdStackcontext;
    uint8_t idx, temperature = THROTTLE_SENSOR_FAULT;
    uint8_t start = 0, stop = VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u;
    uint16_t resolution;
    uint16_t diff;
    if(gl_volt_temp_map == NULL)
    {
        return THROTTLE_SENSOR_FAULT;
    }
#if THERMISTOR_IS_NTC
    if(!((therm_volt <= *gl_volt_temp_map) && 
            !(therm_volt < *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u))))
#else
    if(!((therm_volt >= *gl_volt_temp_map) && 
            !(therm_volt > *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT - 1u))))
#endif /* THERMISTOR_IS_NTC */    
    {
        /* If the code flow comes here, it means that the entry is not present in the table.
         * If the thermistor measured temperature is less than the starting temperature
         * value of the hash table, we will consider the measured tempearture as
         * safe temperature. This will enable the customers to have a smaller hash table
         * of their preferred range.
         * Note: There is no requirement to execute throttling in safe temperature range.
         *
         * However fault condition (0xFF) will be reported if the measured temperature is beyond
         * the highest temperature mapped in the table.
         */
#if THERMISTOR_IS_NTC
        if((therm_volt > (*gl_volt_temp_map)))
#else
        if((therm_volt < (*gl_volt_temp_map)))
#endif /* THERMISTOR_IS_NTC */
        {
            temperature = SAFE_DEFAULT_TEMPERATURE;
        }
    }
    else
    {
#if THERMISTOR_IS_NTC
        if(therm_volt <= *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u))
#else
        if(therm_volt >= *(gl_volt_temp_map + VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u))
#endif /* THERMISTOR_IS_NTC */
        {
            start = VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u;            
        }
        else
        {
            stop = VOLT_TO_TEMP_MAP_TABLE_COUNT / 2u;
        }
        
        for(idx = start; idx <= stop; idx++)
        {
#if THERMISTOR_IS_NTC
            if(therm_volt < *(gl_volt_temp_map + idx))
#else
            if(therm_volt > *(gl_volt_temp_map + idx))
#endif /* THERMISTOR_IS_NTC */
            {
                /* Do not do anything. Continue searching in the table */
            }
            else
            {
                if (start != idx)
                {
#if THERMISTOR_IS_NTC
                    resolution = (*(gl_volt_temp_map + idx - 1u) - *(gl_volt_temp_map + idx))/TEMP_MAP_RESOLUTION;
                    diff = *(gl_volt_temp_map + idx - 1u) - therm_volt;      
#else
                    resolution = (*(gl_volt_temp_map + idx) - *(gl_volt_temp_map + idx - 1u))/TEMP_MAP_RESOLUTION;
                    diff = therm_volt - *(gl_volt_temp_map + idx - 1u);
#endif /* THERMISTOR_IS_NTC */

                    temperature = BASE_MAP_TEMP + TEMP_MAP_RESOLUTION * (idx - 1u);

                    if (resolution > 0u)
                    {
                        temperature += (uint8_t)(diff / resolution);
                    }
                    else
                    {
                        /* No Action */
                    }
                }
                else
                {
                    temperature = BASE_MAP_TEMP + TEMP_MAP_RESOLUTION * start;
                }
                break;
            } 
        }
    }    
    return temperature;
}
#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */

/* Function to get the sensor temperature */
uint8_t ccg_get_sensor_temperature(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t sensor_id)
{
    uint8_t temperature = 0u;
    uint8_t port = ptrPdStackcontext->port;
    (void)sensor_id;
    (void)port;
#if TEMPERATURE_SENSOR_IS_I2C    
    uint8_t read = 0;
    uint8_t addr = SLAVE_REG_ADD + sensor_id;
    
    status = i2cm_reg_read(CCG_I2C_MASTER_SCB_INDEX, THROTTLE_SLAVE_ADDRESS, &read, 1, &addr, 1);
    if(status == CY_PD_STAT_SUCCESS)
    {   /* Start reading from the specified register */
        temperature = read;
    }
#endif /* TEMPERATURE_SENSOR_IS_I2C */

#if TEMPERATURE_SENSOR_IS_THERMISTOR

    uint8_t level, state;
    uint16_t therm_volt;
    state = Cy_SysLib_EnterCriticalSection();

    /* Need to check if sensor is missing. */
    CALL_MAP(hsiom_set_config)(gl_thermistor_gpio[port][sensor_id], APP_GPIO_POLL_ADC_HSIOM);

    Cy_SysLib_DelayUs(20);

    /* Take ADC sample. */
#if (!(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)))
    (void)Cy_USBPD_Adc_SelectVref(ptrPdStackContext->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, CY_USBPD_ADC_VREF_VDDD);
#endif /* !(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
    level = Cy_USBPD_Adc_Sample(solution_fn_handler->Get_PdStack_Context(0)->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, APP_GPIO_POLL_ADC_INPUT);
    therm_volt = ((level * THROTTLE_VDDD_REFERENCE)/PD_ADC_NUM_LEVELS);
#if (!(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)))
    (void)Cy_USBPD_Adc_SelectVref(ptrPdStackContext->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, CY_USBPD_ADC_VREF_PROG);
#endif /* !(defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

    CALL_MAP(hsiom_set_config)(gl_thermistor_gpio[port][sensor_id], HSIOM_MODE_GPIO);

    Cy_SysLib_DelayUs(10);
    Cy_SysLib_ExitCriticalSection(state);
    /* Need to handle cases when the thermistor readings are incorrect or when we are unable to sample ADC */
    temperature = ccg_volt_temp_map(ptrPdStackcontext, therm_volt);

#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */    

    return temperature; 
}

#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */
