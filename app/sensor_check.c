/******************************************************************************
* File Name: sensor_check.c
* \version 2.0
*
* Description: Vin and temperature based throttling source file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include <config.h>
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
#include "cy_pdstack_common.h"
#include <cy_pdstack_dpm.h>
#include <pdo.h>
#include <app.h>
#include <cy_pdutils_sw_timer.h>
#include <sensor_check.h>
#include "srom.h"
#include "psource.h"
#include <thermistor.h>
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_gpio.h"
#include "cy_pdutils.h"
#include "cy_usbpd_config_table.h"
#include "srom_dependency.h"
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE)
extern app_sln_handler_t *solution_fn_handler;
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE) */

#if !(CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE || CCG_LOAD_SHARING_ENABLE)
operating_condition_t ccg_power_throttle_get_oc_ec(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    (void)ptrPdStackContext;
    return SYSTEM_OC_1;
}
#endif /* #if !(CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE || CCG_LOAD_SHARING_ENABLE) */

#if CCG_TEMP_BASED_VOLTAGE_THROTTLING

/* Array to hold the sensor OCs */
static operating_condition_t gl_sensor_oc[NO_OF_TYPEC_PORTS][TEMPERATURE_SENSOR_COUNT];
/* Array to hold the temperature of the sensors*/
static uint8_t gl_sensor_temp[NO_OF_TYPEC_PORTS][TEMPERATURE_SENSOR_COUNT];
/* Array to hold the OC specific threshold for each sensor */
static uint8_t gl_sensor_thresh[NO_OF_TYPEC_PORTS][TEMPERATURE_SENSOR_COUNT];

#if CCG_TEMP_BASED_VOLTAGE_THROTTLING
/* Funtion to get the temperature OC */
static uint8_t ccg_get_temp_oc(uint8_t port)
{ 
    uint8_t i;
    uint8_t temperature;
    uint8_t max_temp_oc = 0;
    operating_condition_t new_temp_oc;

    sensor_data_t *sensor_cfg = pd_get_ptr_auto_cfg_tbl(solution_fn_handler->Get_PdStack_Context(port)->ptrUsbPdContext)->sensor_data;

#if (TEMPERATURE_SENSOR_COUNT > 1)
    for(i = 0; i < TEMPERATURE_SENSOR_COUNT; i++)
#else
    i = 0;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1) */
    {
        if((sensor_cfg[i].sensor_ctrl & THROTTLE_TEMP_SENSOR_EN) != 0u)
        {
            temperature = ccg_get_sensor_temperature(solution_fn_handler->Get_PdStack_Context(port), i);

            if(temperature != (uint8_t)THROTTLE_SENSOR_FAULT)
            {   
                gl_sensor_temp[port][i] = temperature;
            }
            else 
            {
                /*
                 * If a sensor fault or i2c read fail occurs
                 * We will start incrementing the corresponding sensor temperature
                 * We will increment till 254, So the highest OC of the sensor will be reached
                 */
                if(gl_sensor_temp[port][i] < (THROTTLE_MAX_TEMP - THROTTLE_SENSOR_FAULT_INCR))
                {                       
                    gl_sensor_temp[port][i] += THROTTLE_SENSOR_FAULT_INCR;
                }
            }
            
            temperature = gl_sensor_temp[port][i];
            /*
             * Apply hysteresis if we are going higher in power
             */
            if(gl_sensor_temp[port][i] <= gl_sensor_thresh[port][i])
            {
                if(temperature < (THROTTLE_MAX_TEMP - THROTTLE_TEMPERATURE_HYSTERESIS))
                {
                        temperature += THROTTLE_TEMPERATURE_HYSTERESIS;
                }
            }

            if(temperature <= sensor_cfg[i].sensor_oc1)
            {
                new_temp_oc = SYSTEM_OC_1;
            }
            else if(temperature <= sensor_cfg[i].sensor_oc2)
            {
                new_temp_oc = SYSTEM_OC_2;
            }
            else if(temperature <= sensor_cfg[i].sensor_oc3)
            {
                new_temp_oc = SYSTEM_OC_3;
            }
            else
            {
                new_temp_oc = SYSTEM_OC_4;
            }
            
            /*
             * If there is a change in sensor OC, we have to update the sensor threshold 
             */
            if(new_temp_oc != gl_sensor_oc[port][i])
            {
                   if(new_temp_oc == SYSTEM_OC_1)
                {
                    gl_sensor_thresh[port][i] = 0;
                }
                else if(new_temp_oc == SYSTEM_OC_2)
                {
                    gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc1;
                }
                else if(new_temp_oc == SYSTEM_OC_3)
                {
                    if(sensor_cfg[i].sensor_oc2 != 0u)
                    {
                        gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc2;
                    }
                    else
                    {
                        gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc1;                
                    }
                }
                else
                {
                    if(sensor_cfg[i].sensor_oc3 != 0u)
                    {
                        gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc3;
                    }
                    else
                    {
                        if(sensor_cfg[i].sensor_oc2 != 0u)
                        {
                            gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc2;
                        }
                        else
                        {
                            gl_sensor_thresh[port][i] = sensor_cfg[i].sensor_oc1;                
                        }                
                    }
                }
            gl_sensor_oc[port][i] = new_temp_oc;
            }
        }
    }
#if (TEMPERATURE_SENSOR_COUNT > 1)
    for (i = 0; i < TEMPERATURE_SENSOR_COUNT; i++)
#else
    i = 0;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1) */
    {
        if( max_temp_oc < (uint8_t)gl_sensor_oc[port][i])
        {
            max_temp_oc = (uint8_t)gl_sensor_oc[port][i];
        }
    }
    return max_temp_oc;
}
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */

#endif /* #if CCG_TEMP_BASED_VOLTAGE_THROTTLING */

uint16_t ccg_get_battery_voltage(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
#if defined(CY_DEVICE_CCG3PA)    
   /* Measure voltage on VBATT_MON GPIO. */
    uint8_t level, state;
    uint16_t vbatt;

    /* Since we are modifying HSIOM, we need to lock the bus. */
    state = Cy_SysLib_EnterCriticalSection();

    /* Connect the GPIO to AMUXA bus. */
    CALL_MAP(Cy_GPIO_SetHSIOM)(GPIO_PRT0, VBATT_MON_GPIO, HSIOM_SEL_AMUXA);
    Cy_SysLib_DelayUs(10);
    /* Take ADC sample. */
    level = Cy_USBPD_Adc_Sample(ptrPdStackcontext->ptrUsbPdContext, CY_USBPD_ADC_ID_1, CY_USBPD_ADC_INPUT_AMUX_A);
    /* Convert level to voltage. */
    vbatt = Cy_USBPD_Adc_LevelToVolt(ptrPdStackcontext->ptrUsbPdContext, CY_USBPD_ADC_ID_1, level);
    /* Revert the HSIOM connection. */
    CALL_MAP(Cy_GPIO_SetHSIOM)(GPIO_PRT0, VBATT_MON_GPIO, HSIOM_SEL_GPIO);
    Cy_SysLib_DelayUs(10);
    Cy_SysLib_ExitCriticalSection(state);

    /* Convert voltage to account for the resistor divider. */
    return (vbatt * THROTTLE_VOLT_DIVIDER);
#elif (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    return Cy_USBPD_MeasureVbat(ptrPdStackcontext->ptrUsbPdContext);
#endif
}

#if CCG_VIN_BASED_VOLTAGE_THROTTLING 
    
static operating_condition_t gl_battery_oc[NO_OF_TYPEC_PORTS];
static uint8_t gl_thres_volt[NO_OF_TYPEC_PORTS];

#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))

#if VIN_UVP_ENABLE
/* VIN UV comparator enable status */
static bool gl_vin_uv_enable[NO_OF_TYPEC_PORTS];
#endif /* VIN_UVP_ENABLE */

#if VIN_OVP_ENABLE
/* VIN OV comparator enable status */
static bool gl_vin_ov_enable[NO_OF_TYPEC_PORTS];
#endif /* VIN_OVP_ENABLE */

/* Callback to handle VIN UV and OV interrupts */
static void soln_vin_ovp_uvp_cbk(void *callbackContext, bool state);

void ccg_set_sensor_sleep_mode(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
    uint8_t port = ptrPdStackcontext->port;
    (void)port;
#if VIN_UVP_ENABLE
    if(gl_vin_uv_enable[port] == true)
    {
        Cy_USBPD_Fault_VinUvpDis(ptrPdStackcontext->ptrUsbPdContext, CCG_SRC_FET);
        gl_vin_uv_enable[port] = false;
    }
#endif /* VIN_UVP_ENALBE */

#if VIN_OVP_ENABLE
    if(gl_vin_ov_enable[port] == true)
    {
        Cy_USBPD_Fault_VinOvpDis(ptrPdStackcontext->ptrUsbPdContext, CCG_SRC_FET);
        gl_vin_ov_enable[port] = false;    
    }
#endif /* VIN_OVP_ENALBE */
}

/* VIN UV and OV comparator debounce in uS */
#define VIN_UV_OV_DEBOUNCE_US       (10u)

#endif /* ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE)
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
/* Global variable to hold the VIN voltage that was read using ADC */    
static uint16_t gl_last_read_bat_volt = 0;    
#endif /* defined (CCG7D) */
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE) */

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE)
/* Function to get the battery OC */
static uint8_t ccg_get_battery_oc(uint8_t port)
{
    cy_stc_pdstack_context_t *ptrPdStackcontext = solution_fn_handler->Get_PdStack_Context(port);
    auto_cfg_settings_t *battery_cfg = pd_get_ptr_auto_cfg_tbl(ptrPdStackcontext->ptrUsbPdContext);
    operating_condition_t bat_oc;
    uint16_t current_voltage;

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /*
     * For CCG7D, VIN is a system parameter and is common for both ports. 
     * To ensure both the ports shut-down on detecting VIN going above max VIN safe voltage and
     * both ports restart on detecting VIN go below the threshold, We read the VIN value
     * using ADC only for Port 0 and use the same reading for Port 1.
     */
    if(port == TYPEC_PORT_0_IDX)
    {
        current_voltage = ccg_get_battery_voltage(ptrPdStackcontext) / 100u;
        gl_last_read_bat_volt = current_voltage;
    }
    else
    {
        current_voltage = gl_last_read_bat_volt;
    }
#else
    current_voltage = ccg_get_battery_voltage(ptrPdStackcontext) / 100u;   
#endif /* (defined(CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))
    uint16_t threshold;
#endif /* ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */

    /* 
     * Upward hysteresis is applied on top of the threshold voltage for 
     * all OC transitions below maximum VIN safe voltage.
     * Whereas, downward hysteresis is applied when transitioning
     * from maximum VIN safe voltage OC to normal OC.
     */

    /* OC4 and downward hysteresis for maximum VIN safe voltage. */
    if((current_voltage > battery_cfg->vin_fault_max_safe_voltage) ||
       ((gl_battery_oc[port] == SYSTEM_OC_4) &&
        (current_voltage > ((uint16_t)battery_cfg->vin_fault_max_safe_voltage - (uint16_t)THROTTLE_VIN_HYSTERESIS))))
    {
        bat_oc = SYSTEM_OC_4;
    }
    else
    {
        /* OC and upward hysteresis for lower voltages. */
        if(gl_thres_volt[port] <= current_voltage)
        {
            if(current_voltage > THROTTLE_VIN_HYSTERESIS)
            {
                current_voltage -= THROTTLE_VIN_HYSTERESIS;
            }
        }

        if(current_voltage >= battery_cfg->vin_oc1)
        { 
            bat_oc = SYSTEM_OC_1;
        }
        else if(current_voltage >= battery_cfg->vin_oc2)
        { 
            bat_oc = SYSTEM_OC_2;
        }
        else if(current_voltage >= battery_cfg->vin_oc3)
        { 
            bat_oc = SYSTEM_OC_3;
        }
        else
        {
            bat_oc = SYSTEM_OC_4;
        }
    }

    /*
     * If there is a change in battery OC, we have to update Vin threshold 
     */
    if(gl_battery_oc[port] != bat_oc)
    {
        if(bat_oc == SYSTEM_OC_1)
        {
            gl_thres_volt[port] = 0xFF;
        }
        else if(bat_oc == SYSTEM_OC_2)
        {
            gl_thres_volt[port] = battery_cfg->vin_oc1;
        }
        else if(bat_oc == SYSTEM_OC_3)
        {
            if(battery_cfg->vin_oc2 != 0xFFu)
            {
                gl_thres_volt[port] = battery_cfg->vin_oc2;
            }
            else
            {
                gl_thres_volt[port] = battery_cfg->vin_oc1;
            }
        }
        else
        {
            if (battery_cfg->vin_oc3 != 0xFFu)
            {
                gl_thres_volt[port] = battery_cfg->vin_oc3;
            }
            else if (battery_cfg->vin_oc2 != 0xFFu)
            {
                gl_thres_volt[port]  = battery_cfg->vin_oc2;
            }
            else
            {
                gl_thres_volt[port] = battery_cfg->vin_oc1;
            }
        }
    }

    gl_battery_oc[port] = bat_oc;
#if VIN_UVP_ENABLE
    /* VIN UV uses upward hysteresis */
    if((gl_vin_uv_enable[port] == false) && (bat_oc != SYSTEM_OC_4))
    {
        if (battery_cfg->vin_oc3 != 0xFFu)
        {
            threshold = battery_cfg->vin_oc3;
        }
        else if (battery_cfg->vin_oc2 != 0xFFu)
        {
            threshold = battery_cfg->vin_oc2;
        }
        else
        {
            threshold = battery_cfg->vin_oc1;
        }

        /* Convert threshold from 100mV to mV units */
        threshold = (threshold * 100u) - VIN_UV_COMP_HYSTERESIS;

        gl_vin_uv_enable[port] = true;

        Cy_USBPD_Fault_VinUvpEn(ptrPdStackcontext->ptrUsbPdContext, threshold, soln_vin_ovp_uvp_cbk, CCG_SRC_FET, (cy_en_usbpd_vbus_uvp_mode_t)VIN_UVP_MODE);
    }
#endif /* VIN_UVP_ENBALE */

#if VIN_OVP_ENABLE
    /* VIN OV uses downward hysteresis */ 
    if((gl_vin_ov_enable[port] == false) && (bat_oc != SYSTEM_OC_4))
    {
        /* Convert threshold from 100mV to mV units */
        threshold = ((uint16_t)battery_cfg->vin_fault_max_safe_voltage * 100u) + VIN_OV_COMP_HYSTERESIS;

        gl_vin_ov_enable[port] = true;
        Cy_USBPD_Fault_VinOvpEn(ptrPdStackcontext->ptrUsbPdContext, threshold,
            soln_vin_ovp_uvp_cbk, CCG_SRC_FET, (cy_en_usbpd_vbus_ovp_mode_t)VIN_OVP_MODE);
    }
#endif /* VIN_UVP_ENBALE */
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    CY_UNUSED_PARAMETER(gl_last_read_bat_volt);
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */
    return (uint8_t)gl_battery_oc[port];
}
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE) */
#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))

void ccg_throttle_cb(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t power);

static void soln_vin_ovp_uvp_cbk(void *callbackContext, bool state)
{
    cy_stc_pdstack_context_t *context;

    uint8_t vin_oc;
    uint8_t idx;

    for(idx = TYPEC_PORT_0_IDX; idx < NO_OF_TYPEC_PORTS; idx++)
    {
        context = solution_fn_handler->Get_PdStack_Context(idx);

        /* First disable the FET. Then handle and disable protection. */
        psrc_disable(context, NULL);
    }

    for(idx = TYPEC_PORT_0_IDX; idx < NO_OF_TYPEC_PORTS; idx++)
    {
        context = solution_fn_handler->Get_PdStack_Context(idx);
#if VIN_OVP_ENABLE
        /* VIN OV fault event. */
        Cy_USBPD_Fault_VinOvpDis(context->ptrUsbPdContext, CCG_SRC_FET);
        gl_vin_ov_enable[idx] = false;
#endif /* VIN_OVP_ENABLE */

#if VIN_UVP_ENABLE
        /* VIN UV fault event. */
        Cy_USBPD_Fault_VinUvpDis(context->ptrUsbPdContext, CCG_SRC_FET);
        gl_vin_uv_enable[idx] = false;
#endif /* VIN_UVP_ENABLE */
        /* 
         * Set system OC to OC4.
         * If set OC fails here, set OC for OC4 shall be
         * attempted from the task routine.
         */
        fault_handler_clear_counts (context);
        if(gl_battery_oc[idx] != SYSTEM_OC_4)
        {
            /* Set OC4 only if port is not already in OC4 */
            (void)ccg_power_throttle_set_oc(context, SYSTEM_OC_4, ccg_throttle_cb);
            gl_battery_oc[idx] = SYSTEM_OC_4;
#if VIN_UVP_ENABLE
            auto_cfg_settings_t *battery_cfg = pd_get_ptr_auto_cfg_tbl(solution_fn_handler->Get_PdStack_Context(idx)->ptrUsbPdContext);
            if (battery_cfg->vin_oc3 != 0xFFu)
            {
                gl_thres_volt[idx] = battery_cfg->vin_oc3;
            }
            else if (battery_cfg->vin_oc2 != 0xFFu)
            {
                gl_thres_volt[idx]  = battery_cfg->vin_oc2;
            }
            else
            {
                gl_thres_volt[idx] = battery_cfg->vin_oc1;
            }
#endif /* VIN_UVP_ENABLE */ 
            /* Call get battery OC to update status and thresholds to OC4 */
            vin_oc = ccg_get_battery_oc(idx);
           (void)vin_oc;
        }
    }

    for(idx = TYPEC_PORT_0_IDX; idx < NO_OF_TYPEC_PORTS; idx++)
    {
        context = solution_fn_handler->Get_PdStack_Context(idx);

        if (state == false)
        {
            app_event_handler(context, APP_EVT_VIN_UVP_FAULT, NULL);
        }
        else
        {
            app_event_handler(context, APP_EVT_VIN_OVP_FAULT, NULL);
        }
    }
}
#endif /* ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */

#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING */

/* This function returns the new system OC */
uint8_t ccg_get_sys_oc(cy_stc_pdstack_context_t *ptrPdStackcontext, CCG_OC_BUFFER_DATE_TYPE *oc_buffer)
{
    uint8_t vin_oc = (uint8_t)SYSTEM_OC_1;
    uint8_t temp_oc = (uint8_t)SYSTEM_OC_1;
    uint8_t port = ptrPdStackcontext->port;
    (void)port;
    (void)oc_buffer;

#if CCG_HPI_AUTO_CMD_ENABLE
    if (NULL != oc_buffer)
    {
        vin_oc = 0xFF;
        temp_oc = 0xFF;
    }
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

#if CCG_TEMP_BASED_VOLTAGE_THROTTLING
    if((ccg_power_throttle_get_feature_mask(ptrPdStackcontext) & CCG_ENABLE_TEMP_BASED_THROTTLING) != 0u)
    {
        temp_oc = ccg_get_temp_oc(port);
    }
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */  
 
#if CCG_VIN_BASED_VOLTAGE_THROTTLING
    if((ccg_power_throttle_get_feature_mask(ptrPdStackcontext) & CCG_ENABLE_VIN_BASED_THROTTLING) != 0u)
    {
        if((pd_get_ptr_auto_cfg_tbl(ptrPdStackcontext->ptrUsbPdContext)->vin_throttling_ctrl) != 0u)
        {
            vin_oc = ccg_get_battery_oc(ptrPdStackcontext->port);
        }
    }
#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING */

#if CCG_HPI_AUTO_CMD_ENABLE
    if (NULL != oc_buffer)
    {
        ((hpi_oc_buffer_t *)oc_buffer)->oc_vin = vin_oc;
        ((hpi_oc_buffer_t *)oc_buffer)->oc_temp = temp_oc;
        if (vin_oc == 0xFFu)
        {
            vin_oc = (uint8_t)SYSTEM_OC_1;
        }
        if (temp_oc == 0xFFu)
        {
            temp_oc = (uint8_t)SYSTEM_OC_1;
        }
    }
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
    return (CY_USBPD_GET_MAX(CY_USBPD_GET_MAX(vin_oc, temp_oc), (uint8_t)ccg_power_throttle_get_oc_ec(ptrPdStackcontext)));
}

#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
/* This macro defines the number of samples for debounce */
#define THROTTLE_DEBOUNCE_VALUE 10
/* Initialize the debounce variable to 0 */
static uint8_t gl_sensor_debounce_count[NO_OF_TYPEC_PORTS];
/* Holds the OC condition at start of debounce */
static operating_condition_t gl_debounce_oc[NO_OF_TYPEC_PORTS];
static bool gl_debouncing[NO_OF_TYPEC_PORTS]; 
static bool gl_timer_exp[NO_OF_TYPEC_PORTS];

#if !((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))
void ccg_throttle_cb(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t power);
#endif /* !((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */

void ccg_throttle_cb(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t power)
{
    (void)power;
    gl_debouncing[ptrPdStackcontext->port] = false;
}
#if (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE)
static void ccg_debounce_cb(cy_timer_id_t id,  void * callbackCtx)
{
    (void)id;
    cy_stc_pdstack_context_t* context = callbackCtx;
     
    if (gl_debouncing[context->port])
    {
        gl_timer_exp[context->port] = true;
    }
}
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */

void ccg_sensor_debounce_task(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
    uint8_t port = ptrPdStackcontext->port;

    if (gl_timer_exp[port])
    {
#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))
        uint8_t state = Cy_SysLib_EnterCriticalSection();
#endif /* ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */
        operating_condition_t new_oc = (operating_condition_t)ccg_get_sys_oc(ptrPdStackcontext, NULL);
        gl_timer_exp[port] = false;

        if(gl_debounce_oc[port] == new_oc)
        {
            gl_sensor_debounce_count[port]--;
            if(gl_sensor_debounce_count[port] == 0u)
            {
                if (SYSTEM_OC_4 == gl_debounce_oc[port])
                {
                    psrc_disable(ptrPdStackcontext, NULL);
                    fault_handler_clear_counts (ptrPdStackcontext);
                }
                if (ccg_power_throttle_set_oc(ptrPdStackcontext, gl_debounce_oc[port], ccg_throttle_cb) != CY_PDSTACK_STAT_SUCCESS)
                {
                    gl_sensor_debounce_count[port]++;
                }
            }

            if(gl_sensor_debounce_count[port] != 0u)
            {
                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackcontext->ptrTimerContext, ptrPdStackcontext,  GET_APP_TIMER_ID(ptrPdStackcontext,THROTTLE_TIMER_ID), THROTTLE_DEBOUNCE_PERIOD,
                                                ccg_debounce_cb);
            }
        }
        else
        {
            /* Stop the debounce */
            gl_debouncing[port] = false;
        }

#if ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE))
        Cy_SysLib_ExitCriticalSection(state);
#endif /* ((VIN_OVP_ENABLE) || (VIN_UVP_ENABLE)) */
    }
}

/* This function is called from ccg_app_task() */
void ccg_sensor_check(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
    uint8_t system_oc;
    uint8_t new_oc;
    uint8_t port = ptrPdStackcontext->port;

    if(!gl_debouncing[port])
    {
        new_oc = ccg_get_sys_oc(ptrPdStackcontext, NULL);
        system_oc = (uint8_t)ccg_power_throttle_get_oc(ptrPdStackcontext);
        if(system_oc != new_oc)
        {
            gl_sensor_debounce_count[port] = THROTTLE_DEBOUNCE_VALUE;
            gl_debouncing[port] = true;
            gl_debounce_oc[port] = (operating_condition_t)new_oc;
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackcontext->ptrTimerContext, ptrPdStackcontext, GET_APP_TIMER_ID(ptrPdStackcontext,THROTTLE_TIMER_ID), THROTTLE_DEBOUNCE_PERIOD,
                                            ccg_debounce_cb);
        }
    }
}

/* Function to initialize the global variables used in temperature based power throttling */
void ccg_sensor_init(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
    bool temp_throttling_cfg_en = false;
    uint8_t i;
    uint8_t port = ptrPdStackcontext->port;
    sensor_data_t *sensor_cfg = pd_get_ptr_auto_cfg_tbl(ptrPdStackcontext->ptrUsbPdContext)->sensor_data;

#if (TEMPERATURE_SENSOR_COUNT > 1)
    for(i = 0; i < TEMPERATURE_SENSOR_COUNT; i++)
#else
    i = 0;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1) */
    {
        if((sensor_cfg[i].sensor_ctrl & THROTTLE_TEMP_SENSOR_EN) != 0u)
        {
            temp_throttling_cfg_en = true;
#if (TEMPERATURE_SENSOR_COUNT > 1)
            break;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1) */
        }
    }
    
    if((pd_get_ptr_auto_cfg_tbl(ptrPdStackcontext->ptrUsbPdContext)->vin_throttling_ctrl == 0u) && (temp_throttling_cfg_en == false))
    {
        /*
         * If temperature based power throttling and VIN based power throttling 
         * are disabled in the config table, then the start up OC
         * configuration and global variable initializations should not be done.
         */
        return;
    }
    
    uint8_t start_up_oc = (uint8_t)SYSTEM_OC_1; 
#if CCG_TEMP_BASED_VOLTAGE_THROTTLING    

#if (TEMPERATURE_SENSOR_COUNT > 1)
    for(i = 0; i < TEMPERATURE_SENSOR_COUNT; i++)
#else
    i = 0;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1) */
    {
        gl_sensor_oc[port][i] = SYSTEM_OC_1;
        gl_sensor_temp[port][i] = 0;
        gl_sensor_thresh[port][i] = 0;
    }
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */

#if CCG_VIN_BASED_VOLTAGE_THROTTLING
        gl_battery_oc[port] = (operating_condition_t)start_up_oc;
        gl_thres_volt[port] = 0xFF;
#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING*/

    start_up_oc = ccg_get_sys_oc(ptrPdStackcontext, NULL);
    (void)ccg_power_throttle_set_oc(ptrPdStackcontext, (operating_condition_t)start_up_oc, ccg_throttle_cb);
    CY_UNUSED_PARAMETER(port);
}

bool ccg_sensor_is_idle(cy_stc_pdstack_context_t *ptrPdStackcontext)
{
    uint8_t i;
    (void)ptrPdStackcontext;

#if (NO_OF_TYPEC_PORTS > 1)
    for (i = TYPEC_PORT_0_IDX; i < NO_OF_TYPEC_PORTS; i++)
#else
    i = TYPEC_PORT_0_IDX;
#endif /* (NO_OF_TYPEC_PORTS > 1) */
    {
        if(gl_timer_exp[i])
        {
            return false;
        }
    }
    return true;
}
#endif /* (CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_HPI_AUTO_CMD_ENABLE) */

#if CCG_HPI_AUTO_CMD_ENABLE
cy_en_pdstack_status_t ccg_sensor_temp_ec(cy_stc_pdstack_context_t *ptrPdStackcontext, uint8_t *buffer)
{
    uint8_t port = ptrPdStackcontext->port;
    (void) port;
    cy_en_pdstack_status_t status = CY_PDSTACK_STAT_FAILURE;
#if CCG_TEMP_BASED_VOLTAGE_THROTTLING
    if((ccg_power_throttle_get_feature_mask(ptrPdStackcontext) & CCG_ENABLE_TEMP_BASED_THROTTLING) != 0u)
    {
        sensor_data_t *sensor_cfg = &(pd_get_ptr_auto_cfg_tbl(ptrPdStackcontext->ptrUsbPdContext)->sensor_data[0]);
        uint8_t idx, sensor_information[TEMPERATURE_SENSOR_COUNT];
       CY_PDUTILS_MEM_SET(sensor_information, 0xFF, sizeof(sensor_information));
#if (TEMPERATURE_SENSOR_COUNT > 1u)
        for(idx = 0; idx < TEMPERATURE_SENSOR_COUNT; idx++)
#else
        idx = 0;
#endif /* (TEMPERATURE_SENSOR_COUNT > 1u) */
        {
            if((sensor_cfg[idx].sensor_ctrl & THROTTLE_TEMP_SENSOR_EN) != 0u)
            {
                sensor_information[idx] = gl_sensor_temp[port][idx];
            }
            else
            {
                /* If sensor has not been enabled, we assign the value as 0xFF */
                sensor_information[idx] = 0xFF; 
            }
        }
      
        /* We will copy the temperature information of the thermistor/sensor
         * to the buffer address passed to the function by the EC. 
         */
        CY_PDUTILS_MEM_COPY(buffer, sensor_information, TEMPERATURE_SENSOR_COUNT * sizeof(uint8_t));
        status = CY_PDSTACK_STAT_SUCCESS;
    }
#else
    CY_UNUSED_PARAMETER(buffer);
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */

    return status;
}
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || VIN_OVP_ENABLE || VIN_UVP_ENABLE || CCG_HPI_AUTO_CMD_ENABLE) */
