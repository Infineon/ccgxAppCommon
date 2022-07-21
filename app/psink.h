/***************************************************************************//**
* \file psink.h
* \version 1.1.0 
*
* Power Sink (Consumer) manager header file 
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

#ifndef _PSINK_H_
#define _PSINK_H_

#if CY_PD_SINK_ONLY
/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include "cy_pdstack_common.h"
/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function sets the expected VBus voltage when
 * CCG is functioning as a sink. The voltage level is used
 * to configure the Over-Voltage Protection on the device.
 *
 * @param context PD Stack context.
 * @param volt_mV Expected VBus voltage level in mV units.
 *
 * @return None
 */
void psnk_set_voltage (cy_stc_pdstack_context_t * context, uint16_t volt_mV);

/**
 * @brief This function notifies the application code about
 * the amount of current the system is allowed to take from
 * the VBus power supply. The application logic should configure its
 * load and battery charging circuits based on this value so that
 * the power source does not see any overload condition.
 *
 * @param context PD Stack context.
 * @param cur_10mA Maximum allowed current in 10mA units.
 *
 * @return None
 */
void psnk_set_current (cy_stc_pdstack_context_t * context, uint16_t cur_10mA);

/**
 * @brief Function to enable the power consumer path to that the system
 * can received power from the Type-C VBus. The expected voltage and maximum
 * allowed current would have been notified through the psnk_set_voltage()
 * and psnk_set_current() functions.
 *
 * @param context PD Stack context.
 * @return None
 */
void psnk_enable (cy_stc_pdstack_context_t * context);

/**
 * @brief Disable the VBus power sink path and discharge VBus supply down
 * to a safe level. This function is called by the PD stack at times when
 * the system is not allowed to draw power from the VBus supply. The application
 * can use this call to initiate a VBus discharge operation so that a subsequent
 * Type-C connection is sped up. The snk_discharge_off_handler callback
 * function should be called once VBus has been discharged down to vSafe0V.
 *
 * @param  context PD Stack context.
 * @param snk_discharge_off_handler Sink Discharge fet off callback pointer
 * @return None
 */
void psnk_disable (cy_stc_pdstack_context_t * context, cy_pdstack_sink_discharge_off_cbk_t snk_discharge_off_handler);

/**
 * @brief Disable the Sink FET.
 *
 * @param context PD Stack context.
 * @return None
 */
void sink_fet_off(cy_stc_pdstack_context_t * context);

/**
 * @brief Enable the Sink FET.
 *
 * @param context PD Stack context.
 * @return None
 */
void sink_fet_on(cy_stc_pdstack_context_t * context);

#if CCG_HPI_VBUS_C_CTRL_ENABLE
/**
 * @brief Sets the VBUS Consumer FET ON control.
 *
 * @param port PD port index.
 * @param enable enable/disable the HPI vbus cfet control.
 * @return None.
 */
void psnk_set_vbus_cfet_on_ctrl (uint8_t port, uint8_t enable);
#endif /* CCG_HPI_VBUS_C_CTRL_ENABLE */
#endif /* CY_PD_SINK_ONLY */
#endif /* _PSINK_H_ */
/* End of File */

/** \} group_ccgxAppCommon */
