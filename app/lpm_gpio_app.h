/***************************************************************************//**
* \file lpm_gpio_app.h
* \version 1.1.0 
*
* This is GPIO based LPM header file 
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

#ifndef LPM_GPIO_APP_H_ 
#define LPM_GPIO_APP_H_

#include "gpio.h"

/*****************************************************************************
 * Global Function Declarations
 *****************************************************************************/

/**
 * @brief This function initializes the LPM Functionality over GPIO.
 * @param port_pin Chip Enable Pin.
 * @return None.
 */
void lpm_gpio_init(gpio_port_pin_t port_pin);

/**
 * @brief To get the LPM Status.
 * @return Status of LPM.
 */
bool lpm_gpio_get_lpm_stat(void);

/**
 * @brief To clear the LPM Status.
 * @return None
 */
void lpm_gpio_clr_lpm_stat(void);

/**
 * @brief To Set the Interrupt.
 * @param intr interrupt type
 * @return None
 */
void lpm_gpio_set_intr(gpio_intr_t intr);

/**
 * @brief To Clear the Interrupt.
 * @return None
 */
void lpm_gpio_clr_intr(void);

/**
 * @brief To Read the LPM_GPIO.
 * @return gpio status
 */
bool lpm_gpio_read(void);
    
#endif /* LPM_GPIO_APP_H_ */

/** \} group_ccgxAppCommon */

/* [] END OF FILE */
