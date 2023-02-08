/******************************************************************************
* File Name:   lpm_gpio_app.h
* \version 2.0
*
* Description: GPIO based LPM interface header file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon App Common Middleware
* \{
*/

#ifndef LPM_GPIO_APP_H_ 
#define LPM_GPIO_APP_H_

#include "gpio.h"

/*****************************************************************************
 * Global Function Declarations
 *****************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
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

/** \} group_ccgxAppCommon_functions */
#endif /* LPM_GPIO_APP_H_ */

/** \} group_ccgxAppCommon */

/* [] END OF FILE */
