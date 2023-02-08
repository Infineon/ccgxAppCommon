/******************************************************************************
* File Name:   lpm_gpio_app.c
* \version 2.0
*
* Description: GPIO based LPM interface source file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stddef.h>
#include "config.h"
#include "cy_device_headers.h"
#include "lpm_gpio_app.h"
#include "srom.h"

#if CCG_LPM_GPIO_ENABLE
/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
/**
 * @brief Flag to indicate Sleep Entry. 
 */
static bool gl_sleep_mode = false;

/**
 * @brief Chip Enable pin. 
 */
static gpio_port_pin_t gl_lpm_gpio;

/*******************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************/
/**
 * @brief LPM GPIO WakeUp Interrupt Handler.
 * @return None.
 */
static void lpm_gpio_Int_handler(void);

/*******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 * @brief This is Init Fucntion for LPM over GPIO.
 * @param port_pin Chip Enable Pin.
 * @return None.
 * @Note This function has to be called from main.c to make LPM over GPIO Initialise
 */
void lpm_gpio_init(gpio_port_pin_t port_pin)
{   
    gl_sleep_mode = false;
    
    gl_lpm_gpio = port_pin;
    
    CALL_MAP(gpio_set_drv_mode)(gl_lpm_gpio, GPIO_DM_HIZ_DIGITAL);

    /* For all we know the interrupt is active. */
    NVIC_DisableIRQ((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u));

    /* Set the ISR to point to the LPM_GPIO_WAKEUP_IRQ Interrupt. */
    Cy_SysInt_SetVector((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u), &lpm_gpio_Int_handler);

    /* Enable it. */
    CALL_MAP(gpio_clear_intr)(gl_lpm_gpio);
    NVIC_EnableIRQ((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u));

    /* Set the Interrupt Mode for the GPIO */
    CALL_MAP(gpio_int_set_config)(gl_lpm_gpio, GPIO_INTR_FALLING);

    if(CALL_MAP(gpio_read_value)(gl_lpm_gpio) == false)
    {
        /* The GPIO is already low....So go to sleep */
        gl_sleep_mode = true;
    }
}

/**
 * @brief LPM GPIO Interrupt Handler.
 * @return None.
 */
static void lpm_gpio_Int_handler(void)
{
    CALL_MAP(gpio_clear_intr)(gl_lpm_gpio);

    /* 
     * We will come here only as entry into LPM.
     * The rising edge interrupt shall be cleared by
     * firmware before coming here.
     */
    gl_sleep_mode = true;
}

/**
 * @brief LPM GPIO Rising Interrupt Handler.
 * @return None.
 */
static void lpm_gpio_rising_Int_handler(void)
{
    CALL_MAP(gpio_clear_intr)(gl_lpm_gpio);

    /*
     * We will come here only as entry into LPM.
     * The rising edge interrupt shall be cleared by
     * firmware before coming here.
     */
    gl_sleep_mode = false;
}

/**
 * @brief To get the LPM Status.
 * @return Status of LPM.
 */
bool lpm_gpio_get_lpm_stat(void)
{
    return gl_sleep_mode;
}

/**
 * @brief To clear the LPM Status.
 * @param None
 * @return None
 */
void lpm_gpio_clr_lpm_stat(void)
{
    gl_sleep_mode = false;
}

/**
 * @brief To Set the Interrupt.
 * @param intr interrupt type
 * @return None
 */
void lpm_gpio_set_intr(gpio_intr_t intr)
{
    NVIC_DisableIRQ((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u));
    if(intr == GPIO_INTR_FALLING)
    {
        CALL_MAP(gpio_int_set_config)(gl_lpm_gpio, GPIO_INTR_FALLING);
        /* Set the ISR to point to the LPM_GPIO_WAKEUP_IRQ Interrupt. */
        Cy_SysInt_SetVector((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u), &lpm_gpio_Int_handler);
    }
    else
    {
        CALL_MAP(gpio_int_set_config)(gl_lpm_gpio, GPIO_INTR_RISING);
        /* Set the ISR to point to the LPM_GPIO_WAKEUP_IRQ Interrupt. */
        Cy_SysInt_SetVector((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u), &lpm_gpio_rising_Int_handler);
    }
    /* Enable only the LPM_GPIOUp Interrupt */
    CALL_MAP(gpio_clear_intr)(gl_lpm_gpio);
    NVIC_EnableIRQ((IRQn_Type)((gl_lpm_gpio & 0xF0u) >> 4u));
}

/**
 * @brief To Clear the Interrupt.
 * @return None
 */
void lpm_gpio_clr_intr(void)
{
    CALL_MAP(gpio_clear_intr)(gl_lpm_gpio);
}

/**
 * @brief To Read the LPM_GPIO.
 * @return gpio status
 */
bool lpm_gpio_read(void)
{
    return (CALL_MAP(gpio_read_value)(gl_lpm_gpio));
}

#endif /* CCG_LPM_GPIO_ENABLE */

/* [] END OF FILE */
