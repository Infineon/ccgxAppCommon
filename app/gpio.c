/***************************************************************************//**
* \file gpio.c
* \version 1.1.0 
*
* This is GPIO and IO mapping control functions source file
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include "gpio.h"
#include "srom.h"
#include "cy_usbpd_regs.h"

#define GPIO_PRT_PC_PORT_VTRIP_SEL                          ((uint32_t)1u << 24) /* <24:24> R:RW:0: */ /* <24:24> R:RW:0: */

#if (!(SROM_CODE_SYS_GPIO))

#ifdef PAG1S
/*
 * PAG1S has only one GPIO port. Since the code base assumes
 * an array, it is better to use it as an array here. Undefine
 * the macro so that the variable can be created locally.
 */
#undef GPIO
#define GPIO0   (PGPIO_REGS_T)GPIO_BASE_ADDR
#undef HSIOM
#define HSIOM0  (PHSIOM_REGS_T)HSIOM_BASE_ADDR
#endif /* PAG1S */
    
/* GPIO block address array */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
GPIO_PRT_Type* GPIO_V[] = /* PRQA S 6004 */
{
   GPIO_PRT0
#if (!defined(CY_DEVICE_PAG1S))
    ,
    GPIO_PRT1,
    GPIO_PRT2,
    GPIO_PRT3
#if (!defined(CY_DEVICE_CCG3PA) && !defined(CY_DEVICE_CCG3PA2) && !defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_CCG7S))
    ,
    GPIO_PRT4
#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C))
    ,
    GPIO_PRT5
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#endif /* (!defined(CCG3PA) && !defined(CCG3PA2) && !defined(CCG7D)) */
#endif /* (!defined(PAG1S)) */
};

/* HSIOM block address array */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
HSIOM_PRT_Type* HSIOM_V[] = /* PRQA S 6004 */
{
    HSIOM_PRT0
#if (!defined(CY_DEVICE_PAG1S))
    ,
    HSIOM_PRT1,
    HSIOM_PRT2,
    HSIOM_PRT3
#if (!defined(CY_DEVICE_CCG3PA) && !defined(CY_DEVICE_CCG3PA2) && !defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_CCG7S))
    ,
    HSIOM_PRT4
#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C))
    ,
    HSIOM_PRT5
#endif /* defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) */
#endif /* (!defined(CY_DEVICE_CCG3PA) && !defined(CY_DEVICE_CCG3PA2) && !defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_CCG7S)) */
#endif /* (!defined(CY_DEVICE_PAG1S)) */
};
#endif /* (!(SROM_CODE_SYS_GPIO)) */

#if (defined(CY_DEVICE_PAG1S))
        #define NO_OF_GPIO_PORTS        (1u)
#endif
#if ((defined(CY_DEVICE_CCG3PA)) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
        #define NO_OF_GPIO_PORTS        (4u)
#endif
#if ((defined(CY_DEVICE_CCG3)) || (defined(CY_DEVICE_CCG4)) || (defined(DMC)) || (defined(CY_DEVICE_CCG6DF)) || (defined(CY_DEVICE_CCG6SF)))
        #define NO_OF_GPIO_PORTS        (5u)
#endif
#if ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
        #define NO_OF_GPIO_PORTS        (6u)
#endif

/* Interrupt callbacks for each GPIO port. */
static gpio_intr_cb_t gl_gpio_intr_callbacks[NO_OF_GPIO_PORTS];

static void gpio_intr_handler (uint8_t port)
{
    uint32_t intr_stat = GPIO_V[port]->INTR;
    uint32_t intr_stat_flag;
    uint8_t port_pin;
    uint8_t  i;

    /* Clear the interrupt on the port. */
    GPIO_V[port]->INTR = intr_stat & 0xFFu;

    /* Disable interrupts while calling the callback functions. */
    /* QAC suppression 2986: This is a common code for all silicon devices.
     * Even though the GPIO_PORT0_INTR_NO is 0, this operation is retained 
     * as it may change in future for other silicon. */ 
    __NVIC_DisableIRQ((IRQn_Type)(GPIO_PORT0_INTR_NO + port)); /* PRQA S 2986 */

    /* Loop through each pin, and make the callback if interrupt is active. */
    for (i = 0; i < 8u; i++)
    {
        if ((intr_stat & ((uint32_t)1u << i)) != 0u)
        {
            if (gl_gpio_intr_callbacks[port] != NULL)
            {
                port_pin = ((port << 4) | i);
                intr_stat_flag = ((intr_stat >> (16u + i)) & 0x01u);
                gl_gpio_intr_callbacks[port] ((gpio_port_pin_t)port_pin, (bool)intr_stat_flag);
            }
        }
    }

    /* Re-enable the interrupt for this GPIO port, so that any new interrupts can be processed. */
    /* QAC suppression 2986: This is a common code for all silicon devices.
     * Even though the GPIO_PORT0_INTR_NO is 0, this operation is retained 
     * as it may change in future for other silicon. */ 
    __NVIC_EnableIRQ((IRQn_Type)(GPIO_PORT0_INTR_NO + port)); /* PRQA S 2986 */
}

static void port0_gpio_isr (void)
{
    gpio_intr_handler (0u);
}

#if (NO_OF_GPIO_PORTS > 1)
static void port1_gpio_isr (void)
{
    gpio_intr_handler (1u);
}
#endif /* (NO_OF_GPIO_PORTS > 1) */

#if (NO_OF_GPIO_PORTS > 2)
static void port2_gpio_isr (void)
{
    gpio_intr_handler (2u);
}
#endif /* (NO_OF_GPIO_PORTS > 2) */

#if (NO_OF_GPIO_PORTS > 3)
static void port3_gpio_isr (void)
{
    gpio_intr_handler (3u);
}
#endif /* (NO_OF_GPIO_PORTS > 3) */

#if (NO_OF_GPIO_PORTS > 4)
static void port4_gpio_isr (void)
{
    gpio_intr_handler (4u);
}
#endif /* (NO_OF_GPIO_PORTS > 4) */

#if (NO_OF_GPIO_PORTS > 5)
static void port5_gpio_isr (void)
{
    gpio_intr_handler (5u);
}
#endif /* (NO_OF_GPIO_PORTS > 5) */

ATTRIBUTES_SYS_GPIO void gpio_set_value(gpio_port_pin_t port_pin, bool value)
{
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;

    if (value)
    {
        CALL_MAP(GPIO_V)[port]->DR |= ((uint32_t)1u << pin);
    }
    else
    {
        CALL_MAP(GPIO_V)[port]->DR &= ~((uint32_t)1u << pin);
    }
}

ATTRIBUTES_SYS_GPIO bool gpio_read_value(gpio_port_pin_t port_pin)
{
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;

    return ((((CALL_MAP(GPIO_V))[port]->PS & ((uint32_t)1u << pin)) != 0u) ? true : false);
}

ATTRIBUTES_SYS_GPIO void gpio_set_drv_mode(gpio_port_pin_t port_pin, gpio_dm_t drv_mode)
{
    uint8_t pos;
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;
    uint32_t regVal;

    /* Multiply by three as pin uses 3 bits in the register. */
    pos = pin * GPIO_DM_FIELD_SIZE;

    /* Need to mask all other bits. Just update
     * the three bits of the current pin.
     */
    regVal = (CALL_MAP(GPIO_V)[port]->PC & ~(GPIO_DM_FIELD_MASK << pos));
    CALL_MAP(GPIO_V)[port]->PC = (regVal | ((uint32_t)drv_mode << pos));
}

ATTRIBUTES_SYS_GPIO void gpio_int_set_config(gpio_port_pin_t port_pin, uint8_t int_mode)
{
    uint8_t pos;
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;
    uint32_t regVal;

    pos = pin << 1;

    /* Make sure that the interrupt is cleared. */
    CALL_MAP(GPIO_V)[port]->INTR = ((uint32_t)1u << pin);

    /* Set the configuration. */
    regVal = ((CALL_MAP(GPIO_V))[port]->INTR_CFG & ~(GPIO_INT_FIELD_MASK << pos));
    CALL_MAP(GPIO_V)[port]->INTR_CFG = (regVal | ((uint32_t)int_mode << pos));
}

ATTRIBUTES_SYS_GPIO bool gpio_get_intr(gpio_port_pin_t port_pin)
{
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;

    /* Check if intr set */
    if((CALL_MAP(GPIO_V)[port]->INTR & ((uint32_t)1u << pin)) != 0u)
    {
        return true;
    }

    return false;
}

ATTRIBUTES_SYS_GPIO void gpio_clear_intr(gpio_port_pin_t port_pin)
{
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;

    /* Clear interrupt */
    (CALL_MAP(GPIO_V))[port]->INTR = ((uint32_t)1u << pin);
}

ATTRIBUTES_SYS_GPIO void hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode)
{
    uint8_t port = (uint8_t)port_pin >> 4;
    uint8_t pin = (uint8_t)port_pin & 0x0Fu;
    uint32_t regVal;

    regVal = CALL_MAP(HSIOM_V)[port]->PORT_SEL;

    regVal &= ~((uint32_t)HSIOM_PRT_PORT_SEL_IO0_SEL_Msk << (pin << HSIOM_FIELD_SHIFT));
    regVal |= ((uint32_t)hsiom_mode << ((uint32_t)pin << HSIOM_FIELD_SHIFT));

    CALL_MAP(HSIOM_V)[port]->PORT_SEL = regVal;
}

ATTRIBUTES_SYS_GPIO void gpio_hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
    gpio_dm_t drv_mode, bool value)
{
    /* Drive GPIO. */
    CALL_MAP(gpio_set_value)(port_pin, value);
    /* Set Drive Mode of GPIO. */
    CALL_MAP(gpio_set_drv_mode)(port_pin, drv_mode);
    /* Set HSIOM Configuration. */
    CALL_MAP(hsiom_set_config)(port_pin, hsiom_mode);
}

ATTRIBUTES_SYS_GPIO void gpio_set_lvttl_mode(uint8_t port)
{
    CALL_MAP(GPIO_V)[port]->PC |= GPIO_PRT_PC_PORT_VTRIP_SEL;
}

cy_en_pdstack_status_t gpio_register_intr_cb(uint8_t port, gpio_intr_cb_t intr_cb)
{
    cy_en_pdstack_status_t ret = CY_PDSTACK_STAT_BAD_PARAM;

    if (port < NO_OF_GPIO_PORTS)
    {
        if (intr_cb == NULL)
        {
            /* QAC suppression 2986: This is a common code for all silicon devices.
             * Even though the GPIO_PORT0_INTR_NO is 0, this operation is retained 
             * as it may change in future for other silicon. */ 
            __NVIC_DisableIRQ((IRQn_Type)(GPIO_PORT0_INTR_NO + port)); /* PRQA S 2986 */
            gl_gpio_intr_callbacks[port] = intr_cb;
        }
        else
        {
            gl_gpio_intr_callbacks[port] = intr_cb;
            switch (port)
            {
#if (NO_OF_GPIO_PORTS > 1)
                case 1u:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port1_gpio_isr);
                    break;
#endif /* (NO_OF_GPIO_PORTS > 1) */

#if (NO_OF_GPIO_PORTS > 2)
                case 2u:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port2_gpio_isr);
                    break;
#endif /* (NO_OF_GPIO_PORTS > 2) */

#if (NO_OF_GPIO_PORTS > 3)
                case 3u:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port3_gpio_isr);
                    break;
#endif /* (NO_OF_GPIO_PORTS > 3) */

#if (NO_OF_GPIO_PORTS > 4)
                case 4u:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port4_gpio_isr);
                    break;
#endif /* (NO_OF_GPIO_PORTS > 4) */

#if (NO_OF_GPIO_PORTS > 5)
                case 5u:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port5_gpio_isr);
                    break;
#endif /* (NO_OF_GPIO_PORTS > 5) */

                case 0u:
                default:
                    (void)CALL_MAP(Cy_SysInt_SetVector)((IRQn_Type)(GPIO_PORT0_INTR_NO + port), port0_gpio_isr);
                    break;
            }

            /* QAC suppression 2986: This is a common code for all silicon devices.
             * Even though the GPIO_PORT0_INTR_NO is 0, this operation is retained 
             * as it may change in future for other silicon. */ 
            __NVIC_EnableIRQ((IRQn_Type)(GPIO_PORT0_INTR_NO + port)); /* PRQA S 2986 */
        }
    }

    return (ret);
}

/* [] END OF FILE */
