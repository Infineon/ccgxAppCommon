/******************************************************************************
* File Name:   gpio.h
* \version 2.0
*
* Description: GPIO and IO mapping control functions
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


#ifndef GPIO_H_
#define GPIO_H_

#include "stdint.h"
#include "stdbool.h"
#include "cy_pdstack_common.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
#define GPIO_DM_FIELD_SIZE      (3u)            /**< GPIO drive mode field size. */
#define GPIO_DM_FIELD_MASK      ((uint32_t)7u)            /**< GPIO drive mode field mask without offset. */
#define GPIO_INT_FIELD_MASK     ((uint32_t)3u)            /**< GPIO interrupt configuration field mask. */
#define HSIOM_FIELD_SHIFT       (2u)            /**< HSIOM configuration field size as left shift value. */

#define GPIO_PORT0_INTR_NO      (0u)            /**< Interrupt vector number for P0.x pins. */
#define GPIO_PORT1_INTR_NO      (1u)            /**< Interrupt vector number for P1.x pins. */
#define GPIO_PORT2_INTR_NO      (2u)            /**< Interrupt vector number for P2.x pins.
                                                     Only valid if P2.x pins are present. */
#define GPIO_PORT3_INTR_NO      (3u)            /**< Interrupt vector number for P3.x pins.
                                                     Only valid if P3.x pins are present. */
#define GPIO_PORT4_INTR_NO      (4u)            /**< Interrupt vector number for P4.x pins.
                                                     Only valid if P4.x pins are present. */
#define GPIO_PORT5_INTR_NO      (5u)            /**< Interrupt vector number for P5.x pins.
                                                     Only valid if P5.x pins are present. */
#define GPIO_PORT6_INTR_NO      (6u)            /**< Interrupt vector number for P6.x pins.
                                                     Only valid if P6.x pins are present. */
/** \} group_ccgxAppCommon_macros */
/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/
/** \addtogroup group_ccgxAppCommon_enums
* \{
*/

/**
 * @brief List of pins supported on CCGx devices.
 *
 * This enumeration lists the port and pin assignment for the pins available on
 * the CCGx USB-PD controllers. This is a superset of all pins supported across
 * all of the devices in the family, and includes pins such as CC1/CC2 which
 * have fixed functionality.
 *
 * Please refer to the respective device datasheets to identify the pins that
 * can be used as GPIOs.
 */
typedef enum
{
    /* PORT 0 */
    GPIO_PORT_0_PIN_0 = 0x00,           /**< P0.0: Port 0, Pin 0. */
    GPIO_PORT_0_PIN_1,                  /**< P0.1: Port 0, Pin 1. */
    GPIO_PORT_0_PIN_2,                  /**< P0.2: Port 0, Pin 2. */
    GPIO_PORT_0_PIN_3,                  /**< P0.3: Port 0, Pin 3. */
    GPIO_PORT_0_PIN_4,                  /**< P0.4: Port 0, Pin 4. */
    GPIO_PORT_0_PIN_5,                  /**< P0.5: Port 0, Pin 5. */
    GPIO_PORT_0_PIN_6,                  /**< P0.6: Port 0, Pin 6. */
    GPIO_PORT_0_PIN_7,                  /**< P0.7: Port 0, Pin 7. */

    /* PORT 1 */
    GPIO_PORT_1_PIN_0 = 0x10,           /**< P1.0: Port 1, Pin 0. */
    GPIO_PORT_1_PIN_1,                  /**< P1.1: Port 1, Pin 1. */
    GPIO_PORT_1_PIN_2,                  /**< P1.2: Port 1, Pin 2. */
    GPIO_PORT_1_PIN_3,                  /**< P1.3: Port 1, Pin 3. */
    GPIO_PORT_1_PIN_4,                  /**< P1.4: Port 1, Pin 4. */
    GPIO_PORT_1_PIN_5,                  /**< P1.5: Port 1, Pin 5. */
    GPIO_PORT_1_PIN_6,                  /**< P1.6: Port 1, Pin 6. */
    GPIO_PORT_1_PIN_7,                  /**< P1.7: Port 1, Pin 7. */

    /* PORT 2 */
    GPIO_PORT_2_PIN_0 = 0x20,           /**< P2.0: Port 2, Pin 0. */
    GPIO_PORT_2_PIN_1,                  /**< P2.1: Port 2, Pin 1. */
    GPIO_PORT_2_PIN_2,                  /**< P2.2: Port 2, Pin 2. */
    GPIO_PORT_2_PIN_3,                  /**< P2.3: Port 2, Pin 3. */
    GPIO_PORT_2_PIN_4,                  /**< P2.4: Port 2, Pin 4. */
    GPIO_PORT_2_PIN_5,                  /**< P2.5: Port 2, Pin 5. */
    GPIO_PORT_2_PIN_6,                  /**< P2.6: Port 2, Pin 6. */
    GPIO_PORT_2_PIN_7,                  /**< P2.7: Port 2, Pin 7. */

    /* PORT 3 */
    GPIO_PORT_3_PIN_0 = 0x30,           /**< P3.0: Port 3, Pin 0. */
    GPIO_PORT_3_PIN_1,                  /**< P3.1: Port 3, Pin 1. */
    GPIO_PORT_3_PIN_2,                  /**< P3.2: Port 3, Pin 2. */
    GPIO_PORT_3_PIN_3,                  /**< P3.3: Port 3, Pin 3. */
    GPIO_PORT_3_PIN_4,                  /**< P3.4: Port 3, Pin 4. */
    GPIO_PORT_3_PIN_5,                  /**< P3.5: Port 3, Pin 5. */
    GPIO_PORT_3_PIN_6,                  /**< P3.6: Port 3, Pin 6. */
    GPIO_PORT_3_PIN_7,                  /**< P3.7: Port 3, Pin 7. */

    /* PORT 4 */
    GPIO_PORT_4_PIN_0 = 0x40,           /**< P4.0: Port 4, Pin 0. */
    GPIO_PORT_4_PIN_1,                  /**< P4.1: Port 4, Pin 1. */
    GPIO_PORT_4_PIN_2,                  /**< P4.2: Port 4, Pin 2. */
    GPIO_PORT_4_PIN_3,                  /**< P4.3: Port 4, Pin 3. */
    GPIO_PORT_4_PIN_4,                  /**< P4.4: Port 4, Pin 4. */
    GPIO_PORT_4_PIN_5,                  /**< P4.5: Port 4, Pin 5. */
    GPIO_PORT_4_PIN_6,                  /**< P4.6: Port 4, Pin 6. */
    GPIO_PORT_4_PIN_7,                  /**< P4.7: Port 4, Pin 7. */

    /* PORT 5 */
    GPIO_PORT_5_PIN_0 = 0x50,           /**< P5.0: Port 5, Pin 0. */
    GPIO_PORT_5_PIN_1,                  /**< P5.1: Port 5, Pin 1. */
    GPIO_PORT_5_PIN_2,                  /**< P5.2: Port 5, Pin 2. */
    GPIO_PORT_5_PIN_3,                  /**< P5.3: Port 5, Pin 3. */
    GPIO_PORT_5_PIN_4,                  /**< P5.4: Port 5, Pin 4. */
    GPIO_PORT_5_PIN_5,                  /**< P5.5: Port 5, Pin 5. */
    GPIO_PORT_5_PIN_6,                  /**< P5.6: Port 5, Pin 6. */
    GPIO_PORT_5_PIN_7,                  /**< P5.7: Port 5, Pin 7. */

    /* PORT 6 */
    GPIO_PORT_6_PIN_0 = 0x60,           /**< P6.0: Port 6, Pin 0. */
    GPIO_PORT_6_PIN_1                   /**< P6.1: Port 6, Pin 1. */
} gpio_port_pin_t;

/**
 * @brief Various GPIO drive modes supported by the CCGx devices.
 *
 * This enumeration lists the various drive modes supported by CCGx IOs which
 * are configured as GPIOs. Please refer to hsiom_mode_t for the IO mapping
 * settings available for each CCGx IO.
 *
 * @see hsiom_mode_t
 */
typedef enum gpio_dm_t
{
    GPIO_DM_HIZ_ANALOG = 0,             /**< Output buffer off (HiZ), input buffer off */
    GPIO_DM_HIZ_DIGITAL,                /**< Output buffer off (HiZ), input buffer on */
    GPIO_DM_RES_UP,                     /**< Resistive pull-up */
    GPIO_DM_RES_DWN,                    /**< Resistive pull-down */
    GPIO_DM_OD_LOW,                     /**< Drive low and HZI for high */
    GPIO_DM_OD_HIGH,                    /**< Drive high and HZI for low */
    GPIO_DM_STRONG,                     /**< Strong low and high drive */
    GPIO_DM_RES_UPDOWN                  /**< Resistive pull-up and pull-down */
} gpio_dm_t;

/**
   @brief Various GPIO interrupt modes supported by the device.
 */
typedef enum gpio_intr_t
{
    GPIO_INTR_DISABLE = 0,              /**< GPIO interrupt disabled. */
    GPIO_INTR_RISING,                   /**< Interrupt on rising edge of input. */
    GPIO_INTR_FALLING,                  /**< Interrupt on falling edge of input. */
    GPIO_INTR_BOTH                      /**< Interrupt on both rising and falling edges. */
} gpio_intr_t;

/**
 * @brief Various IO matrix configuration modes.
 *
 * Most of the IOs on CCGx devices & DMC (Dock Management Controller),
 * (except fixed function IOs like Vdd, GND, CCx),
 * can be configured to select one from many available options. This enumerated
 * type lists the various IO functions that can be selected for the flexible IOs.
 * Not all functionality can be configured to any selected IO. The selectable
 * functionality for each IO is fixed and cannot be altered. For example, in case
 * of CCG3 or DMC, SWD_CLK cannot be configured  to any other pin other than
 * P2.1. But P2.1 can be configured to function as  GPIO, SCB1_SPI_MOSI,
 * SCB1_I2C_SDA or SCB1_UART_RTS.
 *
 * Please refer to the respective device datasheets for more information about
 * these IO options.
 */
typedef enum hsiom_mode_t
{
    HSIOM_MODE_GPIO             =  0,   /**< Special functions disabled. Used as GPIO. */

#if (defined(CY_DEVICE_CCG3)) || (defined (DMC))
    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_IO         =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_TCPWM_CMP_MATCH  =  9,   /**< TCPWM compare match */
    HSIOM_MODE_SCB0_UART        =  9,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART        = 10,   /**< SCB2 UART IO. */
    HSIOM_MODE_SCB3_UART        = 10,   /**< SCB3 UART IO. */
    HSIOM_MODE_SRSS_EXT_CLK     = 10,   /**< SRSS block external clock. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 11,   /**< TCPWM overflow indicator */
    HSIOM_MODE_P0_DDFT0         = 12,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P0_DDFT1         = 12,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P0_ADC0_CMP_OUT  = 12,   /**< PDSS port0 ADC0 compare out. */
    HSIOM_MODE_P0_ADC1_CMP_OUT  = 12,   /**< PDSS port0 ADC1 compare out. */
    HSIOM_MODE_P0_SWAPT_OUT0    = 12,   /**< PDSS port0 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P0_SWAPT_OUT1    = 12,   /**< PDSS port0 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN      = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_HPD           = 12,   /**< PDSS port0 HPD IO. */
    HSIOM_MODE_P0_TX_DATA       = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_SWD              = 13,   /**< SWD IOs. */
    HSIOM_MODE_SCB0_SPI         = 13,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI         = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB3_SPI         = 14,   /**< SCB3 SPI IO. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C         = 15,   /**< SCB2 I2C IO. */
    HSIOM_MODE_SCB3_I2C         = 15,   /**< SCB3 I2C IO. */
    HSIOM_MODE_USB_VBUS_VALID   = 15    /**< USB VBUS_VALID IO. */

#elif (defined(CY_DEVICE_CCG4))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_IO         =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_SRSS_EXT_CLK     =  8,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_CMP_MATCH  =  9,   /**< TCPWM compare match */
    HSIOM_MODE_SCB0_UART        = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART        = 10,   /**< SCB2 UART IOs except RTS. */
    HSIOM_MODE_SCB3_UART        = 10,   /**< SCB3 UART IOs except RTS. */
    HSIOM_MODE_SCB2_UART_RTS    = 11,   /**< SCB2 UART RTS IO. */
    HSIOM_MODE_SCB3_UART_RTS    = 11,   /**< SCB3 UART RTS IO. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 11,   /**< TCPWM overflow indicator */
    HSIOM_MODE_SWD              = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_ADC0_CMP_OUT  = 12,   /**< PDSS port0 ADC0 compare out. */
    HSIOM_MODE_P0_ADC1_CMP_OUT  = 12,   /**< PDSS port0 ADC1 compare out. */
    HSIOM_MODE_P1_ADC0_CMP_OUT  = 12,   /**< PDSS port1 ADC0 compare out. */
    HSIOM_MODE_P0_TX_DATA       = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P1_TX_DATA       = 12,   /**< PDSS port1 TX data input for debug. */
    HSIOM_MODE_P1_TX_DATA_EN    = 12,   /**< PDSS port1 TX data enable for debug. */
    HSIOM_MODE_P0_SWAPT_OUT0    = 12,   /**< PDSS port0 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_SCB0_SPI         = 13,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB2_SPI         = 13,   /**< SCB2 SPI IOs except slave select line. */
    HSIOM_MODE_SCB3_SPI         = 13,   /**< SCB3 SPI IOs except slave select line. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI_SEL     = 14,   /**< SCB2 SPI slave select IO. */
    HSIOM_MODE_SCB3_SPI_SEL     = 14,   /**< SCB3 SPI slave select IO. */
    HSIOM_MODE_P0_DDFT1         = 14,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P1_ADC1_CMP_OUT  = 14,   /**< PDSS port1 ADC1 compare out. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C         = 15,   /**< SCB2 I2C IO. */
    HSIOM_MODE_SCB3_I2C         = 15,   /**< SCB3 I2C IO. */
    HSIOM_MODE_P0_HPD           = 15,   /**< PDSS port0 HPD IO. */
    HSIOM_MODE_P1_HPD           = 15,   /**< PDSS port1 HPD IO. */
    HSIOM_MODE_P0_SWAPT_OUT1    = 15,   /**< PDSS port0 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN      = 15,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P1_SWAPT_OUT0    = 15,   /**< PDSS port1 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P1_SWAPT_OUT1    = 15,   /**< PDSS port1 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P1_SWAPT_IN      = 15,   /**< PDSS port1 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_DDFT0         = 15,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P1_DDFT0         = 15,   /**< PDSS port1 DDFT0 IO. */
    HSIOM_MODE_P1_DDFT1         = 15    /**< PDSS port1 DDFT1 IO. */

#elif (defined(CY_DEVICE_CCG3PA))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_TR_INPUT   =  8,   /**< TCPWM trigger input */
    HSIOM_MODE_TCPWM_LINE       =  9,   /**< TCPWM line output. */
    HSIOM_MODE_SRSS_EXT_CLK     =  9,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_COMP       = 10,   /**< TCPWM line complement output. */
    HSIOM_MODE_SCB0_UART        = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 11,   /**< SCB1 UART IO. */
    HSIOM_MODE_TCPWM_COMP_MATCH = 11,   /**< TCPWM compare match output. */
    HSIOM_MODE_P0_GPIO_DDFT0    = 12,   /**< PDSS port0 GPIO DDFT0 IO. */
    HSIOM_MODE_P0_GPIO_DDFT1    = 12,   /**< PDSS port0 GPIO DDFT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN      = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_TX_DATA       = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_SWD              = 13,   /**< SWD IOs. */
    HSIOM_MODE_SCB0_SPI         = 14,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */

#elif (defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_IO         =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_SRSS_EXT_CLK     =  8,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_CMP_MATCH  =  9,   /**< TCPWM compare match */
    HSIOM_MODE_SCB0_UART        = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART        = 10,   /**< SCB2 UART IOs except RTS. */
    HSIOM_MODE_SCB3_UART        = 10,   /**< SCB3 UART IOs except RTS. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 11,   /**< TCPWM overflow indicator */
    HSIOM_MODE_SWD              = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_TX_DATA       = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P1_TX_DATA       = 12,   /**< PDSS port1 TX data input for debug. */
    HSIOM_MODE_P1_TX_DATA_EN    = 12,   /**< PDSS port1 TX data enable for debug. */
    HSIOM_MODE_P0_DDFT0         = 12,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P0_DDFT1         = 12,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P1_DDFT0         = 12,   /**< PDSS port1 DDFT0 IO. */
    HSIOM_MODE_P1_DDFT1         = 12,   /**< PDSS port1 DDFT1 IO. */
    HSIOM_MODE_P0_ADC_CMP_OUT   = 12,   /**< PDSS port0 ADC compare out. */
    HSIOM_MODE_P1_ADC_CMP_OUT   = 12,   /**< PDSS port1 ADC compare out. */
    HSIOM_MODE_P1_SWAPT_OUT0    = 12,   /**< PDSS port1 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P1_SWAPT_OUT1    = 12,   /**< PDSS port1 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P1_SWAPT_IN      = 12,   /**< PDSS port1 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_SWAPT_OUT0    = 12,   /**< PDSS port0 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P0_SWAPT_OUT1    = 12,   /**< PDSS port0 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN      = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_HPD           = 12,   /**< PDSS port0 HPD IO. */
    HSIOM_MODE_P1_HPD           = 12,   /**< PDSS port1 HPD IO. */
    HSIOM_MODE_P0_FAULT_OUT0    = 12,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P0_FAULT_OUT1    = 12,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_P1_FAULT_OUT0    = 12,   /**< PDSS port1 fault output 0. */
    HSIOM_MODE_P1_FAULT_OUT1    = 12,   /**< PDSS port1 fault output 1. */
    HSIOM_MODE_SCB0_SPI         = 13,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI         = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB3_SPI         = 14,   /**< SCB3 SPI IO. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C         = 15,   /**< SCB2 I2C IO. */
    HSIOM_MODE_SCB3_I2C         = 15    /**< SCB3 I2C IO. */

#elif (defined(CY_DEVICE_CCG3PA2))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_TR_INPUT   =  8,   /**< TCPWM trigger input */
    HSIOM_MODE_TCPWM_LINE       =  9,   /**< TCPWM line output. */
    HSIOM_MODE_SRSS_EXT_CLK     =  9,   /**< SRSS block external clock. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 10,   /**< TCPWM overflow indicator */
    HSIOM_MODE_TCPWM_CMP_MATCH  = 10,   /**< TCPWM compare match */
    HSIOM_MODE_SCB0_UART        = 11,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 11,   /**< SCB1 UART IO. */
    HSIOM_MODE_P0_ADC0_CMP_OUT  = 12,   /**< PDSS port0 ADC0 compare out. */
    HSIOM_MODE_P0_ADC1_CMP_OUT  = 12,   /**< PDSS port0 ADC1 compare out. */
    HSIOM_MODE_P0_TX_DATA       = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P0_SWAPT_IN      = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_SWAPT_OUT0    = 12,   /**< PDSS port0 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P0_SWAPT_OUT1    = 12,   /**< PDSS port0 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P0_GPIO_DDFT0    = 12,   /**< PDSS port0 GPIO DDFT0 IO. */
    HSIOM_MODE_P0_GPIO_DDFT1    = 12,   /**< PDSS port0 GPIO DDFT1 IO. */
    HSIOM_MODE_P0_FAULT_OUT0    = 13,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P0_FAULT_OUT1    = 13,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_P0_HPD           = 13,   /**< PDSS port0 HPD IO. */
    HSIOM_MODE_P0_RX_FRS        = 13,   /**< PDSS port0 RX FRS GPIO. */
    HSIOM_MODE_SWD              = 13,   /**< SWD IOs. */
    HSIOM_MODE_SCB0_SPI         = 14,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_P0_AFC_TX0       = 14,   /**< PDSS port0 AFC TX data for debug. */
    HSIOM_MODE_P0_AFC_TX1       = 14,   /**< PDSS port0 AFC TX data for debug. */
    HSIOM_MODE_P0_AFC_TX_EN     = 14,   /**< PDSS port0 AFC TX data enable for debug. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */

#elif (defined(PAG1S))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_TR_INPUT   =  8,   /**< TCPWM trigger input */
    HSIOM_MODE_TCPWM_LINE       =  9,   /**< TCPWM line output. */
    HSIOM_MODE_SRSS_EXT_CLK     =  9,   /**< SRSS block external clock. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 10,   /**< TCPWM overflow indicator */
    HSIOM_MODE_TCPWM_CMP_MATCH  = 10,   /**< TCPWM compare match */
    HSIOM_MODE_PTDRV_IN         = 10,   /**< PTDRV PWM output. */
    HSIOM_MODE_EXT_NSN_OUT      = 11,   /**< External NSN out. */
    HSIOM_MODE_EXT_GDRV_IN      = 11,   /**< External gate drive in. */
    HSIOM_MODE_SWD              = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_AFC_TX0       = 12,   /**< PDSS port0 AFC TX data for debug. */
    HSIOM_MODE_P0_AFC_TX_EN     = 12,   /**< PDSS port0 AFC TX data enable for debug. */
    HSIOM_MODE_P0_GPIO_DDFT0    = 13,   /**< PDSS port0 GPIO DDFT0 IO. */
    HSIOM_MODE_P0_GPIO_DDFT1    = 13,   /**< PDSS port0 GPIO DDFT1 IO. */
    HSIOM_MODE_P0_TX_DATA       = 13,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 13,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P0_FAULT_OUT0    = 14,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P0_FAULT_OUT1    = 14,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_P0_ZCD_OUT       = 14,   /**< PDSS port0 zero detect output. */
    HSIOM_MODE_P0_FFWD_PRE_SW_EN_0 = 14,   /**< PDSS port0 feedforward pre sw enable signal 0. */
    HSIOM_MODE_P0_FFWD_PRE_SW_EN_1 = 15,   /**< PDSS port0 feedforward pre sw enable signal 1. */
    HSIOM_MODE_P0_ADC0_CMP_OUT  = 15,   /**< PDSS port0 ADC0 compare out. */
    HSIOM_MODE_P0_PEAKDET_OUT   = 15,   /**< PDSS port0 peak detect output. */
    HSIOM_MODE_P0_EXT_PWM_OUT   = 15,   /**< PDSS port0 external PWM output. */

#elif (defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF))

    HSIOM_MODE_AMUXA            =  6,   /**< AMUXBUS A connection */
    HSIOM_MODE_AMUXB            =  7,   /**< AMUXBUS B connection */
    HSIOM_MODE_TCPWM_IO         =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_SRSS_EXT_CLK     =  8,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_CMP_MATCH  =  9,   /**< TCPWM compare match */
    HSIOM_MODE_SCB0_UART        = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART        = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART        = 10,   /**< SCB2 UART IOs except RTS. */
    HSIOM_MODE_SCB3_UART        = 10,   /**< SCB3 UART IOs except RTS. */
    HSIOM_MODE_SCB_TCPWM_OFLOW  = 11,   /**< TCPWM overflow indicator */
    HSIOM_MODE_SWD              = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_TX_DATA       = 13,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN    = 13,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P1_TX_DATA       = 13,   /**< PDSS port1 TX data input for debug. */
    HSIOM_MODE_P1_TX_DATA_EN    = 13,   /**< PDSS port1 TX data enable for debug. */
    HSIOM_MODE_P0_DDFT0         = 13,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P0_DDFT1         = 13,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P1_DDFT0         = 13,   /**< PDSS port1 DDFT0 IO. */
    HSIOM_MODE_P1_DDFT1         = 13,   /**< PDSS port1 DDFT1 IO. */
    HSIOM_MODE_P0_ADC_CMP_OUT   = 13,   /**< PDSS port0 ADC compare out. */
    HSIOM_MODE_P1_ADC_CMP_OUT   = 13,   /**< PDSS port1 ADC compare out. */
    HSIOM_MODE_P1_SWAPT_OUT0    = 13,   /**< PDSS port1 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P1_SWAPT_OUT1    = 13,   /**< PDSS port1 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P1_SWAPT_IN      = 13,   /**< PDSS port1 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_SWAPT_OUT0    = 13,   /**< PDSS port0 FRS SWAPT OUT0 IO. */
    HSIOM_MODE_P0_SWAPT_OUT1    = 13,   /**< PDSS port0 FRS SWAPT OUT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN      = 13,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_HPD           = 13,   /**< PDSS port0 HPD IO. */
    HSIOM_MODE_P1_HPD           = 13,   /**< PDSS port1 HPD IO. */
    HSIOM_MODE_P0_FAULT_OUT0    = 13,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P0_FAULT_OUT1    = 13,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_P1_FAULT_OUT0    = 13,   /**< PDSS port1 fault output 0. */
    HSIOM_MODE_P1_FAULT_OUT1    = 13,   /**< PDSS port1 fault output 1. */
    HSIOM_MODE_SCB0_SPI         = 14,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI         = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI         = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB3_SPI         = 14,   /**< SCB3 SPI IO. */
    HSIOM_MODE_SCB0_I2C         = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C         = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C         = 15,   /**< SCB2 I2C IO. */
    HSIOM_MODE_SCB3_I2C         = 15    /**< SCB3 I2C IO. */

#elif (defined(CY_DEVICE_CCG7D))

    HSIOM_MODE_AMUXA                =  6,   /**< AMUXBUS A connection. */
    HSIOM_MODE_AMUXB                =  7,   /**< AMUXBUS B connection. */
    HSIOM_MODE_TCPWM_IO             =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_SRSS_EXT_CLK         =  8,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_CMP_MATCH      =  9,   /**< TCPWM compare match. */
    HSIOM_MODE_P1_EXT_SET_BUCK      =  9,   /**< PDSS port1 external set buck. */
    HSIOM_MODE_P1_EXT_SET_BOOST     =  9,   /**< PDSS port1 external set boost. */
    HSIOM_MODE_P1_EXT_RST_SW1       =  9,   /**< PDSS port1 external reset. */
    HSIOM_MODE_P1_EXT_HSRCP         =  9,   /**< PDSS port1 external HS RCP. */
    HSIOM_MODE_P1_EXT_EXT_PWM_OUT   =  9,   /**< PDSS port1 external PWM Output. */
    HSIOM_MODE_P1_EXT_LSZCD         =  9,   /**< PDSS port1 external LS ZCD. */
    HSIOM_MODE_P1_EXT_SCOMP_OUT_EN  =  9,   /**< PDSS port1 external Slope compensation output enable. */
    HSIOM_MODE_P1_EXT_SCAP_RST      =  9,   /**< PDSS port1 external Slope capacitor reset. */
    HSIOM_MODE_SCB0_UART            = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART            = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART            = 10,   /**< SCB2 UART IO. */
    HSIOM_MODE_SCB3_UART            = 10,   /**< SCB3 UART IO. */
    HSIOM_MODE_SCB_TCPWM_OFLOW      = 11,   /**< TCPWM overflow indicator. */
    HSIOM_MODE_P0_EXT_SET_BUCK      = 11,   /**< PDSS port0 external set buck. */
    HSIOM_MODE_P0_EXT_SET_BOOST     = 11,   /**< PDSS port0 external set boost. */
    HSIOM_MODE_P0_EXT_RST_SW1       = 11,   /**< PDSS port0 external reset. */
    HSIOM_MODE_P0_EXT_HSRCP         = 11,   /**< PDSS port0 external HS RCP. */
    HSIOM_MODE_P0_EXT_EXT_PWM_OUT   = 11,   /**< PDSS port0 external PWM Output. */
    HSIOM_MODE_P0_EXT_LSZCD         = 11,   /**< PDSS port0 external LS ZCD. */
    HSIOM_MODE_P0_EXT_SCOMP_OUT_EN  = 11,   /**< PDSS port0 external Slope compensation output enable. */
    HSIOM_MODE_P0_EXT_SCAP_RST      = 11,   /**< PDSS port0 external Slope capacitor reset. */
    HSIOM_MODE_SWD                  = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_TX_DATA           = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN        = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P1_TX_DATA           = 12,   /**< PDSS port1 TX data input for debug. */
    HSIOM_MODE_P1_TX_DATA_EN        = 12,   /**< PDSS port1 TX data enable for debug. */
    HSIOM_MODE_P0_DDFT0             = 12,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P0_DDFT1             = 12,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P1_DDFT0             = 12,   /**< PDSS port1 DDFT0 IO. */
    HSIOM_MODE_P1_DDFT1             = 12,   /**< PDSS port1 DDFT1 IO. */
    HSIOM_MODE_P1_SWAPT_IN          = 12,   /**< PDSS port1 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_SWAPT_IN          = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_FAULT_OUT0        = 12,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P1_FAULT_OUT0        = 12,   /**< PDSS port1 fault output 0. */
    HSIOM_MODE_P0_HPD0              = 13,   /**< PDSS port0 HPD IO 0. */
    HSIOM_MODE_P0_HPD1              = 13,   /**< PDSS port0 HPD IO 1. */
    HSIOM_MODE_P1_HPD0              = 13,   /**< PDSS port1 HPD IO 0. */
    HSIOM_MODE_P1_HPD1              = 13,   /**< PDSS port1 HPD IO 1. */
    HSIOM_MODE_P0_ADC_CMP_OUT0      = 13,   /**< PDSS port0 ADC compare out 0. */
    HSIOM_MODE_P0_ADC_CMP_OUT1      = 13,   /**< PDSS port0 ADC compare out 1. */
    HSIOM_MODE_P1_ADC_CMP_OUT0      = 13,   /**< PDSS port1 ADC compare out 0. */
    HSIOM_MODE_P1_ADC_CMP_OUT1      = 13,   /**< PDSS port1 ADC compare out 1. */
    HSIOM_MODE_P0_EXT_LS_1          = 13,   /**< PDSS port0 Low Side 1. */
    HSIOM_MODE_P0_EXT_HS_1          = 13,   /**< PDSS port0 High Side 1. */
    HSIOM_MODE_P0_EXT_LS_2          = 13,   /**< PDSS port0 Low Side 2. */
    HSIOM_MODE_P0_EXT_HS_2          = 13,   /**< PDSS port0 High Side 2. */
    HSIOM_MODE_P1_EXT_LS_1          = 13,   /**< PDSS port1 Low Side 1. */
    HSIOM_MODE_P1_EXT_HS_1          = 13,   /**< PDSS port1 High Side 1. */
    HSIOM_MODE_P1_EXT_LS_2          = 13,   /**< PDSS port1 Low Side 2. */
    HSIOM_MODE_P1_EXT_HS_2          = 13,   /**< PDSS port1 High Side 2. */
    HSIOM_MODE_P0_AFC_TX_DATA_EN    = 13,   /**< PDSS port0 AFC TX enable. */
    HSIOM_MODE_P0_AFC_TX_DATA       = 13,   /**< PDSS port0 AFC TX data 0. */
    HSIOM_MODE_P0_FAULT_OUT1        = 14,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_P1_FAULT_OUT1        = 14,   /**< PDSS port1 fault output 1. */
    HSIOM_MODE_SCB0_SPI             = 14,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI             = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI             = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB3_SPI             = 14,   /**< SCB3 SPI IO. */
    HSIOM_MODE_SCB0_I2C             = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C             = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C             = 15,   /**< SCB2 I2C IO. */
    HSIOM_MODE_SCB3_I2C             = 15,   /**< SCB3 I2C IO. */
    HSIOM_MODE_P1_AFC_TX_DATA_EN    = 15,   /**< PDSS port1 AFC TX enable. */
    HSIOM_MODE_P1_AFC_TX_DATA       = 15    /**< PDSS port1 AFC TX data 0. */

#elif (defined(CY_DEVICE_CCG7S))

    HSIOM_MODE_AMUXA                =  6,   /**< AMUXBUS A connection. */
    HSIOM_MODE_AMUXB                =  7,   /**< AMUXBUS B connection. */
    HSIOM_MODE_TCPWM_IO             =  8,   /**< TCPWM IO line. */
    HSIOM_MODE_SRSS_EXT_CLK         =  8,   /**< SRSS block external clock. */
    HSIOM_MODE_TCPWM_CMP_MATCH      =  9,   /**< TCPWM compare match. */
    HSIOM_MODE_SCB0_UART            = 10,   /**< SCB0 UART IO. */
    HSIOM_MODE_SCB1_UART            = 10,   /**< SCB1 UART IO. */
    HSIOM_MODE_SCB2_UART            = 10,   /**< SCB2 UART IO. */
    HSIOM_MODE_SCB_TCPWM_OFLOW      = 11,   /**< TCPWM overflow indicator. */
    HSIOM_MODE_P0_EXT_SET_BUCK      = 11,   /**< PDSS port0 external set buck. */
    HSIOM_MODE_P0_EXT_SET_BOOST     = 11,   /**< PDSS port0 external set boost. */
    HSIOM_MODE_P0_EXT_RST_SW1       = 11,   /**< PDSS port0 external reset. */
    HSIOM_MODE_P0_EXT_HSRCP         = 11,   /**< PDSS port0 external HS RCP. */
    HSIOM_MODE_P0_EXT_EXT_PWM_OUT   = 11,   /**< PDSS port0 external PWM Output. */
    HSIOM_MODE_P0_EXT_LSZCD         = 11,   /**< PDSS port0 external LS ZCD. */
    HSIOM_MODE_P0_EXT_SCOMP_OUT_EN  = 11,   /**< PDSS port0 external Slope compensation output enable. */
    HSIOM_MODE_P0_EXT_SCAP_RST      = 11,   /**< PDSS port0 external Slope capacitor reset. */
    HSIOM_MODE_SWD                  = 12,   /**< SWD IOs. */
    HSIOM_MODE_P0_TX_DATA           = 12,   /**< PDSS port0 TX data input for debug. */
    HSIOM_MODE_P0_TX_DATA_EN        = 12,   /**< PDSS port0 TX data enable for debug. */
    HSIOM_MODE_P0_DDFT0             = 12,   /**< PDSS port0 DDFT0 IO. */
    HSIOM_MODE_P0_DDFT1             = 12,   /**< PDSS port0 DDFT1 IO. */
    HSIOM_MODE_P0_SWAPT_IN          = 12,   /**< PDSS port0 FRS SWAPT IN IO. */
    HSIOM_MODE_P0_FAULT_OUT0        = 12,   /**< PDSS port0 fault output 0. */
    HSIOM_MODE_P0_HPD0              = 13,   /**< PDSS port0 HPD IO 0. */
    HSIOM_MODE_P0_HPD1              = 13,   /**< PDSS port0 HPD IO 1. */
    HSIOM_MODE_P0_ADC_CMP_OUT0      = 13,   /**< PDSS port0 ADC compare out 0. */
    HSIOM_MODE_P0_ADC_CMP_OUT1      = 13,   /**< PDSS port0 ADC compare out 1. */
    HSIOM_MODE_P0_EXT_LS_1          = 13,   /**< PDSS port0 Low Side 1. */
    HSIOM_MODE_P0_EXT_HS_1          = 13,   /**< PDSS port0 High Side 1. */
    HSIOM_MODE_P0_EXT_LS_2          = 13,   /**< PDSS port0 Low Side 2. */
    HSIOM_MODE_P0_EXT_HS_2          = 13,   /**< PDSS port0 High Side 2. */
    HSIOM_MODE_P0_AFC_TX_DATA_EN    = 13,   /**< PDSS port0 AFC TX enable. */
    HSIOM_MODE_P0_AFC_TX_DATA       = 13,   /**< PDSS port0 AFC TX data 0. */
    HSIOM_MODE_P0_FAULT_OUT1        = 14,   /**< PDSS port0 fault output 1. */
    HSIOM_MODE_SCB0_SPI             = 14,   /**< SCB0 SPI IO. */
    HSIOM_MODE_SCB1_SPI             = 14,   /**< SCB1 SPI IO. */
    HSIOM_MODE_SCB2_SPI             = 14,   /**< SCB2 SPI IO. */
    HSIOM_MODE_SCB0_I2C             = 15,   /**< SCB0 I2C IO. */
    HSIOM_MODE_SCB1_I2C             = 15,   /**< SCB1 I2C IO. */
    HSIOM_MODE_SCB2_I2C             = 15,   /**< SCB2 I2C IO. */

#else

#error "Unsupported device."

#endif /* CCG */
} hsiom_mode_t;
/** \} group_ccgxAppCommon_enums */
/**
 * @brief Callback type for GPIO interrupt notification.
 * @param port_pin Parameter identifying the GPIO port and pin.
 * @param pin_state State of the GPIO when at the beginning of ISR.
 */
typedef void (*gpio_intr_cb_t)(gpio_port_pin_t port_pin, bool pin_state);

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
/**
 * @brief Sets the GPIO to the required state.
 *
 * This function updates the output state of a GPIO pin. The API does not do any
 * error check and will just update the output state. It is the caller's
 * responsibility to ensure that the HSIOM and drive mode settings for the pin
 * have been selected as required.
 *
 * Care should be taken to make sure that wrong indexing is not done.
 *
 * @param port_pin Pin to be updated.
 * @param value Value to drive on the pin: 0=LOW, 1=HIGH.
 *
 * @return None
 *
 * @see hsiom_set_config
 * @see gpio_set_drv_mode
 */
void gpio_set_value(gpio_port_pin_t port_pin, bool value);

/**
 * @brief Get the GPIO current state.
 *
 * This API retrieves the current state of a pin, assuming it has been configured
 * as a GPIO. It is the caller's responsibility to ensure that the HSIOM and
 * drive mode settings for the pin have been set correctly.
 *
 * The API does not do any error check and will just read the GPIO state
 * register. Care should be taken to make sure that wrong indexing is not done.
 *
 * @param port_pin Pin to be queried.
 *
 * @return Current state of the pin.
 *
 * @see hsiom_set_config
 * @see gpio_set_drv_mode
 */
bool gpio_read_value(gpio_port_pin_t port_pin);

/**
 * @brief Configure the GPIO with the desired drive mode.
 *
 * This API updates the drive mode for the selected CCG device IO. The API
 * does not do any error check and will just set the drive mode of GPIO. The
 * caller should ensure that the HSIOM setting for the pin has been selected
 * correctly.
 *
 * @param port_pin Pin to be configured.
 * @param drv_mode Desired drive mode.
 *
 * @return None
 */
void gpio_set_drv_mode(gpio_port_pin_t port_pin, gpio_dm_t drv_mode);

/**
 * @brief Configure the GPIO with the desired interrupt setting.
 *
 * This API configures the interrupt mode for the specified GPIO. It does not
 * do any error check and will just configure the GPIO interrupt setting. Care
 * should be taken to make sure that wrong indexing is not done.
 *
 * @param port_pin Pin to be configured.
 * @param int_mode Desired interrupt mode.
 *
 * @return None
 */
void gpio_int_set_config(gpio_port_pin_t port_pin, uint8_t int_mode);

/**
 * @brief Register a callback to be called when a GPIO interrupt associated with a specific port is triggered.
 * The gpio_int_set_config should be used to configure the desired interrupts for each of the relevant pins.
 *
 * @param port GPIO port on which interrupts are to be updated.
 * @param intr_cb Pointer to callback function to be called on interrupt.
 * @return CCG_STAT_SUCCESS on success, appropriate error code otherwise.
 */
cy_en_pdstack_status_t gpio_register_intr_cb(uint8_t port, gpio_intr_cb_t intr_cb);

/**
 * @brief Select the IO mode for a CCG device IO.
 *
 * This API configures the IO mode for a CCG device IO. It does not do any
 * error check and will just set the HSIOM configuration for the GPIO. Care
 * should be taken to make sure that wrong indexing is not done.
 *
 * @param port_pin Pin to be configured.
 * @param hsiom_mode Desired IO mode.
 *
 * @return None
 *
 * @see hsiom_mode_t
 */
void hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode);

/**
 * @brief Single function for complete configuration of a CCG IO.
 *
 * This is a single API which can be used to configure the IO mode, the drive
 * mode and the current value of a CCG device IO. No error checks are performed,
 * and the caller should ensure that the settings provided are valid for the
 * selected IO.
 *
 * @param port_pin Pin to be configured.
 * @param hsiom_mode Desired IO mode.
 * @param drv_mode Desired drive mode.
 * @param value Desired output state.
 *
 * @return None
 */
void gpio_hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
    gpio_dm_t drv_mode, bool value);

/**
 * @brief Set the input buffer voltage for a port to LVTTL.
 *
 * This API sets the input buffer voltage for a complete CCG device port to LVTTL
 * levels. This is required to be set for ports that include the IOs used for the
 * I2C based HPI interface.
 *
 * @param port IO port to be configured for LVTTL levels.
 *
 * @return None
 */
void gpio_set_lvttl_mode(uint8_t port);

/**
 * @brief Read the interrupt status on a specific GPIO.
 *
 * This function checks whether there are any active interrupts on the
 * specified GPIO.
 *
 * @param port_pin Pin to be queried.
 *
 * @return true if interrupt is active, false otherwise.
 */
bool gpio_get_intr(gpio_port_pin_t port_pin);

/**
 * @brief Clear interrupt status on the specified GPIO.
 *
 * This function clears any active interrupts on the specified GPIO.
 *
 * @param port_pin Pin to be updated.
 *
 * @return None
 */
void gpio_clear_intr(gpio_port_pin_t port_pin);

/** @cond DOXYGEN_HIDE */

/* Function pointer types used to access ROM-ed versions of SYSTEM module APIs.
 * These correspond to the various functions defined above.
 */
typedef void (*gpio_set_value_fptr)(gpio_port_pin_t port_pin, bool value);
typedef bool (*gpio_read_value_fptr)(gpio_port_pin_t port_pin);
typedef void (*gpio_set_drv_mode_fptr)(gpio_port_pin_t port_pin, gpio_dm_t drv_mode);
typedef void (*gpio_int_set_config_fptr)(gpio_port_pin_t port_pin, uint8_t int_mode);
typedef void (*hsiom_set_config_fptr)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode);
typedef void (*gpio_hsiom_set_config_fptr)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
    gpio_dm_t drv_mode, bool value);
typedef void (*gpio_set_lvttl_mode_fptr)(uint8_t port);
typedef bool (*gpio_get_intr_fptr)(gpio_port_pin_t port_pin);
typedef void (*gpio_clear_intr_fptr)(gpio_port_pin_t port_pin);

/** @endcond */
/** \} group_ccgxAppCommon_functions */
#endif /* GPIO_H_ */

/** \} group_ccgxAppCommon */

/*[]*/

