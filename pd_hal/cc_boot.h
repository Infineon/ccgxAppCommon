/**
 * @file cc_boot.h
 *
 * @brief @{Definitions for CCG CC bootloader.@}
 */

/*
 *
 * Copyright(2014-2020), Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All rights reserved. 
 * 
 * This software, including source code, documentation and related materials 
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its 
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent 
 * protection (United States and foreign), United States copyright laws and 
 * international treaty provisions. Therefore, you may use this Software only 
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to 
 * copy, modify, and compile the Software source code solely for use in 
 * connection with Cypress's integrated circuit products. Any reproduction, 
 * modification, translation, compilation, or representation of this Software 
 * except as specified above is prohibited without the express written 
 * permission of Cypress. 
 * 
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or use
 * of the Software or any product or circuit described in the Software. Cypress
 * does not authorize its products for use in any products where a malfunction
 * or failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By 
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability. 
 *
 ******************************************************************************/

#ifndef _CC_BOOT_H_
#define _CC_BOOT_H_
#include "config.h"
#include "cy_gpio.h"
#include "cy_pdstack_common.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/* VBUS discharge time in ms */
#define VBUS_DISCHARGE_TIME             (300)
    
/*****************************************************************************
 * Enumerated Data Definition
 *****************************************************************************/

typedef enum PD_STATE {
    IDLE,
    CONNECTED,
    SEND_SRC_CAP,
    CONTRACT_ESTD,
#if CC_BOOT_NB_CALL_EN
    PS_RDY,
#endif /* CC_BOOT_NB_CALL_EN */    
}pd_state_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
#ifdef CCG_BOOT
/**
 * @brief This function initializes the PD phy registers.
 * @param None.
 * @return None
 */
void pdss_phy_init(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function initializes the initial regulation.
 * @param None.
 * @return None
 */
void pdss_phy_regulation_init(void);

/**
 * @brief This function disables PWM control.
 * @param None.
 * @return None
 */
void pd_pasc_disable(uint8_t port);
#endif

/**
 * @brief This function implements the Type C state machine
 * @param None
 * @return None
 */
void typec_state_machine(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function implements the PD state machine
 * @param None.
 * @return None
 */
void pd_state_machine(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function resets message ID counter.
 * @param None
 * @return None
 */
void pd_reset_protocol(uint8_t port);
/**
 * @brief This function sends a control message.
 * @param msg_type control message type.
 * @return None
 */
void pd_send_ctl_msg(uint8_t port, cy_en_pd_ctrl_msg_t msg_type);

/**
 * @brief This function sends a data message.
 * @param msg_type data message type.
 * @param dobj pointer to data object
 * @param count data objects count. It should be less than or equal to 7
 * @return None
 */
void pd_send_data_msg(uint8_t port, cy_en_pdstack_data_msg_t msg_type, cy_pd_pd_do_t* dobj,
        uint8_t count);

/*
 * @brief Turn on VBus discharge path.
 *
 * @param port Port on which to enable VBus discharge.
 * @return Void.
 */
void pd_internal_vbus_discharge_on(uint8_t port);

/*
 * @brief Turn off VBus discharge path.
 *
 * @param port Port on which to enable VBus discharge.
 * @return Void.
 */
void pd_internal_vbus_discharge_off(uint8_t port);

/**
 * @brief This function bumps up VBUS voltage to VSAFE_5V
 * @param port Port on which to enable VBus discharge.
 * @return None
 */
void vbus_set_vsafe(uint8_t port);

/**
 * @brief This function turns off VBUS.
 * @param None
 * @return None
 */
void turn_off_vbus(cy_stc_pdstack_context_t *context);

cy_en_usbpd_status_t Cy_PD_HAL_Vconn_Enable(uint8_t port, uint8_t channel);

bool Cy_PD_HAL_Get_Active_CC_Channel(uint8_t port);

#endif /* _CC_BOOT_H_ */
