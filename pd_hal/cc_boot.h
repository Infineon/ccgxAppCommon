/******************************************************************************
* File Name:   cc_boot.h
* \version 2.0
*
* Description: Definitions for CCG CC bootloader
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef _CC_BOOT_H_
#define _CC_BOOT_H_
#include "config.h"
#include "cy_gpio.h"
#include "cy_pdstack_common.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
/* VBUS discharge time in ms */
#define VBUS_DISCHARGE_TIME             (300)
/** \} group_ccgxAppCommon_macros */
/*****************************************************************************
 * Enumerated Data Definition
 *****************************************************************************/
/** \addtogroup group_ccgxAppCommon_enums
* \{
*/

typedef enum PD_STATE {
    IDLE,
    CONNECTED,
    SEND_SRC_CAP,
    CONTRACT_ESTD,
    PS_RDY,
}pd_state_t;
/** \} group_ccgxAppCommon_enums */
/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
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

/**
 * @brief This function enables the 5V VDDD.
 * @param port Port ID
 * @return None
 */
void pd_set_vddd_5v(uint8_t port);

/**
 * @brief This function enables VCONN
 * @param port Port ID
 * @param channel CC1/CC2
*/
cy_en_usbpd_status_t Cy_PD_HAL_Vconn_Enable(uint8_t port, uint8_t channel);

/**
 * @brief This Functions retrieves the active CC Channel based on attach
 * 
 * @param port 
 * @return true 
 * @return false 
 */
bool Cy_PD_HAL_Get_Active_CC_Channel(uint8_t port);

#endif /* _CC_BOOT_H_ */
