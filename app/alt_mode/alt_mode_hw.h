/***************************************************************************//**
* \file alt_mode_hw.h
* \version 1.1.0 
*
* Hardware control for Alternate mode header file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
 
#ifndef ALT_MODE_HW_H_
#define ALT_MODE_HW_H_

/*****************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <config.h>
#include <app.h>

/*****************************************************************************
 * MACRO Definition
 *****************************************************************************/

#define NO_DATA                                 (0u)
/**< Zero data. */

#define HPD_ENABLE_CMD                          (0u)
/**< Enable HPD application command value. */

#define HPD_DISABLE_CMD                         (5u)
/**< Disable HPD application command value. */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef alt_mode_hw_t
  @brief Application HW type values which are used in alt_mode_hw_evt_t structure.
 */
typedef enum
{
    ALT_MODE_MUX = 1,                    /**< HW type - MUX. */
    ALT_MODE_HPD,                        /**< HW type - HPD transceiver/receiver. */

}alt_mode_hw_t;

#if 0
/**
  @ typedef mux_select_t
  @ brief Possible settings for the Type-C Data MUX.
  @ note This type should be extended to cover all possible modes for the MUX.
 */
typedef enum
{
    MUX_CONFIG_ISOLATE,                  /**< Isolate configuration. */
    MUX_CONFIG_SAFE,                     /**< USB Safe State (USB 2.0 lines remain active) */
    MUX_CONFIG_SS_ONLY,                  /**< USB SS configuration. */
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))    
    MUX_CONFIG_DP_2_LANE,                /**< Two lane DP configuration. */
    MUX_CONFIG_DP_4_LANE,                /**< Four lane DP configuration. */
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */
    MUX_CONFIG_USB4_CUSTOM,              /**< USB4 custom configuration. */
    MUX_CONFIG_RIDGE_CUSTOM,             /**< Alpine/Titan Ridge custom configuration. */
    MUX_CONFIG_INIT,                     /**< Enables MUX functionality. */
    MUX_CONFIG_DEINIT,                   /**< Disables MUX functionality. */

} mux_select_t;
#endif /* 0 */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @union alt_mode_hw_evt_t
  @brief Alt mode HW application event/command structure.
 */
typedef union
{

    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing alternate modes HW event/command . */
    struct ALT_MODE_HW_EVT
    {
        uint32_t evt_data    : 16;      /**< Current event/command data. */
        uint32_t hw_type     : 8;       /**< HW type event/command related to. */
        uint32_t data_role   : 8;       /**< Current data role. */
    }hw_evt;                            /**< Union containing the application HW event/command value. */

}alt_mode_hw_evt_t;

/**
  @typedef alt_mode_hw_cmd_cbk_t
  @brief Callback type used for notifications about ALT. MODE Hardware Commands.
 */
typedef void (*alt_mode_hw_cmd_cbk_t) (uint8_t port, uint32_t command);

/*****************************************************************************
 * Global Variable Declaration
 *****************************************************************************/

/** @cond DOXYGEN_HIDE */
extern bool         gl_app_mux_update_req[];
extern mux_select_t gl_app_mux_saved_state[];
extern uint32_t     gl_app_mux_saved_custom_data[];
/** @endcond */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function registers an ALT. MODE hardware callback function.
 *
 * @param port Port for which the callback is registered.
 * @param cbk Callback function for ALT. MODE hardware events.
 *
 * @return None.
 */
void alt_mode_hw_set_cbk(uint8_t port, alt_mode_hw_cmd_cbk_t cbk);

/**
 * @brief This function deinits all HW related to alt modes.
 *
 * @param port Port index the function is performed for.
 *
 * @return None.
 */
void alt_mode_hw_deinit(uint8_t port);

/**
 * @brief This function evaluates received application HW command.
 *
 * @param port Port index the function is performed for.
 * @param cmd_param Pointer to received application HW command data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contain unacceptable fields.
 */
bool eval_app_alt_hw_cmd(uint8_t port, uint8_t *cmd_param);

/**
 * @brief This function evaluates received HPD application command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received HPD application command data.
 *
 * @return true if HPD APP command passed successful, false if HPD APP command
 * is invalid or contain unacceptable fields.
 */
bool eval_hpd_cmd(uint8_t port, uint32_t cmd);

/**
 * @brief This function evaluates received MUX application command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received MUX application command data.
 *
 * @return true if MUX APP command passed successful, false if MUX APP command
 * is invalid or contain unacceptable fields.
 */
bool eval_mux_cmd(uint8_t port, uint32_t cmd);

/**
 * @brief This function sex appropriate MUX configuration based on the function parameters.
 *
 * @param context Pointer to pdstack context.
 * @param cfg Required MUX configuration.
 * @param custom_data Additional data in case of custom AR MUX configuration.
 *
 * @return true if MUX set passed successful, false if MUX setting is invalid or
 * contain unacceptable fields.
 */
bool set_mux(cy_stc_pdstack_context_t* context, mux_select_t cfg, uint32_t custom_data);

#if ICL_ENABLE
/**
 * @brief Ignore changes to MUX settings
 *
 * @param port Port index the function is performed for.
 * @param ignore Ignore changes to any MUX state on this
 *
 * @return None
 */
void ignore_mux_changes(uint8_t port, bool ignore);
#endif /* ICL_ENABLE */

/**
 * @brief Get the current state of the MUX.
 * @param port PD port index.
 * @return Active MUX setting.
 */
mux_select_t get_mux_state(uint8_t port);

/**
 * @brief Check whether the ALT. MODE hardware block is idle.
 *
 * This function is part of the deep-sleep entry checks for the CCG device, and checks whether
 * there are any pending ALT. MODE hardware commands that require the device to be active.
 *
 * @param port Port on which the hardware status is to be checked.
 * @return true if the block is idle, false otherwise.
 */
bool alt_mode_hw_is_idle(uint8_t port);

/**
 * @brief Prepare the Alt. Mode hardware state for device deep-sleep.
 * @param port The port whose hardware state is to be configured.
 * @return None.
 */
void alt_mode_hw_sleep(uint8_t port);

/**
 * @brief Restore Alt. Mode hardware state after device resumes from deep-sleep.
 * @param port The port whose hardware state is to be restored.
 * @return None
 */
void alt_mode_hw_wakeup(uint8_t port);

/**
 * @brief Return HPD state based on HPD Queue events.
 * @param port DP port index
 * @return true if HPD is connected, false otherwise
 */
bool dp_snk_get_hpd_state(uint8_t port);

#endif /* ALT_MODE_HW_H_ */

/* [] END OF FILE */
