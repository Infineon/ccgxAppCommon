/******************************************************************************
* File Name:   cy_pdaltmode_dp_sid.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Display Port Alternate Mode.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_DP_SID_H
#define CY_PDALTMODE_DP_SID_H

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdaltmode_defines.h"
#include "cy_pdaltmode_mngr.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

#define CY_PDALTMODE_DP_SVID                         (0xFF01u)
/**< DisplayPort SVID. */

#define CY_PDALTMODE_DP_ALT_MODE_ID                  (0u)
/**< Unique ID assigned to DP alternate mode in the CCGx SDK. */

#define CY_PDALTMODE_DP_VDO_IDX                      (0u)
/**< Index of VDO buffer element which is used to handle DP VDO. */

#define CY_PDALTMODE_STATUS_UPDATE_VDO               (0x01u)
/**< DP Status Update command VDO used by DFP_U. */

#define CY_PDALTMODE_DFP_D_CONN                      (0x01u)
/**< DP Status Update command VDO when DP is DFP (DFP_D DP data role). */

#define CY_PDALTMODE_UFP_D_CONN                      (0x02u)
/**< UFP_D DP data role configuration. */

#define CY_PDALTMODE_DP_1_3_SIGNALING                (0x0001u)
/**< Standard DP signaling type value in the signaling field for Config VDO. */

#define CY_PDALTMODE_DP_CONFIG_SELECT                (2u)
/**< Value when DFP set configuration for UFP_U as UFP_D in the configuration
     select field for Config VDO. */

#define CY_PDALTMODE_DP_CONFIG_SELECT_DPSRC          (1u)
/**< Value when DFP set configuration for UFP_U as DFP_D in the configuration
     select field of Config VDO. */

#define CY_PDALTMODE_USB_CONFIG_SELECT               (0u)
/**< Value when DFP set configuration as USB in the configuration select field
     for Config VDO. */

#define CY_PDALTMODE_DP_USB_SS_CONFIG                (0u)
/**< Pin assignment mask for USB pin assignment in the Configure Pin Assignment
     field for Config VDO. */

#define CY_PDALTMODE_DP_DFP_D_CONFIG_C               (4u)
/**< Pin assignment mask for C pin assignment in the Configure Pin Assignment field for Config VDO. */

#define CY_PDALTMODE_DP_DFP_D_CONFIG_D               (8u)
/**< Pin assignment mask for D pin assignment in the Configure Pin Assignment field for Config VDO. */

#define CY_PDALTMODE_DP_DFP_D_CONFIG_E               (0x10u)
/**< Pin assignment mask for E pin assignment in the Configure Pin Assignment field for Config VDO. */

#define CY_PDALTMODE_DP_DFP_D_CONFIG_F               (0x20u)
/**< Pin assignment mask for F pin assignment in the Configure Pin Assignment field for Config VDO. */

#define CY_PDALTMODE_DP_INVALID_CFG                  (0xFFu)
/**< Internal DP denotation for invalid pin assignment. */

#define CY_PDALTMODE_HPD_LOW_IRQ_LOW                 (0x0u)
/**< HPD low and IRQ low value obtained from UFP Attention/Status Update VDO. */

#define CY_PDALTMODE_HPD_HIGH_IRQ_LOW                (0x1u)
/**< HPD high and IRQ low value obtained from UFP Attention/Status Update VDO. */

#define CY_PDALTMODE_HPD_LOW_IRQ_HIGH                (0x2u)
/**< HPD low and IRQ high value obtained from UFP Attention/Status Update VDO. */

#define CY_PDALTMODE_HPD_HIGH_IRQ_HIGH               (0x3u)
/**< HPD high and IRQ high value obtained from UFP Attention/Status Update VDO. */
    
#define CY_PDALTMODE_HPD_STATE_BIT_POS               (0x10u)
/**< HPD LVL Bit Position. */

#define CY_PDALTMODE_HPD_IRQ_BIT_POS                 (0x11u)
/**< HPD LVL Bit Position. */
    
#define CY_PDALTMODE_DP_QUEUE_STATE_SIZE             (2u)
/**< Size of one element (in bits) in the HPD queue. */

#define CY_PDALTMODE_DP_HPD_STATE_MASK               (0x0003u)
/**< Mask to extract the first element from the HPD queue. */

#define CY_PDALTMODE_DP_QUEUE_EMPTY_INDEX            (0x0u)
/**< Flag to indicate an empty HPD queue. */

#define CY_PDALTMODE_DP_QUEUE_FULL_INDEX             (7u)
/**< Flag to indicate a full HPD queue. */

#define CY_PDALTMODE_DP_UFP_MAX_QUEUE_SIZE           (4u)
/**< Maximum number of entries in the HPD queue. */
    
#define CY_PDALTMODE_DP_MAX_IRQ_SIZE                (2u)
/**< Maximum number of IRQ entries in the HPD queue. */    

#define GET_HPD_IRQ_STAT(status)        (((status) >> 7u)& 0x3 )
/**< Macros to get HPD state from Attention/Status Update VDO. */

#define CY_PDALTMODE_DP_SINK_CTRL_CMD           (3u)
/**< DP Sink Control APP command value. */

#define CY_PDALTMODE_DP_MUX_CTRL_CMD            (1u)
/**< DP MUX Control APP command value. */

#define CY_PDALTMODE_DP_APP_CFG_CMD             (2u)
/**< DP MUX Configure APP command value. */

#define CY_PDALTMODE_DP_APP_VCONN_SWAP_CFG_CMD  (4u)
/**< DP UFP Vconn Swap command value. */

#define CY_PDALTMODE_DP_APP_CFG_USB_IDX            (6u)
/**< Bit index of USB configuration in APP DP MUX Configure command data. */

#define CY_PDALTMODE_DP_APP_CFG_CMD_MAX_NUMB       (6u)
/**< Maximum value of USB configuration in APP DP MUX Configure command data. */

#define CY_PDALTMODE_DP_ALLOWED_MUX_CONFIG_EVT     (1u)
/**< DP allowed MUX Configuration APP event value. */

#define CY_PDALTMODE_DP_STATUS_UPDATE_EVT          (2u)
/**< DP Status Update APP event value. */

#define CY_PDALTMODE_DP_CFG_CMD_ACK_MASK           (0x200)
/**< Acked field mask for EC DP MUX Configure command data. */

/** \} group_pdaltmode_macros */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/


/**
* \addtogroup group_pdaltmode_enums
* \{
*/

/** \} group_pdaltmode_enums */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
* \ addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_DP_RegModes
****************************************************************************//**
*
* This function analyses Discovery information to find out
* if further DP alternative mode processing is allowed.
*
* \param context
* Pointer to the alt mode context.
*
* \param reg_info
* Pointer to structure which holds alt mode register info.
*
* \return
* Pointer to DP alternative mode command structure if analysis passed
 * successful. In case of failure, function returns NULL pointer.
*
*******************************************************************************/
cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_DP_RegModes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info);

#if DP_GPIO_CONFIG_SELECT

/*******************************************************************************
* Function Name: Cy_PdAltMode_DP_SinkSetPinConfig
****************************************************************************//**
*
* Store the DP Pin configuration based on GPIO status.
*
* \param dp_config
* DP Pin configuration
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_DP_SinkSetPinConfig(uint8_t dp_config);


/*******************************************************************************
* Function Name: Cy_PdAltMode_DP_RegModes
****************************************************************************//**
*
* Return the DP Pin configuration based on GPIO status.
*
* \return
* DP Pin configuration
*
*******************************************************************************/
uint8_t Cy_PdAltMode_DP_SinkGetPinConfig(void);

#endif /* DP_GPIO_CONFIG_SELECT */

/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_DP_SID_H */

/* [] END OF FILE */
