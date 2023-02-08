/******************************************************************************
* File Name:   cy_pdaltmode_intel_ridge_internal.h
* \version 2.0
*
* Description: This header file defines the internal data structures and
*              function prototypes associated with the Intel Alpine/Titan Ridge
*              control interface.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_INTEL_RIDGE_INTERNAL_H
#define CY_PDALTMODE_INTEL_RIDGE_INTERNAL_H

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include "cy_pdaltmode_defines.h"

#include "cy_pdaltmode_hw.h"
#include "cy_usbpd_hpd.h"

#define RIDGE_DISCON_STATE_MASK                  (0x00)
/** @brief Disconnect state bit mask for Status register. */

#define RIDGE_DATA_CONN_PRESENT                  (0x01)
/** @brief Data present field in the status register. */

#if CY_PD_USB4_SUPPORT_ENABLE
#define RIDGE_DATA_CONN_MASK                     (0x3EC1D17D)
/** @brief Data connection bit mask for Status register. */    
#else    
#define RIDGE_DATA_CONN_MASK                     (0x1D179)
/** @brief Data connection bit mask for Status register. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

#define RIDGE_USB4_CONN_MASK                     (0x800011)
/** @brief USB4 connection bit mask for Status register. */

#define RIDGE_USB_STATE_MASK                     (0x31)
/** @brief USB only state bit mask for  Status register. */

#define RIDGE_SAFE_STATE_MASK                    (0x01)
/** @brief Safe state bit mask for  Status register. */

#define RIDGE_DP_4_LANE_MASK                     (0x111)
/** @brief 4 lane DP alt mode state bit mask for  Status register. */

#define RIDGE_DP_2_LANE_MASK                     (0x531)
/** @brief 2 lane DP alt mode state bit mask for Status register. */

#define DP_PIN_CONFIG_C                          (4u)
/** @brief Pin assignment mask for C pin assignment. */

#define DP_PIN_CONFIG_F                          (0x20u)
/** @brief Pin assignment mask for F pin assignment. */
#if ICL_ENABLE
#define RIDGE_TBT_MODE_MASK                      (0x10011)    
#else    
#define RIDGE_TBT_MODE_MASK                      (0x10001)
/** @brief TBT alt mode state bit mask for Titan register. */
#endif /* ICL_ENABLE */    

#define CY_PDALTMODE_RIDGE_DEBUG_MODE_MASK                    (0x11)
/** @brief Debug alt mode state bit mask for Status register. */

#define CY_PDALTMODE_TBT_HOST_CONN_MASK                       (0x01)
/** @brief TBT HOST Connected bit mask for Command register. */

#define CY_PDALTMODE_RIDGE_IRQ_ACK_MASK                          (0x2000)
/** @brief HPD IRQ ACK bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_HPD_IRQ_MASK                          (0x4000)
/** @brief HPD IRQ bit mask for Titan Ridge command/status register. */

#define CY_PDALTMODE_RIDGE_HPD_LVL_MASK                          (0x8000)
/** @brief HPD Level bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_USB_HOST_CONN_MASK                    (0x10)
/** @brief USB HOST Connected bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_DP_HOST_CONN_MASK                     (0x20)
/** @brief DP HOST Connected bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_ACT_CBL_MASK                          (0x400000)
/** @brief Active cable bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_RETIMER_MASK                          (0x4)
/** @brief Retimer bit mask for Titan Ridge command register. */

#define MR_IN_SAFE_STATE_MASK                    (0x4)
/** @brief Safe State bit mask for Maple Ridge command register. */

#define MR_DATA_RST_REQ_MASK                     (0x1000)
/** @brief Data Reset Request Connected bit mask for Maple Ridge command register. */

#define MR_PWR_RST_REQ_MASK                      (0x2000)
/** @brief Power Reset Request Connected bit mask for Maple Ridge command register. */

#define MR_READY_MASK                            (0x4000)
/** @brief Ridge is ready to be functional bit mask for Maple Ridge command register. */

#define RIDGE_STATUS_OCP_MASK                    (0x08)
/** @brief Over-Current status bit in the status register. */

#define EUDO_HOST_PRES_OFFSET                    (13u)
/** @brief Offset of Host present field in USB4 Enter VDO. */

/*****************************************************************************
 * Data Structure Definition
 *****************************************************************************/

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_SetCtrlChangeCbk
****************************************************************************//**
*
* Register a callback for notification of data control register changes.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param cb
* Pointer to callback function.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_SetCtrlChangeCbk(cy_stc_pdaltmode_context_t *ptrAltModeContext, ridge_ctrl_change_cb_t cb);

#endif /* CY_PDALTMODE_INTEL_RIDGE_INTERNAL_H */

/* [] END OF FILE */

