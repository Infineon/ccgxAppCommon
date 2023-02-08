/******************************************************************************
* File Name:   cy_pdaltmode_intel_ridge.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Intel Alpine/Titan Ridge control
*              interface.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_INTEL_RIDGE_H
#define CY_PDALTMODE_INTEL_RIDGE_H

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include "cy_pdaltmode_defines.h"

#include "cy_pdaltmode_hw.h"
#include "cy_usbpd_hpd.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
* \addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_SetDisconnect
****************************************************************************//**
*
* Set Disconnect.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_SetDisconnect(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_SetMux
****************************************************************************//**
*
* This function set AR/TR registers in accordance to input parameters
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param mux_cfg
* MUX configuration.
*
* \param polarity
* Attached target Type-C Polarity.
*
* \param cfg
* Contains AR/TR register settings in case of TBT alt mode is active.
*
* \return
* true if AR/TR was set successful, in the other case - false
*
*******************************************************************************/
bool Cy_PdAltMode_Ridge_SetMux(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t mux_cfg, uint8_t polarity, uint32_t cfg);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_HpdInit
****************************************************************************//**
*
* Enables Titan Ridge the HPD functionality for the specified PD port.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param cbk
* Callback to be used for command completion event.
*
* \return
* Returns CY_PDSTACK_STAT_SUCCESS in case of success, error code otherwise.
*
*******************************************************************************/
cy_en_pdstack_status_t Cy_PdAltMode_Ridge_HpdInit(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_cb_usbpd_hpd_events_t cbk);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_HpdDeInit
****************************************************************************//**
*
* Disables Titan Ridge the HPD functionality for the specified PD port.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_HpdDeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_HpdSendEvt
****************************************************************************//**
*
* Send the desired HPD event out through the Titan Ridge HPD GPIO. Only
* the HPD_EVENT_UNPLUG, HPD_EVENT_UNPLUG and HPD_EVENT_IRQ events
* should be requested.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param evtype
* Type of HPD event to be sent.
*
* \return
* Returns CY_PDSTACK_STAT_SUCCESS in case of success, error code otherwise.
*
*******************************************************************************/
cy_en_pdstack_status_t Cy_PdAltMode_Ridge_HpdSendEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_usbpd_hpd_events_t evtype);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_EvalCmd
****************************************************************************//**
*
* Analysis received ridge command register content.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* * \param stat
* PdAltMode Command register value.
*
* * \param stat_mask
* PdAltMode Bit mask of TR command register which were changed from the previous time
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_EvalCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t stat, uint32_t stat_mask);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_IsHpdChange
****************************************************************************//**
*
* Indicates is HPD level changed from the previous state.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
bool Cy_PdAltMode_Ridge_IsHpdChange(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_SetVpro
****************************************************************************//**
*
* Enable/Disable vPro register in ridge IC.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param en_status
* Enable or disable flag for vPro mode.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_SetVpro(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool en_status);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_UpdateDr
****************************************************************************//**
*
* Updates data role bit in  ridge IC.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_UpdateDr(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if ICL_ENABLE
/*******************************************************************************
* Function Name: Cy_PdAltMode_Ridge_ForceStatusUpdate
****************************************************************************//**
*
* No connection active.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
** \param force_update
* Force next update on this port.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Ridge_ForceStatusUpdate(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t force_update);

#endif /* ICL_ENABLE */
#endif /* CY_PDALTMODE_INTEL_RIDGE_H */

/** \} group_pdaltmode_functions */

/* [] END OF FILE */
