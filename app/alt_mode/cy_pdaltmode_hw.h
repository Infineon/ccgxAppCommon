/******************************************************************************
* File Name:   cy_pdaltmode_hw.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Alternate Mode Hardware Control
*              implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_HW_H
#define CY_PDALTMODE_HW_H

/*****************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "cy_pdaltmode_defines.h"

/*****************************************************************************
 * MACRO Definition
 *****************************************************************************/

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

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
 * Data Structure Definition
 ****************************************************************************/

/**
* \addtogroup group_pdaltmode_data_structures
* \{
*/

/** \} group_pdaltmode_data_structures */

/*****************************************************************************
 * Global Variable Declaration
 *****************************************************************************/

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
* \addtogroup group_pdaltmode_functions
* \{
*/
/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_SetCbk
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \param cbk
* Callback function for ALT. MODE hardware events.
*
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HW_SetCbk(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_cbk_pdaltmode_hw_cmd_t cbk);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_DeInit
****************************************************************************//**
*
* This function deinits all HW related to alt modes.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HW_DeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_EvalAppAltHwCmd
****************************************************************************//**
*
* This function evaluates received HPD application command.
*
* \param ptrAltModeContext
* Pointer to alt mode context.
*
* \param cmd_param
* Pointer to received application HW command data.
*
* \return
* true if APP command passed successful, false if APP command is invalid
* or contain unacceptable fields.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_EvalAppAltHwCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *cmd_param);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_EvalHpdCmd
****************************************************************************//**
*
* This function evaluates received HPD application command.
*
* \param ptrAltModeContext
* Pointer to alt mode context.
*
* \param cmd
* Received HPD application command data.
*
* \return
* true if HPD APP command passed successful, false if HPD APP command
* is invalid or contain unacceptable fields.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_EvalHpdCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t cmd);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_EvalMuxCmd
****************************************************************************//**
*
* This function evaluates received MUX application command.
*
* \param ptrAltModeContext
* Pointer to alt mode context.
*
* \param cmd
* Received MUX application command data.
*
* \return
* true if MUX APP command passed successful, false if MUX APP command
* is invalid or contain unacceptable fields.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_EvalMuxCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t cmd);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_SetMux
****************************************************************************//**
*
* This function sex appropriate MUX configuration based on the function parameters.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \param cfg
* Required MUX configuration.
*
* \param custom_data
* Additional data in case of custom AR MUX configuration.
*
* \return
* true if MUX set passed successful, false if MUX setting is invalid or
* contain unacceptable fields.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_SetMux(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t cfg, uint32_t custom_data);


#if ICL_ENABLE
/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_IgnoreMuxChanges
****************************************************************************//**
*
* Ignore changes to MUX settings
*
* \param ptrAltModeContext
* PdStack Library Context pointer.
*
* \param ignore
* Ignore changes to any MUX state on this
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HW_IgnoreMuxChanges(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool ignore);
#endif /* ICL_ENABLE */


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_GetMuxState
****************************************************************************//**
*
* Get the current state of the MUX.
*
* \param ptrAltModeContext
* Pointer to the altmode context
*
* \return
* Active MUX setting
*
*******************************************************************************/
cy_en_pdaltmode_mux_select_t Cy_PdAltMode_HW_GetMuxState(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_IsIdle
****************************************************************************//**
*
* Check whether the ALT. MODE hardware block is idle.
* This function is part of the deep-sleep entry checks for the CCG device, and checks whether
* there are any pending ALT. MODE hardware commands that require the device to be active.
*
* \param ptrAltModeContext
* Pointer to the altmode context
*
* \return
* true if the block is idle, false otherwise.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_Sleep
****************************************************************************//**
*
* Prepare the Alt. Mode hardware state for device deep-sleep.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HW_Sleep(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_Wakeup
****************************************************************************//**
*
* Restore Alt. Mode hardware state after device resumes from deep-sleep.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HW_Wakeup(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_DpSnkGetHpdState
****************************************************************************//**
*
* Return HPD state based on HPD Queue events.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* true if HPD is connected, false otherwise
*
*******************************************************************************/
bool Cy_PdAltMode_HW_DpSnkGetHpdState(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_MuxCtrlInit
****************************************************************************//**
*
* Initialize the Type-C Data Mux for a specific PD port.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \return
* Returns true if the MUX is initialized successfully, false otherwise.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_MuxCtrlInit(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_MuxCtrlSetCfg
****************************************************************************//**
*
* Set the Type-C MUX to the desired configuration.
*
* \param ptrAltModeContext
* Pointer to the alt mode context.
*
* \param cfg
* Desired MUX configuration.
*
* \return
* Returns true if the operation is successful, false otherwise.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_MuxCtrlSetCfg(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_mux_select_t cfg);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HW_IsHostHpdVirtual
****************************************************************************//**
*
* Check whether the solution is configured to support virtual HPD.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* Returns true if virtual HPD is enabled.
*
*******************************************************************************/
bool Cy_PdAltMode_HW_IsHostHpdVirtual(cy_stc_pdaltmode_context_t * ptrAltModeContext);

#endif /* CY_PDALTMODE_HW_H */

/** \} group_pdaltmode_functions */

/* [] END OF FILE */
