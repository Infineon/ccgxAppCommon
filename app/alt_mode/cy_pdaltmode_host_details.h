/******************************************************************************
* File Name:   cy_pdaltmode_host_details.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Host Details feature.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_HOST_DETAILS_H
#define CY_PDALTMODE_HOST_DETAILS_H

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include "cy_pdaltmode_defines.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
* \addtogroup group_pdaltmode_functions
* \{
*/

#if STORE_DETAILS_OF_HOST
/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_Task
****************************************************************************//**
*
* Host Details Task.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_GetHostDetailsAvailable
****************************************************************************//**
*
* Get Host Details Available
*
* \return
* flag.
*
*******************************************************************************/
bool Cy_PdAltMode_HostDetails_GetHostDetailsAvailable(void);

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_SetSendEuWithHostPresentSet
****************************************************************************//**
*
* Set Eu With Host Present Set bit.
*
* \param value
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_SetSendEuWithHostPresentSet(uint8_t value);

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_GetSendEuWithHostPresentSet
****************************************************************************//**
*
* Get Eu With Host Present bit Set.
*
* \return
* flag that indicates Eu With Host Present bit is Set.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_HostDetails_GetSendEuWithHostPresentSet(void);

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_CheckIfRidgeNeedsToBeUpdated
****************************************************************************//**
*
* Check if Ridge needs to be updated.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param reg_config
* Ridge Register.
*
* \return
* flag.
*
*******************************************************************************/
bool Cy_PdAltMode_HostDetails_CheckIfRidgeNeedsToBeUpdated(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t reg_config);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection
****************************************************************************//**
*
* Status Update After Host Connection.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_StatusUpdateAfterHostConnection(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_ChangeDsPortBehaviorBasedOnHostCapability
****************************************************************************//**
*
* Change Ds Port Behavior Based On Host Capability.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_ChangeDsPortBehaviorBasedOnHostCapability(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType
****************************************************************************//**
*
* Control Mode Based On Host Type.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* * \param mode_mask
* Host Details Control Mode mask .
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_ControlModeBasedOnHostType(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t mode_mask);


/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_Init
****************************************************************************//**
*
* Initiate Host Details structure.
*
* \param hostAltModeContext
* Host PdAltMode Library Context pointer.
*
* * \param deviceAltModeContext
* Device PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_Init(cy_stc_pdaltmode_context_t *hostAltModeContext, cy_stc_pdaltmode_context_t *deviceAltModeContext );

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_SendHardResetCbk
****************************************************************************//**
*
* Timer Callback to send Hard Reset
*
* \param id
* Timer index.
*
* \param ptrContext
* Callback Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_SendHardResetCbk(cy_timer_id_t id, void * ptrContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_RestartDpmState
****************************************************************************//**
*
* DPM Callback to Reset DPM
*
* \param id
* Timer index.
*
* \param ptrContext
* Callback Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_RestartDpmState(cy_timer_id_t id, void * ptrContext);

#endif /* STORE_DETAILS_OF_HOST*/

#endif /* CY_PDALTMODE_HOST_DETAILS_H */

/** \} group_pdaltmode_functions */

/* [] END OF FILE */
