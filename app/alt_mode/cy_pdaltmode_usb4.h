/******************************************************************************
* File Name:   cy_pdaltmode_usb4.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the USB4 mode handler.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef PDALTMODE_CY_PDALTMODE_USB4_H_
#define PDALTMODE_CY_PDALTMODE_USB4_H_


#include <stdint.h>

#include "cy_pdaltmode_defines.h"

/**
* \addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_Usb4_Enter
****************************************************************************//**
*
* Initiate USB4 enter mode command to port partner or cable.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param sopType
* sop packet type.
*
* \param retry
* Indicates that the request is being retried.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Usb4_Enter(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_sop_t sopType, bool retry);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Usb4_UpdateDataStatus
****************************************************************************//**
*
* get USB4 enter mode command parameters.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param eudo
* The Enter USB Data Object used to enter USB4.
*
* \param val
* Original value of the MUX data status register.
*
* \return
* Updated value of the MUX data status register.
*
*******************************************************************************/
uint32_t Cy_PdAltMode_Usb4_UpdateDataStatus (cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_pd_pd_do_t eudo, uint32_t val);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Usb4_DataRstRetryCbk
****************************************************************************//**
*
* Data Reset Retry callback
*
* \param id
* Timer index.
*
* \param context
* Callback Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Usb4_DataRstRetryCbk (cy_timer_id_t id, void *context);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Usb4_TbtHandler
****************************************************************************//**
*
* USB4 TBT Handler
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param vdm_evt
* VDM Task Manager event.
*
* \return
* Return VDM Task.
*
*******************************************************************************/
cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Usb4_TbtHandler(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt);

/** \} group_pdaltmode_functions */

#endif /* PDALTMODE_CY_PDALTMODE_USB4_H_ */
