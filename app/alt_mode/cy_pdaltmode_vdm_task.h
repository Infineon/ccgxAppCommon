/******************************************************************************
* File Name:   cy_pdaltmode_vdm_task.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the VDM Task Manager.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_VDM_TASK_H
#define CY_PDALTMODE_VDM_TASK_H

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>

#include "cy_pdaltmode_defines.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/** Start index of received VDM packet in case of response to DISC SVID command. */
#define PD_SVID_ID_HDR_VDO_START_IDX        (4u)

/** Index of AMA VDO in DISCOVER_ID response from port partner (after skipping the VDM header). */
#define PD_DISC_ID_AMA_VDO_IDX              (3u)

#if SAVE_SUPP_SVID_ONLY
/** Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#define MAX_SVID_VDO_SUPP                   (4u)
#else
/** Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#define MAX_SVID_VDO_SUPP                   (32u)
#endif

/** Maximum number of cable SVIDs VDM task manager can hold in the memory. */
#define MAX_CABLE_SVID_SUPP                 (4u)

/** Standard vendor ID allocated to USB PD specification. */
#define STD_SVID                            (0xFF00u)

/** Apple SVID defined by Apple specification. */
#define APPLE_SVID                          (0x05ACu)

/** Maximum number of DISCOVER_SVID attempts that will be performed. */
#define MAX_DISC_SVID_COUNT                 (10u)

/** Maximum number of Data Reset command retries. */
#define DATA_RST_RETRY_NUMB                 (5u)

/** Bit shift to match host_supp configuration parameter with USB4 EUDO . */
#define USB4_EUDO_HOST_PARAM_SHIFT          (13u)

/** Maximum number of EMCA SOFT_RESET attempts to be made. */
#define MAX_EMCA_DP_RESET_COUNT             (3u)

/*****************************************************************************
 * Data Structure Definition
 ****************************************************************************/

#if CCG_UCSI_ENABLE  
extern bool modal_op_support;
/**< Added check for Modal operation support bit for SOP. If no modal operation, firmware will return success to 
     UCSI command with zero length. */
#endif /* CCG_UCSI_ENABLE */

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/

/**
* \addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_Enable
****************************************************************************//**
*
* Enables VDM task mngr functionality.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_Enable(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_Manager
****************************************************************************//**
*
* Main VDM task mngr function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_Manager(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_MngrDeInit
****************************************************************************//**
*
* This function deinits VDM task manager and resets all related variables.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_MngrDeInit(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_MngrExitModes
****************************************************************************//**
*
* Get the alternate mode state machine to exit any active modes. This is called in response to
* a VConn related fault condition.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_MngrExitModes(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if SYS_DEEPSLEEP_ENABLE

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_IsIdle
****************************************************************************//**
*
* Check whether the VDM task for the port is idle.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* True is VDM Task Idle
*
*******************************************************************************/
bool Cy_PdAltMode_VdmTask_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP))

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_IsUfpDiscStarted
****************************************************************************//**
*
* Starts Discover process when CCG is UFP due to PD 3.0 spec.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* true if Discover process has started, false if VDM manager id busy.
*
*******************************************************************************/
bool Cy_PdAltMode_VdmTask_IsUfpDiscStarted(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) */

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_GetDiscIdResp
****************************************************************************//**
*
* Obtain the last DISC_ID response received by the CCG device from a port partner.
*
* \param ptrAltModeContext
* Pointer to the AltMode Context
*
* \param resp_len_p
* Length of response in PD data objects.
*
* \return
*  Pointer to the actual response data.
*
*******************************************************************************/
uint8_t *Cy_PdAltMode_VdmTask_GetDiscIdResp(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *resp_len_p);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_GetDiscSvidResp
****************************************************************************//**
*
* Obtain the collective DISC_SVID response received by the CCG device from a port partner.
*
* \param port
* PD port index.
*
* \param resp_len_p
* Length of response in PD data objects.
*
* \return
* Pointer to the actual response data.
*
*******************************************************************************/
uint8_t *Cy_PdAltMode_VdmTask_GetDiscSvidResp(uint8_t port, uint8_t *resp_len_p);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_GetEvt
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
cy_en_pdaltmode_vdm_evt_t Cy_PdAltMode_VdmTask_GetEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_GetTask
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_GetTask(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_SetTask
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param task
* Callback function for ALT. MODE hardware events.
*
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_SetTask(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_task_t task);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_SetEvt
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param evt
* Callback function for ALT. MODE hardware events.
*
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_SetEvt(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t evt);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_SetDiscParam
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer..
*
* \param sop
* Callback function for ALT. MODE hardware events.
*
* \param cmd
* Callback function for ALT. MODE hardware events.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_SetDiscParam(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t sop, cy_en_pdstack_std_vdm_cmd_t cmd);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_ResumeHandler
****************************************************************************//**
*
* This function registers an ALT. MODE hardware callback function.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_VdmTask_ResumeHandler(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_InitiateVcsCblDiscovery
****************************************************************************//**
*
* Function to initiate VConn Swap and Cable discovery before we can go ahead with the next SOP'/SOP'' message.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* true - VDM sequence needs to be delayed because we need to do a VConn Swap.
*
*******************************************************************************/
bool Cy_PdAltMode_VdmTask_InitiateVcsCblDiscovery(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_VdmTask_UpdateVcsStatus
****************************************************************************//**
*
* Function to resume alternate mode discovery state machine after cable discovery tasks are complete.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_VdmTask_UpdateVcsStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_VDM_TASK_H */

/* [] END OF FILE */

