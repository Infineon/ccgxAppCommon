/******************************************************************************
* File Name:   cy_pdaltmode_mngr.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Alternate Mode Source
*              implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_MNGR_H
#define CY_PDALTMODE_MNGR_H

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdaltmode_defines.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

#define ATCH_TGT                        (1u)
/**< Internal alt modes manager denotation of attached UFP target. */

#define CABLE                           (2u)
/**< Internal alt modes manager denotation of the attached cable. */

#define MODE_NOT_SUPPORTED              (0xFFu)
/**< Internal alt modes manager denotation of non supported alternate mode. */

#define EMPTY_VDO                       (0x00000000u)
/**< Internal alt modes manager denotation in case if VDO sending not needed. */

#define NONE_VDO                        (0xFFFFFFFFu)
/**< Internal alt modes manager denotation of zero VDO. */

#define VDO_START_IDX                   (1u)
/**< Start index of VDO data in VDM message. */

#define NONE_MODE_MASK                  (0x00000000u)
/**< Empty mask. */

#define FULL_MASK                       (0xFFFFFFFFu)
/**< All bit set full mask. */

#define EN_FLAG_MASK                    (0x00000001u)
/**< This mask is used by IS_FLAG_CHECKED macro to check if bit is set in some mask. */

#define EXIT_ALL_MODES                  (0x7u)
/**< Object position corresponding to an Exit All Modes VDM command. */

#define CBL_DIR_SUPP_MASK               (0x780u)
/**< Mask to find out cable directionality support for cable discovery ID response. */

#define ALT_MODE_EVT_IDX                (0u)
/**< Index of alt mode APP event code. */

#define ALT_MODE_EVT_DATA_IDX           (1u)
/**< Index of data for alt mode APP event. */

#define CY_PDALTMODE_NO_DATA            (0u)
/**< Internal alt modes manager denotation of object without useful data. */

#define MAX_RETRY_CNT                   (9u)
/**< Maximum number of VDM send retries in case of no CRC response, timeout or BUSY response. */

#define AM_SVID_CONFIG_SIZE_IDX         (0u)
/**< Index of size of alternate mode configuration packet. */    
    
#define AM_SVID_CONFIG_OFFSET_IDX       (1u)
/**< Offset of SVID in alternate mode configuration packet. */      
    
#define DFP_ALT_MODE_HPI_OFFSET         (8u)
/**< Offset to get which DFP alt modes should be enabled . */    

#define UFP_ALT_MODE_HPI_MASK           (0xFFu)
/**< Mask to get which UFP alt modes should be enabled . */  

#define SET_FLAG(status, bit_idx)       ((status) |= ((uint32_t)1u << ((uint32_t)(bit_idx))))
/**< Set appropriate bit_idx to 1 in the status variable. */

#define REMOVE_FLAG(status, bit_idx)    ((status) &= (~((uint32_t)1u << ((uint32_t)(bit_idx)))))
/**< Set appropriate bit_idx to 0 in the status variable. */

#define IS_FLAG_CHECKED(status, bit_idx)        (((status) >> (uint32_t)(bit_idx)) & EN_FLAG_MASK)
/**< Return true if bit_idx of the status variable is set to 1. */

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
 * Global Function Declaration
 *****************************************************************************/

/**
* \addtogroup group_pdaltmode_functions
* \{
*/


/*******************************************************************************
* Function Name: Cy_PdAltMode_Task
****************************************************************************//**
*
* Handler for PD Alt Mode level asynchronous tasks .
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None.
*
*******************************************************************************/
void Cy_PdAltMode_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_RegAltModeMngr
****************************************************************************//**
*
* This function register pointers to attached dev/ama/cable info and run
* alt modes manager if success.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param atch_tgt_info
* Pointer to struct which holds discovery info about attached targets.
*
* \param vdm_msg_info
* Pointer to struct which holds info of received/sent VDM
*
* \return
* VDM manager task VDM_TASK_ALT_MODE if CCG support any alternate
* mode. If CCG does not support alternate modes function returns VDM_TASK_EXIT.
*
*******************************************************************************/
cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_RegAltModeMngr(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info, cy_stc_pdaltmode_vdm_msg_info_t* vdm_msg_info);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_AltModeProcess
****************************************************************************//**
*
* This function uses by DFP VDM manager to run alt modes manager processing.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param vdm_evt
* Current DFP VDM manager event.
*
* \return
* DFP VDM manager task based on alt modes manager processing results.
*
*******************************************************************************/
cy_en_pdaltmode_vdm_task_t Cy_PdAltMode_Mngr_AltModeProcess(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pdaltmode_vdm_evt_t vdm_evt);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_EvalRecVdm
****************************************************************************//**
*
* This function run received VDM analysis if CCG is UFP
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param vdm
* Pointer to pd packet which contains received VDM.
*
* \return
* true if received VDM could handled successful and VDM response need
* to be sent with ACK. If received VDM should be NACKed then returns false
*
*******************************************************************************/
bool Cy_PdAltMode_Mngr_EvalRecVdm(cy_stc_pdaltmode_context_t *ptrAltModeContext, const cy_stc_pdstack_pd_packet_t *vdm);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_FormAltModeEvent
****************************************************************************//**
*
* Fill alt mode APP event fields with appropriate values.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param svid
* SVID of alternate mode which event refers to.
*
* \param am_idx
* Index of alternate mode in compatibility table which event refers to.
*
* \param evt
* Alternate mode APP event.
*
* \param data
* Alternate mode APP event data.
*
* \return
* pointer to the event related data.
*
*******************************************************************************/
const uint32_t* Cy_PdAltMode_Mngr_FormAltModeEvent(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid, uint8_t am_idx, cy_en_pdaltmode_app_evt_t evt, uint32_t data);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_EvalAppAltModeCmd
****************************************************************************//**
*
* This function analyzes, parses and run alternate mode analysis function
* if received command is specific alt mode command.
*
* \param port
* PPort index the function is performed for.
*
* \param cmd
* Pointer to received alt mode APP command.
*
* \param data
* Pointer to received alt mode APP command additional data.
*
* \return
* true if APP command passed successful, false if APP command is invalid
* or contains unacceptable fields.
*
*******************************************************************************/
bool Cy_PdAltMode_Mngr_EvalAppAltModeCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t *cmd, uint8_t *data);


/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_IsAltModeMngrIdle
****************************************************************************//**
*
* Check whether the VDM manager for the selected port is idle.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* true if manager is busy, false - if idle.
*
*******************************************************************************/
bool Cy_PdAltMode_Mngr_IsAltModeMngrIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if SYS_DEEPSLEEP_ENABLE
/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_Sleep
****************************************************************************//**
*
* Prepare the alt modes manager for device deep sleep.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/

void Cy_PdAltMode_Mngr_Sleep(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_Wakeup
****************************************************************************//**
*
* Restore the alt modes manager state after waking from deep sleep.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_Wakeup(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /* SYS_DEEPSLEEP_ENABLE */

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_ResetAltModeInfo
****************************************************************************//**
*
* Resets alternate mode command info structure.
*
* \param info
*  Pointer to alternate mode info structure.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_ResetAltModeInfo(cy_stc_pdaltmode_mngr_info_t* info);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetVdmBuff
****************************************************************************//**
*
* Returns pointer to VDM buffer.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* Pointer to VDM buffer.
*
*******************************************************************************/
cy_stc_pdstack_dpm_pd_cmd_buf_t* Cy_PdAltMode_Mngr_GetVdmBuff(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_IsSvidSupported
****************************************************************************//**
*
* Check for presence of alt modes for given svid in alt modes compatibility table.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param svid
*  Received SVID.
*
* \return
* index of supported SVID by CCG.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_Mngr_IsSvidSupported(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetStatus
****************************************************************************//**
*
* Retrieve the current alternate mode status for a PD port.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* The alternate mode status.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_Mngr_GetStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_LayerReset
****************************************************************************//**
*
* Reset alt modes layer.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_LayerReset(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_ResetInfo
****************************************************************************//**
*
* Reset the Alternate mode manager state.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_ResetInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE))

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_ExitAll
****************************************************************************//**
*
* Exit all active alternate modes.
*
* \param ptrPdStackContext
* PdStack Library Context pointer.
*
* \param send_vdm_exit
* Flag which indicates is sending Exit VDM is required during exit all modes procedure.
*
* \param exit_all_cbk
*  Callback which will call after exit all alt modes procedure will be finished.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_ExitAll(cy_stc_pdstack_context_t *ptrPdStackContext, bool send_vdm_exit, cy_pdstack_pd_cbk_t exit_all_cbk);
#endif

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetAltModeNumb
****************************************************************************//**
*
* Get number of the alternate modes for chosen port.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* Number of the alternates modes for the chosen port and current data role.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_Mngr_GetAltModeNumb(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx
****************************************************************************//**
*
* Get index of selected SVID from config table
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param type
* type of the port i.e. port_type_t.
*
* \param svid
* SVID.
*
* \return
* SVID index.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_Mngr_GetAltModesConfigSvidIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, cy_en_pd_port_type_t type, uint16_t svid);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_SetAltModeMask
****************************************************************************//**
*
* Set mask for alternate modes which should be enabled.
*
* \param port
* PD port index.
*
* \param mask
* Mask.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_SetAltModeMask(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t mask);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_SetCustomSvid
****************************************************************************//**
*
* Set alternate modes custom SVID.
*
* \param port
* PD port index.
*
* \param svid
* SVID.
*
* \return
* true if the SVID is set otherwise false.
*
*******************************************************************************/
bool Cy_PdAltMode_Mngr_SetCustomSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint16_t svid);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetCustomSvid
****************************************************************************//**
*
* Returns alternate modes custom SVID.
*
* \param port
* PD port index.
*
* \return
* SVID value.
*
*******************************************************************************/
uint16_t Cy_PdAltMode_Mngr_GetCustomSvid(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetSvidFromIdx
****************************************************************************//**
*
* Returns SVID which is at index position in configuration table if such SVID is available.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param idx
* Index of the SVID.
*
* \return
* SVID value.
*
*******************************************************************************/
uint16_t Cy_PdAltMode_Mngr_GetSvidFromIdx(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t idx);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetModeInfo
****************************************************************************//**
*
* Get alt mode info.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param alt_mode_idx
* Alt mode index.
*
* \return
* Pointer to the alternate mode info.
*
*******************************************************************************/
cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_Mngr_GetModeInfo(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t alt_mode_idx);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetPdAltModeContext
****************************************************************************//**
*
* Get Pd Alt Mode context.
*
* \param port
* PD port index.
*
* \return
* PdAltMode Library Context pointer.
*
*******************************************************************************/
cy_stc_pdaltmode_context_t *Cy_PdAltMode_Mngr_GetPdAltModeContext(uint8_t port);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetCableUsbCap
****************************************************************************//**
*
* This function returns Cable USB capabilities.
*
* \param ptrPdStackContext
* PdStack Library Context pointer.
*
* \return
* Return Cable USB capabilities.
*
*******************************************************************************/
cy_en_pdstack_usb_data_sig_t Cy_PdAltMode_Mngr_GetCableUsbCap(cy_stc_pdstack_context_t *ptrPdStackContext);

#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetActiveAltModeMask
****************************************************************************//**
*
* Get active alt modes.
*
* \param port
* PD port index.
*
* \return
* Return the active alt mode mask info
*
*******************************************************************************/
uint32_t Cy_PdAltMode_Mngr_GetActiveAltModeMask(uint8_t port);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_SetActiveAltModeState
****************************************************************************//**
*
* This function set the alt mode state for given port.
*
* \param port
* Index of Type-C port.
*
* \param alt_mode_idx
* Alt Mode Index of Type-C port.
*
*
* \return
* True if SVID is supported by CCG
*
*******************************************************************************/
void Cy_PdAltMode_Mngr_SetActiveAltModeState(uint8_t port, uint8_t alt_mode_idx);

/*******************************************************************************
* Function Name: Cy_PdAltMode_Mngr_GetSuppAltModes
****************************************************************************//**
*
* Get a bitmap of supported alt modes, given the current connection.
*
* \param port
* PD port index.
*
* \return
* Returns the supported alternate modes.
*
*******************************************************************************/

/**
 * @brief Get a bitmap of supported alt modes, given the current connection 
 * @param port PD port index.
 * @return the supported alternate modes.
 */
uint32_t Cy_PdAltMode_Mngr_GetSuppAltModes(uint8_t port);

#endif /*CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED*/

/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_MNGR_H */

/* [] END OF FILE */
