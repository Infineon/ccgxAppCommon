/******************************************************************************
* File Name:   cy_pdaltmode_intel_vid.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Thunderbolt (Intel VID) alternate
*              mode handler.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_INTEL_VID_H
#define CY_PDALTMODE_INTEL_VID_H

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

#define INTEL_VID                       (0x8087u)
/**< Intel (Thunderbolt 3) SVID. */

#define TBT_ALT_MODE_ID                 (1u)
/**< Unique ID assigned to Thunderbolt mode in the CCGx SDK. */

#define VPRO_ALT_MODE_ID                (2u)
/**< Unique ID assigned to Vpro mode in the CCGx SDK. */    

//#define MAX_TBT_VDO_NUMB                (1u)
/**< Maximum number of VDOs Thunderbolt-3 uses for the alt mode flow. */

#define TBT_VDO_IDX                     (0u)
/**< Index of VDO used for Thunderbolt. */

#define TBT_MODE_VPRO_AVAIL             (0x04000000u)
/**< VPro available bit in TBT Mode VDO. */

#define TBT_MODE_VPRO_DOCK_HOST         (0x04000000u)
/**< Vpro_Dock_and_Host bit in TBT Enter Mode VDO. */

#define GET_VPRO_AVAILABLE_STAT(status) ((status & TBT_MODE_VPRO_AVAIL) != 0)
/**< Macro to get VPro Available bit from Discover Mode response VDO. */

#define GET_LEGACY_TBT_ADAPTER(status)  ((status >> 16) & 0x1)
/**< Macro to get legacy adapter status from Discovery Mode response  VDO. */

#define TBT_EXIT(status)                ((status >> 4) & 0x1)
/**< Macro to get Exit status from Attention VDO. */

#define BB_STATUS(status)               ((status >> 3) & 0x1)
/**< Macro to get BB status from Attention VDO. */

#define USB2_ENABLE(status)             ((status >> 2) & 0x1)
/**< Macro to get USB enable status from Attention VDO. */

#define TBT_DATA_EXIT_MASK              (0xCu)
/**< Mask to indicate that TBT data path should be disabled. */
#if ICL_ENABLE    

#define BB_STS_USB2_ENABLE(status)      ((status >> 2) & 0x3)
/**< Macros to get USB enable and BB status from Attention VDO. */

#define BB_PRESENT_EXIT_MODE            (0x03u)   
/**< Billboard is present and mode has to be exited */
#endif /* ICL_ENABLE */    

/** \} addtogroup group_pdaltmode_macros */

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
* \addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_TBT_RegIntelModes
****************************************************************************//**
*
* This function analysis Discovery information to find out
* if further TBT alternative mode processing is allowed.
*
* \param context
* Pointer to the altmode context.
*
* \param reg_info
* Pointer to structure which holds alt mode register info.
*
* \return
* Pointer to TBT alternative mode command structure if analysis passed
* successful. In case of failure, function returns NULL pointer.
*
*******************************************************************************/
cy_stc_pdaltmode_mngr_info_t* Cy_PdAltMode_TBT_RegIntelModes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info);


/*******************************************************************************
* Function Name: Cy_PdAltMode_TBT_SendExitModeCmd
****************************************************************************//**
*
* Function to initiate TBT mode exit command
*
* \param ptrAltModeContext
* AltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_TBT_SendExitModeCmd(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_INTEL_VID_H */

/* [] END OF FILE */
