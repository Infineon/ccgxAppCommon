/******************************************************************************
* File Name:   cy_pdaltmode_config.h
* \version 2.0
*
* Description: Header file that selects the Alternate Modes supported by PMG1
*              firmware.
*
*******************************************************************************
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_CONFIG_H
#define CY_PDALTMODE_CONFIG_H

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdaltmode_defines.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_dp_sid.h"
#include "cy_pdaltmode_intel_vid.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/*****************************************************************************
 * Global Variable Declaration
 *****************************************************************************/
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
cy_stc_pdaltmode_reg_am_t gl_reg_alt_mode [] =
{
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    {CY_PDALTMODE_DP_SVID, Cy_PdAltMode_DP_RegModes},
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

#if ((TBT_DFP_SUPP) || (TBT_UFP_SUPP))
    {CY_PDALTMODE_TBT_SVID, Cy_PdAltMode_TBT_RegIntelModes},
#endif /* ((TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

#if HPI_AM_SUPP
    {HPI_AM_SVID, Cy_PdAltMode_HPI_RegModes}
#endif /* HPI_AM_SUPP */
};

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

#endif /* CY_PDALTMODE_CONFIG_H */

/* [] END OF FILE */
