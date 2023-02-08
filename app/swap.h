/******************************************************************************
* File Name:   swap.h
* \version 2.0
*
* Description: Swap request (PR_SWAP, DR_SWAP, VCONN_SWAP) handlers.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon App Common Middleware
* \{
*/


#ifndef _SWAP_H_
#define _SWAP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdstack_common.h"
#include "config.h"

/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
/*****************************************************************************
 * Defines
 *****************************************************************************/

#define CY_DR_SWAP_RESPONSE_MASK        0x03    /**< DR SWAP MASK From Config Table */
#define CY_PR_SWAP_RESPONSE_MASK        0x0C    /**< PR SWAP MASK From Config Table */
#define CY_PR_SWAP_RESPONSE_POS            2       /**< PR SWAP Bit Position From Config Table */
#define CY_VCONN_SWAP_RESPONSE_MASK        0x30    /**< VCONN SWAP MASK From Config Table */
#define CY_VCONN_SWAP_RESPONSE_POS        4       /**< VCONN SWAP Bit Position From Config Table */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function evaluates Data role swap request
 *
 * @param context PD Stack context.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_dr_swap(cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler);

/**
 * @brief This function evaluates Power role swap request
 *
 * @param context PD Stack context.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_pr_swap(cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler);

/**
 * @brief This function evaluates VConn swap request
 *
 * @param context PD Stack context.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_vconn_swap(cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler);

#if CY_PD_REV3_ENABLE
/**
 * @brief This function evaluates Fast role swap request
 *
 * @param context PD Stack context.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_fr_swap(cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler);

#endif /* CY_PD_REV3_ENABLE */
/** \} group_ccgxAppCommon_functions */
/** \} group_ccgxAppCommon */

#endif /* _SWAP_H_ */
/* End of File */

