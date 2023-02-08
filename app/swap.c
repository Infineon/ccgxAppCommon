/******************************************************************************
* File Name:   swap.c
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

#include <swap.h>
#include <app.h>
#include "config.h"
#include "cy_usbpd_config_table.h"
#include "cy_pdaltmode_defines.h"

#if ICL_ENABLE
#include <icl.h>    
#endif /* ICL_ENABLE */


static cy_en_pdstack_app_req_status_t get_response(cy_stc_pdstack_context_t * context, uint8_t raw_resp)
{
#if CY_PD_REV3_ENABLE
    cy_stc_pd_dpm_config_t* dpm = &(context->dpmConfig);
#endif /* CY_PD_REV3_ENABLE */

    cy_en_pdstack_app_req_status_t retVal;
    switch(raw_resp)
    {
    case APP_RESP_ACCEPT:
        retVal = CY_PDSTACK_REQ_ACCEPT;
        break;
    case APP_RESP_WAIT:
        retVal =  CY_PDSTACK_REQ_WAIT;
        break;
#if CY_PD_REV3_ENABLE
    case APP_RESP_NOT_SUPPORTED:
        if(dpm->specRevSopLive <= CY_PD_REV2)
        {
            retVal = CY_PDSTACK_REQ_REJECT;
        }
        else
        {
            retVal = CY_PDSTACK_REQ_NOT_SUPPORTED;
        }
        break;
#endif /* CY_PD_REV3_ENABLE */
    default:
        retVal = CY_PDSTACK_REQ_REJECT;
        break;
    }
    return retVal;
}

#if (ROLE_PREFERENCE_ENABLE)

#if (POWER_ROLE_PREFERENCE_ENABLE)
/* Variable storing current preference for power role. */
extern volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

/* Variable storing current preference for data role. */
extern volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];

#endif /* (ROLE_PREFERENCE_ENABLE) */

void eval_dr_swap (cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    cy_en_pdstack_app_req_status_t result = CY_PDSTACK_REQ_REJECT;

    if (((cy_stc_pdaltmode_context_t *)(context->ptrAltModeContext))->appStatus.alt_mode_entered == true)
    {
        result = CY_PDSTACK_REQ_SEND_HARD_RESET;
    }
    else
    {
#if (ROLE_PREFERENCE_ENABLE)
        /* Don't accept DR_SWAP away from preferred role. */
        if (dpm_get_info(port)->curPortType != app_pref_data_role[port])
#endif /* (ROLE_PREFERENCE_ENABLE) */
        {
            /* Use Configuration table for Response */
            uint8_t dr_swap_response = (pd_get_ptr_app_tbl(context->ptrUsbPdContext)->swap_response & CY_DR_SWAP_RESPONSE_MASK);
            result = get_response(context, dr_swap_response);
        }
    }

    app_get_resp_buf(context->port)->reqStatus = result;
    app_resp_handler(context, app_get_resp_buf(context->port));
}

void eval_pr_swap (cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    cy_en_pdstack_app_req_status_t result = CY_PDSTACK_REQ_REJECT;

    /*
     * Default response shall be NOT_SUPPORTED instead of REJECT if current mode
     * is PD REV3 and port role is source/sink only.
     */
    const cy_stc_pdstack_dpm_status_t* dpm = &context->dpmStat;

    #if CY_PD_REV3_ENABLE
        if (
                (context->dpmConfig.specRevSopLive >= CY_PD_REV3) &&
                (dpm->portRole != CY_PD_PRT_DUAL)
           )
        {
            result = CY_PDSTACK_REQ_NOT_SUPPORTED;
        }
    #endif /* CCG_PD_REV3_ENABLE */

#if ((!CY_PD_SINK_ONLY) && (!CY_PD_SOURCE_ONLY))

    uint8_t pdo_mask;

    if(context->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
    {
        pdo_mask = dpm->srcPdoMask;
    }
    else
    {
        pdo_mask = dpm->snkPdoMask;
    }

    /*
     * Default response shall be NOT_SUPPORTED instead of REJECT if current mode
     * is PD REV3 and port role is source/sink only.
     */
#if CY_PD_REV3_ENABLE
    if (
            (context->dpmConfig.specRevSopLive >= CY_PD_REV3) &&
            (dpm->portRole != CY_PD_PRT_DUAL)
       )
    {
        result = CY_PDSTACK_REQ_NOT_SUPPORTED;
    }
#endif /* CY_PD_REV3_ENABLE */

#if (POWER_ROLE_PREFERENCE_ENABLE)
    /* Do not allow PR_SWAP to a non-preferred role. */
    if (app_pref_power_role[port] != dpm->curPortRole)
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
    {
        if (
                (dpm->deadBat == false) && (dpm->portRole == CY_PD_PRT_DUAL) &&
                (
                 ((pdo_mask & (0x1 << CY_PD_EXTERNALLY_POWERED_BIT_POS)) == 0) ||
                 (context->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
                )
           )
        {
            uint8_t pr_swap_response = ((pd_get_ptr_app_tbl(context->ptrUsbPdContext)->swap_response & CY_PR_SWAP_RESPONSE_MASK) >> CY_PR_SWAP_RESPONSE_POS);
            result = get_response(context, pr_swap_response);
        }
    }
#if (POWER_ROLE_PREFERENCE_ENABLE)
    else
    {
        result = REQ_REJECT;
    }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

#else

    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        result = CY_PDSTACK_REQ_NOT_SUPPORTED;
    }

#endif /* ((!CCG_SINK_ONLY) && (!CY_PD_SOURCE_ONLY)) */

    app_get_resp_buf(context->port)->reqStatus = result;
    app_resp_handler(context, app_get_resp_buf(context->port));
}

void eval_vconn_swap (cy_stc_pdstack_context_t * context, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    cy_en_pdstack_app_req_status_t result = CY_PDSTACK_REQ_REJECT;

#if !CCG_VCONN_DISABLE    
    const cy_stc_pd_dpm_config_t* dpm = &(context->dpmConfig);

    if (
            (dpm->vconnLogical == true)
       )
    {
        /* Delay VConn SWAP if the alternate mode discovery sequence requires us to be able to talk to EMCA. */
        if (app_get_status(context->port)->keep_vconn_src)
        {
            result = CY_PDSTACK_REQ_WAIT;
        }
        else
        {
            result = CY_PDSTACK_REQ_ACCEPT;
        }
    }
    else
    {
        uint8_t vconn_swap_response = ((pd_get_ptr_app_tbl(context->ptrUsbPdContext)->swap_response & CY_VCONN_SWAP_RESPONSE_MASK) >> CY_VCONN_SWAP_RESPONSE_POS);
        result = get_response(context, vconn_swap_response);

        if (result == CY_PDSTACK_REQ_ACCEPT)
        {
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)) || (defined(CCG6DF)) || (defined(CCG6SF)))
            if (!pd_is_v5v_supply_on (context->port))
            {
                result = CY_PDSTACK_REQ_WAIT;
            }
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)) || (defined(CCG6DF)) || (defined(CCG6SF))) */

#if VCONN_OCP_ENABLE
            /* Do not allow VCONN_SWAP to become VConn source if fault is active. */
            if ((app_get_status(context->port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
            {
                result = CY_PDSTACK_REQ_REJECT;
            }
#endif /* VCONN_OCP_ENABLE */
        }
    }
#if ICL_ENABLE
        /* Do not accept VConn SWAP to source role if VSYS is not up. */
        if (icl_vsys_is_present() == false)
        {
            result = CY_PDSTACK_REQ_REJECT;
        }
#endif /* ICL_ENABLE */    
#endif /* !CCG_VCONN_DISABLE */    

    app_get_resp_buf(context->port)->reqStatus = result;
    app_resp_handler(context, app_get_resp_buf(context->port));
}

#if ((!CY_PD_SOURCE_ONLY) && (!CCG_SINK_ONLY))
#if CY_PD_REV3_ENABLE

void eval_fr_swap (cy_stc_pdstack_context_t* context, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    /* Always accept, FRS support is enabled/disabled by separate bit in config table */
    cy_en_pdstack_app_req_status_t result = CY_PDSTACK_REQ_ACCEPT;

    app_get_resp_buf(context->port)->reqStatus = result;
    app_resp_handler(context, app_get_resp_buf(context->port));

}

#endif /* CY_PD_REV3_ENABLE */
#endif /* ((!CY_PD_SOURCE_ONLY) && (!CCG_SINK_ONLY)) */

 /* End of File */

