/******************************************************************************
* File Name:   cy_pdaltmode_billboard.c
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the USB Billboard.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/
#include <config.h>
#include <app.h>
#include <cy_pdstack_dpm.h>
#if CCG_HPI_ENABLE
#include <hpi.h>
#include <hpi_internal.h>
#endif /* CCG_HPI_ENABLE */
#include <cy_pdutils_sw_timer.h>
#include <cy_pdaltmode_billboard.h>
#include "srom.h"

#if (CCG_BB_ENABLE != 0)

/**< Billboard connect state variable for BILLBOARD_UPDATE_EVENT. */    
static uint32_t bb_connect_stat[NO_OF_TYPEC_PORTS];           
static volatile bb_handle_t gl_bb[NO_OF_TYPEC_PORTS];

#if CCG_PD_DUALPORT_ENABLE
/* Stores the port to which the billboard device is currently bound. */
static volatile uint8_t bb_active_port = 0xFF;
#endif /* CCG_PD_DUALPORT_ENABLE */

static void bb_set_conn_stat(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t conn_stat)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;

    bb_connect_stat[port] = (uint32_t)conn_stat;

    /* Send BB notifications to the solution */
    /* QAC suppression 0315: This function is designed to accept any type of 4 byte aligned
     * data pointer. In this case the pointer passed is 4 byte aligned and hence safe. */
    sln_pd_event_handler (ptrAltModeContext->pdStackContext, APP_EVT_BB, &(bb_connect_stat[port])); /* PRQA S 0315 */
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Init(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{  
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    uint32_t data;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    if (gl_bb[port].state != BB_STATE_DEINITED)
    {
        return CY_PDSTACK_STAT_BUSY;
    }

    /* Verify that the configuration allows billboard. */
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if(
        (get_auto_config(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_offset != 0u) &&
        (pd_get_ptr_bb_tbl(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_enable != (uint8_t)BB_TYPE_EXTERNAL) && /* PRQA S 3415 */
        (pd_get_ptr_bb_tbl(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_enable < (uint8_t)BB_TYPE_EXT_CONFIGURABLE) /* PRQA S 3415 */
    )
    {
        return CY_PDSTACK_STAT_FAILURE;
    }

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    /* Init alt mode status register by default values */
    data = BB_ALT_MODE_STATUS_INIT_VAL;
    /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
     * and hence it is safe. */
    CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ALT_MODE_STATUS, &data); /* PRQA S 0315 */
    /* Set BB operation model for "EC + Reset" */
    data = HPI_BB_OPER_MODEL;
    /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
     * and hence it is safe. */
    CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_OPER_MODEL, &data); /* PRQA S 0315 */
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    gl_bb[port].type        =  BB_TYPE_EXTERNAL;
    gl_bb[port].state       =  BB_STATE_DISABLED;
    gl_bb[port].alt_status  =  BB_ALT_MODE_STATUS_INIT_VAL;
    gl_bb[port].bb_add_info = 0x00;

    return CY_PDSTACK_STAT_SUCCESS;
}

/* Billboard OFF timer callback. */
static void bb_off_timer_cb (cy_timer_id_t id,  void * callbackCtx)
{
    (void)id;
    cy_stc_pdaltmode_context_t *ptrAltModeContext = callbackCtx;
    uint8_t port = ptrAltModeContext->pdStackContext->port;

    if(gl_bb[port].timeout == 0u)
    {
        /* Disable the billboard interface. */
        (void)Cy_PdAltMode_Billboard_Disable(ptrAltModeContext, false);
    }
    else
    {
        /* Continue the timer until the required delay is achieved. */
        uint32_t timeout = gl_bb[port].timeout;

        if(timeout > BB_OFF_TIMER_MAX_INTERVAL)
        {
            timeout = BB_OFF_TIMER_MAX_INTERVAL;
        }
        gl_bb[port].timeout -= timeout;

        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext, (cy_timer_id_t)APP_BB_OFF_TIMER, timeout, bb_off_timer_cb);

    }
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Enable(cy_stc_pdaltmode_context_t *ptrAltModeContext, bb_cause_t cause)
{
    uint32_t timeout;
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    uint8_t  failinfo;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

#if CCG_PD_DUALPORT_ENABLE
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if ((bb_active_port != 0xFFu) && (bb_active_port != port)) /* PRQA S 3415 */
    {
        return CY_PDSTACK_STAT_BUSY;
    }

    /* Make sure to do the Billboard binding at this stage. */
    bb_active_port = port;
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Check if BB is already running */
    if(gl_bb[port].state != BB_STATE_BILLBOARD)
    {
        /* Queue an enable only if the block is initialized and not in flashing mode. */
        if ((gl_bb[port].state == BB_STATE_DEINITED) ||
                (gl_bb[port].state == BB_STATE_FLASHING))
        {
            return CY_PDSTACK_STAT_NOT_READY;
        }

        /* Allow billboard enumeration only if the configuration allows. */
        /* QAC suppression 3415: The checks on the right hand side of logical operator 
         * are readonly / only required if the check on left hand side is not sufficient to 
         * decide the result of overall expression. Hence, it is not a side effect. */
        if ((cause == BB_CAUSE_AME_SUCCESS) &&
                (pd_get_ptr_bb_tbl(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_always_on == (uint8_t)false)) /* PRQA S 3415 */
        {
            return CY_PDSTACK_STAT_NOT_READY;
        }

        switch (cause)
        {
            case BB_CAUSE_AME_TIMEOUT:
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
                gl_bb[port].bb_add_info |= BB_CAP_ADD_FAILURE_INFO_PD;
                failinfo = gl_bb[port].bb_add_info;
                /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
                 * and hence it is safe. */
                CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ADDL_FAILURE_INFO, &failinfo); /* PRQA S 315 */
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */
                /* Send Connect BB event */
                bb_set_conn_stat(ptrAltModeContext, BB_CONNECT_STAT);
                break;

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
            case BB_CAUSE_AME_FAILURE:
                gl_bb[port].bb_add_info |= BB_CAP_ADD_FAILURE_INFO_PD;
                failinfo = gl_bb[port].bb_add_info;
                /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
                 * and hence it is safe. */
                CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ADDL_FAILURE_INFO, &failinfo); /* PRQA S 0315 */
                break;

            case BB_CAUSE_PWR_FAILURE:
                gl_bb[port].bb_add_info |= BB_CAP_ADD_FAILURE_INFO_PWR;
                failinfo = gl_bb[port].bb_add_info;
                /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
                 * and hence it is safe. */
                CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ADDL_FAILURE_INFO, &failinfo); /* PRQA S 0315 */
                break;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

            case BB_CAUSE_AME_SUCCESS:
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
                failinfo = gl_bb[port].bb_add_info;
                /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
                 * and hence it is safe. */
                CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ADDL_FAILURE_INFO, &failinfo); /* PRQA S 0315 */
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */
                break;

            default:
                /* No statement */
                break;
        }

        /* Ensure that the OFF_TIMER is disabled. */
        Cy_PdUtils_SwTimer_Stop(ptrAltModeContext->pdStackContext->ptrTimerContext, (cy_timer_id_t)APP_BB_OFF_TIMER);

        /* Update the state. */
        gl_bb[port].state = BB_STATE_BILLBOARD;

        /* Disable the billboard interface after the specified timeout in case of SUCCESS status. */
        /* QAC suppression 3415: The checks on the right hand side of logical operator 
         * are readonly / only required if the check on left hand side is not sufficient to 
         * decide the result of overall expression. Hence, it is not a side effect. */
        if(
                (cause == BB_CAUSE_AME_SUCCESS) &&
                (pd_get_ptr_bb_tbl(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_timeout != BB_OFF_TIMER_NO_DISABLE) /* PRQA S 3415 */
          )
        {
            timeout = pd_get_ptr_bb_tbl(ptrAltModeContext->pdStackContext->ptrUsbPdContext)->bb_timeout;
            if (timeout < BB_OFF_TIMER_MIN_VALUE)
            {
                timeout = BB_OFF_TIMER_MIN_VALUE;
            }

            /* Convert time to milliseconds. */
            timeout *= 1000u;
            gl_bb[port].timeout = timeout;

            /* Ensure that the timeout does not exceed parameter boundary. */
            /* QAC suppression 2991, 2995: Based on current value of BB_OFF_TIMER_MIN_VALUE = 60, 
             * this expression is always going to be true. Yet it is retained as the compile 
             * time configuration for BB_OFF_TIMER_MIN_VALUE may be set to 0. */
            if(timeout > BB_OFF_TIMER_MAX_INTERVAL) /* PRQA S 2991, 2995 */
            {
                timeout = BB_OFF_TIMER_MAX_INTERVAL;
            }

            gl_bb[port].timeout -= timeout;
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrAltModeContext->pdStackContext->ptrTimerContext, ptrAltModeContext, (cy_timer_id_t)APP_BB_OFF_TIMER, timeout, bb_off_timer_cb);

        }
        else
        {
            gl_bb[port].timeout = 0;
        }
    }

    return CY_PDSTACK_STAT_SUCCESS;
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_UpdateAltStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t mode_index,
        bb_alt_mode_status_t alt_status)
{
    uint32_t status;
    uint8_t port = ptrAltModeContext->pdStackContext->port;

#if CCG_PD_DUALPORT_ENABLE
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if ((bb_active_port != 0xFFu) && (bb_active_port != port)) /* PRQA S 3415 */
    {
        return CY_PDSTACK_STAT_BUSY;
    }

    /* Make sure to do the Billboard binding at this stage. */
    bb_active_port = port;
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Allow update of alternate mode only if initialized and not in flashing */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].state == BB_STATE_FLASHING))
    {
        return CY_PDSTACK_STAT_NOT_READY;
    }

    if (mode_index >= BB_MAX_ALT_MODES)
    {
        return CY_PDSTACK_STAT_INVALID_ARGUMENT;
    }
    
    status = ((uint32_t)gl_bb[port].alt_status & ~((uint32_t)BB_ALT_MODE_STATUS_MASK << (mode_index << 1)));
    status |= ((uint32_t)alt_status << (mode_index << 1));
    gl_bb[port].alt_status = status;

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    /* Check if status was changed from the previous time */
    if (CALL_MAP(hpi_bb_get_reg)((uint8_t)HPI_DEV_BB_ALT_MODE_STATUS) != status)
    {
        /* Change BB Alt mode status */
        /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
         * and hence it is safe. */
        CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ALT_MODE_STATUS, &status); /* PRQA S 0315 */

        /* Send Connect BB event */
        bb_set_conn_stat(ptrAltModeContext, (uint8_t)BB_CONNECT_STAT);
    }
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    return CY_PDSTACK_STAT_SUCCESS;
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_UpdateAllStatus(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#if CCG_PD_DUALPORT_ENABLE
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if ((bb_active_port != 0xFFu) && (bb_active_port != port)) /* PRQA S 3415 */
    {
        return CY_PDSTACK_STAT_BUSY;
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Allow update of alternate mode only if initialized and not in flashing */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].state == BB_STATE_FLASHING))
    {
        return CY_PDSTACK_STAT_NOT_READY;
    }

    gl_bb[port].alt_status = status;

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    /* Check if status was changed from the previous time */
    if (CALL_MAP(hpi_bb_get_reg)((uint8_t)HPI_DEV_BB_ALT_MODE_STATUS) != status)
    {
        /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
         * and hence it is safe. */
        CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_ALT_MODE_STATUS, &status); /* PRQA S 0315 */
    }
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    return CY_PDSTACK_STAT_SUCCESS;
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_Disable(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool force)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#if CCG_PD_DUALPORT_ENABLE
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if ((bb_active_port != 0xFFu) && (bb_active_port != port)) /* PRQA S 3415 */
    {
        return CY_PDSTACK_STAT_BUSY;
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Allow disable during flashing only if force parameter is true. */
    if (((force != false) && (gl_bb[port].state <= BB_STATE_DISABLED)) ||
            ((force == false) && (gl_bb[port].state > BB_STATE_BILLBOARD)))
    {
        return CY_PDSTACK_STAT_NOT_READY;
    }

    /* Check if BB is already disabled */
    if(gl_bb[port].state != BB_STATE_DISABLED)
    {
        gl_bb[port].state = BB_STATE_DISABLED;
        /* Send Disconnect BB event */
        bb_set_conn_stat(ptrAltModeContext, BB_DISCONNECT_STAT);
    }

#if CCG_PD_DUALPORT_ENABLE
    bb_active_port = 0xFF;
#endif /* CCG_PD_DUALPORT_ENABLE */

    return CY_PDSTACK_STAT_SUCCESS;
}

bool Cy_PdAltMode_Billboard_IsPresent(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;
    if(gl_bb[port].state == BB_STATE_DEINITED)
    {
        return false;
    }

    return true;
}

bool Cy_PdAltMode_Billboard_IsEnabled(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    uint8_t port = ptrAltModeContext->pdStackContext->port;
    if (
            (gl_bb[port].state == BB_STATE_BILLBOARD) ||
            (gl_bb[port].state == BB_STATE_FLASHING)
       )
    {
        return true;
    }

    return false;
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_FlashingCtrl(cy_stc_pdaltmode_context_t *ptrAltModeContext, bool enable)
{
    (void)ptrAltModeContext;
    (void)enable;

    /* Nothing to do as external Billboard flashing is not supported. */
    return CY_PDSTACK_STAT_SUCCESS;
}

bool Cy_PdAltMode_Billboard_IsIdle(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    (void)ptrAltModeContext;
    return true;
    /* Dummy function as there are no billboard tasks. */
}

bool Cy_PdAltMode_Billboard_EnterDeepSleep(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    (void)ptrAltModeContext;

    /* Sleep checks are handled in HPI layer. */
    return true;
}

void Cy_PdAltMode_Billboard_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    /* Dummy function as there are no billboard tasks. */
    (void)ptrAltModeContext;

    return;
}

void Cy_PdAltMode_Billboard_UpdateSelfPwrStatus (cy_stc_pdaltmode_context_t *ptrAltModeContext, uint8_t self_pwrd)
{
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    uint8_t port = ptrAltModeContext->pdStackContext->port;
#if CCG_PD_DUALPORT_ENABLE
    /* QAC suppression 3415: The checks on the right hand side of logical operator 
     * are readonly / only required if the check on left hand side is not sufficient to 
     * decide the result of overall expression. Hence, it is not a side effect. */
    if ((bb_active_port == 0xFFu) || (bb_active_port == port)) /* PRQA S 3415 */
#endif /* CCG_PD_DUALPORT_ENABLE */
    {
        /* QAC suppression 0315: This function uses the pointer after converting to uint8_t *
         * and hence it is safe. */
        CALL_MAP(hpi_bb_reg_update)((uint8_t)HPI_DEV_BB_MISC_INFO, &self_pwrd); /* PRQA S 0315 */
    }
#else
    (void)ptrAltModeContext;
    (void)self_pwrd;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */
}

uint8_t *Cy_PdAltMode_Billboard_GetVersion(void)
{
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    return (CALL_MAP(hpi_bb_get_version)());
#else
    /* No version information available. */
    return NULL;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */
}

cy_en_pdstack_status_t Cy_PdAltMode_Billboard_BindToPort (cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    cy_en_pdstack_status_t ret = CY_PDSTACK_STAT_BUSY;
    uint8_t port = ptrAltModeContext->pdStackContext->port;

#if CCG_PD_DUALPORT_ENABLE
    uint8_t intstate = CyEnterCriticalSection ();

    /* Return error if bb_init() has not been called or has failed. */
    if (gl_bb[port].state == BB_STATE_DEINITED)
    {
        ret = CY_PDSTACK_STAT_FAILURE;
    }
    else
    {
        /* QAC suppression 3415: The checks on the right hand side of logical operator 
         * are readonly / only required if the check on left hand side is not sufficient to 
         * decide the result of overall expression. Hence, it is not a side effect. */
        if ((bb_active_port == 0xFFu) || (bb_active_port == port)) /* PRQA S 3415 */
        {
            bb_active_port = port;
            ret = CY_PDSTACK_STAT_SUCCESS;
        }
    }

    CyExitCriticalSection (intstate);

#else /* !CCG_PD_DUALPORT_ENABLE */
    ret = CY_PDSTACK_STAT_SUCCESS;
#endif /* CCG_PD_DUALPORT_ENABLE */

    return (ret);
}

#endif /* (CCG_BB_ENABLE != 0) */

/* [] END OF FILE */
