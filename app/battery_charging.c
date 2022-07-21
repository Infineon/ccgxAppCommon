/***************************************************************************//**
* \file battery_charging.c
* \version 1.1.0 
*
* This is the Battery Charging source file
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <psource.h>
#if (CY_PD_SINK_ONLY)
#include <psink.h>
#endif /* CY_PD_SINK_ONLY */
#include  "cy_sw_timer.h"
#include <app.h>
#include "cy_pdstack_common.h"
#include <battery_charging.h>
#include "cy_pdstack_dpm.h"
#include "cy_sw_timer_id.h"
#include "cy_usbpd_bch.h"
#include "cy_pdstack_utils.h"
#include "cy_usbpd_mux.h"
#include "cy_usbpd_idac_ctrl.h"
#include "config.h"

#if BCR
#include <gpio.h>
#include <bcr.h>
#include <hpi.h>
#endif

#if CCG_TYPE_A_PORT_ENABLE
#include <type_a.h>
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if ((defined CY_DEVICE_CCG5) || (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6))
#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */
#endif /* ((defined CY_DEVICE_CCG5) || (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6)) */



#if BATTERY_CHARGING_ENABLE

#if defined(BCR) || BCR || QC_AFC_SNK_EN
static bool bc_mismatch_check(cy_stc_pdstack_context_t * context);
#endif /* defined(BCR) || BCR || QC_AFC_SNK_EN */

#define CY_PD_EXT_SRCCAP_PDP_LENGTH         (2)
#define HPI_EVENT_BC_12_EVENTS             (0xC4)    
/**
 * @typedef cy_en_pdstack_bc_fsm_evt_t
 * @brief Enum to hold BC events id
 */
typedef enum
{
    BC_FSM_EVT_ENTRY = 0,           /*  0: BC Event: State entry. */
    BC_FSM_EVT_CMP1_FIRE,           /*  1: BC Event: CMP1 interrupt. */
    BC_FSM_EVT_CMP2_FIRE,           /*  2: BC Event: CMP2 interrupt. */
    BC_FSM_EVT_QC_CHANGE,           /*  3: BC Event: QC state change. */
    BC_FSM_EVT_QC_CONT,             /*  4: BC Event: QC continuous mode entry. */
    BC_FSM_EVT_AFC_RESET_RCVD,      /*  5: BC Event: AFC reset received. */
    BC_FSM_EVT_AFC_MSG_RCVD,        /*  6: BC Event: AFC message received. */
    BC_FSM_EVT_AFC_MSG_SENT,        /*  7: BC Event: AFC message sent. */
    BC_FSM_EVT_AFC_MSG_SEND_FAIL,   /*  8: BC Event: AFC message sending failed. */
    BC_FSM_EVT_TIMEOUT1,            /*  9: BC Event: Timer1 expiry interrupt. */
    BC_FSM_EVT_TIMEOUT2,            /* 10: BC Event: Timer2 expiry interrupt. */
    BC_FSM_EVT_DISCONNECT,          /* 11: BC Event: Device disconnect. */
    BC_FSM_MAX_EVTS                 /* 12: Number of events. */
}cy_en_pdstack_bc_fsm_evt_t;

extern app_sln_handler_t *solution_fn_handler;

#if (!APPLE_SOURCE_DISABLE)
static const uint16_t apple_id_to_cur_map[] = {
    APPLE_AMP_1A,
    APPLE_AMP_2_1A,
    APPLE_AMP_2_4A,
    APPLE_AMP_3A    
};
#endif /* (!APPLE_SOURCE_DISABLE) */

/* Battery Charger Configuration structure*/
static bc_status_t gl_bc_status[NO_OF_BC_PORTS];

#if CCG_HPI_BC_12_ENABLE
    /* Battery 12 current status */
    static uint8_t gl_bc_12_status[NO_OF_BC_PORTS] = { 0u
#if CCG_PD_DUALPORT_ENABLE        
        ,
        0u                        
#endif /* CCG_PD_DUALPORT_ENABLE */       
    };
#endif /* CCG_HPI_BC_12_ENABLE */

#if LEGACY_DYN_CFG_ENABLE
static cy_stc_legacy_charging_cfg_t gl_bc_cfg[NO_OF_BC_PORTS];
#endif /* LEGACY_DYN_CFG_ENABLE */

#if (!defined(CY_DEVICE_CCG5))

/* Function prototypes*/

/* Callback from BC Phy */
static void bc_phy_cbk_handler(void *context, uint32_t event);

/* Callback from timer module */
static void bc_tmr_cbk(cy_timer_id_t id, void * callbackCtx);

/* Callback from power module */
static void bc_pwr_ready_cbk(cy_stc_pdstack_context_t *ptrPdStackContext);

/* BC FSM prototypes */
static void bc_fsm_off(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);

#if (!CY_PD_SINK_ONLY)
/* Handlers for SRC mode operation. */
static void bc_fsm_src_look_for_connect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_src_initial_connect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_src_apple_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
#endif /* (!CY_PD_SINK_ONLY) */

#if (defined (CY_DEVICE_CCG5C) || defined (CY_DEVICE_CCG6))
/* BC 1.2 CDP can only be supported on CCG5C and CCG6. */
static void bc_fsm_src_apply_cdp(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static bool vdm_src_applied[NO_OF_BC_PORTS];
static bool detach_detect_en[NO_OF_BC_PORTS];
#endif /* (defined (CY_DEVICE_CCG5C) || defined (CY_DEVICE_CCG6)) */

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
static void bc_fsm_src_others_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_src_qc_or_afc(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_src_qc_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_src_afc_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
#endif /* (!QC_AFC_CHARGING_DISABLED) */

/* Handlers for Sink mode operation. */
#if (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
static void bc_fsm_sink_start(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_apple_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_apple_brick_id_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_primary_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_type_c_only_source_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_secondary_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_dcp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_sdp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_cdp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
#if BCR || QC_AFC_SNK_EN
static void bc_fsm_sink_afc_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
static void bc_fsm_sink_qc_charger_detected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt);
#endif /* BCR || QC_AFC_SNK_EN */
#endif /* (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */

static void (*const gl_bc_fsm_table [BC_FSM_MAX_STATES]) (cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt) =
{
    bc_fsm_off,                                 /*  0: BC_FSM_OFF */
#if (!(CY_PD_SINK_ONLY))
    bc_fsm_src_look_for_connect,                /*  1: BC_FSM_SRC_LOOK_FOR_CONNECT */
    bc_fsm_src_initial_connect,                 /*  2: BC_FSM_SRC_INITIAL_CONNECT */
    bc_fsm_src_apple_connected,                 /*  3: BC_FSM_SRC_APPLE_CONNECTED */
#else
    bc_fsm_off,                                 /*  1: BC_FSM_SRC_LOOK_FOR_CONNECT */
    bc_fsm_off,                                 /*  2: BC_FSM_SRC_INITIAL_CONNECT */
    bc_fsm_off,                                 /*  3: BC_FSM_SRC_APPLE_CONNECTED */
#endif /* (!(CY_PD_SINK_ONLY)) */

#if (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6)
    bc_fsm_src_apply_cdp,                       /*  4: BC_FSM_SRC_CDP_CONNECTED */
#else
    bc_fsm_off,                                 /*  4: BC_FSM_SRC_CDP_CONNECTED */
#endif /* (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6) */

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
    bc_fsm_src_others_connected,                /*  5: BC_FSM_SRC_OTHERS_CONNECTED */
    bc_fsm_src_qc_or_afc,                       /*  6: BC_FSM_SRC_QC_OR_AFC */
    bc_fsm_src_qc_connected,                    /*  7: BC_FSM_SRC_QC_CONNECTED */
    bc_fsm_src_afc_connected,                   /*  8: BC_FSM_SRC_AFC_CONNECTED */
#else /* QC_AFC_CHARGING_DISABLED */
    bc_fsm_off,                                 /*  5: BC_FSM_SRC_OTHERS_CONNECTED */
    bc_fsm_off,                                 /*  6: BC_FSM_SRC_QC_OR_AFC */
    bc_fsm_off,                                 /*  7: BC_FSM_SRC_QC_CONNECTED */
    bc_fsm_off,                                 /*  8: BC_FSM_SRC_AFC_CONNECTED */
#endif /* (!QC_AFC_CHARGING_DISABLED) */

#if (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
    bc_fsm_sink_start,                          /*  9: BC_FSM_SINK_START */
    bc_fsm_sink_apple_charger_detect,           /* 10: BC_FSM_SINK_APPLE_CHARGER_DETECT */
    bc_fsm_sink_apple_brick_id_detect,          /* 11: BC_FSM_SINK_APPLE_BRICK_ID_DETECT */
    bc_fsm_sink_primary_charger_detect,         /* 12: BC_FSM_SINK_PRIMARY_CHARGER_DETECT */
    bc_fsm_sink_type_c_only_source_connected,   /* 13: BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED */
    bc_fsm_sink_secondary_charger_detect,       /* 14: BC_FSM_SINK_SECONDARY_CHARGER_DETECT */
    bc_fsm_sink_dcp_connected,                  /* 15: BC_FSM_SINK_DCP_CONNECTED */
    bc_fsm_sink_sdp_connected,                  /* 16: BC_FSM_SINK_SDP_CONNECTED */
    bc_fsm_sink_cdp_connected,                  /* 17: BC_FSM_SINK_CDP_CONNECTED */
#if BCR || QC_AFC_SNK_EN
    bc_fsm_sink_afc_charger_detect,             /* 18: BC_FSM_SINK_AFC_CHARGER_DETECT */
    bc_fsm_sink_qc_charger_detected             /* 19: BC_FSM_SINK_QC_CHARGER_DETECT */
#else
    bc_fsm_off,                                 /* 18: BC_FSM_SINK_AFC_CHARGER_DETECT */
    bc_fsm_off                                  /* 19: BC_FSM_SINK_QC_CHARGER_DETECT */
#endif /* BCR || QC_AFC_SNK_EN */
#endif /* (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */
};
#endif /* (!defined(CY_DEVICE_CCG5)) */

static void bc_phy_cbk_handler(void *callbackCtx, uint32_t event)
{
    cy_stc_usbpd_context_t *usbpdcontext=callbackCtx;
    cy_stc_pdstack_context_t *context = (cy_stc_pdstack_context_t *)usbpdcontext->pdStackContext;

    bc_set_bc_evt(context, event);
}

static bool bc_mismatch_check(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport]; 
    
    /* Clear mismatch status from previously tried BC protocol */ 
    bc_stat->mismatch = false;
    /* Read Sink voltage */
    uint16_t cur_volt = app_get_status(cport)->psnk_volt;
    /* Read Sink current */
    uint16_t cur_amp = app_get_status(cport)->psnk_cur * CY_PD_CUR_PER_UNIT;
    
    /* Mismatch Check  only for voltage range. Current can be any supported value. */
    if((cur_volt < bc_stat->min_volt) || (cur_volt > bc_stat->max_volt))
    {
        bc_stat->mismatch = true;
    } 
    
    return bc_stat->mismatch;
}

bool bc_is_bc_connected(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    return bc_stat->connected;
}

#if BCR || QC_AFC_SNK_EN
bool bc_psnk_enable(cy_stc_pdstack_context_t * context)
{
    uint8_t c_port=context->port;
    bc_status_t *bc_stat = &gl_bc_status[c_port];
    if(bc_mismatch_check(context) == false)
    {
        bc_stat->connected = true;
        app_event_handler(context,APP_EVT_BC_DETECTION_COMPLETED, NULL);
        return true;
    }
    else
    {
        bc_stop(context);
        return false;
    }
}
#endif /* BCR || QC_AFC_SNK_EN */

#if BCR || QC_AFC_SNK_EN
static void bc_set_mode(cy_stc_pdstack_context_t * context, uint8_t bc_snk_mode)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    bc_stat->cur_mode = bc_snk_mode;
#if (CCG_HPI_ENABLE && BCR)
    hpi_set_bc_snk_status (cport, bc_stat->cur_mode);
#endif /* CCG_HPI_ENABLE && BCR */
}
#endif /* BCR || QC_AFC_SNK_EN */

#if (!defined(CY_DEVICE_CCG5))
static void bc_tmr_cbk(cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    if(id == ((cy_timer_id_t)APP_BC_GENERIC_TIMER1))
    {
        bc_set_bc_evt(context, BC_EVT_TIMEOUT1);
    }
    else if(id == ((cy_timer_id_t)APP_BC_GENERIC_TIMER2))
    {
        bc_set_bc_evt(context, BC_EVT_TIMEOUT2);
    }
    else
    {
        /* No Statement */
    }
}

static void bc_pwr_ready_cbk(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (!QC_SRC_AFC_CHARGING_DISABLED)
    /* Do nothing */
    CY_UNUSED_PARAMETER(ptrPdStackContext);
#else
    /* Do nothing */
    CY_UNUSED_PARAMETER(ptrPdStackContext);
#endif /* (!QC_AFC_CHARGING_DISABLED)*/
}
#endif /* (!defined(CY_DEVICE_CCG5)) */

cy_en_pdstack_status_t bc_init(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];

    (void)Cy_USBPD_Bch_Phy_Init(context->ptrUsbPdContext, bc_phy_cbk_handler);
    bc_stat->bc_fsm_state = BC_FSM_OFF;

#if LEGACY_DYN_CFG_ENABLE
    /* Copy the configuration from the configuration table. */
    mem_copy((uint8_t *)(&gl_bc_cfg[cport]), (uint8_t *)pd_get_ptr_chg_cfg_tbl(context->ptrUsbPdContext),
            sizeof (cy_stc_legacy_charging_cfg_t));
#endif /* LEGACY_DYN_CFG_ENABLE */

#if QC_AFC_SNK_EN
    bc_stat->max_volt = BC_QC_AFC_SNK_MAX_VOLT;
    bc_stat->min_volt = BC_QC_AFC_SNK_MIN_VOLT;
    bc_stat->max_amp = BC_QC_AFC_SNK_MAX_CUR;
    bc_stat->min_amp = BC_QC_AFC_SNK_MIN_CUR;
#endif /* QC_AFC_SNK_EN */

    return CY_PDSTACK_STAT_SUCCESS;
}

const cy_stc_legacy_charging_cfg_t *bc_get_config(cy_stc_pdstack_context_t *context)
{
#if LEGACY_DYN_CFG_ENABLE
    return((const cy_stc_legacy_charging_cfg_t *)&gl_bc_cfg[context->port]);
#else /* !LEGACY_DYN_CFG_ENABLE */
#if CY_USE_CONFIG_TABLE
    return(pd_get_ptr_chg_cfg_tbl(context->ptrUsbPdContext));
#else
    return (context->ptrUsbPdContext->usbpdConfig->legacyChargingConfig);
#endif /* CY_USE_CONFIG_TABLE */
#endif /* LEGACY_DYN_CFG_ENABLE */
}

bool bc_is_active(cy_stc_pdstack_context_t *context)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];

    if (bc_stat->bc_fsm_state == BC_FSM_OFF)
    {
        return false;
    }

    return true;
}

#if LEGACY_DYN_CFG_ENABLE
cy_en_pdstack_status_t bc_set_config(cy_stc_pdstack_context_t *context, cy_stc_legacy_charging_cfg_t *chg_cfg)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];

    if ((chg_cfg != NULL) && (bc_stat->bc_fsm_state == BC_FSM_OFF))
    {
        mem_copy((uint8_t *)(&gl_bc_cfg[cport]), (uint8_t *)chg_cfg,
                sizeof (cy_stc_legacy_charging_cfg_t));

        return CY_PDSTACK_STAT_SUCCESS;
    }
    
    return CY_PDSTACK_STAT_FAILURE;
}

void bc_set_snk_legacy_mode(cy_stc_pdstack_context_t *context, uint8_t bc_snk_mode)
{
    uint8_t cport=context->port;
    gl_bc_cfg[cport].snkSel = bc_snk_mode;
}
#endif /* LEGACY_DYN_CFG_ENABLE */

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
static uint8_t bc_afc_src_get_vi_count(cy_stc_pdstack_context_t * context)
{
    return bc_get_config(context)->afcSrcCapCnt;
}

static uint8_t* bc_afc_src_get_vi_ptr(cy_stc_pdstack_context_t * context)
{
    return (uint8_t *)((uint32_t)((const uint8_t*)bc_get_config(context)->afcSrcCaps));
}

#if (BCR == 0) && !QC_AFC_SNK_EN
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
uint8_t ccg_get_system_max_pdp(cy_stc_pdstack_context_t * context)
{
    /* Function returns the max PDP value of the system 
     * This information will be used by the current limit function
     */
    uint8_t sys_max_pdp = CCG_MAX_PDP_VALUE;
    if(CY_PD_EXT_SRCCAP_PDP_LENGTH != 0u)
    {
        /* If PD3.0 is supported, the PDP value will be read from the extended source cap */
        cy_stc_pdstack_dpm_ext_status_t* ptrDpmExtStat = &(context->dpmExtStat);
        sys_max_pdp = ptrDpmExtStat->extSrcCap[CY_PD_EXT_SRCCAP_PDP_INDEX];
    }
    else
    {
        /* If not PD3.0 we will use the PDP value configured */    
    }
    return sys_max_pdp;
}
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */

#if LEGACY_DYN_CFG_ENABLE
    
/* This function forms the new VI code based on the port PDP */
void bc_afc_form_vi(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    uint8_t new_vi;
    uint8_t i;
    uint8_t port_pdp = ccg_get_system_max_pdp(context);
    uint32_t new_current;

    for(i = 0; i < gl_bc_cfg[cport].afcSrcCapCnt; i++)
    {
        new_current = CY_USBPD_GET_MIN((((uint32_t)port_pdp * 1000u)/(((uint32_t)AFC_BASE_VOLT/AFC_VOLT_STEP) + (uint32_t)BYTE_GET_UPPER_NIBBLE(gl_bc_cfg[cport].afcSrcCaps[i]))),
                                            ((uint32_t)LEGACY_MAX_CABLE_RATING * 10u));
        /* Convert current to 10mA units */
        new_current = new_current / 10u;
        if(new_current < AFC_BASE_AMP)
        {
            new_current = AFC_BASE_AMP;
        }
        new_vi = (uint8_t)CY_USBPD_GET_MIN(((new_current - AFC_BASE_AMP)/AFC_AMP_STEP), AFC_MAX_AMP);

        /* QAC suppression 2985: CUrrently this is a redundant operation as AFC_MAX_AMP is 0x0F. Yet
         * this operation is retained as the compile time setting of AFC_MAX_AMP may change in future/
         * other application. */
        new_vi &= 0x0Fu; /* PRQA S 2985 */
        gl_bc_cfg[cport].afcSrcCaps[i] = ((gl_bc_cfg[cport].afcSrcCaps[i] & 0xF0u) | (new_vi & 0x0Fu));
    }
}
#endif /* LEGACY_DYN_CFG_ENABLE */
#endif /* (BCR == 0) && !QC_AFC_SNK_EN */
#endif /* (!QC_SRC_AFC_CHARGING_DISABLED) */

cy_en_pdstack_status_t bc_start(cy_stc_pdstack_context_t * context, bc_port_role_t port_role)
{
    uint8_t cport=context->port;
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
#if ((defined CY_DEVICE_CCG3PA) || (defined CY_DEVICE_CCG3PA2) || (defined CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_PAG1S) || (defined(CY_DEVICE_CCG7D)) || (defined(CY_DEVICE_CCG7S)) || (defined(CY_DEVICE_WLC1)))
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
#if (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN))
    /* For CDP only mode of operation, we will check if BC1.2 has been enabled in configuration table */
    if((chg_cfg->srcSel & BC_SRC_1_2_MODE_ENABLE_MASK) != 0u)
    {
        app_bc_12_sm_start(context);
    }
    return CY_PDSTACK_STAT_SUCCESS;
#endif /* (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN))  */

#if (!BCR) && !QC_AFC_SNK_EN
    if (((chg_cfg->srcSel & BATT_CHARGING_SRC_MASK) == 0u) &&
            ((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) == 0u))
    {
        return CY_PDSTACK_STAT_SUCCESS;
    }
#else

#if (!(CY_PD_SINK_ONLY))
    if ((chg_cfg->srcSel & BATT_CHARGING_SRC_MASK) == 0u)
    {
        return CY_PDSTACK_STAT_SUCCESS;
    }
#endif /* (!(CY_PD_SINK_ONLY)) */

#if (!(CY_PD_SOURCE_ONLY))
    if (((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) == 0u)||
         (app_get_status(cport)->bc_snk_disabled))
    {
        return CY_PDSTACK_STAT_SUCCESS;
    }
#endif /* (!(CY_PD_SOURCE_ONLY)) */
#endif /* (!BCR) && !QC_AFC_SNK_EN */

    bc_stat->connected = false;
#if (!BCR) && !QC_AFC_SNK_EN
    bc_stat->cur_mode = BC_CHARGE_NONE;
#else
    bc_set_mode(context, BC_CHARGE_NONE);
#endif /* (!BCR) && !QC_AFC_SNK_EN */

#if (!(CY_PD_SINK_ONLY))
#if (!(CY_PD_SOURCE_ONLY))
    if (port_role == BC_PORT_SOURCE)
#endif /* (!(CY_PD_SOURCE_ONLY)) */
    {
        /* Start only if we are source and BC is not disabled */
#if ((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C)))
        if (
                (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SOURCE)
                && (!app_get_status(cport)->bc_12_src_disabled)
           )
#endif /* ((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C))) */
        {
#if (((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C))) && (CCG_HPI_ENABLE))
            if (hpi_get_sys_pwr_state () == NB_SYS_PWR_STATE_S0)
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_CDP_CONNECTED;
            }
            else
#endif /* (((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C))) && (CCG_HPI_ENABLE)) */
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            }

            bc_stat->bc_evt = BC_EVT_ENTRY;
        }
    }
#endif /* (!(CY_PD_SINK_ONLY)) */

#if (!(CY_PD_SOURCE_ONLY))
#if (!(CY_PD_SINK_ONLY))
    else
#endif
#if (!BCR) && !QC_AFC_SNK_EN
        if ((!app_get_status(cport)->bc_12_snk_disabled))
#endif
        {
            /* Move to start state for sink mode operation. */
            bc_stat->bc_fsm_state = BC_FSM_SINK_START;
            bc_stat->bc_evt = BC_EVT_ENTRY;
        }
#endif /* (!(CY_PD_SOURCE_ONLY)) */
    CY_UNUSED_PARAMETER(ptrDpmConfig);
    CY_UNUSED_PARAMETER(port_role);
    return CY_PDSTACK_STAT_SUCCESS;

#elif (defined(CY_DEVICE_CCG5))
    /* Start only if we are source and BC is not disabled */
    if (
            (ptrDpmConfig->curPortRole== CY_PD_PRT_ROLE_SOURCE) &&
            (!app_get_status(cport)->bc_12_src_disabled)
       )
    {
        app_bc_12_sm_start(context);
    }
    return CY_PDSTACK_STAT_SUCCESS;
#else
    return CY_PDSTACK_STAT_SUCCESS;
#endif /* CCGx */
}

cy_en_pdstack_status_t bc_stop(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    (void)cport;
#if (defined(CY_DEVICE_CCG3PA)) || (defined(CY_DEVICE_CCG3PA2)) || (defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))

#if (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN))
    Cy_USBPD_Bch_BcDis(context->ptrUsbPdContext);
    return CY_PDSTACK_STAT_SUCCESS;
#else

    bc_status_t* bc_stat = &gl_bc_status[cport];

    /* Do nothing if we are already off. */
    if (bc_stat->bc_fsm_state == BC_FSM_OFF)
    {
        return CY_PDSTACK_STAT_SUCCESS;
    }

#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
    /* Disable external Apple source termination. */
    sln_remove_apple_src_term(cport);
#endif /* (LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */

    (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
    cy_sw_timer_stop_range(context->ptrTimerContext,APP_BC_GENERIC_TIMER1,APP_CDP_DP_DM_POLL_TIMER);
    (void)Cy_USBPD_Bch_Phy_RemoveTerm(context->ptrUsbPdContext);
    (void)Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);
    bc_stat->bc_fsm_state = BC_FSM_OFF;
    bc_stat->bc_evt = 0;
    bc_stat->connected = false;
    bc_stat->attach = false;
#if (BCR) || QC_AFC_SNK_EN
    bc_stat->mismatch = false;
    /* Clear sink specific flags and states. */
    bc_stat->cur_volt = CY_PD_VSAFE_0V;
    bc_set_mode(context, BC_CHARGE_NONE);
#else
    bc_stat->cur_mode = BC_CHARGE_NONE;
#endif /* (BCR) || QC_AFC_SNK_EN */
#if CCG_HPI_BC_12_ENABLE
    gl_bc_12_status[cport] = 0x00u;
#endif /* CCG_HPI_BC_12_ENABLE */
#if BCR || QC_AFC_SNK_EN
    app_event_handler(context, APP_EVT_BC_DETECTION_COMPLETED, NULL);
#endif

#if ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
    detach_detect_en[cport] = false;
#endif /* ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6))) */

    /* Clear sink specific flags and states. */
    bc_stat->cur_volt = CY_PD_VSAFE_0V;

#if (((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C))) && (!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    if (ptrDpmConfig->curPortRole ==CY_PD_PRT_ROLE_SINK)
    {
        /* If mode is sink and there is no PD contract, ensure current limit is set to minimum. */
        if (!ptrDpmConfig->contractExist)
        {
            psnk_set_current(context, CY_PD_ISAFE_0A);
        }

        /* Enable DP/DM Mux if we are sink and still attached. */
        if (ptrDpmConfig->attach)
        {
            Cy_USBPD_Mux_ConfigDpDm(context->ptrUsbPdContext,
                    ptrDpmConfig->polarity ? CY_USBPD_DPDM_MUX_CONN_USB_BOT : CY_USBPD_DPDM_MUX_CONN_USB_TOP);
        }
    }
#endif /* (((defined(CY_DEVICE_CCG6)) || (defined(CY_DEVICE_CCG5C))) && (!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
    return CY_PDSTACK_STAT_SUCCESS;
#endif /* (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN) */

#elif (defined(CY_DEVICE_CCG5))
    ccg_bc_dis(cport);
    return CY_PDSTACK_STAT_SUCCESS;

#else
    return CY_PDSTACK_STAT_SUCCESS;
#endif /* CCGx */
}

bool bc_port_is_cdp(cy_stc_pdstack_context_t * context)
{

    bool ret = false;

#ifdef CY_DEVICE_CCG5
    uint8_t cport=context->port;
    if (ccg_bc_is_cdp(cport))
    {
        ret = true;
    }
#endif /* CY_DEVICE_CCG5 */

#if ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
    uint8_t cport=context->port;
    if (gl_bc_status[cport].bc_fsm_state == BC_FSM_SRC_CDP_CONNECTED)
    {
        ret = true;
    }
#else
    CY_UNUSED_PARAMETER(context);
#endif /* ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6))) */

    return ret;
}

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
void bc_debounce(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    uint32_t i;
    bc_dp_dm_state_t new_state;
    cy_en_usbpd_bch_comp_pinput_t pinput = CHGB_COMP_P_DP;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);

    new_state.state = (uint16_t)QC_MODE_RSVD;

    /* Get current status */
    for(i = 0 ; i < 2u ; i++)
    {
        if(i == 1u)
        {
            pinput = CHGB_COMP_P_DM;
        }

        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, pinput, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_325V, CHGB_COMP_NO_INTR) == false)
        {
            new_state.d[i] = (uint8_t)BC_D_GND;
        }
        else if(Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext,BC_CMP_0_IDX, pinput, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_NO_INTR) == false)
        {
            new_state.d[i] = (uint8_t)BC_D_0_6V;
        }
        else
        {
            new_state.d[i] = (uint8_t)BC_D_3_3V;
        }
    }

    /* Do debounce */
    if(bc_stat->dp_dm_status.state == new_state.state)
    {
        bc_stat->old_dp_dm_status.state = bc_stat->dp_dm_status.state;
        cy_sw_timer_stop(context->ptrTimerContext, (uint8_t)APP_BC_DP_DM_DEBOUNCE_TIMER);
        return;
    }

    if(bc_stat->old_dp_dm_status.state != new_state.state)
    {
        /*
         * Do not debounce DP/DM state if current mode is QC Continuous mode and new state
         * translates to non-5V Fixed mode. Only transition out of continuous mode is either
         * 5V fixed mode or disconnect.
         */
        if ((bc_stat->dp_dm_status.state != QC_MODE_CONT) || (new_state.state == QC_MODE_5V) ||
            (new_state.state == 0u))
        {
            (void)cy_sw_timer_start(context->ptrTimerContext,context,(uint8_t)APP_BC_DP_DM_DEBOUNCE_TIMER, APP_BC_DP_DM_DEBOUNCE_TIMER_PERIOD, NULL);
            bc_stat->old_dp_dm_status.state = new_state.state;
            return;
        }
        if(bc_stat->dp_dm_status.state == QC_MODE_CONT)
        {
            return;
        }
    }

    if(cy_sw_timer_is_running(context->ptrTimerContext, (uint8_t)APP_BC_DP_DM_DEBOUNCE_TIMER) == false)
    {
        bc_stat->dp_dm_status.state = bc_stat->old_dp_dm_status.state;
        /*
         * If both DP and DM are Hi-Z, generate CMP1_FIRE interrupt which translates
         * to device disconnect event. Otherwise, generate QC Mode change interrupt.
         */
        if (bc_stat->dp_dm_status.state == 0u)
        {
            bc_set_bc_evt(context, BC_EVT_DISCONNECT);
        }
        else
        {
            /* Proceed with QC state change only if QC is enabled. */
            if ((chg_cfg->srcSel & BC_SRC_QC_MODE_ENABLE_MASK) != 0u)
            {
                bc_set_bc_evt(context, BC_EVT_QC_CHANGE);
            }
        }
    }
}
#endif /* (!QC_AFC_CHARGING_DISABLED) */

cy_en_pdstack_status_t bc_fsm(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
#if (defined CY_DEVICE_CCG5)
    /* Run the CDP state machine where required. */
    ccg_bc_cdp_sm(cport);
    return CY_PDSTACK_STAT_SUCCESS;
#else

    bc_status_t* bc_stat = &gl_bc_status[cport];
    uint8_t evt;
    
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))

#if (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN))
    (void)Cy_USBPD_Bch_CdpSm(context->ptrUsbPdContext);
    return CY_PDSTACK_STAT_SUCCESS;
#endif /* (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN)) */
    /* Execute state machine only if the port is active. */
#if (CCG_TYPE_A_PORT_ENABLE == 1)
    if(cport == TYPEC_PORT_0_IDX)
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
    {
        if (ptrDpmConfig->connect == false)
        {
            (void)bc_stop(context);
            return CY_PDSTACK_STAT_SUCCESS;
        }
    }
#if (CCG_TYPE_A_PORT_ENABLE == 1)
    else
    {
        if (type_a_get_status()->type_a_enabled == false)
        {
            return CY_PDSTACK_STAT_SUCCESS;
        }
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1)) */

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
    if((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) || (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED)
        || (bc_stat->bc_fsm_state == BC_FSM_SRC_AFC_CONNECTED))
    {
        bc_debounce(context);
    }
#endif /* (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED)) */

    /* Get bc event */
    evt = event_group_get_event((volatile uint32_t *)&(bc_stat->bc_evt), true);

    /* Check if any valid event pending, if not return */
    if(evt < ((uint8_t)BC_FSM_MAX_EVTS))
    {
        gl_bc_fsm_table[bc_stat->bc_fsm_state] (context, (cy_en_pdstack_bc_fsm_evt_t)evt);
    }
#endif /* (defined CY_DEVICE_CCG5) */

    return CY_PDSTACK_STAT_SUCCESS;
}

bool bc_sleep(cy_stc_pdstack_context_t * context)
{
#if (defined CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))

#if (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN))

    uint8_t port;
    for(port = 0; port < NO_OF_BC_PORTS; port++)
    {
        if (Cy_USBPD_Bch_Is_Cdp_SmBusy(context->ptrUsbPdContext))
        {
                return false;
        }
    }
    
#else
    uint8_t i;
#if (!BCR) && !QC_AFC_SNK_EN
    bool timer_range_enabled_flag;
#endif /* (!BCR) && !QC_AFC_SNK_EN */

#if (!QC_AFC_CHARGING_DISABLED)
    bool chgb_dp_status_flag, chgb_dm_status_flag;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];
        cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);

        /* Execute state machine only if the port is active. */
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if(i == TYPEC_PORT_0_IDX)
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
        {
            if (ptrDpmConfig->connect == false)
            {
                continue;
            }
        }
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        else
        {
            if (type_a_get_status()->type_a_enabled == false)
            {
                continue;
            }
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
#if(!BCR) && !QC_AFC_SNK_EN
        timer_range_enabled_flag = cy_sw_timer_range_enabled(pdstack_ctx->ptrTimerContext, (uint8_t)APP_BC_GENERIC_TIMER1, (uint8_t)APP_BC_DP_DM_DEBOUNCE_TIMER);
        if ((bc_stat->bc_evt != 0u) || (timer_range_enabled_flag))
#else
        if ((bc_stat->bc_evt != 0)  ||
            (cy_sw_timer_is_running(pdstack_ctx->ptrTimerContext, APP_BC_GENERIC_TIMER1) == true) ||
            (cy_sw_timer_is_running(pdstack_ctx->ptrTimerContext, APP_BC_GENERIC_TIMER2) == true) ||
            (cy_sw_timer_is_running(pdstack_ctx->ptrTimerContext, APP_BC_DP_DM_DEBOUNCE_TIMER) == true)
           )
#endif /* (!BCR) && !QC_AFC_SNK_EN */
        {
            return false;
        }

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
        if (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED)
        {
            /* Enable master DP/DM sense */
            (void)Cy_USBPD_Bch_QcSrcMasterSenseEn(pdstack_ctx->ptrUsbPdContext);
        }
#endif /* (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED)) */

        /* Configure QC RCVR interrupt as wakeup source from DP/DM activity. */
        Cy_USBPD_Bch_Phy_Config_DeepSleep(pdstack_ctx->ptrUsbPdContext);

#if (!QC_AFC_CHARGING_DISABLED)
        /* 
         * Check the DP/DM status before going to sleep.
         * This is to make sure DP/DM debounce edges are not missed between
         * bc_fsm() and bc_sleep().
         */
        chgb_dp_status_flag = Cy_USBPD_Bch_Phy_DpStat(pdstack_ctx->ptrUsbPdContext);
        chgb_dm_status_flag = Cy_USBPD_Bch_Phy_DmStat(pdstack_ctx->ptrUsbPdContext);
        if((((uint8_t)(chgb_dp_status_flag == true) ^ (uint8_t)(bc_stat->dp_dm_status.d[0] == (uint8_t)BC_D_3_3V)) != 0u)||
           (((uint8_t)(chgb_dm_status_flag == true) ^ (uint8_t)(bc_stat->dp_dm_status.d[1] == (uint8_t)BC_D_3_3V)) != 0u))
        {
            return false;
        }
#endif /* (!QC_AFC_CHARGING_DISABLED) */
    }

    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
        bc_status_t* bc_stat = &gl_bc_status[i];

        cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);

        /*
         * Configure DP/DM Comparators to enable device wakeup in QC 2.0 mode.
         * When QC device updates QC2.0 mode, these interrupts will wake up the device.
         * QC2.0 mode request is then debounced in bc_debounce() routine.
         * Setting comparators based on the current QC2.0 mode.
         */
        if ((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) ||
            (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED))
        {
            switch (bc_stat->dp_dm_status.state)
            {
                case (uint16_t)QC_MODE_5V:
                    /* For < 0.6V transition on DP. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                    /* For > 0V transition on DM. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);
                    /* QCOM RCVR interrupt will be used for > 0.6V transition on DP. */
                    break;

                case (uint16_t)QC_MODE_9V:
                    /* For < 0.6V transition on DM. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                    /*
                     * QCOM RCVR interrupts will be used for < 3.3V transition on DP
                     * and > 0.6V transition on DM.
                     */
                    break;

                case (uint16_t)QC_MODE_12V:
                    /* For < 0.6V transition on DP. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                     /* For < 0.6V transition on DM. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                    /* QCOM RCVR interrupts will be used for > 0.6V transition on DP and DM. */
                    break;

                case (uint16_t)QC_MODE_20V:
                    /* Nothing to do here because QC RCVR interrupt will detect <3.3V
                     * transition on DP and DM. */
                    break;

                case (uint16_t)QC_MODE_CONT:
                    /* For < 0.6V transition on DP. */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(pdstack_ctx->ptrUsbPdContext,BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                    /*
                     * QCOM RCVR interrupt will be used for < 3.3V transition on DM and > 0.6V
                     * transition on DP.
                     */
                    break;

                default:
                    /* Intentionally left empty */
                    break;
            }
        }

#endif /* (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED)) */
    }

#endif /* (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN)) */

#elif (defined(CY_DEVICE_CCG5C) || (defined(CY_DEVICE_CCG6)))
    /* CY_DEVICE_CCG5 or CY_DEVICE_CCG5C*/
    uint8_t i;

    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];
#if(!BCR) && !QC_AFC_SNK_EN
        if (
                (bc_stat->bc_evt != 0) ||
                (cy_sw_timer_range_enabled (i, APP_BC_GENERIC_TIMER1, APP_BC_DETACH_DETECT_TIMER))
           )
#else
        if ((bc_stat->bc_evt != 0) ||
                (cy_sw_timer_is_running(i, APP_BC_GENERIC_TIMER1) == true) ||
                (cy_sw_timer_is_running(i, APP_BC_GENERIC_TIMER2) == true) ||
                (cy_sw_timer_is_running(i, APP_BC_DETACH_DETECT_TIMER) == true) ||
                (cy_sw_timer_is_running(i, APP_BC_DP_DM_DEBOUNCE_TIMER) == true))
#endif /* (!BCR) && !QC_AFC_SNK_EN */
        {
            return false;
        }
#if (NO_OF_BC_PORTS > 1)
    }

    for (i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];
#endif /* (NO_OF_BC_PORTS > 1) */

        if(bc_stat->cur_mode != BC_CHARGE_CDP)

        {
        cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);
            /* Configure QC RCVR interrupt as wakeup source from DP/DM activity. */
        Cy_USBPD_Bch_Phy_Config_DeepSleep(pdstack_ctx->ptrUsbPdContext);
        }
    }
#elif (defined(CY_DEVICE_CCG5))
    uint8_t i;

    for (i = 0; i < NO_OF_BC_PORTS; i++)
    {
        /* Cannot go to sleep while we have VDM_SRC enabled. */
        if (ccg_is_cdp_sm_busy (i))
        {
            return false;
        }
    }
#endif /* CCGx */
    return true;
}

#if (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6)
/* This function checks whether the BC 1.2 sink connected to the port has been removed.
 * The CDP state machine is restarted, if so.
 */
void bc_cdp_detach_cbk(cy_timer_id_t id, void * callbackCtx)
{
    cy_stc_pdstack_context_t* context = callbackCtx;
    uint8_t cport=context->port;
    (void)id;

    bc_status_t* bc_stat = &gl_bc_status[cport];

    /* Stop the counter and read the count value */
    if (chgb_get_counter_value(cport) == 0)
    {
        /* Go to look for CDP connection state */
        bc_stat->bc_fsm_state = BC_FSM_SRC_CDP_CONNECTED;
        bc_stat->bc_evt = BC_EVT_ENTRY;
    }

    /* Power down the comparators */
    chgb_disable_cdp_comparators(cport);
}
#endif /* (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6) */

bool bc_wakeup(void)
{
#if (defined(CY_DEVICE_CCG3PA)) || (defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))

#if (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CY_DEVICE_CCG_CDP_EN))
    /* CCG3PA CDP1.2 Do nothing */    
#else
    uint8_t i = 0;
#if (NO_OF_BC_PORTS > 1)
    for(i = 0; i < NO_OF_BC_PORTS; i++)
#else
    i = 0;
#endif /* (NO_OF_BC_PORTS > 1) */
    {
        cy_stc_pdstack_context_t *pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))

        /*
         * If in QC or AFC mode, we might have configured comparators to wakeup device
         * on DP/DM activity. Disable the comparators.
         */
        bc_status_t* bc_stat = &gl_bc_status[i];

        if ((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) ||
            (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED))
        {
            (void)Cy_USBPD_Bch_Phy_DisableComp(pdstack_ctx->ptrUsbPdContext, BC_CMP_0_IDX);
            bc_clear_bc_evt(pdstack_ctx, BC_EVT_CMP1_FIRE);
            (void)Cy_USBPD_Bch_Phy_DisableComp(pdstack_ctx->ptrUsbPdContext, BC_CMP_1_IDX);
            bc_clear_bc_evt(pdstack_ctx, BC_EVT_CMP2_FIRE);
        }
#endif /* (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED)) */

        /* Disable QC RCVR interrupts. */
        Cy_USBPD_Bch_Phy_Config_Wakeup(pdstack_ctx->ptrUsbPdContext);
    }
#endif /* (((defined(CY_DEVICE_CCG3PA) || (defined(CY_DEVICE_CCG3PA2)) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_WLC1))) && (CCG_CDP_EN)) */
    
#elif (defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C))
    uint8_t i = 0;
    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        cy_stc_pdstack_context_t * pdstack_ctx = solution_fn_handler->Get_PdStack_Context(i);
        Cy_USBPD_Bch_Phy_Config_Wakeup(pdstack_ctx->ptrUsbPdContext);
    }
#else /* CCG5 */
    /* Do nothing. */
#endif /* CCGx */
    return true;
}

const bc_status_t* bc_get_status(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    return &gl_bc_status[cport];
}

uint8_t bc_12_get_status(cy_stc_pdstack_context_t * context)
{
    
#if CCG_HPI_BC_12_ENABLE 
    return gl_bc_12_status[context->port];    
#else
    CY_UNUSED_PARAMETER(context);
    return 0u;
#endif /* CCG_HPI_BC_12_ENABLE */
}

void bc_pd_event_handler(cy_stc_pdstack_context_t * context,cy_en_pdstack_app_evt_t evt)
{
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
#if (!BCR) && !QC_AFC_SNK_EN
#if ((!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
    dpm_status_t *dpm_stat = dpm_get_status(port);
#endif /* ((!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
#endif
#if CCG_TYPE_A_PORT_ENABLE
    if(port != TYPEC_PORT_0_IDX)
    {
        /* Only Port 0 is Type C port */
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    /*
     * NOTE: The port index variable is multiplex for this usage model.
     * The application currently supports only one Type-C port. If this
     * gets modified, then the port index handling should be updated to match.
     */

    switch (evt)
    {
        case APP_EVT_DISCONNECT:
        case APP_EVT_VBUS_PORT_DISABLE:
#if ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
#endif /* ((defined(CY_DEVICE_CCG5)) || (defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6))) */
            (void)bc_stop(context);
            break;

        case APP_EVT_PE_DISABLED:
#if (!LEGACY_PD_PARALLEL_OPER)
            /* Start legacy state machine once PE is disabled. */
            if (ptrDpmConfig->curPortRole ==CY_PD_PRT_ROLE_SOURCE)
            {
                /* Already started in case of parallel legacy and PD source operation. */
                if ((chg_cfg->srcSel & BATT_CHARGING_SRC_MASK) != 0u)
                {
                    bc_start(context, BC_PORT_SOURCE);
                }
            }

#if ((!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
            if (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SINK)
            {
#if (!BCR) && !QC_AFC_SNK_EN
                if ((chg_cfg->srcSel & BATT_CHARGING_SINK_MASK) != 0u)
#else
                if ((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) != 0u)
#endif /* (!BCR) && !QC_AFC_SNK_EN */
                {
                    bc_start(context, BC_PORT_SINK);
                }
            }
#endif /* ((!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
#endif /* (!LEGACY_PD_PARALLEL_OPER) */
            break; 
 
#if (LEGACY_PD_PARALLEL_OPER)
        case APP_EVT_TYPEC_ATTACH:
            /* Start legacy state machine once PE is disabled. */

            if (ptrDpmConfig->curPortRole ==CY_PD_PRT_ROLE_SOURCE)
            {
            /* Already started in case of parallel legacy and PD source operation. */
                if ((chg_cfg->srcSel & BATT_CHARGING_SRC_MASK) != 0u)
                {
                    (void)bc_start(context, BC_PORT_SOURCE);
                }
            }
#if ((!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
            else
            {
                if ((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) != 0)
                {
#if (!BCR) && !QC_AFC_SNK_EN
                    /* Disable BC 1.2 if Rp value is anything other than Default-Rp */
                    if(ptrDpmConfig->snkCurLevel == RD_USB)
                    {                        
                        bc_start(context, BC_PORT_SINK);
                    }
#else
                    bc_start(context, BC_PORT_SINK);
#endif /* !BCR && !QC_AFC_SNK_EN */
                }
            }
#endif /* ((!CY_PD_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
            break;
#endif /* (LEGACY_PD_PARALLEL_OPER) */

#if (LEGACY_PD_PARALLEL_OPER)
       case APP_EVT_PD_SINK_DEVICE_CONNECTED:
            if (
                    (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SOURCE) &&
                    ((chg_cfg->srcSel & BATT_CHARGING_SRC_MASK) != 0u)
               )
            {
                /* Sink device responded to SRC CAP. Stop legacy state machine. */
#if (!CCG_BC_12_IN_PD_ENABLE)
                (void)bc_stop(context);
#endif /* (!CCG_BC_12_IN_PD_ENABLE) */
            }
            break;

#if (!BC_SOURCE_ONLY)
#if (!BCR) && !QC_AFC_SNK_EN
       case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            if (
                    (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SINK) &&
                    ((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) != 0u)
               )
            {
                /* PD message received. Stop BC 1.2 state machine. */
                (void)bc_stop(context);
            }
            break;
#else
       case APP_EVT_PKT_RCVD:
            if (
                    (ptrDpmConfig->curPortRole == CY_PD_PRT_ROLE_SINK) &&
                    ((chg_cfg->snkSel & BATT_CHARGING_SINK_MASK) != 0u)
               )
            {
                /* PD message received .
                 * Stop BC state machine if not in QC/AFC mode. */
                if(bc_is_qc_afc_charging_active(context)== false)
                {
                    bc_stop(context);
                }
            }
            break;
#endif /* (!BCR) && !QC_AFC_SNK_EN */
#endif /* (!BC_SOURCE_ONLY) */

#endif /* (LEGACY_PD_PARALLEL_OPER) */

        default:
            /* No Statement */
            break;
    }
}

#if (BCR) || QC_AFC_SNK_EN
bool bc_is_qc_afc_charging_active(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    return ((bc_stat->cur_mode == BC_CHARGE_QC2) || (bc_stat->cur_mode == BC_CHARGE_AFC));
}

bool bc_is_bc_charging_active(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    return (bc_stat->cur_mode != BC_CHARGE_NONE);
}
#endif /* BCR || QC_AFC_SNK_EN */

void bc_set_bc_evt(cy_stc_pdstack_context_t * context, uint32_t evt_mask)
{
    uint8_t cport=context->port;

    bc_status_t* bc_stat = &gl_bc_status[cport];

    uint8_t intr_state = Cy_SysLib_EnterCriticalSection();

    bc_stat->bc_evt |= evt_mask;

    Cy_SysLib_ExitCriticalSection(intr_state);
}

void bc_clear_bc_evt(cy_stc_pdstack_context_t * context, uint32_t evt_mask)
{
    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];

    uint8_t intr_state = Cy_SysLib_EnterCriticalSection();

    bc_stat->bc_evt &= ~evt_mask;

    Cy_SysLib_ExitCriticalSection(intr_state);
}

#if (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
#if (!APPLE_SINK_DISABLE)
static void bc_eval_apple_brick_id(cy_stc_pdstack_context_t * context, bc_apple_brick_id brick_id)
{
    /* Handle detected Apple Brick IDs. */
    switch (brick_id)
    {
        case APPLE_BRICK_ID_3:
            psnk_set_current (context, APPLE_AMP_1A);
            break;

        case APPLE_BRICK_ID_1:
            psnk_set_current (context, APPLE_AMP_2_1A);
            break;

        case APPLE_BRICK_ID_4:
             psnk_set_current (context, APPLE_AMP_2_4A);
            break;

        default:
            /* Rest of the Brick IDs reserved for future use. */
            break;
    }

    /* Ensure VBUS is set to 5V. */
    psnk_set_voltage (context, CY_PD_VSAFE_5V);
#if (BCR) || QC_AFC_SNK_EN
    bc_psnk_enable(context);
#endif
}
#endif /* (!APPLE_SINK_DISABLE) */
#endif /* (!(CY_PD_SOURCE_ONLY)) */

#if (!defined(CY_DEVICE_CCG5))
/* FSM functions start */

static void bc_fsm_off(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    (void)evt;
    uint8_t cport=context->port;
#if (BCR) || QC_AFC_SNK_EN
    bc_set_mode(context,BC_CHARGE_NONE);
#else
    gl_bc_status[cport].cur_mode = BC_CHARGE_NONE;
#endif
}

#if !CY_PD_SINK_ONLY
static void bc_fsm_src_look_for_connect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);

#if  (ENABLE_APPLE_BC12_SUPPORT || (!APPLE_SOURCE_DISABLE))
    uint8_t chgb_dm_comp_ref = (uint8_t)CHGB_VREF_2_2V;
    (void)chgb_dm_comp_ref;
#if  (ENABLE_APPLE_BC12_SUPPORT)
    bool comp0_flag, comp1_flag;
#endif /* (ENABLE_APPLE_BC12_SUPPORT) */
#endif /* (ENABLE_APPLE_BC12_SUPPORT || (!APPLE_SOURCE_DISABLE)) */

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            /*
             * The detection logic varies based on the protocols selected.
             * If only Apple charging is selected, then different Apple source IDs
             * are supported. If Apple charging needs to be supported along with
             * BC 1.2, then only 2.4A Apple charger ID is supported.
             *
             * If Apple charging is selected, keep presenting the Apple terminations.
             * In case of Apple only charging mode, keep presenting the terminations.
             * There is no further action after this.
             *
             * If Apple charging along with BC 1.2 based detection is enabled,
             * then first start with Apple 2.4A termination. Also enable D+ comparator
             * to look for < 2.2V. If this is detected, then switch to BC termination
             * and proceed with BC 1.2 based connection detection logic.
             *
             * If Apple charging is not selected, then proceed directly to BC 1.2
             * terminations and subsequent detection logic.
             *
             * Detach detection for Apple and BC 1.2 DCP sink cannot be done as
             * sink terminations are not present after detection. Only re-attach
             * can be detected. So, In case of BC 1.2 DCP operation, this state
             * shall be re-entered to reapply Apple terminations as required.
             *
             * In case of QC and AFC mode of operation, detach can be detected.
             * When this happens, this state shall be re-entered. Detach handling
             * needs to be done for this.
             *
             * NOTE: There are two cases which are not currently dealt with:
             * 1. In case of Type-C port, when we enter legacy state machine for
             *    the first time, the VBUS may be already be present and the
             *    sink may already be attached and completed its detection logic.
             *    We may need to power cycle VBUS to restart sink's detection
             *    logic. This is not currently done as we start PD and Legacy
             *    together (LEGACY_PD_PARALLEL_OPER).
             *
             * 2. In case of Type-A port attached to a BC 1.2 sink or an Apple
             *    sink, there is no real detach detection. When Apple charging
             *    is enabled, there is also no re-attach detection.
             *
             *    The type-A port current consumption is monitored to switch to
             *    low power regulator when current drops below 300mA. This same
             *    logic moves back to high power regulator, but there is a polling
             *    delay as well as regulator turn on delay involved which can
             *    cause the 5V regulator (which also feeds the CCG3PA device) to
             *    shut-off due to over current.
             *
             *    There may be a work around; when we switch to low power regulator,
             *    also switch to BC 1.2 DCP attach wait state and stay there until
             *    there is an attach. In event of attach, switch to Apple mode of
             *    detection and proceed as usual.
             *
             *    Since the current systems are able to withstand the switch
             *    without disconnecting, no implementation is done currently.
             */
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_1_IDX);
            cy_sw_timer_stop_range(context->ptrTimerContext, APP_BC_GENERIC_TIMER1, APP_BC_DETACH_DETECT_TIMER);
            bc_clear_bc_evt(context, BC_EVT_ALL_MASK);
            (void)Cy_USBPD_Bch_Phy_En(context->ptrUsbPdContext);

#if (!APPLE_SOURCE_DISABLE)
            if ((chg_cfg->srcSel & BC_SRC_APPLE_MODE_ENABLE_MASK) != 0x00u)
            {
#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
                /*
                 * If Apple charging is enabled, then present Apple terminations.
                 * Since parallel operation requires external control, invoke
                 * the solution function instead of the HAL function. It is expected
                 * that the solution handler uses external termination for DP
                 * when parallel operation is required. The solution handler
                 * can choose to use internal termination when parallel operation
                 * is not required. This is useful when only one port requires
                 * parallel operation.
                 */
                sln_apply_apple_src_term(cport, (cy_en_usbpd_bch_src_term_t)chg_cfg->appleSrcId);
                bc_stat->cur_amp = apple_id_to_cur_map[chg_cfg->appleSrcId];
#else
                (void)Cy_USBPD_Bch_Phy_RemoveTerm(context->ptrUsbPdContext);
#if defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)
                if((cy_en_usbpd_bch_src_term_t)chg_cfg->appleSrcId == CHGB_SRC_TERM_APPLE_3A)
                {
                    /*
                     * Apple Charging 3A termination support is available only for CCG7D *A silicon and CCG7S silicons.
                     * For all other silicon, there is no support for Apple Charging 3A terminations
                     * To use an external resistor divider for the same, enable LEGACY_APPLE_SRC_SLN_TERM_ENABLE macro
                     * The solution handler can use the GPIO to drive the external resistor divider to get the required
                     * voltage on the DP and DM lines.
                     */
#if defined(CY_DEVICE_CCG7D)
                    if(Cy_USBPD_Get_Silicon_Rev(context->ptrUsbPdContext) > 0u)
#endif /* defined(CY_DEVICE_CCG7D) */
                    {
                        chgb_dm_comp_ref = (uint8_t)CHGB_VREF_2_9V;
                        (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_APPLE_3A);
                    }
#if defined(CY_DEVICE_CCG7D)
                    else
                    {
                        (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_DCP);
                        Cy_USBPD_Bch_Apply_AppleTermDp(context->ptrUsbPdContext, CHGB_SRC_TERM_APPLE_2_4A);                 
                    }
#endif /* defined(CY_DEVICE_CCG7D) */
                }
                else
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
                {
                    (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_DCP);
                    Cy_USBPD_Bch_Apply_AppleTermDp(context->ptrUsbPdContext, CHGB_SRC_TERM_APPLE_2_4A);
                }
                
                /* Set the active supply current */
                bc_stat->cur_amp = apple_id_to_cur_map[chg_cfg->appleSrcId];

#endif /* LEGACY_APPLE_SRC_SLN_TERM_ENABLE */

#if ENABLE_APPLE_BC12_SUPPORT
                /*
                 * If parallel operation is expected, then setup CMP2 to
                 * detect D+ going below 2.2V and setup CMP1 to detect D- going below 2.2V as well.
                 * If it goes below this level, then it means that a BC 1.2 based sink is attached.
                 */    
                if ((chg_cfg->srcSel & BC_SRC_1_2_MODE_ENABLE_MASK) != 0u)
                {
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_EDGE_FALLING);
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                            (cy_en_usbpd_bch_vref_t)chgb_dm_comp_ref, CHGB_COMP_EDGE_FALLING);
                }
#endif /* ENABLE_APPLE_BC12_SUPPORT */

                /* Indicate connectivity. */
                bc_stat->attach = true;
                bc_stat->connected = true;
                bc_stat->cur_mode = BC_CHARGE_APPLE;

#if CCG_TYPE_A_PORT_ENABLE
                if (cport == TYPE_A_PORT_ID)
                {
                    type_a_update_status(true, true);
                }
#endif /* CCG_TYPE_A_PORT_ENABLE */
            }
            else
#endif /* (!APPLE_SOURCE_DISABLE) */
            {
                /*
                 * If in DCP mode, do not change the setting, else this is
                 * a detach. Indicate the same.
                 */
                if (bc_stat->cur_mode != BC_CHARGE_DCP)
                {
                    bc_stat->cur_mode = BC_CHARGE_NONE;
                    bc_stat->attach = false;

#if CCG_TYPE_A_PORT_ENABLE
                    if (cport == TYPE_A_PORT_ID)
                    {
                        type_a_update_status(false, false);
                    }
#endif /* CCG_TYPE_A_PORT_ENABLE */
                }

                /* Ensure DCP terminations are on by default. */
                (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_DCP);

                /* Set Comp1 to look for > 0.4V on D+ */
                (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);
            }

            /* Ensure that VBUS is 5V. But do this only if PD is disabled. */
            if (
#if CCG_TYPE_A_PORT_ENABLE
                    (cport != TYPEC_PORT_0_IDX) ||
#endif /* CCG_TYPE_A_PORT_ENABLE */
                    (dpm_stat->pdDisabled != false)
                )
            {
                psrc_set_voltage(context, CY_PD_VSAFE_5V);
                psrc_enable(context, bc_pwr_ready_cbk);
            }
            break;

        case BC_FSM_EVT_CMP1_FIRE:
#if CCG_TYPE_A_PORT_ENABLE
            /* Switch to high power VBUS regulator as soon as connect is detected. */
            if (cport == TYPE_A_PORT_ID)
            {
                type_a_update_status (true, false);
            }
#endif /* CCG_TYPE_A_PORT_ENABLE */
            if(bc_stat->cur_mode == BC_CHARGE_APPLE)
            {
                /*
                 * A BC 1.2 based sink has been attached. Need to switch the
                 * terminations to BC 1.2.
                 */
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext,BC_CMP_1_IDX);
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext,BC_CMP_0_IDX);
                bc_clear_bc_evt(context, BC_EVT_ALL_MASK);

                /*
                 * This may be a glitch. Do a small debounce to ensure that
                 * we are attached to a BC device. Also, latest iphones are
                 * also causing a glitch on DP during connect. This need to
                 * be filtered out. There is no real debounce logic required.
                 * If a BC device is attached, the line would stay low beyond
                 * a fixed duration; if not the line shall revert back to 2.2V.
                 */
                (void)cy_sw_timer_start(context->ptrTimerContext,context,(uint8_t)APP_BC_GENERIC_TIMER1, APP_BC_APPLE_DETECT_TIMER_PERIOD, bc_tmr_cbk);
            }
            else
            {
                bc_stat->attach = true;
                bc_stat->cur_mode = BC_CHARGE_NONE;
                bc_stat->cur_amp = BC_AMP_LIMIT;
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext,BC_CMP_0_IDX);
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext,BC_CMP_1_IDX);
                bc_stat->bc_fsm_state = BC_FSM_SRC_INITIAL_CONNECT;
                bc_set_bc_evt(context, BC_EVT_ENTRY);
            }
            break;

#if ENABLE_APPLE_BC12_SUPPORT
        case BC_FSM_EVT_CMP2_FIRE:
            if(bc_stat->cur_mode == BC_CHARGE_APPLE)
            {
                /*
                 * A BC 1.2 based sink has been attached. Need to switch the
                 * terminations to BC 1.2.
                 */
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_1_IDX);
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);                
                bc_clear_bc_evt(context, BC_EVT_ALL_MASK);

                /*
                 * This may be a glitch. Do a small debounce to ensure that
                 * we are attached to a BC device. Also, latest iphones are
                 * also causing a glitch on DP during connect. This need to
                 * be filtered out. There is no real debounce logic required.
                 * If a BC device is attached, the line would stay low beyond
                 * a fixed duration; if not the line shall revert back to 2.2V.
                 */
                (void)cy_sw_timer_start(context->ptrTimerContext,context,(uint8_t)APP_BC_GENERIC_TIMER1, APP_BC_APPLE_DETECT_TIMER_PERIOD, bc_tmr_cbk);
            }
            break;

        case BC_FSM_EVT_TIMEOUT1:
            if(bc_stat->cur_mode == BC_CHARGE_APPLE)
            {
                /*
                 * If the DP/DM voltage continues below 2.2V, we are attached to
                 * BC device; proceed with BC detection. If not, go back to Apple
                 * mode and wait for DP/DM to go down again.
                 */
                comp0_flag = (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                            (cy_en_usbpd_bch_vref_t)chgb_dm_comp_ref, CHGB_COMP_NO_INTR) == false);
                comp1_flag = (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == false);

                if (comp0_flag || comp1_flag)
                {
#if LEGACY_APPLE_SRC_SLN_TERM_ENABLE                    
                    sln_remove_apple_src_term(cport);
#endif /* LEGACY_APPLE_SRC_SLN_TERM_ENABLE */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_PAG1S))
                    (void)Cy_USBPD_Bch_Phy_RemoveTerm(context->ptrUsbPdContext);    
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_PAG1S)) */

                    /* Ensure DCP terminations are on by default. */
                    (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_DCP);

                    bc_stat->attach = false;
                    bc_stat->connected = false;
                    bc_stat->cur_mode = BC_CHARGE_NONE;

                    /* Set Comp1 to look for > 0.4V on D+ */
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);

#if CCG_TYPE_A_PORT_ENABLE
                    /*
                     * Switch to low power regulator for the time being.
                     */
                    if (cport == TYPE_A_PORT_ID)
                    {
                        type_a_update_status(false, false);
                    }
#endif /* CCG_TYPE_A_PORT_ENABLE */
                }
                else
                {
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_EDGE_FALLING);
                    (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                            (cy_en_usbpd_bch_vref_t)chgb_dm_comp_ref, CHGB_COMP_EDGE_FALLING);
                }
            }
            break;
#endif /* (ENABLE_APPLE_BC12_SUPPORT) */

        default:
            /* No Statement */
            break;
    }
}

static void bc_fsm_src_initial_connect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            bc_stat->comp_rising = false;
            /* Set Comp1 to look for < 0.4V on D+ for 1 second */
            (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
            /* Set Comp2 to ensure DP does not go above 2.2V. */
            (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_EDGE_RISING);
            /* Start TGLITCH_BC_DONE timer to ascertain Apple or others */
            (void)cy_sw_timer_start (context->ptrTimerContext,context, APP_BC_GENERIC_TIMER1, APP_BC_DCP_DETECT_TIMER_PERIOD, bc_tmr_cbk);
            break;

        case BC_FSM_EVT_CMP1_FIRE:
            if(bc_stat->comp_rising == false)
            {
                /*
                 * If AFC or BC1.2 mode is enabled, then we will
                 * have to determine those devices.
                 */
                if ((chg_cfg->srcSel & (BC_SRC_1_2_MODE_ENABLE_MASK
                                | BC_SRC_AFC_MODE_ENABLE_MASK)) != 0u)
                {
                    bc_clear_bc_evt (context, BC_EVT_TIMEOUT2);
                    (void)cy_sw_timer_start (context->ptrTimerContext,context,(cy_sw_timer_id_t)APP_BC_GENERIC_TIMER2, 150u, bc_tmr_cbk);
                }
                /*
                 * In QC only mode, stop DCP detect timer and wait for DP to rise
                 * above 0.6V again.
                 */
                else
                {
                    cy_sw_timer_stop (context->ptrTimerContext,APP_BC_GENERIC_TIMER1);
                    bc_clear_bc_evt (context, BC_EVT_TIMEOUT1);
                }
                /* Set Comp1 to look for > 0.4V on D+. */
                (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);
                bc_stat->comp_rising = true;
            }
            else
            {
                /* The DP line has gone above 0.6V. Re-start detection. */
                cy_sw_timer_stop(context->ptrTimerContext,APP_BC_GENERIC_TIMER2);
                bc_clear_bc_evt(context, BC_EVT_TIMEOUT2);
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
                bc_set_bc_evt(context, BC_EVT_ENTRY);
            }
            break;


        case BC_FSM_EVT_CMP2_FIRE:
            /*
             * If TGLITCH_BC_DONE timer is running, that means DP went above 2V before
             * glitch filter timer expired. We will have to wait for DP to come back in 0.4 - 2V range
             * and then start device detection again. Till then we will stay in DCP only mode.
             */
            if (cy_sw_timer_is_running (context->ptrTimerContext,APP_BC_GENERIC_TIMER1))
            {
                /* DP went above 2V. We should stop DCP Detect timer. */
                cy_sw_timer_stop(context->ptrTimerContext,APP_BC_GENERIC_TIMER1);
                bc_clear_bc_evt(context, BC_EVT_TIMEOUT1);
                /* Stop Apple device detect timer as well. */
                cy_sw_timer_stop(context->ptrTimerContext,APP_BC_GENERIC_TIMER2);
                bc_clear_bc_evt(context, BC_EVT_TIMEOUT2);

                /* Stop Comp0. */
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
                bc_clear_bc_evt(context, BC_EVT_CMP1_FIRE);

                /*
                 * From this point on we should wait for DP to fall below 2V. When it falls below
                 * 2V, we can start device detection again.
                 */
                (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_EDGE_FALLING);
            }
            else
            {
                /* DP is now below 2V. Start device detection again. */
                (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext,BC_CMP_1_IDX);
                bc_stat->bc_fsm_state = BC_FSM_SRC_INITIAL_CONNECT;
                bc_set_bc_evt(context, BC_EVT_ENTRY);
            }
            break;

#if (!QC_AFC_CHARGING_DISABLED)
        case BC_FSM_EVT_TIMEOUT1:
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            bc_clear_bc_evt(context, BC_EVT_CMP1_FIRE);
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_1_IDX);
            bc_clear_bc_evt(context, BC_EVT_CMP2_FIRE);
            cy_sw_timer_stop(context->ptrTimerContext, (uint8_t)APP_BC_GENERIC_TIMER2);
            bc_clear_bc_evt(context, BC_EVT_TIMEOUT2);
            bc_stat->bc_fsm_state = BC_FSM_SRC_OTHERS_CONNECTED;
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            break;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        case BC_FSM_EVT_TIMEOUT2:
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            bc_clear_bc_evt(context, BC_EVT_CMP1_FIRE);
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_1_IDX);
            bc_clear_bc_evt(context, BC_EVT_CMP2_FIRE);
            cy_sw_timer_stop(context->ptrTimerContext, (cy_sw_timer_id_t)APP_BC_GENERIC_TIMER1);
            bc_clear_bc_evt(context, BC_EVT_TIMEOUT1);

            /* Treat this as a BC1.2 device. */
            /* Indicate BC1.2 device is connected so that current
             * monitoring can start. */
#if CCG_TYPE_A_PORT_ENABLE
            if (cport == TYPE_A_PORT_ID)
            {
                type_a_update_status (true, true);
            }
#endif /* CCG_TYPE_A_PORT_ENABLE */
            bc_stat->connected = true;
            bc_stat->cur_mode = BC_CHARGE_DCP;
            bc_stat->cur_amp = BC_AMP_LIMIT;
            /* Go back to init state and wait for DP attach event similar
             * to Apple mode. */
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            break;
        default:
            /* No Statement */
            break;
    }
}

static void bc_fsm_src_apple_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
        /*DO NOTHING */
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(evt);

}
#endif /* !CY_PD_SINK_ONLY */

#if (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6)

void cdp_look_for_dev_disconnect_cb(cy_stc_pdstack_context_t * context, cy_timer_id_t id)
{
    /* If the port is configured as CDP and a sink was previously connected, check whether it has been
     * removed.
     */
    uint8_t cport=context->port;
    if (detach_detect_en[cport])
    {
        /* Configure the comparators and select the output */
        chgb_config_cdp_comparators(cport);

        /* Configure and start the Counter */
        chgb_counter_start(cport);

        /* Start 5ms timer and handle detach detection in cbk */
        cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_DETACH_DETECT_TIMER, 5, bc_cdp_detach_cbk);
    }

    /* Restart the polling timer. */
    cy_sw_timer_start(context->ptrTimerContext,context,APP_CDP_DP_DM_POLL_TIMER, 500, cdp_look_for_dev_disconnect_cb);
}

static void bc_fsm_src_apply_cdp(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_1_IDX);
            cy_sw_timer_stop_range(context->ptrTimerContext, APP_BC_GENERIC_TIMER1, APP_CDP_DP_DM_POLL_TIMER);
            bc_clear_bc_evt(context, BC_EVT_ALL_MASK);
            detach_detect_en[cport] = false;
            Cy_USBPD_Bch_Phy_En(context->ptrUsbPdContext);

            bc_stat->cur_mode = BC_CHARGE_CDP;
            Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext, CHGB_SRC_TERM_CDP);

            /* Set comparator 1 to fire when D+ > 0.325V */
            Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);
            /* Set comparator 2 to fire when D+ > 0.85V */
            Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_85V, CHGB_COMP_EDGE_RISING);
            vdm_src_applied[cport] = false;
            bc_stat->comp_rising = false;
            break;

        case BC_FSM_EVT_CMP1_FIRE:
            if (!bc_stat->comp_rising)
            {
                /* Voltage in 0.4 to 0.8 range. Turn on VDM_SRC */
                if(!vdm_src_applied[cport])
                {
                    vdm_src_applied[cport] = true;
                    bc_stat->comp_rising = true;
                    chgb_apply_vdm_src(cport);

                    /* Set comparator 1 to fire when D+ < 0.325V */
                    Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
                }
            }
            else
            {
                /* Voltage below 0.4V , turn off VDM_SRC */
                bc_stat->comp_rising = false;
                if (vdm_src_applied[cport])
                {
                    Cy_USBPD_Bch_Remove_VdmSrc(context->ptrUsbPdContext);
                    vdm_src_applied[cport] = false;
                }
                Cy_USBPD_Bch_Phy_DisableComp(context->ptrTimerContext, BC_CMP_0_IDX);
                Cy_USBPD_Bch_Phy_DisableComp(context->ptrTimerContext, BC_CMP_1_IDX);
                bc_set_bc_evt(context, BC_EVT_ENTRY);
            }
            break;

        case BC_FSM_EVT_CMP2_FIRE:
            /* Voltage gone above 0.8. Stop CDP detection */
            Cy_USBPD_Bch_Phy_DisableComp(context->ptrTimerContext, BC_CMP_0_IDX);
            cy_sw_timer_stop_range(context->ptrTimerContext, APP_BC_GENERIC_TIMER1, APP_BC_DP_DM_DEBOUNCE_TIMER);
            if(vdm_src_applied[cport])
            {
                Cy_USBPD_Bch_Remove_VdmSrc(context->ptrUsbPdContext);
                vdm_src_applied[cport] = false;
            }
            /* Isolate the DPDM lines from the charge detect block */
            Cy_USBPD_Bch_Phy_Isolate_DpDm(context->ptrUsbPdContexts);

            /* Start a timer to poll DP/DM lines to detect USB device disconnection. */
            detach_detect_en[cport] = true ;
            cy_sw_timer_start(context->ptrTimerContext,context,APP_CDP_DP_DM_POLL_TIMER, 500, cdp_look_for_dev_disconnect_cb);
            break;

        default:
            break;
    }
}
#endif /* (defined CY_DEVICE_CCG5C) || (defined CY_DEVICE_CCG6) */

#if (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED))
static void bc_stop_src_cap_on_detect(cy_stc_pdstack_context_t * context)
{
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
    if (
#if CCG_TYPE_A_PORT_ENABLE
         (cport == TYPEC_PORT_0_IDX) &&
#endif /* CCG_TYPE_A_PORT_ENABLE */
         (false == dpm_stat->pdDisabled)
       )
    {
        (void)Cy_PdStack_Dpm_Disable(context);
    }

}

static void bc_fsm_src_others_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            bc_stat->connected = true;

            if((chg_cfg->srcSel &
               (BC_SRC_AFC_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK)) != 0u)
            {
                bc_stat->dp_dm_status.state = (uint16_t)QC_MODE_5V;
                bc_stat->old_dp_dm_status.state = (uint16_t)QC_MODE_5V;

                /* Try detecting QC or AFC */
                (void)Cy_USBPD_Bch_Phy_ConfigSrcTerm(context->ptrUsbPdContext,CHGB_SRC_TERM_QC);

                /* Let voltage settle */
                Cy_SysLib_DelayUs(100);
            }
            /* Set Comp1 to look for < 0.4V on D- to ensure no short on DP/DM */
            (void)Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
            break;
        case BC_FSM_EVT_CMP1_FIRE:
            /* Move to next state */
            (void)Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            bc_clear_bc_evt(context, BC_EVT_CMP1_FIRE);
            if((chg_cfg->srcSel &
               (BC_SRC_AFC_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK)) != 0u)
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_QC_OR_AFC;
            }
            else
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            }
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            break;
        default:
            /* Intentionally left empty */
            break;
    }
}

static void bc_fsm_src_qc_or_afc(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            if((chg_cfg->srcSel & BC_SRC_AFC_MODE_ENABLE_MASK) != 0u)
            {
                bc_stat->afc_src_msg_count = 0;
                bc_stat->afc_src_match_count = 0;
                bc_stat->afc_src_is_matched = false;
                (void)Cy_USBPD_Bch_AfcSrcInit(context->ptrUsbPdContext);
                (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
            }
            if((chg_cfg->srcSel & BC_SRC_QC_MODE_ENABLE_MASK) != 0u)
            {
                (void)Cy_USBPD_Bch_QcSrcInit(context->ptrUsbPdContext);
            }
            break;
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            break;
        case BC_FSM_EVT_QC_CHANGE:
            /* Not AFC move to QC detected */
            (void)Cy_USBPD_Bch_AfcSrcStop(context->ptrUsbPdContext);
            bc_stat->cur_mode = BC_CHARGE_QC2;
            bc_stat->bc_fsm_state = BC_FSM_SRC_QC_CONNECTED;
            bc_set_bc_evt(context, BC_EVT_QC_CHANGE);
            break;
        case BC_FSM_EVT_AFC_MSG_RCVD:
            /* Not QC, move to AFC detected */
            (void)Cy_USBPD_Bch_QcSrcStop(context->ptrUsbPdContext);
            bc_stat->cur_mode = BC_CHARGE_AFC;
            bc_stat->cur_amp = BC_AMP_LIMIT;
            bc_stat->bc_fsm_state = BC_FSM_SRC_AFC_CONNECTED;
            bc_set_bc_evt(context, BC_EVT_AFC_MSG_RCVD);
            bc_stop_src_cap_on_detect(context);
            break;
        case  BC_FSM_EVT_AFC_MSG_SEND_FAIL:
            (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
            break;
        case BC_FSM_EVT_AFC_RESET_RCVD:
            (void)Cy_USBPD_Bch_AfcSrcStop(context->ptrUsbPdContext);
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->afc_src_is_matched = false;
            bc_stat->afc_tx_active = (uint8_t)false;
            (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
            break;
        default:
            /* Intentionally left empty */
            break;
    }
}

#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
void qc_set_cf_limit(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    /* Get the current port PDP value */
    uint8_t port_pdp = ccg_get_system_max_pdp(context);
    
    gl_bc_status[cport].cur_amp = (uint16_t)CY_USBPD_GET_MIN(((port_pdp * CCG_POWER_PRECISION_MULT)/psrc_get_voltage(context)),
                                    LEGACY_MAX_CABLE_RATING);
    Cy_USBPD_CF_Enable(context->ptrUsbPdContext, gl_bc_status[cport].cur_amp);
    app_get_status(cport)->cur_fb_enabled = true;
}
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */


static void bc_fsm_src_qc_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    int pulse_count;
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)    
    bool new_qc_state = false;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */
    uint32_t new_volt = 0;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);

    switch(evt)
    {
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
        (void)Cy_USBPD_Bch_QcSrcContModeStop(context->ptrUsbPdContext);
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)            
            /* On QC detach, disable the current foldback on the port */
            Cy_USBPD_CF_Disable(context->ptrUsbPdContext);
            (void)Cy_PdStack_Dpm_SetCf(context, false);
            app_get_status(cport)->cur_fb_enabled = false;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */            
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            if(app_get_status(cport)->psrc_volt != CY_PD_VSAFE_5V)
            {
                /* Update the voltage to VSAFE_5V on exit from QC state */
                psrc_set_voltage(context, CY_PD_VSAFE_5V);
                psrc_enable(context, bc_pwr_ready_cbk);
            }
            break;
        case BC_FSM_EVT_QC_CHANGE:
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)            
            /* We have detected a voltage change. Set this variable to update the CF value */
            new_qc_state = true;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */    
            if(bc_stat->dp_dm_status.state != QC_MODE_CONT)
            {
                bc_stat->cur_mode = BC_CHARGE_QC2;
                /* Disable Continuous mode operation */
                (void)Cy_USBPD_Bch_QcSrcContModeStop(context->ptrUsbPdContext);
                bc_clear_bc_evt(context, BC_EVT_QC_CONT);
            }
            
            if(bc_stat->dp_dm_status.state != QC_MODE_CONT)
            {
                /*
                 * Here if we detect a voltage other than 5V in QC mode we stop sending source caps.
                 */
                bc_stop_src_cap_on_detect(context);
            }
            switch(bc_stat->dp_dm_status.state)
            {
                case (uint16_t)QC_MODE_5V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                
                    bc_stat->cur_amp = QC_AMP_5V;   
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(context, CY_PD_VSAFE_5V);
                    psrc_enable(context, bc_pwr_ready_cbk);
                    break;
                case (uint16_t)QC_MODE_9V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                    
                    bc_stat->cur_amp = QC_AMP_9V;
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(context, CY_PD_VSAFE_9V);
                    psrc_enable(context, bc_pwr_ready_cbk);
                    break;
                case (uint16_t)QC_MODE_12V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                    
                    bc_stat->cur_amp = QC_AMP_12V;
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(context, CY_PD_VSAFE_12V);
                    psrc_enable(context, bc_pwr_ready_cbk);
                    break;
                case (uint16_t)QC_MODE_20V:
                    if((chg_cfg->qcSrcType == BC_SRC_QC_VER_2_CLASS_B_VAL) ||
                       (chg_cfg->qcSrcType == BC_SRC_QC_VER_3_CLASS_B_VAL)
                       )
                    {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))
                        bc_stat->cur_amp = QC_AMP_20V;    
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))*/
                        psrc_set_voltage(context, CY_PD_VSAFE_20V);
                        psrc_enable(context, bc_pwr_ready_cbk);
                    }
                    break;
                case (uint16_t)QC_MODE_CONT:
                    if(chg_cfg->qcSrcType >= BC_SRC_QC_VER_3_CLASS_A_VAL)
                    {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                        
                        bc_stat->cur_amp = QC_AMP_CONT;
                        bc_stat->cur_mode = BC_CHARGE_QC3;
#endif /* (!((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                        /* Enable Continuous mode operation */
                        (void)Cy_USBPD_Bch_QcSrcContModeStart(context->ptrUsbPdContext);

#if (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_QC_3_0_DISABLE)
                        /* 
                         * When entering into QC3 continuous mode, a new explicit 
                         * voltage request call is needed to remove cable 
                         * compensation voltage from the existing VBUS voltage.
                         */
                        psrc_set_voltage(context, app_get_status(cport)->psrc_volt);
#endif /* (CY_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_QC_3_0_DISABLE) */
                    }
                    break;
                default:
                    /* Intentionally left empty */
                    break;
            }
            break;
        case BC_FSM_EVT_QC_CONT:
            pulse_count = Cy_USBPD_Bch_Get_QcPulseCount(context->ptrUsbPdContext);
            if(pulse_count > 0)
            {
                /* Voltage change in mV units. Each pulse cause 200mV change */
                new_volt = (uint32_t)pulse_count * QC_CONT_VOLT_CHANGE_PER_PULSE;
                new_volt = new_volt + app_get_status(cport)->psrc_volt;
            }
            else
            {
                new_volt = 0u - pulse_count;
                /* Voltage change in mV units. Each pulse cause 200mV change */
                new_volt = new_volt * QC_CONT_VOLT_CHANGE_PER_PULSE;
                if(new_volt <= ((uint32_t)app_get_status(cport)->psrc_volt - QC3_MIN_VOLT))
                {
                    new_volt = app_get_status(cport)->psrc_volt - new_volt;
                }
                else
                {
                    new_volt = QC3_MIN_VOLT;
                }
            }

            /* Check for minimum and maximum voltage levels and limit to allowed range. */
            if(new_volt < QC3_MIN_VOLT)
            {
                new_volt = QC3_MIN_VOLT;
            }
            if ((chg_cfg->qcSrcType  == BC_SRC_QC_VER_3_CLASS_A_VAL) && (new_volt > CY_PD_VSAFE_12V))
            {
                new_volt = CY_PD_VSAFE_12V;
            }
            if ((chg_cfg->qcSrcType == BC_SRC_QC_VER_3_CLASS_B_VAL) && (new_volt > CY_PD_VSAFE_20V))
            {
                new_volt = CY_PD_VSAFE_20V;
            }
            /* If the voltage is higher than 5V, then we should stop PD. */
            if(new_volt > CY_PD_VSAFE_5V)
            {
                bc_stop_src_cap_on_detect(context);
            }
            /* Update voltage only if there is a difference. */
            if(app_get_status(cport)->psrc_volt != new_volt)
            {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                        
                bc_stat->cur_amp = QC_AMP_CONT;    
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                psrc_set_voltage(context, (uint16_t)new_volt);
                psrc_enable(context, bc_pwr_ready_cbk);
            }
            /* Clear out the handled pulse count. */
            Cy_USBPD_Bch_Update_QcPulseCount(context->ptrUsbPdContext, pulse_count);
            break;
        default:
            /* Intentionally left empty */
            break;
    }
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
    if(
#if CCG_TYPE_A_PORT_ENABLE
      (cport == BC_PORT_0_IDX) && 
#endif /* CCG_TYPE_A_PORT_ENABLE */    
        ((app_get_status(cport)->cur_fb_enabled == false) || (new_qc_state == true)))
    {
        /* If current foldback has not been enabled on the port or if there is a QC state change
         * We update the current foldback on the port
         */
        qc_set_cf_limit(context);        
    }

#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */    
}

static void bc_afc_src_evaluate_match(cy_stc_pdstack_context_t * context)
{

    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    if(bc_stat->afc_src_msg_count >= 3u)
    {
        if(bc_stat->afc_src_match_count >= 2u)
        {
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            psrc_set_voltage(context, (AFC_BASE_VOLT + (BYTE_GET_UPPER_NIBBLE(bc_stat->afc_src_matched_byte) * AFC_VOLT_STEP)));
            bc_stat->cur_amp = ((uint16_t)AFC_BASE_AMP + (BYTE_GET_LOWER_NIBBLE((uint16_t)bc_stat->afc_src_matched_byte) * (uint16_t)AFC_AMP_STEP));
            psrc_enable(context, bc_pwr_ready_cbk);
            (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
        }
        else
        {
            /* Enter default operation and clear AFC counters if 3 attempts fail  */
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->cur_volt = CY_PD_VSAFE_5V;
            psrc_set_voltage(context, CY_PD_VSAFE_5V);
            bc_stat->cur_amp = BC_AMP_LIMIT;
            psrc_enable(context, bc_pwr_ready_cbk);
        }
    }
    else
    {
        (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
    }
}

static void bc_afc_src_handle_rcvd_msg(cy_stc_pdstack_context_t * context)
{
    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    /* Match Data received and send proper response same byte if match else all */
    uint8_t rcvd_vi = Cy_USBPD_Bch_Get_AfcDataPtr(context->ptrUsbPdContext)[0];
    uint8_t i;
    uint8_t *src_vi = bc_afc_src_get_vi_ptr(context);
    uint8_t src_count = bc_afc_src_get_vi_count(context);
    /* Set tx active flag. */
    bc_stat->afc_tx_active = (uint8_t)true;
    for(i = 0; i < src_count; i++)
    {
        if(((rcvd_vi & 0xF0u) == (src_vi[i] & 0xF0u) ) &&
           ((rcvd_vi & 0xFu) <= (src_vi[i] & 0xFu)))
        {
            bc_stat->afc_src_cur_match_byte = rcvd_vi;
            bc_stat->afc_src_is_matched = true;
            Cy_USBPD_Bch_Set_AfcTxData(context->ptrUsbPdContext, &rcvd_vi, 1);
            return;
        }
    }
    bc_stat->afc_src_is_matched = false;
    /* If there was no match, we clear both the AFC counters */
    bc_stat->afc_src_msg_count = 0;
    bc_stat->afc_src_match_count = 0;
    Cy_USBPD_Bch_Set_AfcTxData(context->ptrUsbPdContext, src_vi, src_count);
}

static void bc_fsm_src_afc_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t* bc_stat = &gl_bc_status[cport];

    switch(evt)
    {
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            (void)Cy_USBPD_Bch_AfcSrcStop(context->ptrUsbPdContext);
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(context, BC_EVT_ENTRY);
            if(app_get_status(cport)->psrc_volt != CY_PD_VSAFE_5V)
            {
                /* Update the voltage to VSAFE_5V on exit from AFC state */
                psrc_set_voltage(context, CY_PD_VSAFE_5V);
                psrc_enable(context, bc_pwr_ready_cbk);
            }
            break;
        case BC_FSM_EVT_AFC_MSG_RCVD:
            bc_afc_src_handle_rcvd_msg(context);
            break;
        case BC_FSM_EVT_AFC_RESET_RCVD:
            (void)Cy_USBPD_Bch_AfcSrcStop(context->ptrUsbPdContext);
            bc_stat->cur_volt = CY_PD_VSAFE_5V;
            psrc_set_voltage(context, CY_PD_VSAFE_5V);
            psrc_enable(context, bc_pwr_ready_cbk);
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->afc_src_is_matched = false;
            bc_stat->afc_tx_active = (uint8_t)false;
            (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
            break;
        case BC_FSM_EVT_AFC_MSG_SENT:
            if(bc_stat->afc_src_is_matched == true)
            {
               /* Increment AFC counters on match only */ 
               bc_stat->afc_src_msg_count++;
               if(bc_stat->afc_src_cur_match_byte == bc_stat->afc_src_last_match_byte)
                {
                    bc_stat->afc_src_match_count++;
                    if(bc_stat->afc_src_match_count == 2u)
                    {
                        bc_stat->afc_src_matched_byte = bc_stat->afc_src_cur_match_byte;
                    }
                }
            }
            bc_stat->afc_tx_active = (uint8_t)false;
            bc_stat->afc_src_last_match_byte = bc_stat->afc_src_cur_match_byte;
            bc_afc_src_evaluate_match(context);
            break;
        case BC_FSM_EVT_AFC_MSG_SEND_FAIL:

            /* If transmission was active, increment msg count. */
            if (bc_stat->afc_tx_active == (uint8_t)true)
            {
                bc_stat->afc_tx_active = (uint8_t)false;
                bc_stat->afc_src_msg_count++;
                bc_afc_src_evaluate_match(context);
            }
            else
            {
                /* This is a timeout event. Restart hardware state machine. */
                (void)Cy_USBPD_Bch_AfcSrcStart(context->ptrUsbPdContext);
            }
            break;
        default:
            /* Intentionally left empty */
            break;
    }
}
#endif /* (!(QC_SRC_AFC_CHARGING_DISABLED || QC_AFC_CHARGING_DISABLED)) */

#if (!(CY_PD_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)

#if BCR || QC_AFC_SNK_EN

#if (LEGACY_PD_PARALLEL_OPER)
static void pd_stop_on_bc_hv_detect(cy_stc_pdstack_context_t * context)
{
    cy_stc_pdstack_dpm_status_t* dpm_stat = &(context->dpmStat);
    if(false == dpm_stat->pdDisabled)
    {
        Cy_PdStack_Dpm_Disable(context);
    }
}
#endif /* LEGACY_PD_PARALLEL_OPER */

void bc_snk_try_next_protocol(cy_stc_pdstack_context_t * context)
{
    uint8_t c_port=context->port;
    bc_status_t *bc_stat = &gl_bc_status[c_port];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);

    /* Try AFC protocol if enabled. */
    if (chg_cfg->snkSel & BC_SINK_AFC_MODE_ENABLE_MASK)
    {
        if(bc_stat->cur_mode == BC_CHARGE_DCP)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_AFC_CHARGER_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
            bc_set_mode(context,BC_CHARGE_AFC);
#if (LEGACY_PD_PARALLEL_OPER)
            /* QC/AFC detection starts so stop DPM. */
            pd_stop_on_bc_hv_detect(context);
#endif /* LEGACY_PD_PARALLEL_OPER */
            return;
        }
     }

    /* Try QC 2.0 protocol if enabled . */
    if (
            (chg_cfg->snkSel & BC_SINK_QC_MODE_ENABLE_MASK) &&
            (bc_stat->max_amp <= QC2_MAX_CURRENT)
       )
    {
        if((bc_stat->cur_mode == BC_CHARGE_DCP)||(bc_stat->cur_mode == BC_CHARGE_AFC))
        {
                /* Try QC 2.0 protocol if enabled. */
                    bc_stat->bc_fsm_state = BC_FSM_SINK_QC_CHARGER_DETECTED;
                    bc_set_mode(context,BC_CHARGE_QC2);

                    bc_set_bc_evt (context, BC_EVT_ENTRY);
#if (LEGACY_PD_PARALLEL_OPER)
                    /* QC/AFC detection starts so stop DPM. */
                    pd_stop_on_bc_hv_detect(context);
#endif
                    return;
        }
    }

    /* Ensure VBUS is set to 5V. */
    psnk_set_voltage (context, CY_PD_VSAFE_5V);

    /* Try Apple protocol if enabled and BC1.2 current mismatch is still present. */
    if (
            (chg_cfg->snkSel & BC_SINK_APPLE_MODE_ENABLE_MASK) &&
            (bc_stat->max_amp > CY_PD_I_1P5A)
       )
    {
        bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
        bc_set_bc_evt (context, BC_EVT_ENTRY);
    }
    else
    {
        /* BC 1.2 DCP mode mismatch check. */
        if (
                (bc_stat->min_volt > CY_PD_VSAFE_5V) ||
                (bc_stat->max_amp > CY_PD_I_1P5A)
           )
        {
            bc_stop(context);
        }
        else
        {
            /* QC/AFC/Apple detections are disabled , stay in DCP mode. */
            /* DCP is connected. */
            bc_set_mode(context,BC_CHARGE_DCP);
            bc_psnk_enable(context);
            Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);
        }
    }
}

#endif /* BCR || QC_AFC_SNK_EN */

static void bc_fsm_sink_start(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t cport=context->port;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
    if (evt == BC_FSM_EVT_ENTRY)
    {

#if BCR || QC_AFC_SNK_EN
        Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
        cy_sw_timer_stop_range(context->ptrTimerContext, APP_BC_GENERIC_TIMER1, APP_BC_DETACH_DETECT_TIMER);
        bc_clear_bc_evt(context, BC_EVT_ALL_MASK);
#endif /* BCR || QC_AFC_SNK_EN */
        /* Set up CHGDET hardware block for operation. */
        Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);
        Cy_USBPD_Bch_Phy_En(context->ptrUsbPdContext);

#if (BCR) || QC_AFC_SNK_EN
        bc_set_mode(context,BC_CHARGE_NONE);

        /* Start BC1.2, if enabled. */
        if (chg_cfg->snkSel & BC_SINK_1_2_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_PRIMARY_CHARGER_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
        }
        /* Move to Apple charger detection state, if enabled. */
        else if (chg_cfg->snkSel & BC_SINK_APPLE_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
        }
        /* No legacy charging mode is enabled. Assume TYPE-C only source.Stop BC FSM. */
        else
        {
            bc_stop(context);
        }
#else
        bc_stat->cur_mode = BC_CHARGE_NONE;

#if (!APPLE_SINK_DISABLE)
        /* Move to Apple charger detection state, if enabled. */
        if (chg_cfg->snkSel & BC_SINK_APPLE_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
        }
        else
#endif /* (!APPLE_SINK_DISABLE) */
        {
            /* Start BC1.2, if enabled. */
            if (chg_cfg->snkSel & BC_SINK_1_2_MODE_ENABLE_MASK)
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_PRIMARY_CHARGER_DETECT;
                bc_set_bc_evt (context, BC_EVT_ENTRY);
            }
            /* No legacy charging mode is enabled. Assume TYPE-C only source. */
            else
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
                bc_set_bc_evt (context, BC_EVT_ENTRY);
            }
        }
#endif /* BCR || QC_AFC_SNK_EN */
    }
}

static void bc_fsm_sink_apple_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
 uint8_t c_port=context->port;
#if (!APPLE_SINK_DISABLE)
    bool apple_charger_detected = false;
    bc_status_t *bc_stat = &gl_bc_status[c_port];
#if (!BCR) && !QC_AFC_SNK_EN
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
#endif /* !BCR && !QC_AFC_SNK_EN */
    /*
     * CCG Sink needs to detect if it is connected to Apple charger or not.
     * Apple charger is expected to drive >1V on both D+/-. So measure D+/-
     * voltage and determine the type of charger.
     */
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apple RDAT_LKG resistors on D+ and D-. */
        Cy_USBPD_Bch_ApplyRdatLkgDp(context->ptrUsbPdContext);
        Cy_USBPD_Bch_ApplyRdatLkgDm(context->ptrUsbPdContext);

        /*
         * Need to review if a timer shall be used here instead of just checking current
         * voltage on D+/-.
         */

        /* Check if D+ > 1V */
        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
        {
            /* Check if D- > 1V. */
            if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
            {
                /* Apple charger detected. */
                apple_charger_detected = true;
            }
        }

        if (apple_charger_detected == true)
        {
            /* Now we now that CCG is connected to Apple charger. Detect Brick ID */
#if BCR || QC_AFC_SNK_EN
            bc_set_mode(context,BC_CHARGE_APPLE);
#else
            bc_stat->cur_mode = BC_CHARGE_APPLE;
#endif /* BCR || QC_AFC_SNK_EN */

            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_BRICK_ID_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
        }
        /* Apple charger not detected. */
#if BCR || QC_AFC_SNK_EN
        /* No legacy charging mode is enabled. Assume TYPE-C only source. */
        else
        {
            bc_stop(context);
        }
#else
        else
        {
            /* Start BC1.2, if enabled. */
            if (chg_cfg->snkSel & BC_SINK_1_2_MODE_ENABLE_MASK)
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_PRIMARY_CHARGER_DETECT;
                bc_set_bc_evt (context, BC_EVT_ENTRY);
            }
            /* No legacy charging mode is enabled. Assume TYPE-C only source. */
            else
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
                bc_set_bc_evt (context, BC_EVT_ENTRY);
            }
        }
#endif /* BCR || QC_AFC_SNK_EN */
    }
#else
    (void)c_port;
    (void)evt;
#endif /* (!APPLE_SINK_DISABLE) */
}

static void bc_fsm_sink_apple_brick_id_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
 uint8_t c_port=context->port;
#if (!APPLE_SINK_DISABLE)
    /* Detect Apple Brick ID here as required by Apple Brick ID spec. */

    /*
     * DP and DM can be connected to three terminations:
     * TERM1 : 1 - 2.22 V
     * TERM2 : 2.22 - 2.89 V
     * TERM3 : 2,.89+ V
     * Encoding user here is : TERM1 : 1, TERM2: 2, TERM3: 3.
     */
    bc_apple_term dp_term = APPLE_TERM1, dm_term = APPLE_TERM1;
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /*
         * We already know that DP is greater than 1V. Check if DP is greater than
         * 2.9V. If yes, we have term3 on DP. If not, check if DP is greater than 2.2V.
         * If yes, we have term2 on DP. Else, DP has term1.
         */

        /* Need to enable 2.9V detection for Apple brick ID. */
        Cy_USBPD_Bch_Enable_AppleDet(context->ptrUsbPdContext);

        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_2_9V, CHGB_COMP_NO_INTR) == true)
        {
            dp_term = APPLE_TERM3;
        }
        else
        {
            if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == true)
            {
                dp_term = APPLE_TERM2;
            }
        }

        /* Similar test for DM. */
        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_2_9V, CHGB_COMP_NO_INTR) == true)
        {
            dm_term = APPLE_TERM3;
        }
        else
        {
            if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == true)
            {
                dm_term = APPLE_TERM2;
            }
        }
        /* Disable 2.9V detection for Apple brick ID. */
        Cy_USBPD_Bch_Disable_AppleDet(context->ptrUsbPdContext);

        /* Evaluate Apple termination detected. */
        bc_eval_apple_brick_id (context, dp_term | (dm_term << 0x04));

    }
#else
    (void)c_port;
    (void)evt;
#endif /* (!APPLE_SINK_DISABLE) */
}

static void bc_fsm_sink_primary_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port=context->port;
#if BCR || QC_AFC_SNK_EN
    const cy_stc_legacy_charging_cfg_t *chg_cfg = bc_get_config(context);
#endif /* BCR || QC_AFC_SNK_EN */
    bc_status_t* bc_stat = &gl_bc_status[c_port];
    /* This state is for primary charger detect. Refer BC 1.2 spec for details. */
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apply terminations on D+/-. */
        Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_PCD);
        cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1, APP_BC_VDP_DM_SRC_ON_PERIOD, bc_tmr_cbk);
    }
    else if(evt == BC_FSM_EVT_TIMEOUT1)
    {
        /* Now measure D- and see if D- is pulled up to VDP_SRC. */
        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_0_325V, CHGB_COMP_NO_INTR) == true)
        {
#if BCR || QC_AFC_SNK_EN
            if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
            {
                /* Check if D- > 1V. */
                if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
                {
                    /* Apple charger detected. */
                    /* Move to Apple charger detection state, if enabled. */
                    if (chg_cfg->snkSel & BC_SINK_APPLE_MODE_ENABLE_MASK)
                    {
                        bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
                        bc_set_bc_evt (context, BC_EVT_ENTRY);
                    }
                    else
                    {
                        /* TYPE-C only source connected. */
                        bc_stop(context);
                    }
                }
            }
            else
#endif /* BCR || QC_AFC_SNK_EN */
            {
                /* Start timer for source to diffrentiate between primary and secondary detection */
                cy_sw_timer_start(context->ptrTimerContext,context, APP_BC_GENERIC_TIMER2, APP_BC_VDMSRC_EN_DIS_PERIOD, bc_tmr_cbk);
            }
        }
#if BCR || QC_AFC_SNK_EN
        /* Apple pull-ups are not seen now but anyway move to Apple charger detection state, if enabled. */
        else if (chg_cfg->snkSel & BC_SINK_APPLE_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
        }
#endif /* BCR || QC_AFC_SNK_EN */
        else
        {
            /* TYPE-C only source connected. */
#if BCR || QC_AFC_SNK_EN
            bc_stop(context);
#else
            bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
            bc_set_bc_evt (context, BC_EVT_ENTRY);
#endif /* BCR || QC_AFC_SNK_EN */
        }
        /* Remove applied terminations */
        Cy_USBPD_Bch_Phy_RemoveTerm(context->ptrUsbPdContext);
    }
    else if (evt == BC_FSM_EVT_TIMEOUT2)
    {
        /* Proceed to secondary detection for CDP/DCP detection. */
        bc_stat->bc_fsm_state = BC_FSM_SINK_SECONDARY_CHARGER_DETECT;
        bc_set_bc_evt (context, BC_EVT_ENTRY);
    }
}

static void bc_fsm_sink_type_c_only_source_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    /* Do Nothing */
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    uint8_t c_port=context->port;
    (void) c_port;
    (void) evt;

    Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);

#if BCR || QC_AFC_SNK_EN
    bc_psnk_enable(context);
#endif /* BCR || QC_AFC_SNK_EN */

#if ((defined(CCG5C)) || (defined(CCG6)))
    /* Enable DP/DM Mux if not done so far. */
    Cy_USBPD_Mux_ConfigDpDm(context->ptrUsbPdContext,
            ptrDpmConfig->polarity ? CY_USBPD_DPDM_MUX_CONN_USB_BOT : CY_USBPD_DPDM_MUX_CONN_USB_TOP);
#endif /* ((defined(CCG5C)) || (defined(CCG6))) */

#if CCG_HPI_BC_12_ENABLE
        gl_bc_12_status[c_port] = BC_SINK_SDP_CONNECTED;
#endif /* CCG_HPI_BC_12_ENABLE */ 

}

static void bc_fsm_sink_secondary_charger_detect(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port=context->port;
    bc_status_t* bc_stat = &gl_bc_status[c_port];

    /*
     * This state is used to perform secondary charger detect. Refer BC 1.2 spec
     * for details.
     */

    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apply terminations on D+/-. */
        Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_SCD);
        /* Start timer to apply VDM_SRC for TVDM_SRC_ON */
        cy_sw_timer_start(context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1, APP_BC_VDP_DM_SRC_ON_PERIOD, bc_tmr_cbk);
    }
    else if(evt == BC_FSM_EVT_TIMEOUT1)
    {
        /* Now measure D+ and see if D- is pulled up to VDM_SRC. */
        if (Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext,BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_0_325V, CHGB_COMP_NO_INTR) == true)
        {
            /* DCP connected. */
#if BCR || QC_AFC_SNK_EN
            bc_set_mode(context, BC_CHARGE_DCP);
#else
            bc_stat->cur_mode = BC_CHARGE_DCP;
#endif /* BCR || QC_AFC_SNK_EN */
            bc_stat->bc_fsm_state = BC_FSM_SINK_DCP_CONNECTED;
        }
        else
        {
            /* CDP connected. */
#if BCR || QC_AFC_SNK_EN
            bc_set_mode(context, BC_CHARGE_CDP);
#else
            bc_stat->cur_mode = BC_CHARGE_CDP;
#endif /* BCR || QC_AFC_SNK_EN */
            bc_stat->bc_fsm_state = BC_FSM_SINK_CDP_CONNECTED;
        }
        bc_set_bc_evt (context, BC_EVT_ENTRY);
        
#if CCG_HPI_BC_12_ENABLE        
        gl_bc_12_status[c_port] = BC_SINK_SEC_CHARGER_DETECT;
#endif /* CCG_HPI_BC_12_ENABLE */        

        /* Remove applied terminations */
        Cy_USBPD_Bch_Phy_RemoveTerm(context->ptrUsbPdContext);
    }
}

#if BCR || QC_AFC_SNK_EN
static void bc_fsm_sink_dcp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port=context->port;
    bc_status_t* bc_stat = &gl_bc_status[c_port];

    switch (evt)
    {
        case BC_FSM_EVT_ENTRY:
            /*
             * We know that DCP is connected. If charger supports QC it will open
             * the D+/- short due to this D- will fall below VDP_SRC.
             */

            /* Put back VDP_SRC and IDM_SINK to measure D-. */
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_PCD);
            /* Set up the comparator to monitor D- and see if it is pulled down. */
            Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                    CHGB_VREF_0_325V, CHGB_COMP_EDGE_FALLING);
            /*
             * Start TGLITCH_BC_DONE (1.5s). If D- is pulled low within this time,
             * then charger is QC/AFC. Otherwise it is DCP.
             */
            cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1,
                APP_BC_GLITCH_BC_DONE_TIMER_PERIOD, bc_tmr_cbk);
            bc_stat->cur_timer = BC_SINK_TIMER_BC_DONE;

            break;

        case BC_FSM_EVT_TIMEOUT1:
            if (bc_stat->cur_timer == BC_SINK_TIMER_BC_DONE)
            {
                /* DCP didn't remove D+/- short. So charger does not support QC.Stay in DCP mode */

                /* Set the battery charging current to 1.5A. */
                psnk_set_current (context, CY_PD_I_1P5A);
                bc_psnk_enable(context);
            }
            else if (bc_stat->cur_timer == BC_SINK_TIMER_DM_HIGH)
            {
                /*
                 * D+/- short removed and D- sampled for T_GLITCH_DM_HIGH
                 * timer. QC/AFC charger detected.
                 */
                bc_snk_try_next_protocol(context);
            }
            Cy_USBPD_Bch_Phy_DisableComp(context->ptrUsbPdContext, BC_CMP_0_IDX);
            break;

        case BC_FSM_EVT_CMP1_FIRE:
            if (bc_stat->cur_timer == BC_SINK_TIMER_BC_DONE)
            {
                /*
                 * DCP removed D+/- short. Sink is expected to debounce D- for
                 * T_GLITCH_DM_HIGH (40ms) before making VBUS request. Set up the
                 * comparator and timer.
                 */
                cy_sw_timer_stop (context->ptrTimerContext,APP_BC_GENERIC_TIMER1);
                Cy_USBPD_Bch_Phy_Config_Comp(context->ptrUsbPdContext, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                    CHGB_VREF_0_325V, CHGB_COMP_EDGE_RISING);
                cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1,
                    APP_BC_GLITCH_DM_HIGH_TIMER_PERIOD, bc_tmr_cbk);
                bc_stat->cur_timer = BC_SINK_TIMER_DM_HIGH;
            }
            else if (bc_stat->cur_timer == BC_SINK_TIMER_DM_HIGH)
            {
                /* D- wasn't held low for T_GLITCH_DM_HIGH. */
                cy_sw_timer_stop (context->ptrTimerContext, APP_BC_GENERIC_TIMER1);
            }
            break;

        default:
            break;
    }
}
#else
static void bc_fsm_sink_dcp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* DCP is connected. */
        psnk_set_current (context, CY_PD_I_1P5A);
        Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);

#if ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
        /* Enable DP/DM Mux if not done so far. */
        Cy_USBPD_Mux_ConfigDpDm(context->ptrUsbPdContext,
                ptrDpmConfig->polarity ? CY_USBPD_DPDM_MUX_CONN_USB_BOT : CY_USBPD_DPDM_MUX_CONN_USB_TOP);
#endif /* ((defined(CCG5C)) || (defined(CCG6))) */

        /* Turn On the sink fet */
        psnk_enable(context);

#if CCG_HPI_BC_12_ENABLE
        gl_bc_12_status[c_port] = BC_SINK_DCP_CONNECTED;
        hpi_queue_bc_evt(c_port);
#endif /* CCG_HPI_BC_12_ENABLE */

    }
}
#endif /* BCR || QC_AFC_SNK_EN */

static void bc_fsm_sink_sdp_connected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    uint8_t c_port=context->port;
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* SDP is connected. */
#if BCR || QC_AFC_SNK_EN
#if !CY_PD_SINK_ONLY
        psnk_set_current (context, CY_PD_ISAFE_DEF);
        bc_psnk_enable(context);
#else
        /* In case of BCR2 use higher TypeC current limit. */
        bc_stop(context);
#endif /* !CY_PD_SINK_ONLY  */
#else
        psnk_set_current (context, CY_PD_ISAFE_DEF);
#endif /* BCR || QC_AFC_SNK_EN */
        Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);

#if ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
        /* Enable DP/DM Mux if not done so far. */
        Cy_USBPD_Mux_ConfigDpDm(context->ptrUsbPdContext,
                ptrDpmConfig->polarity ? CY_USBPD_DPDM_MUX_CONN_USB_BOT : CY_USBPD_DPDM_MUX_CONN_USB_TOP);
#endif /* ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6))) */

    }
}

static void bc_fsm_sink_cdp_connected(cy_stc_pdstack_context_t* context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port=context->port;
    cy_stc_pd_dpm_config_t * ptrDpmConfig = &(context->dpmConfig);
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* CDP is connected. */
        /* Set the battery charging current to 1.5A. */
        psnk_set_current (context, CY_PD_I_1P5A);
        Cy_USBPD_Bch_Phy_Dis(context->ptrUsbPdContext);

#if ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6)))
        /* Enable DP/DM Mux if not done so far. */
        Cy_USBPD_Mux_ConfigDpDm(context->ptrUsbPdContext,
                ptrDpmConfig->polarity ? CY_USBPD_DPDM_MUX_CONN_USB_BOT : CY_USBPD_DPDM_MUX_CONN_USB_TOP);
#endif /* ((defined(CY_DEVICE_CCG5C)) || (defined(CY_DEVICE_CCG6))) */
    }
    
    /* Turn On the sink fet */
    psnk_enable(context);
       
#if CCG_HPI_BC_12_ENABLE
    gl_bc_12_status[c_port] = BC_SINK_CDP_CONNECTED;
    hpi_queue_bc_evt(c_port);
#endif /* CCG_HPI_BC_12_ENABLE */    
}

#if BCR || QC_AFC_SNK_EN
static uint8_t bc_sink_calculate_required_current(cy_stc_pdstack_context_t* context, uint16_t cur10mA)
{
    uint8_t c_port = context->port;
    bc_status_t* bc_stat = &gl_bc_status[c_port];
    uint32_t new_current;
    /* Convert current to 10mA units */
    new_current = cur10mA;

    if(new_current < AFC_BASE_AMP)
    {
        /* AFC Min current is 750mA */
        new_current = AFC_BASE_AMP;
    }
    /* AFC protocol defines max. charging current 3A */
    return ((CY_USBPD_GET_MIN(((new_current - AFC_BASE_AMP)/AFC_AMP_STEP), AFC_MAX_AMP))  & 0x0F);
}

static void bc_fsm_sink_afc_charger_detect(cy_stc_pdstack_context_t* context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port = context->port;
    bc_status_t* bc_stat = &gl_bc_status[c_port];
    uint16_t vbus = 0;
    uint8_t i;
    uint8_t rcvd_vi;
    bool new_VI_BYTE_flag = false;

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            /* Set AFC sink termination. */
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext,CHGB_SINK_TERM_AFC);

            Cy_USBPD_Bch_AfcSinkInit(context->ptrUsbPdContext);
            /* Form initial request. */
            /* Set Vmax from rotary switch at first */
            bc_stat->afc_snk_cur_vi_byte = (((((bc_stat->max_volt - AFC_BASE_VOLT)/AFC_VOLT_STEP) << 4 ) & 0xF0) |
                                    bc_sink_calculate_required_current(context, bc_stat->max_amp));

            psnk_set_voltage (context, bc_stat->max_volt);
            bc_stat->cur_amp = CY_USBPD_GET_MIN(bc_stat->max_amp, CY_PD_I_3A);
            psnk_set_current (context, CY_USBPD_GET_MIN(bc_stat->max_amp, CY_PD_I_3A));

            Cy_USBPD_Bch_Set_AfcTxData(context->ptrUsbPdContext, &bc_stat->afc_snk_cur_vi_byte, 1);
            /* Enable AFC interrupt, init rx buffer and start transaction. */
            Cy_USBPD_Bch_AfcSinkStart(context->ptrUsbPdContext);

            bc_stat->afc_retry_count = 0;
            cy_sw_timer_start(context->ptrTimerContext,context,APP_BC_GENERIC_TIMER2, APP_BC_AFC_SNK_VI_BYTE_PERIOD, bc_tmr_cbk);
            break;

        case BC_FSM_EVT_TIMEOUT2:
            /* Check received data in buffer */
            if(Cy_USBPD_Bch_AfcGetRxDataCount(context->ptrUsbPdContext) > 0)
            {
                /* Check if sent and received VI_BYTE are equal. */
                rcvd_vi = *Cy_USBPD_Bch_AfcGetRxDataPtr(context->ptrUsbPdContext);
                if(rcvd_vi == bc_stat->afc_snk_cur_vi_byte)
                {
                    vbus = vbus_get_value(context);
                    uint16_t vbus_vi = ((rcvd_vi >> 4) * AFC_VOLT_STEP) + AFC_BASE_VOLT;
                    /* Check if required VBUS voltage is set with +/- 5% tolerance */
                    if((vbus > (vbus_vi - div_round_up(vbus_vi, 20))) && (vbus < (vbus_vi + div_round_up(vbus_vi, 20))))
                    {
                        /* VBUS is set as required, send ping with the same VI_BYTE */

                        if(bc_mismatch_check(context) == false)
                        {
                            if(bc_stat->connected == false)
                            {
                                bc_stat->cur_volt = vbus_vi;
                                bc_set_mode(context,BC_CHARGE_AFC);
                                bc_psnk_enable(context);
                            }
                        }
                        else
                        {
                            Cy_USBPD_Bch_AfcSinkStop(context->ptrUsbPdContext);
                            bc_stat->connected = false;
                            bc_stat->cur_mode = BC_CHARGE_AFC;
                            /* No suitable capabilities were found. Try QC mode. */
                            bc_snk_try_next_protocol(context);
                            break;
                        }
                    }
                    else
                    {
                        /* Do nothing. SRC awaits on 3 equal VI_BYTEs, so send VI_BYTE again.  */
                    }
                }
                else
                {
                    /* AFC SRC capabilities are received , choose new VI_BYTE */
                    /* Do reverse scan as AFC capabilities are ordered from the lowest to the highest voltages. */
                    for( i= (Cy_USBPD_Bch_AfcGetRxDataCount(context->ptrUsbPdContext)) ; i > 0 ; i-- )
                    {
                        /* At first check if provided voltage is suitable. */
                        if((bc_stat->afc_snk_cur_vi_byte & 0xF0) >= (*(Cy_USBPD_Bch_AfcGetRxDataPtr(context->ptrUsbPdContext) + (i-1)) & 0xF0))
                        {
                            /* if voltages are equal check if provided current is suitable */
                            if(bc_sink_calculate_required_current(context, bc_stat->min_amp) <= (*(Cy_USBPD_Bch_AfcGetRxDataPtr(context->ptrUsbPdContext) + (i-1)) & 0x0F))
                            {
                                uint16_t vbus_vi = ((*(Cy_USBPD_Bch_AfcGetRxDataPtr(context->ptrUsbPdContext) + (i-1)) >> 4) * AFC_VOLT_STEP) + AFC_BASE_VOLT;

                                /* Get next AFC SRC capability byte if provided voltage is less than minimally required. */
                                if(vbus_vi < bc_stat->min_volt)
                                {
                                    continue;
                                }

                                psnk_set_voltage (context, vbus_vi);
                                bc_stat->afc_snk_cur_vi_byte = *(Cy_USBPD_Bch_AfcGetRxDataPtr(context->ptrUsbPdContext) + (i-1));
                                new_VI_BYTE_flag = true;
                                break;
                            }
                        }
                        else
                        {
                            /* Try the next capability from the list. */
                        }
                    }
                    if(new_VI_BYTE_flag == false)
                    {
                        Cy_USBPD_Bch_AfcSinkStop(context->ptrUsbPdContext);
                        bc_stat->connected = false;
                        /* No suitable capabilities were found. Try QC mode. */
                        bc_set_mode(context,BC_CHARGE_AFC);
                        /* Try Apple protocol in case of mismatch or stay in DCP */
                        bc_snk_try_next_protocol(context);
                        break;
                    }
                }
                /* Send old or BYTE_VI */
                Cy_USBPD_Bch_Set_AfcTxData(context->ptrUsbPdContext, &bc_stat->afc_snk_cur_vi_byte, 1);
                Cy_USBPD_Bch_AfcSinkStart(context->ptrUsbPdContext);
                cy_sw_timer_start(context->ptrTimerContext,context, APP_BC_GENERIC_TIMER2, APP_BC_AFC_SNK_VI_BYTE_PERIOD, bc_tmr_cbk);
            }
            else
            {
                /* Nothing was received from SRC. Do retry. */
                bc_stat->afc_retry_count ++;
                if(bc_stat->afc_retry_count < AFC_DETECT_RETRY_COUNT)
                {
                    Cy_USBPD_Bch_Set_AfcTxData(context->ptrUsbPdContext, &bc_stat->afc_snk_cur_vi_byte, 1);
                    Cy_USBPD_Bch_AfcSinkStart(context->ptrUsbPdContext);
                    cy_sw_timer_start(context->ptrTimerContext,context, APP_BC_GENERIC_TIMER2, APP_BC_AFC_SNK_VI_BYTE_PERIOD, bc_tmr_cbk);
                }
                else
                {
                    Cy_USBPD_Bch_AfcSinkStop(context->ptrUsbPdContext);
                    bc_stat->connected = false;
                    /* AFC charger is not found. Try QC mode. */
                    bc_set_mode(context,BC_CHARGE_AFC);
                    /* Try Apple protocol in case of mismatch or stay in DCP */
                    bc_snk_try_next_protocol(context);
                    break;
                }
            }
            break;

        /* Nothing to handle. Wait on timeout event. */
        case BC_FSM_EVT_AFC_MSG_SENT:
        case BC_FSM_EVT_AFC_MSG_RCVD:
            break;

        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            Cy_USBPD_Bch_AfcSinkStop(context->ptrUsbPdContext);
            bc_stop(context);

            break;

        case BC_FSM_EVT_AFC_RESET_RCVD:
            /* AFC Reset. Treat as failure and try next BC protocol. */
            Cy_USBPD_Bch_AfcSinkStop(context->ptrUsbPdContext);
            psnk_set_voltage(context, CY_PD_VSAFE_5V);
            bc_stat->cur_volt = CY_PD_VSAFE_5V;

            bc_set_mode(context,BC_CHARGE_AFC);

            bc_snk_try_next_protocol(context);
            break;

        default:
            break;

    }
}

static void bc_apply_sink_term(cy_stc_pdstack_context_t * context, uint16_t voltage)
{
    switch (voltage)
    {
        case CY_PD_VSAFE_9V:
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_QC_9V);
            break;
        case CY_PD_VSAFE_12V:
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_QC_12V);
            break;
        case CY_PD_VSAFE_20V:
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_QC_20V);
            break;
        default:
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(context->ptrUsbPdContext, CHGB_SINK_TERM_QC_5V);
            break;
    }
}

static void bc_fsm_sink_qc_charger_detected(cy_stc_pdstack_context_t * context, cy_en_pdstack_bc_fsm_evt_t evt)
{
    uint8_t c_port=context->port;
    bc_status_t* bc_stat = &gl_bc_status[c_port];
    uint16_t vbus = 0;

    switch (evt)
    {
        case BC_FSM_EVT_ENTRY:
            /* QC charger detected. Ask for Vbus max.*/
            /* QC does not support 15V request */
            if(bc_stat->max_volt == CY_PD_VSAFE_15V)
            {
                if(bc_stat->min_volt < CY_PD_VSAFE_15V)
                {
                    bc_stat->requested_qc_volt = CY_PD_VSAFE_12V;
                }
                else
                {
                    bc_stat->connected = false;
                    /* Try Apple protocol in case of mismatch or stay in DCP */
                    bc_snk_try_next_protocol(context);
                }
            }
            else
            {
                bc_stat->requested_qc_volt = bc_stat->max_volt;
            }

            bc_apply_sink_term(context, bc_stat->requested_qc_volt);
            psnk_set_voltage (context, bc_stat->requested_qc_volt);
            /* Now wait for T_V_NEW_REQUEST and then check new voltage. */
            cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1,
                APP_BC_V_NEW_REQUEST_TIMER_PERIOD, bc_tmr_cbk);
            break;

        case BC_FSM_EVT_TIMEOUT1:
            /*
             * T_V_NEW_REQUEST timer timed out. Ask for higher VBUS if current
             * VBUS is 5V.
             */
            vbus = vbus_get_value(context);

            /* Check if required VBUS voltage is set with +/- 5% tolerance */
            if((vbus > (bc_stat->requested_qc_volt - div_round_up(bc_stat->requested_qc_volt, 20))) &&
                vbus < (bc_stat->requested_qc_volt + div_round_up(bc_stat->requested_qc_volt, 20)))
            {
                /* VBUS is present.Enable PSink. */
                psnk_set_current (context, CY_USBPD_GET_MIN(bc_stat->max_amp, CY_PD_I_1P5A));
                bc_set_mode(context,BC_CHARGE_QC2);

                if(bc_mismatch_check(context) == false)
                {
                    if(bc_stat->connected == false)
                    {
                        bc_psnk_enable(context);
                    }
                }
                else
                {
                    bc_stat->connected = false;
                    /* Try Apple protocol in case of mismatch or stay in DCP */
                    bc_snk_try_next_protocol(context);
                }
            }
            else
            {
                /* No requested VBUS from charger. Try to request lower Vbus. */
                switch (bc_stat->requested_qc_volt)
                {
                    case CY_PD_VSAFE_20V:
                        bc_stat->requested_qc_volt = CY_PD_VSAFE_12V;
                        break;
                    case CY_PD_VSAFE_12V:
                        bc_stat->requested_qc_volt = CY_PD_VSAFE_9V;
                        break;
                    case CY_PD_VSAFE_9V:
                        bc_stat->requested_qc_volt = CY_PD_VSAFE_5V;
                        break;
                    default:
                        bc_stat->requested_qc_volt = CY_PD_VSAFE_5V;
                        break;
                }

                if(bc_stat->min_volt > bc_stat->requested_qc_volt)
                {
                    bc_stat->connected = false;
                    /* Try Apple protocol in case of mismatch or stay in DCP */
                    bc_snk_try_next_protocol(context);
                    break;
                }

                /* Try to request lower Vbus from charger */
                psnk_set_voltage (context, bc_stat->requested_qc_volt);
                bc_apply_sink_term(context, bc_stat->requested_qc_volt);
                /* Start timer again */
                /* Now wait for T_V_NEW_REQUEST and then check new voltage. */
                cy_sw_timer_start (context->ptrTimerContext,context,APP_BC_GENERIC_TIMER1,
                    APP_BC_V_NEW_REQUEST_TIMER_PERIOD, bc_tmr_cbk);
            }
            break;
        default:
            break;
    }
}
#endif /* BCR || QC_AFC_SNK_EN */
#endif /* (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */

/* FSM functions end */
#endif /* (!defined(CY_DEVICE_CCG5) */

#if ((!LEGACY_DYN_CFG_ENABLE) || (QC_AFC_CHARGING_DISABLED))

void bc_afc_form_vi(cy_stc_pdstack_context_t * context)
{
     /* DO NOTHING */
    CY_UNUSED_PARAMETER(context);
}

#endif /* ((!LEGACY_DYN_CFG_ENABLE) || (QC_AFC_CHARGING_DISABLED)) */

#if QC_AFC_CHARGING_DISABLED
void qc_set_cf_limit(cy_stc_pdstack_context_t * context)
{
    /* DO NOTHING */
    CY_UNUSED_PARAMETER(context);
}

#endif /* QC_AFC_CHARGING_DISABLED */

#else /* !BATTERY_CHARGING_ENABLE */
/* Dummy definitions to suppress the undefined reference for libraries */
const cy_stc_legacy_charging_cfg_t *bc_get_config(cy_stc_pdstack_context_t * context)
{
    (void)context;
    return NULL;
}

cy_en_pdstack_status_t bc_stop(cy_stc_pdstack_context_t * context)
{
    (void)context;    
    return CY_PDSTACK_STAT_SUCCESS;
}

bool bc_is_active(cy_stc_pdstack_context_t * context)
{
    (void)context;    
    return false;
}

cy_en_pdstack_status_t bc_set_config(cy_stc_pdstack_context_t * context, cy_stc_legacy_charging_cfg_t *chg_cfg)
{
    (void)context;
    (void)chg_cfg;
    return CY_PDSTACK_STAT_FAILURE;
}

uint8_t ccg_get_system_max_pdp(cy_stc_pdstack_context_t * context)
{
    (void)context;    
    return 45;
}

void qc_set_cf_limit(cy_stc_pdstack_context_t * context)
{
    (void)context;
}

void bc_afc_form_vi(cy_stc_pdstack_context_t * context)
{
    (void)context;
}

const bc_status_t* bc_get_status(cy_stc_pdstack_context_t * context)
{
    (void)context;    
    return NULL;
}

uint8_t bc_12_get_status(cy_stc_pdstack_context_t * context)
{ 
    (void)context;
    return 0;
}
#endif /* BATTERY_CHARGING_ENABLE */

/* End of File */
