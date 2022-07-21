/***************************************************************************//**
* \file vdm_task_mngr.c
* \version 1.1.0 
*
* VDM task manager source file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <vdm.h>
#include "cy_pdstack_common.h"
#include <app.h>
#include "vdm_task_mngr.h"
#if (CCG_HPI_ENABLE)
#include <hpi.h>
#endif /* (CCG_HPI_ENABLE) */
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#if CCG_USB4_SUPPORT_ENABLE
#include <intel_ridge_internal.h>
#include <intel_vid.h>
#endif /* CCG_USB4_SUPPORT_ENABLE */

/**
 * @typedef vdm_task_status_t
 * @brief struct to hold vdm manager status
 */
typedef struct
{
    /* Current VDM manager state */
    vdm_task_t      vdm_task;
    /* Current VDM manager event */
    vdm_evt_t       vdm_evt;
    /* Info about cable, device/AMA */
    atch_tgt_info_t atch_tgt;
    /* Struct with received/sent VDM info */
    vdm_msg_info_t  vdm_msg;
    /* Sent/received VDM retry counters */
    uint8_t         rec_retry_cnt;
    /* Holds svid index if we have more than one Disc SVID response */
    uint8_t         svid_idx;
    /* Holds count of D_SVID requests sent. */
    uint8_t         dsvid_cnt;
#if CCG_USB4_SUPPORT_ENABLE
    /* Enter USB data object. */
    dpm_pd_cmd_buf_t eudo_buf;
    /* Flag indicating USB4 status. */
    usb4_flag_t      usb4_flag;
    /* Ridge/SoC configuration status. */
    ridge_reg_t      intel_reg;
    /* Alt modes supported flag */
    uint8_t          alt_modes_not_supp;
#endif /* CCG_USB4_SUPPORT_ENABLE */
}vdm_task_status_t;

#if VDM_RESP_QUERY_SUPPORTED

#define MAX_DISC_ID_RESP_LEN    (MAX_NO_OF_DO)
#define MAX_DISC_SVID_RESP_LEN  (18u)

/* Array to store the D_ID response received from the port partner. */
static uint32_t vdm_disc_id_resp[NO_OF_TYPEC_PORTS][MAX_DISC_ID_RESP_LEN];
static uint8_t  vdm_disc_id_resp_len[NO_OF_TYPEC_PORTS];

/* Array to store the D_SVID response received from the port partner. */
static uint32_t vdm_disc_svid_resp[NO_OF_TYPEC_PORTS][MAX_DISC_SVID_RESP_LEN];
static uint8_t  vdm_disc_svid_resp_len[NO_OF_TYPEC_PORTS];

#endif /* VDM_RESP_QUERY_SUPPORTED */

/* Main structure to hold variables for VDM task manager */
vdm_task_status_t vdm_stat[NO_OF_TYPEC_PORTS];

/* Init VDM task mngr */
static vdm_task_t vdm_task_mngr_init(uint8_t port);

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
/* Discover ID cmd handler */
static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt);
/* Discovery SVID cmd handler */
static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
#if CCG_USB4_SUPPORT_ENABLE
static vdm_task_t usb4_tbt_handler(uint8_t port, vdm_evt_t vdm_evt);
static pd_do_t* get_eudo(uint8_t port);
static vdm_task_t resume_vdm_handler(uint8_t port);
#endif /* CCG_USB4_SUPPORT_ENABLE */
/* Sets recieved VDM response command as failed */
static void set_vdm_failed(uint8_t port, fail_status_t fail_stat);
/* Check VDM retries */
static bool vdm_retry_check(uint8_t port, fail_status_t fail_stat);
/* Parses received VDM resp to vdm_msg_info_t struct */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm);
/* Callback function for sent VDM */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t *rec_vdm);
/* Function to send VDM */
static vdm_task_t send_vdm(uint8_t port);
/* Checks if VDM mngr is enabled */
static bool is_vdm_mngr_enabled(uint8_t port);
/* Returns pointer to vdm_msg_info_t struct */
static vdm_msg_info_t* get_msg(uint8_t port);
/* Composes vdm_msg_info_t data to dpm_pd_cmd_buf_t struct */
static uint8_t compose_vdm(uint8_t port);
/* Reset vdm mngr info */
static void reset_vdm_mngr(uint8_t port);

/* VDM field set/get functions. */
vdm_evt_t get_evt(uint8_t port);
vdm_task_t get_task(uint8_t port);
void set_task(uint8_t port, vdm_task_t task);
void set_evt(uint8_t port, vdm_evt_t evt);
void set_disc_param(uint8_t port, uint8_t sop, std_vdm_cmd_t cmd);

/**
   @typedef cable_dp_reset_state_t
   @brief Enumeration tracking current cable (SOP'') soft reset state.
 */
typedef enum {
    CABLE_DP_RESET_IDLE = 0,    /**< No Cable (SOP'') reset state. */
    CABLE_DP_RESET_WAIT,        /**< Waiting for response from cable (SOP'') soft reset. */
    CABLE_DP_RESET_RETRY,       /**< Cable soft reset (SOP'') to be attempted. */
    CABLE_DP_RESET_DONE,         /**< Cable soft reset (SOP'') has been completed. */
    CABLE_DP_RESET_SEND_SPRIME  /**< Cable soft reset (SOP'') has been completed and send SOP Prime msg again. */
} cable_dp_reset_state_t;

/* Maximum number of EMCA SOFT_RESET attempts to be made. */
#define MAX_EMCA_DP_RESET_COUNT         (3u)

static cable_dp_reset_state_t vdm_emca_rst_state[NO_OF_TYPEC_PORTS] = {
    CABLE_DP_RESET_IDLE
#if (NO_OF_TYPEC_PORTS > 1)
    ,
    CABLE_DP_RESET_IDLE
#endif /* (NO_OF_TYPEC_PORTS > 1) */
};

static uint8_t vdm_emca_rst_count[NO_OF_TYPEC_PORTS] = {
    0
#if (NO_OF_TYPEC_PORTS > 1)
    ,
    0
#endif /* (NO_OF_TYPEC_PORTS > 1) */
};

void enable_vdm_task_mngr(uint8_t port)
{
    /* If the VDM Task is already running, do nothing. */
    if (app_get_status(port)->vdm_task_en == false)
    {
        set_task(port, VDM_TASK_INIT);
        app_get_status(port)->vdm_task_en = true;
        app_get_status(port)->vdm_prcs_failed = false;

        /* Make sure all alternate mode manager state has been cleared at start. */
        alt_mode_mngr_reset_info (port);

#if HPI_AM_SUPP
        /* Check if there any custom alt mode available and save it. */
        app_get_status(port)->custom_hpi_svid = get_custom_svid(port);
#endif /* HPI_AM_SUPP */

#if DFP_ALT_MODE_SUPP
#if (DEFER_VCONN_SRC_ROLE_SWAP)
        if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
        {
            /* We are DFP, don't relinquish VConn Source role until Alt. Mode processing is complete. */
            app_get_status(port)->keep_vconn_src = true;
        }
        else
        {
            /* If we are UFP, we can allow the port partner to become VConn source. */
            app_get_status(port)->keep_vconn_src = false;
        }
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
#endif /* DFP_ALT_MODE_SUPP */
    }
}

static vdm_task_t vdm_task_mngr_init(uint8_t port)
{
    vdm_task_t ret = VDM_TASK_EXIT;

    /* Reset vdm mngr info */
    reset_vdm_mngr(port);
    set_evt(port, VDM_EVT_RUN);

#if CCG4_DOCK
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        /* in CCG4 Dock US is UFP only, so it should not init D_ID */
        if (port == 0u)
            return VDM_TASK_EXIT;
    }
#endif /* CCG4_DOCK */

    if (get_alt_mode_numb(port) != false)
    {
        /* Check if current data role is DFP */
        if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
        {
            ret = VDM_TASK_DISC_ID;
    }
        else
        {
            ret = VDM_TASK_REG_ATCH_TGT_INFO;
        }
    }

    return ret;

}

bool is_vdm_task_idle(uint8_t port)
{
    /*
       If VDM manager is enabled, check
       1. Whether BUSY timer is running.
       2. Whether the ALT mode tasks are idle.
     */
    return !(is_vdm_mngr_enabled (port) &&
            ((timer_is_running (port, APP_VDM_BUSY_TIMER)) ||
             (get_task(port) != VDM_TASK_ALT_MODE) ||
             (is_alt_mode_mngr_idle(port) == false))
            );
}

void vdm_task_mngr(uint8_t port)
{
    /* The VDM task enabled check is performed by the caller. */
    if (
            (timer_is_running(port, APP_VDM_BUSY_TIMER) == true)
#if MUX_DELAY_EN
            || (app_get_status(port)->is_mux_busy != false)
#endif /* MUX_DELAY_EN */
#if CCG_USB4_SUPPORT_ENABLE
            || (vdm_stat[port].usb4_flag == USB4_PENDING)
#endif /* CCG_USB4_SUPPORT_ENABLE */
       )
    {
        return;
    }
    /* Check if cable reset is required */
    if (app_get_status(port)->trig_cbl_rst != false)
    {
        if (dpm_pd_command (port, DPM_CMD_SEND_CABLE_RESET, NULL, NULL) == CCG_STAT_SUCCESS)
        {
            timer_start (port, APP_VDM_BUSY_TIMER, APP_CABLE_VDM_START_DELAY, NULL);
            app_get_status(port)->trig_cbl_rst = false;
        }
        return;
    }

    /* Get current vdm task */
    switch (get_task(port))
    {
        case VDM_TASK_WAIT:
            break;

        case VDM_TASK_INIT:
            set_task(port, vdm_task_mngr_init(port));
            break;

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        case VDM_TASK_DISC_ID:
            set_task(port, vdm_task_mng_disc_id(port, get_evt(port)));
            break;

        case VDM_TASK_DISC_SVID:
            set_task(port, vdm_task_mng_disc_svid(port, get_evt(port)));
            break;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

        case VDM_TASK_SEND_MSG:
            compose_vdm(port);
            set_task(port, send_vdm(port));
            break;

#if CCG_USB4_SUPPORT_ENABLE
        case VDM_TASK_USB4_TBT:
            set_task(port, usb4_tbt_handler(port, get_evt(port)));
            break;
#endif /* CCG_USB4_SUPPORT_ENABLE */

        case VDM_TASK_REG_ATCH_TGT_INFO:
            set_task(port, reg_alt_mode_mngr(port, &vdm_stat[port].atch_tgt, get_msg(port)));
            set_evt(port, VDM_EVT_RUN);
            break;

        case VDM_TASK_ALT_MODE:
            set_task(port, vdm_task_mngr_alt_mode_process(port, get_evt(port)));
            if ((get_task(port) == VDM_TASK_SEND_MSG) || (get_task(port) == VDM_TASK_ALT_MODE))
            {
                set_evt(port, VDM_EVT_RUN);
            }
            break;

        case VDM_TASK_EXIT:
            vdm_task_mngr_deinit(port);
            app_get_status(port)->vdm_prcs_failed = true;
            break;

        default:
            return;

    }
}

static bool vdm_retry_check(uint8_t port, fail_status_t fail_stat)
{
    (vdm_stat[port].rec_retry_cnt)++;

    /* If we don't receive response more than rec_retry_cnt notify that fail */
    if (vdm_stat[port].rec_retry_cnt > MAX_RETRY_CNT)
    {
        set_vdm_failed(port, fail_stat);
        return false;
    }

    /* Try to send msg again */
    set_task(port, VDM_TASK_SEND_MSG);

    if (fail_stat == BUSY)
    {
        /* Start a BUSY timer to delay the retry attempt if port partner is busy. */
        timer_start(port, APP_VDM_BUSY_TIMER, APP_VDM_FAIL_RETRY_PERIOD, NULL);
    }
    else
    {
        /* Fix for Ellisys VDMD tests: Delay the retry by 10 ms. */
        timer_start(port, APP_VDM_BUSY_TIMER, 10u, NULL);
    }

    return true;
}

static void set_vdm_failed(uint8_t port, fail_status_t fail_stat)
{
    /* Reset retry counters */
    vdm_stat[port].rec_retry_cnt = 0;
    set_evt(port, VDM_EVT_FAIL);
    /* Set failed and save failed command */
    get_msg(port)->VDM_HDR.cmd = get_vdm_buff(port)->cmdDo->std_vdm_hdr.cmd;
    /* Save Failure code in the object position field */
    get_msg(port)->VDM_HDR.objPos = (uint32_t)fail_stat;
}

/* Received VDM callback */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t* rec_vdm)
{
    uint32_t response;
    bool     run_task_flag = false;

    /* If response ACK */
    if (status == RES_RCVD)
    {
        /* Check whether received message is a VDM and treat as NAK otherwise. */
        if ((rec_vdm->msg != DATA_MSG_VDM) || (rec_vdm->len == 0))
        {
            /* Notify manager with failed event. */
            set_vdm_failed(port, NACK);
            run_task_flag = true;
        }
        else
        {
            /* Handle Standard VDM */
            if (rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.vdmType == VDM_TYPE_STRUCTURED)
            {
                response = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmdType;

#if CY_PD_REV3_ENABLE
                /* Update VDM version based on the value received from UFP. */
                if (rec_vdm->sop == SOP)
                {
                    app_get_status(port)->vdm_version =
                        CY_USBPD_GET_MIN(app_get_status(port)->vdm_version, rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.stVer);
                }
#endif /* CY_PD_REV3_ENABLE */

                /* Actual response received. */
                switch (response)
                {
                    case CMD_TYPE_RESP_ACK:
                        {
#if CCG_USB4_SUPPORT_ENABLE
                            if (
                                   (app_get_status(port)->usb4_active == false)   &&
                                   (vdm_stat[port].usb4_flag > USB4_TBT_CBL_FIND) &&
                                  ((rec_vdm->sop == SOP_PRIME) || (rec_vdm->sop == SOP_DPRIME))
                                )
                            {
                                parse_vdm(port, rec_vdm);
                                set_task(port, VDM_TASK_USB4_TBT);
                                set_evt(port, VDM_EVT_EVAL);
                                return;
                            }
#endif /* CCG_USB4_SUPPORT_ENABLE */
                            /* Check if received response is expected. */
                            if ((rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd) ==
                                    (get_vdm_buff(port)->cmdDo[VDM_HEADER_IDX].std_vdm_hdr.cmd))
                            {
                                parse_vdm(port, rec_vdm);
                                set_evt(port, VDM_EVT_EVAL);

                                /* Reset timer counter when ACK. */
                                vdm_stat[port].rec_retry_cnt = 0;
                                /* Continue the state machine operation. */
                                run_task_flag = true;
                            }
                            else if (vdm_retry_check (port, BUSY) == false)
                                /* Check for retries. If failure persists after all retries, go to exit. */
                            {
                                run_task_flag = true;
                            }
                        }
                        break;

                    case CMD_TYPE_INITIATOR:
                    case CMD_TYPE_RESP_BUSY:
                        /* Target is BUSY. */
                        {
                            /* Check for retries. If failure persists after all retries, go to exit. */
                            if (vdm_retry_check (port, BUSY) == false)
                            {
                                run_task_flag = true;
                            }
                        }
                        break;

                    default:
                        /* Target NAK-ed the message. */
                        {
                            /* Notify manager with failed event */
                            set_vdm_failed(port, NACK);
                            run_task_flag = true;
                        }
                        break;
                }
            }
            /* Handle UVDM */
            else
            {
                parse_vdm(port, rec_vdm);
                set_evt(port, VDM_EVT_EVAL);
                /* Reset timer counter when ACK */
                vdm_stat[port].rec_retry_cnt = 0;
                run_task_flag = true;
            }
        }
    }
    /* Attention related handler */
    else if (get_msg(port)->VDM_HDR.cmd == VDM_CMD_ATTENTION)
    {
        /* This statement need to notify alt mode that Attention VDM was successfully sent */
        if ((status == CMD_SENT) && (get_task(port) == VDM_TASK_WAIT))
        {
            /* Start a timer. Command will be retried when timer expires. */
            timer_stop(port, APP_VDM_BUSY_TIMER);
            set_evt(port, VDM_EVT_EVAL);
            /* Reset retry counter */
            vdm_stat[port].rec_retry_cnt = 0;
            /* Continue the state machine operation. */
            run_task_flag = true;
        }
        else if (status == SEQ_ABORTED)
        {
            /* Try to send msg again */
            run_task_flag = true;
        }
    }
    else
    {
        /* Good CRC not received or no response (maybe corrupted packet). */
        if ((status == CMD_FAILED) || (status == RES_TIMEOUT))
        {
            /* Check for retries. If failure persists after all retries, go to exit. */
            if (vdm_retry_check (port, (fail_status_t)status) == false)
            {
                run_task_flag = true;
            }
        }
        else
        {
            if (status == SEQ_ABORTED)
            {
                /* Try to send msg again */
                set_task(port, VDM_TASK_SEND_MSG);
            }
        }
    }

    /* Check if we need run any task */
    if (run_task_flag == true)
    {
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if CCG_USB4_SUPPORT_ENABLE
        if (
                (app_get_status(port)->usb4_active == false)   &&
                (vdm_stat[port].usb4_flag > USB4_TBT_CBL_FIND) &&
                ((rec_vdm->sop == SOP_PRIME) || (rec_vdm->sop == SOP_DPRIME))
           )
        {
            /* Set USB4 Task */
            set_task(port, VDM_TASK_USB4_TBT);
            set_evt(port, VDM_EVT_FAIL);
            return;
        }
#endif /* CCG_USB4_SUPPORT_ENABLE */

        switch (vdm_stat[port].vdm_msg.VDM_HDR.cmd)
        {
            case VDM_CMD_DSC_IDENTITY:
                set_task(port, VDM_TASK_DISC_ID);
                break;
            case VDM_CMD_DSC_SVIDS:
                set_task(port, VDM_TASK_DISC_SVID);
                break;
            default:
                set_task(port, VDM_TASK_ALT_MODE);
                break;
        }
#else /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
        set_task(port, VDM_TASK_ALT_MODE);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    }
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

void set_disc_param(uint8_t port, uint8_t sop, std_vdm_cmd_t cmd)
{
    vdm_msg_info_t *msg_p = get_msg (port);

    /* Form Discover ID VDM packet. */
    msg_p->vdm_header.val   = NO_DATA;
    msg_p->VDM_HDR.svid     = STD_SVID;
    msg_p->VDM_HDR.cmd      = cmd;
    msg_p->vdo_numb         = NO_DATA;
    msg_p->sopType         = sop;
    msg_p->VDM_HDR.vdmType = VDM_TYPE_STRUCTURED;

}

#if RIDGE_SLAVE_ENABLE
static void update_usb_support_vars(uint8_t port, bool ufp_ack)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    app_status_t       *app      = app_get_status(port);
    vdm_msg_info_t     *msg_p    = get_msg (port);

    /* First set USB support flags based on cable marker properties. */
    if (dpm_stat->cblType == PROD_TYPE_ACT_CBL)
    {
        if (dpm_stat->cblVdo2.act_cbl_vdo2.usb2Supp)
        {
            app->usb2Supp = true;
        }
        if (dpm_stat->cblVdo2.act_cbl_vdo2.ssSupp)
        {
            app->usb3_supp = true;
        }
    }
    else if (dpm_stat->cblType == PROD_TYPE_PAS_CBL)
    {
        app->usb2Supp = true;

        if (dpm_stat->cblVdo.pas_cbl_vdo.usbSsSup > USB_GEN_2)
        {
            app->usb3_supp = true;
        }
    }

    /* If UFP response is available, confirm USB support in its D_ID response. */
    if (ufp_ack)
    {
        if ((msg_p->vdo[PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.devCap & DEV_CAP_USB_2_0) != 0)
        {
            app->usb2Supp = true;
        }

        if ((msg_p->vdo[PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.devCap & DEV_CAP_USB_3_2) != 0)
        {
            app->usb3_supp = true;
        }
    }
}
#endif /* RIDGE_SLAVE_ENABLE */

static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_msg_info_t *msg_p  = get_msg (port);
    vdm_task_t ret         = VDM_TASK_EXIT;
    const dpm_status_t *dpm_stat = dpm_get_info(port);

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_get_info(port)->specRevSopLive >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif /* ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover ID VDM packet */
            set_disc_param(port, SOP, VDM_CMD_DSC_IDENTITY);
            ret  = VDM_TASK_SEND_MSG;
            break;

            /* Evaluate received response */
        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
#if CCG_USB4_SUPPORT_ENABLE
                /* Check if USB4 mode already entered */
                if (app_get_status(port)->usb4_active != false)
                {
                    vdm_stat[port].usb4_flag = USB4_FAILED;
                    if ((msg_p->vdo[PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.altModes & UFP_NON_PH_ALT_MODE_SUPP_MASK) == false)
                    {
                        ret = VDM_TASK_WAIT;
                        break;
                    }
                }
                /* Check if CCG and port partner supports USB 4.0 */
                else if (
                        (dpm_stat->specRevSopLive < PD_REV3) ||
                        ((msg_p->vdo[PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.devCap & DEV_CAP_USB_4_0) == 0) ||
                        ((get_gl_vdm_id(port)[PRODUCT_TYPE_VDO_3_IDX].dfp_vdo.hostCap & HOST_CAP_USB_4_0) == 0) ||
                        (app_get_status(port)->usb4_data_rst_cnt > DATA_RST_RETRY_NUMB) ||
                        ((PD_GET_PTR_TBTHOST_CFG_TBL(port)->usb4_support != USB_ROLE_HOST) &&
                         (PD_GET_PTR_TBTHOST_CFG_TBL(port)->usb4_support != USB_ROLE_DRD))
                   )
                {
                    /*
                     * Cannot proceed with USB4 if we do not have USB4 host support, the UFP does not have USB4
                     * device support, we are in PD 2.0 contract or if there have been too many data resets.
                     */
                    vdm_stat[port].usb4_flag = USB4_FAILED;
                }

                /* Check if cable is USB 4.0 capable */
                if (
                        (vdm_stat[port].usb4_flag != USB4_FAILED)  &&
                        (dpm_stat->emcaPresent != false)
                   )
                {
                    if (msg_p->vdo[PRODUCT_TYPE_VDO_1_IDX - VDO_START_IDX].ufp_vdo_1.altModes == false)
                    {
                        /*
                         * Discovery process could not be run if Alternate Modes field in 
                         * UFP VDO 1 response is set to zero.
                         */
                        vdm_stat[port].alt_modes_not_supp = true;
                    }

                    /* Fill EUDO */
                    get_eudo(port)->enterusb_vdo.usbMode    = CCG_USB_MODE_USB4;
                    get_eudo(port)->enterusb_vdo.cableType  = false;
                    get_eudo(port)->enterusb_vdo.cableSpeed = dpm_stat->cblVdo.std_cbl_vdo.usbSsSup;
                  
                    /* Set TBT Host config params from config table */
                    get_eudo(port)->val |= (PD_GET_PTR_TBTHOST_CFG_TBL(port)->host_support << EUDO_HOST_PRES_OFFSET);

                    /* Set active cable type */
                    if (dpm_stat->cblType == PROD_TYPE_ACT_CBL)
                    {
                        get_eudo(port)->enterusb_vdo.cableType = CBL_TYPE_ACTIVE_REDRIVER;
                        /* If VDO version is 1.3 and higher */
                        if ((dpm_stat->cblVdo2.val != NO_DATA) && (dpm_stat->cblVdo.act_cbl_vdo1.vdoVersion >= CBL_VDO_VERS_1_3))
                        {
                            if (dpm_stat->cblVdo2.act_cbl_vdo2.optIsolated != false)
                            {
                                get_eudo(port)->enterusb_vdo.cableType = CBL_TYPE_OPTICAL;
                            }
                            else if (
                                        (dpm_stat->cblVdo2.act_cbl_vdo2.phyConn != false)  ||
                                        (dpm_stat->cblVdo2.act_cbl_vdo2.activeEl == false)
                                    )
                            {
                                get_eudo(port)->enterusb_vdo.cableType = CBL_TYPE_ACTIVE_REDRIVER;
                            }
                            else
                            {
                                get_eudo(port)->enterusb_vdo.cableType = CBL_TYPE_ACTIVE_RETIMER;
                            }
                        }
                    }

                    /* Set EUDO cable current: Only honor 3A and 5A capable cables. */
                    get_eudo(port)->enterusb_vdo.cableCurrent = 0;
                    if (
                            (
                                (dpm_stat->cblType == PROD_TYPE_PAS_CBL) ||
                                (dpm_stat->cblVdo.std_cbl_vdo.vbusThruCbl != 0)
                            ) &&
                            (
                             (dpm_stat->cblVdo.std_cbl_vdo.vbusCur == CBL_VBUS_CUR_3A) ||
                             (dpm_stat->cblVdo.std_cbl_vdo.vbusCur == CBL_VBUS_CUR_5A)
                            )
                       )
                    {
                        get_eudo(port)->enterusb_vdo.cableCurrent = dpm_stat->cblVdo.std_cbl_vdo.vbusCur + 1;
                    }

                    /* Set USB4 not supported as default */
                    vdm_stat[port].usb4_flag = USB4_FAILED;
                    
                    /* Active cable handler */
                    if (dpm_stat->cblType == PROD_TYPE_ACT_CBL)
                    {
                        if (
                                (dpm_stat->cblVdmVersion == STD_VDM_VER2) &&
                                (dpm_stat->cblVdo.act_cbl_vdo1.vdoVersion >= CBL_VDO_VERS_1_3)
                            )
                        {
                            /* Active cable supports USB4 */
                            if (
                                    (dpm_stat->cblVdo2.act_cbl_vdo2.usb4Supp == false)   &&
                                    (dpm_stat->cblVdo.act_cbl_vdo.usbSsSup >= USB_GEN_1)
                                )
                            {
                                /* Set TBT_Active_link_training in case of USB4 support and VDO version 1.3 */
                                vdm_stat[port].intel_reg.ridge_stat.act_link_train = true;
                                /* Set ridge register cable speed */
                                get_eudo(port)->enterusb_vdo.cableSpeed = (dpm_stat->cblVdo.pas_cbl_vdo.usbSsSup >= USB_GEN_3) ?
                                    USB_GEN_3 : dpm_stat->cblVdo.std_cbl_vdo.usbSsSup;
                                /* Enter USB4 mode */
                                enter_usb4(port, SOP_PRIME);
                            }
                        }
                        else if (dpm_stat->cblModeEn != false)
                        {
                            /* Go through TBT discovery */
                            vdm_stat[port].usb4_flag = USB4_TBT_CBL_FIND;
                        }                      
                    }
                    else
                    {
                        /* Since we know cable marker is present; a non-active cable has to be passive. */

                        /* Set cable speed in EUDO */
                        get_eudo(port)->enterusb_vdo.cableSpeed = (dpm_stat->cblVdo.std_cbl_vdo.usbSsSup > USB_GEN_3) ?
                                USB_GEN_3 : dpm_stat->cblVdo.std_cbl_vdo.usbSsSup;
                       
                        if ((dpm_stat->cblVdo.std_cbl_vdo.usbSsSup == USB_GEN_2) && (dpm_stat->cblModeEn != false))
                        {
                            /* Run TBT cable discovery */
                            vdm_stat[port].usb4_flag = USB4_TBT_CBL_FIND;
                        }
                        else if (dpm_stat->cblVdo.std_cbl_vdo.usbSsSup != USB_2_0)
                        {
                            if (dpm_stat->cblVdo.std_cbl_vdo.usbSsSup == USB_GEN_3)
                            {
                                /* Set TBT_Active_link_training in case of USB4 support and VDO version 1.3 */
                                vdm_stat[port].intel_reg.ridge_stat.act_link_train = true;
                            }
                            /* Enter USB4 mode */
                            enter_usb4(port, SOP);      
                        }
                    }
                }
#endif /* CCG_USB4_SUPPORT_ENABLE */
#if VDM_RESP_QUERY_SUPPORTED
                /* Store the D_ID response received. */
                vdm_disc_id_resp[port][0] = msg_p->vdm_header.val;
                memcpy ((uint8_t *)&vdm_disc_id_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                vdm_disc_id_resp_len[port] = msg_p->vdo_numb + 1;
#endif /* VDM_RESP_QUERY_SUPPORTED */

#if RIDGE_SLAVE_ENABLE
                /* Analyze Disc ID SOP & SOP' responses for USB2/USB3 compatibility */
                update_usb_support_vars (port, true);
#endif /* RIDGE_SLAVE_ENABLE */

                /* Set svid idx to zero before start DISC SVID */
                vdm_stat[port].svid_idx  = NO_DATA;
                vdm_stat[port].dsvid_cnt = 0;

                /* Copy ID header to info struct */
                vdm_stat[port].atch_tgt.ama_vdo.val = msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].val;
                /* Copy AMA VDO to info struct */
                vdm_stat[port].atch_tgt.tgt_id_header.val = msg_p->vdo[VDO_START_IDX - 1].val;

                /* If AMA and cable (if present) do not need Vconn, disable VConn. */
                if (
                        (dpm_stat->vconnRetain == 0) &&
                        ((msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.prodType != PROD_TYPE_AMA) ||
                         (msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].std_ama_vdo.vconReq == 0)) &&
                        ((dpm_stat->emcaPresent == false) ||
                         (dpm_stat->cblVdo.std_cbl_vdo.cblTerm == CBL_TERM_BOTH_PAS_VCONN_NOT_REQ))
                   )
                {
                    vconn_disable(port, dpm_stat->revPol);
                }

                /* If alt modes not supported. */
                if (msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.modSupport == false)
                {
#if CCG_UCSI_ENABLE
                    modal_op_support = false;
#endif /*CCG_UCSI_ENABLE*/
#if CCG_USB4_SUPPORT_ENABLE
                    vdm_stat[port].alt_modes_not_supp = true;
                    if (vdm_stat[port].usb4_flag == USB4_FAILED)
                        break;
#else
                    break;
#endif /* CCG_USB4_SUPPORT_ENABLE */

                }
#if CCG_UCSI_ENABLE
                modal_op_support = true;
#endif /*CCG_UCSI_ENABLE*/

                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
#if RIDGE_SLAVE_ENABLE
            /* Update USB support variables based on cable capabilities alone. */
            update_usb_support_vars (port, false);
#endif /* RIDGE_SLAVE_ENABLE */

        default:
            break;
    }

    return ret;
}

/* Checks if input SVID already saved. */
static bool is_svid_stored(uint16_t *svid_arr, uint16_t svid)
{
    uint8_t  idx;

    /* Go through all SVIDs and compare with input SVID */
    for (idx = 0; idx < MAX_SVID_VDO_SUPP; idx++)
    {
        /* If input SVID already saved */
        if (svid_arr[idx] == svid)
        {
            return true;
        }
    }

    return false;
}

/*
   This function saves received Discover SVID resp,
   returns true if a NULL SVID was received.
 */
static bool save_svids(uint8_t port, uint16_t *svid_arr, uint8_t max_svid)
{
    uint8_t         idx, vdo_count;
    uint16_t        svid;
    uint8_t         svid_idx = vdm_stat[port].svid_idx;
    bool            retval   = false;
    vdm_msg_info_t  *msg_p   = get_msg (port);

    /* Compare received SVIDs with supported SVIDs */
    vdo_count = msg_p->vdo_numb;

    /* Stop further discovery if this response does not have the maximum no. of DOs. */
    if (vdo_count < (MAX_NO_OF_DO - 1))
    {
        retval = true;
    }

    for (idx = 0; idx < (vdo_count * 2); idx++)
    {
        if ((idx & 1) == 0)
        {
            /* Upper half of the DO. */
            svid = msg_p->vdo[idx >> 1].val >> 16u;
        }
        else
        {
            /* Lower half of the DO. */
            svid = msg_p->vdo[idx >> 1].val & 0xFFFFu;
        }

        if (svid_idx < (max_svid - 1))
        {
            /* Stop on NULL SVID. */
            if (svid == NO_DATA)
            {
                retval = true;
            }
            else
            {
                /* If SVID not saved previously then save */
                if (is_svid_stored(svid_arr, svid) == false)
                {
#if SAVE_SUPP_SVID_ONLY
                    if (get_alt_modes_config_svid_idx(port, gl_dpm_port_type[port], svid) != MODE_NOT_SUPPORTED)
#endif /* SAVE_SUPP_SVID_ONLY */
                    {
                        svid_arr[svid_idx] = svid;
                        svid_idx++;
                    }
                }
#if CCG_USB4_SUPPORT_ENABLE
                /* Check if TBT svid is supported by cable if USB4 related
                 * TBT cable Disc Mode procedure should be provided
                 */
                if (
                       (get_msg (port)->sopType == SOP_PRIME)         &&
                       (vdm_stat[port].usb4_flag == USB4_TBT_CBL_FIND) &&
                       (svid == TBT_SVID)
                    )
                {
                    /* Start TBT cable Discover mode process */
                    vdm_stat[port].usb4_flag = USB4_TBT_CBL_DISC_RUN;
                }
#endif /* CCG_USB4_SUPPORT_ENABLE */

            }
        }
        else
        {
            /* Cannot continue as we have no more space. */
            retval = true;
            break;
        }
    }

    /* Set zero after last SVID in info */
    svid_arr[svid_idx] = NO_DATA;
    vdm_stat[port].svid_idx = svid_idx;

    /* Terminate discovery after MAX_DISC_SVID_COUNT attempts. */
    vdm_stat[port].dsvid_cnt++;
    if (vdm_stat[port].dsvid_cnt >= MAX_DISC_SVID_COUNT)
        retval = true;

    return retval;
}

static bool vdm_is_cbl_svid_req(uint8_t port)
{
    bool ret = false;
    const dpm_status_t *dpm_stat   = dpm_get_info(port);
    
    /* If cable was detected and supports alternate modes, send SOP' disc svid */
    if ((dpm_stat->emcaPresent != false) &&
            (dpm_stat->cblModeEn != false))
    {
        if (
                (dpm_stat->vconnLogical)
        #if VCONN_OCP_ENABLE
                && ((app_get_status(port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) == 0)
        #endif /* VCONN_OCP_ENABLE */
           )
        {
            if (!vconn_is_present (port))
            {
                /*
                 * We are VConn source and VConn is off. Enable and apply a delay to let
                 * the EMCA power up.
                 */
                if (vconn_enable (port, dpm_stat->revPol))
                {
                    /* Set a flag to indicate that we need to do Cable SVID checks. */
                    ret = true;

                    /* Start a timer to delay the retry attempt. */
                    timer_start(port, APP_VDM_BUSY_TIMER, APP_CABLE_POWER_UP_DELAY, NULL);
                }
            }
            else
            {
                /* Set a flag to indicate that we need to do Cable SVID checks. */
                ret = true;
            }
        }
    }    
    
    return ret;
}

static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_task_t          ret        = VDM_TASK_EXIT;
    const dpm_status_t *dpm_stat   = dpm_get_info(port);
    vdm_msg_info_t     *msg_p      = get_msg (port);
#if VDM_RESP_QUERY_SUPPORTED
    uint32_t tmp;
#endif /* VDM_RESP_QUERY_SUPPORTED */

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_stat->specRevSopLive >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
#if CCG_USB4_SUPPORT_ENABLE 
            if (vdm_stat[port].usb4_flag == USB4_TBT_CBL_FIND)
            {
                /* Enable Vconn if required */
                vdm_is_cbl_svid_req(port);
                /* Form Discover SVID VDM packet to find out TBT cable */
                set_disc_param(port, SOP_PRIME, VDM_CMD_DSC_SVIDS);
                ret  = VDM_TASK_SEND_MSG;            
            }
            else
#endif /* CCG_USB4_SUPPORT_ENABLE */
            {
                /* Form Discover SVID VDM packet */
                set_disc_param(port, SOP, VDM_CMD_DSC_SVIDS);
                ret  = VDM_TASK_SEND_MSG;
            }
            break;

        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
                /* For attached target response */
                if (msg_p->sopType == (uint8_t)SOP)
                {
#if VDM_RESP_QUERY_SUPPORTED
                    if (vdm_stat[port].svid_idx == 0)
                    {
                        /* Save the DISC_SVID response. */
                        vdm_disc_svid_resp[port][0] = msg_p->vdm_header.val;
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] = msg_p->vdo_numb + 1;
                    }
                    else
                    {
                        /* Save the incremental DISC_SVID response. */
                        tmp = CY_USBPD_GET_MIN (msg_p->vdo_numb, MAX_DISC_SVID_RESP_LEN - vdm_disc_svid_resp_len[port]);
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][vdm_disc_svid_resp_len[port]], (uint8_t *)msg_p->vdo,
                                tmp * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] += tmp;
                    }
#endif /* VDM_RESP_QUERY_SUPPORTED */

                    if (msg_p->vdo[0].val == NONE_VDO)
                    {
                        /* No SVID supported */
                        break;
                    }

                    /* Save received SVIDs and check if a NULL SVID was received. */
                    if (save_svids (port, vdm_stat[port].atch_tgt.tgt_svid, MAX_SVID_VDO_SUPP) != false)
                    {
                        /* Check if cable Disc SVID should be done */
                        if (vdm_is_cbl_svid_req(port) != false)
                        {
                            vdm_stat[port].svid_idx  = 0;
                            vdm_stat[port].dsvid_cnt = 0;
                            set_disc_param(port, SOP_PRIME, VDM_CMD_DSC_SVIDS);
                            ret = VDM_TASK_SEND_MSG;
                        }
                        else
                        {
                            /*
                             * We are either not VConn source or failed to turn VConn ON.
                             * Skip SOP' checks in the unlikely case where this happens.
                             */
                            set_evt(port, VDM_EVT_RUN);
                            ret = VDM_TASK_REG_ATCH_TGT_INFO;
                        }
                    }
                    else
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        set_disc_param(port, SOP, VDM_CMD_DSC_SVIDS);
                        ret = VDM_TASK_SEND_MSG;
                    }
                }
                /* For cable response */
                else
                {
                    /* If the EMCA returned any DOs, save the content. */
                    if ((msg_p->vdo[VDO_START_IDX - 1].val != NONE_VDO) &&
                            (save_svids(port, vdm_stat[port].atch_tgt.cbl_svid, MAX_CABLE_SVID_SUPP) == false))
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        set_disc_param(port, SOP_PRIME, VDM_CMD_DSC_SVIDS);
                        ret = VDM_TASK_SEND_MSG;
                    }
                    else
                    {
                        /*
                           If EMCA did not support any SVIDs of interest and does not require VConn for operation,
                           we can disable VConn.
                         */
                        if (vdm_stat[port].atch_tgt.cbl_svid[0] == NO_DATA)
                        {
                            if (
                                    (dpm_stat->vconnRetain == 0) &&
                                    (dpm_stat->cblVdo.std_cbl_vdo.cblTerm == CBL_TERM_BOTH_PAS_VCONN_NOT_REQ)
                               )
                            {
                                vconn_disable(port, dpm_stat->revPol);
                            }
                        }

                        /* Move to the next step. */
                        set_evt(port, VDM_EVT_RUN);
                        ret = VDM_TASK_REG_ATCH_TGT_INFO;
#if CCG_USB4_SUPPORT_ENABLE
                        if (vdm_stat[port].usb4_flag == USB4_TBT_CBL_DISC_RUN)
                        {
                            /* Run Enter USB4 related Disc mode process */
                            ret = VDM_TASK_USB4_TBT;
                        }
                        else if (
                                    (vdm_stat[port].usb4_flag == USB4_TBT_CBL_FIND) &&
                                    (dpm_stat->cblType == PROD_TYPE_PAS_CBL)
                                )
                        {
                            /* Enter USB4 mode */
                            enter_usb4(port, SOP);
                        }
#endif /* CCG_USB4_SUPPORT_ENABLE */
                    }
                }
            }
#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
#if CCG_USB4_SUPPORT_ENABLE
            if (vdm_stat[port].usb4_flag != USB4_FAILED)
            {
                if (dpm_stat->cblType == PROD_TYPE_PAS_CBL)
                {
                    /* Enter USB4 mode */
                    enter_usb4(port, SOP);
                }
                else
                {
                    /* Resume VDM handling */
                    ret = resume_vdm_handler(port);
                }
                break;
            }
#endif /* (CCG_USB4_SUPPORT_ENABLE) */
            /* If cable SVID fails */
            if (msg_p->sopType == (uint8_t)SOP_PRIME)
            {
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
            break;

        default:
            break;
    }

    return ret;
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static void reset_vdm_mngr(uint8_t port)
{
    vdm_stat[port].rec_retry_cnt      = NO_DATA;
    vdm_stat[port].svid_idx           = NO_DATA;
    vdm_stat[port].dsvid_cnt          = NO_DATA;

#if CCG_USB4_SUPPORT_ENABLE
    vdm_stat[port].alt_modes_not_supp = NO_DATA;
    vdm_stat[port].usb4_flag          = USB4_NONE;
    vdm_stat[port].intel_reg.val      = NO_DATA;
#endif /* CCG_USB4_SUPPORT_ENABLE */

    /* Clear arrays which hold SVIDs */
    memset(&vdm_stat[port].atch_tgt, NO_DATA, sizeof(atch_tgt_info_t));
    /* Store the pointer to the cable VDO discovered by PD stack. */
    vdm_stat[port].atch_tgt.cblVdo = &(dpm_get_info(port)->cblVdo);
    /* Clear the SOP'' reset state. */
    vdm_emca_rst_state[port] = CABLE_DP_RESET_IDLE;
    vdm_emca_rst_count[port] = 0;

    /* Stop the VDM task delay timer. */
    timer_stop(port, APP_VDM_BUSY_TIMER);

#if VDM_RESP_QUERY_SUPPORTED
    vdm_disc_id_resp_len[port] = 0;
    memset ((uint8_t *)&vdm_disc_id_resp[port], 0, MAX_NO_OF_DO * sizeof (uint32_t));
    vdm_disc_svid_resp_len[port] = 0;
    memset ((uint8_t *)&vdm_disc_svid_resp[port], 0, MAX_DISC_SVID_RESP_LEN * sizeof (uint32_t));
#endif /* VDM_RESP_QUERY_SUPPORTED */
}

void vdm_task_mngr_deinit(uint8_t port)
{
    /* CDT 247011 re-fix */
    app_get_status(port)->vdm_prcs_failed = false;

    /* If VDM task is not active, no need to go through the rest of the steps. */
    if (app_get_status(port)->vdm_task_en != false)
    {
        reset_vdm_mngr(port);

        /* Clear all application status flags. */
        app_get_status(port)->vdm_task_en      = false;
        app_get_status(port)->alt_mode_entered = false;
        app_get_status(port)->cbl_rst_done     = false;
        app_get_status(port)->trig_cbl_rst     = false;

#if (DEFER_VCONN_SRC_ROLE_SWAP)
        app_get_status(port)->keep_vconn_src   = false;
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */

        /* Exit from alt mode manager */
        vdm_task_mngr_alt_mode_process(port, VDM_EVT_EXIT);

        /* Deinit App HW */
        alt_mode_hw_deinit(port);
        
        app_get_status(port)->cable_retimer_supp = false;
    }
}

void vdm_task_mngr_exit_modes(uint8_t port)
{
    if (
            (dpm_get_info(port)->curPortType == PRT_TYPE_DFP) &&
            (app_get_status(port)->alt_mode_entered != false)
       )
    {
        /* Exit from alt mode manager. */
        vdm_task_mngr_alt_mode_process(port, VDM_EVT_EXIT);
    }
}

static void sop_dp_soft_reset_cb(uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    switch (resp)
    {
        case CMD_SENT:
            /* Do nothing. */
            break;

        case RES_RCVD:
            /* Proceed with rest of alternate mode state machine. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            break;

        default:
            /* Retry the cable SOFT_RESET. */
            vdm_emca_rst_count[port]++;
            if (vdm_emca_rst_count[port] >= MAX_EMCA_DP_RESET_COUNT)
            {
                vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            }
            else
            {
                vdm_emca_rst_state[port] = CABLE_DP_RESET_RETRY;
            }
            break;
    }
}

static void send_sop_dp_soft_reset(uint8_t port)
{
    dpm_pd_cmd_buf_t dpm_cmd_param;
    ccg_status_t     api_stat;

    dpm_cmd_param.cmdSop      = SOP_DPRIME;
    dpm_cmd_param.noOfCmdDo = 0;
    dpm_cmd_param.datPtr      = 0;
    dpm_cmd_param.timeout      = 15;

    api_stat = dpm_pd_command (port, DPM_CMD_SEND_SOFT_RESET_EMCA, &dpm_cmd_param, sop_dp_soft_reset_cb);
    switch (api_stat)
    {
        case CCG_STAT_SUCCESS:
            /* Wait for the cable SOFT_RESET response. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_WAIT;
            break;

        case CCG_STAT_BUSY:
        case CCG_STAT_NOT_READY:
            /* Need to retry the command. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_RETRY;
            break;

        default:
            /* Connection state changed. Abort the Cable SOFT_RESET process. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            break;
    }
}

/* Fill DPM cmd buffer with properly VDM info */
static uint8_t compose_vdm(uint8_t port)
{
    uint8_t           idx;
    dpm_pd_cmd_buf_t  *vdm_buf = get_vdm_buff(port);
    vdm_msg_info_t    *msg_p   = get_msg(port);

    vdm_buf->cmdSop                = msg_p->sopType;
    vdm_buf->noOfCmdDo           = msg_p->vdo_numb + VDO_START_IDX;
    vdm_buf->cmdDo[VDM_HEADER_IDX] = msg_p->vdm_header;

    if (msg_p->VDM_HDR.vdmType == VDM_TYPE_STRUCTURED)
    {
        msg_p->VDM_HDR.cmdType    = CMD_TYPE_INITIATOR;

        if (vdm_buf->cmdSop == SOP)
        {
            vdm_buf->cmdDo[VDM_HEADER_IDX].std_vdm_hdr.stVer = app_get_status(port)->vdm_version;
        }
        else
        {
            vdm_buf->cmdDo[VDM_HEADER_IDX].std_vdm_hdr.stVer = dpm_get_info(port)->cblVdmVersion;
        }

        /* Set exceptions for Enter/Exit mode cmd period */
        switch (msg_p->VDM_HDR.cmd)
        {
            case VDM_CMD_ENTER_MODE:
                vdm_buf->timeout = PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case VDM_CMD_EXIT_MODE:
                vdm_buf->timeout = PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case VDM_CMD_ATTENTION:
                /* No timeout required for attention messages. */
                vdm_buf->timeout = NO_DATA;
                break;
            default:
                vdm_buf->timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
                break;
        }
    }
    /* Handle UVDM */
    else
    {
        vdm_buf->timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
    }

    /* Copy VDOs to send buffer */
    if (msg_p->vdo_numb > NO_DATA)
    {
        for (idx = 0; idx < msg_p->vdo_numb; idx++)
        {
            vdm_buf->cmdDo[idx + VDO_START_IDX].val = msg_p->vdo[idx].val;
        }
    }

    return true;
}

/* Parse received VDM and save info in  */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm)
{
    uint8_t vdo_idx;
    vdm_msg_info_t *msg_p  = get_msg (port);

    msg_p->vdm_header = rec_vdm->dat[VDM_HEADER_IDX];
    msg_p->vdo_numb   = (rec_vdm->len - VDO_START_IDX);
    msg_p->sopType   = rec_vdm->sop;

    /* If VDO is present in received VDM */
    if (rec_vdm->len > VDO_START_IDX)
    {
        for (vdo_idx = 0; vdo_idx < rec_vdm->len; vdo_idx++)
        {
            msg_p->vdo[vdo_idx].val = rec_vdm->dat[vdo_idx + VDO_START_IDX].val;
        }
    }

    return true;
}

static vdm_task_t send_vdm(uint8_t port)
{
    if ((gl_dpm_port_type[port] == PRT_TYPE_DFP) && (get_vdm_buff(port)->cmdSop == SOP_DPRIME))
    {
        switch (vdm_emca_rst_state[port])
        {
            /* If SOP'' SOFT RESET has not been done or a retry is pending, try to send it. */
            case CABLE_DP_RESET_IDLE:
            case CABLE_DP_RESET_RETRY:
                send_sop_dp_soft_reset(port);
                /* Intentional fall-through. */
            case CABLE_DP_RESET_WAIT:
                return VDM_TASK_SEND_MSG;
            case CABLE_DP_RESET_DONE:
                if (app_get_status(port)->cbl_rst_done != false)
                {
                    /* Send SOP Prime VDM again after cable reset */
                    get_msg(port)->sopType = SOP_PRIME;
                    vdm_emca_rst_state[port] = CABLE_DP_RESET_SEND_SPRIME;
                    return VDM_TASK_SEND_MSG;
                }
                else
                {
                    break;
                }
            default:
                /* EMCA SOFT_RESET done. We can proceed. */
                break;
        }
    }

    app_get_status(port)->vdm_retry_pending = (vdm_stat[port].rec_retry_cnt != MAX_RETRY_CNT);
    if (dpm_pd_command (port, DPM_CMD_SEND_VDM, get_vdm_buff(port), vdm_rec_cbk) == CCG_STAT_SUCCESS)
    {
        return VDM_TASK_WAIT;
    }

    /* If fails - try to send VDM again */
    return VDM_TASK_SEND_MSG;
}

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
bool is_ufp_disc_started(uint8_t port)
{
    if (is_vdm_task_idle(port) == true)
    {
        /* Set VDM Task to send DISC ID VDM */
        set_task(port, VDM_TASK_DISC_ID);
        set_evt(port, VDM_EVT_RUN);
        return true;
    }

    return false;
}
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

bool is_vdm_mngr_enabled(uint8_t port)
{
    return (bool)app_get_status(port)->vdm_task_en;
}

vdm_evt_t get_evt(uint8_t port)
{
    return vdm_stat[port].vdm_evt;
}

vdm_task_t get_task(uint8_t port)
{
    return vdm_stat[port].vdm_task;
}

void set_task(uint8_t port, vdm_task_t task)
{
    vdm_stat[port].vdm_task = task;
}

void set_evt(uint8_t port, vdm_evt_t evt)
{
    vdm_stat[port].vdm_evt = evt;
}

static vdm_msg_info_t* get_msg(uint8_t port)
{
    return &vdm_stat[port].vdm_msg;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

uint8_t *vdm_get_disc_id_resp(cy_stc_pdstack_context_t * context, uint8_t *resp_len_p)
{
    (void) context;
    uint8_t *ptr = NULL;

    /* Check for bad pointer argument. */
    if (resp_len_p == NULL)
        return NULL;

    /* Set response length to zero by default. */
    *resp_len_p = 0;

#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    if ((port < NO_OF_TYPEC_PORTS) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
    {
        *resp_len_p = vdm_disc_id_resp_len[port];
        ptr         = (uint8_t *)vdm_disc_id_resp[port];
    }
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */

    return (ptr);
}

uint8_t *vdm_get_disc_svid_resp(cy_stc_pdstack_context_t * context, uint8_t *resp_len_p)
{
    (void) context;
    uint8_t *ptr = NULL;

    /* Check for bad pointer argument. */
    if (resp_len_p == NULL)
        return NULL;

    /* Set response length to zero by default. */
    *resp_len_p = 0;

#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    if ((port < NO_OF_TYPEC_PORTS) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
    {
        *resp_len_p = vdm_disc_svid_resp_len[port];
        ptr         = (uint8_t *)vdm_disc_svid_resp[port];
    }
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */

    return (ptr);
}

/* [] END OF FILE */
