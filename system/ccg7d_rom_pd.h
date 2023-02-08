/******************************************************************************
* File Name: pd.h
* \version 2.0
*
* Description: Header file for CCG7D ROM Compatibility
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef PD_H_
#define PD_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/*******************************************************************************
 * Enumerated Data Definitions
 ******************************************************************************/
/** \addtogroup group_ccgxAppCommon_enums
* \{
*/

/**
 * @typedef port_role_t
 * @brief Enum of the PD port roles.
 */
typedef enum
{
    PRT_ROLE_SINK = 0,                  /**< Power sink */
    PRT_ROLE_SOURCE,                    /**< Power source */
    PRT_DUAL                            /**< Dual Role Power device: can be source or sink. */
} port_role_t;

/**
 * @typedef pd_rev_t
 * @brief Enumeration of the PD spec revisions.
 */
typedef enum
{
    PD_REV1 = 0,                        /**< USB-PD spec revision 1.0. Not supported. */
    PD_REV2,                            /**< USB-PD spec revision 2.0. */
    PD_REV3,                            /**< USB-PD spec revision 3.0. */
    PD_REV_RSVD                         /**< Undefined USB-PD spec revision. */
} pd_rev_t;

/**
 * @typedef extd_msg_t
 * @brief Enum of the extended data message types.
 */
typedef enum
{
    EXTD_MSG_SRC_CAP_EXTD = 1,          /**< 0x01: Source_Capabilities_Extended message. */
    EXTD_MSG_STATUS,                    /**< 0x02: Status message. */
    EXTD_MSG_GET_BAT_CAP,               /**< 0x03: Get_Battery_Cap message. */
    EXTD_MSG_GET_BAT_STATUS,            /**< 0x04: Get_Battery_Status message. */
    EXTD_MSG_BAT_CAP,                   /**< 0x05: Battery_Capabilities message. */
    EXTD_MSG_GET_MANU_INFO,             /**< 0x06: Get_Manufacturer_Info message. */
    EXTD_MSG_MANU_INFO,                 /**< 0x07: Manufacturer_Info message. */
    EXTD_MSG_SECURITY_REQ,              /**< 0x08: Security_Request message. */
    EXTD_MSG_SECURITY_RESP,             /**< 0x09: Security_Response message. */
    EXTD_MSG_FW_UPDATE_REQ,             /**< 0x0A: Firmware_Update_Request message. */
    EXTD_MSG_FW_UPDATE_RESP,            /**< 0x0B: Firmware_Update_Response message. */
    EXTD_MSG_PPS_STATUS,                /**< 0x0C: PPS_Status message. */
    EXTD_MSG_COUNTRY_INFO,              /**< 0x0D: Country_Info message. */
    EXTD_MSG_COUNTRY_CODES,             /**< 0x0E: Country_Codes message. */
    EXTD_MSG_SNK_CAP_EXTD               /**< 0x0F: Sink_Capabilities_Extended message. */
} extd_msg_t;

/**
 * @typedef sop_t
 * @brief Enum of the SOP (Start Of Frame) types.
 */
typedef enum
{
    SOP = 0,                            /**< SOP: Used for communication with port partner. */
    SOP_PRIME,                          /**< SOP': Cable marker communication. */
    SOP_DPRIME,                         /**< SOP'': Cable marker communication. */
    SOP_P_DEBUG,                        /**< SOP'_Debug */
    SOP_DP_DEBUG,                       /**< SOP''_Debug */
    HARD_RESET,                         /**< Hard Reset */
    CABLE_RESET,                        /**< Cable Reset */
    SOP_INVALID                         /**< Undefined ordered set. */
} sop_t;

/**
 * @typedef port_type_t
 * @brief Enum of the PD port types.
 */
typedef enum
{
    PRT_TYPE_UFP = 0,                   /**< Upstream facing port. USB device or Alternate mode accessory. */
    PRT_TYPE_DFP,                       /**< Downstream facing port. USB host or Alternate mode controller. */
    PRT_TYPE_DRP                        /**< Dual Role data device: can be UFP or DFP. */
} port_type_t;

/**
 * @typedef app_req_status_t
 * @brief Enum of the PD Request results. Enum fields map to the control
 * message field in the PD spec.
 */
typedef enum
{
    REQ_SEND_HARD_RESET = 1,            /**< Invalid message. Send Hard Reset. */
    REQ_ACCEPT = 3,                     /**< Send Accept message. */
    REQ_REJECT = 4,                     /**< Send Reject message. */
    REQ_WAIT = 12,                      /**< Send Wait message. */
    REQ_NOT_SUPPORTED = 16              /**< Send Not_Supported message. Will translate to Reject message under PD 2.0 */
} app_req_status_t;

/**
 * @typedef resp_status_t
 * @brief Enum of the response status to DPM commands.
 */
typedef enum
{
    SEQ_ABORTED = 0,                    /**< PD AMS aborted. */
    CMD_FAILED,                         /**< PD AMS failed. */
    RES_TIMEOUT,                        /**< No response received. */
    CMD_SENT,                           /**< PD command has been sent. Response wait may be in progress. */
    RES_RCVD                            /**< Response received. */
} resp_status_t;

/**
 * @typedef dpm_pd_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types.
 */
typedef enum
{
    DPM_CMD_SRC_CAP_CHNG = 0,           /**< 00: Source Caps changed notification. Used to trigger fresh contract. */
    DPM_CMD_SNK_CAP_CHNG,               /**< 01: Sink Caps changed notification. Used to trigger fresh contract. */
    DPM_CMD_SEND_GO_TO_MIN,             /**< 02: Send GotoMin message to port partner. */
    DPM_CMD_GET_SNK_CAP,                /**< 03: Send Get_Sink_Cap message to port partner. */
    DPM_CMD_GET_SRC_CAP,                /**< 04: Send Get_Source_Cap message to port partner. */
    DPM_CMD_SEND_HARD_RESET,            /**< 05: Send Hard Reset. */
    DPM_CMD_SEND_SOFT_RESET,            /**< 06: Send Soft Reset to port partner. */
    DPM_CMD_SEND_CABLE_RESET,           /**< 07: Send Cable Reset. */
    DPM_CMD_SEND_SOFT_RESET_EMCA,       /**< 08: Send Soft Reset to cable marker. */
    DPM_CMD_SEND_DR_SWAP,               /**< 09: Send DR_Swap request. */
    DPM_CMD_SEND_PR_SWAP,               /**< 0A: Send PR_Swap request. */
    DPM_CMD_SEND_VCONN_SWAP,            /**< 0B: Send VCONN_Swap request. */
    DPM_CMD_SEND_VDM,                   /**< 0C: Send VDM message. */
    DPM_CMD_SEND_EXTENDED,              /**< 0D: Send extended data message. */
    DPM_CMD_GET_SRC_CAP_EXTENDED,       /**< 0E: Send Get_Source_Cap_Extended message. */
    DPM_CMD_GET_STATUS,                 /**< 0F: Send Get_Status message. */
    DPM_CMD_SEND_BATT_STATUS,           /**< 10: Send Battery_Status data message. */
    DPM_CMD_SEND_ALERT,                 /**< 11: Send Alert message. */
    DPM_CMD_SEND_NOT_SUPPORTED,         /**< 12: Send Not_Supported message. */
    DPM_CMD_INITIATE_CBL_DISCOVERY,     /**< 13: Initiate cable discovery (preceded by VConn Swap if required). */
    DPM_CMD_SEND_DATA_RESET,            /**< 14: Send a USB4 Data_Reset message. */
    DPM_CMD_SEND_ENTER_USB,             /**< 15: Send a USB4 Enter_USB message to port partner or cable marker. */
    DPM_CMD_GET_SNK_CAP_EXTENDED,       /**< 16: Send Get_Sink_Cap_Extended message. */
    DPM_CMD_SEND_REQUEST,               /**< 17: Send Request data message. */
    DPM_CMD_GET_PPS_STATUS,             /**< 18: Send Get_PPS_Status message. */
    DPM_CMD_GET_COUNTRY_CODES,          /**< 19: Send Get_Country_Codes message. */
    DPM_CMD_SEND_INVALID = 0xFFu        /**< FF: Invalid command code. */
} dpm_pd_cmd_t;

/**
 * @typedef pd_devtype_t
 * @brief Enum of the attached device type.
 */
typedef enum
{
    DEV_SNK = 1,                        /**< Power sink device is attached. */
    DEV_SRC,                            /**< Power source device is attached. */
    DEV_DBG_ACC,                        /**< Debug accessory is attached. */
    DEV_AUD_ACC,                        /**< Audio accessory is attached. */
    DEV_PWRD_ACC,                       /**< Powered accessory is attached. */
    DEV_VPD,                            /**< Vconn powered device is attached. */
    DEV_UNSUPORTED_ACC                  /**< Unsupported device type is attached. */
} pd_devtype_t;

/**
 * @typedef vdm_type_t
 * @brief Enum of the VDM types.
 */
typedef enum
{
    VDM_TYPE_UNSTRUCTURED = 0,          /**< Unstructured VDM. */
    VDM_TYPE_STRUCTURED                 /**< Structured VDM. */
} vdm_type_t;

/**
 * @typedef std_vdm_cmd_t
 * @brief Enum of the standard VDM commands.
 */
typedef enum
{
    VDM_CMD_DSC_IDENTITY = 1,           /**< Discover Identity command. */
    VDM_CMD_DSC_SVIDS,                  /**< Discover SVIDs command. */
    VDM_CMD_DSC_MODES,                  /**< Discover Modes command. */
    VDM_CMD_ENTER_MODE,                 /**< Enter Mode command. */
    VDM_CMD_EXIT_MODE,                  /**< Exit Mode command. */
    VDM_CMD_ATTENTION,                  /**< Attention message. */
    VDM_CMD_DP_STATUS_UPDT = 16,        /**< DisplayPort Status Update message. */
    VDM_CMD_DP_CONFIGURE = 17           /**< DisplayPort Configure command. */
} std_vdm_cmd_t;

/**
 * @typedef std_vdm_cmd_type_t
 * @brief Enum of the standard VDM command types.
 */
typedef enum
{
    CMD_TYPE_INITIATOR = 0,             /**< VDM sent by command initiator. */
    CMD_TYPE_RESP_ACK,                  /**< ACK response. */
    CMD_TYPE_RESP_NAK,                  /**< NAK response. */
    CMD_TYPE_RESP_BUSY                  /**< BUSY response. */
} std_vdm_cmd_type_t;

/**
 * @typedef std_vdm_prod_t
 * @brief Enum of the standard VDM product types.
 */
typedef enum
{
    PROD_TYPE_UNDEF    = 0,             /**< Undefined device type. */
    PROD_TYPE_HUB      = 1,             /**< Hub device type. */
    PROD_TYPE_PERI     = 2,             /**< Peripheral device type. */
    PROD_TYPE_PSD      = 3,             /**< Power Sink Device. */
    PROD_TYPE_PAS_CBL  = 3,             /**< Passive Cable. */
    PROD_TYPE_ACT_CBL  = 4,             /**< Active Cable. */
    PROD_TYPE_AMA      = 5,             /**< Alternate Mode Accessory. */
    PROD_TYPE_VPD      = 6,             /**< Vconn powered device. */
    PROD_TYPE_RSVD     = 7              /**< Reserved. Shall not be used. */
} std_vdm_prod_t;

/**
 * @typedef std_vdm_conn_t
 * @brief Enum of the standard VDM connector types.
 */
typedef enum
{
    CONN_TYPE_RSVD = 0,     /**< Reserved, for compatibility with legacy systems. */
    CONN_TYPE_RSVD1,        /**< Reserved, Shall Not be used. */
    CONN_TYPE_RECEPTACLE,   /**< USB Type-C Receptacle. */
    CONN_TYPE_PLUG          /**< USB Type-C Plug. */
} std_vdm_conn_t;

/**
 * @typedef std_vdm_ver_t
 * @brief Enum for the standard VDM version.
 */
typedef enum
{
    STD_VDM_VER1 = 0,                   /**< VDM version 1.0 */
    STD_VDM_VER2,                       /**< VDM version 2.0 */
    STD_VDM_VER3,                       /**< VDM version 3.0 */
    STD_VDM_VER4                        /**< VDM version 4.0 */
} std_vdm_ver_t;

/**
 * @typedef pe_cbl_state_t
 * @brief Enum of the Policy Engine cable discovery states.
 */
typedef enum
{
    CBL_FSM_DISABLED = 0,                       /**< Cable state machine is inactive. */
    CBL_FSM_ENTRY,                              /**< Cable state machine starting up. */
    CBL_FSM_SEND_SOFT_RESET,                    /**< Cable state machine sending Soft Reset to cable marker. */
    CBL_FSM_SEND_DSC_ID                         /**< Cable state machine waiting for cable response. */
} pe_cbl_state_t;

/**
 * @typedef dpm_typec_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types that can be initiated through
 * the dpm_typec_command API.
 * @see dpm_typec_command
 */
typedef enum
{
    DPM_CMD_SET_RP_DFLT = 0,            /**< Command to select Default Rp. */
    DPM_CMD_SET_RP_1_5A,                /**< Command to select 1.5 A Rp. */
    DPM_CMD_SET_RP_3A                   /**< Command to select 3 A Rp. */,
    DPM_CMD_PORT_DISABLE,               /**< Command to disable the USB-PD port. */
    DPM_CMD_TYPEC_ERR_RECOVERY,         /**< Command to initiate Type-C error recovery. */
    DPM_CMD_TYPEC_INVALID               /**< Invalid command type. */
} dpm_typec_cmd_t;

/**
 * @typedef dpm_typec_cmd_resp_t
 * @brief Enum of the DPM (Device Policy Manager) response types.
 */
typedef enum
{
    DPM_RESP_FAIL = 0,                  /**< Command failed. */
    DPM_RESP_SUCCESS                    /**< Command succeeded. */
} dpm_typec_cmd_resp_t;

/**
 * @typedef typec_fsm_state_t
 * @brief Enum of the Type-C FSM states. This is for internal stack usage.
 * @warning The ordering of elements must not be altered unless the state table
 * is also updated to match.
 */
typedef enum
{
    TYPEC_FSM_DISABLED = 0,             /**< Type-C state machine is disabled. */
    TYPEC_FSM_ERR_RECOV,                /**< Error Recovery state. */
    TYPEC_FSM_ATTACH_WAIT,              /**< AttachWait.SRC or AttachWait.SNK state. */
#if (!(CCG_TRY_SRC_SNK_DISABLE))
    TYPEC_FSM_TRY_SRC,                  /**< Try.SRC state. */
    TYPEC_FSM_TRY_WAIT_SNK,             /**< TryWait.SNK state. */
    TYPEC_FSM_TRY_SNK,                  /**< Try.SNK state. */
    TYPEC_FSM_TRY_WAIT_SRC,             /**< TryWait.SRC state. */
#endif /* (!(CCG_TRY_SRC_SNK_DISABLE)) */
#if (!CCG_SINK_ONLY)
    TYPEC_FSM_UNATTACHED_SRC,           /**< Unattached.SRC state. */
#endif /* (!CCG_SINK_ONLY) */
#if (!(CCG_SOURCE_ONLY))
    TYPEC_FSM_UNATTACHED_SNK,           /**< Unattached.SNK state. */
#endif /* (!(CCG_SOURCE_ONLY)) */
#if (!CCG_SINK_ONLY)
    TYPEC_FSM_UNATTACHED_WAIT_SRC,      /**< UnattachedWait.SRC state. */
#endif /* (!CCG_SINK_ONLY) */
    TYPEC_FSM_AUD_ACC,                  /**< AudioAccessory state. */
    TYPEC_FSM_DBG_ACC,                  /**< DebugAccessory state. */
#if (!CCG_SINK_ONLY)
    TYPEC_FSM_ATTACHED_SRC,             /**< Attached.SRC state. */
#endif /* (!CCG_SINK_ONLY) */
#if (!(CCG_SOURCE_ONLY))
    TYPEC_FSM_ATTACHED_SNK,             /**< Attached.SNK state. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    TYPEC_FSM_MAX_STATES                /**< Invalid Type-C state. */
} typec_fsm_state_t;

/**
 * @enum pe_fsm_state_t
 * @brief Enumeration of Policy Engine states for a USB-PD port. This is for internal stack usage.
 * @warning The ordering of elements must not be altered unless the state table in the stack
 * source is also updated.
 */
typedef enum
{
    PE_FSM_OFF = 0,                             /**< 00: Policy Engine not started. */
    PE_FSM_HR_SEND,                             /**< 01: Send HardReset */
#if (!CCG_SINK_ONLY)
    PE_FSM_HR_SRC_TRANS_DFLT,                   /**< 02: PE_SRC_Transition_to_default */
    PE_FSM_HR_SRC_RECOVER,                      /**< 03: Policy Engine waiting for recovery before enabling VBus. */
    PE_FSM_HR_SRC_VBUS_ON,                      /**< 04: Policy Engine enabling VBus after Hard Reset completion. */
#endif /* (!CCG_SINK_ONLY) */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_HR_SNK_TRANS_DFLT,                   /**< 05: PE_SNK_Transition_to_default */
    PE_FSM_HR_SNK_WAIT_VBUS_OFF,                /**< 06: Policy Engine waiting for VBus turning off. */
    PE_FSM_HR_SNK_WAIT_VBUS_ON,                 /**< 07: Policy Engine waiting for VBus to turn back on. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_BIST_TEST_DATA,                      /**< 08: BIST test data state. */
    PE_FSM_BIST_CM2,                            /**< 09: PE_BIST_Carrier_Mode */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_SNK_STARTUP,                         /**< 10: PE_SNK_Startup */
    PE_FSM_SNK_WAIT_FOR_CAP,                    /**< 11: PE_SNK_Wait_for_Capabilities */
    PE_FSM_SNK_EVAL_CAP,                        /**< 12: PE_SNK_Evaluate_Capability */
    PE_FSM_SNK_SEL_CAP,                         /**< 13: PE_SNK_Select_Capability */
#endif /* (!(CCG_SOURCE_ONLY)) */
#if (!CCG_SINK_ONLY)
    PE_FSM_SRC_STARTUP,                         /**< 14: PE_SRC_Startup */
    PE_FSM_SRC_WAIT_NEW_CAP,                    /**< 15: PE_SRC_Wait_New_Capabilities */
#if (!(CCG_CBL_DISC_DISABLE))
    PE_FSM_SRC_SEND_CBL_SR,                     /**< 16: PE_CBL_Soft_Reset */
    PE_FSM_SRC_SEND_CBL_DSCID,                  /**< 17: PE_CBL_Get_Identity */
#endif /* (!(CCG_CBL_DISC_DISABLE)) */
    PE_FSM_SRC_SEND_CAP,                        /**< 18: PE_SRC_Send_Capabilities */
    PE_FSM_SRC_DISCOVERY,                       /**< 19: PE_SRC_Discovery */
    PE_FSM_SRC_NEG_CAP,                         /**< 20: PE_SRC_Negotiate_Capability */
    PE_FSM_SRC_TRANS_SUPPLY,                    /**< 21: PE_SRC_Transition_Supply */
    PE_FSM_SRC_SEND_PS_RDY,                     /**< 22: Policy Engine waiting to send PS_RDY. */
#endif /* (!CCG_SINK_ONLY) */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_SNK_TRANS,                           /**< 23: PE_SNK_Transition_Sink */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_SR_SEND,                             /**< 24: Policy Engine sending Soft Reset. */
    PE_FSM_SR_RCVD,                             /**< 25: Policy Engine received Soft Reset. */
    PE_FSM_VRS_VCONN_ON,                        /**< 26: Policy Engine waiting for VConn to turn on. */
    PE_FSM_VRS_VCONN_OFF,                       /**< 27: Policy Engine waiting for VConn to turn off. */
    PE_FSM_SWAP_EVAL,                           /**< 28: Evaluate received swap command. */
    PE_FSM_SWAP_SEND,                           /**< 29: Waiting to send swap command. */
    PE_FSM_DRS_CHANGE_ROLE,                     /**< 30: Change data role. */
#if ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))
    PE_FSM_PRS_SRC_SNK_TRANS,                   /**< 31: Source to Sink PR_Swap transition start. */
    PE_FSM_PRS_SRC_SNK_VBUS_OFF,                /**< 32: Initial source waiting for VBus turning off. */
    PE_FSM_PRS_SRC_SNK_WAIT_PS_RDY,             /**< 33: Initial source waiting for PS_RDY. */
    PE_FSM_PRS_SNK_SRC_WAIT_PS_RDY,             /**< 34: Initial sink waiting for PS_RDY. */
    PE_FSM_PRS_SNK_SRC_VBUS_ON,                 /**< 35: Initial sink turning VBus ON. */
    PE_FSM_FRS_CHECK_RP,                        /**< 36: Initial sink checking Rp to send FR_Swap message. */
    PE_FSM_FRS_SRC_SNK_CC_SIGNAL,               /**< 37: Initial source sending FR_Swap signal. */
#endif /* ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY)) */
    PE_FSM_READY,                               /**< 38: PE_Ready state. */
    PE_FSM_SEND_MSG,                            /**< 39: Policy Engine sending new AMS. */
    PE_FSM_EVAL_DATA_RESET,                     /**< 40: Policy Engine Handling Data_Reset request. */
    PE_FSM_SEND_DATA_RESET,                     /**< 41: Policy Engine initiating Data_Reset request. */
    PE_FSM_EVAL_ENTER_USB,                      /**< 42: Policy Engine handling Enter USB request. */
    PE_FSM_MAX_STATES                           /**< 43: Invalid Policy Engine state. */
}pe_fsm_state_t;

/**
 * @typedef pd_contract_status_t
 * @brief Enum of possible PD contract negotiation scenarios that are used to
 * signal the application event handler. This status will be reported in byte 0
 * of the event data passed along with the APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE
 * event. Bytes 3:1 of the event data are not used; and bytes 7:4 will report
 * the RDO where applicable.
 */
typedef enum
{
    PD_CONTRACT_NEGOTIATION_SUCCESSFUL      = 0x01,     /**< PD contract negotiation successful. */
    PD_CONTRACT_CAP_MISMATCH_DETECTED       = 0x03,     /**< PD contract negotiated, but capability mismatch
                                                             is present. */
    PD_CONTRACT_REJECT_CONTRACT_VALID       = 0x00,     /**< Contract rejected by CCG, but previous contract
                                                             is still valid. */
    PD_CONTRACT_REJECT_CONTRACT_NOT_VALID   = 0x04,     /**< Contract rejected by CCG and previous contract
                                                             became invalid. */
    PD_CONTRACT_REJECT_NO_CONTRACT          = 0x08,     /**< Contract rejected by CCG and there was no previous
                                                             contract. */
    PD_CONTRACT_REJECT_EXPLICIT_CONTRACT    = 0x0C,     /**< Request rejected by port partner while in previous
                                                             explicit contract. */
    PD_CONTRACT_REJECT_NO_EXPLICIT_CONTRACT = 0x10,     /**< Request rejected by port partner with no previous
                                                             explicit contract. */
    PD_CONTRACT_PS_READY_NOT_RECEIVED       = 0x14,     /**< Failed to receive PS_RDY after Accept. */
    PD_CONTRACT_PS_READY_NOT_SENT           = 0x18      /**< Failed to send PS_RDY after Accept. */
} pd_contract_status_t;

/**
 * @typedef app_evt_t
 * @brief Enum of events that are signalled to the application.
 */
typedef enum
{
    ROM_APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS,         /**< 0x00: Unexpected high voltage seen on VBus. */
    ROM_APP_EVT_TYPE_C_ERROR_RECOVERY,              /**< 0x01: Type-C error recovery initiated. */
    ROM_APP_EVT_CONNECT,                            /**< 0x02: Type-C connect detected. */
    ROM_APP_EVT_DISCONNECT,                         /**< 0x03: Type-C disconnect(detach) detected. */
    ROM_APP_EVT_EMCA_DETECTED,                      /**< 0x04: Cable (EMCA) discovery successful. */
    ROM_APP_EVT_EMCA_NOT_DETECTED,                  /**< 0x05: Cable (EMCA) discovery timed out. */
    ROM_APP_EVT_ALT_MODE,                           /**< 0x06: Alternate mode related event. */
    ROM_APP_EVT_APP_HW,                             /**< 0x07: MUX control related event. */
    ROM_APP_EVT_BB,                                 /**< 0x08: Billboard status change. */
    ROM_APP_EVT_RP_CHANGE,                          /**< 0x09: Rp termination change detected. */
    ROM_APP_EVT_HARD_RESET_RCVD,                    /**< 0x0A: Hard Reset received. */
    ROM_APP_EVT_HARD_RESET_COMPLETE,                /**< 0x0B: Hard Reset processing completed. */
    ROM_APP_EVT_PKT_RCVD,                           /**< 0x0C: New PD message received. */
    ROM_APP_EVT_PR_SWAP_COMPLETE,                   /**< 0x0D: PR_SWAP process completed. */
    ROM_APP_EVT_DR_SWAP_COMPLETE,                   /**< 0x0E: DR_SWAP process completed. */
    ROM_APP_EVT_VCONN_SWAP_COMPLETE,                /**< 0x0F: VConn_SWAP process completed. */
    ROM_APP_EVT_SENDER_RESPONSE_TIMEOUT,            /**< 0x10: Sender response timeout occurred. */
    ROM_APP_EVT_VENDOR_RESPONSE_TIMEOUT,            /**< 0x11: Vendor message response timeout occurred. */
    ROM_APP_EVT_HARD_RESET_SENT,                    /**< 0x12: Hard Reset sent by CCG. */
    ROM_APP_EVT_SOFT_RESET_SENT,                    /**< 0x13: Soft Reset sent by CCG. */
    ROM_APP_EVT_CBL_RESET_SENT,                     /**< 0x14: Cable Reset sent by CCG. */
    ROM_APP_EVT_PE_DISABLED,                        /**< 0x15: PE.Disabled state entered. */
    ROM_APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE,   /**< 0x16: Contract negotiation completed. */
    ROM_APP_EVT_VBUS_OVP_FAULT,                     /**< 0x17: VBus Over Voltage fault detected. */
    ROM_APP_EVT_VBUS_OCP_FAULT,                     /**< 0x18: VBus Over Current fault detected. */
    ROM_APP_EVT_VCONN_OCP_FAULT,                    /**< 0x19: VConn Over Current fault detected. */
    ROM_APP_EVT_VBUS_PORT_DISABLE,                  /**< 0x1A: PD port disable completed. */
    ROM_APP_EVT_TYPEC_STARTED,                      /**< 0x1B: PD port enable (start) completed. */
    ROM_APP_EVT_FR_SWAP_COMPLETE,                   /**< 0x1C: FR_SWAP process completed. */
    ROM_APP_EVT_TEMPERATURE_FAULT,                  /**< 0x1D: Over Temperature fault detected. */
    ROM_APP_EVT_HANDLE_EXTENDED_MSG,                /**< 0x1E: Extended message received and needs to be handled. */
    ROM_APP_EVT_VBUS_UVP_FAULT,                     /**< 0x1F: VBus Under Voltage fault detected. */
    ROM_APP_EVT_VBUS_SCP_FAULT,                     /**< 0x20: VBus Short Circuit fault detected. */
    ROM_APP_EVT_TYPEC_ATTACH_WAIT,                  /**< 0x21: Type-C AttachWait state entered. For internal use only. */
    ROM_APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED,    /**< 0x22: Type-C transition from AttachWait to Unattached. For
                                                           internal use only. */
    ROM_APP_EVT_TYPEC_ATTACH,                       /**< 0x23: Type-C attach event. */
    ROM_APP_EVT_CC_OVP,                             /**< 0x24: Over Voltage on CC/VConn line detected. */
    ROM_APP_EVT_SBU_OVP,                            /**< 0x25: Over Voltage on SBU1/SBU2 line detected. */
    ROM_APP_EVT_ALERT_RECEIVED,                     /**< 0x26: Alert message received. For internal use only. */
    ROM_APP_EVT_SRC_CAP_TRIED_WITH_NO_RESPONSE,     /**< 0x27: Src Cap tried with no response. For internal use only. */
    ROM_APP_EVT_PD_SINK_DEVICE_CONNECTED,           /**< 0x28: Sink device connected. For internal use only. */
    ROM_APP_EVT_VBUS_RCP_FAULT,                     /**< 0x29: VBus Reverse Current fault detected. */
    ROM_APP_EVT_STANDBY_CURRENT,                    /**< 0x2A: Standby Current. */
    ROM_APP_EVT_DATA_RESET_RCVD,                    /**< 0x2B: USB4 Data Reset message received. USB connection
                                                           should be disabled by DFP on receiving this event. */
    ROM_APP_EVT_DATA_RESET_SENT,                    /**< 0x2C: USB4 Data Reset message sent. USB connection should
                                                           be disabled by DFP on receiving this event. */
    ROM_APP_EVT_DATA_RESET_CPLT,                    /**< 0x2D: USB4 Data Reset process complete. No handling required. */
    ROM_APP_EVT_USB_ENTRY_CPLT,                     /**< 0x2E: USB4 entry process complete. */
    ROM_APP_EVT_DATA_RESET_ACCEPTED,                /**< 0x2F: USB4 Data Reset Accepted. USB connection can be
                                                           re-enabled by DFP on receiving this event. */
    ROM_APP_EVT_CONFIG_ERROR,                       /**< 0x30: Configuration table error event. */
    ROM_APP_EVT_POWER_CYCLE,                        /**< 0x31: Power cycle / Reset event. */
    ROM_APP_EVT_VBUS_IN_UVP_FAULT,                  /**< 0x32: Vbus_in undervoltage fault detected. */
    ROM_APP_EVT_VBUS_IN_OVP_FAULT,                  /**< 0x33: Vbus_in overvoltage fault detected. */
    ROM_APP_EVT_SYSTEM_OT_FAULT,                    /**< 0x34: System overtemperature fault detected. */
    ROM_APP_EVT_CRC_ERROR,                          /**< 0x35: PD CRC error detected. */
    ROM_APP_EVT_HR_PSRC_ENABLE,                     /**< 0x36: PSRC enable is about to be called after Hard reset. */

    ROM_APP_EVT_TYPEC_RP_DETACH,                    /**< 0x37: Rp removal detected while in the Attached.SNK state. */
    ROM_APP_EVT_PR_SWAP_ACCEPTED,                   /**< 0x38: PR-SWAP accepted by CCG or port partner. */
    ROM_APP_EVT_HR_SENT_RCVD_DEFERRED,              /**< 0x39: Deferred Hard Reset Sent/Received event handling to
                                                           accommodate retimer communication delay timing. */
    ROM_APP_EVT_BAD_SINK_APDO_SEL,                  /**< 0x3A: APDO selection in PD 2.0 or less revision */

    ROM_APP_EVT_BC_DETECTION_COMPLETED,             /**< 0x3B: Legacy battery charging protocol detection completed . */

    ROM_APP_EVT_SNK_FET_ENABLE,                     /**< 0x3C: HPI Enable  SNK FET cmd event. */
    ROM_APP_EVT_SNK_FET_DISABLE,                    /**< 0x3D: HPI Disable SNK FET cmd event. */
    ROM_APP_EVT_SAFE_PWR_ENABLE,                    /**< 0x3E: HPI Enable Safe PWR path cmd event. */
    ROM_APP_EVT_SAFE_PWR_DISABLE,                   /**< 0x3F: HPI Disable Safe PWR path cmd event. */
    ROM_APP_EVT_FAULT_CLEANED,                      /**< 0x40: OVP/OCP/OTP fault state is cleaned. */
    ROM_APP_EVT_MISMATCH_CLEANED,                   /**< 0x41: MISMATCH fault state is cleaned. */

    ROM_APP_EVT_CUST_ALT_MODE_CHANGED,              /**< 0x42: This is a custom event that could be set for notification of alternate
                                                           mode specific conditions like mode entry and mode exit. */
    ROM_APP_EVT_CUST_MODE_DISC_CMPL,                /**< 0x43: Apple sequencing is finished . */

    ROM_APP_EVT_VBAT_GND_SCP_FAULT,                 /**< 0x3B: Battery to ground short circuit fault detected. */
    ROM_APP_EVT_VIN_UVP_FAULT,                      /**< 0x3C: Regulator input under voltage fault detected. */
    ROM_APP_EVT_VIN_OVP_FAULT,                      /**< 0x3D: Regulator input over voltage fault detected. */
    ROM_APP_EVT_BIST_STM_ENTRY,                     /**< 0x3E: BIST STM Entry event. */
    ROM_APP_EVT_BIST_STM_EXIT,                      /**< 0x3F: BIST STM Exit event. */
    ROM_APP_EVT_ILIM_FAULT,                         /**< 0x40: Inductor Limit fault detected. */
    ROM_APP_EVT_VREG_INRUSH_FAULT,                  /**< 0x41: Vreg inrush detect fault detected. */
    ROM_APP_EVT_VREG_BOD_FAULT,                     /**< 0x42: Device Brown Out Detect fault detected. */
    ROM_APP_EVT_VCONN_SCP_FAULT,                    /**< 0x43: VConn Short Circuit fault detected. */
    ROM_APP_TOTAL_EVENTS                            /**< 0x44: Total number of application events. */
} app_evt_t;

/**
 * @typedef pd_ams_type
 * @brief Type of USB-PD Atomic Message Sequence (AMS).
 */
typedef enum
{
    PD_AMS_NONE = 0,                            /**< No AMS active. */
    PD_AMS_NON_INTR,                            /**< Non-interruptible AMS is active. */
    PD_AMS_INTR                                 /**< Interruptible AMS is active. */
} pd_ams_type;

/**
 * @typedef vdm_ams_t
 * @brief Enumeration of application responses to policy manager.
 */
typedef enum
{
    VDM_AMS_RESP_READY = 0,                     /**< Response is ready */
    VDM_AMS_RESP_NOT_REQ,                       /**< No response required */
    VDM_AMS_RESP_FROM_EC,                       /**< Response will come from EC */
    VDM_AMS_RESP_NOT_SUPP                       /**< Send a NOT_SUPPORTED response. */
} vdm_ams_t;
/** \} group_ccgxAppCommon_enums */
/*****************************************************************************
 * Data Structure Definitions
 ****************************************************************************/
/** @cond DOXYGEN_HIDE */
typedef struct {
    volatile uint32_t ctrl;                               /* 0x40050000u */
    volatile uint32_t status;                             /* 0x40050004u */
    volatile uint32_t cmd_resp_ctrl;                      /* 0x40050008u */
    volatile uint32_t cmd_resp_status;                    /* 0x4005000cu */
    volatile uint32_t rsrvd0[4];
    volatile uint32_t spi_ctrl;                           /* 0x40050020u */
    volatile uint32_t spi_status;                         /* 0x40050024u */
    volatile uint32_t rsrvd1[6];
    volatile uint32_t uart_ctrl;                          /* 0x40050040u */
    volatile uint32_t uart_tx_ctrl;                       /* 0x40050044u */
    volatile uint32_t uart_rx_ctrl;                       /* 0x40050048u */
    volatile uint32_t uart_rx_status;                     /* 0x4005004cu */
    volatile uint32_t uart_flow_ctrl;                     /* 0x40050050u */
    volatile uint32_t rsrvd2[3];
    volatile uint32_t i2c_ctrl;                           /* 0x40050060u */
    volatile uint32_t i2c_status;                         /* 0x40050064u */
    volatile uint32_t i2c_m_cmd;                          /* 0x40050068u */
    volatile uint32_t i2c_s_cmd;                          /* 0x4005006cu */
    volatile uint32_t i2c_cfg;                            /* 0x40050070u */
    volatile uint32_t rsrvd3[99];
    volatile uint32_t tx_ctrl;                            /* 0x40050200u */
    volatile uint32_t tx_fifo_ctrl;                       /* 0x40050204u */
    volatile uint32_t tx_fifo_status;                     /* 0x40050208u */
    volatile uint32_t rsrvd4[13];
    volatile uint32_t tx_fifo_wr;                         /* 0x40050240u */
    volatile uint32_t rsrvd5[47];
    volatile uint32_t rx_ctrl;                            /* 0x40050300u */
    volatile uint32_t rx_fifo_ctrl;                       /* 0x40050304u */
    volatile uint32_t rx_fifo_status;                     /* 0x40050308u */
    volatile uint32_t rsrvd6;
    volatile uint32_t rx_match;                           /* 0x40050310u */
    volatile uint32_t rsrvd7[11];
    volatile uint32_t rx_fifo_rd;                         /* 0x40050340u */
    volatile uint32_t rx_fifo_rd_silent;                  /* 0x40050344u */
    volatile uint32_t rsrvd8[46];
    volatile uint32_t ez_data[32];                        /* 0x40050400u */
    volatile uint32_t rsrvd9[608];
    volatile uint32_t intr_cause;                         /* 0x40050e00u */
    volatile uint32_t rsrvd10[31];
    volatile uint32_t intr_i2c_ec;                        /* 0x40050e80u */
    volatile uint32_t rsrvd11;
    volatile uint32_t intr_i2c_ec_mask;                   /* 0x40050e88u */
    volatile uint32_t intr_i2c_ec_masked;                 /* 0x40050e8cu */
    volatile uint32_t rsrvd12[12];
    volatile uint32_t intr_spi_ec;                        /* 0x40050ec0u */
    volatile uint32_t rsrvd13;
    volatile uint32_t intr_spi_ec_mask;                   /* 0x40050ec8u */
    volatile uint32_t intr_spi_ec_masked;                 /* 0x40050eccu */
    volatile uint32_t rsrvd14[12];
    volatile uint32_t intr_m;                             /* 0x40050f00u */
    volatile uint32_t intr_m_set;                         /* 0x40050f04u */
    volatile uint32_t intr_m_mask;                        /* 0x40050f08u */
    volatile uint32_t intr_m_masked;                      /* 0x40050f0cu */
    volatile uint32_t rsrvd15[12];
    volatile uint32_t intr_s;                             /* 0x40050f40u */
    volatile uint32_t intr_s_set;                         /* 0x40050f44u */
    volatile uint32_t intr_s_mask;                        /* 0x40050f48u */
    volatile uint32_t intr_s_masked;                      /* 0x40050f4cu */
    volatile uint32_t rsrvd16[12];
    volatile uint32_t intr_tx;                            /* 0x40050f80u */
    volatile uint32_t intr_tx_set;                        /* 0x40050f84u */
    volatile uint32_t intr_tx_mask;                       /* 0x40050f88u */
    volatile uint32_t intr_tx_masked;                     /* 0x40050f8cu */
    volatile uint32_t rsrvd17[12];
    volatile uint32_t intr_rx;                            /* 0x40050fc0u */
    volatile uint32_t intr_rx_set;                        /* 0x40050fc4u */
    volatile uint32_t intr_rx_mask;                       /* 0x40050fc8u */
    volatile uint32_t intr_rx_masked;                     /* 0x40050fccu */
} SCB_PRT_REGS_T, *PSCB_PRT_REGS_T;

typedef struct {
    volatile uint32_t PORT_SEL;                           /* 0x40020000u */
} HSIOM_REGS_T, *PHSIOM_REGS_T;

typedef struct {
    volatile uint32_t DR;                                 /* 0x40040000u */
    volatile uint32_t PS;                                 /* 0x40040004u */
    volatile uint32_t PC;                                 /* 0x40040008u */
    volatile uint32_t INTR_CFG;                           /* 0x4004000cu */
    volatile uint32_t INTR;                               /* 0x40040010u */
    volatile uint32_t RSRVD0;
    volatile uint32_t PC2;                                /* 0x40040018u */
    volatile uint32_t RESERVED1[9];
    volatile uint32_t DR_SET;                             /* 0x40040040u */
    volatile uint32_t DR_CLR;                             /* 0x40040044u */
    volatile uint32_t DR_INV;                             /* 0x40040048u */
    volatile uint32_t RESERVED2[1005];
    volatile uint32_t INTR_CAUSE;                         /* 0x40041000u */
    volatile uint32_t RESERVED3[3];
    volatile uint32_t DFT_IO_TEST;                        /* 0x40041010u */
} GPIO_REGS_T, *PGPIO_REGS_T;

#define GPIO0_BASE_ADDR                                  (0x40040000u)
#define GPIO1_BASE_ADDR                                  (0x40040100u)
#define GPIO2_BASE_ADDR                                  (0x40040200u)
#define GPIO3_BASE_ADDR                                  (0x40040300u)

#define GPIO0       ((PGPIO_REGS_T) GPIO0_BASE_ADDR)
#define GPIO1       ((PGPIO_REGS_T) GPIO1_BASE_ADDR)
#define GPIO2       ((PGPIO_REGS_T) GPIO2_BASE_ADDR)
#define GPIO3       ((PGPIO_REGS_T) GPIO3_BASE_ADDR)

#define HSIOM0_BASE_ADDR                                 (0x40020000u)
#define HSIOM1_BASE_ADDR                                 (0x40020100u)
#define HSIOM2_BASE_ADDR                                 (0x40020200u)
#define HSIOM3_BASE_ADDR                                 (0x40020300u)

#define HSIOM0       ((PHSIOM_REGS_T) HSIOM0_BASE_ADDR)
#define HSIOM1       ((PHSIOM_REGS_T) HSIOM1_BASE_ADDR)
#define HSIOM2       ((PHSIOM_REGS_T) HSIOM2_BASE_ADDR)
#define HSIOM3       ((PHSIOM_REGS_T) HSIOM3_BASE_ADDR)

#define SCB_PRT0_BASE_ADDR                               (0x40050000u)
#define SCB_PRT1_BASE_ADDR                               (0x40060000u)
#define SCB_PRT2_BASE_ADDR                               (0x40070000u)
#define SCB_PRT3_BASE_ADDR                               (0x40080000u)

#define SCB_PRT0       ((PSCB_PRT_REGS_T) SCB_PRT0_BASE_ADDR)
#define SCB_PRT1       ((PSCB_PRT_REGS_T) SCB_PRT1_BASE_ADDR)
#define SCB_PRT2       ((PSCB_PRT_REGS_T) SCB_PRT2_BASE_ADDR)
#define SCB_PRT3       ((PSCB_PRT_REGS_T) SCB_PRT3_BASE_ADDR)
/** @endcond */

/** \addtogroup group_ccgxAppCommon_data_structures
* \{
*/
/**
 * @brief Union to hold CC status.
 */
typedef union cc_state
{
    uint16_t state;                     /**< Combined status of CC1 and CC2. */
    uint8_t  cc[2];                     /**< Individual status of CC1(cc[0]) and CC2(cc[1]). */
} cc_state_t;

/**
 * @brief Union to hold the PD header defined by the USB-PD specification. Lower 16 bits hold the message
 * header and the upper 16 bits hold the extended message header (where applicable).
 */
typedef union
{
    uint32_t val;                               /**< Header expressed as a 32-bit word. */

    /** @brief PD message header broken down into component fields. Includes 2-byte extended message header. */
    struct ROM_PD_HDR
    {
        uint32_t msg_type   : 5;                /**< Bits 04:00 - Message type. */
        uint32_t data_role  : 1;                /**< Bit     05 - Data role. */
        uint32_t spec_rev   : 2;                /**< Bits 07:06 - Spec revision. */
        uint32_t pwr_role   : 1;                /**< Bit     08 - Power role. */
        uint32_t msg_id     : 3;                /**< Bits 11:09 - Message ID. */
        uint32_t len        : 3;                /**< Bits 14:12 - Number of data objects. */
        uint32_t extd       : 1;                /**< Bit     15 - Extended message. */
        uint32_t data_size  : 9;                /**< Bits 24:16 - Extended message size in bytes. */
        uint32_t rsvd1      : 1;                /**< Bit     25 - Reserved. */
        uint32_t request    : 1;                /**< Bit     26 - Chunk request. */
        uint32_t chunk_no   : 4;                /**< Bits 30:27 - Chunk number. */
        uint32_t chunked    : 1;                /**< Bit     31 - Chunked message. */
    } rom_hdr;                                      /**< PD message header split into component fields. */
} pd_hdr_t;

/**
 * @brief Union to hold the PD extended header.
 */
typedef union
{
    uint16_t val;                               /**< Extended header expressed as 2-byte integer value. */

    /** @brief Extended header broken down into respective fields. */
    struct ROM_EXTD_HDR_T
    {
        uint16_t data_size  : 9;                /**< Bits 08:00 - Extended message size in bytes. */
        uint16_t rsvd1      : 1;                /**< Bit     09 - Reserved. */
        uint16_t request    : 1;                /**< Bit     10 - Chunk request. */
        uint16_t chunk_no   : 4;                /**< Bits 14:11 - Chunk number. */
        uint16_t chunked    : 1;                /**< Bit     15 - Chunked message. */
    } extd;                                     /**< Extended header broken down into respective fields. */
} pd_extd_hdr_t;

/**
 * @union pd_do_t
 * @brief Union to hold a PD data object. All USB-PD data objects are 4-byte values which are interpreted
 * according to the message type, length and object position. This union represents all possible interpretations
 * of a USB-PD data object.
 */
typedef union
{

    uint32_t val;                                   /**< Data object interpreted as an unsigned integer value. */

    /** @brief Structure of a BIST data object. */
    struct ROM_BIST_DO
    {
        uint32_t rsvd1                      : 16;   /**< Reserved field. */
        uint32_t rsvd2                      : 12;   /**< Reserved field. */
        uint32_t mode                       : 4;    /**< BIST mode. */
    } bist_do;                                      /**< DO interpreted as a BIST data object. */

    /** @brief Structure representing a Fixed Supply PDO - Source. */
    struct ROM_FIXED_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 100mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t pk_current                 : 2;    /**< Peak current. */
        uint32_t reserved                   : 2;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t usb_suspend_sup            : 1;    /**< USB suspend supported. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_src;                                    /**< DO interpreted as a Fixed Supply PDO - Source. */

    /** @brief Structure representing a Variable Supply PDO - Source. */
    struct ROM_VAR_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_src;                                      /**< DO interpreted as a Variable Supply PDO - Source. */

    /** @brief Structure representing a Battery Supply PDO - Source. */
    struct ROM_BAT_SRC
    {
        uint32_t max_power                  : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_src;                                      /**< DO interpreted as a Battery Supply PDO - Source. */

    /** @brief Structure representing a generic source PDO. */
    struct ROM_SRC_GEN
    {
        uint32_t max_cur_power              : 10;   /**< Maximum current in 10 mA or power in 250 mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type. */
    } src_gen;                                      /**< DO interpreted as a generic PDO - Source. */

    /** @brief Structure representing a Fixed Supply PDO - Sink. */
    struct ROM_FIXED_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t rsrvd                      : 3;    /**< Reserved field. */
        uint32_t fr_swap                    : 2;    /**< FR swap support. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t high_cap                   : 1;    /**< Higher capability possible. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_snk;                                    /**< DO interpreted as a Fixed Supply PDO - Sink. */

    /** @brief Structure representing a Variable Supply PDO - Sink. */
    struct ROM_VAR_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_snk;                                      /**< DO interpreted as a Variable Supply PDO - Sink. */

    /** @brief Structure representing a Battery Supply PDO - Sink. */
    struct ROM_BAT_SNK
    {
        uint32_t op_power                   : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_snk;                                      /**< DO interpreted as a Battery Supply PDO - Sink. */

    /** @brief Structure representing a Fixed or Variable Request Data Object. */
    struct ROM_RDO_FIXED_VAR
    {
        uint32_t max_op_current             : 10;   /**< Maximum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_fix_var;                                  /**< DO interpreted as a fixed/variable request. */

    /** @brief Structure representing a Fixed or Variable Request Data Object with GiveBack. */
    struct ROM_RDO_FIXED_VAR_GIVEBACK
    {
        uint32_t min_op_current             : 10;   /**< Minimum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_fix_var_gvb;                              /**< DO interpreted as a fixed/variable request with giveback. */

    /** @brief Structure representing a Battery Request Data Object. */
    struct ROM_RDO_BAT
    {
        uint32_t max_op_power               : 10;   /**< Maximum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_bat;                                      /**< DO interpreted as a Battery request. */

    /** @brief Structure representing a Battery Request Data Object with GiveBack. */
    struct ROM_RDO_BAT_GIVEBACK
    {
        uint32_t min_op_power               : 10;   /**< Minimum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_bat_gvb;                                  /**< DO interpreted as a Battery request with giveback. */

    /** @brief Structure representing a generic Request Data Object. */
    struct ROM_RDO_GEN
    {
        uint32_t min_max_power_cur          : 10;   /**< Min/Max power or current requirement. */
        uint32_t op_power_cur               : 10;   /**< Operating power or current requirement. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack supported flag = 0. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_gen;                                      /**< DO interpreted as a generic request message. */

    /** @brief Structure representing a Generic Request Data Object with GiveBack. */
    struct ROM_RDO_GEN_GVB
    {
        uint32_t max_power_cur              : 10;   /**< Min/Max power or current requirement. */
        uint32_t op_power_cur               : 10;   /**< Operating power or current requirement. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack supported flag = 1. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_gen_gvb;                                  /**< DO interpreted as a generic request with giveback. */

    /** @brief Structure representing a Structured VDM Header Data Object. */
    struct ROM_STD_VDM_HDR
    {
        uint32_t cmd                        : 5;    /**< VDM command id. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t cmd_type                   : 2;    /**< VDM command type. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsvd2                      : 2;    /**< Reserved field. */
        uint32_t st_ver                     : 2;    /**< Structured VDM version. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Structured. */
        uint32_t svid                       : 16;   /**< SVID associated with VDM. */
    } std_vdm_hdr;                                  /**< DO interpreted as a Structured VDM header. */

    /** @brief Structure representing an Unstructured VDM header data object as defined by Cypress. */
    struct ROM_USTD_VDM_HDR
    {
        uint32_t cmd                        : 5;    /**< Command id. */
        uint32_t seq_num                    : 4;    /**< Sequence number. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t cmd_type                   : 2;    /**< Command type. */
        uint32_t vdm_ver                    : 2;    /**< VDM version. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Unstructured. */
        uint32_t svid                       : 16;   /**< SVID associated with VDM. */
    } ustd_vdm_hdr;                                 /**< DO interpreted as a Cypress unstructured VDM header. */

    /** @brief Structure representing an Unstructured VDM header data object as defined by QC 5.0/4.0 spec. */
    struct ROM_USTD_QC_PPS_HDR
    {
        uint32_t cmd_0                      : 8;    /**< Command code #0. */
        uint32_t cmd_1                      : 7;    /**< Command code #1. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Unstructured. */
        uint32_t svid                       : 16;   /**< SVID associated with message. */
    } ustd_qc_pps_hdr;                              /**< DO interpreted as a QC 5.0/4.0 Unstructured VDM header. */

    /** @brief Structure representing an Unstructured VDM data object as defined by QC 5.0/4.0 spec. */
    struct ROM_QC_PPS_DATA_VDO
    {
        uint32_t data_0                     : 8;    /**< Command data #0. */
        uint32_t data_1                     : 8;    /**< Command data #1. */
        uint32_t data_2                     : 8;    /**< Command data #2. */
        uint32_t data_3                     : 8;    /**< Command data #3. */
    } qc_pps_data_vdo;                              /**< DO interpreted as a QC 5.0/4.0 Unstructured VDM data object. */

    /** @brief Structure representing a Standard ID_HEADER VDO. */
    struct ROM_STD_VDM_ID_HDR
    {
        uint32_t usb_vid                    : 16;   /**< 16-bit vendor ID. */
#if CCG_PD_REV3_ENABLE
        uint32_t rsvd1                      : 5;    /**< Reserved field. */
        uint32_t conn_type                  : 2;    /**< Connector type. */
        uint32_t prod_type_dfp              : 3;    /**< Product type as DFP. */
#else
        uint32_t rsvd1                      : 10;   /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
        uint32_t mod_support                : 1;    /**< Whether alternate modes are supported. */
        uint32_t prod_type                  : 3;    /**< Product type as UFP. */
        uint32_t usb_dev                    : 1;    /**< USB device supported. */
        uint32_t usb_host                   : 1;    /**< USB host supported. */
    } std_id_hdr;                                   /**< DO interpreted as a Standard ID_HEADER VDO. */

    /** @brief Cert Stat VDO structure. */
    struct ROM_STD_CERT_VDO
    {
        uint32_t usb_xid                    : 32;   /**< 32-bit XID value. */
    } std_cert_vdo;                                 /**< DO interpreted as a Cert Stat VDO. */

    /** @brief Product VDO structure. */
    struct ROM_STD_PROD_VDO
    {
        uint32_t bcd_dev                    : 16;   /**< 16-bit bcdDevice value. */
        uint32_t usb_pid                    : 16;   /**< 16-bit product ID. */
    } std_prod_vdo;                                 /**< DO interpreted as a Product VDO. */

    /** @brief Cable VDO structure as defined in USB-PD r2.0. */
    struct ROM_STD_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t sop_dp                     : 1;    /**< Whether SOP'' controller is present. */
        uint32_t vbus_thru_cbl              : 1;    /**< Whether cable allows VBus power through. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t ssrx2                      : 1;    /**< Whether SSRX2 has configurable direction. */
        uint32_t ssrx1                      : 1;    /**< Whether SSRX1 has configurable direction. */
        uint32_t sstx2                      : 1;    /**< Whether SSTX2 has configurable direction. */
        uint32_t sstx1                      : 1;    /**< Whether SSTX1 has configurable direction. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Whether cable has a plug: Should be 0. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd1                      : 4;    /**< Reserved field. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } std_cbl_vdo;                                  /**< DO interpreted as a PD 2.0 cable VDO. */

    /** @brief Passive cable VDO structure as defined by PD 3.0. */
    struct ROM_PAS_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t rsvd2                      : 2;    /**< Reserved field. */
        uint32_t max_vbus_volt              : 2;    /**< Max. VBus voltage supported. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Reserved field. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd3                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } pas_cbl_vdo;                                  /**< DO interpreted as a PD 3.0 passive cable VDO. */

    /** @brief Active cable VDO structure as defined by PD 3.0. */
    struct ROM_ACT_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t sop_dp                     : 1;    /**< Whether SOP'' controller is present. */
        uint32_t vbus_thru_cbl              : 1;    /**< Whether cable conducts VBus through. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t max_vbus_volt              : 2;    /**< Max. VBus voltage supported. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Reserved field. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } act_cbl_vdo;                                  /**< DO interpreted as a PD 3.0 active cable VDO. */

    /** @brief Active Cable VDO 1 structure as defined by PD 3.0, Version 1.2 */
    struct ROM_ACT_CBL_VDO_1
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t sop_dp                     : 1;    /**< Whether SOP'' controller is present. */
        uint32_t vbus_thru_cbl              : 1;    /**< Whether cable conducts VBus through. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t sbu_type                   : 1;    /**< Whether SBU connections are passive/active. */
        uint32_t sbu_supp                   : 1;    /**< Whether SBU connections are supported, 1=Not supported. */
        uint32_t max_vbus_volt              : 2;    /**< Max. VBus voltage supported. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } act_cbl_vdo1;                                 /**< DO interpreted as a PD 3.0 Active Cable VDO 1. */

    /** @brief Active Cable VDO 2 structure as defined by PD 3.0, Version 1.2 */
    struct ROM_ACT_CBL_VDO_2
    {
        uint32_t usb_gen                    : 1;    /**< USB Generation. */
        uint32_t rsvd0                      : 1;    /**< Reserved field. */
        uint32_t opt_isolated               : 1;    /**< Optically Isolated Active Cable. */
        uint32_t ss_lanes                   : 1;    /**< Whether cable supports 1 or 2 USB 3.2 lanes. */
        uint32_t ss_supp                    : 1;    /**< Whether cable supports USB 3.2 signaling. */
        uint32_t usb2_supp                  : 1;    /**< Whether cable supports USB 2.0 data. */
        uint32_t usb2_hub_hops              : 2;    /**< Number of USB 2.0 hub hops contributed by the cable. */   
        uint32_t usb4_supp                  : 1;    /**< Whether cable supports USB 4. */
        uint32_t active_el                  : 1;    /**< Active element. */   
        uint32_t phy_conn                   : 1;    /**< Physical connection. */        
        uint32_t u3_u0_trans                : 1;    /**< Type of USB U3 to U0 transition. */
        uint32_t u3_power                   : 3;    /**< USB 3.2 U3 power. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t shutdown_temp              : 8;    /**< Shutdown temperature. */
        uint32_t max_op_temp                : 8;    /**< Maximum operating temperature. */
    } act_cbl_vdo2;                                 /**< DO interpreted as a PD 3.0 Active Cable VDO 2. */

    /** @brief AMA VDO structure as defined by PD 2.0. */
    struct ROM_STD_AMA_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported. */
        uint32_t vbus_req                   : 1;    /**< Whether device requires VBus. */
        uint32_t vcon_req                   : 1;    /**< Whether device requires VConn. */
        uint32_t vcon_pwr                   : 3;    /**< VConn power required. */
        uint32_t ssrx2                      : 1;    /**< Whether SSRX2 has configurable direction. */
        uint32_t ssrx1                      : 1;    /**< Whether SSRX1 has configurable direction. */
        uint32_t sstx2                      : 1;    /**< Whether SSTX2 has configurable direction. */
        uint32_t sstx1                      : 1;    /**< Whether SSTX1 has configurable direction. */
        uint32_t rsvd1                      : 12;   /**< Reserved field. */
        uint32_t ama_fw_ver                 : 4;    /**< AMA firmware version. */
        uint32_t ama_hw_ver                 : 4;    /**< AMA hardware version. */
    } std_ama_vdo;                                  /**< DO interpreted as a PD 2.0 AMA VDO. */

    /** @brief AMA VDO structure as defined by PD 3.0. */
    struct ROM_STD_AMA_VDO_PD3
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported. */
        uint32_t vbus_req                   : 1;    /**< Whether device requires VBus. */
        uint32_t vcon_req                   : 1;    /**< Whether device requires VConn. */
        uint32_t vcon_pwr                   : 3;    /**< VConn power required. */
        uint32_t rsvd1                      : 13;   /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t ama_fw_ver                 : 4;    /**< AMA firmware version. */
        uint32_t ama_hw_ver                 : 4;    /**< AMA hardware version. */
    } std_ama_vdo_pd3;                              /**< DO interpreted as a PD 3.0 AMA VDO. */

    /** @brief Discover_SVID response structure. */
    struct ROM_STD_SVID_RESP_VDO
    {
        uint32_t svid_n1                    : 16;   /**< SVID #1 */
        uint32_t svid_n                     : 16;   /**< SVID #2 */
    } std_svid_res;                                 /**< DO interpreted as a DISCOVER_SVID response. */

    /** @brief DisplayPort Mode VDO as defined by VESA spec. */
    struct ROM_STD_DP_VDO
    {
        uint32_t port_cap                   : 2;    /**< Port capability. */
        uint32_t signal                     : 4;    /**< Signalling supported. */
        uint32_t recep                      : 1;    /**< Whether Type-C connector is plug or receptacle. */
        uint32_t usb2_0                     : 1;    /**< USB 2.0 signalling required. */
        uint32_t dfp_d_pin                  : 8;    /**< DFP_D pin assignments supported. */
        uint32_t ufp_d_pin                  : 8;    /**< UFP_D pin assignments supported. */
        uint32_t rsvd                       : 8;    /**< Reserved field. */
    } std_dp_vdo;                                   /**< DO interpreted as a DisplayPort Mode response. */

    /** @brief DisplayPort status update VDO as defined by VESA spec. */
    struct ROM_DP_STATUS_VDO
    {
        uint32_t dfp_ufp_conn               : 2;    /**< Whether DFP_D/UFP_D is connected. */
        uint32_t pwr_low                    : 1;    /**< Low power mode. */
        uint32_t en                         : 1;    /**< DP functionality enabled. */
        uint32_t mult_fun                   : 1;    /**< Multi-function mode preferred. */
        uint32_t usb_cfg                    : 1;    /**< Request switch to USB configuration. */
        uint32_t exit                       : 1;    /**< Exit DP mode request. */
        uint32_t hpd_state                  : 1;    /**< Current HPD state. */
        uint32_t hpd_irq                    : 1;    /**< HPD IRQ status. */
        uint32_t rsvd                       : 23;   /**< Reserved field. */
    } dp_stat_vdo;                                  /**< DO interpreted as a DisplayPort status update. */

    /** @brief DisplayPort configure VDO as defined by VESA spec. */
    struct ROM_DP_CONFIG_VDO
    {
        uint32_t sel_conf                   : 2;    /**< Select configuration. */
        uint32_t sign                       : 4;    /**< Signalling for DP protocol. */
        uint32_t rsvd1                      : 2;    /**< Reserved. */
        uint32_t dfp_asgmnt                 : 8;    /**< DFP_D pin assignment. */
        uint32_t ufp_asgmnt                 : 8;    /**< UFP_D pin assignment. */
        uint32_t rsvd2                      : 8;    /**< Reserved field. */
    } dp_cfg_vdo;                                   /**< DO interpreted as a DisplayPort Configure command. */

    /** @brief Programmable Power Supply Source PDO. */
    struct ROM_PPS_SRC
    {
        uint32_t max_cur                    : 7;    /**< Maximum current in 50 mA units. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t min_volt                   : 8;    /**< Minimum voltage in 100 mV units. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t max_volt                   : 8;    /**< Maximum voltage in 100 mV units. */
        uint32_t rsvd3                      : 2;    /**< Reserved field. */
        uint32_t pps_pwr_limited            : 1;    /**< Whether PPS power has been limited. */
        uint32_t apdo_type                  : 2;    /**< APDO type: Should be 0 for PPS. */
        uint32_t supply_type                : 2;    /**< PDO type: Should be 3 for PPS APDO. */
    } pps_src;                                      /**< DO interpreted as a Programmable Power Supply - Source. */

    /** @brief Programmable Power Supply Sink PDO. */
    struct ROM_PPS_SNK
    {
        uint32_t op_cur                     : 7;    /**< Operating current in 50 mA units. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t min_volt                   : 8;    /**< Minimum voltage in 100 mV units. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t max_volt                   : 8;    /**< Maximum voltage in 100 mV units. */
        uint32_t rsvd3                      : 1;    /**< Reserved field. */
        uint32_t cur_fb                     : 1;    /**< Whether current foldback is required. */
        uint32_t rsvd4                      : 1;    /**< Reserved field. */
        uint32_t apdo_type                  : 2;    /**< APDO type: Should be 0 for PPS. */
        uint32_t supply_type                : 2;    /**< PDO type: Should be 3 for PPS APDO. */
    } pps_snk;                                      /**< DO interpreted as a Programmable Power Supply - Sink. */

    /** @brief Programmable Request Data Object. */
    struct ROM_RDO_PPS
    {
        uint32_t op_cur                     : 7;    /**< Operating current in 50 mA units. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t out_volt                   : 11;   /**< Requested output voltage in 20 mV units. */
        uint32_t rsvd2                      : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Whether unchunked extended messages are supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend flag. */
        uint32_t usb_comm_cap               : 1;    /**< Whether sink supports USB communication. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch flag. */
        uint32_t rsvd3                      : 1;    /**< Reserved field. */
#if CCG_PD_REV3_ENABLE
        uint32_t obj_pos                    : 4;    /**< Object position. */
#else
        uint32_t obj_pos                    : 3;    /**< Object position - index to source PDO. */
        uint32_t rsvd4                      : 1;    /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
    } rdo_pps;                                      /**< DO interpreted as a PPD Request. */

    /** @brief PD 3.0 Alert Data Object. */
    struct ROM_ADO_ALERT
    {
        uint32_t rsvd1                      :16;    /**< Reserved field. */
        uint32_t hot_swap_bats              :4;     /**< Identifies hot-swappable batteries whose status has changed. */
        uint32_t fixed_bats                 :4;     /**< Identifies fixed batteries whose status has changed. */
        uint32_t rsvd2                      :1;     /**< Reserved field. */
        uint32_t bat_status_change          :1;     /**< Battery status changed. */
        uint32_t ocp                        :1;     /**< Over-Current event status. */
        uint32_t otp                        :1;     /**< Over-Temperature event status. */
        uint32_t op_cond_change             :1;     /**< Operating conditions changed. */
        uint32_t src_input_change           :1;     /**< Power source input changed. */
        uint32_t ovp                        :1;     /**< Over-Voltage event status. */
    } ado_alert;                                    /**< DO interpreted as a PD 3.0 alert message. */

    /** @brief Thunderbolt UFP Discover Modes Response Data Object. */
    struct ROM_TBT_UFP_VDO
    {
        uint32_t intel_mode                 : 16;   /**< Thunderbolt (Intel) modes identifier. */
        uint32_t adapter                    : 1;    /**< Legacy TBT Adapter or Device. */
        uint32_t rsvd0                      : 9;    /**< Reserved field. */
        uint32_t vpro_supp                  : 1;    /**< Whether supports vPro mode. */
        uint32_t rsvd1                      : 5;    /**< Reserved field. */
    } tbt_ufp_vdo;                                  /**< Data Object interpreted as a Thunderbolt3 mode VDO. */

#if (!CCG_USB4_SUPPORT_ENABLE)
    /** @brief Thunderbolt Discover Modes Response Data Object. */
    struct ROM_TBT_VDO
    {
        uint32_t intel_mode                 : 16;   /**< Thunderbolt (Intel) modes identifier. */
        uint32_t cbl_speed                  : 3;    /**< Data bandwidth supported by the Type-C cable. */
        uint32_t cbl_gen                    : 2;    /**< Thunderbolt cable generation. */
        uint32_t cbl_type                   : 1;    /**< Whether cable is non-optical or optical. */
        uint32_t cbl                        : 1;    /**< Type of Type-C cable: Passive or Active. */
        uint32_t link_training              : 1;    /**< Type of link training supported by active cable. */
        uint32_t leg_adpt                   : 1;    /**< Whether this is a legacy Thunderbolt adapter. */
        uint32_t rsvd0                      : 1;    /**< Reserved field. */
        uint32_t vpro_dock_host             : 1;    /**< Whether the device supports VPRO feature. */
        uint32_t rsvd1                      : 5;    /**< Reserved field. */
    } tbt_vdo;                                      /**< DO interpreted as a Thunderbolt Discovery response. */    
#else
    /** @brief Thunderbolt Discover Modes Response Data Object. */
    struct ROM_TBT_VDO
    {
        uint32_t intel_mode                 : 16;   /**< Thunderbolt (Intel) modes identifier. */
        uint32_t cbl_speed                  : 3;    /**< Data bandwidth supported by the Type-C cable. */
        uint32_t cbl_gen                    : 2;    /**< Thunderbolt cable generation. */
        uint32_t cbl_type                   : 1;    /**< Whether cable is non-optical or optical. */
        uint32_t b22_retimer_cbl            : 1;    /**< Type of Type-C cable: Redriver or Retimer. */
        uint32_t link_training              : 1;    /**< Type of link training supported by active cable. */
        uint32_t adapter                    : 1;    /**< Legacy TBT Adapter or Device. */
        uint32_t cable_active               : 1;    /**< Whether cable reports active or passive in ID HDR in discover id response. */
        uint32_t vpro_dock_host             : 1;    /**< Whether the device supports VPRO feature. */
        uint32_t rsvd1                      : 3;    /**< Reserved field. */
        uint32_t rsvd2                      : 2;    /**< Reserved field. */
    } tbt_vdo;                                      /**< DO interpreted as a Thunderbolt Discovery response. */
    
    /** @brief Thunderbolt Discover Modes Response Data Object. */
    struct TBT_CBL_VDO
    {
        uint32_t intel_mode                 : 16;   /**< Thunderbolt (Intel) modes identifier. */
        uint32_t cbl_speed                  : 3;    /**< Data bandwidth supported by the Type-C cable. */
        uint32_t cbl_gen                    : 2;    /**< Thunderbolt cable generation. */
        uint32_t cbl_type                   : 1;    /**< Whether cable is non-optical or optical. */
        uint32_t b22_retimer_cbl            : 1;    /**< Type of Type-C cable: Redriver or Retimer. */
        uint32_t link_training              : 1;    /**< Type of link training supported by active cable. */
        uint32_t rsvd1                      : 8;    /**< Reserved field. */
    } tbt_cbl_vdo;                                  /**< DO interpreted as a Thunderbolt Discovery response. */ 
#endif

    /** @brief UFP VDO #1 */
    struct ROM_UFP_VDO_1
    {
        uint32_t usb_sig                    : 3;    /**< USB signaling supported. */
        uint32_t alt_modes                  : 3;    /**< Alt. modes supported bit-map. */
#if CCG_PD_REV3_ENABLE
        uint32_t rsvd0                      : 16;   /**< Reserved field. */
        uint32_t conn_type                  : 2;    /**< Connector type. */
#else
        uint32_t rsvd0                      : 18;   /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
        uint32_t dev_cap                    : 4;    /**< Device Capability. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version field. */
    } ufp_vdo_1;                                    /**< DO interpreted as UFP VDO1 data object. */

    /** @brief DFP VDO */
    struct ROM_DFP_VDO
    {
        uint32_t port_numb                  : 5;    /**< Unique port number to identify a specific port on a multi-port device. */
#if CCG_PD_REV3_ENABLE
        uint32_t rsvd0                      : 17;   /**< Reserved field. */
        uint32_t conn_type                  : 2;    /**< Connector type. */
#else
        uint32_t rsvd0                      : 19;   /**< Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
        uint32_t host_cap                   : 3;    /**< Host capability. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version field. */
    } dfp_vdo;                                  /**< DO interpreted as UFP VDO1 data object. */

    /** @brief Enter USB Data Object. */
    struct ROM_ENTERUSB_VDO
    {
        uint32_t rsvd0                      : 13;   /**< Reserved field. */
        uint32_t host_present               : 1;    /**< Whether a host is connected. */
        uint32_t host_tbt_supp              : 1;    /**< Whether host supports Thunderbolt. */
        uint32_t host_dp_supp               : 1;    /**< Whether host supports DisplayPort. */
        uint32_t host_pcie_supp             : 1;    /**< Whether host supports PCIe. */
        uint32_t cable_current              : 2;    /**< Current carrying capacity of the cable. */
        uint32_t cable_type                 : 2;    /**< Type of cable. */
        uint32_t cable_speed                : 3;    /**< Data rate supported by the cable. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t usb3_drd                   : 1;    /**< Whether the DFP is USB 3.2 DRD capable. */
        uint32_t usb4_drd                   : 1;    /**< Whether the DFP is USB4 DRD capable. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t usb_mode                   : 3;    /**< Mode of USB communication (2.0, 3.2 or 4.0) */
        uint32_t rsvd3                      : 1;    /**< Reserved field. */
    } enterusb_vdo;                                 /**< DO interpreted as an Enter USB Data Object. */

    /** @cond DOXYGEN_HIDE */
    struct ROM_SLICE_VDO
    {
        uint32_t slice_mode                 : 16;
        uint32_t module_type                : 2;
        uint32_t rsvd                       : 14;
    } slice_vdo;

    struct ROM_SLICE_SUBHDR
    {
        uint32_t am_addr                    : 20;
        uint32_t vdo_cnt                    : 3;
        uint32_t multi_part                 : 1;
        uint32_t data_cnt                   : 8;
    } slice_subhdr;
    /** @endcond */

} pd_do_t;

/**
 * @brief PD port status corresponding to the Status Data Block (SSDB)
 * See Table 6-39 of USB-PD R3 specification.
 */
typedef struct
{
    uint8_t  intl_temperature;                  /**< Byte 0: Port's internal temperature. 0 if not supported. */
    uint8_t  present_input;                     /**< Byte 1: Reports current input power status. */
    uint8_t  battery_input;                     /**< Byte 2: Reports the current battery status. */
    uint8_t  event_flags;                       /**< Byte 3: Event flags. */
    uint8_t  temp_status;                       /**< Byte 4: Temperature status. */
    uint8_t  power_status;                      /**< Byte 5: Power status. */
    uint8_t  pwr_state_chg;                     /**< Byte 6: Power state change. */
    uint8_t  dummy;                             /**< Byte 7: Reserved field used for 4 byte alignment. */
} pd_power_status_t;


/**
 * @brief Struct to hold response to policy manager.
 * Note: The srom_app_resp_t structure is only used for responses that have a single DO. This may need
 * to get extended if more DOs are to be supported.
 */
typedef struct
{
    pd_do_t             resp_do;                /**< Response data object. */
    app_req_status_t    req_status;             /**< Request status. */
} srom_app_resp_t;

/**
 * @brief Struct to hold response to policy manager.
 */
typedef struct
{
    pd_do_t     resp_buf[CY_PD_MAX_NO_OF_DO]; /**< Data objects buffer */
    uint8_t     do_count;               /**< Data objects count */
    vdm_ams_t   no_resp;                /**< Response type. */
} rom_vdm_resp_t;

/**
 * @brief Struct to hold PD command buffer.
 * @warning When providing pointer to the extended data make sure original buffer
 * is always 4-byte aligned. i.e, even if 1 byte data is required, 4 bytes should be used to
 * store that data.
 */
typedef struct
{
    sop_t               cmd_sop;                /**< SOP type */
    extd_msg_t          extd_type;              /**< Extended Message Type */
    pd_extd_hdr_t       extd_hdr;               /**< Extended Header */
    uint8_t             no_of_cmd_do;           /**< No of data objects including VDM header */
    uint8_t*            dat_ptr;                /**< Data Pointer in case of extended message only*/
    uint8_t             timeout;                /**< Timeout value, in ms, for a response.
                                                 *   If set to zero, the PD stack will not wait for a VDM response
                                                 *   and jump to ready state after this buffer has been sent.
                                                 */
    pd_do_t             cmd_do[CY_PD_MAX_NO_OF_DO];   /**< Command data objects. */
} dpm_pd_cmd_buf_t;

/**
 * @brief Structure to hold PD Contract information.
 */
typedef struct
{
    uint16_t cur_pwr;           /**< PD contract current/power. */
    uint16_t max_volt;          /**< PD contract max voltage in mV */
    uint16_t min_volt;          /**< PD contract min voltage in mV */
} contract_t;

/**
 * @brief Stucture to hold PD Contract information passed with APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE
 * event to the application.
 */
typedef struct
{
    pd_do_t              rdo;           /**< RDO associated with the contract. */
    pd_contract_status_t status;        /**< Status of the contract. */
} pd_contract_info_t;

/**
 * @brief Struct to hold a PD packet.
 */
typedef struct
{
    sop_t       sop;                    /**< Packet type. */
    uint8_t     len;                    /**< Length in data objects. */
    uint8_t     msg;                    /**< Message code. */
    port_type_t     data_role;              /**< Data role. */
    pd_hdr_t    hdr;                    /**< Message header. */
    pd_do_t     dat[CY_PD_MAX_NO_OF_DO];      /**< Data objects associated with the message. */
} pd_packet_t;

/**
 * @brief Struct to hold extended PD packets (messages).
 */
typedef struct
{
    sop_t     sop;                    /**< Packet type. */
    uint8_t     len;                    /**< Length of the message: Unused for unchunked extended messages. */
    uint8_t     msg;                    /**< Message code. */
    port_type_t     data_role;              /**< Data role. */
    pd_hdr_t    hdr;                    /**< Message header, including extended header. */
    pd_do_t     dat[CY_PD_MAX_EXTD_PKT_WORDS];/**< Data associated with the message. */
} pd_packet_extd_t;


/**
 * @brief PD callback prototype.
 * This is a stack internal callback function used by the USB-PD Protocol
 * layer to send events to the Policy Engine. The events notified correspond
 * to Policy Engine events such as HARD RESET or SOFT RESET received.
 *
 * @param port PD port index.
 * @param Type of event being notified.
 */
typedef void (*pd_cbk_t)(uint8_t port, uint32_t event);

/**
 * @brief DPM PD command callback. This is the type of callback function used by the Policy Engine
 * to report results of a command to the application layer.
 *
 * @param port PD port index.
 * @param resp Response code.
 * @param pkt_ptr Pointer to any PD packet associated with the response.
 */
typedef void (*dpm_pd_cmd_cbk_t)(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr);

/**
 * @brief Application response callback. This is the type of callback used by the stack to
 * receive application level responses associated with a PD message such as a SWAP request.
 *
 * @param port PD port index.
 * @param resp Pointer to the structure holding response information.
 */
typedef void (*app_resp_cbk_t)(uint8_t port, srom_app_resp_t* resp);

/**
 * @brief VDM response callback. This is the type of callback used by the stack to receive
 * application level responses to a VDM received from the port partner or cable marker.
 *
 * @param port PD port index.
 * @param resp Pointer to structure holding response information.
 */
typedef void (*vdm_resp_cbk_t)(uint8_t port, rom_vdm_resp_t* resp);

/**
 * @brief Power ready callback. Type of callback used by the stack to receive notification
 * from the power source/sink hardware manager that the requested power transition has
 * been completed.
 *
 * @param port PD port index.
 */
typedef void (*pwr_ready_cbk_t)(uint8_t port);

/**
 * @brief Sink discharge off callback. Callback type used by the stack to receive
 * notification that the sink discharge circuit has been turned off.
 *
 * @param port PD port index.
 */
typedef void (*sink_discharge_off_cbk_t)(uint8_t port);

/**
 * @brief Type C command response callback. Type of callback used by the stack to
 * report results of a dpm_typec_command API call to the application layer.
 *
 * @param port PD port index.
 * @param resp Response code.
 */
typedef void (*dpm_typec_cmd_cbk_t)(uint8_t port, dpm_typec_cmd_resp_t resp);

/**
 * @brief Struct to hold the application interface. The application is expected to
 * fill the structure with pointers to functions that use the on-board circuitry to
 * accomplish tasks like source/sink power turn on/off. All the functions in this
 * structure must be non-blocking and take minimum execution time.
 *
 * @warning The application must check the callback pointer passed by the
 * stack is not NULL.
 */
typedef struct
{
    void (*app_event_handler) (
            uint8_t port,               /**< PD port index. */
            app_evt_t evt,              /**< Type of event. */
            const void* dat             /**< Event related data. */
            );                          /**< App event handler callback. */

#if (!CCG_SINK_ONLY)
    void (*psrc_set_voltage) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt_mV            /**< Target voltage in mV units. */
            );                          /**< Set power source voltage in mV units. */

    void (*psrc_set_current) (
            uint8_t port,               /**< PD port index. */
            uint16_t cur_10mA           /**< Expected operating current in 10 mA units. */
            );                          /**< Set power source current in 10mA units. */

    void (*psrc_enable) (
            uint8_t port,               /**< PD port index. */
            pwr_ready_cbk_t pwr_ready_handler   /**< Function to be called after power enable. */
            );                          /**< Enable the power supply. The pwr_ready_handler, if not NULL, must be
                                         *   called when the power supply is ready.
                                         */

    void (*psrc_disable) (
            uint8_t port,               /**< PD port index. */
            pwr_ready_cbk_t pwr_ready_handler   /**< Function to be called after power disable. */
            );                          /**< Disable the power supply. The pwr_ready_handler, if not NULL,
                                         *   must be called when the power supply has been discharged to Vsafe0V. */
#endif /* (!CCG_SINK_ONLY) */

    bool (*vconn_enable) (
            uint8_t port,               /**< PD port index. */
            uint8_t channel             /**< CC channel on which to enable VConn. */
            );                          /**< Turn VCONN supply ON. Return true if VCONN was turned ON. */

    void (*vconn_disable) (
            uint8_t port,               /**< PD port index. */
            uint8_t channel             /**< CC channel on which to disable VConn. */
            );                          /**< Turn VCONN supply OFF. */

    bool (*vconn_is_present) (
            uint8_t port                /**< PD port index. */
            );                          /**< Check whether VConn supply is ON. */

    bool (*vbus_is_present) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt,              /**< Expected voltage in mV units. */
            int8_t per                  /**< Allowed voltage deviation in percentage units. */
            );                          /**< Check whether VBus voltage is within expected range. */

    void (*vbus_discharge_on) (
            uint8_t port                /**< PD port index. */
            );                          /**< Turn on VBUS discharge circuit. */

    void (*vbus_discharge_off) (
            uint8_t port                /**< PD port index. */
            );                          /**< Turn off VBUS discharge circuit. */

#if (!(CCG_SOURCE_ONLY))
    void (*psnk_set_voltage) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt_mV            /**< Target voltage in mV units. */
            );                          /**< Set power sink voltage, in mV units. */

    void (*psnk_set_current) (
            uint8_t port,               /**< PD port index. */
            uint16_t cur_10mA           /**< Operating current in 10 mA units. */
            );                          /**< Set power sink current, in 10mA units. */

    void (*psnk_enable) (
            uint8_t port                /**< PD port index. */
            );                          /**< Enable power sink related circuitry. */

    void (*psnk_disable) (
            uint8_t port,               /**< PD port index. */
            sink_discharge_off_cbk_t snk_discharge_off_handler  /**< Callback to be called after discharge is done. */
            );                          /**< Disable power sink related circuitry. */

#if (!(SROM_CODE_PD_PD))
    void (*eval_src_cap) (
            uint8_t port,               /**< PD port index. */
            const pd_packet_t* src_cap, /**< Pointer to list of received source caps. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Evaluate received source caps and provide the RDO to be used
                                             to negotiate contract. */
#endif /* (!(SROM_CODE_PD_PD)) */
#endif /* (!(CCG_SOURCE_ONLY)) */

#if (!CCG_SINK_ONLY)
#if (!(SROM_CODE_PD_PD))
    void (*eval_rdo)(
            uint8_t port,               /**< PD port index. */
            pd_do_t rdo,                /**< Received RDO object. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Evaluate sink request message. */
#endif /* (!(SROM_CODE_PD_PD)) */
#endif /* (!CCG_SINK_ONLY) */

    void (*eval_dr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles DR swap request received by port. */
    void (*eval_pr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles pr swap request received by port. */

    void (*eval_vconn_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles VCONN swap request received by port. */

    void (*eval_vdm) (
            uint8_t port,               /**< PD port index. */
            const pd_packet_t *vdm,     /**< Pointer to received VDM. */
            vdm_resp_cbk_t vdm_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handle VDMs (all structured/unstructured VDMs need to be handled) received
                                             by the port. */
#if ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))
    void (*eval_fr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handle FR swap request received by the specified port. */
#endif /* ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))  */

    uint16_t (*vbus_get_value) (
            uint8_t port                /**< PD port index. */
            );                          /**< Get current VBUS value in mV from application. */

#if (!(SROM_CODE_PD_PD))
#if (!CCG_SINK_ONLY)
    uint32_t (*psrc_get_voltage) (
            uint8_t port                /**< PD port index. */
            );                          /**< Get expected VBUS value in mV from application. This is to include any
                                              additional compensation done for drops. */
#endif /* (!CCG_SINK_ONLY) */
#else
    void (*update_rdo) (
            uint8_t port,               /**< PD port index. */
            const pd_packet_t* src_cap, /**< Pointer to list of received source caps. */
            srom_app_resp_t *app_resp        /**< Return parameter to report response through. */
            );                          /**< Update RDO prepared by default eval_src_cap function. */
#endif /* (!(SROM_CODE_PD_PD)) */

#if (CCG_USB4_SUPPORT_ENABLE)
    void (*eval_enter_usb) (
            uint8_t port,                       /**< PD port index. */
            const pd_packet_t* eudo,            /**< Pointer to received Enter USB Data Object. */
            app_resp_cbk_t app_resp_handler     /**< Callback to report response through. */
            );                                  /**< Function to evaluate Enter USB request. */
#endif /* (CCG_USB4_SUPPORT_ENABLE) */

} app_cbk_t;
/** \} group_ccgxAppCommon_data_structures */
/**
 * @brief PD Device Policy Status structure. This structure holds all of the configuration and status
 * information associated with a port on the CCG device. Members of this structure should not be
 * directly modified by any of the application code.
 *
 * @warning Initial elements of this structure maps directly to config table
 * fields and hence must not be moved around or changed.
 */
typedef struct
{
    port_role_t port_role;          /**< Port role: Sink, Source or Dual. */
    port_role_t dflt_port_role;     /**< Default port role: Sink or Source. */
    uint8_t src_cur_level;      /**< Type C current level in the source role. */
    uint8_t is_src_bat;         /**< Power source is connected to a battery. */
    uint8_t is_snk_bat;         /**< Power sink is connected to a battery. */
    uint8_t snk_usb_susp_en;    /**< USB suspend supported indication. */
    uint8_t snk_usb_comm_en;    /**< USB communication supported indication. */
    uint8_t swap_response;      /**< Response to be sent for each USB-PD SWAP command.
                                     - Bits 1:0 => DR_SWAP response.
                                     - Bits 3:2 => PR_SWAP response.
                                     - Bits 5:4 => VCONN_SWAP response.
                                     Allowed values are: 0=ACCEPT, 1=REJECT, 2=WAIT, 3=NOT_SUPPORTED.
                                 */

    uint8_t toggle;             /**< DRP toggle is enabled. */
    uint8_t src_pdo_count;      /**< Source PDO count from the config table or updated at runtime by the EC. */
    uint8_t src_pdo_mask;       /**< Source PDO mask from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_count;      /**< Sink PDO count from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_mask;       /**< Sink PDO mask from the config table or updated at runtime by the EC. */
    uint8_t rp_supported;       /**< Supported Rp values bit mask.
                                     - Bit 0 => Default Rp supported.
                                     - Bit 1 => 1.5 A Rp supported.
                                     - Bit 2 => 3 A Rp supported.
                                 */

    bool pd_support;         /**< USB-PD supported. */
    uint8_t try_src_snk;        /**< Try Source/ Try Sink control knob. */
    bool cbl_dsc;            /**< Cable discovery control knob. */
    uint8_t db_support;         /**< Dead battery support control knob. */
    bool err_recov;          /**< Error recovery control knob.*/
    uint8_t port_disable;       /**< PD port disable flag. */
    uint8_t frs_enable;         /**< FRS enable flags. */
    uint8_t vconn_retain;       /**< Whether VConn should be retained in ON state. */

    uint16_t reserved_3[5];     /**< Reserved words for padding to 4-byte aligned address. */

    pd_do_t src_pdo[CY_PD_MAX_NO_OF_PDO];     /**< Source PDO loaded from the config table or updated at runtime by the EC. */
    pd_do_t snk_pdo[CY_PD_MAX_NO_OF_PDO];     /**< Sink PDO loaded from the config table or updated at runtime by the EC. */
    uint16_t snk_max_min[CY_PD_MAX_NO_OF_PDO];/**< Max min current from the config table or updated at runtime by the EC. */

    port_type_t cur_port_type;  /**< Current port type: UFP or DFP. */
    port_role_t cur_port_role;  /**< Current Port role: Sink or Source. */
    uint8_t volatile snk_cur_level;     /**< Type C current level in sink role. */
    uint8_t volatile bootup;    /**< Flag to indicate chip bootup, used to check dead battery. */
    uint8_t volatile dead_bat;  /**< Flag to indicate dead battery operation. */
    uint8_t drp_period;         /**< Time period for DRP toggling. */
    uint8_t src_period;         /**< Time period for which to stay as a SRC for a DRP device. */
    uint8_t snk_period;         /**< Time period for which to stay as a SNK for a DRP device. */
    uint8_t volatile skip_scan; /**< Skip CC scanning control knob. */

    uint8_t polarity;                   /**< CC channel polarity (CC1=0, CC2=1) */
    uint8_t rev_pol;                    /**< CC channel reverse polarity. */
    bool volatile connect;           /**< Port connected but not debounced yet. */
    bool volatile attach;               /**< Port attached indication. */
    port_role_t role_at_connect;        /**< Port role when the port moved to the attached state. */
    pd_devtype_t attached_dev;          /**< Type of device attached. */
    bool volatile contract_exist;    /**< PD contract exists indication. */
    bool volatile pd_connected;         /**< Ports are PD connected indication. */
    bool volatile pd_disabled;       /**< PD disabled indication. */
    bool volatile ra_present;        /**< Ra present indication. */
    bool volatile emca_present;         /**< EMCA cable present indication. */
    uint8_t volatile bist_cm2_enabled;  /**< BIST Carrier Mode 2 going on */
    bool volatile bist_stm_enabled;     /**< BIST_Shared_Test_Mode enabled flag. */
    std_vdm_prod_t cbl_type;            /**< Stores the cable type. */
    std_vdm_ver_t cbl_vdm_version;      /**< Stores the cable VDM version. */
    bool volatile vconn_logical;     /**< VCONN logical status. */
    uint8_t cur_src_pdo_count;          /**< Source PDO count in the last sent source cap. */
    uint8_t cur_snk_pdo_count;          /**< Sink PDO count in the last sent sink cap. */
    bool cbl_wait;                   /**< Flag to indicate cable discovery is waiting for some event. */
    pe_cbl_state_t cbl_state;           /**< Cable discovery state machine state. */
    uint8_t cbl_soft_reset_tried;       /**< Stores number of cable soft reset tries. */

    typec_fsm_state_t typec_fsm_state;  /**< Type C generic FSM state. */
    dpm_pd_cmd_t dpm_pd_cmd;            /**< Current DPM PD command. */
    bool dpm_pd_cmd_active;          /**< Indicate DPM PD command was registered. */
    bool dpm_typec_cmd_active;       /**< Indicate DPM Type C command was registered. */
    bool dpm_enabled;                   /**< DPM enabled flag. */
    bool dpm_init;                      /**< DPM Initialized flag. */

    bool dpm_safe_disable;           /**< DPM sage disable flag. Used to make sure that the port is disabled
                                             correctly after a fault occurrence. */

    
    dpm_typec_cmd_t dpm_typec_cmd;      /**< Current DPM Type C command. */
    uint8_t src_cur_level_live;         /**< Current Type C current level in the source role. */
    cc_state_t cc_live;                 /**< Live CC status. */
    cc_state_t cc_status;               /**< Current debounced CC status. */
    cc_state_t cc_old_status;           /**< Old CC status. */
    cc_state_t cc_rd_status;            /**< Rd status. */
    pd_rev_t spec_rev_sop_live;         /**< Current spec rev for SOP messages. */

    pd_rev_t spec_rev_sop_prime_live;   /**< Current spec revision for SOP Prime/DPrime messages. */
    pd_rev_t spec_rev_cbl;              /**< Spec revision of the currently connected cable. */
    pd_rev_t spec_rev_peer;             /**< Spec revision of the currently connected peer. */
    bool unchunk_sup_live;              /**< Mutual unchunk support with the currently connected peer. */
    bool unchunk_sup_peer;              /**< Unchunk support of the currently connected peer. */
    bool snk_rp_detach_en;              /**< Flag to indicate sink will detach on Rp removal instead of VBUS removal. */
    bool cur_fb;                        /**< Flag to indicate current foldback is active */
    pd_ams_type non_intr_response;      /**< Flag to indicate stack is waiting for App response to a
                                            non interruptible AMS */

    
    bool fr_rx_disabled;                /**< FRS receive disabled by EC. */
    bool fr_rx_en_live;                 /**< Fast role signal detection enabled */
    bool fr_tx_disabled;                /**< FRS transmit disabled by EC. */
    bool fr_tx_en_live;                 /**< Fast role signal auto send enabled */
    volatile bool fault_active;         /**< Flag to indicate the a fault consition exists. */
    pe_fsm_state_t pe_fsm_state;        /**< Holds the current state of Policy Engine (PE). */
    uint32_t volatile pe_evt;           /**< Stores policy engine events. */
    uint32_t volatile typec_evt;        /**< Stores Type C events. */
    contract_t contract;                /**< Current pd contract. */
    pd_do_t alert;                      /**< Alert status */

    pd_do_t cbl_vdo;                    /**< Stores the last received cable VDO. */
    bool cbl_mode_en;                   /**< Whether cable supports alternate modes. */

    uint16_t src_cap_start_delay;       /**< Place holder for src cap start delay in milliseconds */

    app_cbk_t* app_cbk;                 /**< Application callback pointer. */
    dpm_pd_cmd_cbk_t dpm_pd_cbk;        /**< Pointer to DPM PD callback function. */
    dpm_typec_cmd_cbk_t dpm_typec_cbk;  /**< Pointer to DPM Type C callback function. */
    dpm_pd_cmd_buf_t* cmd_p;            /**< Pointer to DPM command buffer. */
    dpm_pd_cmd_buf_t dpm_cmd_buf;       /**< Local DPM command buffer. */

    uint16_t cur_snk_max_min[CY_PD_MAX_NO_OF_PDO];    /**< Max min current/power of current sink capabilities. */
    pd_do_t cur_src_pdo[CY_PD_MAX_NO_OF_PDO];         /**< Current source PDOs sent in source cap messages. */
    pd_do_t cur_snk_pdo[CY_PD_MAX_NO_OF_PDO];         /**< Current sink PDOs sent in sink cap messages. */
    pd_do_t src_cur_rdo;                        /**< Stores the current rdo received by source */
    pd_do_t src_last_rdo;                       /**< Stores the last rdo received by source */
    pd_do_t src_rdo;                            /**< Last RDO received (when operating as a source) that resulted in
                                                     a contract. */
    pd_do_t snk_rdo;                            /**< Last RDO sent (when operating as a sink) that resulted in
                                                     a contract. */
    pd_do_t snk_sel_pdo;                        /**< Selected PDO which resulted in contract (when sink). */
    pd_do_t src_sel_pdo;                        /**< Selected PDO which resulted in contract (when source). */

    pd_packet_t *src_cap_p;                     /**< Pointer to the current/last source cap message received.
                                                     May be NULL. Data pointed to by this should not be changed. */

    uint32_t padval;                            /**< Fields below need to be properly aligned to 4 byte boundary */
    pd_power_status_t port_status;              /**< Port power status. */
    uint8_t ext_src_cap[CY_PD_EXT_SRCCAP_BUF_SIZE];    /**< Buffer to hold extended source caps. */
    uint8_t pps_status[CY_PD_EXT_SRCCAP_BUF_SIZE];     /**< Buffer to hold PPS status. */
    uint8_t ext_src_cap_size;                           /**< Size of the Extended Source Cap response. */
    uint8_t dpm_err_info;                       /**< Additional error status for DPM commands. */
    bool    pwr_limited;                        /**< Whether SRC PDOs have been limited due to cable capability. */
    pd_do_t cbl_vdo_2;                          /**< Stores the last received Active Cable VDO #2. */
    uint8_t ext_snk_cap[CY_PD_EXT_SRCCAP_BUF_SIZE];    /**< Buffer to hold extended sink caps. */
    uint32_t rand_base;                         /**< Temporary variable used for random number generation. */
    bool    pd3_src_cc_busy;                    /**< Whether we should keep Rp at SinkTxNG as a PD 3.0 Source. */

#if DPM_DEBUG_SUPPORT
    /* Parameters added for Thunderbolt dock support. */
    pd_do_t active_rdo;                                 /**< RDO that has been accepted last. */
    uint8_t contract_flags;                             /**< Stores the contract flags. */
    uint8_t src_pdo_flags;                              /**< Stores the source PDO flags */
    uint8_t err_recov_reason;                           /**< Reason for port entering Type-C error recovery. */
    uint8_t sopdp_soft_reset_reason;                    /**< Reason for port issuing SOP'' SoftReset. */
    uint8_t sopp_soft_reset_reason;                     /**< Reason for port issuing SOP' SoftReset. */
    uint8_t cable_reset_reason;                         /**< Reason for port issuing Cable Reset. */
    uint8_t hard_reset_reason;                          /**< Reason for port issuing Hard Reset. */
    uint8_t soft_reset_reason;                          /**< Reason for port issuing Soft Reset. */
    uint8_t sopdp_present;                              /**< Whether SOP'' cable controller is present. */
    uint8_t cc_stat[2];                                 /**< Current state of the CC1 and CC2 pins. */

    /* Debug counters  */
    uint8_t  connection_count;                          /**< Number of connections since power-up. */
    uint8_t  fault_count;                               /**< Number of faults in current connection. */
    uint8_t  contr_negotiation_count;                   /**< Number of contracts made during current connection. */
    uint16_t pd_msgs_sent;                              /**< Number of messages sent during current connection. */
    uint16_t pd_msgs_rxd;                               /**< Number of messages received during current connection. */
#endif /* DPM_DEBUG_SUPPORT */
    
    uint8_t rev3_en;                                    /**< Flag indicating that PD Revision 3.0 support is enabled. */
    uint8_t hw_drp_toggle_en;                           /**< Flag indicating that Hardware based DRP toggling is enabled. */
    uint8_t try_src_snk_dis;                            /**< Flag indicating that Try.SRC Try.SNK is disabled. */
    uint8_t frs_rx_en;                                  /**< Flag indicating that FRS as Initial Sink is supported. */
    uint8_t frs_tx_en;                                  /**< Flag indicating that FRS as Initial Source is supported. */
    uint8_t pps_src_en;                                 /**< Flag indicating that PPS source is supported. */
    uint8_t usb4_en;                                    /**< Flag indicating that USB4 messages are supported. */
    uint8_t pps_snk_en;                                 /**< Flag indicating that PPS SINK RDO from EC is supported. */
    pd_do_t pps_snk_rdo;                                /**< PPS SINK RDO from EC */

#if(BCR)
    uint8_t bcr_fault_status;                           /**< Fault status for BCR fault pin. */
#endif /* BCR */
    uint16_t dpm_dis_req_cnt;                           /**< Number of dpm disable requests active. */
    bool auto_vcs_disable;                              /**< Option to disable Automatic VConn Swap from the Stack. */
} dpm_status_t;

/** @endcond */
#endif /* PD_H_ */

/* End of file */
