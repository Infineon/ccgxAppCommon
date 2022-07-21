/***************************************************************************//**
* \file alt_mode_mngr.h
* \version 1.1.0 
*
* Alternate Mode Manager header file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef ALT_MODES_MNGR_H_
#define ALT_MODES_MNGR_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <cy_pdstack_common.h>
#include <config.h>
#include <vdm_task_mngr.h>
#include <cy_usbpd_defines.h>
/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

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

#define MAX_SUPP_ALT_MODES              (4u)
/**< Maximum number of alternate modes which alt modes manager could operates in the same time.
     Setting this to larger values will increase RAM requirement for the projects. */

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

#define ALT_MODE_EVT_SIZE               (2u)
/**< Size of alt mode APP event data in 4 byte words. */

#define ALT_MODE_EVT_IDX                (0u)
/**< Index of alt mode APP event code. */

#define ALT_MODE_EVT_DATA_IDX           (1u)
/**< Index of data for alt mode APP event. */

#define NO_DATA                         (0u)
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

#define VDM_HDR                          vdm_header.std_vdm_hdr
/**< Quick access to Standard VDM header fields macros. */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef alt_mode_mngr_state_t
  @brief This enumeration holds all possible Alt modes manager states when DFP.
 */
typedef enum
{
    ALT_MODE_MNGR_STATE_DISC_MODE = 0,  /**< Alt modes manager discovery mode state. */
    ALT_MODE_MNGR_STATE_PROCESS         /**< Alt modes manager alternate modes processing state. */
}alt_mode_mngr_state_t;

/**
  @typedef alt_mode_state_t
  @brief This enumeration holds all possible states of each alt mode which is handled by Alt modes manager.
 */
typedef enum
{
    ALT_MODE_STATE_DISABLE = 0,         /**< State when alternate mode functionality is disabled. */
    ALT_MODE_STATE_IDLE,                /**< State when alternate mode is idle. */
    ALT_MODE_STATE_INIT,                /**< State when alternate mode initiate its functionality. */
    ALT_MODE_STATE_SEND,                /**< State when alternate mode needs to send VDM message. */
    ALT_MODE_STATE_WAIT_FOR_RESP,       /**< State while alternate mode wait for VDM response. */
    ALT_MODE_STATE_PAUSE,               /**< State while alternate mode is on pause and waiting for some trigger. */
    ALT_MODE_STATE_FAIL,                /**< State when alternate mode VDM response fails. */
    ALT_MODE_STATE_RUN,                 /**< State when alternate mode need to be running. */
    ALT_MODE_STATE_EXIT                 /**< State when alternate mode exits and resets all related variables. */
}alt_mode_state_t;

/**
  @typedef alt_mode_app_evt_t
  @brief This enumeration holds all possible application events related to Alt modes handling.
 */
typedef enum
{
    AM_NO_EVT = 0,                      /**< Empty event. */
    AM_EVT_SVID_NOT_FOUND,              /**< Sends to EC if UFP doesn't support any SVID. */
    AM_EVT_ALT_MODE_ENTERED,            /**< Alternate mode entered. */
    AM_EVT_ALT_MODE_EXITED,             /**< Alternate mode exited. */
    AM_EVT_DISC_FINISHED,               /**< Discovery process was finished. */
    AM_EVT_SVID_NOT_SUPP,               /**< CCGx doesn't support received SVID. */
    AM_EVT_SVID_SUPP,                   /**< CCGx supports received SVID. */
    AM_EVT_ALT_MODE_SUPP,               /**< CCGx supports alternate mode. */
    AM_EVT_SOP_RESP_FAILED,             /**< UFP VDM response failed. */
    AM_EVT_CBL_RESP_FAILED,             /**< Cable response failed. */
    AM_EVT_CBL_NOT_SUPP_ALT_MODE,       /**< Cable capabilities couldn't provide alternate mode hanling. */
    AM_EVT_NOT_SUPP_PARTNER_CAP,        /**< CCGx and UFP capabilities not consistent. */
    AM_EVT_DATA_EVT,                    /**< Specific alternate mode event with data. */
#if CCG_UCSI_ENABLE
    AM_EVT_SUPP_ALT_MODE_CHNG,          /**< Same functionality as ALT_MODE_SUPP; new one for UCSI */
#endif /*CCG_UCSI_ENABLE*/
}alt_mode_app_evt_t;

/**
  @typedef alt_mode_app_cmd_t
  @brief This enumeration holds all possible APP command related to Alt modes handling.
 */
typedef enum
{
    AM_NO_CMD = 0,                      /**< Empty command. */
    AM_SET_TRIGGER_MASK,                /**< Sets trigger mask to prevent auto-entering of the selected
                                             alternate modes. */
    AM_CMD_ENTER = 3,                   /**< Enter to selected alternate mode. */
    AM_CMD_EXIT,                        /**< Exit from selected alternate mode. */
    AM_CMD_SPEC,                        /**< Specific alternate EC mode command with data. */
#if CY_PD_REV3_ENABLE
    AM_CMD_RUN_UFP_DISC                 /**< Runs Discover command when CCG is UFP due to PD 3.0 spec . */
#endif /* CY_PD_REV3_ENABLE */
}alt_mode_app_cmd_t;

/**
 * @typedef fail_status_t
 * @brief Enum of the VDM failure response status.
 */
typedef enum
{
    BUSY = 0,                           /**<  Target is BUSY.  */
    GOOD_CRC_NOT_RSVD,                  /**<  Good CRC wasn't received.  */
    TIMEOUT,                            /**<  No response or corrupted packet.  */
    NACK                                /**<  Sent VDM NACKed.  */
} fail_status_t;


/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @union alt_mode_evt_t
  @brief Alt modes manager application event/command structure.
 */
typedef union
{
    uint32_t val;                        /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing alternate modes manager event/command . */
    struct ALT_MODE_EVT
    {
        uint32_t data_role       : 1;    /**< Current event sender data role. */
        uint32_t alt_mode_evt    : 7;    /**< Alt mode event index from alt_mode_app_evt_t structure. */
        uint32_t alt_mode        : 8;    /**< Alt mode ID. */
        uint32_t svid            : 16;   /**< Alt mode related SVID. */
    }alt_mode_event;                     /**< Union containing the alt mode event value. */

    /** @brief Struct containing alternate modes manager event/command data. */
    struct ALT_MODE_EVT_DATA
    {
        uint32_t evt_data        : 24;   /**< Alt mode event's data. */
        uint32_t evt_type        : 8;    /**< Alt mode event's type. */
    }alt_mode_event_data;                /**< Union containing the alt mode event's data value. */

}alt_mode_evt_t;

/**
 * @brief This type of the function is used by alt modes manager to communicate with
 * any of supported alt modes.
 *
 * @param port Port index the function is performed for .
 *
 * @return None.
 */
typedef void
(*alt_mode_cbk_t) (
        uint8_t         port);

/**
 * @brief This type of the function is used by alt modes manager to run alternate
 * mode analysis of received APP command.
 *
 * @param port Port index the function is performed for.
 * @param hpi_cmd Received APP command data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contains unacceptable fields.
 */
typedef bool
(*alt_mode_app_cbk_t) (
        uint8_t         port,
        alt_mode_evt_t  app_cmd);

/**
  @struct alt_mode_reg_info_t
  @brief This structure holds all necessary information on Discovery Mode stage
  for supported alternate mode when alt modes manager registers new alt mode.
 */
typedef struct
{
    uint8_t atch_type;                      /**< Target of disc svid (cable or device/ama). */
    uint8_t data_role;                      /**< Current data role. */
    uint8_t alt_mode_id;                    /**< Alt mode ID. */
    cy_en_pd_sop_t cbl_sop_flag;                     /**< Sop indication flag. */
    cy_pd_pd_do_t svid_emca_vdo;                  /**< SVID VDO from cable Discovery mode response. */
    cy_pd_pd_do_t svid_vdo;                       /**< SVID VDO from Discovery mode SOP response. */
    const atch_tgt_info_t* atch_tgt_info;   /**< Attached trgets info (dev/ama/cbl) from Discovery ID response. */
    alt_mode_app_evt_t app_evt;             /**< APP event. */
} alt_mode_reg_info_t;

/**
  @struct alt_mode_info_t
  @brief This structure holds all necessary information  for interaction between
  alt modes manager and selected alternate mode.
 */
typedef struct
{
    alt_mode_state_t mode_state;                /**< Alternate mode state. */
    alt_mode_state_t sop_state[(uint8_t)CY_PD_SOP_DPRIME + 1u]; /**< VDM state for SOP/SOP'/SOP" packets. */
    uint8_t vdo_max_numb;                       /**< Maximum number of VDO that alt mode can handle */
    uint8_t obj_pos;                            /**< Alternate mode object position. */
    uint8_t cbl_obj_pos;                        /**< Cabel object position. */
    uint8_t alt_mode_id;                        /**< Alternate mode ID. */
    cy_pd_pd_do_t vdm_header;                         /**< Buffer to hold VDM header. */
    cy_pd_pd_do_t* vdo[(uint8_t)CY_PD_SOP_DPRIME + 1u];               /**< Pointers array to alt mode VDO buffers */
    uint8_t vdo_numb[(uint8_t)CY_PD_SOP_DPRIME + 1u];           /**< Current number of VDOs used for processing in VDO buffers */
    alt_mode_cbk_t cbk;                         /**< Alternate mode callback function. */
    bool is_active;                             /**< Active mode flag. */
    bool custom_att_obj_pos;                    /**< Object position field in Att VDM used by alt mode as custom. */
    bool uvdm_supp;                             /**< Flag to indicate if alt mode support unstructured VDMs. */
    bool set_mux_isolate;                       /**< Flag to indicate if MUX should be set to safe state while
                                                     ENTER/EXIT alt mode is being processed. */
    /* Application control information */
    bool app_evt_needed;                        /**< APP event flag. */
    alt_mode_evt_t app_evt_data;                /**< APP event data. */
    alt_mode_app_cbk_t eval_app_cmd;            /**< APP command cbk. */
} alt_mode_info_t;

/**
 * @typedef reg_alt_modes
 * @brief Alternate mode handler type.
 */
typedef alt_mode_info_t* 
(*reg_alt_modes)(
        uint8_t port, 
        alt_mode_reg_info_t* reg_info);

/**
 * @struct reg_am_t
 * @brief structure to hold the alternate modes SVID and handler.
 */
typedef struct
{
    uint16_t svid;                          /**< Alternate mode SVID. */
    reg_alt_modes reg_am_ptr;               /**< Alternate mode SVID handler. */
} reg_am_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function register pointers to attached dev/ama/cable info and run
 * alt modes manager if success.
 *
 * @param port Port index the function is performed for.
 * @param atch_tgt_info Pointer to struct which holds discovery info about attached targets.
 * @param vdm_msg_info Pointer to struct which holds info of received/sent VDM
 *
 * @return VDM manager task VDM_TASK_ALT_MODE if CCG support any alternate
 * mode. If CCG doesn't support alternate modes function returns VDM_TASK_EXIT.
 */
vdm_task_t reg_alt_mode_mngr(uint8_t port, atch_tgt_info_t* atch_tgt_info, vdm_msg_info_t* vdm_msg_info);

/**
 * @brief This function uses by DFP VDM manager to run alt modes manager processing.
 *
 * @param port Port index the function is performed for.
 * @param vdm_evt Current DFP VDM manager event.
 *
 * @return DFP VDM manager task based on alt modes manager processing results.
 */
vdm_task_t vdm_task_mngr_alt_mode_process(uint8_t port, vdm_evt_t vdm_evt);

/**
 * @brief This function run received VDM analisys if CCG is UFP
 *
 * @param port Port index the function is performed for.
 * @param vdm Pointer to pd packet which contains received VDM.
 *
 * @return true if received VDM could handled successful and VDM response need
 * to be sent with ACK. If received VDM should be NACKed then returns false
 */
bool eval_rec_vdm(uint8_t port, const cy_stc_pdstack_pd_packet_t *vdm);

/**
 * @brief Fill alt mode APP event fields with appropriate values.
 *
 * @param port Port index the event is performed for.
 * @param svid SVID of alternate mode which event refers to.
 * @param am_idx Index of alternate mode in compatibility table which event refers to.
 * @param evt Alternate mode APP event.
 * @param data Alternate mode APP event data.
 *
 * @return pointer to the event related data.
 */
const uint32_t* form_alt_mode_event(uint8_t port, uint16_t svid, uint8_t am_idx, alt_mode_app_evt_t evt, uint32_t data);

/**
 * @brief This function analyzes, parses and run alternate mode analysis function
 * if received command is specific alt mode command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received alt mode APP command.
 * @param data Pointer to received alt mode APP command additional data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contains unacceptable fields.
 */
bool eval_app_alt_mode_cmd(uint8_t port, uint8_t *cmd, uint8_t *data);

/**
 * @brief Check whether the VDM manager for the selected port is idle.
 *
 * @param port Port index the function is performed for.
 *
 * @return true if manager is busy, false - if idle.
 */
bool is_alt_mode_mngr_idle(uint8_t port);

/**
 * @brief Prepare the alt modes manager for device deep sleep.
 *
 * @param port Index of USB-PD port to be prepared for sleep.
 *
 * @return None
 */
void alt_mode_mngr_sleep(uint8_t port);

/**
 * @brief Restore the alt modes manager state after waking from deep sleep.
 *
 * @param port Index of USB-PD port to be restored.
 *
 * @return None
 */
void alt_mode_mngr_wakeup(uint8_t port);

/**
 * @brief Resets alternate mode command info structure.
 *
 * @param info Pointer to alternate mode info structure.
 *
 * @return None.
 */
void reset_alt_mode_info(alt_mode_info_t* info);

/**
 * @brief Returns pointer to VDM buffer.
 *
 * @param port Index of Type-C port.
 *
 * @return Pointer to VDM buffer.
 */
cy_stc_pdstack_dpm_pd_cmd_buf_t* get_vdm_buff(uint8_t port);

/**
 * @brief Check for presence of alt modes for given svid in alt modes compatibility table.
 *
 * @param svid Received SVID.
 * @param port Index of Type-C port.
 *
 * @return index of supported SVID by CCG.
 */
uint8_t is_svid_supported(uint16_t svid, uint8_t port);

/**
 * @brief Retrieve the current alternate mode status for a PD port.
 *
 * @param port PD port to be queried.
 * @return The alternate mode status.
 */
uint8_t alt_mode_get_status(uint8_t port);

/**
 * @brief Reset alt modes layer.
 * @param port PD port index.
 * @return None
 */
void alt_mode_layer_reset(uint8_t port);

/**
 * @brief Reset the Alternate mode manager state.
 * @param port PD port index.
 * @return None
 */
void alt_mode_mngr_reset_info(uint8_t port);

/**
 * @brief Exit all active alternate modes.
 * @param port PD port index.
 * @param send_vdm_exit Flag which indicates is sending Exit VDM is required during exit all modes procedure.
 * @param exit_all_cbk Callback which will call after exit all alt modes procedure will be finifed.
 * @return None
 */
void alt_mode_mngr_exit_all(uint8_t port, bool send_vdm_exit, cy_pdstack_pd_cbk_t exit_all_cbk);

/**
 * @brief Get number of the alternate modes for choosen port.
 * @param port PD port index.
 * @return Number of the alternates modes for the choosen port and current data role.
 */
uint8_t get_alt_mode_numb(uint8_t port);

/**
 * @brief Get index of selected SVID from config table 
 * @param port PD port index.
 * @param type type of the port i.e. cy_en_pd_port_type_t.
 * @param svid SVID.
 * @return SVID index.
 */
uint8_t get_alt_modes_config_svid_idx(uint8_t port, cy_en_pd_port_type_t type, uint16_t svid);

/**
 * @brief Set mask for alternate modes which should be enabled 
 * @param port PD port index.
 * @param mask mask.
 * @return void.
 */
void set_alt_mode_mask(uint8_t port, uint16_t mask);

/**
 * @brief Set alternate modes custom SVID 
 * @param port PD port index.
 * @param svid SVID.
 * @return true if the SVID is set otherwise false.
 */
bool set_custom_svid(uint8_t port, uint16_t svid);

/** 
 * @brief Returns alternate modes custom SVID. 
 * @param port PD port index.
 * @return SVID value. */
uint16_t get_custom_svid(uint8_t port);

/**
 * @brief Returns SVID which is at index position in configuration table if such SVID is avaliable 
 * @param port PD port index.
 * @param idx index of the SVID.
 * @return SVID value. */
uint16_t get_svid_from_idx(uint8_t port, uint8_t idx);

/**
 * @brief Get alt mode info 
 * @param port PD port index.
 * @param alt_mode_idx alt mode index.
 * @return pointer to the alternate mode info.
 */
alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx);
/** @cond DOXYGEN_HIDE */

#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )
/* Fast access functions */
    
/**
 * @brief Get active alt modes 
 * @param port PD port index.
 * @return the active alt mode mask info.
 */
uint32_t get_active_alt_mode_mask(uint8_t port);

/**
 * @brief This function set the alt mode state for given port.
 *
 * @param port Index of Type-C port.
 * @param alt mode Index of Type-C port.
 *
 * @return True if SVID is supported by CCG.
 */
void set_alt_mode_state(uint8_t port_idx, uint8_t alt_mode_idx);

/**
 * @brief Get a bitmap of supported alt modes, given the current connection 
 * @param port PD port index.
 * @return the supported alternate modes.
 */
uint32_t get_supp_alt_modes(uint8_t port);

#endif /*CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED*/

/** @endcond */

#endif /* ALT_MODES_MNGR_H_ */

/* [] END OF FILE */

