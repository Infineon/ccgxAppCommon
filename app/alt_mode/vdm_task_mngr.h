/***************************************************************************//**
* \file vdm_task_mngr.h
* \version 1.1.0 
*
* VDM task manager header file.
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _VDM_TASK_MNGR_H_
#define _VDM_TASK_MNGR_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include "cy_pdstack_common.h"
#include "config.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define PD_SVID_ID_HDR_VDO_START_IDX        (4u)
/**< Start index of received VDM packet in case of response to DISC SVID command. */

#define PD_DISC_ID_AMA_VDO_IDX              (3u)
/**< Index of AMA VDO in DISCOVER_ID response from port partner (after skipping the VDM header). */

#if SAVE_SUPP_SVID_ONLY
#define MAX_SVID_VDO_SUPP                   (4u)
/**< Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#else
#define MAX_SVID_VDO_SUPP                   (32u)
/**< Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#endif

#define MAX_CABLE_SVID_SUPP                 (4u)
/**< Maximum number of cable SVIDs VDM task manager can hold in the memory. */

#define STD_SVID                            (0xFF00u)
/**< Standard vendor ID allocated to USB PD specification. */

#define MAX_DISC_SVID_COUNT                 (10u)
/**< Maximum number of DISCOVER_SVID attempts that will be performed. */

#define DATA_RST_RETRY_NUMB                 (5u)
/**< Maximum number of Data Reset command retries. */

#define USB4_EUDO_HOST_PARAM_SHIFT          (13u)
/**< Bit shift to match host_supp configuration parameter with USB4 EUDO . */
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef vdm_task_t
  @brief This enumeration lists the various VDM manager tasks to handle VDMs.
 */
typedef enum
{
    VDM_TASK_WAIT = 0,           /**< DFP manager wait task while waiting for VDM response. */
    VDM_TASK_INIT,               /**< This task is responsible for initializing of  VDM manager. */
    VDM_TASK_DISC_ID,            /**< This task is responsible for VDM Discovery ID flow. */
    VDM_TASK_DISC_SVID,          /**< This task is responsible for VDM Discovery SVID flow. */
    VDM_TASK_REG_ATCH_TGT_INFO,  /**< This task is responsible for registering of Discovery result
                                      information in alt mode manager. */
    VDM_TASK_USB4_TBT,           /**< This task handles the USB4 data discovery and entry. */
    VDM_TASK_EXIT,               /**< This task deinits  VDM task manager. */
    VDM_TASK_SEND_MSG,           /**< This task is responsible for forming and sending VDM message . */
    VDM_TASK_ALT_MODE            /**< This task is responsible for running of alt mode manager . */
}vdm_task_t;

/**
  @typedef vdm_evt_t
  @brief This enumeration lists the various VDM manager events to handle VDMs.
 */
typedef enum
{
    VDM_EVT_RUN = 0,             /**< This event is responsible for running any of DFP VDM manager task . */
    VDM_EVT_EVAL,                /**< This event is responsible for evaluating VDM response . */
    VDM_EVT_FAIL,                /**< This event notifies task manager task if VDM response fails . */
    VDM_EVT_EXIT                 /**< This event runs exiting from VDM task manager task . */
}vdm_evt_t;

#if CCG_USB4_SUPPORT_ENABLE
/**
  @typedef usb4_flag_t
  @brief This enumeration lists the various vdm manager flags due to USB4 related handling.
 */
typedef enum
{
    USB4_NONE = 0,                 /**< Empty flag . */
    USB4_FAILED,                   /**< This flag indicates of USB4 discovery procedure failure. */
    USB4_PENDING,                  /**< This flag indicates that USB4 entry handling should be processed. */
    USB4_TBT_CBL_FIND,             /**< This flag is responsible for initiating of finding TBT VID in cable Disc SVID response. */
    USB4_TBT_CBL_DISC_RUN,         /**< This flag is responsible for initiating of TBT cable Disc mode. */
    USB4_TBT_CBL_ENTER_SOP_P,
    USB4_TBT_CBL_ENTER_SOP_DP,

}usb4_flag_t;
#endif /* CCG_USB4_SUPPORT_ENABLE */    

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @brief This struct holds alternate mode discovery information which is used by alt modes manager.
 */
typedef struct
{
    cy_pd_pd_do_t tgt_id_header;                      /**< Holds Device/AMA discovery ID header . */
    cy_pd_pd_do_t ama_vdo;                            /**< Holds AMA discovery ID response VDO . */
    const cy_pd_pd_do_t* cblVdo;                     /**< Pointer to cable VDO. */
    uint16_t tgt_svid[MAX_SVID_VDO_SUPP];       /**< Holds received SVID for Device/AMA. */
    uint16_t cbl_svid[MAX_SVID_VDO_SUPP];       /**< Holds received SVID for cable. */
}atch_tgt_info_t;

/**
  @brief This struct holds received/sent VDM information which is used by VDM alternative modes managers.
 */
typedef struct
{
    cy_pd_pd_do_t vdm_header;                         /**< Holds VDM buffer. */
    uint8_t sopType;                           /**< VDM SOP type. */
    uint8_t vdo_numb;                           /**< Number of received VDOs in VDM. */
    cy_pd_pd_do_t vdo[CY_PD_MAX_NO_OF_VDO];                 /**< VDO objects buffer. */
}vdm_msg_info_t;

#if CCG_UCSI_ENABLE  
extern bool modal_op_support;
/**< Added check for Modal operation support bit for SOP. If no modal operation, firmware will return success to 
     UCSI command with zero length. */
#endif /* CCG_UCSI_ENABLE */

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/

/**
 * @brief Enables VDM task mngr functionality.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void enable_vdm_task_mngr(uint8_t port);

/**
 * @brief Main VDM task mngr function.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void vdm_task_mngr(uint8_t port);

/**
 * @brief This function deinits VDM task manager and resets all related variables.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void vdm_task_mngr_deinit(uint8_t port);

/**
 * @brief Get the alternate mode state machine to exit any active modes. This is called in response to
 * a VConn related fault condition.
 *
 * @param port PD port index.
 * @return None
 */
void vdm_task_mngr_exit_modes(uint8_t port);

/**
 * @brief Check whether the VDM task for the port is idle.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
bool is_vdm_task_idle(uint8_t port);

#if ((CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP))
/**
 * @brief Starts Discover process when CCG is UFP due to PD 3.0 spec.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return true if Discover process has started, false if VDM manager id busy. 
 */
bool is_ufp_disc_started(uint8_t port);
#endif /* (CY_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) */

/**
 * @brief Obtain the last DISC_ID response received by the CCG device from a port partner.
 * @param context PD Stack context.
 * @param resp_len_p Length of response in PD data objects.
 * @return Pointer to the actual response data.
 */
uint8_t *vdm_get_disc_id_resp(cy_stc_pdstack_context_t * context, uint8_t *resp_len_p);

/**
 * @brief Obtain the collective DISC_SVID response received by the CCG device from a port partner.
 * @param context PD Stack context.
 * @param resp_len_p Length of response in PD data objects.
 * @return Pointer to the actual response data.
 */
uint8_t *vdm_get_disc_svid_resp(cy_stc_pdstack_context_t * context, uint8_t *resp_len_p);

/**
 * @brief Initiate USB4 enter mode command to port partner or cable.
 * @param port PD port index.
 * @param sopType sop packet type.
 * @return None.
 */
void enter_usb4(uint8_t port, cy_en_pd_sop_t sopType);

/**
 * @brief Function to update the MUX settings after USB4 mode entry.
 * @param port PD port index.
 * @param eudo The Enter USB Data Object used to enter USB4.
 * @param val Original value of the MUX data status register.
 * @return Updated value of the MUX data status register.
 */
uint32_t usb4_update_data_status (uint8_t port, cy_pd_pd_do_t eudo, uint32_t val);

#endif /* _VDM_TASK_MNGR_H_ */

/* [] END OF FILE */

