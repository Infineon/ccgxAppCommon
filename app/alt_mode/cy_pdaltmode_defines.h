/******************************************************************************
* File Name:   cy_altmode_defines.h
* \version 2.0
*
* Description: This header file provides common data structures of the
*              Alternate mode feature.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#if !defined(CY_ALTMODE_DEFINES_H)
#define CY_ALTMODE_DEFINES_H

#include "config.h"
#include "cy_usbpd_defines.h"
#include "cy_pdstack_common.h"
#include "app_timer_id.h"
#include "cy_scb_i2c.h"
#include "cy_usbpd_config_table.h"

/**
********************************************************************************
*  PDAltMode Middleware Library
*
* The PDAltMode middleware implements state machines defined in:
* * **Universal Serial Bus Power Delivery Specification**.
* * **VESA DisplayPort Alt Mode on USB Type-C Standard**.
* * **ThunderboltTM Interconnect Specification**.
* * **Goshen Ridge Thunderbolt/USB4 Controller**.
*
* The middleware provides a set of Alt Mode APIs through which the application can
* initialize, monitor and configure different VDM Alt Modes:
* * Display Port.
* * TBT.
* * USB4.
*
* The PDAltMode middleware operates on top of the USBPD driver included in the
* MTB PDL CAT2(mtb-pdl-cat2) Peripheral Driver Library and the PdStack
* Middleware.
*
* The PDAltMode Middleware is released in source form.
*
* <b>Features:</b>
* 1. Support PD Alternative Modes entering and simultaneously handling up to 4
* AltMode.
* 2. Support External I2C Slave Interface.
* 3. Support the following AltModes by default:
*    * DP
*    * TBT
*    * USB4
*
********************************************************************************
* \section section_pdaltmode_general_description General Description
********************************************************************************
*
* Include cy_pdaltmode_common.h, cy_pdaltmode_defines.h to
* get access to all functions and other declarations in this library. See the
* \ref section_pdaltmode_quick_start to start using the PDStack.
*
* Refer to the \ref section_pdaltmode_toolchain section for compatibility
* information.
*
* Refer to the \ref section_pdaltmode_changelog section for change history.
*
* PdAltMode operates on the top of the usbpd driver. The usbpd driver has
* some prerequisites for proper operation.
* Refer to the "USBPD (USB Power Delivery)" section of the MTB PDL CAT2(mtb-pdl-cat2)
* Peripheral Driver Library API Reference Manual.
* Also, refer to the section for
* the different PDStack middleware restrictions and limitations.
*
********************************************************************************
* \section section_pdaltmode_quick_start Quick Start Guide
********************************************************************************
*
* PDAltMode middleware can be used in various Development Environments such as
* ModusToolbox, MBED, etc. Refer to the \ref section_pdaltmode_toolchain
* section.
*
* The below steps describe the simplest way of enabling the PDAltMode
* middleware in the application.
*
* 1. Open/Create an application where PdAltMode functionality is needed.
*
* 2. Add the PDAltMode middleware to your project. This quick start guide
* assumes that the environment is configured to use the MTB CAT2 Peripheral
* Driver Library(PDL) for development and the PDL is included in the project.
* If you are using the ModusToolbox development environment select the
* application in the Project Explorer window and select the PDAltMode Middleware
* in the Library Manager.
*
* 3. Include cy_pdaltmode_defines.h and other header files to get access to all
* functions and other declarations in this library.
*
* 4. Define the following data structures required by the PDAltMode Middleware:
*   * VDM Info Configuration
*    cy_stc_pdaltmode_vdm_info_config_t  * vdmInfoConfig;
*   * Alt Mode Manager Configuration
*    cy_stc_pdaltmode_cfg_settings_t  * altModeCfg;
*   * TBT Configuration
*    cy_stc_pdaltmode_tbthost_cfg_settings_t  * tbtCfg;
*   * DP Configuration
*    cy_stc_pdaltmode_dp_cfg_settings_t  * dpCfg;
*
* The PDLatMode library uses these set of configuration for running internal
* state machine that implement different PD Alternatives Modes.
*
* 5. Initialize the PDAltMode middleware once at the start.
*
* 6. Start the VDM and AltMode managers operations.
*    This initializes the DP, TBT and etc configuration and start state machines
*    that control external mux.
*
* 7. Invoke Cy_PdAltMode_Manager_Task function from the main processing loop of the
* application to handle the Vdm and AltMode Managers tasks for each PD Port.
*
********************************************************************************
* \section section_pdaltmode_toolchain Supported Software and Tools
********************************************************************************
*
* This version of the PDAltMode Middleware was validated for the compatibility
* with the following software and tools:
*
* <table class="doxtable">
*   <tr>
*     <th>Software and Tools</th>
*     <th>Version</th>
*   </tr>
*   <tr>
*     <td>ModusToolbox Software Environment</td>
*     <td>3.0</td>
*   </tr>
*   <tr>
*     <td>mtb-pdl-cat2</td>
*     <td>1.5.0</td>
*   </tr>
*   <tr>
*     <td>pdstack</td>
*     <td>2.0.0</td>
*   </tr>
*   <tr>
*     <td>GCC Compiler</td>
*     <td>9.3.1</td>
*   </tr>
*   <tr>
*     <td>IAR Compiler</td>
*     <td>8.42.2</td>
*   </tr>
*   <tr>
*     <td>Arm Compiler 6</td>
*     <td>6.13</td>
*   </tr>
* </table>
*
********************************************************************************
* \section section_pdaltmode_changelog Changelog
********************************************************************************
*
* <table class="doxtable">
*   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
*   <tr>
*     <td>1.0</td>
*     <td>Initial Version</td>
*     <td></td>
*   </tr>
* </table>
*
********************************************************************************
* \section section_pdaltmode_more_information More Information
********************************************************************************
*
* For more information, refer to the following documents:
*
* * <a href="https://www.cypress.com/products/modustoolbox-software-environment">
*      <b>ModusToolbox Software Environment, Quick Start Guide, Documentation,
*         and Videos</b>
*   </a>
*
* * <a href="http://www.cypress.com/an232553">
*      <b>AN232553 Getting Started with PMG1 MCU on ModusToolbox</b>
*   </a>
*
* * <a href="http://www.cypress.com/an232565">
*      <b>AN232565 EZ-PD PMG1 Hardware Design Guidelines and Checklist</b>
*   </a>
*
* * <a href="https://infineon.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html">
*   <b>PDL API Reference</b></a>
*
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s0-architecture-technical-reference">
*      <b>PMG1-S0 Architecture Technical Reference Manual</b>
*   </a>
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s0-registers-technical-reference-manual">
*      <b>PMG1-S0 Technical Reference Manual</b>
*   </a>
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s1-architecture-technical-reference">
*      <b>PMG1-S1 Architecture Technical Reference Manual</b>
*   </a>
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s1-registers-technical-reference-manual">
*      <b>PMG1-S1 Technical Reference Manual</b>
*   </a>
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s2-architecture-technical-reference">
*      <b>PMG1-S2 Architecture Technical Reference Manual</b>
*   </a>
* * <a href="https://www.cypress.com/documentation/technical-reference-manuals/pmg1-family-pmg1-s2-registers-technical-reference-manual">
*      <b>PMG1-S2 Technical Reference Manual</b>
*   </a>
*
* * <a href="http://www.cypress.com/ds231596">
*      <b>PMG1-S0 Datasheet</b>
*   </a>
* * <a href="http://www.cypress.com/ds231597">
*      <b>PMG1-S1 Datasheet</b>
*   </a>
* * <a href="http://www.cypress.com/ds231598">
*      <b>PMG1-S2 Datasheet</b>
*   </a>
* * <a href="http://www.cypress.com/ds231598">
*      <b>CCG7D Datasheet</b>
*   </a>
* * <a href="http://www.cypress.com/ds231598">
*      <b>CCG7S Datasheet</b>
*   </a>
*
* * <a href="http://www.cypress.com">
*      <b>Cypress Semiconductor</b>
*   </a>
*
* \note
* The links to the other software component's documentation (middleware and PDL)
* point to GitHub to the latest available version of the software.
* To get documentation of the specified version, download from GitHub and unzip
* the component archive. The documentation is available in
* the <i>docs</i> folder.
*
********************************************************************************
*
* \defgroup group_pdaltmode_macros Macros
* \brief
* This section describes the PDAltMode Macros.
*
* \defgroup group_pdaltmode_functions Functions
* \brief
* This section describes the PDAltMode Function Prototypes.
*
* \defgroup group_pdaltmode_data_structures Data Structures
* \brief
* This section describes the data structures defined by the PDAltMode.
*
* \defgroup group_pdaltmode_enums Enumerated Types
* \brief
* This section describes the enumeration types defined by the PDAltMode.
*
*/


/*******************************************************************************
*                              Type Definitions
*******************************************************************************/

#ifndef DFP_ALT_MODE_SUPP
/* Enable Alternate Mode support when CCG is DFP. */
#define DFP_ALT_MODE_SUPP                           (0u)
#endif /* DFP_ALT_MODE_SUPP */

#ifndef UFP_ALT_MODE_SUPP
/* Enable Alt mode as UFP */
#define UFP_ALT_MODE_SUPP                           (0u)
#endif /* UFP_ALT_MODE_SUPP */

#ifndef TBT_DFP_SUPP
/* Enable DisplayPort Source support as DFP. */
#define TBT_DFP_SUPP                                (0u)
#endif /* TBT_DFP_SUPP */

#ifndef TBT_UFP_SUPP
/* Enable DisplayPort Source support as DFP. */
#define TBT_UFP_SUPP                                (0u)
#endif /* TBT_UFP_SUPP */

#ifndef DP_DFP_SUPP
/* Enable DisplayPort Source support as DFP. */
#define DP_DFP_SUPP                                 (0u)
#endif /* DP_DFP_SUPP */

#ifndef DP_UFP_SUPP
/* Enable DisplayPort Source support as DFP. */
#define DP_UFP_SUPP                                 (0u)
#endif /* DP_UFP_SUPP */

#ifndef RIDGE_SLAVE_ENABLE
/* Enable Ridge Slave Interface*/
#define RIDGE_SLAVE_ENABLE                          (0u)
#endif /* RIDGE_SLAVE_ENABLE */

#ifndef VIRTUAL_HPD_ENABLE
/* Virtual HPD usage enable */
#define VIRTUAL_HPD_ENABLE                          (0u)
#endif /* VIRTUAL_HPD_ENABLE */

#ifndef VPRO_WITH_USB4_MODE
/* Enable vPro for USB4 */
#define VPRO_WITH_USB4_MODE                         (0u)
#endif /* VPRO_WITH_USB4_MODE */

#ifndef STORE_DETAILS_OF_HOST
/* Data Exchange between ports enabled */
#define STORE_DETAILS_OF_HOST                       (0u)
#endif /* STORE_DETAILS_OF_HOST */

#ifndef VIRTUAL_HPD_DOCK
#define VIRTUAL_HPD_DOCK                            (0u)
#endif /* VIRTUAL_HPD_DOCK */

#ifndef GATKEX_CREEK
#define GATKEX_CREEK                                (0u)
#endif /* GATKEX_CREEK */

#ifndef MUX_DELAY_EN
#define MUX_DELAY_EN                                (0u)
#endif /* MUX_DELAY_EN */

#ifndef CCG_BB_ENABLE
#define CCG_BB_ENABLE                               (0u)
#endif /* CCG_BB_ENABLE */

#ifndef AM_PD3_FLOW_CTRL_EN
#define AM_PD3_FLOW_CTRL_EN                         (0u)
#endif /* AM_PD3_FLOW_CTRL_EN */

#if (defined (CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || \
        defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_PMG1S3))
/* Enabled features common for CCG5, CCG5C, CCG6, CCG6SF, CCG6DF and PMG1S3 */
#define CY_PD_CCG5_TO_PMG1S3_FEATURE                (1u)
#endif

#ifndef CY_PD_CCG5_TO_PMG1S3_FEATURE
#define CY_PD_CCG5_TO_PMG1S3_FEATURE                (0u)
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */


#if STORE_DETAILS_OF_HOST
#define DO_NOT_UPDATE_US_IF_T_AME_TIMER_IS_RUNNING   (1u)
#endif /* STORE_DETAILS_OF_HOST */

/* Features that are not used in Dock Solution */
#ifndef SAVE_SUPP_SVID_ONLY
#define SAVE_SUPP_SVID_ONLY                         (0u)  /**< Enable saving only SVIDs which are supported by CCG. */
#endif /* SAVE_SUPP_SVID_ONLY */
#define DP_UFP_DONGLE                               (0u)
#define DP_GPIO_CONFIG_SELECT                       (0u)
#define AMD_SUPP_ENABLE                             (0u)
#define MUX_UPDATE_PAUSE_FSM                        (0u)
#define ICL_ENABLE                                  (0u)
#define ICL_ALT_MODE_HPI_DISABLED                   (0u)
#define ICL_ALT_MODE_EVTS_DISABLED                  (0u)
#define ICL_ENABLE_WAIT_FOR_SOC                     (0u)
#define BB_RETIMER_ENABLE                           (0u)
#define BB_RETIMER_DEBUG_MODE_SUPP                  (0u)
#define CCG_UCSI_ENABLE                             (0u)

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

/** Timer IDs for the Alternate Mode Layer */
#define ALT_MODE_TIMER_START_ID                                 (APP_TIMERS_RESERVED_START_ID + 16u)

/** 0: Timer for initiating virtual HPD dequeue. */
#define APP_RIDGE_INIT_HPD_DEQUEUE_TIMER_ID                     (ALT_MODE_TIMER_START_ID + 0u)
/** Timer period in ms for initiating virtual HPD dequeue. */
#define RIDGE_INIT_HPD_DEQUEUE_TIMER_PERIOD                     (1u)
/** 1: Timer used to initiate Virtual HPD IRQ CLEAR ACK to the Thunderbolt Controller. */
#define APP_INITIATE_SEND_IRQ_CLEAR_ACK                         (ALT_MODE_TIMER_START_ID + 1u)
/** Timer period for initiating Virtual HPD IRQ CLEAR ACK. */
#define APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD                  (1u)
/** 2: Timer used to send Hard Reset upon USB3 Host connected */
#define SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_TIMER         (ALT_MODE_TIMER_START_ID + 2u)
/** Timer Period used to send Hard Reset upon USB3 Host connected */
#define SEND_HARD_RESET_UPON_USB3_HOST_CONNECTION_PERIOD        (50u)
/** 3: Timer used to Restart Ds Port*/
#define RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER              (ALT_MODE_TIMER_START_ID + 3u)
/**  Timer Period used to Restart Ds Port */
#define RESTART_DS_PORT_USB3_HOST_CONNECTION_TIMER_PERIOD       (650u)
/** 4: Timer used to delay US port SUB COnnection bya AME Timer  */
#define DELAY_US_PORT_USB_CONNECTION_BYAMETIMOUT_TIMER          (ALT_MODE_TIMER_START_ID + 4u)
/** Timer Period used to delay US port SUB COnnection bya AME Timer  */
#define DELAY_US_PORT_USB_CONNECTION_BYAMETIMOUT_TIMER_PERIOD   (500)
/** 5: Timer used to update Ridge Status after Hard Reset */
#define TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET              (ALT_MODE_TIMER_START_ID + 5u)
/** Timer Period used to update Ridge Status after Hard Reset */
#define TIMER_UPDATE_RIDGE_STATUS_AFTER_HARD_RESET_PERIOD       (500u)
/** 6: Goshen Ridge Mux Delay Timer */
#define GR_MUX_DELAY_TIMER                                      (ALT_MODE_TIMER_START_ID + 6u)
/** Goshen Ridge Mux Delay Timer Period */
#define GR_MUX_VDM_DELAY_TIMER_PERIOD                           (10u)
/** 7: Send Data reset from Ridge controller timer */
#define SEND_DATA_RESET_FROM_RIDGE_TIMER                        (ALT_MODE_TIMER_START_ID + 7u)
/** Send Data reset from Ridge controller timer period (in ms). */
#define SEND_DATA_RESET_FROM_RIDGE_TIMER_PERIOD                 (50u)
/** 8: Alt Mode Callback Timer Period */
#define ALT_MODE_CBK_TIMER                                      (ALT_MODE_TIMER_START_ID + 8u)
/** 8: Alt Mode Attention Callback Timer Period */
#define ALT_MODE_ATT_CBK_TIMER                                    (ALT_MODE_TIMER_START_ID + 9u)
/** Periodicity of checking if alt mode which is on pause could be run. */
#define APP_ALT_MODE_POLL_PERIOD                                (5u)

#if ((ICL_SLAVE_ENABLE) && (ICL_ENABLE_WAIT_FOR_SOC))
#define TBT_MODE_EXIT_CHECK_TIMER                        (ALT_MODE_TIMER_START_ID + 10u)
#endif /* ((ICL_SLAVE_ENABLE) && (ICL_ENABLE_WAIT_FOR_SOC)) */

#if MUX_DELAY_EN && ICL_ENABLE
#define APP_HPD_DELAY_TIMER                                (ALT_MODE_TIMER_START_ID + 11u)
#endif /* MUX_DELAY_EN && ICL_ENABLE */

#if ICL_SLAVE_ENABLE
/* Timers that are not used in Dock Solution */
#define ICL_SOC_TIMEOUT_TIMER
#endif

/** App Mux VDM delay timer period in ms */
#define APP_MUX_VDM_DELAY_TIMER_PERIOD                  (10u)

/** Periodicity of checking if paused HPD Dequeuing could be run. */
#define APP_HPD_DEQUE_POLL_PERIOD                       (3u)

/** Mask to identify is vPro mode supported in HPI host capabilities register. */
#define APP_HPI_VPRO_SUPP_MASK                          (0x10u)

/** Mask to identify active alternate modes. */
#define APP_ALT_MODE_STAT_MASK                          (0x3Fu)

/** Mask to identify that Discovery process completed */
#define APP_DISC_COMPLETE_MASK                          (0x80u)

/** Mask to identify is USB4 mode entered.*/
#define APP_USB4_ACTIVE_MASK                            (0x40u)

/** Mask to identify that USB4 functionality is enabled */
#define USB4_EN_HOST_PARAM_MASK                         (0x20u)

/** VDM busy timer period (in ms). */
#define APP_VDM_BUSY_TIMER_PERIOD                       (50u)

/** Enter USB4 retry (on failure) timer period in ms. */
#define APP_USB4_ENTRY_RETRY_PERIOD                     (5u)

/** VDM retry (on failure) timer period in ms. */
#define APP_VDM_FAIL_RETRY_PERIOD                       (100u)

/** Time allowed for cable power up to be complete. */
#define APP_CABLE_POWER_UP_DELAY                        (55u)

/** Cable query delay period in ms. */
#define APP_CABLE_VDM_START_DELAY                       (5u)

/** tAME timer period (in ms). */
#define APP_AME_TIMEOUT_TIMER_PERIOD                    (800u)

/** Delay to be used between cable discovery init commands. */
#define APP_CBL_DISC_TIMER_PERIOD                       (100u)

/** Timer period used to run Vconn swap after V5V was lost and recovered while UFP. */
#define APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD           (1000u)

/** Maximum number of alternate modes which alt modes manager could operates in the same time.
     Setting this to larger values will increase RAM requirement for the projects. */
#define MAX_SUPP_ALT_MODES                              (4u)

#if SAVE_SUPP_SVID_ONLY
/** Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#define MAX_SVID_VDO_SUPP                               (4u)
#else
/** Maximum number of attached target SVIDs VDM task manager can hold in the memory. */
#define MAX_SVID_VDO_SUPP                               (32u)
#endif

/** Size of alt mode APP event data in 4 byte words. */
#define ALT_MODE_EVT_SIZE                               (2u)

/** Maximum number of VDOs DP uses for the alt mode flow. */
#define MAX_DP_VDO_NUMB                                 (1u)

/** Maximum number of VDOs Thunderbolt-3 uses for the alt mode flow. */
#define MAX_TBT_VDO_NUMB                                (1u)

#define CY_PDALTMODE_NO_DATA                            (0u)
/**< Zero data. */

#define CY_PDALTMODE_HPD_ENABLE_CMD                     (0u)
/**< Enable HPD application command value. */

#define CY_PDALTMODE_HPD_DISABLE_CMD                    (5u)
/**< Disable HPD application command value. */

/** Standard SVID */
#define CY_PDATMODE_STD_SVID                            (0xFF00u)

/** Display port SVID defined by VESA specification. */
#define CY_PDALTMODE_DP_SVID                            (0xFF01u)

/** Thunderbolt SVID defined by Intel specification. */
#define CY_PDALTMODE_TBT_SVID                           (0x8087u)

/** UUID Size */
#define UUID_SIZE                                       (16u)

/** USB4 Mode */
#define CY_PDALTMODE_USB_MODE_USB4                      (2u)

/** CCG controlled switch for DisplayPort and USB lines. */
#define DP_MUX                                          (1u)

#define CY_PDALTMODE_RIDGE_TBT_MODE_MASK                      (0x10001)
/**< @brief TBT alt mode state bit mask for Titan register. */

#define CY_PDALTMODE_RIDGE_DP_MODE_MASK                       (0x101)
/**< @brief TBT alt mode state bit mask for Titan register. */

#define CY_PDALTMODE_RIDGE_DEBUG_MODE_MASK                    (0x11)
/**< @brief Debug alt mode state bit mask for Status register. */

#define CY_PDALTMODE_TBT_HOST_CONN_MASK                       (0x01)
/**< @brief TBT HOST Connected bit mask for Command register. */

#define CY_PDALTMODE_RIDGE_IRQ_ACK_MASK                       (0x2000)
/**< @brief HPD IRQ ACK bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_HPD_IRQ_MASK                       (0x4000)
/**< @brief HPD IRQ bit mask for Titan Ridge command/status register. */

#define CY_PDALTMODE_RIDGE_HPD_LVL_MASK                       (0x8000)
/**< @brief HPD Level bit mask for Titan Ridge command register. */

#define CY_PDALTMODE_RIDGE_DATA_RESET_MASK                    (0x10000)
/**< @brief Data reset bit mask for Ridge command register. */

#define CY_PDALTMODE_RIDGE_POWER_RESET_MASK                   (0x20000)
/**< @brief Power reset bit mask for Ridge command register. */

#define CY_PDALTMODE_RIDGE_RDY_BIT_MASK                       (0x40000)
/**< @brief GR Ready bit mask. */

#define CY_PDALTMODE_RIDGE_HOST_ALL_CONN_MASK                 (0x39)
/**< @brief Host all connection bit mask. */

#define FAULT_APP_PORT_VCONN_FAULT_ACTIVE                     (0x01)
/**< Status bit that indicates VConn fault is active. */

#define FAULT_APP_PORT_V5V_SUPPLY_LOST                        (0x10)
/**< Status bit that indicates that V5V supply (for VConn) has been lost. */

/** \} group_pdstack_macros */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
* \addtogroup group_pdaltmode_enums
* \{
*/

/**
  @typedef cy_en_pdaltmode_vdm_vconn_rqt_state_t
  @brief This enumeration lists the VDM state with respect to VConn Swap and cable discovery.
 */
typedef enum
{
    VCONN_RQT_INACTIVE = 0,             /**< No VConn request active. */
    VCONN_RQT_PENDING,                  /**< VConn swap and cable discovery request is pending. */
    VCONN_RQT_ONGOING,                  /**< VDM state machine waiting for VConn Swap and cable discovery completion. */
    VCONN_RQT_FAILED                    /**< VConn Swap request rejected by port partner. */
} cy_en_pdaltmode_vdm_vconn_rqt_state_t;

#if STORE_DETAILS_OF_HOST
/**
 * @typedef cy_en_pdaltmode_control_bits
 * @brief This enumeration holds all possible Alt modes.
 */
typedef enum
{
    NONE_MODE                   = 0,            /**< None Mode */
    TBT_MODE_DFP                = (1<<0),       /**< TBT DFP  */
    TBT_MODE_UFP                = (1<<1),       /**< TBT UFP */
    DP_MODE_DFP                 = (1<<2),       /**< DP DFP */
    DP_MODE_UFP                 = (1<<3),       /**< DP UFP */
    USB4_MODE_DFP               = (1<<4),       /**< USB4 DFP */
    USB4_MODE_UFP               = (1<<5),       /**< USB4 UFP */
}cy_en_pdaltmode_control_bits;
#endif /* STORE_DETAILS_OF_HOST */

/**
 * @typedef cy_en_pdaltmode_mngr_state_t
 * @brief This enumeration holds all possible Alt modes manager states when DFP.
 */
typedef enum
{
    ALT_MODE_MNGR_STATE_DISC_MODE = 0,  /**< Alt modes manager discovery mode state. */
    ALT_MODE_MNGR_STATE_PROCESS         /**< Alt modes manager alternate modes processing state. */
}cy_en_pdaltmode_mngr_state_t;

/**
 * @typedef cy_en_pdaltmode_state_t
 * @brief This enumeration holds all possible states of each alt mode which is handled by Alt modes manager.
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
}cy_en_pdaltmode_state_t;

/**
 * @typedef cy_en_pdaltmode_app_evt_t
 * @brief This enumeration holds all possible application events related to Alt modes handling.
 */
typedef enum
{
    AM_NO_EVT = 0,                      /**< Empty event. */
    AM_EVT_SVID_NOT_FOUND,              /**< Sends to EC if UFP does not support any SVID. */
    AM_EVT_ALT_MODE_ENTERED,            /**< Alternate mode entered. */
    AM_EVT_ALT_MODE_EXITED,             /**< Alternate mode exited. */
    AM_EVT_DISC_FINISHED,               /**< Discovery process was finished. */
    AM_EVT_SVID_NOT_SUPP,               /**< CCGx does not support received SVID. */
    AM_EVT_SVID_SUPP,                   /**< CCGx supports received SVID. */
    AM_EVT_ALT_MODE_SUPP,               /**< CCGx supports alternate mode. */
    AM_EVT_SOP_RESP_FAILED,             /**< UFP VDM response failed. */
    AM_EVT_CBL_RESP_FAILED,             /**< Cable response failed. */
    AM_EVT_CBL_NOT_SUPP_ALT_MODE,       /**< Cable capabilities could not provide alternate mode handling. */
    AM_EVT_NOT_SUPP_PARTNER_CAP,        /**< CCGx and UFP capabilities not consistent. */
    AM_EVT_DATA_EVT,                    /**< Specific alternate mode event with data. */
    AM_EVT_SUPP_ALT_MODE_CHNG,          /**< Same functionality as ALT_MODE_SUPP; new one for UCSI */
}cy_en_pdaltmode_app_evt_t;

/**
 * @typedef cy_en_pdaltmode_dp_state_t
 * @brief This enumeration holds all possible DP states.
 */
typedef enum
{
    DP_STATE_IDLE = 0,                  /**< Idle state. */
    DP_STATE_ENTER = 4,                 /**< Enter mode state. */
    DP_STATE_EXIT = 5,                  /**< Exit mode state. */
    DP_STATE_ATT = 6,                   /**< DP Attention state. */
    DP_STATE_STATUS_UPDATE = 16,        /**< DP Status Update state. */
    DP_STATE_CONFIG = 17                /**< DP Configure state. */
}cy_en_pdaltmode_dp_state_t;

/**
  @typedef cy_en_pdaltmode_vdm_task_t
  @brief This enumeration lists the various VDM manager tasks to handle VDMs.
 */
typedef enum
{
    VDM_TASK_WAIT = 0,           /**< DFP manager wait task while waiting for VDM response. */
    VDM_TASK_INIT,               /**< This task is responsible for initializing of VDM manager. */
    VDM_TASK_DISC_ID,            /**< This task is responsible for VDM Discovery ID flow. */
    VDM_TASK_DISC_SVID,          /**< This task is responsible for VDM Discovery SVID flow. */
    VDM_TASK_REG_ATCH_TGT_INFO,  /**< This task is responsible for registering of Discovery result
                                      information in alt mode manager. */
    VDM_TASK_USB4_TBT,           /**< This task handles the USB4 data discovery and entry. */
    VDM_TASK_EXIT,               /**< This task deinits  VDM task manager. */
    VDM_TASK_SEND_MSG,           /**< This task is responsible for forming and sending VDM message . */
    VDM_TASK_ALT_MODE            /**< This task is responsible for running of alt mode manager . */
}cy_en_pdaltmode_vdm_task_t;

/**
  @typedef cy_en_pdaltmode_vdm_evt_t
  @brief This enumeration lists the various VDM manager events to handle VDMs.
 */
typedef enum
{
    VDM_EVT_RUN = 0,             /**< This event is responsible for running any of DFP VDM manager task . */
    VDM_EVT_EVAL,                /**< This event is responsible for evaluating VDM response . */
    VDM_EVT_FAIL,                /**< This event notifies task manager task if VDM response fails . */
    VDM_EVT_EXIT                 /**< This event runs exiting from VDM task manager task . */
}cy_en_pdaltmode_vdm_evt_t;

#if CY_PD_USB4_SUPPORT_ENABLE
/**
  @typedef cy_en_pdaltmode_usb4_flag_t
  @brief This enumeration lists the various vdm manager flags due to USB4 related handling.
 */
typedef enum
{
    USB4_NONE = 0,                  /**< Empty flag . */
    USB4_FAILED,                    /**< This flag indicates of USB4 discovery procedure failure. */
    USB4_PENDING,                   /**< This flag indicates that USB4 entry handling should be processed. */
    USB4_TBT_CBL_FIND,              /**< This flag is responsible for initiating of finding TBT VID in cable Disc SVID response. */
    USB4_TBT_CBL_DISC_RUN,          /**< This flag is responsible for initiating of TBT cable Disc mode. */
    USB4_TBT_CBL_ENTER_SOP_P,       /**< This flag is responsible for initiating of TBT cable SOP' Enter mode. */
    USB4_TBT_CBL_ENTER_SOP_DP,      /**< This flag is responsible for initiating of TBT cable SOP" Disc mode. */

}cy_en_pdaltmode_usb4_flag_t;
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

/**
  @ typedef cy_en_pdaltmode_mux_select_t
  @ brief Possible settings for the Type-C Data MUX.
  @ note This type should be extended to cover all possible modes for the MUX.
 */
typedef enum
{
    MUX_CONFIG_ISOLATE,                  /**< Isolate configuration. */
    MUX_CONFIG_SAFE,                     /**< USB Safe State (USB 2.0 lines remain active) */
    MUX_CONFIG_SS_ONLY,                  /**< USB SS configuration. */
    MUX_CONFIG_DP_2_LANE,                /**< Two lane DP configuration. */
    MUX_CONFIG_DP_4_LANE,                /**< Four lane DP configuration. */
    MUX_CONFIG_USB4_CUSTOM,              /**< USB4 custom configuration. */
    MUX_CONFIG_RIDGE_CUSTOM,             /**< Alpine/Titan Ridge custom configuration. */
    MUX_CONFIG_INIT,                     /**< Enables MUX functionality. */
    MUX_CONFIG_DEINIT,                   /**< Disables MUX functionality. */
} cy_en_pdaltmode_mux_select_t;

/**
  @typedef cy_en_pdaltmode_tbt_state_t
  @brief This enumeration holds all possible TBT states.
 */
typedef enum
{
    TBT_STATE_IDLE = 0,                 /**< Idle state. */
    TBT_STATE_ENTER = 4,                /**< Enter mode state. */
    TBT_STATE_EXIT = 5,                 /**< Exit mode state. */
    TBT_STATE_ATT = 6,                  /**< Attention state. */
} cy_en_pdaltmode_tbt_state_t;

/**
  @ typedef cy_en_pdaltmode_mux_poll_status_t
  @ brief Possible states for the MUX handler.
 */
typedef enum
{
    MUX_STATE_IDLE,                      /**< MUX idle state. */
    MUX_STATE_FAIL,                      /**< MUX switch failed. */
    MUX_STATE_BUSY,                      /**< MUX is busy. */
    MUX_STATE_SUCCESS                    /**< MUX switched successfully. */
} cy_en_pdaltmode_mux_poll_status_t;

/**
 * @typedef cy_en_pdaltmode_ridge_slave_irq_state_t
 * @brief List of states of IRQ queue/dequeue state machine.
 */
typedef enum
{
    DFP_IRQ_DEFAULT_STATE,
    DFP_IRQ_SENT_TO_TR,
    DFP_IRQ_CLR_BY_CCG,
    DFP_IRQ_CLR_ACK_BY_TR,
}cy_en_pdaltmode_ridge_slave_irq_state_t;

/**
   @typedef cy_en_pdaltmode_cable_dp_reset_state_t
   @brief Enumeration tracking current cable (SOP'') soft reset state.
 */
typedef enum {
    CABLE_DP_RESET_IDLE = 0,    /**< No Cable (SOP'') reset state. */
    CABLE_DP_RESET_WAIT,        /**< Waiting for response from cable (SOP'') soft reset. */
    CABLE_DP_RESET_RETRY,       /**< Cable soft reset (SOP'') to be attempted. */
    CABLE_DP_RESET_DONE,         /**< Cable soft reset (SOP'') has been completed. */
    CABLE_DP_RESET_SEND_SPRIME  /**< Cable soft reset (SOP'') has been completed and send SOP Prime msg again. */
} cy_en_pdaltmode_cable_dp_reset_state_t;

/**
  @typedef cy_en_pdaltmode_tbt_cbl_speed_t
  @brief This enumeration holds all possible TBT cable speeds.
 */
typedef enum
{
    TBT_CBL_SPEED_GEN_1 = 1,          /**< USB3.1 gen1 cable (10Gb/s TBT support). */
    TBT_CBL_SPEED_10_GB = 2,          /**< 10Gb/s. */
    TBT_CBL_SPEED_10_20_GB = 3,       /**< 10Gb/s and 20Gb/s. */
} cy_en_pdaltmode_tbt_cbl_speed_t;

/**
  @typedef cy_en_pdaltmode_tbt_cbl_gen_t
  @brief This enumeration holds all possible TBT cable generations.
 */
typedef enum
{
    TBT_CBL_GEN_3 = 0,          /**< 3rd generation TBT (10.3125 and 20.625 Gb/s). */
    TBT_CBL_GEN_4 = 1,          /**< 4th generation TBT (10.0, 10.3125, 20.0 and 20.625 Gb/s). */
} cy_en_pdaltmode_tbt_cbl_gen_t;

/**
  @typedef cy_en_pdaltmode_hw_type_t
  @brief Application HW type values which are used in alt_mode_hw_evt_t structure.
 */
typedef enum
{
    ALT_MODE_MUX = 1,                    /**< HW type - MUX. */
    ALT_MODE_HPD,                        /**< HW type - HPD transceiver/receiver. */

}cy_en_pdaltmode_hw_type_t;

/**
  @typedef cy_en_pdaltmode_dp_port_cap_t
  @brief This enumeration holds possible DP capabilities.
 */
typedef enum
{
    DP_PORT_CAP_RSVD = 0,               /**< Reserved capability. */
    DP_PORT_CAP_UFP_D,                  /**< UFP is UFP_D-capable. */
    DP_PORT_CAP_DFP_D,                  /**< UFP is DFP_D-capable. */
    DP_PORT_CAP_BOTH                    /**< UFP is DFP_D and UFP-D capable. */
}cy_en_pdaltmode_dp_port_cap_t;

/**
  @typedef cy_en_pdaltmode_dp_conn_t
  @brief This enumeration holds possible DFP_D/UFP_D Connected status (Status Update message).
 */
typedef enum
{
    DP_CONN_NONE = 0,                   /**< Neither DFP_D nor UFP_D is connected. */
    DP_CONN_DFP_D,                      /**< DFP_D is connected. */
    DP_CONN_UFP_D,                      /**< UFP_D is connected. */
    DP_CONN_BOTH                        /**< Both DFP_D and UFP_D are connected. */
}cy_en_pdaltmode_dp_conn_t;

/**
  @typedef cy_en_pdaltmode_dp_stat_bm_t
  @brief This enumeration holds corresponding bit positions of Status Update VDM fields.
 */
typedef enum
{
    DP_STAT_DFP_CONN = 0,               /**< DFP_D is connected field bit position. */
    DP_STAT_UFP_CONN,                   /**< UFP_D is connected field bit position. */
    DP_STAT_PWR_LOW,                    /**< Power Low field bit position. */
    DP_STAT_EN,                         /**< Enabled field bit position. */
    DP_STAT_MF,                         /**< Multi-function Preferred field bit position. */
    DP_STAT_USB_CNFG,                   /**< USB Configuration Request field bit position. */
    DP_STAT_EXIT,                       /**< Exit DP Request field bit position. */
    DP_STAT_HPD,                        /**< HPD state field bit position. */
    DP_STAT_IRQ                         /**< HPD IRQ field bit position. */
}cy_en_pdaltmode_dp_stat_bm_t;

/**
  @enum cy_en_pdaltmode_app_cmd_t
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
#if CCG_PD_REV3_ENABLE
    AM_CMD_RUN_UFP_DISC                 /**< Runs Discover command when CCG is UFP due to PD 3.0 spec . */
#endif /* CCG_PD_REV3_ENABLE */
}cy_en_pdaltmode_app_cmd_t;

/**
 * @enum cy_en_pdaltmode_fail_status_t
 * @brief Enum of the VDM failure response status.
 */
typedef enum
{
    BUSY = 0,                           /**<  Target is BUSY.  */
    GOOD_CRC_NOT_RSVD,                  /**<  Good CRC wasn't received.  */
    TIMEOUT,                            /**<  No response or corrupted packet.  */
    NACK                                /**<  Sent VDM NACKed.  */
} cy_en_pdaltmode_fail_status_t;

/**
 * @typedef cy_en_pdaltmode_ridge_slave_reg_addr_t
 *
 * @brief List of Alpine/Titan Ridge slave interface registers.
 *
 * The Thunderbolt Alternate Mode specification defines the following set of registers
 * that should be implemented by a USB-PD port controller in Thunderbolt enabled systems.
 */
typedef enum
{
    RIDGE_REG_CONTROLLER_STATE = 0x03,          /**< PD controller state register. */
    RIDGE_REG_CCG_COMMAND      = 0x50,          /**< Data Control register. */
    RIDGE_REG_CCG_STATUS       = 0x5F,          /**< Data Status register. */
    RIDGE_REG_RETIMER_DEBUG    = 0x5D,          /**< Retimer debug register */
    RIDGE_UUID_REG             = 0x80           /**< UUID register */
} cy_en_pdaltmode_ridge_slave_reg_addr_t;

/**
 * @brief Billboard implementation model.
 */
typedef enum
{
    BB_TYPE_NONE,               /**< No billboard device */
    BB_TYPE_EXTERNAL,           /**< External billboard device */
    BB_TYPE_INTERNAL,           /**< Device supports internal USB module to implement billboard */
    BB_TYPE_EXT_CONFIGURABLE,   /**< External billboard device which supports configuration through PD controller. */
    BB_TYPE_EXT_CFG_HUB,        /**< External billboard device behind USB hub. */
    BB_TYPE_EXT_HX3PD           /**< Billboard/DMC integrated in HX3PD hub. */
} bb_type_t;

/**
 * @brief Billboard module states.
 */
typedef enum
{
    BB_STATE_DEINITED,          /**< Module is not initialized */
    BB_STATE_DISABLED,          /**< Module is initialized but not enabled */
    BB_STATE_BILLBOARD,         /**< Module is active with billboard enumeration */
    BB_STATE_LOCKED,            /**< Module is active with additional functionality */
    BB_STATE_FLASHING,          /**< Module is active with flashing mode enumeration */

} bb_state_t;

/** \} group_pdaltmode_enums */

/**
* \addtogroup group_pdaltmode_data_structures
* \{
*/

/**
 * @struct cy_stc_pdaltmode_vdm_d_modes_t
 * @brief This structure hold all VDM Configuration Info.
 */
typedef struct
{
    uint32_t dataObjsVid;           /**< Data object VID */
    uint32_t modeDataObj[7];        /**< Discovery Modes Data objects */
    uint16_t disModeLength;         /**< Discovery Modes Data objects length */
} cy_stc_pdaltmode_vdm_d_modes_t;

/**
 * @struct cy_stc_pdaltmode_vdm_info_config_t
 * @brief This structure hold all VDM Configuration Info.
 */
typedef struct
{
    uint32_t discId[7];                             /**< Discovery ID data objects */
    uint16_t dIdLength;                             /**< Discovery ID data objects length */
    uint32_t sVid[7];                               /**< Discovery SVID data objects */
    uint16_t sVidLength;                            /**< Discovery SVID data objects length */
    cy_stc_pdaltmode_vdm_d_modes_t discMode[12];    /**< Discovery Modes data objects */
} cy_stc_pdaltmode_vdm_info_config_t;


/**
  @union cy_stc_pdaltmode_hw_evt_t
  @brief Alt mode HW application event/command structure.
 */
typedef union
{

    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Structure containing alternate modes HW event/command . */
    struct ALT_MODE_HW_EVT
    {
        uint32_t evt_data    : 16;      /**< Current event/command data. */
        uint32_t hw_type     : 8;       /**< HW type event/command related to. */
        uint32_t data_role   : 8;       /**< Current data role. */
    }hw_evt;                            /**< Union containing the application HW event/command value. */

}cy_stc_pdaltmode_hw_evt_t;

/**
 * @union cy_stc_pdaltmode_alt_mode_evt_t
 * @brief Alt modes manager application event/command structure.
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

}cy_stc_pdaltmode_alt_mode_evt_t;

/**
 * @union cy_stc_pdaltmode_ridge_reg_t
 * @brief Union to hold AR/TR Registers
 */
typedef union
{
    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing USB-PD controller status to be reported to Alpine/Titan Ridge.
        Using the structure definition corresponding to Titan Ridge in all cases.
     */
    struct USBPD_STATUS_REG
    {
        uint32_t data_conn_pres  : 1;   /**< B0: Whether data connection is present. */
        uint32_t conn_orien      : 1;   /**< B1: CC polarity. */

#if CY_PD_USB4_SUPPORT_ENABLE
        uint32_t retimer         : 1;   /**< B2: Retimer or Redriver. */
#else
        uint32_t active_cbl      : 1;   /**< B2: Active cable. From B22 of Cable MODE Response. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

        uint32_t ovc_indn        : 1;   /**< B3: Over-Current indication. */
        uint32_t usb2_conn       : 1;   /**< B4: USB 2.0 connection. Set when no ALT MODES are active. */
        uint32_t usb3_conn       : 1;   /**< B5: USB 3.1 connection. Set when no ALT MODES are active. */
        uint32_t usb3_speed      : 1;   /**< B6: USB Gen2/Gen1 speed. From B18-B16 of Cable MODE Response. */
        uint32_t usb_dr          : 1;   /**< B7: Data role. DFP=0, UFP=1. */
        uint32_t dp_conn         : 1;   /**< B8: DP connection status. */
        uint32_t dp_role         : 1;   /**< B9: DP direction. Source=0, Sink=1. */
        uint32_t dp_pin_assign   : 2;   /**< B[11-10]: DP pin assignment. 4-lane='b00 2-lane='b01 */
        uint32_t dbg_acc_mode    : 1;   /**< B12: USB Type-C Debug accessory mode. */
        uint32_t irq_ack         : 1;   /**< B13: Set after receiving GoodCRC from Attention message with IRQ_HPD. */
        uint32_t hpd_irq         : 1;   /**< B14: HPD IRQ received from DP Sink. */
        uint32_t hpd_lvl         : 1;   /**< B15: HPD level received from DP Sink. */
        uint32_t tbt_conn        : 1;   /**< B16: TBT connection status. */
        uint32_t tbt_type        : 1;   /**< B17: TBT type. From B16 of UFP MODE Response. */
        uint32_t cbl_type        : 1;   /**< B18: TBT cable type. From B21 of Cable MODE Response. */
        uint32_t pro_dock_detect : 1;   /**< B19: Reporting of vPro Support. */
        uint32_t act_link_train  : 1;   /**< B20: Active TBT link training. From B23 of Cable MODE Response. */
        uint32_t dbg_alt_m_conn  : 1;   /**< B21: NIDnT Alt mode defined in MIPI SVID = 0xFF03. */

#if CY_PD_USB4_SUPPORT_ENABLE
        uint32_t active_cbl      : 1;   /**< B22: Active cable or passive cable. */
        uint32_t usb4_conn       : 1;   /**< B23: USB4 connection status. */
#else
        uint32_t rsvd2           : 1;   /**< B22: Reserved. */
        uint32_t force_lsx       : 1;   /**< B23: Set to zero. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

        uint32_t pwr             : 1;   /**< B24: Indicates PWR mismatch (Used only in TBT BPD). */
        uint32_t cbl_spd         : 3;   /**< B[27-25]: Cable speed. From B18-B16 of Cable MODE Response. */
        uint32_t cbl_gen         : 2;   /**< B[29-28]: Cable generation. From B20-19 of Cable MODE Response. */
        uint32_t rsvd4           : 1;   /**< B30: Reserved. */
        uint32_t interrupt_ack   : 1;   /**< B31: Interrupt indication (Set by EC when in I2C slave Mode). */
    }ridge_stat;                        /**< PD-controller status. */

    /** @brief Struct containing USB-PD controller command register fields for Alpine/Titan Ridge. */
    struct USBPD_CMD_REG
    {
        uint32_t tbt_host_conn   : 1;   /**< B0: TBT host connected. */
#if CY_PD_USB4_SUPPORT_ENABLE
        uint32_t rsvd0           : 1;   /**< B1: Reserved. */
#else
        uint32_t soft_rst        : 1;   /**< B1: Issue USB-PD Controller soft reset. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
        uint32_t i2c_int_ack     : 1;   /**< B2: Alpine/Titan Ridge acknowledge for the interrupt. */
#if CY_PD_USB4_SUPPORT_ENABLE
        uint32_t in_safe_state   : 1;   /**< B3: Ridge DP sets this after confirming high speed I/Os are in USB Type-C Safe State.. */
#else
        uint32_t rsvd1           : 1;   /**< B3: Reserved. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
        uint32_t usb_host_conn   : 1;   /**< B4: Indicates that USB Host is connected (TBT device only). */
        uint32_t dp_host_conn    : 1;   /**< B5: Indicates that DP Host is connected (TBT device only). */
#if BB_RETIMER_ENABLE
        uint32_t rsvd3           : 6;   /**< B[11-6]: Reserved. */
        uint32_t wr_to_retimer   : 1;   /**< B12: If set, write following 4 bytes to retimer's Debug Mode */
#else
        uint32_t rsvd3           : 7;   /**< B[12-6]: Reserved. */
#endif
        uint32_t irq_ack         : 1;   /**< B13: IRQ ACK PD Controller to Titan Ridge. */
        uint32_t hpd_irq         : 1;   /**< B14: HPD IRQ when acting as DP Sink. */
        uint32_t hpd_lvl         : 1;   /**< B15: HPD level when acting as DP Sink. */
#if CY_PD_USB4_SUPPORT_ENABLE
        uint32_t data_rst_req    : 1;   /**< B16: Indicates to the PD that Data Reset should be done. */
        uint32_t pwr_rst_req     : 1;   /**< B17: Indicates to the PD that Power Reset should be done. */
        uint32_t ridge_ready     : 1;   /**< B18: Indicates to the PD that the Ridge is ready to be functional. */
        uint32_t rsvd4           : 13;  /**< B[31-19]: Reserved. */
#else
        uint32_t rsvd4           : 16;  /**< B[31-16]: Reserved. */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */

    }usbpd_cmd_reg;                     /**< PD-controller command register. */

}cy_stc_pdaltmode_ridge_reg_t;

/**
 * @struct cy_stc_pdaltmode_tbthost_cfg_settings_t
 * @brief Structure to hold Thunderbolt Host related config settings.
 */
typedef struct
{
    uint8_t tbtCtrlrType;       /**< Type of Thunderbolt Controller used in the design. */
    uint8_t prefPwrRole;        /**< Preferred power role for the design. Will be enforced through PR_SWAP. */
    uint8_t prefDataRole;       /**< Preferred data role for the design. Will be enforced through DR_SWAP. */
    uint8_t hpdHandling;        /**< Type of HPD handling (GPIO/I2C) used in the design. */
    uint8_t vproCapable;        /**< Whether the design is capable of working with VPro docks. Requires
                                     HPD handling to be I2C based. */
    uint8_t sbuConfig;          /**< Type of SBU configuration used in the design. */
    uint8_t usb4Support;        /**< USB4 roles supported by the host design. */
    uint8_t usb3Support;        /**< USB 3.2 roles supported by the host design. */
    uint8_t hostSupport;        /**< Protocol capabilities (TBT, DP, PCIe) of the host controller. */
    uint8_t nonTbtMux;              /**< Reserved byte for future use. */
    uint8_t rsvd1;              /**< Reserved byte for future use. */
} cy_stc_pdaltmode_tbthost_cfg_settings_t;



/**
 * @struct cy_stc_pdaltmode_custom_alt_cfg_settings_t
 * @brief Struct to hold the Custom Alt mode settings.
 */
typedef struct
{
    uint8_t reserved0;               /**< Reserved for future */
    uint16_t custom_alt_mode;        /**< Custom alt mode */
    uint8_t custom_dfp_mask;         /**< DFP alt modes mask */
    uint8_t custom_ufp_mask;         /**< UFP alt modes mask */
    uint8_t reserved1[2];            /**< Reserved for future */
} cy_stc_pdaltmode_custom_alt_cfg_settings_t;

/**
 * @struct cy_stc_pdaltmode_dp_cfg_settings_t
 * @brief Struct to hold Thunderbolt Host related config settings.
 */


/**
 * @struct cy_stc_pdaltmode_icl_tgl_cfg_settings_t
 * @brief Struct to hold the ICL/TGL settings.
 */
typedef struct
{
    uint8_t table_len;                  /**< Table length in bytes */
    uint8_t icl_i2c_pmc_address;        /**< Configuring I2C slave address to Intel PMC */
    uint8_t icl_i2c_retimer_address[2]; /**< Configuring I2C master address to retimers */
    uint8_t icl_tgl_selection;          /**< Platform selection ICL/TGL */
    uint8_t icl_dual_retimer_enable;    /**< Retimer/Dual retimer enable/disable */
    uint16_t soc_mux_init_delay;        /**< Soc Mux initialization delay in milli seconds */
    uint16_t soc_mux_config_delay;      /**< Soc Mux Config delay in milli seconds */
    uint8_t hpd_irq_explicit_clear;     /**< Flag which indicates whether HPD_IRQ bit should be auto-cleared by CCG */
    uint8_t reserved0;                  /**< Reserved for future */
} cy_stc_pdaltmode_icl_tgl_cfg_settings_t;

/**
 * @struct cy_stc_pdaltmode_amd_cfg_settings_t
 * @brief Struct to hold the AMD APU related config settings.
 */
typedef struct
{
    uint8_t table_len;                  /**< Table length in bytes */
    uint8_t amd_platform_type;          /**< Type AMD APU used in the design. */
    uint8_t amd_apu_mode;               /**< APU Polling or Interrupt based mode */
    uint8_t amd_apu_address;            /**< Configuring I2C slave address */
    uint8_t amd_apu_index;              /**< Configuring APU index which corresponds to slave address */
    uint8_t pref_data_role;             /**< Preferred data role for the design. Will be enforced through DR_SWAP. */
    uint8_t pref_pwr_role;              /**< Preferred power role for the design. Will be enforced through PR_SWAP. */
    uint8_t retimer_type;               /**< Retimer IC type which used in design. */
    uint8_t i2c_retimer_address;        /**< Configuring I2C Retimer address */
    uint8_t usb4_captive;               /**< Indicates whether design supports USB4 handling */
    uint8_t host_support;               /**< Protocol capabilities (TBT, DP, PCIe) of the host controller. */
    uint8_t usb4_support;               /**< USB4 roles supported by the host design. */
    uint8_t usb3_support;               /**< USB 3.2 roles supported by the host design. */
    uint8_t reserved[3];                /**< Reserved for future */
} cy_stc_pdaltmode_amd_cfg_settings_t;

/**
 * @struct cy_stc_pdaltmode_atch_tgt_info_t
 * @brief This struct holds alternate mode discovery information which is used by alt modes manager.
 */
typedef struct
{
    cy_pd_pd_do_t tgt_id_header;                /**< Holds Device/AMA discovery ID header . */
    cy_pd_pd_do_t ama_vdo;                      /**< Holds AMA discovery ID response VDO . */
    const cy_pd_pd_do_t* cblVdo;                /**< Pointer to cable VDO. */
    uint16_t tgt_svid[MAX_SVID_VDO_SUPP];       /**< Holds received SVID for Device/AMA. */
    uint16_t cbl_svid[MAX_SVID_VDO_SUPP];       /**< Holds received SVID for cable. */
}cy_stc_pdaltmode_atch_tgt_info_t;

/**
 * @struct cy_stc_pdaltmode_vdm_msg_info_t
 * @brief This struct holds received/sent VDM information which is used by VDM alternative modes managers.
 */
typedef struct
{
    cy_pd_pd_do_t vdm_header;                   /**< Holds VDM buffer. */
    uint8_t sopType;                            /**< VDM SOP type. */
    uint8_t vdo_numb;                           /**< Number of received VDOs in VDM. */
    cy_pd_pd_do_t vdo[7u];                      /**< VDO objects buffer. */
}cy_stc_pdaltmode_vdm_msg_info_t;

/**
  @struct cy_stc_pdaltmode_alt_mode_reg_info_t
  @brief This structure holds all necessary information on Discovery Mode stage
  for supported alternate mode when alt modes manager registers new alt mode.
 */
typedef struct
{
    uint8_t atch_type;                      /**< Target of disc svid (cable or device/ama). */
    uint8_t data_role;                      /**< Current data role. */
    uint8_t alt_mode_id;                    /**< Alt mode ID. */
    cy_en_pd_sop_t cbl_sop_flag;            /**< Sop indication flag. */
    cy_pd_pd_do_t svid_emca_vdo;            /**< SVID VDO from cable Discovery mode response. */
    cy_pd_pd_do_t svid_vdo;                 /**< SVID VDO from Discovery mode SOP response. */
    const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info;   /**< Attached targets info (dev/ama/cbl) from Discovery ID response. */
    cy_en_pdaltmode_app_evt_t app_evt;      /**< APP event. */
} cy_stc_pdaltmode_alt_mode_reg_info_t;

/**
 * @brief Type of callback function used for notification of control register changes.
 */
typedef void (*ridge_ctrl_change_cb_t)(
        uint8_t port                    /**< Port on which control register change happened. */
        );


/**
 * @brief SOC Alert write function callback.
 */
typedef void (*cy_pdaltmode_soc_intr_write_cbk_t)(uint8_t intrState);


/**
 * @brief SOC I2C write reconfigure callback.
 */
typedef void (*cy_pdaltmode_soc_i2c_wr_reconfigure)(void);


/**
 * @brief Billboard USB descriptor update function callback.
 */
typedef void (*cy_pdaltmode_bb_bos_decr_update_cbk_t)(void* context);

/**
  @struct cy_stc_pdaltmode_ridge_t
  @brief This structure holds Ridge Register value
 */
typedef struct
{
    cy_stc_pdaltmode_ridge_reg_t    ridge_stat;                     /**< Status register value reported to the Ridge. */
    cy_stc_pdaltmode_ridge_reg_t    cmd_reg;                        /**< Command register value provided by the Ridge. */
    uint8_t                         polarity;                       /**< Connection polarity. */
    cy_cb_usbpd_hpd_events_t        hpd_cbk;                        /**< HPD callback used when using virtual HPD. */
    ridge_ctrl_change_cb_t          ctrl_change_cb;                 /**< Callback to called when control register is changed. */
    bool                            hpd_update_req;                 /**< Status of virtual HPD update. */
    uint8_t                         ridge_hpd_state;                /**< Global TR HPD state */
    uint8_t                         ridge_reg_uuid[UUID_SIZE];      /**< UUID */

    volatile bool                   ridge_slave_task_pending;
    volatile uint32_t               ridge_slave_cmd_reg;            /**< Command register for each PD port. */
    volatile uint32_t               ridge_slave_stat_reg;           /**< Status register for each PD port. */

#if BB_RETIMER_ENABLE
    volatile uint32_t  ridge_slave_debug_reg[NO_OF_TYPEC_PORTS];    /**< Debug Mode data from retimer for each PD port. */
    volatile uint8_t   ridge_slave_overlay_num[NO_OF_TYPEC_PORTS];  /**< NIDnT overlay number for each PD port. */
#endif /* BB_RETIMER_ENABLE */

    cy_pdaltmode_soc_intr_write_cbk_t   soc_intr_write;
    cy_pdaltmode_soc_i2c_wr_reconfigure soc_i2c_wr_reconfigure;

    volatile bool                   hpd_run;                        /**< Flag indicating that HPD dequeue process should be run. */
    bool                            virtual_hpd_enable;             /**< Whether this platform uses Virtual (I2C) based HPD. */
#if VIRTUAL_HPD_ENABLE
    uint8_t                         handling_irq_for_dfp;           /**< Holds current IRQ queue/dequeue state. */
#endif

    bool                            ridge_slave_enabled;            /**< Ridge Slave enable flag. */

#if ICL_ENABLE
    volatile bool                   ridge_delay_soc_alert;          /**<Flag indicating that RIDGE soc alert for TBT controller should be delayed. */
#endif
}cy_stc_pdaltmode_ridge_t;


/**
 * @struct cy_stc_pdaltmode_vdm_task_status_t
 * @brief struct to hold vdm manager status
 */
typedef struct
{
    /** Current VDM manager state */
    cy_en_pdaltmode_vdm_task_t      vdm_task;
    /** Current VDM manager event */
    cy_en_pdaltmode_vdm_evt_t vdm_evt;
    #if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    /** Info about cable, device/AMA */
    cy_stc_pdaltmode_atch_tgt_info_t atch_tgt;
    #endif
    /** Struct with received/sent VDM info */
    cy_stc_pdaltmode_vdm_msg_info_t  vdm_msg;
    /** Sent/received VDM retry counters */
    uint8_t         rec_retry_cnt;
    /** Holds svid index if we have more than one Disc SVID response */
    uint8_t         svid_idx;
    /** Holds count of D_SVID requests sent. */
    uint8_t         dsvid_cnt;
    /** VDM EMCA Resets count */
    uint8_t         vdm_emca_rst_count;
    /** VDM EMCA Reset state */
    cy_en_pdaltmode_cable_dp_reset_state_t vdm_emca_rst_state;
    /** VDM VCONN SWAP Request state */
    cy_en_pdaltmode_vdm_vconn_rqt_state_t vdm_vcs_rqt_state;
    /** VDM VCONN SWAP Request count */
    uint8_t vdm_vcs_rqt_count;
#if CY_PD_USB4_SUPPORT_ENABLE
    /** Enter USB data object. */
    cy_stc_pdstack_dpm_pd_cmd_buf_t eudo_buf;
    /** Flag indicating USB4 status. */
    cy_en_pdaltmode_usb4_flag_t      usb4_flag;
    /** Alt modes supported flag */
    uint8_t          alt_modes_not_supp;
#if AMD_SUPP_ENABLE
    /** AMD APU related status. */
    uint32_t         amd_status;
#endif /* AMD_SUPP_ENABLE */
#if RIDGE_SLAVE_ENABLE
    /** Ridge/SoC configuration status. */
    cy_stc_pdaltmode_ridge_reg_t      intel_reg;
#endif /* RIDGE_SLAVE_ENABLE */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
}cy_stc_pdaltmode_vdm_task_status_t;


/**
 * @brief This type of the function is used by alt modes manager to communicate with
 * any of supported alt modes.
 *
 * @param port Port index the function is performed for .
 *
 * @return None.
 */
typedef void (*alt_mode_cbk_t) (void *context);

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
typedef bool (*alt_mode_app_cbk_t) (void *context, cy_stc_pdaltmode_alt_mode_evt_t  app_cmd);


/**
 * @brief This type of the function is used by app layer to call MUX polling
 * function.
 *
 * @param port Port index the function is performed for.
 * @return MUX polling status
 */
typedef cy_en_pdaltmode_mux_poll_status_t (*mux_poll_fnc_cbk_t)(void *context);

/**
  @struct cy_stc_pdaltmode_app_status_t
  @brief This structure holds all necessary information  for interaction between
  alt modes manager and selected alternate mode.
 */
typedef struct
{
    vdm_resp_t vdmResp;                     /**< Buffer for VDM responses. */
    uint8_t vdm_task_en;                    /**< Flag to indicate is vdm task manager enabled. */
    uint8_t disc_cbl_pending;               /**< Flag to indicate is cable discovery is pending. */
    uint8_t cbl_disc_id_finished;           /**< Flag to indicate that cable disc id finished. */
    uint8_t vdm_version;                    /**< Live VDM version. */
    uint8_t alt_mode_trig_mask;             /**< Mask to indicate which alt mode should be enabled by EC. */
    uint8_t dfp_alt_mode_mask;              /**< Mask to enable DFP alternate modes. */
    uint8_t ufp_alt_mode_mask;              /**< Mask to enable UFP alternate modes. */
    uint16_t custom_hpi_svid;               /**< Holds custom alternate mode SVID received from HPI. */
    bool alt_mode_entered;                  /**< Flag to indicate is alternate modes currently entered. */
    bool vdm_prcs_failed;                   /**< Flag to indicate is vdm process failed. */
    bool vdm_retry_pending;                 /**< Whether VDM retry on timeout is pending. */
    uint8_t custom_hpi_host_cap_control;    /**< Holds custom host capabilities register value received from HPI. */
    bool cbl_rst_done;                      /**< Flag to indicate that cable reset was provided. */
    bool trig_cbl_rst;                      /**< Flag to trigger cable reset. */
    bool is_mux_busy;                       /**< Flag to indicate that mux is switching. */
    cy_pdstack_vdm_resp_cbk_t vdm_resp_cbk; /**< VDM response handler callback. */
    bool is_vdm_pending;                    /**< VDM handling flag for MUX callback. */
    mux_poll_fnc_cbk_t mux_poll_cbk;        /**< Holds pointer to MUX polling function. */
    bool usb4_active;                       /**< Indicates that USB4 mode was entered */
    uint8_t usb4_data_rst_cnt;              /**< Indicates number of Dat Reset retries */
    bool retimer_dis_req;                   /**< Flag to indicate disable Retimer request in ridge layer */
    bool usb2Supp;                          /**< USB2 supported flag for Ridge related applications. */
    bool usb3Supp;                          /**< USB3 supported flag for Ridge related applications. */
    bool skip_mux_config;                   /**< Flag to indicate do not configure MUX */
    bool cable_retimer_supp;                /**< Retimer supported flag for Ridge related applications. */
    cy_en_pdaltmode_mux_poll_status_t mux_stat; /**< Indicates current MUX status */
    #if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    cy_pd_pd_do_t tbtCblVdo;                /**< Holds TBT cable VDO. */
    #endif
} cy_stc_pdaltmode_app_status_t;


/**
 * @struct cy_stc_pdaltmode_mngr_info_t
 * @brief This structure holds all necessary information  for interaction between
 * alt modes manager and selected alternate mode.
 */
typedef struct
{
    cy_en_pdaltmode_state_t mode_state;         /**< Alternate mode state. */
    cy_en_pdaltmode_state_t sop_state[CY_PD_SOP_DPRIME + 1u]; /**< VDM state for SOP/SOP'/SOP" packets. */
    uint8_t vdo_max_numb;                       /**< Maximum number of VDO that alt mode can handle */
    uint8_t obj_pos;                            /**< Alternate mode object position. */
    uint8_t cbl_obj_pos;                        /**< Cable object position. */
    uint8_t alt_mode_id;                        /**< Alternate mode ID. */
    cy_pd_pd_do_t vdm_header;                   /**< Buffer to hold VDM header. */
    cy_pd_pd_do_t* vdo[CY_PD_SOP_DPRIME + 1u];  /**< Pointers array to alt mode VDO buffers */
    uint8_t vdo_numb[CY_PD_SOP_DPRIME + 1u];    /**< Current number of VDOs used for processing in VDO buffers */
    alt_mode_cbk_t cbk;                         /**< Alternate mode callback function. */
    bool is_active;                             /**< Active mode flag. */
    bool custom_att_obj_pos;                    /**< Object position field in Att VDM used by alt mode as custom. */
    bool uvdm_supp;                             /**< Flag to indicate if alt mode support unstructured VDMs. */
    bool set_mux_isolate;                       /**< Flag to indicate if MUX should be set to safe state while
                                                     ENTER/EXIT alt mode is being processed. */
    /* Application control information */
    bool app_evt_needed;                        /**< APP event flag. */
    cy_stc_pdaltmode_alt_mode_evt_t app_evt_data;           /**< APP event data. */
    alt_mode_app_cbk_t eval_app_cmd;            /**< APP command cbk. */
} cy_stc_pdaltmode_mngr_info_t;


/**
 * @typedef reg_alt_modes
 * @brief Alternate mode handler type.
 */
typedef cy_stc_pdaltmode_mngr_info_t*
(*reg_alt_modes)(
        void *context,
        cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info);

/**
  @typedef cy_cbk_pdaltmode_hw_cmd_t
  @brief Callback type used for notifications about ALT. MODE Hardware Commands.
 */
typedef void (*cy_cbk_pdaltmode_hw_cmd_t) (void *context, uint32_t command);

/**
 * @struct cy_stc_pdaltmode_reg_am_t
 * @brief structure to hold the alternate modes SVID and handler.
 */
typedef struct
{
    uint16_t svid;                          /**< Alternate mode SVID. */
    reg_alt_modes reg_am_ptr;               /**< Alternate mode SVID handler. */
} cy_stc_pdaltmode_reg_am_t;

/**
 * @struct cy_stc_pdaltmode_alt_mode_mngr_status_t
 * @brief struct to hold alt modes manager status
 */
typedef struct
{
    /** Holds info when register alt mode */
    cy_stc_pdaltmode_alt_mode_reg_info_t   reg_info;
    /** Supported alternate modes. */
    uint32_t              am_supported_modes;
    /** Exited alternate modes. */
    uint32_t              am_exited_modes;
    /** Active alternate modes. */
    uint32_t              am_active_modes;
    /** Pointers to each alt mode info structure */
    cy_stc_pdaltmode_mngr_info_t*      alt_mode_info[MAX_SUPP_ALT_MODES];
    /** Number of existed alt modes */
    uint8_t               alt_modes_numb;
    /** Buffer to hold VDM */
    cy_stc_pdstack_dpm_pd_cmd_buf_t      vdm_buf;
    /** Holds application event data */
    uint32_t              app_evt_data[ALT_MODE_EVT_SIZE];
    /** Current alt modes mngr status */
    cy_en_pdaltmode_mngr_state_t state;
    /** Pointer to vdm_msg_info_t struct in vdm task mngr */
    cy_stc_pdaltmode_vdm_msg_info_t       *vdm_info;
    /** Hold current SVID index for discovery mode command */
    uint8_t               svid_idx;
    /** Check whether the device is a PD 3.0 supporting UFP. */
    uint8_t               pd3_ufp;
    /** Exit all alt modes procedure callback */
    cy_pdstack_pd_cbk_t   exit_all_cbk;
    /** Flag to indicate that exit all alt modes is required */
    bool                  exit_all_flag;
#if HPI_AM_SUPP
    /** Flag to indicate is custom alt mode reset is required */
    bool                  reset_custom_mode;
#endif /* HPI_AM_SUPP */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    cy_pd_pd_do_t               att_header;
    /** Holds unprocessed attention VDM header */
    cy_pd_pd_do_t               att_vdo;
    /** Holds unprocessed attention VDO */
    cy_stc_pdaltmode_mngr_info_t      *att_alt_mode;
    /** Holds pointer to alt mode which saved attention is related to */
#endif
}cy_stc_pdaltmode_alt_mode_mngr_status_t;

/**
 * @struct cy_stc_pdaltmode_dp_status_t
 * @brief DP Status
 */
typedef struct
{
    cy_stc_pdaltmode_mngr_info_t info;                      /**< Alt Mode Manager info  */
    cy_pd_pd_do_t vdo[MAX_DP_VDO_NUMB];                     /**< VDO */
    cy_en_pdaltmode_dp_state_t state;                       /**< DP State */
    cy_en_pdaltmode_mux_select_t dp_mux_cfg;                /**< DP Mux Configuration */
    cy_pd_pd_do_t config_vdo;                               /**< Config VDO */
    bool dp_active_flag;                                    /**< DP Active flag */
    uint8_t ccg_dp_pins_supp;                               /**< CCG's DP pin supported mask*/
    uint8_t partner_dp_pins_supp;                           /**< Port partner's DP pin supported mask*/
    bool dp_2_lane_active;                                  /**< DP2 Lane active flag */
    uint16_t hpd_state;                                     /**< HPD State */
    uint8_t queue_read_index;                               /**< HPD queue read index */
#if !VIRTUAL_HPD_ENABLE
    uint8_t saved_hpd_state;                                /**< Saved HPD state */
#endif /* !VIRTUAL_HPD_ENABLE */
    cy_pd_pd_do_t status_vdo;                               /**< Status VDO */
#if CY_PD_CCG5_TO_PMG1S3_FEATURE
    uint8_t vconn_swap_req;                                 /**< VCONN Swap request */
    uint8_t vconn_init_retry_cnt;                           /**< VCONN init retry count*/
    uint8_t vconn_cbk_retry_cnt;                            /**< VCONN retry count callback */
#endif /* CY_PD_CCG5_TO_PMG1S3_FEATURE */
#if DP_DFP_SUPP
    bool dp_exit;                                           /**< DP Exit */
    uint8_t dp_4_lane;                                      /**< DP4 Lane */
    uint8_t dp_2_lane;                                      /**< DP2 Lane */
    uint8_t dp_cfg_ctrl;                                    /**< DP Configuration control */
    uint8_t max_sop_supp;                                   /**< Max SOP supported */
    uint8_t cable_config_supp;                              /**< Cable configuration support */
    const cy_stc_pdaltmode_atch_tgt_info_t* tgt_info_ptr;   /**< Attached target info pointer */
    bool dp_act_cbl_supp;                                   /**< DP active cable support */
    cy_pd_pd_do_t cable_vdo[MAX_DP_VDO_NUMB];               /**< Cable VDO */
#if MUX_UPDATE_PAUSE_FSM
    cy_en_pdaltmode_dp_state_t prev_state;                  /**< DP previous state */
#endif /* MUX_UPDATE_PAUSE_FSM */
#endif /* DP_DFP_SUPP */

}cy_stc_pdaltmode_dp_status_t;

/**
 * @struct cy_stc_pdaltmode_tbt_status_t
 * @brief TBT Status
 */
typedef struct
{
    cy_stc_pdaltmode_mngr_info_t            info;                   /**< Alt Mode Manager info */
    cy_en_pdaltmode_tbt_state_t             state;                  /**< TBT state */
    cy_pd_pd_do_t                           vdo[MAX_TBT_VDO_NUMB];  /**< VDO array */
    cy_pd_pd_do_t                           enter_mode_vdo;         /**< Enter mode VDO */
    cy_stc_pdaltmode_ridge_reg_t            ridge_status;           /**< Ridge Status */
    uint8_t                                 max_sop_supp;           /**< Max SOP supported */
    uint8_t                                 vpro_supp;              /**< VPro supported */
#if TBT_DFP_SUPP
#if CY_PD_USB4_SUPPORT_ENABLE
    cy_pd_pd_do_t                           dev_vdo;                /**< Device VDO */
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
    const cy_stc_pdaltmode_atch_tgt_info_t* tgt_info_ptr;           /**< Attached target info pointer */
#endif /* TBT_DFP_SUPP */
}cy_stc_pdaltmode_tbt_status_t;

/**
 * @struct cy_stc_pdaltmode_hw_details_t
 * @brief Alt Mode HW details
 */
typedef struct
{
    cy_cbk_pdaltmode_hw_cmd_t hw_cmd_cbk;                    /**< Holds command callback information. */
    volatile cy_en_usbpd_hpd_events_t alt_mode_hpd_state;     /* Holds current HPD pin status */
    cy_en_pdaltmode_mux_select_t app_mux_state;             /**< App mux state */
    uint32_t hw_sln_data;                                    /**< Holds hw solution event/command data */
    uint32_t app_mux_saved_custom_data;
    cy_en_pdaltmode_mux_select_t app_mux_saved_state;
    bool app_mux_update_req;
    volatile uint8_t mux_cur_state;                            /**< Store the current MUX config. */
    volatile bool alt_mode_cmd_pending;                         /**< Holds current HPD command status. */
}cy_stc_pdaltmode_hw_details_t;

/**
 * @struct cy_stc_pdaltmode_host_details_t
 * @brief Host Details Status
 */
typedef struct
{
    /*
     *   Mode mask:
     *   Bit 0 - TBT DFP mode
     *   Bit 1 - TBT UFP mode
     *   Bit 2 - DP DFP mode
     *   Bit 3 - DP UFP mode
    */
    void *          dsAltModeContext;                   /**< PD AltMode context pointer for DS port*/
    void *          usAltModeContext;                   /**< PD AltMode context pointer for US port*/
    uint32_t        back_up_ridge_status;               /**< Back Up Ridge status*/
    uint8_t         host_mode_mask;                     /**< Host Mode Mask */
    uint8_t         ds_mode_mask;                       /**< DS mode mask  */
    uint8_t         host_dp_2_lane_mode_ctrl;           /**< Host DP 2 Lane mode control */
    uint8_t         ds_dp_2_lane_mode_ctrl;             /**< DS DP 2 Lane mode control */
    cy_stc_pdaltmode_ridge_reg_t     current_host_cap;  /**< Current Host Capability */
    cy_pd_pd_do_t   host_eudo;                          /**< Host USB4 data object */
    uint32_t        host_details;                       /**< Host details */
    uint8_t         gr_rdy_bit;                         /**< Ridge gr_rdy bit value */
    bool            is_host_details_available;          /**< Is Host details available */
    uint8_t         ds_send_eu_with_host_present_set;   /**< Send USB4 eu with host present bit set  */
}cy_stc_pdaltmode_host_details_t;

/**
 * @brief Internal billboard module handle structure.
 *
 * No explicit structure for the handle is expected to be created outside of
 * the billboard module implementation.
 */
typedef struct bb_handle
{
    /* Common billboard fields. */
    bb_type_t type;                     /**< Billboard implementation type */
    bb_state_t state;                   /**< Billboard current state */
    uint8_t num_alt_modes;              /**< Number of valid alternate modes */
    uint8_t bb_add_info;                /**< AdditionalFailureInfo field in billboard
                                             capability descriptor. */
    uint32_t alt_status;                /**< Current alternate mode status -
                                             The status can hold a maximum of 16
                                             alternate modes (2bits each). */
    uint32_t timeout;                   /**< Pending timeout count in ms for billboard
                                             interface disable. */

    /* Internal billboard fields. */
    uint8_t *ep0_buffer;                /**< EP0 data buffer pointer in case of
                                             internal billboard implementation. */
    bool usb_configured;                /**< Internal flag indicating whether the
                                             configuration is enabled or not. This is valid
                                             only for internal billboard implementation. */
    uint8_t usb_port;                   /**< USB port index used in case the device
                                             has multiple USB ports. */
    bool flashing_mode;                 /**< USB flashing mode state. Valid only for
                                             internal billboard implementation. */
    bool usb_i2cm_mode;                 /**< USB I2C master bridge mode state. Valid only
                                             for internal billboard implementation
                                             supporting I2C master bridge interface. */
    bool queue_enable;                  /**< Billboard interface enable request
                                             queued. Valid only for internal
                                             billboard implementation. */
    bool queue_disable;                 /**< Billboard interface disable request
                                             queued. Valid only for internal
                                             billboard implementation. */
    bool queue_i2cm_enable;             /**< USB-I2C master mode enable request.
                                             Valid only for internal billboard
                                             implementation. */

    cy_pdaltmode_bb_bos_decr_update_cbk_t update_bos_descr_cbk;     /**< callback for updating BOS descriptor */

} bb_handle_t;

/**
 * @struct cy_stc_pdaltmode_context_t
 * @brief Structure to PDSTACK Alt Mode Middleware context information.
 */
typedef struct
{
    cy_stc_pdstack_context_t *                      pdStackContext;                     /**< PD Stack Library context Pointer */
    cy_stc_pdaltmode_app_status_t                   appStatus;                          /**< Application Layer status  */
    cy_stc_pdaltmode_alt_mode_mngr_status_t         altModeStatus;                      /**< Alt Mode Manager Status */
    cy_stc_pdaltmode_mngr_info_t                    altModeInfo;                        /**< Alt Mode Info */
    cy_stc_pdaltmode_vdm_task_status_t              vdmStat;                            /**< VDM Task Manager Status */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    cy_stc_pdaltmode_hw_details_t                   hwDetails;                            /**< Alt Mode HW details */
    cy_stc_pdaltmode_dp_status_t                    dpStatus;                           /**< DP Status */
    cy_stc_pdaltmode_tbt_status_t                   tbtStatus;                          /**< TBT Status */
    cy_stc_pdaltmode_ridge_t                        ridge;                              /**< Ridge Status */
    cy_stc_pdaltmode_host_details_t                 hostDetails;                        /**< Host Details */
#endif
    uint8_t                                         vdmIdVdoCnt;                        /**< VDM Id VDO count */
    uint8_t                                         vdmSvidVdoCnt;                      /**< VDM SVID VDO count */
    volatile uint8_t * volatile                     fault_status_p;                     /**< Pointer to Fault status bits for this port. */
    uint8_t                                         rsvd;                               /**< Reserved. */
    volatile bb_handle_t                            billboard;                            /**< Billboard interface internal handle structure */
    cy_pd_pd_do_t                                   vdmIdVdoResp[CY_PD_MAX_NO_OF_DO];   /**< Stores the actual Discover ID response data */
    cy_pd_pd_do_t                                   vdmSvidVdoResp[CY_PD_MAX_NO_OF_DO]; /**< Stores the actual Discover SVID response data */
    cy_pd_pd_do_t *                                 vdmIdVdoP;                          /**< Stores pointer to Discover ID response data */
    cy_pd_pd_do_t *                                 vdmSvidVdoP;                        /**< Stores pointer to Discover SVID response data */
    uint16_t                                        vdmModeDataLen;                     /**< Stores Discover Modes response VDO count */
    uint8_t *                                       vdmModeDataP;                       /**< Stores pointer to Discover Modes response data */
    const cy_stc_pdaltmode_vdm_info_config_t *      vdmInfoConfig;                      /**< VDM Info config table */

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    const cy_stc_pdaltmode_cfg_settings_t *         altModeCfg;                         /**< Alt Mode config table */
    const cy_stc_pdaltmode_tbthost_cfg_settings_t * tbtCfg;                             /**< TBT config table*/
    const cy_stc_pdaltmode_dp_cfg_settings_t *      dpCfg;                              /**< DP  config Table */
    const cy_stc_pdaltmode_icl_tgl_cfg_settings_t * iclCfg;                             /**< ICL config Table */
    const cy_stc_pdaltmode_amd_cfg_settings_t *     amdCfg;                             /**< AMD config Table */
    #endif

} cy_stc_pdaltmode_context_t;

/** \} group_pdaltmode_data_structures */

#endif /* CY_ALTMODE_DEFINES_H */

/* [] END OF FILE */
