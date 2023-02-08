/******************************************************************************
* File Name:   pdss_hal.h
* \version 2.0
*
* Description: CCG PD PHY driver module header file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef _PDSS_HAL_H_
#define _PDSS_HAL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <cy_pdstack_common.h>
#include "cy_usbpd_hpd.h"
#include "cy_usbpd_typec.h"
#include "cy_usbpd_vbus_ctrl.h"
/**
* \addtogroup group_ccgxAppCommon_macros
* \{
*/
/*******************************************************************************
 * CCG Device Specific Constants.
 * Note: These values should not be modified.
 ******************************************************************************/

#define PDSS_DRP_TOGGLE_PERIOD_MS               (75u)           /**< Hardware based DRP toggle period in ms. */
#define PDSS_DRP_HIGH_PERIOD_MS                 (30u)           /**< Rp assert period in ms for hardware DRP toggle. */
#define PDSS_DRP_TOGGLE_PERIOD_VAL              (2500u)         /**< Hardware DRP toggle period in LF clock cycles. */
#define PDSS_DRP_HIGH_PERIOD_VAL                (1100u)         /**< Rp assert period in LF clock cycles. */
#define PDSS_CC_FILT_CYCLES                     (10)            /**< Number of LF clock cycles used for filtering
                                                                     CC comparator output. */

/** @cond DOXYGEN_HIDE */
#define PDSS_VBUS_IN_REF_SEL_VAL                (1u) /* 500mV */

/* The TRIM6 and TRIM3 registers needs to be overwritten by firmware. */
#define SILICON_TRIM6_V1P575_TRIM_VALUE         (3u)
#define SILICON_TRIM3_V0P55_TRIM_VALUE          (2u)

/* TRIM0_TX_TRIM register needs to be overwritten by firmware. */
#define TRIM0_TX_TRIM_VALUE_3A                  (2u)
#define TRIM0_TX_TRIM_VALUE_900MA               (0u)

/** @endcond */

#define PGDO_PD_ISINK_TERMINAL_VAL              (0x3F)  /**< Maximum value of PGDO pull-down current sink. */
#define PGDO_PD_ISINK_TERMINAL_VAL_1            (0x1F)  /**< Maximum value of PGDO pull-down current sink extension. */

#define AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFDu)
/**< Select types of PD control messages for which GoodCRC response should be sent. All except
     GoodCRC are enabled. */

#define AUTO_DATA_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFFu)
/**< Select types of PD data messages for which GoodCRC response should be sent. */

#define AUTO_EXTD_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFFu)
/**< Select types of PD extended data messages for which GoodCRC response should be sent. */

#define PD_MSG_HDR_REV2_IGNORE_MASK             (0x8010)
/**< Mask representing bits that should be ignored in a PD 2.0 message header. */

#define EXPECTED_GOOD_CRC_HDR_MASK              (0x7E0Fu)
/**< Mask representing header bits that should be matched for expected GoodCRC detection. */

#define EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3    (PD_MSG_HDR_REV2_IGNORE_MASK)
/**< Mask representing additional header bits that should be matched for expected GoodCRC detection
     during PD 3.0 operation. */

#define EXPECTED_GOOD_CRC_CLEAR_MASK            (0xF1FFu)
/**< Mask to clear expected GoodCRC header field. */

#define RX_CNT_MAX_VAL                          (0x0Fu)
/**< 1/16 of the number of RX clock cycles required by CC receiver to detect idle state. */

#define RX_UI_BOUNDARY_DELTA_VAL                (0x02u)
/**< CC receiver voltage threshold selection. Must be set to 2 for a 12 MHz RX CLK. */

/** @cond DOXYGEN_HIDE */

/* Recommended CC transceiver settings for various CCG devices. */

#if (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG4) || defined(CY_DEVICE_CCG4PD3))

#define RX_CNT_MAX_CFG                  (RX_CNT_MAX_VAL << PDSS_RX_CC_RX_CNT_MAX_POS)
#define RX_UI_BOUNDARY_DELTA_CFG        (RX_UI_BOUNDARY_DELTA_VAL << PDSS_RX_CC_RX_UI_BOUNDARY_DELTA_POS)
#define RX_CC_CFG                       (RX_CNT_MAX_CFG | RX_UI_BOUNDARY_DELTA_CFG | PDSS_RX_CC_DELAY_AUTO)

#define RX_SOP_GOOD_CRC_EN_CTRL_CFG     (PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN | \
                                         PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_PRIME_SOP_GOOD_CRC_EN |   \
                                         PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN)

#define RX_SOP_GOOD_CRC_EN_CTRL_REV3_CFG (PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN |        \
                                          PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_PRIME_SOP_GOOD_CRC_EN |          \
                                          PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN |      \
                                          PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN |    \
                                          PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_PRIME_SOP_GOOD_CRC_EN |      \
                                          PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN)

#endif /* (defined(CY_DEVICE_CCG3) || defined(CY_DEVICE_CCG4) || defined(CY_DEVICE_CCG4PD3)) */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_PAG1S) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

#define RX_CNT_MAX_CFG                  (RX_CNT_MAX_VAL << PDSS_RX_CC_0_CFG_RX_CNT_MAX_POS)
#define RX_UI_BOUNDARY_DELTA_CFG        (RX_UI_BOUNDARY_DELTA_VAL << PDSS_RX_CC_0_CFG_RX_UI_BOUNDARY_DELTA_POS)
#define RX_CC_CFG                       (RX_CNT_MAX_CFG | RX_UI_BOUNDARY_DELTA_CFG)

#define RX_SOP_GOOD_CRC_EN_CTRL_CFG     (0u)
#define RX_SOP_GOOD_CRC_EN_CTRL_REV3_CFG (0u)

#endif /* (defined(CY_DEVICE_CCGx)) */

#if ADVDM_SUPPORT
#define  EN_DEBUG_PRIME_SOP_DET     (11u)
#define  EN_DEBUG_DBL_PRIME_DET     (12u) 
#endif

#define RX_ORDER_SET_CTRL_CFG           (PDSS_RX_ORDER_SET_CTRL_SOP_CMP_OPT |           \
                                         PDSS_RX_ORDER_SET_CTRL_RST_CMP_OPT |           \
                                         PDSS_RX_ORDER_SET_CTRL_PREAMBLE_SOP_EN |       \
                                         PDSS_RX_ORDER_SET_CTRL_PREAMBLE_RST_EN)
/**< RX_ORDER_SET_CTRL register configuration. */

#define CRC_COUNTER_CFG                 (300u)
/**< CRC_COUNTER register configuration, 1ms. */

#define BUS_IDLE_CNT_VAL                (10u)
#define IDLE_COUNTER_VAL                (40u)
#define IFG_COUNTER_VAL                 (10u)

#define BUS_IDLE_CNT_CFG                (BUS_IDLE_CNT_VAL << PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_POS)
#define IDLE_COUNTER_CFG                (IDLE_COUNTER_VAL << PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_POS)
#define IFG_COUNTER_CFG                 (IFG_COUNTER_VAL << PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_POS)

#define INTER_PACKET_COUNTER_CFG        (BUS_IDLE_CNT_CFG | IDLE_COUNTER_CFG | IFG_COUNTER_CFG)
/**< INTER_PACKET_COUNTER register configuration. */

#define EXTENDED_DATA_BIT_LOC_VAL                    (15u)      /**< Location of extended bit in header. */
#define EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_VAL    (16u)      /**< Location of first data byte in extended message. */
#define EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_VAL     (24u)      /**< Location of second data byte in extended message. */
#define CHUNK_BIT_LOCATION_VAL                       (15u)      /**< Location of chunked bit in extended header. */

#define HEADER_INFO_CFG         ((EXTENDED_DATA_BIT_LOC_VAL << 8) |                     \
                                 (EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_VAL << 12) |    \
                                 (EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_VAL << 17) |     \
                                 (CHUNK_BIT_LOCATION_VAL << 22))

/** @endcond */

#define PDSS_MAX_TX_MEM_SIZE                    (16u)   /**< PD transmit FIFO size in 4-byte words. */
#define PDSS_MAX_TX_MEM_HALF_SIZE               (8u)    /**< Half of PD transmit FIFO size in 4-byte words. */

#define PDSS_MAX_RX_MEM_SIZE                    (16u)   /**< PD receive FIFO size in 4-byte words. */
#define PDSS_MAX_RX_MEM_HALF_SIZE               (8u)    /**< Half of PD receive FIFO size in 4-byte words. */

/** @cond DOXYGEN_HIDE */

/* PD block interrupt vector numbers for various supported devices. */
#if defined(CY_DEVICE_CCG4)
#define PD_PORT0_INTR0                          (14u)
#define PD_PORT1_INTR0                          (15u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (7u)

#elif defined(CY_DEVICE_CCG3)
#define PD_PORT0_INTR0                          (13u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (255u)

#elif (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2))
#define PD_PORT0_INTR0                          (10u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (5u)
#define PD_PORT1_INTR1                          (255u)

#elif (defined (CY_DEVICE_CCG5C) || defined (CY_DEVICE_CCG6))
#define PD_PORT0_INTR0                          (14u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (7u)
#define PD_PORT1_INTR1                          (255u)

#elif defined (CY_DEVICE_CCG5)
#define PD_PORT0_INTR0                          (15u)
#define PD_PORT1_INTR0                          (16u)

#define PD_PORT0_INTR1                          (7u)
#define PD_PORT1_INTR1                          (8u)

#elif defined(CY_DEVICE_PAG1S)
#define PD_PORT0_INTR0                          (5u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (2u)
#define PD_PORT1_INTR1                          (255u)

#elif defined(CY_DEVICE_CCG6DF)
#define PD_PORT0_INTR0                          (14u)
#define PD_PORT1_INTR0                          (15u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (7u)

#elif defined(CY_DEVICE_CCG6SF)
#define PD_PORT0_INTR0                          (14u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (255u)

#elif defined (CY_DEVICE_CCG7D) 
#define PD_PORT0_INTR0                          (13u)
#define PD_PORT1_INTR0                          (14u)

#define PD_PORT0_INTR1                          (5u)
#define PD_PORT1_INTR1                          (6u)

#elif defined (CY_DEVICE_CCG7S)

#define PD_PORT0_INTR0                          (11u)
#define PD_PORT1_INTR0                          (14u)

#define PD_PORT0_INTR1                          (5u)
#define PD_PORT1_INTR1                          (6u)

#else /* CY_DEVICE_CCGx */
#error "Unsupported CCG device family."
#endif /* CY_DEVICE_CCGx */

#define PD_INTR_MASK                            (0x3FF7FBFFu)

#if CY_PD_REV3_ENABLE

#define RX_INTERRUPTS       (PDSS_INTR0_TX_GOODCRC_MSG_DONE | \
                             PDSS_INTR0_RX_STATE_IDLE       | \
                             PDSS_INTR0_RCV_RST             | \
                             PDSS_INTR0_RX_SRAM_HALF_END    | \
                             PDSS_INTR0_COLLISION_TYPE3)
#else

#define RX_INTERRUPTS       (PDSS_INTR0_TX_GOODCRC_MSG_DONE | \
                             PDSS_INTR0_RX_STATE_IDLE       | \
                             PDSS_INTR0_RCV_RST             | \
                             PDSS_INTR0_COLLISION_TYPE3)

#endif /* CY_PD_REV3_ENABLE */

#define RCV_INTR_MASK       (PDSS_INTR0_RX_STATE_IDLE                 | \
                             PDSS_INTR0_EOP_ERROR                     | \
                             PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE      | \
                             PDSS_INTR0_RCV_BAD_PACKET_COMPLETE       | \
                             PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE      | \
                             PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE | \
                             PDSS_INTR0_RX_SOP                        | \
                             PDSS_INTR0_RX_SRAM_HALF_END              | \
                             PDSS_INTR0_RX_OVER_RUN)

#define TX_INTERRUPTS       (PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE | \
                             PDSS_INTR0_CRC_RX_TIMER_EXP         | \
                             PDSS_INTR0_COLLISION_TYPE1          | \
                             PDSS_INTR0_TX_PACKET_DONE           | \
                             PDSS_INTR0_COLLISION_TYPE2)

#define RST_TX_INTERRUPTS   (PDSS_INTR0_TX_HARD_RST_DONE | PDSS_INTR0_COLLISION_TYPE4)

#define EN_DEFAULT_SOP_DET_VAL          (1u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_PRIME_SOP_DET_VAL            (2u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DBL_SOP_DET_VAL              (4u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DEBUG_PRIME_DET_VAL          (8u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DEBUG_DBL_DET_VAL            (16u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_RX_CABLE_RESET_DET_VAL       (32u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_RX_HARD_RESET_DET_VAL        (64u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)



/* CC threshold definitions. */
#define PD_CMP_VSEL_0_2_V               (0u)
#define PD_CMP_VSEL_0_4_V               (1u)
#define PD_CMP_VSEL_1_43_V              (2u)
#define PD_CMP_VSEL_0_655_V             (3u)
#define PD_CMP_VSEL_0_8_V               (4u)
#define PD_CMP_VSEL_1_235_V             (5u)
#define PD_CMP_VSEL_1_77_V              (6u)
#define PD_CMP_VSEL_2_6_V               (7u)

#define DN_COMP_IDX                     (0u)
#define UP_COMP_IDX                     (1u)

/* Rp threshold index limits in the thresholds LUT. */
#define LOWER_LIMIT_IDX                 (0u)
#define HIGHER_LIMIT_IDX                (1u)

/* No of Rd rows and entries per row, in the thresholds LUT. */
#define RD_ROW_NO                       (3u)
#define RD_COL_WIDTH                    (4u)

/* No of entries per Rp row in the thresholds LUT. */
#define RP_COL_WIDTH                    (2u)

/* DEBUG_CC_1 Register Setting */
#define PDSS_TX_STOP_ON_SWAP_MASK       (1u << 26u)

/* Swap CTRL settings for FRS TX */
#define FRS_TX_IRQ_WIDTH                (450u)
#define FRS_TX_SWAP_CTRL1_DFLT_VAL      (FRS_TX_IRQ_WIDTH | PDSS_SWAPT_CTRL1_DEFAULT_LEVEL |    \
                                         (2u << PDSS_SWAPT_CTRL1_COMMAND_POS) |                 \
                                         PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE)

/****************************************************************************
 * Constants associated with PD ADC (Comparator) settings on CCG.
 ***************************************************************************/

/* Maximum ADC count. */
#define PD_ADC_MAX_VALUE                        (0xFFu)

/* Number of ADC levels. */
#define PD_ADC_NUM_LEVELS                       (PD_ADC_MAX_VALUE + 1u)

/* Bandgap voltage in millivolt. */
#define PD_ADC_BAND_GAP_VOLT_MV                 (1200u)

/*
 * The minimum comparator output shall be limited to this value to guard
 * against ground bounce and other ADC noise.
 */
#define PD_ADC_LEVEL_MIN_THRESHOLD              (4u)

/*
 * The maximum comparator output shall be limited to this value to guard
 * against max level.
 */
#define PD_ADC_LEVEL_MAX_THRESHOLD              (254u)

/* Reference voltage in MX_PD IP (2.0 V) */
#define MX_PD_ADC_REF_VOLT_MV                   (2000u)

/** Loop timeout count for ADC. */
#define PD_ADC_TIMEOUT_COUNT                    (200UL)

/****************************************************************************
 * GPIO mapping on different CCG families for various functions.
 * These mappings are used for the default pins for these functions, which
 * are optimized for the targeted functions.
 ***************************************************************************/

#ifdef CY_DEVICE_CCG4

/*
 * The FET control lines are fixed and cannot be changed. The following
 * are the selected IO lines for the same. The definitions match silicon
 * and should not be modified.
 */
/* Port 1 PCTRL0 GPIO */
#define VBUS_P_CTRL0_GPIO_P1                     (GPIO_PORT_1_PIN_6)
/* Port 1 PCTRL1 GPIO */
#define VBUS_P_CTRL1_GPIO_P1                     (GPIO_PORT_2_PIN_1)
/* Port 1 CCTRL0 GPIO */
#define VBUS_C_CTRL0_GPIO_P1                     (GPIO_PORT_1_PIN_7)
/* Port 1 CCTRL1 GPIO */
#define VBUS_C_CTRL1_GPIO_P1                     (GPIO_PORT_3_PIN_0)

/* Port 2 PCTRL0 GPIO */
#define VBUS_P_CTRL0_GPIO_P2                     (GPIO_PORT_4_PIN_2)
/* Port 2 PCTRL1 GPIO */
#define VBUS_P_CTRL1_GPIO_P2                     (GPIO_PORT_3_PIN_7)
/* Port 2 CCTRL0 GPIO */
#define VBUS_C_CTRL0_GPIO_P2                     (GPIO_PORT_4_PIN_1)
/* Port 2 CCTRL1 GPIO */
#define VBUS_C_CTRL1_GPIO_P2                     (GPIO_PORT_3_PIN_6)

#endif /* CY_DEVICE_CCG4 */

/** @endcond */

#define VBUS_OCP_MODE_EXT               (0u)  /**< OCP through external hardware. */
#define VBUS_OCP_MODE_INT               (1u)  /**< Internal OCP without software debounce or hardware gate control. */
#define VBUS_OCP_MODE_INT_AUTOCTRL      (2u)  /**< Internal OCP with hardware gate control. */
#define VBUS_OCP_MODE_INT_SW_DB         (3u)  /**< Internal OCP with software debounce. */
#define VBUS_OCP_MODE_POLLING           (4u)  /**< Solution level polling based OCP. */

#define VBUS_OCP_GPIO_ACTIVE_LOW        (0u)  /**< GPIO low indicates fault condition. */
#define VBUS_OCP_GPIO_ACTIVE_HIGH       (1u)  /**< GPIO high indicates fault condition. */

#ifdef CY_DEVICE_PAG1S
/* 
 * PDSS_PORT0_PASC frequency in MHz and KHz. This need to match what is
 * configured in PSoC Creator schematic.
 */
#define PASC_CLK_FREQ_MHZ                       (24)
#define PASC_CLK_FREQ_KHZ                       (PASC_CLK_FREQ_MHZ * 1000)

/*
 * Conversion algorithm for frequnecy in KHz to time in units of PASC clock
 * cycle numbers.
 */
#define PASC_FREQ_KHZ_TO_CYCLE_TIME(freq)       PASC_CLK_FREQ_KHZ / (freq)
/*
 * Conversion algorithm for time in nano seconds to time in units of PASC clock
 * cycle numbers. The macro does not account for round off and uses integer floor
 * value always. The caller is expected to ensure that required frequency is
 * made a multiple of 42ns.
 */
#define PASC_NS_TO_CYCLE_TIME(time)             (PASC_CLK_FREQ_MHZ * (time) /\
        1000)

/*
 * TODO: Remove post validation, keep it always enabled
 * Enable variable PTDRV_IN max width based on line condition
 */
#define PASC_PTDRV_IN_VAR_WIDTH_ENABLE          (1u)

/*
 * Enable digital peak for peak detection in valley alogrithm
 */
#define PASC_DIGITAL_PEAK_ENABLE                (0u)

#if PASC_DIGITAL_PEAK_ENABLE
/*
 * Enable digital peak calibration values optimzation offset
 */
#define PASC_DIGITAL_PEAK_OPTIMIZE_ENABLE       (0u)

#if PASC_DIGITAL_PEAK_OPTIMIZE_ENABLE
/*
 * Digital peak calibration offset value.
 * Can be both positive and negative.
 */
#define PASC_P2P_OPTIMIZE_OFFSET                (0)

#define PASC_Z2P_OPTIMIZE_OFFSET                (0)
#endif /* PASC_DIGITAL_PEAK_OPTIMIZE_ENABLE */
#endif /* PASC_DIGITAL_PEAK_ENABLE */

#endif /* CY_DEVICE_PAG1S */

/* Ideal Rsense value in 0.1mOhm units */
#define CSA_IDEAL_RSENSE                         (50u)

#if (defined(CY_DEVICE_CCG7D)|| defined(CY_DEVICE_CCG7S))
#define SFLASH_VBE_LOW_TEMP_TRIM_ADDR            (0x0FFFF479)
/**< Low tempetarure VBJT code sflash address. */

#define SFLASH_VBE_HIGH_TEMP_TRIM_ADDR           (0x0FFFF478)
/**< High tempetarure VBJT code sflash address. */

#define SFLASH_VBE_25_C_TEMP_TRIM_ADDR           (0x0FFFF47A)
/**< 25C tempetarure VBJT code sflash address. */

#define INTERNAL_BJT_LOW_TEMP                    (-40)
/**< Low temperature which corresponds to low tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define INTERNAL_BJT_HIGH_TEMP                   (120u)
/**< High temperature which corresponds to high tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define INTERNAL_BJT_ROOM_TEMP                   (25)
/**< Room temperature used for VBJT code stored in sflash (in Celsius degrees) */
    
#define VBJT_LSB_TEMP_DELTA                      (5u)    
/**< Approximate temperature change for each LSB change in VBJT ADC reading. */

#define INTERNAL_BJT_FAULT_TEMP                               (255)
/**< Fault temperature value returned when sffash VBJT code is not correct (in Celsius degrees) */

#else
#define SFLASH_VBE_LOW_TEMP_TRIM_ADDR                         (0x0FFFF427)
/**< Low tempetarure VBJT code sflash address. */

#define SFLASH_VBE_HIGH_TEMP_TRIM_ADDR                        (0x0FFFF426)
/**< High tempetarure VBJT code sflash address. */

#define SFLASH_VBE_25_C_TEMP_TRIM_ADDR                        (0x0FFFF428)
/**< 25C tempetarure VBJT code sflash address. */

#define INTERNAL_BJT_LOW_TEMP                                 (-40)
/**< Low temperature which corresponds to low tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define INTERNAL_BJT_HIGH_TEMP                                (120u)
/**< High temperature which corresponds to high tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define INTERNAL_BJT_ROOM_TEMP                                (25)
/**< Room temperature used for VBJT code stored in sflash (in Celsius degrees) */

#define INTERNAL_BJT_FAULT_TEMP                               (255u)
/**< Fault temperature value returned when sffash VBJT code is not correct (in Celsius degrees) */

#endif    /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

#if FW_FAULT_TRIGGER_ENABLE
/* Information on HPI command that triggers fault condition from firmware */
#define FAULT_TRIG_HPI_REG                          (HPI_DEV_REG_USERDEF_0A)
#define FAULT_TRIG_CMN_CMD_SIZE                     (2u)
#define FAULT_TRIG_PORT_CMD_SIZE                    (3u)
#define FAULT_TRIG_CMD_CMN_FAULT                    (0u)
#define FAULT_TRIG_CMD_TYPE_ID                      (0u)
#define FAULT_TRIG_FAULT_TYPE_ID                    (1u)
#define FAULT_TRIG_PORT_ID                          (2u)
#endif /* FW_FAULT_TRIGGER_ENABLE */
/** \} group_ccgxAppCommon_macros */
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/
/** \addtogroup group_ccgxAppCommon_enums
* \{
*/
#if 0
/**
 * @typedef PD_ADC_VREF_T
 * @brief ADC block reference voltage selection for the CCG device.
 */
typedef enum PD_ADC_VREF_T
{
    PD_ADC_VREF_PROG = 0,               /**< Programmable reference voltage from the RefGen block. */
    PD_ADC_VREF_VDDD                    /**< VDDD supply used as ADC reference voltage. */
} PD_ADC_VREF_T;
#endif

/**
 * @typedef PD_ADC_ID_T
 * @brief ADC block IDs on the CCG device.
 */
typedef enum PD_ADC_ID_T
{
    PD_ADC_ID_0,                        /**< ADC-0 in the PD block. Supported by all devices. */
    PD_ADC_ID_1,                        /**< ADC-1 in the PD block. Not supported on CCG5x or CCG6x. */
#ifdef CCG3
    PD_ADC_ID_OVUV,                     /**< Dedicated ADC for Over-Voltage/Under-Voltage detection.
                                             Only available in the CCG3 device. */
#endif /* CCG3 */
    PD_ADC_NUM_ADC                      /**< Maximum number of ADCs in the PD block. */
} PD_ADC_ID_T;

/**
 * @typedef PD_ADC_INPUT_T
 * @brief Enumeration of PD ADC input sources. Refer to CCG device datasheet and TRM for
 * more details.
 */
typedef enum PD_ADC_INPUT_T
{
    PD_ADC_INPUT_AMUX_A,                /**< AMUX_A bus. */
    PD_ADC_INPUT_AMUX_B,                /**< AMUX_B bus. */
    PD_ADC_INPUT_BANDGAP,               /**< BANDGAP input. */
    PD_ADC_INPUT_BJT,                   /**< BJT. */
    PD_ADC_NUM_INPUT                    /**< Number of ADC inputs available. */
} PD_ADC_INPUT_T;

/**
 * @typedef PD_ADC_INT_T
 * @brief PD comparator interrupt configuration enumeration.
 * Note: These are the settings for INTR_1_CFG ADC output, not ADC_SAR_CTRL.
 */
typedef enum PD_ADC_INT_T
{
    PD_ADC_INT_DISABLED,                /**< Comparator interrupt disabled. */
    PD_ADC_INT_FALLING,                 /**< Comparator interrupt on falling edge. */
    PD_ADC_INT_RISING,                  /**< Comparator interrrupt on rising edge. */
    PD_ADC_INT_BOTH                     /**< Comparator interrupt on either edge. */
} PD_ADC_INT_T;

/**
 * @typedef pd_phy_evt_t
 * @brief PD PHY state enumeration.
 */
typedef enum
{
    PD_PHY_EVT_TX_MSG_COLLISION,        /**< Bus busy at message transmission. */
    PD_PHY_EVT_TX_MSG_PHY_IDLE,         /**< Bus idle, ready for message transmission. */
    PD_PHY_EVT_TX_MSG_FAILED,           /**< Message transmission was not successful. */
    PD_PHY_EVT_TX_MSG_SUCCESS,          /**< Message transmission was successful. */
    PD_PHY_EVT_TX_RST_COLLISION,        /**< Bus busy just before reset transmission. */
    PD_PHY_EVT_TX_RST_SUCCESS,          /**< Reset transmission was successful. */
    PD_PHY_EVT_RX_MSG,                  /**< Message received. */
    PD_PHY_EVT_RX_MSG_CMPLT,            /**< Message received and GoodCRC sent aka collision type 3. */
    PD_PHY_EVT_RX_RST,                  /**< Reset was received. */
    PD_PHY_EVT_FRS_SIG_RCVD,            /**< FRS signal was received. */
    PD_PHY_EVT_FRS_SIG_SENT,            /**< FRS signal was transmitted. */
    PD_PHY_EVT_CRC_ERROR                /**< Message recieved with CRC error. */
} pd_phy_evt_t;

/**
 * @typedef vbus_ovp_mode_t
 * @brief CCG OVP modes enumeration.
 */
typedef enum
{
    VBUS_OVP_MODE_ADC,                  /**< OVP using CCG internal ADC. */
    VBUS_OVP_MODE_UVOV,                 /**< OVP using the UVOV block on CCGx. Actual gate control is done
                                             by firmware. UVOV block is not supported on CCG4. */
    VBUS_OVP_MODE_UVOV_AUTOCTRL         /**< OVP using the OVOV block with automatic gate driver control.
                                             UVOV block is not supported on CCG4. */
} vbus_ovp_mode_t;

#if 0
/**
 * @typedef vbus_uvp_mode_t
 * @brief CCG UVP modes enumeration.
 */
typedef enum
{
    VBUS_UVP_MODE_ADC,                  /**< UVP using CCG internal ADC. */
    VBUS_UVP_MODE_INT_COMP,             /**< UVP using internal comparator. Actual gate control is done
                                             by firmware. Not supported on CCG4. */
    VBUS_UVP_MODE_INT_COMP_AUTOCTRL     /**< UVP using internal comparator with automatic gate driver control.
                                             Not supported on CCG4. */
} vbus_uvp_mode_t;
#endif

/**
 * @typedef frs_tx_source_t
 * @brief Enum to hold various tx sources of FR-SWAP.
 */
typedef enum
{
    FRS_TX_SOURCE_CPU = 0,              /**< FRS signal sent through FW intervention. */
    FRS_TX_SOURCE_GPIO,                 /**< FRS signal triggered based on GPIO input. */
    FRS_TX_SOURCE_ADC1,                 /**< FRS signal triggered based on ADC-0 comparator output. */
    FRS_TX_SOURCE_ADC2                  /**< FRS signal triggered based on ADC-1 comparator output. */
} frs_tx_source_t;

/**
 * @typedef pd_fet_dr_t
 * @brief Enum to hold power FET gate drive modes.
 */
typedef enum
{
    PD_FET_DR_ACTIVE_HIGH = 0,          /**< Drive GPIO high to enable FET. */
    PD_FET_DR_ACTIVE_LOW,               /**< Drive GPIO low to enable FET. */
    PD_FET_DR_N_JN_FET,                 /**< Use dedicated N-channel MOSFET gate driver. */
    PD_FET_DR_P_JN_FET                  /**< Use dedicated P-channel MOSFET gate driver. */
} pd_fet_dr_t;

/**
 * @typedef sbu_switch_state_t
 * @brief Enum to hold SBU connection state for CCG3/CCG5. CCG3 and CCG5 provide internal
 * switch to route SBU1/SBU2 signals to AUX_P/AUX_N, LSTX/LSRX, or Isolate.
 */
typedef enum
{
    SBU_NOT_CONNECTED,                  /**< SBU pin is isolated. */
    SBU_CONNECT_AUX1,                   /**< Connect SBU pin to AUX_P. */
    SBU_CONNECT_AUX2,                   /**< Connect SBU pin to AUX_N. */
    SBU_CONNECT_LSTX,                   /**< Connect SBU pin to LSTX. */
    SBU_CONNECT_LSRX,                   /**< Connect SBU pin to LSRX. */
    SBU_MAX_STATE                       /**< Invalid value: not supported. */
} sbu_switch_state_t;

/**
 * @typedef aux_resistor_config_t
 * @brief Enum to hold resistor configuration for AUX1 and AUX2. Values assigned
 * are the bit position of corresponding configuration in sbu_ctrl register.
 * Only applicable to CCG3 and CCG5 devices.
 */
typedef enum
{
    AUX_NO_RESISTOR = 0,                        /**< No resistor. */
#ifdef CCG3
    AUX_1_1MEG_PU_RESISTOR = 9,                 /**< AUX1 1M0hm Pullup resistor. */
    AUX_1_100K_PD_RESISTOR = 10,                /**< AUX1 100KOhm Pulldown resistor. */
    AUX_1_470K_PD_RESISTOR = 11,                /**< AUX1 470KOhm Pulldown resistor. */
    AUX_2_100K_PU_RESISTOR = 12,                /**< AUX2 100KOhm Pullup resistor. */
    AUX_2_4P7MEG_PD_RESISTOR = 13,              /**< AUX2 4.7M0hm Pulldown resistor. */
    AUX_2_1MEG_PD_RESISTOR = 14,                /**< AUX2 1M0hm Pulldown resistor. */
#else
    AUX_1_1MEG_PU_RESISTOR = 7,                 /**< AUX1 1M0hm Pullup resistor. */
    AUX_1_100K_PD_RESISTOR = 8,                 /**< AUX1 100KOhm Pulldown resistor. */
    AUX_1_470K_PD_RESISTOR = 9,                 /**< AUX1 470KOhm Pulldown resistor. */
    AUX_2_100K_PU_RESISTOR = 10,                /**< AUX2 100KOhm Pullup resistor. */
    AUX_2_4P7MEG_PD_RESISTOR = 11,              /**< AUX2 4.7M0hm Pulldown resistor. */
    AUX_2_1MEG_PD_RESISTOR = 12,                /**< AUX2 1M0hm Pulldown resistor. */
#endif
    AUX_MAX_RESISTOR_CONFIG                     /**< Not supported. */
} aux_resistor_config_t;

/**
 * @typedef lscsa_app_config_t
 * @brief Enumeration of all possible Low Side CSA applications. Only applicable to CCG3PA and CCG3PA2.
 */
typedef enum
{
    LSCSA_OCP_CONFIG,                           /**< OCP comparator gain control CCG3PA only. */
    LSCSA_EA_CONFIG,                            /**< EA comparator gain control CCG3PA only. */
    LSCSA_PFC_OFF_CONFIG,                       /**< PFC OFF comparator gain control CCG3PA only. */
    LSCSA_PFC_ON_CONFIG,                        /**< PFC ON comparator gain control CCG3PA only. */
    LSCSA_SR_OFF_CONFIG,                        /**< SR OFF comparator gain control CCG3PA only. */
    LSCSA_SR_ON_CONFIG,                         /**< SR ON comparator gain control CCG3PA only. */
    LSCSA_MAX_CONFIG_VALUE                      /**< End of comparator list. */
} lscsa_app_config_t;

/**
 * @typedef comp_id_t
 * @brief List of dedicated comparators supported by the PD block on various CCG devices. The actual
 * comparators supported vary across device families. See comments associated with each comparator ID
 * for list of devices that support this comparator.
 */
typedef enum
{
    COMP_ID_UV                    = 0,          /**< UV comparator. Common for CCG3PA, CCG5x and CCG6x. */
    COMP_ID_OV                    = 1,          /**< OV comparator. Common for CCG3PA, CCG5x and CCG6x. */
    COMP_ID_VBUS_C_MON            = 2,          /**< VBUS_C_MON comparator. CCG3PA and CCG3PA2 only. */

#if (defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6))
    COMP_ID_VBUS_MON              = 2,          /**< VBUS_MON comparator. CCG5, CCG5C and CCG6 only. */
    COMP_ID_VSYS_DET              = 3,          /**< VSYS detection. CCG5, CCG5C and CCG6 only. */
#else
    COMP_ID_VSYS_DET              = 2,          /**< VSYS detection. CCG6DF and CCG6SF only. */
#endif /* (defined(CY_DEVICE_CCG5) || defined(CY_DEVICE_CCG5C) || defined(CY_DEVICE_CCG6)) */

    COMP_ID_VBUS_DISCHARGE        = 3,          /**< Discharge comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_LSCSA_SCP             = 4,          /**< SCP comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_DP_DETACH             = 4,          /**< D+ voltage comparator. CCG6 only. */
    COMP_ID_DM_DETACH             = 5,          /**< D- voltage comparator. CCG6 only. */

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2))
    COMP_ID_LSCSA_OCP             = 5,          /**< OCP comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_LSCSA_PFC             = 6,          /**< PFC comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_VSRC_NEW_P            = 7,          /**< VSRC_NEW comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_VSRC_NEW_M            = 8,          /**< VSRC_NEW comparator. CCG3PA and CCG3PA2 only. */
#elif defined(CY_DEVICE_PAG1S)
    COMP_ID_LSCSA_OCP             = 4,          /**< OCP comparator. PAG1S only. */
    COMP_ID_LSCSA_PFC             = 5,          /**< PFC comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_VSRC_NEW_P            = 6,          /**< VSRC_NEW comparator. CCG3PA and CCG3PA2 only. */
    COMP_ID_VSRC_NEW_M            = 7,          /**< VSRC_NEW comparator. CCG3PA and CCG3PA2 only. */
#else /* CY_DEVICE_CCG7D || CY_DEVICE_CCG7S */
    COMP_ID_LSCSA_PFC             = 2,          /**< CBL_1, PFC comparator. CCG7D. */
    COMP_ID_LSCSA_SR              = 4,          /**< CBL_0, SR comparator. CCG7D. */
    COMP_ID_VSRC_NEW_M            = 5,          /**< VSRC_NEW comparator. CCG7D. */
    COMP_ID_VSRC_NEW_P            = 6,          /**< VSRC_NEW comparator. CCG7D. */
#endif /* CY_DEVICE_CCGx */

    COMP_ID_MAXIUM                 = 9           /**< End of comparator list. */

} comp_id_t;

/**
 * @typedef comp_tr_id_t
 * @brief IDs of trim enabled comparators in CCG3PA and CCG3PA2 devices.
 */
typedef enum
{
    COMP_TR_ID_SR                 = 0,          /**< SR comparator. CCG3PA and CCG3PA2 only. */
    COMP_TR_ID_MAX                = 1           /**< End of comparator list. */
} comp_tr_id_t;

/**
 * @typedef filter_id_t
 * @brief The outputs of the comparators listed in comp_id_t and comp_tr_id_t can be passed through a
 * filter for debouncing. As with the comparators, the filters supported vary across device families.
 * This enumeration lists various filters supported and the comments indicate the devices that support
 * them.
 */
typedef enum
{
    FILTER_ID_UV        = 0,                    /**< UV comparator filter, common for CCG3PA, CCG3PA2, CCG5, CCG5C
                                                     and CCG6. Can run based on HF or LF clock. */
    FILTER_ID_OV        = 1,                    /**< OV comparator filter, common for CCG3PA, CCG3PA2, CCG5, CCG5C
                                                     and CCG6. Can run based on HF or LF clock. */
    FILTER_ID_DISCH_EN  = 2,                    /**< Discharge enable filter for CCG3PA and CCG3PA2. Can run based
                                                     on HF or LF clock. */
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2) || defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_CCG6SF))
    FILTER_ID_LSCSA_SCP = 3,                    /**< SCP filter for CCG3PA and CCG3PA2. Can run based on HF or LF
                                                     clock. */
    FILTER_ID_CCG6_SCP  = 3,                    /**< SCP filter for CCG6. Uses internal PD block generated clock. */
    FILTER_ID_LSCSA_OCP = 4,                    /**< OCP filter for CCG3PA. Can run based on HF or LF clock. */
    FILTER_ID_CSA_OCP   = 4,                    /**< OCP filter for CCG6. Runs based on HF clock. */
    FILTER_ID_LSCSA_PFC = 5,                    /**< PFC filter for CCG3PA. Can run based on HF or LF clock. */
    FILTER_ID_LSCSA_SR  = 6,                    /**< SR filter for CCG3PA. Can run based on HF or LF clock. */
    FILTER_ID_CCG6_RCP  = 6,                    /**< RCP filter for CCG6. Uses internal PD block generated clock. */
#elif defined(CY_DEVICE_PAG1S)
    FILTER_ID_LSCSA_OCP = 3,                    /**< OCP filter for PAG1S. Can run based on HF or LF clock. */
    FILTER_ID_LSCSA_PFC = 4,                    /**< PFC filter for PAG1S. Can run based on HF or LF clock. */
    FILTER_ID_LSCSA_SR  = 5,                    /**< SR filter for PAG1S. Can run based on HF or LF clock. */
    FILTER_ID_VSRC_NEW_P = 8,                   /**< VSRC_NEW_P filter for PAG1S. Can run based on HF or LF clock. */
    FILTER_ID_VSRC_NEW_M = 9,                   /**< VSRC_NEW_M filter for PAG1S. Can run based on HF or LF clock. */
    FILTER_ID_PDS_SCP = 10,                     /**< Virtual number for SCP as it is to be configured differently. */
    FILTER_ID_FF_OV = 11,                       /**< Virtual number for FF_OV as it is to be configured differently. */
#else /* CY_DEVICE_CCG7D || CY_DEVICE_CCG7S */
    FILTER_ID_LSCSA_SR  = 3,                    /**< CBL_0, SR filter for CCG7D. Can run based on HF or LF clock. */
    FILTER_ID_LSCSA_PFC = 4,                    /**< CBL_1, PFC filter for CCG7D. Can run based on HF or LF clock. */
    FILTER_ID_VSRC_NEW_P = 5,                   /**< VSRC_NEW_P filter for CCG7D. Can run based on LF clock. */
    FILTER_ID_VSRC_NEW_M = 6,                   /**< VSRC_NEW_M filter for CCG7D. Can run based on LF clock. */
    FILTER_ID_CSA_SCP = 10,                     /**< Virtual number for SCP as it is to be configured differently. */
    FILTER_ID_CSA_OCP = 11,                     /**< Virtual number for OCP as it is to be configured differently. */
    FILTER_ID_PDS_SCP = 12,                     /**< Virtual number for VBAT_GND SCP as it is to be configured differently. */
    FILTER_ID_VIN_UVP = 13,                     /**< Virtual number for VIN UVP as it is to be configured differently. */
    FILTER_ID_VIN_OVP = 14,                     /**< Virtual number for VIN OVP as it is to be configured differently. */
    FILTER_ID_ILIM_DET = 15,                    /**< Virtual number for ILIM DET as it is to be configured differently. */
    FILTER_ID_VREG_INRUSH_DET = 16,             /**< Virtual number for VREG INRUSH DET as it is to be configured differently. */
    FILTER_ID_BROWN_OUT_DET = 17,               /**< Virtual number for VDDD brown out as it is to be configured differently. */
#endif /* CCGx */
    FILTER_ID_MAX                               /**< Number of supported filters. */
} filter_id_t;

/**
 * @typedef filter_edge_detect_cfg_t
 * @brief List of edge triggered interrupt based on the filter output.
 */
typedef enum
{
    FILTER_CFG_POS_DIS_NEG_DIS = 0u,                 /**< Interrupt disabled. */
    FILTER_CFG_POS_DIS_NEG_EN,                  /**< Negative edge detection interrupt. */
    FILTER_CFG_POS_EN_NEG_DIS,                  /**< Positive edge detection interrupt. */
    FILTER_CFG_POS_EN_NEG_EN,                   /**< Both edge detection interrupt. */
    FILTER_CFG_MAX                              /**< Invalid interrupt configuration. */
} filter_edge_detect_cfg_t;

/**
 * @typedef dpdm_mux_cfg_t
 * @brief List of possible settings for the DP/DM MUX on the CCG5x and CCG6x devices.
 */
typedef enum
{
    DPDM_MUX_CONN_NONE         = 0x00,          /**< No connections enabled through the DPDM Mux. */
    DPDM_MUX_CONN_USB_TOP      = 0x11,          /**< Connect D+/D- to D+/D- on the top side of the connector. */
    DPDM_MUX_CONN_UART_TOP     = 0x22,          /**< Connect UART TX/RX to D+/D- on the top side of the connector. */
    DPDM_MUX_CONN_USB_BOT      = 0x44,          /**< Connect D+/D- to D+/D- on the bottom side of the connector. */
    DPDM_MUX_CONN_UART_BOT     = 0x88,          /**< Connect UART TX/RX to D+/D- on the bottom side of the connector. */
    DPDM_MUX_CONN_USB_TOP_UART = 0x99,          /**< Connect D+/D- to top and UART TX/RX to bottom side. */
    DPDM_MUX_CONN_USB_BOT_UART = 0x66           /**< Connect D+/D- to bottom and UART TX/RX to top side. */
} dpdm_mux_cfg_t;

/**
 * @typedef ccg_refgen_op_t
 * @brief List of refgen outputs on the CCG6 device.
 */
typedef enum
{
    CCG6_VREF_VBUS_UV = 2,              /**< Vref used for UVP on CCG6. */
    CCG6_VREF_VBUS_OV = 3,              /**< Vref used for OVP on CCG6. */
    CCG6_VREF_CSA_OCP = 5,              /**< Vref used for CSA OCP on CCG6. */
    CCG6_VREF_CSA_SCP = 7,              /**< Vref used for CSA SCP on CCG6. */
    CCG6_VREF_RCP_CSA = 8,              /**< Vref used for Reverse current sensing on CCG6. */
    CCG6_VREF_RCP_OVP = 10,             /**< Vref used for RCP based on VBus over-voltage on CCG6. */

    CCG6DF_VREF_VBUS_UV = 2,            /**< Vref used for UVP on CCG6DF. */
    CCG6DF_VREF_VBUS_OV = 3,            /**< Vref used for OVP on CCG6DF. */
    CCG6DF_VREF_CSA_OCP = 5,            /**< Vref used for CSA OCP on CCG6DF. */
    CCG6DF_VREF_CSA_SCP = 7,            /**< Vref used for CSA SCP on CCG6DF. */
    CCG6DF_VREF_RCP_CSA = 8,            /**< Vref used for Reverse current sensing on CCG6DF. */
    CCG6DF_VREF_RCP_OVP = 10            /**< Vref used for RCP based on VBus over-voltage on CCG6DF. */
} ccg_refgen_op_t;

/**
 * @typedef pasc_mode_t
 * @brief List of secondary control PWM operation modes.
 */
typedef enum
{
    PASC_MODE_FF,                       /**< PWM operation in fixed frequency mode. */
    PASC_MODE_PFF,                      /**< PWM operation in pseudo fixed frequency mode. */
    PASC_MODE_QR                        /**< PWM operation in Quasi Resonant frequency mode. */

} pasc_mode_t;

#if FW_FAULT_TRIGGER_ENABLE
typedef enum
{
    FAULT_TRIG_WDT_RESET = 0u,
    FAULT_TRIG_HARD_FAULT,
    FAULT_TRIG_CONFIG_ERR,
    FAULT_TRIG_POWER_CYCLE,
    FAULT_TRIG_VREG_INRUSH_FAULT,
    FAULT_TRIG_VDDD_BOD_FAULT,
    TOTAL_CMN_FAULTS 
} cmn_fault_id_t;

typedef enum 
{
    FAULT_TRIG_VBUS_UV = 0u,
    FAULT_TRIG_VBUS_OV,
    FAULT_TRIG_VBUS_OC,
    FAULT_TRIG_VBUS_SC,
    FAULT_TRIG_VBUS_RC,
    FAULT_TRIG_VBUS_IN_UV,
    FAULT_TRIG_VBUS_IN_OV,
    FAULT_TRIG_VCONN_OC,
    FAULT_TRIG_SYSTEM_OT,
    FAULT_TRIG_CRC_ERR,
    FAULT_TRIG_CC_OV,
    FAULT_TRIG_CC_SC,
    FAULT_TRIG_SBU_OV,
    FAULT_TRIG_ILIM_FAULT,
    FAULT_TRIG_VBAT_GND_SCP,
    FAULT_TRIG_VIN_UV,
    FAULT_TRIG_VIN_OV,
    TOTAL_PORT_FAULTS
} port_fault_id_t;
#endif /* FW_FAULT_TRIGGER_ENABLE */
/** \} group_ccgxAppCommon_enums */
#if 0
/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/
/** \addtogroup group_ccgxAppCommon_data_structures
* \{
*/
/**
 * @brief The structure contains the valley table to be used to implement
 * the valley algorithm for PAG1S based power adapter solution. The table
 * contains the frequency selection parameters at various voltage and 
 * load conditions.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
typedef struct
{
    uint16_t v0;                        /**< Voltage level V0 corresponding to first index in table. */
    uint16_t v1;                        /**< Voltage level V1 corresponding to second index in table.
                                             V1 >= V0. Paramter validation is not done and is expected
                                             to be loaded with valid values only. */
    uint16_t v2;                        /**< Voltage level V2 corresponding to third index in table.
                                             V2 >= V1. Paramter validation is not done and is expected
                                             to be loaded with valid values only. */
    uint16_t max_cur;                   /**< Maximum current supported by this design in 10mA units.
                                             The 10 entries in the valley table corresponds to each
                                             10% increment on this current rating. */
    uint8_t safe_valley;                /**< Safe (highest) valley threshold which can deliver maximum 
                                             load without affecting performance. [1-64]. Loading invalid
                                             valley number can result in unstable / undefined behaviour. */
    uint8_t reserved_0[3];              /**< Reserved byte to maintain alignment of the table. */
    uint8_t low_line_table[4][10];      /**< Two dimensional valley number table for high input voltage line
                                             which contains four entries matching each voltage thresholds.
                                             The last array contains any higher voltage above v2. 
                                             v0 <= v1 <= v2.
                                             The 10 entries in each array corresponds to 10%, 20%
                                             load current. Valid valley number is between [1-64]. 
                                             Parameter validation is not done and is expected to be 
                                             loaded with valid values only. 
                                             Valley number should be decrease as load increases. */
    uint8_t high_line_table[4][10];     /**< Two dimensional valley number table for high input voltage line
                                             which contains four entries matching each voltage thresholds.
                                             The last array contains any higher voltage above v2. 
                                             v0 <= v1 <= v2.
                                             The 10 entries in each array corresponds to 10%, 20%
                                             load current. Valid valley number is between [1-64]. 
                                             Parameter validation is not done and is expected to be 
                                             loaded with valid values only. 
                                             Valley number should be decrease as load increases. */
    uint8_t update_interval;            /**< Valley update interval in units of 1ms. */
} pasc_valley_table_t;
/** \} group_ccgxAppCommon_data_structures */
/**
 * @typedef PD_ADC_CB_T
 * @brief PD ADC comparator interrupt callback type. This callback
 * is called when the desired ADC event/interrupt occurs.
 * Available events:
 * - true : Input voltage is higher than the reference.
 * - false : Input voltage is lower than the reference.
 *
 * @param port PD port on which the ADC event occured.
 * @param comp_out Specifies the type of event.
 */
typedef void (*PD_ADC_CB_T)(uint8_t port, bool comp_out);

/**
 * @typedef pd_phy_cbk_t
 * @brief PD PHY callback prototype. This function will be used to notify
 * the stack about PHY events.
 *
 * @param port PD port on which the PHY event occured.
 * @param event Type of PD PHY event.
 */
typedef void(*pd_phy_cbk_t)(uint8_t port, pd_phy_evt_t event);

/**
 * @typedef vbus_cf_cbk_t
 * @brief VBus current foldback callback function.
 *
 * @param port PD port on which the event occured.
 * @param cf_state Whether in CF mode or not.
 */
typedef void (*vbus_cf_cbk_t)(uint8_t port, bool cf_state);

/**
 * @typedef pd_cmp_cbk_t
 * @brief Comparator interrupt callback function.
 *
 * @param port PD port on which the event occured.
 * @param state State of the comparator.
 */
typedef void (*pd_cmp_cbk_t)(uint8_t port, bool state);

/**
 * @typedef pd_supply_change_cbk_t
 * @brief Callback function used to provide notification about input supply changes.
 *
 * @param port PD port associated with the supply.
 * @param supply_id ID of the supply on which change is detected.
 * @param present Whether the identified supply is now present (true) or absent (false).
 */
typedef void (*pd_supply_change_cbk_t)(uint8_t port, ccg_supply_t supply_id, bool present);

/**
 * @typedef pasc_valley_cbk_t
 * @brief Callback function used to provide notification on valley change completion.
 *
 * @param port PD port associated with the supply.
 * @param status Status of the transition whether it completed successfully or not.
 */
typedef void (*pasc_valley_cbk_t)(uint8_t port, ccg_status_t status);

/**
 * @typedef vbus_load_chg_cbk_t
 * @brief Callback function used to provide notification on VBUS load change.
 *
 * @param port PD port associated with the supply.
 *
 * <B>Applicable devices:</B> PAG1S
 */
typedef void (*vbus_load_chg_cbk_t)(uint8_t port);

/**
 * @typedef pasc_ptdrv_cont_cbk_t
 * @brief Callback function used to provide notification on PTDRV contention.
 *
 * @param port PD port associated with the supply.
 *
 * <B>Applicable devices:</B> PAG1S
 */
typedef void (*pasc_ptdrv_cont_cbk_t)(uint8_t port);


/**
 * @typedef pasc_cal_fail_cbk_t
 * @brief Callback function that allows application to handle calibration failure.
 *
 * @param port PD port associated with the supply.
 *
 * <B>Applicable devices:</B> PAG1S
 */
typedef void (*pasc_cal_fail_cbk_t)(uint8_t port);

#if PASC_PTDRV_IN_VAR_WIDTH_ENABLE
/**
 * @typedef pasc_ptdrv_in_var_width_cbk_t
 * @brief Callback function to provide notification on
 * PTDRV_IN max width interrupt.
 *
 * @param port PD port associated with the supply.
 *
 * <B>Applicable devices:</B> PAG1S
 */
typedef void (*pasc_ptdrv_in_var_width_cbk_t)(uint8_t port);
#endif /* PASC_PTDRV_IN_VAR_WIDTH_ENABLE */

#if PASC_FF_OV_ENABLE
/**
 * @typedef pasc_ptdrv_in_var_width_cbk_t
 * @brief Callback function to provide notification on
 * feedforward overvoltage (FF_OV).
 *
 * @param port PD port associated with the supply.
 *
 * <B>Applicable devices:</B> PAG1S
 */
typedef void (*pasc_ff_ov_cbk_t)(uint8_t port);
#endif /* PASC_FF_OV_ENABLE */

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/
/**
 * @brief This function initializes the PDSS IP with necessary clock and interrupt
 * handlers.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_hal_init(uint8_t port);

/**
 * @brief This function configures the PD block for deepsleep entry.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if the port is not busy and has been configured to go to
 * deepsleep, false otherwise. Also returns true if the block was not enabled.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_phy_deepsleep(uint8_t port);

/**
 * @brief This function configures all PD blocks supported by the device
 * after deepsleep exit.
 * @return Returns true if successful, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_phy_wakeup(void);

/**
 * @brief This function cleans up the PD block after a disconnect.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_hal_cleanup(uint8_t port);

/**
 * @brief Register a callback that can be used for notification of power supply changes.
 * @param cb Callback function pointer.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6.
 */
void pd_hal_set_supply_change_evt_cb(pd_supply_change_cbk_t cb);

/**
 * @brief This function initializes the PD phy registers and registers a callback
 * (provided by the PD protocol state machine) which will be called when interrupts
 * associated with the PD transceiver are received.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param cbk The phy event handler callback.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_phy_init(uint8_t port, pd_phy_cbk_t cbk);

/**
 * @brief This function configures the PD phy based on the current power role, data
 * role and contract status of the specified port. This API does not enable the receiver.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_phy_refresh_roles(uint8_t port);

/**
 * @brief This function enables transmission of unchunked extended messages.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_phy_en_unchunked_tx(uint8_t port);

/**
 * @brief This function disables transmission of unchunked extended messages.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_phy_dis_unchunked_tx(uint8_t port);

/**
 * @brief This function loads the PD message in FIFO and configures the
 * necessary registers in preparation to sending the message out. The actual
 * message sending has to be triggered by calling pd_phy_send_msg().
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param sop Sop type.
 * @param retries Number of retries.
 * @param dobj_count No of data objects(each 32 bit) in data
 * @param header PD Header in lower 16 bits and optional unchunked extended header in upper 16 bits.
 * @param unchunked Unchunked message if true.
 * @param buf Pointer to message. Message buffer is a DWORD array that holds the data associated with
 * data and extended data messages.
 * @return Returns true if successful, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_phy_load_msg(uint8_t port, cy_en_pd_sop_t sop, uint8_t retries,
        uint8_t dobj_count, uint32_t header, bool unchunked, uint32_t* buf);

/**
 * @brief This function starts the transmission of a message already loaded in FIFO.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if successful, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_phy_send_msg(uint8_t port);

/**
 * @brief Reset the PD receive and transmit state machines.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_phy_reset_rx_tx_sm(uint8_t port);

/**
 * @brief This function starts transmission of BIST Carrier Mode 2. Once this API
 * returns successfully, the PHY continues to send the CM2 pattern until
 * pd_phy_abort_bist_cm2() is called.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_phy_send_bist_cm2(uint8_t port);

/**
 * @brief This function stops transmission of BIST Carrier Mode 2.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_phy_abort_bist_cm2(uint8_t port);

/**
 * @brief This function stops transmission of any ongoing PD message.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_phy_abort_tx_msg(uint8_t port);

/**
 * @brief This function starts transmission of a cable reset or a hard reset
 * as requested.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param sop SOP type indicating Cable Reset or Hard Reset.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_phy_send_reset(uint8_t port, cy_en_pd_sop_t sop);

/**
 * @brief This function returns the received packet. Since the interrupt handlers
 * uses the same buffer to receive all incoming messages, the caller of this
 * function needs to copy the data out before a new message is received.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Pointer to the received PD packet.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
cy_stc_pd_packet_extd_t* pd_phy_get_rx_packet(uint8_t port);

/**
 * @brief This function checks if the PD phy is busy for the specified port.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if the phy is busy, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_phy_is_busy(uint8_t port);

/**
 * @brief This function initializes the Type-C registers in the PD block.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_typec_init(uint8_t port);

/**
 * @brief This function starts the Type C line comparators. pdss_typec_init()
 * should have been called before calls to this function.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_typec_start(uint8_t port);

/**
 * @brief: This function stops the Type-C line comparators.
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return: ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_typec_stop(uint8_t port);

/**
 * @brief This function configures and enables Rp termination on the specified CC line.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @param rp_val Rp value.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_en_rp(uint8_t port, uint8_t channel, cy_en_pd_rp_term_t rp_val);

/**
 * @brief This function disables Rp termination on the specified CC line.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_dis_rp(uint8_t port, uint8_t channel);

/**
 * @brief This function enables a resistive Rp termination (not accurate to match
 * any Type-C current levels) which can be used to save power while there is no
 * Type-C connection and the CCG device is in sleep.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_en_dpslp_rp(uint8_t port);

/**
 * @brief This function disables the resistive Rp termination so that the
 * accurate current source based Rp termination can be applied.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_dis_dpslp_rp(uint8_t port);

/**
 * @brief This function enables Rd termination on the specified CC line.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_en_rd(uint8_t port, uint8_t channel);

/**
 * @brief This function enables the Rd termination without initializing the block completely.
 * This is used in some Type-C fault handling use cases.
 * @param port Port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_rd_enable(uint8_t port);

/**
 * @brief This function disables Rd termination on the specified CC line.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_dis_rd(uint8_t port, uint8_t channel);

/**
 * @brief Enable Dead Battery Rd on the specified PD port.
 * This function is used to remove the trimmed Rd and re-enable the
 * dead-battery Rd on the PD port prior to going through a device reset.
 * This method is only used in dead-battery use cases, and allows device
 * flashing while the CCG device is powered through the Type-C connection.
 *
 * @param port Port on which dead-battery Rd is to be enabled.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6
 * devices which support the dead-battery Rd (Rd-db) termination.
 */
void pd_typec_en_deadbat_rd(uint8_t port);

/**
 * @brief This function updates the tx trim settings when in the sink role. It
 * must be called whenever an Rp change on the port partner side is detected.
 * This is required to meet the CC transmitter characteristics defined by the
 * USB-PD specification.
 * @param port Port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
void pd_typec_snk_update_trim(uint8_t port);

/**
 * @brief This function returns current status of both CC lines. The function
 * identifies the current termination (Rp or Rd) applied by the CCG device and
 * interprets the voltage on the CC line based on the current use case.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return cc_state_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
cy_pd_cc_state_t pd_typec_get_cc_status(uint8_t port);

/**
 * @brief This function sets the CC polarity for the receiver circuit.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param polarity Type-C connection orientation. false when CC1 active and true
 * when CC2 active.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_typec_set_polarity(uint8_t port, bool polarity);

/**
 * @brief Check whether the 5V supply to provide VConn power is present.
 * @param port Port index.
 * @return true if 5V supply is present.
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6.
 */
bool pd_is_v5v_supply_on(uint8_t port);

/**
 * @brief This function turns on Vconn for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
ccg_status_t pd_vconn_enable(uint8_t port, uint8_t channel);

/**
 * @brief This function turns off Vconn for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
ccg_status_t pd_vconn_disable(uint8_t port, uint8_t channel);

/**
 * @brief Enable Over-Current detection on the VConn power source.
 * @param port PD port to be configured.
 * @param debounce Debounce period in milliseconds.
 * @param cb Callback function to be called on fault detection.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
void pd_hal_vconn_ocp_enable(uint8_t port, uint8_t debounce, PD_ADC_CB_T cb);

/**
 * @brief Disable Over-Current detection on the VConn power source.
 * @param port PD port to be configured.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
void pd_hal_vconn_ocp_disable(uint8_t port);

/**
 * @brief Enable Short-Circuit detection on the VConn power source.
 * @param port PD port to be configured.
 * @param cb Callback function to be called on fault detection.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_hal_vconn_scp_enable(uint8_t port, PD_ADC_CB_T cb);

/**
 * @brief Disable Short-Circuit detection on the VConn power source.
 * @param port PD port to be configured.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_hal_vconn_scp_disable(uint8_t port);

/**
 * @brief This function gets Vconn status for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return Returns true if Vconn is turned on, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
bool pd_is_vconn_present(uint8_t port, uint8_t channel);

/**
 * @brief This function converts the voltage in millivolt to ADC units.
 *
 * It takes a 16-bit voltage value in millivolts and returns the
 * corresponding 8-bit ADC reading. This function does not perform any ADC
 * operations.
 *
 * The minimum value is limited by the PD_ADC_LEVEL_MIN_THRESHOLD value.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param volt Voltage in mV.
 * @return Returns the 8-bit ADC reading corresponding to volt.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint8_t pd_adc_volt_to_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt);

/**
 * @brief This function converts the ADC units to voltage in millivolts.
 *
 * It takes an 8-bit ADC reading and returns the corresponding 16-bit voltage
 * value in millivolts. This function does not perform any ADC operations.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param level The 8-bit ADC reading.
 * @return Returns voltage in mV.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint16_t pd_adc_level_to_volt(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level);

/**
 * @brief This function converts the ADC level to VBAT voltage in millivolts.
 * It takes an 8-bit ADC reading and returns the corresponding 16-bit voltage
 * value in millivolts. This function does not perform any ADC operations.
 * pd_adc_sample should be called to convert a divided version of VBAT to
 * ADC levels before calling this function.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param level The 8-bit ADC reading.
 * @return Returns voltage in mV.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
uint16_t pd_adc_get_vbat_voltage(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level);

/**
 * @brief This function converts the ADC level to VBUS voltage in millivolts.
 * It takes an 8-bit ADC reading and returns the corresponding 16-bit voltage
 * value in millivolts. This function does not perform any ADC operations.
 * pd_adc_sample should be called to convert a divided version of VBus to
 * ADC levels before calling this function.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param level The 8-bit ADC reading.
 * @return Returns voltage in mV.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint16_t pd_adc_get_vbus_voltage(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level);

/**
 * @brief This function configures the ADC for comparator functionality with
 * the requested threshold with no interrupts enabled.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @param level Comparator level.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t  pd_adc_free_run_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level);

/**
 * @brief This function configures the ADC for comparator functionality with
 * the requested threshold, and registers a callback which shall be called
 * when the comparator output changes.
 *
 * This function configures the ADC block as a comparator. The function takes
 * the input to be configured and the ADC comparator threshold. It also takes a
 * callback. If the callback is not NULL, then the threshold is configured and
 * interrupts are enabled. If the callback is NULL, then the ADC / comparator
 * is set to the low power state and interrupts are disabled.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @param level Comparator level.
 * @param int_cfg Interrupt configuration.
 * @param cb Callback to be called on interrupt.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_adc_comparator_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level, PD_ADC_INT_T int_cfg, PD_ADC_CB_T cb);

/**
 * @brief This function temporarily configures the comparator as requested and
 * checks whether the input exceeds the specified digital level.
 *
 * This function restores the comparator to its previous state after operation. This
 * is useful when the comparator is already configured to function with a certain
 * input and level with interrupt and another reading needs to be done without
 * having to re-configure the block after the sampling.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @param level Value to compare the input voltage against.
 * @return Returns true if voltage > level, false otherwise.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_adc_comparator_sample(uint8_t port, PD_ADC_ID_T adc_id,
        PD_ADC_INPUT_T input, uint8_t level);

/**
 * @brief This function gets the current comparator status.
 *
 * This function does not configure the ADC / comparator. It just returns the
 * current state of the comparator. If true is returned, then the input voltage
 * is greater than the reference and if false, the input voltage is lower than
 * the reference.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return Returns the comparator output.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
bool pd_adc_get_comparator_status(uint8_t port, PD_ADC_ID_T adc_id);

/**
 * @brief This function samples the ADC.
 *
 * This function enables the ADC block to function as an ADC and returns the
 * sample value in ADC units. This function disables any previously configured
 * comparator interrupts / settings before sampling and restores them after the
 * sampling is done. If any interrupt scenario happens across the sampling, the
 * information is lost.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @return Returns the ADC sample.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint8_t pd_adc_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input);

 /**
 * @brief This function samples the ADC using VDDD (3.3V) as Vref.
 *
 * This function enables the ADC block to function as an ADC and returns the
 * sample value in ADC units. This function disables any previously configured
 * comparator interrupts / settings before sampling and restores them after the
 * sampling is done. If any interrupt scenario happens across the sampling, the
 * information is lost.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @return Returns the ADC sample.
 */
uint8_t pd_adc_sample_ref_vddd(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input);

/**
 * @brief This function calibrates the specified ADC for operation.
 *
 * This function calibrates the specified ADC by identifying the VDDD voltage
 * for reference. It should be noted that by calling the function, the
 * previously calculated threshold levels may have to be changed based on the
 * VDDD reading. The VDDD level is calculated based on the bandgap voltage
 * which is expected to be constant.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return Returns the VDDD value in mV after calibration.
 *
 * <B>Applicable devices:</B> CCG3, CCG4.
 */
uint16_t pd_adc_calibrate(uint8_t port, PD_ADC_ID_T adc_id);

/**
 * @brief This function initializes the PD ADC block.
 *
 * This function enables the PD block and the registers required for ADC
 * operation. It then calibrates the ADC to identify the VDDD voltage. This
 * function does not start any ADC operations.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_adc_init(uint8_t port, PD_ADC_ID_T adc_id);

#if 0
/**
 * @brief This function selects the reference voltage used by the ADC block
 * on the CCG device. A 2.0 V supply generated by the RefGen block in the PD
 * IP is used as ADC reference by default. This API can be used to select
 * the VDDD supply as the ADC reference. This is useful in cases where voltages
 * greater than 2.0 V need to be measured.
 *
 * Note: Since the VDDD supply level could vary across time, the ADC volts
 * per division value needs to be calibrated before taking any ADC readings.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param vref_sel ADC reference selection.
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
ccg_status_t pd_adc_select_vref(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_VREF_T vref_sel);
#endif

/**
 * @brief This function gets the ADC level that corresponds to the actual
 * voltage on vbus. It also takes into account the VBus monitor divider.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param volt Voltage in 50mV units.
 * @param per Percentage margin on the voltage.
 * @return Returns the ADC level.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint8_t pd_get_vbus_adc_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt,
        int8_t per);

/**
 * @brief Enable connection of the internal VBus divider to AMUX bus.
 * In case of PAG1S, this function call connects VBUS_DIV to ADC AMUX.
 * Caller is expected to ensure that the ADC is not configured for
 * interrupt using VBUS_MON.
 * @param port PD port to be updated.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, PAG1S.
 */
void pd_connect_vbus_div_to_amux(uint8_t port);

/**
 * @brief Disconnect the internal VBus divider from AMUX bus.
 * In case of PAG1S, this function call disconnects VBUS_DIV from ADC AMUX,
 * and allows GLOBAL AMUX connection to ADC. Caller is expected to
 * ensure that the ADC is not configured for interrupt using VBUS_MON.
 * @param port PD port to be updated.
 * @return true if disconnected, false if disconnection not allowed.
 *
 * <B>Applicable devices:</B> CCG3, PAG1S.
 */
bool pd_disconnect_vbus_div_from_amux(uint8_t port);

/**
 * @brief Enable/disable automatic hardware toggle operation as part of deep sleep cycle.
 * @param port PD port to be updated.
 * @param enable Whether automatic toggle is to be enabled.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
void pd_hal_config_auto_toggle(uint8_t port, bool enable);

/**
 * @brief Check whether automatic DRP toggle operation is active.
 * @param port PD port to be checked.
 * @return Current status of automatic DRP toggle operation.
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
bool pd_hal_is_auto_toggle_active(uint8_t port);

/**
 * @brief Abort any ongoing automatic DRP toggle operation.
 * @param port Port on which toggle is to be aborted.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
void pd_hal_abort_auto_toggle(uint8_t port);

/**
 * @brief Restart Type-C state machine once auto toggle operation is complete.
 * This is normally required when the port has just exited the auto DRP toggle
 * stage, and the state machine needs to handle further operations.
 * @param port Port on which state machine restart is required.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_hal_typec_sm_restart(uint8_t port);

/**
 * @brief Facilitates to set deep sleep/bandgap reference.
 * Function should be called for setting deep sleep reference just before
 * entry into deep sleep state. Function should be called with band gap
 * refernce just after exit from deep sleep state.
 * @param port PD port to change reference.
 * @param flag If true signifies deep sleep reference, else band gap
 * @return None.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2.
 */
void pd_hal_set_reference(uint8_t port, bool flag);

/**
 * @brief Measure the current voltage on the VBus-in.
 * @param port PD port to be measured.
 * @return VBus-in voltage in mV units.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint16_t pd_hal_measure_vbus_in(uint8_t port);

/**
 * @brief Measure the current voltage on the VBus supply.
 * @param port PD port to be measured.
 * @return VBus voltage in mV units.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
uint16_t pd_hal_measure_vbus(uint8_t port);

/**
 * @brief Measure the current currently being provided through the VBus supply.
 *
 * @param port PD port on which current is to be measured.
 * @return VBus active current in 10 mA units.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6, PAG1S.
 */
uint16_t pd_hal_measure_vbus_cur(uint8_t port);

/**
 * @brief This function enables the fast role swap receive functionality. Callback
 * registered in pd_phy_init will be called when fast role swap signal is received.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success, otherwise returns false
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
bool pd_frs_rx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap receive functionality.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG5C, CCG6.
 */
bool pd_frs_rx_disable(uint8_t port);

/**
 * @brief This function enables the fast role swap transmit functionality. Callback
 * registered in pd_phy_init will be called when fast role swap signal is transmitted.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 *
 * <B>Applicable devices:</B> CCG3.
 */
bool pd_frs_tx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap transmit functionality.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 *
 * <B>Applicable devices:</B> CCG3.
 */
bool pd_frs_tx_disable(uint8_t port);

/**
 * @brief Configures the drive modes for the FETs.
 *
 * This function allows the application to select polarity of drive for CCG4
 * devices and N-FET drive or P-FET drive for CCG3 devices. The configuration
 * should match the hardware implementation on the board.
 *
 * CCG3 devices support N-FET by default for both PCTRL and CCTRL. Override this
 * only for the boards with P-FETs. Standard Cypress reference schematics
 * and kits use N-FETs and changing this shall result in board damage.
 *
 * CCG4 devices support active high polarity for PCTRL (source) and active
 * low polarity for CCTRL (sink) by default. This configuration matches the
 * standard Cypress reference schematics and kits and changing this shall result
 * in board damage.
 *
 * This function is expected to be called once, before the pdss_hal is initialized.
 * Calls during stack operation are not restricted but are likely
 * to result in spurious behaviour and / or board damage.
 *
 * @param pctrl_drive Select the type of gate driver used for the Provider FET.
 * @param cctrl_drive Select the type of gate driver used for the Consumer FET.
 * @return None
 *
 * @warning Misconfiguration of this parameter will result in board damage.
 *
 * <B>Applicable devices:</B> CCG3, CCG4.
 */
void pd_hal_set_fet_drive(pd_fet_dr_t pctrl_drive, pd_fet_dr_t cctrl_drive);

/**
 * @brief Configures FET parameters for CCG3 devices.
 *
 * This function configures various FET control parameters for the device.
 * It should be called before the PD HAL is intialized. Also it should be called
 * only once during initialization. Since this is high voltage FET configuration,
 * any wrong configuration shall result in damage of the device and boards. The
 * function should be called only when the default behaviour needs to be
 * overridden.
 *
 * Caller should ensure that the FET access is not done before it is configured
 * and is never configured while the PD stack is in operation.
 *
 * CCG3 devices shall use dual FET configuration with spacing of 10LF cycles
 * by default. If this function is not invoked, then the default configuration
 * shall be used. CCG3 shall always function in dual FET mode as the FET controls
 * are dedicated IOs.
 *
 * @param dual_fet Set to true if the system uses dual FETs for each direction.
 *      Otherwise set to false.
 * @param spacing Spacing in LF-cycles between dual FETs for firmware based
 *      turn-on and turn-off. Auto shut-off and turn-on happens simultaneously.
 *      Valid only for dual_fet configuration. Set to zero for simultaneous
 *      control.
 * @return None
 *
 * @warning Misconfiguration of this parameter shall result in board damage.
 *
 * <B>Applicable devices:</B> CCG3.
 */
void pd_hal_dual_fet_config(bool dual_fet, uint8_t spacing);

/**
 * @brief Resets the gate driver edge detector to clear any fault state.
 * @param port Port index.
 * @param pgdo_type Flag indicating the gate driver to be cleared, true for P_CTRL
 * and false for C_CTRL.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_reset_edge_det(uint8_t port, bool pgdo_type);

/**
 * @brief Removes internal feeedback resistor divider.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2.
 */
void pd_remove_internal_fb_res_div(void);

/**
 * @brief Turn on producer FET using the internal gate driver or dedicated GPIO.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_on_seq On CCG3, this bit selects which FET turns on first.
 *  - 0: FET 0 turns on first.
 *  - 1: FET 1 turns on first.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_pfet_on(uint8_t port, bool turn_on_seq);

/**
 * @brief Turn off producer FET using the internal gate driver.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_off_seq On CCG3, this bit selects which FET turns off first.
 *  - 0: FET 0 turns off first.
 *  - 1: FET 1 turns off first.
 * @return Void.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_pfet_off(uint8_t port, bool turn_off_seq);

/**
 * @brief Turn on consumer FET using the internal gate driver.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_on_seq On CCG3, this bit selects which FET turns on first.
 *  - 0: FET 0 turns on first.
 *  - 1: FET 1 turns on first.
 * @return Void.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_cfet_on(uint8_t port, bool turn_on_seq);

/**
 * @brief Turn off consumer FET using the internal gate driver.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_off_seq On CCG3, this bit selects which FET turns off first.
 *  - 0: FET 0 turns off first.
 *  - 1: FET 1 turns off first.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_cfet_off(uint8_t port, bool turn_off_seq);

/**
 * @brief Check whether the Sink FET has been turned on.
 * @param port PD port index.
 * @return true if the sink path is on to allow system to charge.
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6
 */
bool pd_hal_is_sink_fet_on(uint8_t port);

/**
 * @brief Turn on the internal VBus discharge path.
 * @param port Port on which to enable VBus discharge.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_discharge_on(uint8_t port);

/**
 * @brief Turn off the internal VBus discharge path.
 * @param port Port on which to enable VBus discharge.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_discharge_off(uint8_t port);

/**
 * @brief Turn on the internal VBUS_IN discharge path.
 * @param port Port on which to enable VBUS_IN discharge.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_internal_vbus_in_discharge_on(uint8_t port);

/**
 * @brief Turn off VBUS_IN discharge path.
 * @param port Port on which to enable VBUS_IN discharge.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_internal_vbus_in_discharge_off(uint8_t port);

/**
 * @brief Enable Over Current Protection (OCP) control using the internal Current Sense Amplifier.
 *
 * @param port PD port index.
 * @param av_bw Gain selection for the Current Sense Amplifier.
 * @param vref_sel Reference voltage used to detect Over-Current condition.
 * @param pctrl Type of gate driver to be turned off on fault. Should be set to true.
 * @param mode Mode of OCP fault handling.
 * @param debounce_ms OCP software debounce timeout in ms.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_ocp_en(uint8_t port, uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        uint8_t mode, uint8_t debounce_ms);

/**
 * @brief Disable Over Current Protection (OCP) control on the PD port.
 * @param port PD port index.
 * @param pctrl Type of gate driver to be turned off on fault. Should be set to true.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_ocp_dis(uint8_t port, bool pctrl);

/**
 * @brief Enable load change monitor ISR capability.
 *
 * The function allows multiplexing the internal OCP comparator to perform low to high
 * load transition on VBUS along with the regular OCP functionality.
 *
 * @param port PD port index.
 * @param cur VBUS current threshold in 10mA units to trigger ISR
 * @param filter Filter settings in filter clock cycles to be used.
 * @param cbk Callback function to be invoked when a change is detected.
 *
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_internal_vbus_load_change_isr_enable(uint8_t port, uint32_t cur, 
        uint8_t filter, vbus_load_chg_cbk_t cbk);

/**
 * @brief Enable Over Voltage Protection (OVP) control using the internal UV-OV block.
 * @param port PD port index.
 * @param volt Contract Voltage in mV units.
 * @param per Percentage margin on the contract voltage.
 * @param cb Callback to be called on fault detection.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @param mode OVP mode selection.
 * @param filter_sel The delay in filter clock cycles between the OVP detection and trigger.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_ovp_en(uint8_t port, uint16_t volt, int8_t per,
    PD_ADC_CB_T cb, bool pctrl, vbus_ovp_mode_t mode, uint8_t filter_sel);

#if PASC_FF_OV_ENABLE
/**
 * @brief Enable feedforward Over Voltage Protection (FF_OV) control using the internal feedforward block.
 * @param port PD port index.
 * @param ac_volt Line voltage threshold to generate FF_OV event.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @param mode FF_OV protection mode selection. False for hardware controlled. True for firmware controlled.
 * @param cbk Callback to be called on fault detection.
 * @return None
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_internal_ff_ov_en(uint8_t port, uint32_t ac_volt, bool pctrl, 
        bool mode, pasc_ff_ov_cbk_t cbk);

/**
 * @brief Disable feedforward Over Voltage Protection (FF_OV) control.
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @return None
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_internal_ff_ov_dis(uint8_t port, bool pctrl);
#endif /* PASC_FF_OV_ENABLE */

/**
 * @brief Disable Over Voltage Protection (OVP) control.
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_ovp_dis(uint8_t port, bool pctrl);

#if 0
/**
 * @brief Enable Under Voltage Protection (UVP) control using the internal UV-OV block.
 * UVP is only expected to be used while CCG is the power source.
 *
 * @param port PD port index.
 * @param volt Minimum contract voltage in mV units.
 * @param per Percentage of voltage drop that is acceptable.
 * @param cb Callback to be called on fault detection.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @param mode UVP handling mode selection.
 * @param filter_sel The delay in filter clock cycles between the UVP detection and trigger.
 *
 * @return Null.
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_uvp_en(uint8_t port, uint16_t volt, int8_t per, PD_ADC_CB_T cb, bool pctrl,
        vbus_uvp_mode_t mode, uint8_t filter_sel);
#endif
/**
 * @brief Disable Under Voltage Protection (UVP) control.
 *
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @return Null.
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_internal_vbus_uvp_dis(uint8_t port, bool pctrl);

/**
 * @brief Enable Short Circuit Protection (SCP) control using the internal Current
 * Sense Amplifier. SCP can only be enabled when CCG is the power source.
 *
 * @param port PD port index.
 * @param vsense The reference voltage that should be compared against the output
 * of the Current Sense Amplifier to detect SCP. This value is dependent on the
 * max. current, the sense impedance and the CSA gain.
 * @param filter_sel The delay in filter clock cycles between the SCP detection and trigger.
 * @param pctrl Flag indicating the type of gate driver to be controlled on SCP. Should
 * be set to true.
 * @param mode SCP fault handling mode selection.
 * @return Null
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6, PAG1S.
 */
void pd_internal_vbus_scp_en(uint8_t port, uint32_t vsense, uint8_t filter_sel, bool pctrl,
    uint8_t mode);

/**
 * @brief Disable Short Circuit Protection (SCP) control.
 *
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled on SCP. Should
 * be set to true.
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6, PAG1S.
 */
void pd_internal_vbus_scp_dis(uint8_t port, bool pctrl);

#if 0
/**
 * @brief Enable VIN Under Voltage Protection (UVP) control.
 * @param port PD port index.
 * @param threshold VIN UV voltage threshold in mV units.
 * @param cb Callback to be called on fault detection.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @param mode OVP mode selection.
 * @param filter_sel The delay in filter clock cycles between the fault detection and trigger.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vin_uvp_en(uint8_t port, uint16_t threshold,
    PD_ADC_CB_T cb, bool pctrl, vbus_uvp_mode_t mode, uint8_t filter_sel);
#endif

/**
 * @brief Disable VIN Under Voltage Protection (UVP) control.
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vin_uvp_dis(uint8_t port, bool pctrl);

/**
 * @brief Enable VIN Over Voltage Protection (OVP) control.
 * @param port PD port index.
 * @param threshold VIN OV voltage threshold in mV units.
 * @param per Percentage margin on the contract voltage.
 * @param cb Callback to be called on fault detection.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @param mode OVP mode selection.
 * @param filter_sel The delay in filter clock cycles between the fault detection and trigger.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vin_ovp_en(uint8_t port, uint16_t threshold,
    PD_ADC_CB_T cb, bool pctrl, vbus_ovp_mode_t mode, uint8_t filter_sel);

/**
 * @brief Disable VIN Over Voltage Protection (OVP) control.
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled, true for
 * P_CTRL and false for C_CTRL.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vin_ovp_dis(uint8_t port, bool pctrl);

/**
 * @brief Enable VBAT to GND Short Circuit Protection (SCP) control.
 *
 * @param port PD port index.
 * @param cb Callback to be called on fault detection.
 * @param pctrl Flag indicating the type of gate driver to be controlled on SCP. Should
 * be set to true.
 * @param mode SCP fault handling mode selection.
 * @return Null
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vbat_gnd_scp_en(uint8_t port, PD_ADC_CB_T cb, bool pctrl, uint8_t mode);

/**
 * @brief Disable VBAT to GND Short Circuit Protection (SCP) control.
 *
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled on SCP. Should
 * be set to true.
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vbat_gnd_scp_dis(uint8_t port, bool pctrl);

/**
 * @brief Enable Buck-Boost Inductor Current Limit (iLim) Protection.
 *
 * @param port PD port index.
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_bb_ilim_det_en(uint8_t port);

/**
 * @brief Disable Buck-Boost Inductor Current Limit (iLim) Protection.
 *
 * @param port PD port index.
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_bb_ilim_det_dis(uint8_t port);

/**
 * @brief Enable Vreg Inrush current Protection.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vreg_inrush_det_en(void);

/**
 * @brief Disable Vreg Inrush current Protection.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_vreg_inrush_det_dis(void);

/**
 * @brief Vreg inrush fault active status.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return Vreg Inrush fault active status
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool pd_internal_vreg_inrush_status(void);

/**
 * @brief Enable Vddd brown out detection handling in Firmware.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_brown_out_det_en(void);

/**
 * @brief Disable Vddd brown out protection.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return NULL
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_internal_brown_out_det_dis(void);

/**
 * @brief Vddd brown out fault active status.
 * This fault is common for the device, hence it is not port specific.
 *
 * @param NULL
 * @return Vddd BOD fault active status
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool pd_internal_brown_out_status(void);

/**
 * @brief Enable Reverse Current Protection (RCP) control using internal Current
 * Sense Amplifier. RCP can only be enabled when CCG is the power source.
 * Since RCP response needs to be fast, the function only supports an automatic
 * FET turn-off mode of handling.
 *
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled on RCP. Should
 * be set to true.
 * @param csa_det_en Enable RCP detection by measuring voltage drop between CSN and CSP pins.
 * @param cmp_det_en Enable RCP detection by measuring voltage drop between VBus and CSN pins.
 * @param ovp_det_en Enable RCP detection by measuring voltage on the CSN pin.
 * @return Null
 *
 * <B>Applicable devices:</B> CCG6.
 */
void pd_internal_vbus_rcp_en(uint8_t port, bool pctrl, uint8_t csa_det_en, uint8_t cmp_det_en, uint8_t ovp_det_en);

/**
 * @brief Disable Reverse Current Protection (RCP) control using internal Current
 * Sense Amplifier. All forms of RCP detection will be disabled.
 *
 * @param port PD port index.
 * @param pctrl Flag indicating the type of gate driver to be controlled on RCP. Should
 * be set to true.
 * @return None
 *
 * <B>Applicable devices:</B> CCG6.
 */
void pd_internal_vbus_rcp_dis(uint8_t port, bool pctrl);

/**
 * @brief Initiate the soft enable of the Provider FET to control inrush current.
 * This function is called as part of the pd_internal_pfet_on call and slowly increases
 * the gate driver pull-down current strength until the FET is fully turned ON.
 *
 * @param port PD port index.
 * @param fet_status Flag indicating whether the gate driver is already in the enabled
 * state. Used in cases where there are back-to-back pd_internal_pfet_on() calls.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
void pd_internal_pfet_soft_start_on (uint8_t port, uint8_t fet_status);

/**
 * @brief Resets the soft state state machine associated with the Provider FET
 * gate driver. This needs to be called whenever the FET is being disabled so that
 * soft start is used during the next enable sequence.
 *
 * @param port PD port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
void pd_internal_pfet_soft_start_off (uint8_t port);

/**
 * @brief Initiate the soft enable of the Consumer FET to control inrush current.
 * This function is called as part of the pd_internal_cfet_on call and slowly increases
 * the gate driver pull-down current strength until the FET is fully turned ON.
 *
 * @param port PD port index.
 * @param fet_status Flag indicating whether the gate driver is already in the enabled
 * state. Used in cases where there are back-to-back pd_internal_cfet_on() calls.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
void pd_internal_cfet_soft_start_on (uint8_t port, uint8_t fet_status);

/**
 * @brief Resets the soft state state machine associated with the Consumer FET
 * gate driver. This needs to be called whenever the FET is being disabled so that
 * soft start is used during the next enable sequence.
 *
 * @param port PD port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
void pd_internal_cfet_soft_start_off (uint8_t port);

/**
 * @brief Disable automatic hardware control on a gate driver due to a specified
 * comparator and output filter. Does not affect automatic control of the gate
 * driver by other comparators of filters.
 *
 * @param port PD port index.
 * @param pctrl true for provider FET driver, false for consumer FET driver.
 * @param filter_index Filter whose control is being disabled.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_fet_automode_disable(uint8_t port, bool pctrl, filter_id_t filter_index);

/**
 * @brief Enable automatic hardware control on a gate driver due to a specified
 * comparator and output filter. Does not affect automatic control of the gate
 * driver by other comparators of filters.
 *
 * @param port PD port index.
 * @param pctrl true for provider FET driver, false for consumer FET driver.
 * @param filter_index Filter whose control is being enabled.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_fet_automode_enable(uint8_t port, bool pctrl, filter_id_t filter_index);

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S) || defined(CCG7D))

/**
 * @brief Configure Feedback DAC control of CCG3PA device.
 *
 * @param port PD port index.
 * @param dac_value iDAC value. Negative for source and positive for sink.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
void pd_hal_set_fb_dac(uint8_t port, int16_t dac_value);

/**
 * @brief Read Feedback DAC control of CCG3PA device.
 *
 * @param port PD port index.
 * @return dac_value signed DAC value. Positive is sink and negative is source.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
int16_t pd_hal_get_fb_dac(uint8_t port);

/**
 * @brief Enable constant voltage mode.
 * 
 * @param port PD port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
void pd_hal_enable_cv(uint8_t port);

/**
 * @brief Enable constant voltage mode.
 * 
 * @param port PD port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
void pd_hal_disable_cv(uint8_t port);

/**
 * @brief Disable internal VBUS regulator.
 * 
 * @param port PD port index.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
void pd_hal_disable_vreg(uint8_t port);

#endif  /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S) || defined(CCG7D)) */

/**
 * @brief Enable/disable the internal VBus monitor function in the CCGx device.
 *
 * @param enable Whether to enable internal VBus Monitor.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4.
 */
void pd_hal_enable_internal_vbus_mon(bool enable);

/**
 * @brief Configure the SBU switch in the CCGx device to connect the SBU1/2 to pins in the
 * desired orientation.
 *
 * @param port PD port on which SBU switch is to be configured.
 * @param sbu1_state Desired SBU1 switch state.
 * @param sbu2_state Desired SBU2 switch state.
 *
 * @return CCG_STAT_SUCCESS or CCG_STAT_INVALID_ARGUMENT.
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG5C, CCG6.
 */
ccg_status_t sbu_switch_configure(uint8_t port, sbu_switch_state_t sbu1_state, sbu_switch_state_t sbu2_state);

/**
 * @brief Returns SBU1 switch state.
 *
 * @param port PD port to be queried.
 * @return SBU1 switch state
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG5C, CCG6.
 */
sbu_switch_state_t get_sbu1_switch_state(uint8_t port);

/**
 * @brief Returns SBU2 switch state.
 *
 * @param port PD port to be queried.
 * @return SBU2 switch state
 *
 * <B>Applicable devices:</B> CCG3, CCG5, CCG5C, CCG6.
 */
sbu_switch_state_t get_sbu2_switch_state(uint8_t port);

/**
 * @brief Configure AUX1 and AUX2 resistor configuration.
 *
 * @param port PD port to be configured.
 * @param aux1_config Desired AUX1 resistor configuration
 * @param aux2_config Desired AUX2 resistor configuration
 * @return None
 *
 * <B>Applicable devices:</B> CCG3.
 */
void aux_resistor_configure(uint8_t port, aux_resistor_config_t aux1_config,
        aux_resistor_config_t aux2_config);

/**
 * @brief Returns AUX1 resistor configuration.
 * @param port PD port to be queried.
 * @return AUX1 resistor config
 *
 * <B>Applicable devices:</B> CCG3.
 */
aux_resistor_config_t get_aux1_resistor_config(uint8_t port);

/**
 * @brief Returns AUX2 resistor configuration
 * @param port PD port to be queried.
 * @return AUX2 resistor config
 *
 * <B>Applicable devices:</B> CCG3.
 */
aux_resistor_config_t get_aux2_resistor_config(uint8_t port);

/**
 * @brief Enable VCONN Comparator for VCONN monitoring. This function is
 * only applicable in VConn powered UFP applications.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3.
 */
void pd_enable_vconn_comp(void);

/**
 * @brief Get current status of VCONN supply.
 * This function checks if port partner is providing VCONN and returns the status.
 * Expectation is that VCONN comparator is enabled first using pd_enable_vconn_comp
 * function before calling this function. This function is expected to be used
 * by VCONN powered accessories.
 *
 * @return true if VCONN is present, false otherwise
 *
 * <B>Applicable devices:</B> CCG3.
 */
bool pd_get_vconn_status(void);

/**
 * @brief Disconnect Ra from VCONN line
 * This function removes Ra from VCONN. This can be used by AMAs to
 * reduce power consumption.
 *
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG3PA2.
 */
void pd_disconnect_ra(void);

/**
 * @brief Register a callback for notification of CC Short Circuit (SC)
 * fault condition.
 * @param cb Callback function pointer.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void ccg_set_cc_sc_fault_cb(PD_ADC_CB_T cb);

/**
 * @brief Register a callback for notification of CC/SBU fault conditions.
 * @param cb Callback function pointer.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6, CCG7D.
 */
void ccg_set_fault_cb(PD_ADC_CB_T cb);

/* BC 1.2 specific functions for CCG5 notebook. */

/**
 * @brief Check whether CDP state machine is active.
 * @param port PD port to be checked.
 * @return True if CDP state machine is active, false otherwise.
 *
 * <B>Applicable devices:</B> CCG5.
 */
bool ccg_is_cdp_sm_busy(uint8_t port);

/**
 * @brief Enable DCP operation on the PD port.
 * @param port PD port to be updated.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5.
 */
void ccg_bc_dcp_en(uint8_t port);

/**
 * @brief Check whether the BC 1.2 port is configured as CDP. This only checks for configuration
 * and not actual operation as CDP.
 * @param port PD port to be queried.
 * @return true if the port is configured as CDP, false otherwise.
 *
 * <B>Applicable devices:</B> CCG5.
 */
bool ccg_bc_is_cdp(uint8_t port);

/**
 * @brief Enable CDP operation on the PD port. The CDP state machine
 * should be called periodically until the HAL reports state machine
 * idle.
 * @param port PD port on which CDP state machine is to be started.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5.
 */
void ccg_bc_cdp_en(uint8_t port);

/**
 * @brief Disable all BC 1.2 operation on the PD port.
 * @param port Port to be updated.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5.
 */
void ccg_bc_dis(uint8_t port);

/**
 * @brief CDP state machine function.
 * @param port PD port for CDP operation.
 * @return Returns whether CDP state machine is still active.
 *
 * <B>Applicable devices:</B> CCG5.
 */
bool ccg_bc_cdp_sm(uint8_t port);

/**
 * @brief Configure the DP/DM MUX on the CCG device as required.
 * @param port PD port to be configured.
 * @param conf Type of connection to be made.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6.
 */
void ccg_config_dp_dm_mux(uint8_t port, dpdm_mux_cfg_t conf);

/**
 * @brief Notify the HAL that an OVP condition is pending on the CC line. This
 * will update the logic used to poll CC status in the HAL.
 * @param port PD port index on which fault is pending.
 * @return None
 *
 * <B>Applicable devices:</B> CCG5, CCG5C, CCG6, CCG7D.
 */
void pd_hal_set_cc_ovp_pending(uint8_t port);

/**
 * @brief Notify the HAL that an SCP condition is pending on the CC line. This
 * will update the logic used to poll CC status in the HAL.
 * @param port PD port index on which fault is pending.
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_hal_set_cc_scp_pending(uint8_t port);

/**
 * @brief Select the comparator block and input setting used for VBus detach
 * detection. CCG will use the selected settings to detect Type-C disconnection
 * based on removal of the VBus power.
 *
 * @param adc_id Select the comparator (ADC) block to be used.
 * @param adc_inp Select the comparator input to be used.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_hal_set_vbus_detach_params(PD_ADC_ID_T adc_id, PD_ADC_INPUT_T adc_inp);

/**
 * @brief Identify the CCGx ADC that is used for VBus detach detection.
 * @return ID of the ADC block used for VBus detach detection.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
PD_ADC_ID_T pd_hal_get_vbus_detach_adc(void);

/**
 * @brief Identify the ADC input that is used for VBus detach detection.
 * @return ADC block input used for VBus detach detection.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
PD_ADC_INPUT_T pd_hal_get_vbus_detach_input(void);

/**
 * @brief Specify the voltage division ratio between the voltage applied
 * on VBUS_MON and the actual VBus voltage. The commonly used resistor divider
 * ratio used is 1:10, giving a voltage division ratio of 1/11.
 *
 * @param divider Ratio between VBUS_MON voltage and VBus voltage.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6, PAG1S.
 */
void pd_hal_set_vbus_mon_divider(uint8_t divider);

/**
 * @brief Specify the RSENSE used for CSA operation. The function is valid only
 * for CCG3PA/CCG3PA2 implementation currently. The value specified is in 100uOhm
 * units and should include the parasitic resistances as well.
 *
 * @param port PD port index.
 * @param rsense The total Rsense in 100uOhm units.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
void pd_hal_set_vbus_csa_rsense(uint8_t port, uint8_t rsense);

/**
 * @brief Returns the RSENSE value used for CSA operation. The function is valid
 * only for CCG3PA/CCG3PA2 implementation currently. The value specified is in
 * 100uOhm units.
 *
 * @param port PD port index
 * @return The total Rsense in 100uOhm units.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D.
 */
uint8_t pd_hal_get_vbus_csa_rsense(uint8_t port);

/**
 * @brief Calculates the gain and reference settings for a specific current
 * requirement. This function eliminates the precalculated table.
 *
 * @param cur_10mA Current setting in 10mA unit for which CSA setting is required.
 * @param gain_sel Pointer to variable where the gain setting code corresponding
 * to the current setting needs to be stored.
 * @param vref_sel Pointer to variable where the voltage reference code
 *  corresponding to the current setting needs to be stored.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_lscsa_calc_cfg(uint32_t cur_10mA, uint8_t *gain_sel, uint8_t *vref_sel);

/**
 * @brief Sets up LSCSA for specified application.
 *
 * @param port USB-PD port index.
 * @param lscsa_app LSCSA application
 * @param gain_sel Gain selection code to be used
 * @return ccg_status_t
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
ccg_status_t pd_lscsa_cfg(uint8_t port, lscsa_app_config_t lscsa_app, uint8_t gain_sel);

/**
 * @brief Calculates the reference setting for a specific current and gain
 * requirement.
 *
 * @param port USB-PD port
 * @param cur_10mA Current setting in 10mA unit for which CSA setting is required.
 * @param gain Gain setting for CSA
 * to the current setting needs to be stored.
 * @param vref_sel Pointer to variable where the voltage reference code
 *  corresponding to the current setting needs to be stored.
 * @param vin_rsense Whether to use Vin Rsense for calculation or Vout/Vbus Rsense.
 *  true if Vin Rsense, false if Vout/Vbus Rsense. 
 * @return None
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_csa_calc_ref(uint8_t port, uint32_t cur_10mA, uint8_t gain, uint8_t *vref_sel, bool vin_rsense);

/**
 * @brief Enables VBus Current foldback on the specified port.
 *
 * @param port USB-PD port on which to enable CF.
 * @param cur Operating current in 10mA units.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_cf_enable(uint8_t port, uint32_t cur);

/**
 * @brief Disables VBus Current foldback on the specified port.
 *
 * @param port USB-PD port.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_cf_disable(uint8_t port);

/**
 * @brief Retrieves the status of the CF mode hardware.
 *
 * @param port USB-PD port.
 * @return Whether the hardware module is in CF mode or not.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
bool pd_cf_get_status(uint8_t port);

/**
 * @brief Enables hardware monitoring for VBus Current foldback on the specified port.
 * The function invokes the callback when a change happens from the current state.
 * When the monitor is enabled, the calling function can specify the default state
 * to allow for both high to low and low to high detection at enable. This avoids
 * any race condition while enabling the monitoring. Monitoring should be enabled
 * only when no VBUS transition is enabled.
 *
 * @param port USB-PD port on which to enable CF.
 * @param reset_state Current CF state as expected by the calling function.
 * @param cbk Function to be called when CF event is detected.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_cf_mon_enable(uint8_t port, bool reset_state, vbus_cf_cbk_t cbk);

/**
 * @brief Disables hardware monitoring for VBus Current foldback on the specified port.
 * Monitoring should be disabled when there is VBUS transition as well as when not in
 * PPS mode operation.
 *
 * @param port USB-PD port on which to enable CF.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_cf_mon_disable(uint8_t port);

/** @cond DOXYGEN_HIDE */

/**
 * @brief Solution level handler for cases where VSYS is not present. This is a
 * user-defined handler function that needs to be defined in cases where the user
 * wants to block device operation in bus powered mode. This function can be used
 * to do any application level de-initialization to revert IOs to default state.
 * Please note that PD disable should not be done here as removing Rd will cause
 * a device reset.
 *
 * @return None
 */
void soln_no_vsys_handler(void);

/**
 * @brief Solution level handler for VSYS removal event. This is a user-defined
 * handler function that needs to be defined in cases where the user wants to block
 * the device operation in bus powered mode. This function will be called in ISR
 * context and hence should not block operation for a long time.
 *
 * @return None
 */
void soln_vsys_removal_handler(void);

/**
 * @brief  Switch the CCGx device supply (VDDD) from VSYS to VBus or vice versa.
 * @param use_vsys If true, switch from VBus to VSYS; if false, switch from VSYS to VBus.
 * @return Returns true if the switch operation is successful.
 */
bool pd_hal_switch_vddd_supply(bool use_vsys);

/**
 * @brief Get the VSYS and VBus regulator status for a given PD port.
 * @param port PD port index.
 * @return Contents of the vreg_vsys_ctrl register for this port.
 */
uint32_t ccg_get_vsys_status(uint8_t port);

/** @endcond */

/**
 * @brief Enables the SR comparator.
 *
 * @param port USB-PD port on which to enable comparator.
 * @param cur Operating current in 10mA units.
 * @param edge Which edge interrupt configure for.
 * @param cbk Function to be called when interrupt happens.
 * @return State of the comparator after configruation.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
bool pd_set_sr_comp(uint8_t port, uint32_t cur,
    filter_edge_detect_cfg_t edge, pd_cmp_cbk_t cbk);

/**
 * @brief Disables SR comparator on the specified port.
 *
 * @param port USB-PD port.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_stop_sr_comp(uint8_t port);

/**
 * @brief Enables the PFC comparator.
 *
 * @param port USB-PD port on which to enable comparator.
 * @param cur Operating current in 10mA units.
 * @param edge Which edge interrupt configure for.
 * @param cbk Function to be called when interrupt happens.
 *
 * @return State of the comparator after configruation.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
bool pd_set_pfc_comp(uint8_t port, uint32_t cur,
    filter_edge_detect_cfg_t edge, pd_cmp_cbk_t cbk);

/**
 * @brief Disables PFC comparator on the specified port.
 *
 * @param port USB-PD port.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
void pd_stop_pfc_comp(uint8_t port);

/**
 * @brief Samples the PFC comparator. The function samples against the
 * requested current and restores the previous settings after sampling. This
 * function is useful when the comparator is already configured for interrupt
 * and needs to be multiplexed to sample at a different current.
 *
 * @param port USB-PD port on which to sample comparator.
 * @param cur Operating current in 10mA units.
 * @return State of the comparator on sampling.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
bool pd_sample_pfc_comp(uint8_t port, uint32_t cur);

/**
 * @brief Gets the current status of the VBUS / LSCSA comparators.
 *
 * @param port USB-PD port.
 * @param id comparator filter ID for which the status has to be retrieved.
 * @param is_filtered Whether to get the filtered or unfiltered status.
 * @return State of the comparator.
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S.
 */
bool pd_cmp_get_status(uint8_t port, filter_id_t id, bool is_filtered);

#if BCR
/**
 * @brief Turns the VBUS Consumer FET (on VBUS_P_CTRL) on slowly
 *
 * @param delay Delay between current sink steps on the gate driver
 * @param step Step change in current sink
 * @param threshold Threshold at which delay switches from 'delay' to 1ms
 *
 * @return None
 */
void vbus_fet_slow_on(uint8_t delay, uint8_t step, uint8_t threshold);

/**
 * @brief Turns the VBUS Consumer FET (on VBUS_P_CTRL) on weakly
 *        The FET turn on with isink value 0x01.
 *
 * @param None
 *
 * @return None
 */
void vbus_fet_weakly_on(void);

/**
 * @brief Adjust isnk of the VBUS Consumer FET (on VBUS_P_CTRL)
 *
 * @param inks controlled isnk value
 *
 * @return None
 */
void vbus_fet_adjust(uint8_t isnk);

/**
 * @brief Turns the VBUS Consumer FET (on VBUS_P_CTRL) on strongly
 *        as isnk set strong pull-down.
 *
 * @param None
 *
 * @return None
 */
void vbus_fet_strongly_on(void);
#endif /* BCR */

#if VBTR_ENABLE
/**
 * @brief Gets the current status of the VBTR module.
 *
 * @param port USB-PD port.
 * @return Idle state of the VBTR module. Returns False if VBTR Vbus voltage
 * transition is in progress.
 *
 * <B>Applicable devices:</B> PAG1S, CCG7D.
 */
bool pd_hal_vbtr_is_idle(uint8_t port);
/**
 * @brief Configures VBUS transition operation for the requested idac value.
 *
 * @param port USB-PD port.
 * @param dac_req signed iDAC value. Negative for source and positive for sink.
 * @param vbtr_cb callback to notify vbtr events
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S, CCG7D.
 */
void pd_hal_set_vbtr_idac(uint8_t port, int16_t dac_req, PD_ADC_CB_T vbtr_cb);
/**
 * @brief Aborts the ongoing VBUS transition operation.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S, CCG7D.
 */
void pd_hal_vbtr_abort(uint8_t port);
#endif /* VBTR_ENABLE */

#if IBTR_ENABLE
/**
 * @brief Gets the current status of the IBTR module.
 *
 * @param port USB-PD port.
 * @return Idle state of the IBTR module. Returns False if IBTR refgen
 * transition is in progress.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool pd_hal_ibtr_is_idle(uint8_t port);
/**
 * @brief Configures EA comparator refgen transition operation 
 * for the requested value.
 *
 * @param port USB-PD port.
 * @param refgen_dac Refgen value for EA comparator.
 * @param ibtr_cb callback to notify ibtr events
 * @return None.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_hal_ibtr_set(uint8_t port, uint16_t refgen_req, PD_ADC_CB_T ibtr_cb);
/**
 * @brief Aborts the ongoing IBTR operation.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_hal_ibtr_abort(uint8_t port);
#endif /* IBTR_ENABLE */

/**
 * @brief Enable SR sensing.
 * NOTE: This should not be called as it is already taken care of as part of
 * HAL handlers.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_srsense_enable(uint8_t port);

/**
 * @brief Disable the SR sensing and gate driver.
 * NOTE: This should not be called as it is already taken care of as part of
 * HAL handlers.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_srsense_disable(uint8_t port);

/**
 * @brief Enable SR gate driver.
 * NOTE: If the gate driver needs to be selectively enabled, then invoke this
 * API.
 * NOTE: If the CCG_SRGDRV_DISABLE_ON_TRANSITION macro is enabled, the
 * gate driver has to be explicitly enabled.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_srgdrv_enable(uint8_t port);

/**
 * @brief Disable the SR gate driver.
 * NOTE: If the gate driver needs to be selectively disabled, then invoke this
 * API.
 * NOTE: If the CCG_SRGDRV_DISABLE_ON_TRANSITION macro is enabled, the
 * gate driver has to be explicitly enabled/disabled while transition happens.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_srgdrv_disable(uint8_t port);

/**
 * @brief Measure the line voltage equivalent via line feedforward
 *
 * @param port USB-PD port.
 * @return Line voltage in volts.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
uint16_t pd_hal_measure_line_volt(uint8_t port);

/**
 * @brief Measure the EA voltage. The function saturates at 2V.
 *
 * @param port USB-PD port.
 * @return EA voltage in mV.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
uint16_t pd_hal_measure_ea_volt(uint8_t port);

#if defined(PAG1S)
/**
 * @brief Handle failure in PASC calibration
 *
 * @param port Type-C port index.
 * @return None
 */
void pd_pasc_cal_fail_cbk(uint8_t port);
#endif /* defined(PAG1S) */

/**
 * @brief Run the PWM based tasks for regulation control.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_poll_task(uint8_t port);

/**
 * @brief This function does blocking Vbus transition to VSAFE_5V from lower iDAC 
 * setting to zero. It assumes that the caller has already setup the iDAC to negative
 * value. Calling the function with a positive iDAC setting shall result in EA
 * being loaded directly to zero and CV mode disabled. If the VBUS is higher than
 * 5V, the VBUS_IN safe voltage correction is expected to drop the voltage.
 *
 * The function is expected to be called only during HAL start / stop functions
 * and should not be invoked while in attached state.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S, CCG7D.
 */
void pd_vbus_set_vbtr_vsafe_5V(uint8_t port);

/**
 * @brief This function initiates a transition of VBUS_IN voltage from VSAFE_5V to the
 * selected voltage.
 *
 * The function must be called only in un-attached state. If it needs to be aborted,
 * then VBUS_IN has to be first set to 5V using pd_pasc_vbus_in_set_volt_abort(). This
 * implementation is available for low power mode functionality and is not expected to
 * be called explicitly by the user.
 *
 * @param port USB-PD port.
 * @param voltmV Voltage to be set in mV units
 * @param cbk The callback function to be invoked on completion of the transition.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_vbus_in_set_volt(uint8_t port, uint16_t voltmV, PD_ADC_CB_T cbk);

/**
 * @brief This function aborts and restores the VBUS_IN transition to normal mode.
 *
 * The function must be called only in un-attached state. This implementation is 
 * available for low power mode functionality and is not expected to be called
 * explicitly by the user.
 *
 * @param port USB-PD port.
 * @return None.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_vbus_in_set_volt_abort(uint8_t port);

/**
 * @brief Return calibration status
 *
 * @param port USB-PD port.
 *
 * @return True when successful.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
bool pd_pasc_is_cal_success(uint8_t port);

/**
 * @brief Start and initalize the secondary side regulation control (SSC).
 * The function is invoked directly from the pd_pasc_init function and
 * should not be invoked outside of the existing initialization sequence.
 *
 * @param port USB-PD port.
 * @param Callback function to handle calibration failure in application.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_start(uint8_t port, pasc_cal_fail_cbk_t cbk);

/**
 * @brief Get the current valley for PWM operation.
 *
 * @param port USB-PD port.
 * @return Current operating valley.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
uint8_t pd_pasc_get_valley(uint8_t port);

/**
 * @brief Update the operating valley for PWM operation.
 * The funtion only triggers the transition and does not wait for
 * completion. The callback is triggered on completion or if an
 * error is encountered. If a valley update is called while in progress
 * the existing request shall be aborted and callback shall not be
 * triggered.
 *
 * @param port USB-PD port.
 * @param valley New valley to move to.
 * @param cb Pointer to callback function to be invoked on completion of
 *      the request.
 * @return Status of the update request.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
ccg_status_t pd_pasc_set_valley(uint8_t port, uint8_t valley, pasc_valley_cbk_t cb);

/**
 * @brief Initialize and start the firmare valley algorithm.
 * 
 * Valley algorithm is applicable only for QR operation in SSC mode PAG1Sdesigns.
 * The valley table provides the various valleys for optimal frequency of operation.
 * The function does not validate the parameters and the caller is expected to 
 * provide only valid data matching the system.
 *
 * @param port USB-PD port.
 * @param table The valley table to be used for performing the algorithm.
 * @return Status of the update request.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_valley_algo_enable(uint8_t port, const pasc_valley_table_t *table);

/**
 * @brief Enable contention detection interrupt in SSC mode operation.
 *
 * The function enables contention detection between PWM signal and SR to avoid
 * cross conduction during sudden load transitions.
 *
 * @param port USB-PD port.
 * @param cbk Callback function pointer.
 * @return Status of the update request.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_ptdrv_cont_det_enable(uint8_t port, pasc_ptdrv_cont_cbk_t cbk);

/**
 * @brief Disable contention detection interrupt in SSC mode operation.
 *
 * @param port USB-PD port.
 * @return Status of the update request.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_ptdrv_cont_det_disable(uint8_t port);

#if PASC_PTDRV_IN_VAR_WIDTH_ENABLE
/**
 * @brief Enable PTDRV_IN max width interrupt in SSC mode operation.
 *
 * @param port USB-PD port.
 * @param cbk Callback function pointer.
 * @return None
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_ptdrv_in_var_width_enable(uint8_t port, pasc_ptdrv_in_var_width_cbk_t cb);

/**
 * @brief Disable PTDRV_IN max width interrupt in SSC mode operation.
 *
 * @param port USB-PD port.
 * @return None
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_ptdrv_in_var_width_disable(uint8_t port);
#endif /* PASC_PTDRV_IN_VAR_WIDTH_ENABLE */

/**
 * @brief Enable the low power operation for the regulation.
 *
 * This function should be called only from the detach handler. Calling this
 * from anywhere else can result in unstable system. This is used when system
 * needs to be in low power mode operation.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_lp_enable(uint8_t port);

/**
 * @brief Disable the low power operation for the regulation.
 *
 * This function should be called only from the early attach handler. Calling this
 * from anywhere else can result in unstable system. This is used when system
 * needs to be in low power mode operation.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_lp_disable(uint8_t port);

/**
 * @brief Low power task handler for regulation.
 *
 * This function should be called only from the main task loop. Calling this
 * from anywhere else can result in unstable system. This is used when system
 * needs to be in low power mode operation.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_lp_task(uint8_t port);

/**
 * @brief Returns whether system regulator low power mode operation is active or not.
 *
 * This function should be called from the sleep / deep sleep check to start the deep
 * sleep mode operation.
 *
 * @param port USB-PD port.
 * @return true=If LP mode is active; false=if not enabled.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
bool pd_pasc_lp_is_active(uint8_t port);

/**
 * @brief Returns whether system regulator low power mode operation is ready or not.
 *
 * This function should be called from the sleep / deep sleep check to start the deep
 * sleep mode operation.
 *
 * @param port USB-PD port.
 * @return true=If LP mode is ready; false=if not ready.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
bool pd_pasc_lp_ds_allowed(uint8_t port);

/**
 * @brief Send stop signal to PAG1P device.
 *
 * This function should be called only when a fatal regulation error is detected.
 * The function sends stop signal to the PAG1P device in SSC mode operation. PAG1P
 * on receiving this signal shall stop pulsing the primary side. This shall result
 * in VBUS_IN going to zero. The only recovery path for this is AC power cycle at
 * the primary end.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> PAG1S.
 */
void pd_pasc_send_stop_signal(uint8_t port);

/**
 * @brief Returns HPD state machine status.
 *
 * @param port USB-PD port.
 *
 * @return HPD state machine status
 */

bool hpdt_ctrl1_reg_check_start(uint8_t port);

/**
 * @brief Initializes the Buck-Boost regulation.
 *
 * This function should be called only during HAL init.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_bb_init(uint8_t port);

/**
 * @brief Enables the Buck-Boost regulation with soft start.
 *
 * This function should be called to start the regulation with soft 
 * start sequence. pd_bb_init() must have been called prior to this function.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_bb_enable(uint8_t port);

/**
 * @brief Disables the Buck-Boost regulation.
 *
 * This function resets buck-boost operation until pd_bb_enable() is called.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_bb_disable(uint8_t port);

/**
 * @brief Power Down the Buck-Boost block for power saving.
 *
 * This function should be called only during Deep Sleep entry.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_bb_power_down(uint8_t port);

/**
 * @brief Power Up the Buck-Boost block for active mode.
 *
 * This function should be called only during Deep Sleep exit.
 *
 * @param port USB-PD port.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
void pd_bb_power_up(uint8_t port);

/**
 * @brief Buck-Boost regulation enable status.
 *
 * This function returns the status of Buck-Boost regulation enable.
 *
 * @param port USB-PD port.
 *
 * @return Buck-boost enable status.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool pd_bb_is_enabled(uint8_t port);

/**
 * @brief Buck-Boost regulation ready status.
 *
 * This function returns the Buck-Boost regulation enable 
 * sequence done and regulator ready status.
 *
 * @param port USB-PD port.
 *
 * @return Buck-boost enable status.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool pd_bb_is_ready(uint8_t port);

/*
 * Function measures and returns battery voltage on VIN.
 */
uint16_t pd_hal_measure_vbat(uint8_t port);

/*
 * Function enables/disables the CC OVP.
 */
void pd_hal_cc_ovp_ctrl(uint8_t port, bool flag);

/**
 * @brief Measure internal temperature function.
 *
 * This function returns the temperature value obtained 
 * from the internal BJT. The reading from the BJT is mapped to 
 * temperature value using SFLASH trims for HOT, ROOM and COLD
 *
 * @param port USB-PD port.
 *
 * @return Temperature indicated by the internal BJT.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
uint8_t pd_hal_measure_internal_temp(uint8_t port);

#if FW_FAULT_TRIGGER_ENABLE
/**
 * @brief Function triggers faults that are common among all the Type-C ports.
 *
 * @param fault Unique index of the fault to be triggerd.
 *
 * @return bool true: Requested fault triggered. false: Requested fault compile time disabled / not supported.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool trigger_common_fault(cmn_fault_id_t fault);

/**
 * @brief Function triggers faults that are specific to each Type-C port.
 *
 * @param port Port index for which the fault needs to be triggered.
 * @param fault Unique index of the fault to be triggerd.
 *
 * @return bool true: Requested fault triggered. false: Requested fault compile time disabled / not supported.
 *
 * <B>Applicable devices:</B> CCG7D.
 */
bool trigger_port_fault(uint8_t port, port_fault_id_t fault);
#endif /* FW_FAULT_TRIGGER_ENABLE */
/** \} group_ccgxAppCommon_functions */
#endif
#endif /* _PDSS_HAL_H_ */

/* End of file */
