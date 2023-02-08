/*
 * @cc_boot.c
 * \version 2.0
 * @brief CC bootloader policy source file.
 *
 * Copyright (2014-2023), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <config.h>
#include "flash.h"
#include "cy_gpio.h"
#include "srom.h"
#include "cc_boot.h"
#include "uvdm.h"
#include "system.h"  
#include "pdss_hal.h"
#include <cy_pdutils_sw_timer.h>
#include "app_timer_id.h"
#include "cy_pdstack_timer_id.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_common.h"
#include "cy_pdutils.h"
#if CCG_BOOT
/** Pointer array for HW IP register structure. */
static PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
{
    PDSS0
#if CCG_PD_DUALPORT_ENABLE
    ,
    PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
};
#if CC_BOOT
#if SOURCE_BOOT
/* CC1/2 trims */
#define SFLASH_PDSS_PORT0_RP_MODE_DEF_TRIM(port) (*(volatile uint8_t *)((0x0FFFF410u) + \
        ((port) << (CCG_FLASH_ROW_SHIFT_NUM))))
#endif /* SOURCE_BOOT */

/*
 * Trim bits to change the OCP trip point
 * 0000 - unused
 * 0001 - 1.2u (4.8mA)
 * 0010 - 2.4u (9.6mA)
 * 0100 - 4.8u (19.2 mA)
 * 1000 -  7.2u (28.8 mA)
 */
#define PDSS_TRIM_BB_20VCONN_2_BB_20VCONN_OCP_TRIM_MASK     (0x0000000fUL) /* <0:3> R:RW:8: */
#define PDSS_TRIM_BB_20VCONN_2_BB_20VCONN_OCP_TRIM_POS      (0UL)

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
#define BG_ISNK_DAC_CTRL_0_5V(port) (*(volatile uint8_t *)((0x0ffff461u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))
#define BG_ISNK_DAC_CTRL_1_5V(port) (*(volatile uint8_t *)((0x0ffff460u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))

#define BG_ISNK_DAC_CTRL_COMBINED_5V(port) (BG_ISNK_DAC_CTRL_0_5V(port) | \
        ((uint32_t)BG_ISNK_DAC_CTRL_1_5V(port) << 8))

#define BG_ISNK_DAC_CTRL_0_20V(port) (*(volatile uint8_t *)((0x0ffff464u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))
#define BG_ISNK_DAC_CTRL_1_20V(port) (*(volatile uint8_t *)((0x0ffff463u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))

#define BG_ISNK_DAC_CTRL_COMBINED_20V(port) (BG_ISNK_DAC_CTRL_0_20V(port) | \
        ((uint32_t)BG_ISNK_DAC_CTRL_1_20V(port) << 8))

/* Source DAC trim - 3V */
#define BG_ISRC_DAC_CTRL_0_3V(port)  (*(volatile uint8_t *)((0x0ffff46Eu) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))
/* Source DAC trim - 5V */
#define BG_ISRC_DAC_CTRL_0_5V(port)  (*(volatile uint8_t *)((0x0ffff462u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))
/* Source DAC trim - 20V */
#define BG_ISRC_DAC_CTRL_0_20V(port) (*(volatile uint8_t *)((0x0ffff465u) + \
        (((uint32_t)port) << (CCG_FLASH_ROW_SHIFT_NUM))))
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

/* 
 * Bit position in T_VCONN field for slow turn ON and 
 * enable the slow charging of 20vconn switch 
 * for inrush current control
 */
#define PDSS_VCONN20_CTRL_T_VCONN_SLOW_TURN_ON_POS      (12u)

/* VCONN OCP trim */
#define VCONN_OCP_TRIM(port) (*(volatile uint8_t *)((0x0ffff13cu) + \
        (((uint32_t)(port)) * (0x32u))))

/* VCONN OCP offset on CC2 line in steps of 4.8mA */
#define VCONN_OCP_TRIM_CC2_OFFSET       (1u)

#define CCG_SILICON_REV00_VALUE         (0x11)

/* Ordered set detection. SOP and Hard Reset. */
#define UFP_OS_CFG                      (0x0000410Bu)

/* USBPD header fields macros */
#define PDSS_MSG_SZ(x)                  ((((x) >> 12) & 0x7u) + 1u)

#define DATA_ROLE_DFP                   (0x0020u)

#define RX_INTERRUPTS1                  \
    (PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE |      \
     PDSS_INTR0_RCV_BAD_PACKET_COMPLETE  |      \
     PDSS_INTR0_TX_GOODCRC_MSG_DONE      |      \
     PDSS_INTR0_RCV_RST                  |      \
     PDSS_INTR0_COLLISION_TYPE3)

/* Index of PD Header in data message */
#define PD_HDR_IDX                      (0u)

#define VBUS_P_NGDO_EN_LV_0             (1 << 13)
#define VBUS_P_NGDO_EN_LV_1             (1 << 12)
#define VBUS_P_PLDN_EN_LV_0             (1 << 15)
#define VBUS_P_PLDN_EN_LV_1             (1 << 14)

/* Hardware required averaging values - design team recommendation. */
#define PDSS_NUM_PREAMBLE_AVG_VALUE     (5u)
#define PDSS_NUM_TRANS_AVG_VALUE        (0x31u)

/* This translates to around 50ms of CC debounce. */
#define MAX_DEBOUNCE_ATTACH_COUNT       (2500u)

#define MAX_DEBOUNCE_DETACH_COUNT       (2500u)

/* Structured VDM Response values*/
#define SVDM_DSC_ID_HEADER              (0xFF008041u)
#define SVDM_DSC_ID_HEADER_VDO          (0x920004B4u)
#if defined(CY_DEVICE_CCG7D)
#define SVDM_DSC_ID_PRODUCT_VDO         (0xF6A00000u)
#elif defined(CY_DEVICE_CCG7S)
#define SVDM_DSC_ID_PRODUCT_VDO         (0xF6B00000u)
#else
#define SVDM_DSC_ID_PRODUCT_VDO         (0xF6600000u)
#endif
#define SVDM_DSC_SVID_HEADER            (0xFF008042u)
#define SVDM_DSC_SVID_VDO1              (0x04B40000u)
#define SVDM_DSC_MODE_HEADER            (0x04B48043u)
#define SVDM_DSC_MODE_VDO               (0x00000001u)
#define SVDM_ENTER_MODE_HEADER          (0x04B48044u)

/*
 * CCG3PA discharge drive strength setting. This setting is meant for *A or above
 * silicon. But we are using the same for all revisions to reduce code space.
 */
#define CCG3PA_DISCHG_DS_VBUS_C             (4u)

/*
 * VBUS_IN startup threshold voltage before starting Buck-boost soft start.
 */
#define BB_VBUS_IN_STARTUP_VOLT             (2500u)

/*
 * VBUS_IN startup threshold voltage before starting Buck-boost EA mode.
 */
#if BOOT_BB_STARTUP_WITH_LOAD_EN
#define BB_VBUS_IN_STARTUP_THRES            (3000u)
#else /* !BOOT_BB_STARTUP_WITH_LOAD_EN */
#define BB_VBUS_IN_STARTUP_THRES            (4500u)
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
/*
 * Source IDAC value corresponding to BB_VBUS_IN_STARTUP_THRES
 */
#define BB_SOFT_START_THRES_SRC_IDAC        (((5000u) - (BB_VBUS_IN_STARTUP_THRES)) / (20u))

/*
 * Source IDAC step size for software VBTR operation.
 * 1 unit = 20mV.
 */
#define BB_STARTUP_VBTR_STEP_SIZE           (5u)

/*
 * Source IDAC step width or interval between steps
 * for software VBTR operation in uS units.
 */
#define BB_STARTUP_VBTR_STEP_WIDTH_US       (100u)

/*
 * VBUS_IN startup monitor timer period.
 */
#define BB_SOFT_START_MON_TIMER_MS          (1u)

/*
 * BB soft start duty increase step.
 */
#define BB_SOFT_START_DUTY_STEP_UP_PER      (1u)
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

#if BOOT_BB_FCCM_MODE_EN
/*
 * FCCM mode enable delay after startup
 */
#define BB_FCCM_EN_DELAY_TIMER_MS           (2u)
#endif /* BOOT_BB_FCCM_MODE_EN */

/*
 * VBUS_IN measurement hysterisis in mV
 */
#define VBUS_MEASUREMENT_HYSTERISIS         (200u)

/*
 * VBUS_IN maximum voltage allowed to reach during soft start.
 */
#define PASC_VBUS_IN_STARTUP_MAX_VOLT       (5250u)

/*
 * VBUS_IN voltage threshold to indicate whether PAG1P is in open loop mode during bootloader.
 */
#define PASC_VBUS_IN_COLD_START_THRESHOLD   (4000u)

/*
 * VBUS_IN minimum voltage allowed to reach before sending start pulses.
 */
#define PASC_VBUS_IN_STARTUP_MIN_VOLT       (4500u)

/*
 * VBUS_IN slope measurement sample count
 * For N sample measurements the count shall be N+1
 */
#define PASC_VBUS_IN_SLOPE_SAMPLE_COUNT     (11u)

/*
 * VBUS_IN startup EA DAC value.
 */
#define VBUS_IN_STARTUP_SRC_EA              (85)

/* PAG1S VBUS_IN resistor divider for Type-C VBUS monitoring using ADC. */
#define AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS    (9)

#define VBUS_MON_DIV_8P_VAL                 (25u)   /* Multiplied by 2. */

/*
 * GPIO to be used as PWM signal to the primary. This is defined for PAG1S
 * silicon and should not be changed.
 */
#define PASC_PTDRV_GPIO                     (GPIO_PORT_0_PIN_1)

/* 
 * Number of manual pulses to be sent as the start up sequence to the primary
 * controller. This is defined by the PAG1P requirements and should not be
 * changed.
 */
#define PASC_PTDRV_PULSE_COUNT              (4)

/*
 * Delay to be applied in us as the OFF time for the start up pulsing.
 * This is defined for PAG1P and should not be modified.
 */
#define PASC_PTDRV_PULSE_OFF_TIME_US        (23)

/* Time in us taken for peak calibration to complete. */
#define PASC_PTDRV_CAL_TIME                 (100)

/* This is required only on PSVP due to a glitch on SR gate. */
#define PASC_HIP_SEQ_ZCD_EN_ON_DLY_VALUE    (0x1F)
#define PASC_HIP_SEQ_ZCDF_EN_ON_DLY_VALUE   (0x0F)

static pd_state_t gl_pd_state[NO_OF_TYPEC_PORTS];
static uint32_t volatile gl_pd_event[NO_OF_TYPEC_PORTS];
static uint32_t gl_rcvd_pkt[CY_PD_MAX_PD_PKT_WORDS];
static uint8_t volatile gl_tr_msg_id[NO_OF_TYPEC_PORTS];

static uint8_t volatile gl_first_msg_rcvd[NO_OF_TYPEC_PORTS];

/* Stores Current received packet's message id */
static uint8_t gl_cur_rec_msg_id[NO_OF_TYPEC_PORTS];

/*
 * Variable stores message Id of last processed packet.
 * It's used along with gl_cur_rec_msg_id to detect whether
 * new packet is received
 */
static uint8_t volatile gl_rec_msg_id[NO_OF_TYPEC_PORTS];

static cy_pd_pd_do_t dobject[8];
static volatile uint32_t gl_cc_count[NO_OF_TYPEC_PORTS][2] = {{0, 0}
#if CCG_PD_DUALPORT_ENABLE
, {0, 0}
#endif
};
static volatile uint32_t gl_active_channel[NO_OF_TYPEC_PORTS];

/** Pointer array for HW IP register structure. */
static PPDSS_TRIMS_REGS_T gl_pdss_trims[NO_OF_TYPEC_PORTS] =
{
    PDSS_TRIMS0
#if CCG_PD_DUALPORT_ENABLE
    ,
    PDSS_TRIMS1
#endif /* CCG_PD_DUALPORT_ENABLE */
};

#if (SOURCE_BOOT)
 /* Port type DFP,UFP or DRP */
uint8_t gl_cur_port_type = CY_PD_PRT_TYPE_DFP;
cy_pd_pd_do_t src_pdo[1]= {{0x0A01905A}};
#endif /* SOURCE_BOOT */

uint8_t ccg_get_si_revision(void)
{
    return (get_silicon_revision() - CCG_SILICON_REV00_VALUE);
}

void pd_reset_protocol(uint8_t port)
{
    gl_rec_msg_id[port] = 0;
    gl_tr_msg_id[port] = 0;
    gl_first_msg_rcvd[port] = (uint8_t)false;
}

/**
 * @brief This function sends PD message
 * @param msg_type Type of PD message.
 * @param dobj Pointer to data object
 * @param count Number of data objects to send. It should be less than or equal to seven.
 * @return None
 */
static void send_message(uint8_t port, uint8_t msg_type, uint32_t* pdo, uint8_t count)
{
    uint32_t i;          /* Buf load index */
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];
    /*Wait 1ms to over previous message transmission */
    Cy_SysLib_Delay(1);
    /* Update msg type, ID, count and data role(port type) in the packet header */
    PDSS_REG->tx_header = CY_PD_HEADER(msg_type, gl_tr_msg_id[port], count);
#if (SOURCE_BOOT)
    PDSS_REG->tx_header |= CY_PD_DR_PR_ROLE(gl_cur_port_type, CY_PD_PRT_ROLE_SOURCE);
#endif /*SOURCE_BOOT*/

    for (i = 0; i < count; i++)
    {
        PDSS_REG->tx_mem_data[i]  = pdo[i];
    }

    /* Begin transmission. */
    PDSS_REG->tx_ctrl |= PDSS_TX_CTRL_TX_GO;

    /* Delay to complete message transmission(~2ms) and good CRC (~ 0.5 ms) */
    Cy_SysLib_Delay(3);
    /* Increment message IDs */
    gl_tr_msg_id[port] = (gl_tr_msg_id[port] + 1) & CY_PD_MAX_MESSAGE_ID;
}

#if (defined (CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

/* Buck-Boost (BB) clock frequency in kHz */

#define BBCLK_KHZ                               (24000u)

/*
 * Conversion algorithm for frequency in KHz to time in units of BB clock 
 * cycle numbers.
 */
#define FREQ_KHZ_TO_CYCLE_TIME(freq)            ((BBCLK_KHZ) / (freq))

/* VBUS_IN discharge drive strenth */
#define CCG7D_DISCHG_DS_VBUS_IN                 (16u)

#if SOURCE_BOOT
/* Function to initialize buck-boot for 5V regulation */
static void pdss_phy_bb_init(uint8_t port);
#endif /* SOURCE_BOOT */

/* Functions to enable/disable the buck-boost in bootloader mode of operation */
static void pdss_enable_bb(cy_stc_pdstack_context_t *context);
static void pdss_disable_bb(cy_stc_pdstack_context_t *context);

#if VBAT_GND_FET_IO_ENABLE
/* Type-C GND FET GPIO list */
uint8_t gl_vbat_gnd_fet_gpio[NO_OF_TYPEC_PORTS] = 
{
    VBAT_GND_FET_PIN_PORT_0,
#if (NO_OF_TYPEC_PORTS > 1)
    VBAT_GND_FET_PIN_PORT_1
#endif /* (NO_OF_TYPEC_PORTS >= 1) */
};
#endif /* VBAT_GND_FET_IO_ENABLE */
#endif /* defined (CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

#if (defined(CY_DEVICE_PAG1S) && (CCG_REG_SEC_CTRL != 0))

/* Initilization value for the SRSNS_2_CTRL register. */
#define PDSS_SRSNS_2_CTRL_INIT_VALUE            (0x6048)

void pd_pasc_disable(uint8_t port)
{
    (void)port;

    /* Set HSIOM configuration for PWM for GPIO to prevent any false pulses. */
    gpio_hsiom_set_config(PASC_PTDRV_GPIO, HSIOM_MODE_GPIO, GPIO_DM_STRONG, false);

    /* Hold PASC in reset. */
    PDSS->debug_ctrl |= PDSS_DEBUG_CTRL_RESET_PASC;

    PDSS->pasc_ctrl &= ~PDSS_PASC_CTRL_PA_EN;

    /* Power down the EA block. */
    PDSS->ea_ctrl |= PDSS_EA_CTRL_EA_PD;

    /* Disable PWM block. */
    PDSS->pwm_0_ctrl &= ~(PDSS_PWM_0_CTRL_ENABLE_PWM | PDSS_PWM_0_CTRL_PWM_ISO_N |
            PDSS_PWM_0_CTRL_ENABLE_PWDM_DAC | PDSS_PWM_0_CTRL_ENABLE_PWM_DAC_FF | 
            PDSS_PWM_0_CTRL_ENABLE_PWM_HCLAMP | PDSS_PWM_0_CTRL_ENABLE_PWM_LCLAMP |
            PDSS_PWM_0_CTRL_ENABLE_PWM_SKIP | PDSS_PWM_0_CTRL_ENABLE_PWM_BURST_EXIT);
}

static void pd_pasc_init(void)
{
    uint32_t tmp = 0;

    /* PASC mode is set to QR by default */

    /* Enable the SR sense block. */
    tmp = PDSS->srsns_0_ctrl;
    tmp &= ~(PDSS_SRSNS_0_CTRL_ZCD_PD | PDSS_SRSNS_0_CTRL_ZCDF_PD |
        PDSS_SRSNS_0_CTRL_NSN_PD | PDSS_SRSNS_0_CTRL_PEAKDET_PD);
    tmp |= (PDSS_SRSNS_0_CTRL_SRSNS_ISO_N | PDSS_SRSNS_0_CTRL_FEEDFWD_EN);
    PDSS->srsns_0_ctrl = tmp;

    /* 
     * BIT14:13: To enable Fast NSN comparator with 200mV ref.
     * BIT6: To set NSN threshold to zero volt.
     * BIT3: To enable NSN hysterisis
     */
    PDSS->srsns_2_ctrl = PDSS_SRSNS_2_CTRL_INIT_VALUE;

    /* Configure EA for PWM mode operation. */
    PDSS->pds_ea_1_ctrl |= PDSS_PDS_EA_1_CTRL_EN_PWM_MODE;

    /* Setting for 9:1 turns ratio. */
    CY_USBPD_REG_FIELD_UPDATE(PDSS->srsns_1_ctrl, PDSS_SRSNS_1_CTRL_FFWD_TEST, 0x040F);

    /* Enable PWM block. */
    PDSS->pwm_0_ctrl |= (PDSS_PWM_0_CTRL_ENABLE_PWM | PDSS_PWM_0_CTRL_PWM_ISO_N |
            PDSS_PWM_0_CTRL_ENABLE_PWDM_DAC | PDSS_PWM_0_CTRL_ENABLE_PWM_DAC_FF | 
            PDSS_PWM_0_CTRL_ENABLE_PWM_HCLAMP | PDSS_PWM_0_CTRL_ENABLE_PWM_LCLAMP |
            PDSS_PWM_0_CTRL_ENABLE_PWM_SKIP | PDSS_PWM_0_CTRL_ENABLE_PWM_BURST_EXIT);

    /*
     * Skip entry = 450mV, Burst entry = 450mV, burst exit = 450mV, Lclamp = 350mV, Hclamp = 2.4V
     * Increase filter to 0x1F (maximum). ** silicon has a different minimum level for skip entry,
     * skip exit, burst exit, and VREF_LOW. So set them differently. The current configuration 
     * is set to avoid burst mode.
     */
    if (get_silicon_revision() != CCG_SILICON_REV00_VALUE)
    {
        CY_USBPD_REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL, 8);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL, 8);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->pwm_0_ctrl, PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL, 8);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL, 7);
    }
    else
    {
        CY_USBPD_REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL, 4);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL, 4);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->pwm_0_ctrl, PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL, 0);
        CY_USBPD_REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL, 3);
    }
    CY_USBPD_REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_HCLAMP_SEL, 3);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->intr15_cfg_0_pwm, PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_SEL, 0x1F);
    
    CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIM_PWM_0, PDSS_TRIM_PWM_0_CAP_DAC_TRIM,  24);
    CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIM_PWM_1, PDSS_TRIM_PWM_1_CAP_DAC_FF_TRIM,  28);
    
    CY_USBPD_REG_FIELD_UPDATE(PDSS->hip_seq_gen_0_ctrl, PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_ON_DLY,
            PASC_HIP_SEQ_ZCD_EN_ON_DLY_VALUE);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->hip_seq_gen_0_ctrl, PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_ON_DLY,
            0x07);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->hip_seq_gen_1_ctrl, PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_ON_DLY,
            PASC_HIP_SEQ_ZCDF_EN_ON_DLY_VALUE);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->hip_seq_gen_1_ctrl, PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_ON_DLY,
            0x05);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->hip_seq_gen_2_ctrl, PDSS_HIP_SEQ_GEN_2_CTRL_NSN_IDLE_TIME,
            0xF0);
    
    /*
     * PWM_DAC and PWM_DAC_FF setting need to be changed to support below
     * configuration.
     * For 85V : 0.5uA (fixed) + 0.1uA (FF) 
     * so that the ramp is ~28uS (from vreflo to vclamp_hi)
     * For 380V : 0.5uA (Fixed) + 0.5uA (FF) - and this will happen 
     * automatically with same setting of PWM DAC as used in 85V case.
     * So that the ramp is of 17uS  (from Vreflo to vclamp_hi)
     * Note: Fixed DAC gain set to 0.2x (2.4u*10/30) and 
     * feedfwd DAC set to 0.03x(2.4u*5/30).
     */
    tmp = PDSS->pwm_1_ctrl & ~(PDSS_PWM_1_CTRL_PWM_DAC_MASK |
            PDSS_PWM_1_CTRL_PWM_DAC_FF_MASK);
    tmp |= ((6 << PDSS_PWM_1_CTRL_PWM_DAC_POS) |
            (3 << PDSS_PWM_1_CTRL_PWM_DAC_FF_POS));
    PDSS->pwm_1_ctrl = tmp;

    /*
     * Configure the AUDIO_TMIN and AUDIO_TMAX to avoid audible buzz. These
     * registers do not configure the frequency but act as TRIM registers to
     * match avoid buzz due to burst mode.
     */
    CY_USBPD_REG_FIELD_UPDATE(PDSS->mode_3_ctrl, PDSS_MODE_3_CTRL_AUDIO_TMAX, 0x4FF);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->mode_4_ctrl, PDSS_MODE_4_CTRL_AUDIO_TMIN, 0x4FA);

    /* Enable peak reset pulse */
    CY_USBPD_REG_FIELD_UPDATE(PDSS->peakgen_0_ctrl, PDSS_PEAKGEN_0_CTRL_PEAK_RESET_PULSE,
            1);

    /* Not performing the calibration */
    PDSS->peakgen_0_ctrl |= PDSS_PEAKGEN_0_CTRL_CALIBRATE_OVERRIDE;

    /* Peak kill needs to be disabled as silicon does not trigger correctly. */
    PDSS->peakgen_1_ctrl &= ~PDSS_PEAKGEN_1_CTRL_PEAK_KILL_EN;

    /* Allow PAG1S hardware to take over PWM control. */
    PDSS->pasc_ctrl |= PDSS_PASC_CTRL_PA_EN;

    /*
     * No calibration as it is being overridden. But, wait for the delay. Not mandatory.
     */
    Cy_SysLib_DelayUs(PASC_PTDRV_CAL_TIME);
}
#endif /* (defined(CY_DEVICE_PAG1S) && (CCG_REG_SEC_CTRL != 0)) */

void pd_send_ctl_msg(uint8_t port, cy_en_pd_ctrl_msg_t msg_type)
{
    send_message(port, msg_type, NULL, 0);
}

void pd_send_data_msg(uint8_t port, cy_en_pdstack_data_msg_t msg_type, cy_pd_pd_do_t* dobj, uint8_t count)
{
    send_message(port, msg_type, &(dobj->val), count);
}

#if !CC_BOOT_NB_CALL_EN
void start_systick_timer(uint8_t timems)
{
    CM0->syst_cvr = 0;
    CM0->syst_rvr = ((timems * SYSCLK_1MS_COUNT_VALUE) - 1);
    CM0->syst_csr = (CM0_SYST_CSR_ENABLE | CM0_SYST_CSR_CLKSOURCE);
}
#endif /* CC_BOOT_NB_CALL_EN */    

void analyze_rx(uint8_t port)
{
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];
    uint32_t msg_type ;
    uint32_t count ;
    uint8_t i, size;
    /* Get the size of RX packet from RX_HEADER register. */
    size = PDSS_MSG_SZ(PDSS_REG->rx_header);
    /* Store the header. */
    gl_rcvd_pkt[0] = PDSS_REG->rx_header;
    /* Copy the data into the specified buffer */
    for (i = 0; i < (size-1); i++)
    {
        *(gl_rcvd_pkt + i+1) = PDSS_REG->rx_mem_data[i];
    }

    /* Update current message id */
    gl_cur_rec_msg_id[port]= CY_PD_GET_PD_HDR_ID(gl_rcvd_pkt[PD_HDR_IDX]);
    msg_type = CY_PD_GET_PD_HDR_TYPE(gl_rcvd_pkt[PD_HDR_IDX]);
    count = CY_PD_GET_PD_HDR_CNT(gl_rcvd_pkt[PD_HDR_IDX]);

    /* Checks if a goodcrc message */
    if ((msg_type == CY_PD_CTRL_MSG_GOOD_CRC) && (count == 0))
    {
        return;
    }

    /* If first message received or valid new message received */
    if((gl_first_msg_rcvd[port] == false)
        || ((gl_rec_msg_id)[port] != gl_cur_rec_msg_id[port]))
    {
        /* Valid packet received */
        gl_first_msg_rcvd[port] = true;
        gl_rec_msg_id[port] = gl_cur_rec_msg_id[port];
        gl_pd_event[port] = CY_PD_PE_EVT_PKT_RCVD;
    }
}

/**
 * PDSS interrupt handler
 * Interrupt and bottom half handling for Tx and Rx paths.
 */
void pdss_phy_port0_intr0_handler(void)
{
    PPDSS_REGS_T PDSS_REG = gl_pdss[0];;
    uint32_t intr0_cause;

    /* Read interrupt causes. */
    intr0_cause = PDSS_REG->intr0_masked;

    if (intr0_cause != 0)
    {
        /* Clear interrupts. */
        PDSS_REG->intr0 = intr0_cause;

        if (intr0_cause & PDSS_INTR0_TX_GOODCRC_MSG_DONE)
        {
            analyze_rx(0);
        }
        if (intr0_cause & PDSS_INTR0_RCV_RST)
        {
            gl_pd_event[0] = CY_PD_PE_EVT_HARD_RESET_RCVD;
        }
    }
}

#if CCG_PD_DUALPORT_ENABLE
/**
 * PDSS interrupt handler
 * Interrupt and bottom half handling for Tx and Rx paths.
 */
void pdss_phy_port1_intr0_handler(void)
{
    PPDSS_REGS_T PDSS = gl_pdss[1];     
    uint32_t intr0_cause;

    /* Read interrupt causes. */
    intr0_cause = PDSS->intr0_masked;

    if (intr0_cause != 0)
    {
        /* Clear interrupts. */
        PDSS->intr0 = intr0_cause;

        if (intr0_cause & PDSS_INTR0_TX_GOODCRC_MSG_DONE)
        {
            analyze_rx(1);
        }
        if (intr0_cause & PDSS_INTR0_RCV_RST)
        {
            gl_pd_event[1] = CY_PD_PE_EVT_HARD_RESET_RCVD;
        }
    }
}
#endif /* CCG_PD_DUALPORT_ENABLE */

void pdss_phy_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint32_t tmp;
    uint8_t port = ptrPdStackContext->port;
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];

#if (VBUS_CTRL_TYPE == VBUS_CTRL_OPTO_FB) 
    /* Enable shunt */
    PDSS_REG->ea_ctrl |= PDSS_EA_CTRL_EN_SHNT | PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB;
#ifndef CY_DEVICE_PAG1S
    PDSS_TRIMS->trim_ea1_0 = 0xA0;
#endif /* !CY_DEVICE_PAG1S */
#else /* (VBUS_CTRL_TYPE_P1 != VBUS_CTRL_OPTO_FB) */
#if (!defined(CY_DEVICE_PAG1S) && !defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_CCG7S))
    PDSS_TRIMS->trim_ea1_0 = 0x20;
#endif /* (!defined(CY_DEVICE_PAG1S) && !defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_CCG7S)) */
#endif /* VBUS_CTRL_TYPE_P1 */

    /* IP enable. */
    PDSS_REG->ctrl &= ~PDSS_CTRL_IP_ENABLED;
    PDSS_REG->ctrl = PDSS_CTRL_IP_ENABLED;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    PDSS_REG->dpslp_ref_ctrl = ((PDSS_REG->dpslp_ref_ctrl & ~PDSS_DPSLP_REF_CTRL_PD_DPSLP) |
            PDSS_DPSLP_REF_CTRL_IGEN_EN);

#ifdef CY_DEVICE_PAG1S
    /*
     * Also turn on the NGDO. Since this requires delay in stages, putting it
     * across the available delay to save code space.
     */
    PDSS->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
#endif /* CY_DEVICE_PAG1S */
    Cy_SysLib_DelayUs(100);

#ifdef CY_DEVICE_PAG1S
    /* Update Refgen setting */
    PDSS->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;
    /*
     * Continue turn on of NGDO. Since this requires delay in stages, putting
     * it across the available delay to save code space.
     */
    PDSS->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_CP_EN);
#endif /* CY_DEVICE_PAG1S */

     /*Configure CRC_COUNTER reg*/
    PDSS_REG->crc_counter = CRC_COUNTER_CFG;

    /*Configure INTER_PACKET_COUNTER reg*/
    PDSS_REG->inter_packet_counter = INTER_PACKET_COUNTER_CFG;

    /* Enable active circuitry and DC paths. */
    PDSS_REG->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Refgen settings for CCG7D */
    PDSS_REG->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS_REG->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;

    /*
    * As register bits are re-purposed to enable independent control of
    * offset with GM, following settings needs to be added to make it
    * equivalent to A0 silicon.
    */
    PDSS_REG->bb_ea_0_ctrl = ((PDSS_REG->bb_ea_0_ctrl &
        ~(PDSS_BB_EA_0_CTRL_BB_EA_TRIM_CVAMP_IBIAS_MASK |
          PDSS_BB_EA_0_CTRL_BB_EA_TRIM_CCAMP_IBIAS_MASK)) |
         ((uint32_t)1u << PDSS_BB_EA_0_CTRL_BB_EA_TRIM_CVAMP_IBIAS_POS) |
         ((uint32_t)1u << PDSS_BB_EA_0_CTRL_BB_EA_TRIM_CCAMP_IBIAS_POS));

    CY_USBPD_REG_FIELD_UPDATE(PDSS_REG->bb_ea_2_ctrl, PDSS_BB_EA_2_CTRL_BB_EA_T_EA,
        0x1505Cu);

    /* Configure 40vreg inrush trims */
    if(port == 0)
    {
        CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIMS0->trim_bb_40vreg_1, PDSS_TRIM_BB_40VREG_1_VREG_TRIM_INRUSH, 0x0B);
    }
#if (CCG_PD_DUALPORT_ENABLE)
    else
    {
        CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIMS1->trim_bb_40vreg_1, PDSS_TRIM_BB_40VREG_1_VREG_TRIM_INRUSH, 0x0B);        
    }
#endif /* (CCG_PD_DUALPORT_ENABLE) */
    
    /* Configure 40VREG for 5V and active mode */
    CY_USBPD_REG_FIELD_UPDATE(PDSS_REG->refgen_4_ctrl, PDSS_REFGEN_4_CTRL_SEL12, 2);
    PDSS_REG->bb_40vreg_ctrl = ((PDSS_REG->bb_40vreg_ctrl &
        ~(PDSS_BB_40VREG_CTRL_BB_40VREG_T_VREG_3P3_MASK |
        PDSS_BB_40VREG_CTRL_BB_40VREG_T_INRSH_CLAMP_ON)) |
        PDSS_BB_40VREG_CTRL_BB_40VREG_ISO_N |
        PDSS_BB_40VREG_CTRL_BB_40VREG_VREG_ACT_EN |
        PDSS_BB_40VREG_CTRL_BB_40VREG_VBG_EN |
        PDSS_BB_40VREG_CTRL_BB_40VREG_T_DIS_INRSH_CLAMP);


    /* Wait for VDDD to settle for 5V. */
    Cy_SysLib_Delay(1);
#else    
    /* REF-GEN updates from CDT 275028. */
    PDSS_REG->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS_REG->refgen_2_ctrl  = 0x003D6461;
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S); */

#if (SOURCE_BOOT)
#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    gl_pdss_trims[port]->trim_cc_1 = SFLASH_PDSS_PORT0_RP_MODE_DEF_TRIM((uint32_t)port);
    gl_pdss_trims[port]->trim_cc_2 = SFLASH_PDSS_PORT0_RP_MODE_DEF_TRIM((uint32_t)port);
#else
    PDSS_TRIMS->trim_cc_1 = SFLASH_PDSS_PORT0_RP_MODE_DEF_TRIM(port);
    PDSS_TRIMS->trim_cc_2 = SFLASH_PDSS_PORT0_RP_MODE_DEF_TRIM(port);
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S)) */

#if FPGA
    PDSS_REG->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
        PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN | PDSS_CC_CTRL_0_RX_EN |
        (0x03 << PDSS_CC_CTRL_0_RP_MODE_POS) | PDSS_CC_CTRL_0_DFP_EN | PDSS_CC_CTRL_0_CMP_EN |
        (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
        PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | (4u << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS));
    PDSS_REG->cc_ctrl_0 = ((PDSS_REG->cc_ctrl_0 & ~(PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2 |
                        PDSS_CC_CTRL_0_CC_1V2 | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK |
                        PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK)));
#else
    /*
     * Enable proper Rp and enable comparators.
     * Set Rp mode and enable references for source operation.
     */
    PDSS_REG->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
        PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN | PDSS_CC_CTRL_0_RX_EN |
        PDSS_CC_CTRL_0_RP_MODE_MASK | PDSS_CC_CTRL_0_DFP_EN | PDSS_CC_CTRL_0_CMP_EN |
        (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
        PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | (4u << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS));
#endif /* FPGA */
  
#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_PAG1S))
    PDSS->pump_ctrl &= ~ (PDSS_PUMP_CTRL_BYPASS_LV | PDSS_PUMP_CTRL_PD_PUMP);
#endif /* (defined(CCG3PA) || defined(PAG1S)) */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    /* Enable the pump only if it is not already on. */
    if ((gl_pdss[port]->pump5v_ctrl & PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN) == 0)
    {
        gl_pdss[port]->pump5v_ctrl = PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN | PDSS_PUMP5V_CTRL_PUMP5V_BYPASS_LV;
    }
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
    PDSS_REG->tx_ctrl |= CY_PD_DR_PR_ROLE(CY_PD_PRT_TYPE_DFP, CY_PD_PRT_ROLE_SOURCE);
#else
#if (defined(CY_DEVICE_SERIES_WLC1))

    /* Rd Enable is not needed for WICG1  only enable comparators.
     * Connect Up comparaator to CC2 and Down comparator to CC1
     */
    PDSS_REG->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
            PDSS_CC_CTRL_0_RX_EN |
            (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
            PDSS_CC_CTRL_0_CMP_EN);
#else

    /* Enable proper Rd and enable comparators.
     * Connect Up comparator to CC2 and Down comparator to CC1
     */
    PDSS_REG->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
            PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RX_EN |
            (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
            PDSS_CC_CTRL_0_CMP_EN);

#endif /* CY_DEVICE_SERIES_WLC1 */
#endif /* SOURCE_BOOT */
    /*
     * GoodCRC auto response configuration. Since the receiver is controlled,
     * these registers can be enabled all the time.
     */
#if (defined(CY_DEVICE_CCG3))
    PDSS->rx_default_sop_goodcrc_ctrl_0 = AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG;
    PDSS->rx_default_sop_goodcrc_ctrl_1 = 0xFFFFFFFF;
#endif /*CY_DEVICE_CCG3*/
    /*Configure RX_SOP_GOOG_CRC_EN_CTRL*/
    PDSS_REG->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_CFG;
    PDSS_REG->rx_order_set_ctrl = UFP_OS_CFG;
    PDSS_REG->tx_ctrl |= CY_PD_CTRL_MSG_GOOD_CRC;

    /* Receive C-Connector configuration. */
    PDSS_REG->rx_cc_0_cfg = RX_CC_CFG;

    /* Update the averaging logic */
    tmp = PDSS_REG->debug_cc_1 & ~(PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_MASK |
            PDSS_DEBUG_CC_1_NUM_TRANS_AVG_MASK);
    tmp |= (PDSS_NUM_PREAMBLE_AVG_VALUE << PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_POS) |
            (PDSS_NUM_TRANS_AVG_VALUE << PDSS_DEBUG_CC_1_NUM_TRANS_AVG_POS);
    PDSS_REG->debug_cc_1 = tmp;

#if (defined (CY_DEVICE_CCG3))
    tmp = PDSS->s8usbpd_trim_6 & ~PDSS_S8USBPD_TRIM_6_V1P575_TRIM_MASK;
    tmp |= SILICON_TRIM6_V1P575_TRIM_VALUE;
    PDSS->s8usbpd_trim_6 = tmp;

    tmp = PDSS->s8usbpd_trim_3 & ~PDSS_S8USBPD_TRIM_3_V0P55_TRIM_MASK;
    tmp |= SILICON_TRIM3_V0P55_TRIM_VALUE;
    PDSS->s8usbpd_trim_3 = tmp;

    /*
     * Clearing of TX_TRIM field is enough for Rp = 1.5A or Default termination
     * at initialization time. Later TX_TRIM could be updated for Rp = 3A
     * termination.
     */
    PDSS->s8usbpd_trim_0 &= (~(PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK));
#endif /* CY_DEVICE_CCG3 */
    /* Register sync interrupt handler. */
    if(port == 0)
    {
        NVIC_DisableIRQ((IRQn_Type)PD_PORT0_INTR0);
        (void)Cy_SysInt_SetVector((IRQn_Type)PD_PORT0_INTR0, &pdss_phy_port0_intr0_handler);
        NVIC_EnableIRQ((IRQn_Type)PD_PORT0_INTR0);
    }
#if CCG_PD_DUALPORT_ENABLE    
    else
    {
        NVIC_DisableIRQ((IRQn_Type)PD_PORT1_INTR0);
        (void)Cy_SysInt_SetVector((IRQn_Type)PD_PORT1_INTR0, &pdss_phy_port1_intr0_handler);
        NVIC_EnableIRQ((IRQn_Type)PD_PORT1_INTR0);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Clear receive interrupts. */
    PDSS_REG->intr0 = RX_INTERRUPTS1;

    /* Enable receive interrupts. */
    PDSS_REG->intr0_mask |= RX_INTERRUPTS1;

#if (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_PAG1S))
#if (VBUS_CTRL_TYPE != VBUS_CTRL_OPTO_FB)
    {
        PDSS->ea_ctrl |= PDSS_EA_CTRL_RES_DIV_BYPASS;
    }
#endif /* (VBUS_CTRL_TYPE != VBUS_CTRL_OPTO_FB) */
#endif /* (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_PAG1S)) */

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
    PDSS_REG->dischg_shv_ctrl[1] = ((PDSS_REG->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (CCG7D_DISCHG_DS_VBUS_IN << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
#if SOURCE_BOOT
    /* Enable buck-boost for regulated 5V. */
    pdss_phy_bb_init(port);
#endif /* SOURCE_BOOT */
#if VBAT_GND_FET_IO_ENABLE
    /* Remove the pulldown to free up the GPIO */
	CALL_MAP(gpio_hsiom_set_config)(gl_vbat_gnd_fet_gpio[port], 
        HSIOM_MODE_GPIO, GPIO_DM_STRONG, (bool)1);
#endif /* VBAT_GND_FET_IO_ENABLE */
#endif /* defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */
}

#if (defined(CY_DEVICE_PAG1S) && (CCG_REG_SEC_CTRL != 0))
void pdss_phy_regulation_init(void)
{
    uint32_t tmp = 0;
    uint16_t vbus_in_new;
    uint16_t vbus_in_max;
    /* VBUS_IN measurement sample count */
    uint8_t i = 1u;
    /* Flag to indicate whether bootloader entry was done from a state where
     * PAG1P was performing open loop soft start or it was already regulating
     */
    bool cold_start;

    /* Enable shunt */
    PDSS->ea_ctrl |= PDSS_EA_CTRL_EN_SHNT | PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB;

    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORT0_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_PASC_IDX] = PDSS_PORT0_PASC_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_VBTR_IDX] = PDSS_PORT0_VBTR_CLK_DIV_ID;

    /* IP enable. */
    PDSS->ctrl &= ~PDSS_CTRL_IP_ENABLED;
    PDSS->ctrl = PDSS_CTRL_IP_ENABLED;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    PDSS->dpslp_ref_ctrl = ((PDSS->dpslp_ref_ctrl & ~PDSS_DPSLP_REF_CTRL_PD_DPSLP) |
            PDSS_DPSLP_REF_CTRL_IGEN_EN);

    /* Update Refgen setting */
    PDSS->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;

    /* Enable EA to hold the current VBUS_IN voltage */
    tmp = PDSS->ea_ctrl & ~(PDSS_EA_CTRL_ISNK_EN | PDSS_EA_CTRL_ISRC_EN |
    PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK | PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);
    PDSS->ea_ctrl = (tmp | PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN);
    CY_USBPD_REG_FIELD_UPDATE(PDSS->ea_ctrl, PDSS_EA_CTRL_ISRC_DAC_CTRL, VBUS_IN_STARTUP_SRC_EA);

    vbus_in_new = pd_hal_measure_vbus_in(0);

    if(vbus_in_new < PASC_VBUS_IN_COLD_START_THRESHOLD)
    {
        cold_start = true;
    }
    else
    {
        cold_start = false;
    }

    if(true == cold_start)
    {
        /* Let VBUS_IN reach 5.2V */
        while(vbus_in_new < PASC_VBUS_IN_STARTUP_MAX_VOLT)
        {
            vbus_in_new = pd_hal_measure_vbus_in(0);
        }
    }

    /* 
     * Observe consecutive samples of VBUS_IN not going above
     * maximum smaple.
     * For N sample measurements the count shall be N+1
     */
    vbus_in_max = vbus_in_new;

    while(i < PASC_VBUS_IN_SLOPE_SAMPLE_COUNT)
    {
        vbus_in_new = pd_hal_measure_vbus_in(0);

        /* If the new sample is greater than maximum sample, then restart count */
        if(vbus_in_new >= (vbus_in_max + VBUS_MEASUREMENT_HYSTERISIS))
        {
            vbus_in_max = vbus_in_new;
            i = 0;
        }
       
        /* Consider it as good VBUS_IN sample only if it is lesser than max sample */ 
        if((vbus_in_max - vbus_in_new) >= VBUS_MEASUREMENT_HYSTERISIS)
        {
            i++;
        }

        if(true == cold_start)
        {
            if(vbus_in_new < PASC_VBUS_IN_STARTUP_MIN_VOLT)
            {
                break;
            }
        }

        Cy_SysLib_DelayUs(10u);
    }

    gpio_hsiom_set_config(PASC_PTDRV_GPIO, HSIOM_MODE_PTDRV_IN,
            GPIO_DM_STRONG, false);

    /* Enbale PASC to take over voltage control */
    pd_pasc_init();

    /* Set VBUS_IN to VSAFE_5V */
    vbus_set_vsafe(0);
}
#endif /* (defined(CY_DEVICE_PAG1S) && (CCG_REG_SEC_CTRL != 0)) */

#if (SOURCE_BOOT)
#if (defined (CY_DEVICE_CCG3))
void turn_on_vbus(void)
{
    if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
    {
        PDSS->ngdo_ctrl_0 |= (VBUS_P_PLDN_EN_LV_0|VBUS_P_PLDN_EN_LV_1);
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_PLDN_EN_LV_0 | VBUS_P_PLDN_EN_LV_1);
        PDSS->ngdo_ctrl_0 |= (VBUS_P_NGDO_EN_LV_0 | VBUS_P_NGDO_EN_LV_1);
        PDSS->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_RST_EDGE_DET;
    }
}

void turn_off_vbus(void)
{
    if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_PLDN_EN_LV_0|VBUS_P_PLDN_EN_LV_1);
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_NGDO_EN_LV_0 | VBUS_P_NGDO_EN_LV_1);
        PDSS->ngdo_ctrl_0 |= (VBUS_P_PLDN_EN_LV_0 | VBUS_P_PLDN_EN_LV_1);
    }
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    PDSS->vbus_ctrl |= PDSS_VBUS_CTRL_DISCHG_EN;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PDSS->vbus_ctrl &= ~PDSS_VBUS_CTRL_DISCHG_EN;
}

#elif (defined(CY_DEVICE_CCG3PA) || defined(CY_DEVICE_CCG3PA2))
void turn_on_vbus(void)
{
    PDSS->pgdo_pu_1_cfg = (PDSS_PGDO_PU_1_CFG_SEL_ON_OFF |
        PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE);
}

void turn_off_vbus(void)
{
     PDSS->pgdo_pu_1_cfg = PDSS_PGDO_PU_1_CFG_DEFAULT;
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    PDSS->dischg_shv_ctrl[0] = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG |
            (CCG3PA_DISCHG_DS_VBUS_C << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PDSS->dischg_shv_ctrl[0] &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}
#elif (defined(CY_DEVICE_PAG1S))
void turn_on_vbus(void)
{
#if (CCG_REG_SEC_CTRL != 0)
    uint32_t vbus_in = 0;
    uint16_t timeout = 1000;

    /* 
     * In case of secondary side controlled regulation, we need to
     * ensure that VBUS_IN is in safe 5V range before turning on the
     * FET. If VBUS_IN is above 5.25V, try to discharge for 100ms.
     * If VBUS_IN has not dropped below the threshold within this time,
     * stop discharging and wait until VBUS_IN is safe for connect.
     */
    while(1)
    {
        vbus_in = pd_hal_measure_vbus_in(0);

        if (vbus_in > (uint32_t)apply_threshold(VSAFE_5V, 5))
        {
            if(timeout > 0)
            {
                pd_internal_vbus_in_discharge_on(0);
                timeout--;
            }
            else
            {
                pd_internal_vbus_in_discharge_off(0);
            }
            Cy_SysLib_DelayUs(100);
        }
        else
        {
            pd_internal_vbus_in_discharge_off(0);
            break;
        }
    }
#endif /* (CCG_REG_SEC_CTRL != 0) */

    PDSS->ngdo_1_cfg = (PDSS_NGDO_1_CFG_SEL_ON_OFF |
        PDSS_NGDO_1_CFG_GDRV_EN_ON_VALUE);
}

void turn_off_vbus(void)
{
    PDSS->ngdo_1_cfg = PDSS_NGDO_1_CFG_DEFAULT;
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    (void)port;
    PDSS->dischg_shv_ctrl[0] |= PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    (void)port;
    PDSS->dischg_shv_ctrl[0] &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

void pd_internal_vbus_in_discharge_on(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
    /* This is VBUS_IN discharge path. Only discharge to 5V. */
    PDSS->dischg_shv_ctrl[1] &= ~(PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
    /* Enable the comparator. */
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] &= ~PDSS_COMP_CTRL_COMP_PD;
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_internal_vbus_in_discharge_off(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] |= PDSS_COMP_CTRL_COMP_PD;
    PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */
}

uint16_t pd_hal_measure_vbus_in(uint8_t port)
{
    (void)port;
    uint16_t tmp = 0;

    /* Enable the ADC and power it down. Select REFGEN as source */
    PDSS->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;
    PDSS->adc_ctrl &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;

    /* Select VBUS_IN for voltage measurement */
    PDSS->amux_nhvn_ctrl |= (1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);
    Cy_SysLib_DelayUs(20);

    /* Sample Vbus voltage using ADC. */
    PDSS->intr0 = PDSS_INTR0_SAR_DONE;
    PDSS->adc_ctrl &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;
    PDSS->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N |
        ((APP_VBUS_POLL_ADC_INPUT << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
    PDSS->adc_sar_ctrl |= PDSS_ADC_SAR_CTRL_SAR_EN;
    /* Wait for SAR done interrupt status or timeout. */
    while (((PDSS->intr0 & PDSS_INTR0_SAR_DONE) == 0) && (tmp < PD_ADC_TIMEOUT_COUNT))
    {
        tmp++;
    }
    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    Cy_SysLib_DelayUs(2);
    tmp = ((PDSS->adc_sar_ctrl & PDSS_ADC_SAR_CTRL_SAR_OUT_MASK) >> PDSS_ADC_SAR_CTRL_SAR_OUT_POS);
    PDSS->intr0 = PDSS_INTR0_SAR_DONE;

    /* Convert ADC sample to voltage level */
    tmp = ((tmp * MX_PD_ADC_REF_VOLT_MV * VBUS_MON_DIV_8P_VAL)/(PD_ADC_NUM_LEVELS << 1u));

    /* Revert AMUX */
    PDSS->amux_nhvn_ctrl &= ~(1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);

    return tmp;
}

/* 
 * The function assumes that the EA iDAC is set below 5V and does not 
 * require transition from high to low.
 */
void vbus_set_vsafe(uint8_t port)
{
    (void)port;
    uint32_t regval;

    /* Ensure EA feedback is enabled. */
    PDSS->ea_ctrl |= (PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN);

    /* Initialize VBTR and set initial and final values for sink IDAC */
    PDSS->vbtr_cfg = (PDSS_VBTR_CFG_SRC_EN);
    PDSS->vbtr_snk_init_fin_value = 0;
    regval = CY_USBPD_REG_FIELD_GET(PDSS->ea_ctrl, PDSS_EA_CTRL_ISRC_DAC_CTRL);
    PDSS->vbtr_src_init_fin_value = (regval << PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_INIT_POS);

    /* Clear VBTR status and start the operation */
    PDSS->intr8 = PDSS_INTR8_VBTR_OPR_DONE;
    PDSS->vbtr_ctrl |= PDSS_VBTR_CTRL_START;

    /* Wait untill operation is done */
    while((PDSS->intr8 & PDSS_INTR8_VBTR_OPR_DONE) == 0);

    /* Clear the iDAC settings to make it zero. */
    PDSS->ea_ctrl &= ~(PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK | PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);

    /* Clear VBTR status */
    PDSS->intr8 = PDSS_INTR8_VBTR_OPR_DONE;
    PDSS->vbtr_ctrl &= ~PDSS_VBTR_CTRL_START;

    /* Leave the EA CV mode disabled. */
    PDSS->ea_ctrl &= ~(PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN | PDSS_EA_CTRL_ISNK_EN);
}
#elif defined(CY_DEVICE_CCG5)

void turn_on_vbus(uint8_t port)
{
    PPDSS_REGS_T PDSS = gl_pdss[port];
    uint32_t regval = 0;

    /* Turn on the fet. */
    regval = PDSS->pgdo_1_cfg[1];
    regval |= (PDSS_PGDO_1_CFG_SEL_ON_OFF | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
    PDSS->pgdo_1_cfg[1] = regval;
    set_pd_ctrl_voltage(port, 5000);
}

/* AUTO_MODE control mask for PGDO_1_CFG and PGDO_PU_1_CFG registers. */
#define PGDO_1_CFG_AUTO_SEL_MASK       (PDSS_PGDO_1_CFG_SEL_CSA_OC |\
                                        PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_FILT2_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_CC1_OCP |\
                                        PDSS_PGDO_1_CFG_SEL_CC2_OCP |\
                                        PDSS_PGDO_1_CFG_SEL_CC1_OVP |\
                                        PDSS_PGDO_1_CFG_SEL_CC2_OVP |\
                                        PDSS_PGDO_1_CFG_SEL_SBU1_OVP_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_SBU2_OVP_MASK)
void turn_off_vbus(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pgdo_2_cfg[1]  = 0;
    pd->pgdo_1_cfg[1] &= ~(PGDO_1_CFG_AUTO_SEL_MASK | PDSS_PGDO_1_CFG_AUTO_MODE);

    /* Program PGDO back to its default (OFF) state. */
    pd->pgdo_1_cfg[1] &= ~(PDSS_PGDO_1_CFG_SEL_ON_OFF |
            PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE);
}

#define DISCHG_DRIVE_STRENGTH_VBUS_HI_REVA (0x03u)

void pd_internal_vbus_discharge_on(uint8_t port)
{
    uint32_t regval;
    PPDSS_REGS_T PDSS = gl_pdss[port];  
    /* Enable the VBus discharge circuit. */
    regval = PDSS->dischg_shv_ctrl;
    regval &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK;
    regval |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG |
              (DISCHG_DRIVE_STRENGTH_VBUS_HI_REVA << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
    PDSS->dischg_shv_ctrl = regval;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PPDSS_REGS_T PDSS = gl_pdss[port];
    PDSS->dischg_shv_ctrl &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

#elif (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))

/* Buck-boost PWM configuration macros */
#define BB_PWM_FREQ_KHZ                         (400u)
#define BB_PWM_SS_FREQ_KHZ                      (200u)
#define BB_PWM_SS_DUTY_PER                      (5u)

/* Defines clock cycles for minimum duty cycle possible */
#define BB_HS1_LS2_MIN_DUTY_CYCLE_CLK           (6u)

/* 
 * Defines clock cycles offset from number of BB_CLK cycles 
 * for maximum duty cycle. 
 */
#define BB_HS1_LS2_MAX_DUTY_CYCLE_CLK_OFST      (3u)

#define BB_PWM_PER_TO_CLK(x)                    ((((BBCLK_KHZ) / (BB_PWM_FREQ_KHZ)) * (x))/(100))
#define BB_PWM_SS_PER_TO_CLK(x)                 ((((BBCLK_KHZ) / (BB_PWM_SS_FREQ_KHZ)) * (x))/(100))

/* EA_OUT Pull Down bit position in T_EA field */
#define BB_EA_2_CTRL_BB_EA_T_EA_OUT_PD_POS      (15u)

/* BB Forced Continuous/Current Conduction Mode */
#define BB_MODE_FCCM                            (4u)

/* BB Pulse Skip Mode */
#define BB_MODE_PSM                             (3u)

/* Fixed iLim Gain settings */
#define ILIM_DET_GAIN                           (10u)

/* 
 * Default iLim reference for buck-boost hardware IP.
 * iLim is set to 10A.
 * Default Rsense is 5mOhm.
 * Gain of 10.
 * Reference offset of 200mV considered.
 */
#define ILIM_DET_VREF_SEL7                      (57u)

/* Default RSENSE value is 5mOhm. */
#define LSCSA_DEF_RSENSE                        (50)

/* Minimum VREF allowed in mV */
#define VREF_VOLT_MIN                           (130u)

/* VREF voltage step size in mV. */
#define VREF_VOLT_STEP                          (10u)

#if BOOT_BB_FCCM_MODE_EN
/* LSZCD trim value backup */
static uint32_t gl_lszcd_trim[NO_OF_TYPEC_PORTS];
#endif /* BOOT_BB_FCCM_MODE_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
/* Variable to keep buck-boost soft start PWM duty */
static uint16_t gl_bb_ss_pwm_duty[NO_OF_TYPEC_PORTS];
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

static void pdss_turn_on_pfet(uint8_t port)
{
#if !BOOT_SRC_FET_BYPASS_EN
    /* Provider FET operation is not needed in case of source FET bypass mode */
	    gl_pdss[port]->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
	    Cy_SysLib_DelayUs(50);
	    gl_pdss[port]->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_CP_EN);
	    Cy_SysLib_DelayUs(50);
	    gl_pdss[port]->bb_ngdo_0_gdrv_en_ctrl = (PDSS_BB_NGDO_0_GDRV_EN_CTRL_SEL_ON_OFF |
	            PDSS_BB_NGDO_0_GDRV_EN_CTRL_GDRV_EN_ON_VALUE);
#endif /* !BOOT_SRC_FET_BYPASS_EN */
}

static void pdss_turn_off_pfet(uint8_t port)
{
#if !BOOT_SRC_FET_BYPASS_EN
    /* Provider FET operation is not needed in case of source FET bypass mode */
    gl_pdss[port]->bb_ngdo_0_gdrv_en_ctrl = PDSS_BB_NGDO_0_GDRV_EN_CTRL_DEFAULT;
    Cy_SysLib_DelayUs(50);
    gl_pdss[port]->ngdo_ctrl &= ~(PDSS_NGDO_CTRL_NGDO_CP_EN);
    Cy_SysLib_DelayUs(50);
    gl_pdss[port]->ngdo_ctrl &= ~(PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
#endif /* !BOOT_SRC_FET_BYPASS_EN */
}

void turn_on_vbus(cy_stc_pdstack_context_t *context)
{
    /* Turn ON buck-boost regulation */
    pdss_enable_bb(context);

}

void turn_off_vbus(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
    pdss_turn_off_pfet(port);
    /* Turn OFF buck-boost regulation */
    pdss_disable_bb(context);
}

void pd_internal_vbus_in_discharge_on(uint8_t port)
{
#if VBUS_IN_DISCHARGE_EN
    /* CCG7D is VIN powered, so non comparator based discharge can be enabled */
    gl_pdss[port]->dischg_shv_ctrl[1] |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_internal_vbus_in_discharge_off(uint8_t port)
{
#if VBUS_IN_DISCHARGE_EN
    /* CCG7D is VIN powered, so non comparator based discharge can be enabled */
    gl_pdss[port]->dischg_shv_ctrl[1] = ((gl_pdss[port]->dischg_shv_ctrl[1] & 
        ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    uint32_t reg_val;
    
    /* Enable VBUS_C discharge */

    reg_val = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
    reg_val |= (VBUS_C_DISCHG_DS << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);
    
    gl_pdss[port]->dischg_shv_ctrl[0] = reg_val;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    gl_pdss[port]->dischg_shv_ctrl[0] &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

uint16_t pd_hal_measure_vbus_in(uint8_t port)
{
    uint16_t tmp = 0;
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Enable the ADC and power it down. Select REFGEN as source */
    pd->adc_ctrl[APP_VBUS_POLL_ADC_ID] = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;
    pd->adc_ctrl[APP_VBUS_POLL_ADC_ID] &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;

    /* Select VBUS_IN for voltage measurement */
    pd->amux_nhvn_ctrl |= (1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);
    Cy_SysLib_DelayUs(20);

    /* Sample Vbus voltage using ADC. */
    pd->intr0 = (1 << (PDSS_INTR0_SAR_DONE_POS + APP_VBUS_POLL_ADC_INPUT));
    pd->adc_ctrl[APP_VBUS_POLL_ADC_ID] &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;
    pd->adc_ctrl[APP_VBUS_POLL_ADC_ID] = PDSS_ADC_CTRL_ADC_ISO_N |
        ((APP_VBUS_POLL_ADC_INPUT << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
    pd->adc_sar_ctrl[APP_VBUS_POLL_ADC_ID] |= PDSS_ADC_SAR_CTRL_SAR_EN;
    /* Wait for SAR done interrupt status or timeout. */
    while (((pd->intr0 & (1 << (PDSS_INTR0_SAR_DONE_POS + APP_VBUS_POLL_ADC_ID))) == 0) && (tmp < PD_ADC_TIMEOUT_COUNT))
    {
        tmp++;
    }
    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    Cy_SysLib_DelayUs(2);
    tmp = ((pd->adc_sar_ctrl[APP_VBUS_POLL_ADC_ID] & PDSS_ADC_SAR_CTRL_SAR_OUT_MASK) >> PDSS_ADC_SAR_CTRL_SAR_OUT_POS);
    pd->intr0 = (1 << (PDSS_INTR0_SAR_DONE_POS + APP_VBUS_POLL_ADC_INPUT));

    /* Convert ADC sample to voltage level */
    tmp = ((tmp * MX_PD_ADC_REF_VOLT_MV * VBUS_MON_DIV_8P_VAL)/(PD_ADC_NUM_LEVELS << 1u));

    /* Revert AMUX */
    pd->amux_nhvn_ctrl &= ~(1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);

    return tmp;
}

#if BOOT_BB_STARTUP_WITH_LOAD_EN
/* AMUX settings for < 9V VBUS_IN */

/* 20% VBUS_IN AMUX selection bit for OV comparator */
#define AMUX_NHVN_OV_VBUS_IN_20_PER_POS     (2u)

/* 20% VBUS_IN AMUX selection bit for OV comparator */
#define AMUX_OV_VBUS_IN_20_PER_POS          (7u)

#define VBUS_C_20_PER_DIV                   (5u)

/* OVP min reference voltage in mV. */
#define BB_OVP_REF_VOLT_MIN                 (200u)

/* OVP reference voltage step size in mV. */
#define BB_OVP_REF_VOLT_STEP                (10u)

/* Max. VREF setting. */
#define BB_VREF_MAX_SETTING                 (199u)

/*
 * Function to enable vbus_in voltage comparator.
 * Uses UVP comparator for buck-boost soft start aid.
 * Comparator is set as auto-cut off for buck-boost operation.
 * dir: True for upward change detection and bb cut-off
 *    : False for downward change detection and bb cut-off
 */
static void pd_bb_vbus_in_comp_en(uint8_t port, uint16_t volt, bool dir)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint16_t vref;
    uint32_t regval;
    uint32_t comp_id = (uint32_t)COMP_ID_UV;
    uint32_t filter_id = (uint32_t)FILTER_ID_UV;

    /* Select 20% VBUS_IN to OV comparator instead of VBUS_C. */
    pd->amux_nhvn_ctrl |= (1u << AMUX_NHVN_OV_VBUS_IN_20_PER_POS);
    pd->amux_ctrl |= (1u << AMUX_OV_VBUS_IN_20_PER_POS);

    /* Calculate the actual reference voltage. Cap the value to the max. supported. */
    vref = (((volt / VBUS_C_20_PER_DIV) - BB_OVP_REF_VOLT_MIN) / BB_OVP_REF_VOLT_STEP);

    if(vref > BB_VREF_MAX_SETTING)
    {
        vref = BB_VREF_MAX_SETTING;
    }

    /* UV comparator VREF[2]. */
    regval = pd->refgen_1_ctrl;
    regval &= ~(PDSS_REFGEN_1_CTRL_SEL2_MASK);
    regval |= ((uint32_t)vref << PDSS_REFGEN_1_CTRL_SEL2_POS);
    pd->refgen_1_ctrl = regval;

    /* Turn on comparator. */
    pd->comp_ctrl[comp_id] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[comp_id] &= ~PDSS_COMP_CTRL_COMP_PD;

    Cy_SysLib_DelayUs(10);

    /* Filter configuration. */
    pd->intr5_filter_cfg[filter_id] &= ~(
            PDSS_INTR5_FILTER_CFG_FILT_EN |
            PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK |
            PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
            PDSS_INTR5_FILTER_CFG_DPSLP_MODE |
            PDSS_INTR5_FILTER_CFG_FILT_BYPASS |
            PDSS_INTR5_FILTER_CFG_FILT_RESET);

    /*
     * Set soft start comparator filter to 1uS only as fast response is needed.
     * Subtracting 1 from filter clock cycle value as 0 translates to 1-2
     * clock cycles delay.
     */
    regval = pd->intr5_filter_cfg[filter_id];
    regval |= (((2u - 1u) & 0x1Fu) << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS);
    if (dir == true)
    {
        regval |= ((uint32_t)FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS);
    }
    else
    {
        regval |= ((uint32_t)FILTER_CFG_POS_DIS_NEG_EN << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) |
                   PDSS_INTR5_FILTER_CFG_FILT_RESET ;
    }
    regval |= PDSS_INTR5_FILTER_CFG_FILT_EN | PDSS_INTR5_FILTER_CFG_DPSLP_MODE;

    pd->intr5_filter_cfg[filter_id] = regval;

    /* Clear interrupt. */
    pd->intr5 = ((uint32_t)1u << filter_id);

    /* Enable Interrupt. */
    pd->intr5_mask |= ((uint32_t)1u << filter_id);
}

/* Function to disable vbus_in comparator. */
static void pd_bb_vbus_in_comp_dis(uint8_t port)
{
    /* Disable comparator. */
    gl_pdss[port]->comp_ctrl[COMP_ID_UV] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    gl_pdss[port]->comp_ctrl[COMP_ID_UV] |= PDSS_COMP_CTRL_COMP_PD;
}

/* Functon to get status of vbus_in monitor comparator.
 * Returs : True if comparator as detected change.
 *        : False if comparator has not yet trigerred.
 */
static bool pd_bb_vbus_in_comp_status(uint8_t port)
{
    return ((gl_pdss[port]->intr5_masked & ((uint32_t)1u << (uint32_t)FILTER_ID_UV)) != 0u);
}

static void pdss_bb_ss_pwm_duty_set(uint8_t port, uint16_t duty)
{
    CY_USBPD_REG_FIELD_UPDATE(gl_pdss[port]->bbctrl_buck_sw_ctrl, 
        PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_SS_PW_HS1, 
        (BB_PWM_SS_PER_TO_CLK(duty) - 1));
}
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

#if BOOT_BB_FCCM_MODE_EN
static void pdss_bb_set_mode(uint8_t port, uint8_t mode)
{
    if(mode == BB_MODE_FCCM)
    {
        /* HSRCP and GDRVO configuration for FCCM mode*/
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss_trims[port]->trim_bb_gdrvo_1,
            PDSS_TRIM_BB_GDRVO_1_TRIM_HSRCP, 0x3F);
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss[port]->bb_gdrvo_1_ctrl, 
            PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_T_HSRCP, 0x40);

        gl_pdss[port]->bbctrl_func_ctrl = ((gl_pdss[port]->bbctrl_func_ctrl &
            ~(PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_MASK |
              PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_MASK)) |
             ((uint32_t)0u << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_POS) |
             ((uint32_t)4u << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_POS));

        /* LSZCD and corresponding trims configuration for buck-only FCCM mode. */
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss_trims[port]->trim_bb_gdrvi_1,
        PDSS_TRIM_BB_GDRVI_1_TRIM_LSZCD, 0u);
        gl_pdss[port]->bb_gdrvi_0_ctrl |= (1u <<
        (6u + PDSS_BB_GDRVI_0_CTRL_BB_GDRVI_T_LSZCD_POS));

        /* Disable skip as this is FCCM mode */
        gl_pdss[port]->bb_pwm_1_ctrl &= ~PDSS_BB_PWM_1_CTRL_BB_PWM_EN_SKIP_COMP;
        gl_pdss[port]->bbctrl_func_ctrl2 &= ~PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_DIG_EN;
    }
    else if(mode == BB_MODE_PSM)
    {
        /* Enable skip as this is PSM mode */
        gl_pdss[port]->bb_pwm_1_ctrl |= PDSS_BB_PWM_1_CTRL_BB_PWM_EN_SKIP_COMP;
        gl_pdss[port]->bbctrl_func_ctrl2 |= PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_DIG_EN;

        /* HSRCP and GDRVO configuration for PSM mode */
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss[port]->bb_gdrvo_1_ctrl, 
            PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_T_HSRCP, 0u);
        gl_pdss[port]->bbctrl_func_ctrl = ((gl_pdss[port]->bbctrl_func_ctrl &
            ~(PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_MASK |
              PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_MASK)) |
             ((uint32_t)2u << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_POS) |
             ((uint32_t)4u << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_POS));

        /* LSZCD and corresponding trims configuration for buck-only PSM mode. */
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss_trims[port]->trim_bb_gdrvi_1,
        PDSS_TRIM_BB_GDRVI_1_TRIM_LSZCD, gl_lszcd_trim[port]);
        gl_pdss[port]->bb_gdrvi_0_ctrl &= ~(1u <<
        (6u + PDSS_BB_GDRVI_0_CTRL_BB_GDRVI_T_LSZCD_POS));
    }
}

static void pdss_bb_fccm_en_cb_t(cy_timer_id_t id, void *callbackContext)
{
    (void)id;
    uint8_t port = (((cy_stc_pdstack_context_t *)callbackContext)->port);
    pdss_bb_set_mode(port, BB_MODE_FCCM);
        pdss_turn_on_pfet(port);
}

static void pdss_bb_switch_to_ea(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
#if BOOT_BB_STARTUP_WITH_LOAD_EN
    uint8_t i;
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

    /* Stop soft start PWM */
    gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE;

    /* Release pull down on EA_OUT as voltage has reached threshold */
    gl_pdss[port]->bb_ea_2_ctrl &= ~((uint32_t)1u << BB_EA_2_CTRL_BB_EA_T_EA_OUT_PD_POS <<
        PDSS_BB_EA_2_CTRL_BB_EA_T_EA_POS);

    /* Reset Buck-Boost state machine */
    gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;
    Cy_SysLib_DelayUs(20);
    gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;

    /* Clear all the existing faults in buck-boost latch */
    gl_pdss[port]->bbctrl_func_ctrl3 |= (PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_CLR |
                PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_ILIM_FAULT_DET_CLR);

    /* Enable regular or EA based PWM */
    gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_EA_MODE;

#if BOOT_BB_FORCED_BUCK_EN
    /* Set Soft Start DONE only for 2-switch regulators */
    gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE_DONE;
#endif /* BOOT_BB_FORCED_BUCK_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
    Cy_SysLib_DelayUs(20);

    /* Use software VBTR to reach VSAFE_5V */
    i = BB_SOFT_START_THRES_SRC_IDAC;
    while(i >= BB_STARTUP_VBTR_STEP_SIZE)
    {
        i -= BB_STARTUP_VBTR_STEP_SIZE;
        CY_USBPD_REG_FIELD_UPDATE(gl_pdss[port]->bb_ea_3_ctrl,
            PDSS_BB_EA_3_CTRL_BB_ISRC_DAC_CTRL, i);
        Cy_SysLib_DelayUs(BB_STARTUP_VBTR_STEP_WIDTH_US);
    }
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

#if (VBUS_CTRL_TRIM_ADJUST_ENABLE)
    /* Set trimmed IDAC for VSAFE_5V */
    gl_pdss[port]->bb_ea_3_ctrl = ((gl_pdss[port]->bb_ea_3_ctrl &
        ~(PDSS_BB_EA_3_CTRL_BB_ISRC_DAC_CTRL_MASK |
          PDSS_BB_EA_3_CTRL_BB_ISNK_DAC_CTRL_MASK)) |
         ((uint32_t)BG_ISRC_DAC_CTRL_0_5V(((uint32_t)port)) << PDSS_BB_EA_3_CTRL_BB_ISRC_DAC_CTRL_POS) |
         (((BG_ISNK_DAC_CTRL_COMBINED_5V((port)))) << PDSS_BB_EA_3_CTRL_BB_ISNK_DAC_CTRL_POS));
#endif /* VBUS_CTRL_TRIM_ADJUST_ENABLE */

#if BOOT_BB_FCCM_MODE_EN
    /*
     * Enable Master FCCM mode after a delay to avoid overshoot/undershoot
     * because of negative inductor current.
     */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,
    		GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER), BB_FCCM_EN_DELAY_TIMER_MS, pdss_bb_fccm_en_cb_t);
#else /* !BOOT_BB_FCCM_MODE_EN */
    /* Turn on provider FET once BB startup is completed */
    pdss_turn_on_pfet(port);
#endif /* BOOT_BB_FCCM_MODE_EN */
}
#endif /* BOOT_BB_FCCM_MODE_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
static void pdss_bb_en_with_load_cb_t(cy_timer_id_t id, void *callbackContext)
{
    (void)id;
	cy_stc_pdstack_context_t * context = ((cy_stc_pdstack_context_t *)callbackContext);
    uint8_t port = (context->port);

    /* 
     * Keep monitoring VBUS_IN to reach soft start cut-off voltage.
     * And keep incrementing the soft start duty every 1mS if the voltage 
     * is not building up.
     */
    if(pd_bb_vbus_in_comp_status(port) == false)
    {
        /* 
         * Increase soft start duty and start monitoring again.
         */
        gl_bb_ss_pwm_duty[port] += BB_SOFT_START_DUTY_STEP_UP_PER;

        pdss_bb_ss_pwm_duty_set(port, gl_bb_ss_pwm_duty[port]);
		
		(void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,
				GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER), BB_SOFT_START_MON_TIMER_MS, pdss_bb_en_with_load_cb_t);
    }
    else
    {
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER));
        pd_bb_vbus_in_comp_dis(port);
        gl_bb_ss_pwm_duty[port] = BB_PWM_SS_DUTY_PER;
        pdss_bb_ss_pwm_duty_set(port, gl_bb_ss_pwm_duty[port]);

        /* Switch to EA mode */
        pdss_bb_switch_to_ea(context);
    }
}
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */
#if BOOT_BB_STARTUP_WITH_LOAD_EN
static void pdss_bb_enable_cb_t(cy_timer_id_t id, void *callbackContext)
{
    (void)id;
    cy_stc_pdstack_context_t * context = ((cy_stc_pdstack_context_t *)callbackContext);
    uint8_t port = (context->port);
    
#if BOOT_BB_STARTUP_WITH_LOAD_EN
    /* ADC can have 10% error, so take two samples to catch higher value */
    uint16_t vbus_in = pd_hal_measure_vbus_in(port);

    /* 
     * Keep monitoring VBUS_IN to reach soft start cut-off voltage.
     * And keep incrementing the soft start duty every 1mS if the voltage 
     * is not building up.
     */
    if((vbus_in < BB_VBUS_IN_STARTUP_THRES))
    {
        /* 
         * Increase soft start duty and start monitoring again. 
         * Increment duty by only 2% to avoid overshoot as bootloader 
         * soft start cut off is ADC based polling method and can have latency.
         */
        gl_bb_ss_pwm_duty[port] += 2;

        pdss_bb_ss_pwm_duty_set(port, gl_bb_ss_pwm_duty[port]);

        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,
        		GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER), BB_SOFT_START_MON_TIMER_MS, pdss_bb_enable_cb_t);
    }
    else
    {
    	CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER));
        gl_bb_ss_pwm_duty[port] = BB_PWM_SS_DUTY_PER;
        pdss_bb_ss_pwm_duty_set(port, gl_bb_ss_pwm_duty[port]);

#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

        /* Stop soft start PWM */
        gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE;

        /* Release pull down on EA_OUT as voltage has reached threshold */
        gl_pdss[port]->bb_ea_2_ctrl &= ~(1 << BB_EA_2_CTRL_BB_EA_T_EA_OUT_PD_POS << 
            PDSS_BB_EA_2_CTRL_BB_EA_T_EA_POS);

        /* Reset Buck-Boost state machine */
        gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;
        Cy_SysLib_DelayUs(20);
        gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;

        /* Clear all the existing faults in buck-boost latch */
        gl_pdss[port]->bbctrl_func_ctrl3 |= (PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_CLR |
                    PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_ILIM_FAULT_DET_CLR); 

        /* Enable regular or EA based PWM */
        gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_EA_MODE;

#if BOOT_BB_FORCED_BUCK_EN
        /* Set Soft Start DONE only for 2-switch regulators */
        gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE_DONE;
#endif /* BOOT_BB_FORCED_BUCK_EN */

#if BOOT_BB_FCCM_MODE_EN
        /* 
         * Enable Master FCCM mode after a delay to avoid overshoot/undershoot 
         * because of negative inductor current. 
         */
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,
        		GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER), BB_FCCM_EN_DELAY_TIMER_MS, pdss_bb_fccm_en_cb_t);
#endif /* BOOT_BB_FCCM_MODE_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
    }
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */
}
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

static void pdss_enable_bb(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
#if BOOT_BB_FCCM_MODE_EN
    /* Select PSM mode by default during startup */
    pdss_bb_set_mode(port, BB_MODE_PSM);
#endif /* BOOT_BB_FCCM_MODE_EN */

    /* Do not turn on the Buck-Boost if Vbus-in voltage is not in safe range */
    pd_internal_vbus_in_discharge_on(port);
#if BOOT_BB_STARTUP_WITH_LOAD_EN
    /*
     * Use comparator based buck-boost auto cut-off for load based startup logic.
     * This is needed because duty will be increased based on startup voltage
     * monitor timer and output voltage may overshoot more than required cut-off.
     * Also, continuous polling of comparator status is required along with
     * monitor timer, to make sure soft start to EA mode switch is not delayed.
     * Delayed switch would result in no PWM for some time and output voltage
     * shall drain out. This continuous polling shall result in blocking for few
     * mS. Which can be avoided by using interrupt handler based on requirement.
     *
     * The above logic is not required for noload startup as startup ramp is
     * slow with the default soft start duty and polling based ADC measurement
     * is sufficient.
     */
    pd_bb_vbus_in_comp_en(port, BB_VBUS_IN_STARTUP_VOLT, false);

    while (pd_bb_vbus_in_comp_status(port) == false)
    {
        /* No statement */
    }

#else /* !BOOT_BB_STARTUP_WITH_LOAD_EN */
    while ((pd_hal_measure_vbus_in(port) > BB_VBUS_IN_STARTUP_VOLT))
    {
        /* No statement */
    }
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */
    pd_internal_vbus_in_discharge_off(port);

    /* Reset Buck-Boost state machine */
    gl_pdss[port]->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;
    Cy_SysLib_DelayUs(20);
    gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_RST_CTRL;

    /* Clear all the existing faults in buck-boost latch */
    gl_pdss[port]->bbctrl_func_ctrl3 |= (PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_CLR |
                PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_ILIM_FAULT_DET_CLR); 

#if 0
    /* Do not turn on the Buck-Boost if Vbus-in voltage is not in safe range */
    pd_internal_vbus_in_discharge_on(port);
    while ((pd_hal_measure_vbus_in(port) > BB_VBUS_IN_STARTUP_VOLT));
    pd_internal_vbus_in_discharge_off(port);
#endif

#if BOOT_BB_FORCED_BUCK_EN
    /* 
     * Clear soft start done status before enabling 
     * soft start for 2-switch regulators.
     */
    gl_pdss[port]->bbctrl_func_ctrl &= ~PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE_DONE;
#endif /* BOOT_BB_FORCED_BUCK_EN */

#if BOOT_BB_STARTUP_WITH_LOAD_EN
    /* Set IDAC to soft start cut off voltage */
    CY_USBPD_REG_FIELD_UPDATE(gl_pdss[port]->bb_ea_3_ctrl,
        PDSS_BB_EA_3_CTRL_BB_ISRC_DAC_CTRL, BB_SOFT_START_THRES_SRC_IDAC);

    /* Enable VBUS_IN comparator for soft start cut-off */
    pd_bb_vbus_in_comp_en(port, BB_VBUS_IN_STARTUP_THRES, true);

#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

    /* Soft start enable with buck mode only by default */
    gl_pdss[port]->bbctrl_func_ctrl |=  (PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_EN);

#if BOOT_BB_STARTUP_WITH_LOAD_EN
    /*
     * Start timer to monitor VBUS_IN startup.
     * If voltage is not reaching cut-off voltage, keep increasing duty cycle
     * of soft start PWM.
     */
    (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(context->ptrTimerContext, context,
    		GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER), BB_SOFT_START_MON_TIMER_MS, pdss_bb_en_with_load_cb_t);

    /* Wait until startup reaches cut-off voltage */
    while(pd_bb_vbus_in_comp_status(port) == false)
    {
        /* No statement */
    }

    /* Disable VBUS_IN comparator */
    pd_bb_vbus_in_comp_dis(port);

    /* Disable timer used during BB enable */
    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER));

    /* Reset soft start PWM duty */
    gl_bb_ss_pwm_duty[port] = BB_PWM_SS_DUTY_PER;
    pdss_bb_ss_pwm_duty_set(port, gl_bb_ss_pwm_duty[port]);

#else /* !BOOT_BB_STARTUP_WITH_LOAD_EN */
    /* Wait until startup reaches cut-off voltage */
    while((pd_hal_measure_vbus_in(port) < BB_VBUS_IN_STARTUP_THRES));
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

    /*
     * Call soft start to EA switch as voltage has reached required
     * threshold.
     */
    pdss_bb_switch_to_ea(context);
}

static void pdss_disable_bb(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;
    /* Disable buck-boost */
    gl_pdss[port]->bbctrl_func_ctrl &=  ~(PDSS_BBCTRL_FUNC_CTRL_BBCTRL_STARTUP_MODE |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_EA_MODE |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_EN);

    /* Disable timer used during BB enable */
    CALL_MAP(Cy_PdUtils_SwTimer_Stop)(context->ptrTimerContext, GET_APP_TIMER_ID(context,APP_HAL_GENERIC_TIMER));

    /* 
     * EA_OUT is at VDDD initially.
     * Required startup voltage is around 5V and it corresponds to
     * low EA_OUT. EA takes longer time to reach lower value from VDDD.
     * Hence, pull down EA_OUT to keep EA_OUT at low level initially before
     * enabling EA during buck-boost enable.
     * Pull down is maintained until VBUS voltage reaches required threshold.
     * Pull down is done here to provide enough time for EA_OUT to drop to zero.
     */
    gl_pdss[port]->bb_ea_2_ctrl |= (1 << BB_EA_2_CTRL_BB_EA_T_EA_OUT_PD_POS << 
        PDSS_BB_EA_2_CTRL_BB_EA_T_EA_POS);
}
static void pdss_phy_bb_init(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    /* 
     * EA_OUT is at VDDD initially.
     * Required threshold voltage is around 5V and it corresponds to
     * low EA_OUT. EA takes longer time to reach lower value from VDDD.
     * Hence, pull down EA_OUT to keep EA_OUT at low level initially and enable EA.
     * Pull down is maintained until VBUS voltage reaches required threshold.
     */
    pd->bb_ea_2_ctrl |= (1 << BB_EA_2_CTRL_BB_EA_T_EA_OUT_PD_POS << 
        PDSS_BB_EA_2_CTRL_BB_EA_T_EA_POS);

    /* Buck-Boost controller configuration */

#if BOOT_BB_FORCED_BUCK_EN
    pd->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_FORCE_BUCK_MODE;
#endif /* BOOT_BB_FORCED_BUCK_EN */

    /* Set BB soft start and normal clock. */
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_ff_strtup, 
        PDSS_BBCTRL_FF_STRTUP_BBCTRL_SS_FREQ, 
        FREQ_KHZ_TO_CYCLE_TIME(BB_PWM_SS_FREQ_KHZ));
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_clk_ctrl1, 
        PDSS_BBCTRL_CLK_CTRL1_BBCTRL_CLK_FIX_FREQ, 
        (FREQ_KHZ_TO_CYCLE_TIME(BB_PWM_FREQ_KHZ) - 1));

    /* 
     * Bootloader needs only regulated VSAFE_5V output.
     * Also, bootloader assumes VIN is minimum 5.5V.
     * Hence, need buck configuration only.
     */
    pd->bbctrl_buck_sw_ctrl = ((pd->bbctrl_buck_sw_ctrl & 
        ~(PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MIN_DUTY_CYCLE_MASK | 
        PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MAX_DUTY_CYCLE_MASK |
        PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_SS_PW_HS1_MASK)) |
        ((BB_PWM_SS_PER_TO_CLK(BB_PWM_SS_DUTY_PER) - 1) << 
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_SS_PW_HS1_POS) |
        (BB_HS1_LS2_MIN_DUTY_CYCLE_CLK << 
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MIN_DUTY_CYCLE_POS) |
        ((FREQ_KHZ_TO_CYCLE_TIME(BB_PWM_FREQ_KHZ) - 
            BB_HS1_LS2_MAX_DUTY_CYCLE_CLK_OFST) << 
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MAX_DUTY_CYCLE_POS));
#if BOOT_BB_STARTUP_WITH_LOAD_EN
    gl_bb_ss_pwm_duty[port] = BB_PWM_SS_DUTY_PER;
#endif /* BOOT_BB_STARTUP_WITH_LOAD_EN */

    /* 
     * Configure and enable iLim fault for buck-boost.
     * Buck-Boost shall limit HS1 pulse width if iLim is detected.
     * Firmware need not to handle this fault.
     */

    /* iLim reference configuration */
    CY_USBPD_REG_FIELD_UPDATE(pd->refgen_2_ctrl, PDSS_REFGEN_2_CTRL_SEL7, 
        ILIM_DET_VREF_SEL7);

    /* iLim filter configuration. */
    regval = pd->intr17_cfg_5;
    regval &= ~(PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_EN | 
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_CFG_MASK |
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_SEL_MASK | 
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_RESET |
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_BYPASS);
    regval |= (FILTER_CFG_POS_EN_NEG_EN << PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_CFG_POS) |
        (16u << PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_SEL_POS) |
        PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_EN;
    pd->intr17_cfg_5 = regval;

    /* Disable all faults to BB except iLim. */
    pd->bbctrl_func_ctrl3 |= (PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VIN_UV |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VIN_OV |
#if !BOOT_BB_STARTUP_WITH_LOAD_EN
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VOUT_UV |
#endif /* !BOOT_BB_STARTUP_WITH_LOAD_EN */
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VOUT_OV |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_OCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_SCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_PDS_SCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VREG_INRUSH |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VSRC_NEW_N |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VSRC_NEW_P |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD);

    /* Configure and enable PWM control */
    pd->bb_pwm_0_ctrl |= PDSS_BB_PWM_0_CTRL_BB_PWM_EN_PWMCOMP;
    pd->bb_pwm_1_ctrl |= (PDSS_BB_PWM_1_CTRL_BB_PWM_ISO_N |
        PDSS_BB_PWM_1_CTRL_BB_PWM_EN_MODE_DET |
        PDSS_BB_PWM_1_CTRL_BB_PWM_EN_SKIP_COMP |
        PDSS_BB_PWM_1_CTRL_BB_PWM_ENABLE_PWM |
        PDSS_BB_PWM_1_CTRL_BB_PWM_EN_SKIP_COMP);
    pd->bb_pwm_2_ctrl |= PDSS_BB_PWM_2_CTRL_BB_PWM_EN_VIN_RES;

    /* PWM skip configuration */
    pd->bbctrl_func_ctrl2 = ((pd->bbctrl_func_ctrl2 &
        ~(PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_RST_SW_BLNK_TIM_MASK |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_LOW_MASK |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_HIGH_MASK)) |
        (4 << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_RST_SW_BLNK_TIM_POS) |
        (8 << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_LOW_POS) |
        (12 << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_HIGH_POS) | 
        (PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_VBST_REFRESH_DISABLE) |
        (PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_DIG_EN));

    /* Disable buck and boost refresh and configure ZCD and RCP */
    pd->bbctrl_func_ctrl = ((pd->bbctrl_func_ctrl &
        ~(PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_MASK |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_MASK)) |
        (2 << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVO_RCP_DISABE_POS) |
        (4 << PDSS_BBCTRL_FUNC_CTRL_BBCTRL_GDRVI_ZCD_DISABE_POS) |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_DISABLE_BOOST_REFRESH |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_DISABLE_BUCK_REFRESH);

    /* Set skip comparator reference to 400mV */
    CY_USBPD_REG_FIELD_UPDATE(pd->refgen_3_ctrl, PDSS_REFGEN_3_CTRL_SEL9, 27);
    
    /* Configure and enable 20CSA (VOUT sense). */
    pd->csa_scp_0_ctrl = ((pd->csa_scp_0_ctrl & 
        ~(PDSS_CSA_SCP_0_CTRL_PD_CSA |
        PDSS_CSA_SCP_0_CTRL_AV1_MASK)) |
        (3 << PDSS_CSA_SCP_0_CTRL_AV1_POS) |
        PDSS_CSA_SCP_0_CTRL_CSA_ISO_N);
    pd->csa_scp_1_ctrl |= PDSS_CSA_SCP_1_CTRL_CSA_EN_IBIAS;

    /* Configure and enable 40CSA (VIN sense). */
    /* Update slope compensation by 50% */
    pd->bb_40csa_0_ctrl |= (PDSS_BB_40CSA_0_CTRL_BB_40CSA_EN_SCOMP_DAC_LV |
        PDSS_BB_40CSA_0_CTRL_BB_40CSA_EN_40CSA_LEV_LV);
    pd->bb_40csa_1_ctrl |= (PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_SCOMP_VOUTBYR_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_BIAS_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_40CSA_STG1_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_ENABLE_40CSA_LV);
    pd->bb_40csa_2_ctrl &= ~PDSS_BB_40CSA_2_CTRL_BB_40CSA_PD_ILIMCMP_LV;
    pd->bb_40csa_3_ctrl = (pd->bb_40csa_3_ctrl &
        ~(PDSS_BB_40CSA_3_CTRL_BB_40CSA_SEL_IDAC_LV_MASK)) |
        (24u << PDSS_BB_40CSA_3_CTRL_BB_40CSA_SEL_IDAC_LV_POS) |
        PDSS_BB_40CSA_3_CTRL_BB_40CSA_ISO_N;

    /* Enable Gate drivers */
#if !BOOT_BB_FORCED_BUCK_EN
    pd->bb_gdrvo_0_ctrl = ((pd->bb_gdrvo_0_ctrl &
        ~(PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSDR2_PD |
        PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSRCP_PD |
        PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_KEEPOFF_EN)) |
        PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_VBST_COMP_EN);
    pd->bb_gdrvo_1_ctrl = ((pd->bb_gdrvo_1_ctrl &
        ~(PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_GDRV_PD |
        PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_LSDR2_PD |
        PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_T_HSRCP_MASK)) |
        PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_ISO_N);
#endif /* !BOOT_BB_FORCED_BUCK_EN */
    pd->bb_gdrvi_1_ctrl = ((pd->bb_gdrvi_1_ctrl &
        ~(PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_LSDR1_PD |
        PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_HSDR1_PD |
        PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_LSZCD_PD |
        PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_KEEPOFF_EN)) |
        PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_VBST_COMP_EN);
    pd->bb_gdrvi_2_ctrl = ((pd->bb_gdrvi_2_ctrl &
        ~(PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_GDRV_PD)) |
        PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_ISO_N);

    /* 
     * Filter configuration for gate drivers. Default configuration.
     * This setting is missing in default register values.
     */
    CY_USBPD_REG_FIELD_UPDATE(pd->intr17_cfg_7, 
        PDSS_INTR17_CFG_7_BB_GDRVO_VBST_COMP_OUT_FILT_CFG, 3);
    CY_USBPD_REG_FIELD_UPDATE(pd->intr17_cfg_8, 
        PDSS_INTR17_CFG_8_BB_GDRVI_VBST_COMP_OUT_FILT_CFG, 3);

    /* Keep CC/CF reference to maximum before enabling EA */
    CY_USBPD_REG_FIELD_UPDATE(pd->refgen_3_ctrl, PDSS_REFGEN_3_CTRL_SEL10, 0xFF);

    /* Power up EA block, configuration is done during CV enable */
    pd->bb_ea_0_ctrl = ((pd->bb_ea_0_ctrl &
        ~(PDSS_BB_EA_0_CTRL_BB_EA_PD)) |
        PDSS_BB_EA_0_CTRL_BB_EA_ISO_N |
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CV | 
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CVAMP |
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CCAMP |
        PDSS_BB_EA_0_CTRL_BB_EA_ISRC_EN |
        PDSS_BB_EA_0_CTRL_BB_EA_ISNK_EN);

#if BOOT_BB_FCCM_MODE_EN
    /* 
     * Keep a backup of LSZCD trim, which need to be changed dynamically
     * during PSM and FCCM mode selection.
     */
    gl_lszcd_trim[port] = CY_USBPD_REG_FIELD_GET(gl_pdss_trims[port]->trim_bb_gdrvi_1,
        PDSS_TRIM_BB_GDRVI_1_TRIM_LSZCD);
#endif /* BOOT_BB_FCCM_MODE_EN */

    /* Buck-Boost enable is done when VBUS ON is required */
}
#endif /* CY_DEVICE_CCGx */

void dr_swap_to_UFP(uint8_t port)
{
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];
    /*Send Accept message in response to DR_SWAP request and change data role to UFP */
    pd_send_ctl_msg(port, CY_PD_CTRL_MSG_ACCEPT);
    PDSS_REG->tx_ctrl &= ~DATA_ROLE_DFP;
    gl_cur_port_type = CY_PD_PRT_TYPE_UFP;
}

#endif/* (SOURCE_BOOT) */

#if CC_BOOT_NB_CALL_EN
    
static uint8_t gl_pd_next_state[NO_OF_TYPEC_PORTS];    
static volatile bool gl_set_lock[NO_OF_TYPEC_PORTS];

void pd_fsm_cb_t(cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t * context = ((cy_stc_pdstack_context_t *)callbackContext);
    uint8_t port = (context->port);
    if(gl_pd_next_state[port] == PS_RDY)
    {
        gl_set_lock[port] = false;
        gl_pd_state[port] = PS_RDY;
    }
    else if(gl_pd_next_state[port] == CONNECTED)
    {
        pd_internal_vbus_discharge_off(port);
#if SOURCE_BOOT
        turn_on_vbus(context);
#endif /* SOURCE_BOOT */
        /*Update PD state to CONNECTED*/
        gl_set_lock[port] = false;
        gl_pd_state[port] = CONNECTED;
    }
    else if(gl_pd_next_state[port] == IDLE)
    {
        pd_internal_vbus_discharge_off(port);
        /*Update PD state to IDLE*/
        gl_set_lock[port] = false;
        gl_pd_state[port] = IDLE;
    }
}

#endif /* CC_BOOT_NB_CALL_EN */

void typec_state_machine(cy_stc_pdstack_context_t *ptrPdStackContext)
{  
    uint8_t port = ptrPdStackContext->port;
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];
    
#if CC_BOOT_NB_CALL_EN
    if(gl_set_lock[port] != false)
    {
        return;
    }
#endif

    if (gl_pd_state[port] == IDLE)
    {
        volatile bool status;
        uint8_t i;

        for(i=0; i < 2; i++)
        {
            status = false;

#if (SOURCE_BOOT)
            if(i == 0)
            {
                /* Connect both Down and Up comparator to CC1*/
                PDSS_REG->cc_ctrl_0 &= ~(
                        PDSS_CC_CTRL_0_CMP_DN_CC1V2 |
                        PDSS_CC_CTRL_0_CMP_UP_CC1V2
#if FPGA
                        /* On PSVP, CC_1V2 determines comparator connection as well. */
                        | PDSS_CC_CTRL_0_CC_1V2
#endif /* FPGA */
                        ); 
            }
            else
            {
                /* Connect both Down and Up comparator to CC2*/
                PDSS_REG->cc_ctrl_0 |= (
                        PDSS_CC_CTRL_0_CMP_DN_CC1V2 |
                        PDSS_CC_CTRL_0_CMP_UP_CC1V2
#if FPGA
                        /* On PSVP, CC_1V2 determines comparator connection as well. */
                        | PDSS_CC_CTRL_0_CC_1V2
#endif /* FPGA */
                        );
            }

            /* Let comparator output stabilize. */
#if FPGA
            Cy_SysLib_DelayUs(100);
#else /* !FPGA */
            Cy_SysLib_DelayUs(10);
#endif /* FPGA */

            /* Check if CC line is in 0.2 - 2.6 V range which indicates that Rd is connected. */
            if((PDSS_REG->intr1_status & (PDSS_INTR1_STATUS_VCMP_DN_STATUS |
                            PDSS_INTR1_STATUS_VCMP_UP_STATUS))
                    == PDSS_INTR1_STATUS_VCMP_DN_STATUS)
            {
                status = true;
            }
#else
            if(i== 0)
            {
                /* Connect CC1 to UP Comparator. */
            	PDSS_REG->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CMP_UP_CC1V2;
                Cy_SysLib_DelayUs(10);

                /* Check status of UP comparaor. */
                status = PDSS_REG->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS;
            }
            else
            {
                /* Connect CC2 to UP Comparator. */
            	PDSS_REG->cc_ctrl_0 |= PDSS_CC_CTRL_0_CMP_UP_CC1V2;
                Cy_SysLib_DelayUs(10);

                /* Check status of UP comparaor. */
                status = PDSS_REG->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS;
            }
#endif /* SOURCE_BOOT */

            /* Is CC voltage in connected range. */
            if(status)
            {
                gl_cc_count[port][i]++;
                /* Debounce the status. */
                if(gl_cc_count[port][i] > MAX_DEBOUNCE_ATTACH_COUNT)
                {
                    /* Store active CC id. */
                    gl_active_channel[port] = i;
                    /* Reset debounce count. */
                    gl_cc_count[port][0] = 0;
                    gl_cc_count[port][1] = 0;
                    pd_reset_protocol(port);
                    /*Change PD state from IDLE to CONNECTED.*/
                    gl_pd_state[port] = CONNECTED;

                    /* Select active CC line. */
                    PDSS_REG->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CC_1V2;
                    PDSS_REG->cc_ctrl_0 |= (i << 2);

#if (SOURCE_BOOT)
                    turn_on_vbus(ptrPdStackContext);
#endif /*SOURCE_BOOT*/
                    return;
                }
            }
            else
            {
                /* Reset debounce count. */
                gl_cc_count[port][i] = 0;
            }
        }
    }
    else
    {
        /* Check if CC line is no longer in connected state. */
#if SOURCE_BOOT
        if(PDSS_REG->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS)
#else
        if (!(PDSS_REG->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS))
#endif /* SOURCE_BOOT */
        {
            gl_cc_count[port][gl_active_channel[port]]++;
            /* Detach status debounce. */
            if(gl_cc_count[port][gl_active_channel[port]] > MAX_DEBOUNCE_DETACH_COUNT)
            {
                gl_cc_count[port][0] = 0;
                gl_cc_count[port][1] = 0;
#if (SOURCE_BOOT)
                turn_off_vbus(ptrPdStackContext);
                pd_internal_vbus_discharge_on(port);
#if CC_BOOT_NB_CALL_EN
                gl_pd_next_state[port] = IDLE;
                gl_set_lock[port] = true;
                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        CY_PDSTACK_PD_GENERIC_TIMER, VBUS_DISCHARGE_TIME, pd_fsm_cb_t);
#else    
                Cy_SysLib_Delay(VBUS_DISCHARGE_TIME);
                pd_internal_vbus_discharge_off(port);
#endif /* CC_BOOT_NB_CALL_EN */
#else /* SINK */
                /*
                 * In power bank case, if device is powered by external VDDD, VDDD gets 
                 * shorted to VBUS_IN line. This shall result connecting VDDD to the
                 * Type-C VBUS line. This also includes cases where we start as dead
                 * dead battery device and then get charged. So if any time VBUS has to
                 * be removed in course of PD / Type-C state machine, ensure that internal 
                 * VBUS regulator is disabled. In event of dead battery, this shall lead
                 * to device reset. This is the safest recovery path. CDT 276535.
                 * TODO: Check and disable only if it is enabled. This can be done inside
                 * the API or outside based on other call usage model.
                 * Also, this code can be removed if the VBATT monitoring can be done
                 * continously. But this code can still be in place to avoid any corner
                 * case handling.
                 */
#ifndef CY_DEVICE_CCG7D
                PDSS_REG->vreg_vsys_ctrl &= ~(PDSS_VREG_VSYS_CTRL_VREG_EN);
#endif /* CY_DEVICE_CCG7D */
#endif /* (SOURCE_BOOT) */
#if !CC_BOOT_NB_CALL_EN
                gl_pd_state[port] = IDLE;
#endif /* !CC_BOOT_NB_CALL_EN */                
                return;
            }
        }
        else
        {
            gl_cc_count[port][gl_active_channel[port]] = 0;
        }
    }
}

void pd_state_machine(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint8_t port = ptrPdStackContext->port;
#if SOURCE_BOOT
    PPDSS_REGS_T PDSS_REG = gl_pdss[port];
#endif /* SOURCE_BOOT*/
    uint32_t msg_type ;
    uint32_t count ;
    cy_pd_pd_do_t* d_obj;
    uint8_t no_of_vdo=0;
    cy_pd_pd_do_t *vdm_resp;
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
    uint8_t intr_state;

#if CC_BOOT_NB_CALL_EN
    if(gl_set_lock[port] != false)
    {
        return;
    }
#endif /* CC_BOOT_NB_CALL_EN */

    if(gl_pd_event[port] == CY_PD_PE_EVT_HARD_RESET_RCVD)
    {
        /*Clear PD event*/
        intr_state = Cy_SysLib_EnterCriticalSection();
        gl_pd_event[port] = 0;
        Cy_SysLib_ExitCriticalSection(intr_state);
        pd_reset_protocol(port);

#if (SOURCE_BOOT)
        /*Turn off VBUS. Wait for VBUS to discharge and then turn on VBUS*/
        turn_off_vbus(ptrPdStackContext);
        pd_internal_vbus_discharge_on(port);
#if CC_BOOT_NB_CALL_EN
        gl_pd_next_state[port] = CONNECTED;
        gl_set_lock[port] = true;
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
        		CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,CY_PDSTACK_PD_GENERIC_TIMER), VBUS_DISCHARGE_TIME, pd_fsm_cb_t);
        return;
#else    
        Cy_SysLib_Delay(VBUS_DISCHARGE_TIME);
        pd_internal_vbus_discharge_off(port);
        turn_on_vbus(port);

        /*Update PD state to CONNECTED*/
        gl_pd_state[port] = CONNECTED;
#endif /* CC_BOOT_NB_CALL_EN */        
#endif /* SOURCE_BOOT */
    }
    else if(gl_pd_event[port] == CY_PD_PE_EVT_PKT_RCVD)
    {
        /*Clear PD event*/
        intr_state = Cy_SysLib_EnterCriticalSection();
        gl_pd_event[port] = 0;
        Cy_SysLib_ExitCriticalSection(intr_state);

        msg_type = CY_PD_GET_PD_HDR_TYPE(gl_rcvd_pkt[PD_HDR_IDX]);
        count = CY_PD_GET_PD_HDR_CNT(gl_rcvd_pkt[PD_HDR_IDX]);

        /* Data Message */
        if(count != 0)
        {
#if (SOURCE_BOOT)
            /* Handle RDO request*/
            if(msg_type == CY_PDSTACK_DATA_MSG_REQUEST)
            {
                /* Send Accept message in reponse to RDO*/
                pd_send_ctl_msg(port, CY_PD_CTRL_MSG_ACCEPT);
#if CC_BOOT_NB_CALL_EN
                gl_pd_next_state[port] = PS_RDY;
                gl_set_lock[port] = true;
                (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                		CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,CY_PDSTACK_PD_GENERIC_TIMER), 50u, pd_fsm_cb_t);
                return;
#else    
                Cy_SysLib_Delay(50);
                /* Send PS_RDY message */
                pd_send_ctl_msg(port, CTRL_MSG_PS_RDY);

                /* Change PD state from SEND_SRC_CAP to CONTRACT_ESTD */
                gl_pd_state[port] = CONTRACT_ESTD;
                
#endif /* CC_BOOT_NB_CALL_EN */
            }
#else /* Sink Bootloader*/
            /* Send RDO after receving SRC_CAP message*/
            if(msg_type == CY_PDSTACK_DATA_MSG_SRC_CAP)
            {
                dobject[0].val = 0x10000000;
                pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_REQUEST, dobject,1);
            }
#endif /* SOURCE_BOOT */
            if(msg_type == CY_PDSTACK_DATA_MSG_VDM)
            {
#if (SOURCE_BOOT)
#if !CC_BOOT_IGNORE_DATA_ROLE
                /* if datarole ==  DFP mode,  ignore vdm */
                if(gl_cur_port_type == CY_PD_PRT_TYPE_DFP)
                {
                    return;
                }
#endif /* !CC_BOOT_IGNORE_DATA_ROLE */
#endif /*SOURCE_BOOT*/
                d_obj = (cy_pd_pd_do_t*)(&(gl_rcvd_pkt[1]));
                if(d_obj->std_vdm_hdr.vdmType == true)
                {
                    /* Handle Structured VDMs*/
                    switch(d_obj->std_vdm_hdr.cmd)
                    {
                        case CY_PDSTACK_VDM_CMD_DSC_IDENTITY:
                            dobject[0].val = SVDM_DSC_ID_HEADER;
                            dobject[1].val = SVDM_DSC_ID_HEADER_VDO;
                            dobject[2].val = 0;
                            dobject[3].val = SVDM_DSC_ID_PRODUCT_VDO;
                            pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_VDM, dobject, 4);
                            break;
                        case CY_PDSTACK_VDM_CMD_DSC_SVIDS:
                            dobject[0].val = SVDM_DSC_SVID_HEADER;
                            dobject[1].val = SVDM_DSC_SVID_VDO1;
                            pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_VDM, dobject, 2);
                            break;
                        case CY_PDSTACK_VDM_CMD_DSC_MODES:
                            dobject[0].val = SVDM_DSC_MODE_HEADER;
                            dobject[1].val = SVDM_DSC_MODE_VDO;
                            pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_VDM, dobject, 2);
                            break;
                        case CY_PDSTACK_VDM_CMD_ENTER_MODE:
                            dobject[0].val = SVDM_ENTER_MODE_HEADER;
                            pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_VDM, dobject, 1);
                            break;
                    }
                }
                else
                {
                    /* Handle Un-structured VDMs*/
                    response_state = uvdm_handle_cmd (ptrPdStackContext, gl_rcvd_pkt, &vdm_resp, &no_of_vdo, NULL);
                    /* Respond with UVDM response. */
                    if ((response_state == UVDM_HANDLED_RESPONSE_READY) && (no_of_vdo))
                    {
                        pd_send_data_msg(port, CY_PDSTACK_DATA_MSG_VDM, vdm_resp, no_of_vdo);
                    }
                    /* Respond with NAK if UVDM not recognized and VID is CY VID. */
                    else if (response_state == UVDM_NOT_HANDLED)
                    {
                        /* Check if header has CY VID. */
                        if ((gl_rcvd_pkt[1] >> 16) == CY_PD_CY_VID)
                        {
                            dobject[0].val = gl_rcvd_pkt[1];
                            /* Set NAK . */
                            dobject[0].ustd_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                            pd_send_data_msg (port, CY_PDSTACK_DATA_MSG_VDM, dobject, 1);
                        }
                    }
                }
            }
        }
#if SOURCE_BOOT
        /* Control Message*/
        else
        {
            if(msg_type == CY_PD_CTRL_MSG_DR_SWAP)
            {
                /*Handle DR_SWAP request only when PD contract is established otherwise ignore the request*/
                if(gl_pd_state[port] == CONTRACT_ESTD)
                {
#if CC_BOOT_DR_SWAP_REJECT_ENABLE
                    pd_send_ctl_msg (port, CY_PD_CTRL_MSG_REJECT);
#else
                    /* Handle DR_Swap  only if current data role is DFP */
                    if(gl_cur_port_type == CY_PD_PRT_TYPE_UFP)
                    {
                        pd_send_ctl_msg (port, CY_PD_CTRL_MSG_REJECT);
                    }
                    else
                    {
                        dr_swap_to_UFP(port);
                    }
#endif /* CC_BOOT_DR_SWAP_REJECT_ENABLE */
                }
            }
        }
#endif /*SOURCE_BOOT*/
    }

#if SOURCE_BOOT
    
#if CC_BOOT_NB_CALL_EN
    if (gl_pd_state[port] == PS_RDY)
    {
        /* Send PS_RDY message */
        pd_send_ctl_msg(port, CY_PD_CTRL_MSG_PS_RDY);

        /* Change PD state from SEND_SRC_CAP to CONTRACT_ESTD */
        gl_pd_state[port] = CONTRACT_ESTD;    
    }
    else
#endif /* CC_BOOT_NB_CALL_EN */

    if (gl_pd_state[port] == CONNECTED)
    {
        /* After receiving hard request or detecting sink connection, Source bootloader enters into CONNECTED state
         * and its data role is set to DFP.*/
        PDSS_REG->tx_ctrl |= DATA_ROLE_DFP;
        gl_cur_port_type = CY_PD_PRT_TYPE_DFP;
        /* Move to SEND_SRC_CAP state */
        gl_pd_state[port] = SEND_SRC_CAP;
#if CC_BOOT_NB_CALL_EN
        (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
        		CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,CY_PDSTACK_PD_GENERIC_TIMER), 180u, NULL);
#else    
        /*Start systick timer with timeout 180 ms*/
        start_systick_timer (180);
#endif /* CC_BOOT_NB_CALL_EN */        
    }
    else if (gl_pd_state[port] == SEND_SRC_CAP)
    {
#if CC_BOOT_NB_CALL_EN
        if(CALL_MAP(Cy_PdUtils_SwTimer_IsRunning) (ptrPdStackContext->ptrTimerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,CY_PDSTACK_PD_GENERIC_TIMER)) == false)
        {
            pd_send_data_msg (port, CY_PDSTACK_DATA_MSG_SRC_CAP, src_pdo, 1);
            (void)CALL_MAP(Cy_PdUtils_SwTimer_Start)(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            		CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext,CY_PDSTACK_PD_GENERIC_TIMER), 180u, NULL);
        }
#else    
        /* When systick timer expires, Send SRC_CAP Message with 5V pdo and restart the timer with 180ms.
         * Bootloader sends SRC_CAP message after every 180 ms until it receives RDO request. 
         */        
        if(CM0->syst_csr & CM0_SYST_CSR_COUNTFLAG)
        {
            pd_send_data_msg (port, DATA_MSG_SRC_CAP, src_pdo, 1);
            start_systick_timer (180);
        }
#endif /* CC_BOOT_NB_CALL_EN */        
    }
#endif /*SOURCE_BOOT*/
}

cy_en_usbpd_status_t Cy_PD_HAL_Vconn_Enable(uint8_t port, uint8_t channel)
{

#if (defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regVal;

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
    PPDSS_TRIMS_REGS_T trimRegs = gl_pdss_trims[port];
     /*
     * CDT 360166:
     * Enable VCONN SCP in HW cut off and retry mode initially
     * until VCONN switch is ON. This is to cut off sudden load on VCONN that 
     * can cause Regulator Inrush fault.
     * This protection is enabled by default to address faulty active cables.
     */
    pd->vconn20_ctrl |= ((uint32_t)1u << (7u + PDSS_VCONN20_CTRL_T_VCONN_POS));

    if (channel == CY_PD_CC_CHANNEL_1)
    {
        /* Revert OCP trim offset */
    	CY_USBPD_REG_FIELD_UPDATE(trimRegs->trim_bb_20vconn_2,
            PDSS_TRIM_BB_20VCONN_2_BB_20VCONN_OCP_TRIM, 
            VCONN_OCP_TRIM(port));

        /* Enable SC detection on CC1/2 pins. */
        pd->vconn20_ctrl &= ~PDSS_VCONN20_CTRL_EN_OCP_CC2;
        pd->vconn20_ctrl |= PDSS_VCONN20_CTRL_EN_OCP_CC1;
    }
    else
    {
        /* Set OCP trim offset for CC2 line only */
    	CY_USBPD_REG_FIELD_UPDATE(trimRegs->trim_bb_20vconn_2,
            PDSS_TRIM_BB_20VCONN_2_BB_20VCONN_OCP_TRIM, 
            (uint32_t)VCONN_OCP_TRIM(port) + (uint32_t)VCONN_OCP_TRIM_CC2_OFFSET);

        /* Enable SC detection on CC1/2 pins. */
        pd->vconn20_ctrl &= ~PDSS_VCONN20_CTRL_EN_OCP_CC1;
        pd->vconn20_ctrl |= PDSS_VCONN20_CTRL_EN_OCP_CC2;
    }

#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1)) */

    /* Turn on the VConn switch. */
    if (channel == CY_PD_CC_CHANNEL_1)
    {
        regVal = pd->vconn20_cc1_switch_1_ctrl;
        regVal |= PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_ON_OFF |
            PDSS_VCONN20_CC1_SWITCH_1_CTRL_EN_SWITCH_CC1_ON_VALUE |
            PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_CC1_OVP |
            PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_CC2_OVP;
        pd->vconn20_cc1_switch_1_ctrl = regVal;

        /* Reset edge detector. */
        pd->vconn20_cc1_switch_1_ctrl |= PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc1_switch_1_ctrl &= ~PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
    }
    else
    {
        regVal = pd->vconn20_cc2_switch_1_ctrl;
        regVal |= PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_ON_OFF |
            PDSS_VCONN20_CC2_SWITCH_1_CTRL_EN_SWITCH_CC2_ON_VALUE |
            PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_CC1_OVP |
            PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_CC2_OVP;
        pd->vconn20_cc2_switch_1_ctrl = regVal;

        /* Reset edge detector. */
        pd->vconn20_cc2_switch_1_ctrl |= PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc2_switch_1_ctrl &= ~PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
    }

    /* Turn on the VConn pump. */
    regVal = pd->vconn20_pump_en_1_ctrl;
    regVal |= PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_PUMP_EN_1_CTRL_EN_VCONN20_PUMP_ON_VALUE |
        PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC1_OVP | PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC2_OVP;
    pd->vconn20_pump_en_1_ctrl = regVal;

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1))
     /* 
     * Turn ON & enable the slow charging of 20vconn switch 
     * for inrush current control.
     */
    Cy_SysLib_DelayUs(100);

    /* Turn on VCONN slow switch */
    pd->vconn20_ctrl |= ((uint32_t)1u << (PDSS_VCONN20_CTRL_T_VCONN_SLOW_TURN_ON_POS +
        PDSS_VCONN20_CTRL_T_VCONN_POS));

    /* Start firmware VCONN OCP and SCP handling after VCONN switch output is stable */   
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1)) */
#endif /* (defined(CY_DEVICE_CCG6) || defined(CY_DEVICE_CCG6DF) || defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) || defined(CY_DEVICE_SERIES_WLC1)) */

    return CY_USBPD_STAT_SUCCESS;
}

bool Cy_PD_HAL_Get_Active_CC_Channel(uint8_t port)
{
    return gl_active_channel[port];
}
#endif /* CC_BOOT */
void pd_set_vddd_5v(uint8_t port)
{
	gl_pdss[port]->bb_40vreg_ctrl &= (~PDSS_BB_40VREG_CTRL_BB_40VREG_T_VREG_3P3_MASK);
}
#endif /* CCG_BOOT */
