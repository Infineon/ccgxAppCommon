/******************************************************************************
* File Name:   i2c.c
*
* Description: I2C slave driver source file.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "cy_device_headers.h"
#include "i2c.h"
#include "cy_pdstack_utils.h"
#include "cy_sw_timer.h"
#include "cy_sw_timer_id.h"
#include "system.h"
#include "srom.h"
#include "app.h"

/* Preamble size of read command. */
#define I2C_SCB_PREAMBLE_SIZE (0x02u)
#define I2C_SCB_TX_TRIGGER    (0x04u)
#define I2C_SCB_RX_TRIGGER    (0x04u)

/* Legacy register default addresses */
#define SCB_TX_CTRL_DEFAULT                                 (0x00000107u)
#define SCB_RX_CTRL_DEFAULT                                 (0x00000107u)
#define SCB_I2C_CTRL_DEFAULT                                (0x0000fb88u)
#define SCB_I2C_S_CMD_DEFAULT                               (0x00u)

/* Global variable to keep track of solution level functions */
extern app_sln_handler_t *solution_fn_handler;

cy_stc_pdstack_context_t * Get_PdStack_Context(uint8_t portIdx)
{
    return solution_fn_handler->Get_PdStack_Context(portIdx);
}

#if (!(SROM_CODE_SCB_I2C))

/* SCB register structures for direct access. */
/* QAC suppression 6004: Global variables pointing to registers are exempt from this namerule check. */
CySCB_Type * const SCB_PRT[I2C_BLOCK_COUNT] = /* PRQA S 6004 */
{
    SCB0,
#if (I2C_BLOCK_COUNT > 1)
    SCB1,
#endif /* (I2C_BLOCK_COUNT > 1) */
#if (I2C_BLOCK_COUNT > 2)
    SCB2,
#endif /* (I2C_BLOCK_COUNT > 2) */
#if (I2C_BLOCK_COUNT > 3)
    SCB3
#endif /* (I2C_BLOCK_COUNT > 3) */
};

/* User-provided I2C configuration for each SCB block. */
static i2c_scb_config_t gl_i2c_scb_config[I2C_BLOCK_COUNT];

/* Slave ACK disable for I2C slave blocks. */
static volatile bool i2c_ack_disable[I2C_BLOCK_COUNT];

/* Indicates whether there is a pending slave ACK. */
static volatile bool i2c_ack_pending[I2C_BLOCK_COUNT];

#endif /* (!(SROM_CODE_SCB_I2C)) */

/* Update the SCB TX FIFO trigger level. */
#define i2c_update_tx_trig_level(scb_index,level) \
    CALL_MAP(SCB_PRT)[(scb_index)]->TX_FIFO_CTRL = ((level) & SCB_TX_FIFO_CTRL_TRIGGER_LEVEL_Msk);

/* Update the SCB RX FIFO trigger level. */
#define i2c_update_rx_trig_level(scb_index,level) \
    CALL_MAP(SCB_PRT)[(scb_index)]->RX_FIFO_CTRL = ((level) & SCB_RX_FIFO_CTRL_TRIGGER_LEVEL_Msk);

/*
   Enable the SCB transmitter.
   Note: SCBv2 does not have a transmit enable. Use the FIFO clear instead.
 */
#define i2c_scb_tx_enable(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->TX_FIFO_CTRL &= ~SCB_TX_FIFO_CTRL_CLEAR_Msk;

/*
   Disable the SCB transmitter.
   Note: SCBv2 does not have a transmit enable. Use the FIFO clear instead.
 */
#define i2c_scb_tx_disable(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->TX_FIFO_CTRL |= SCB_TX_FIFO_CTRL_CLEAR_Msk;

/*
   Enable the SCB receiver.
   Note: SCBv2 does not have a receive enable. Use the FIFO clear instead.
 */
#define i2c_scb_rx_enable(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->RX_FIFO_CTRL &= ~SCB_RX_FIFO_CTRL_CLEAR_Msk;

/*
   Disable the SCB receiver.
   Note: SCBv2 does not have a receive enable. Use the FIFO clear instead.
 */
#define i2c_scb_rx_disable(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->RX_FIFO_CTRL |= SCB_RX_FIFO_CTRL_CLEAR_Msk;

/* Enable the SCB interrupt vector. */
#define i2c_scb_vic_int_enable(scb_index) \
    CALL_MAP(__NVIC_EnableIRQ)((IRQn_Type)(scb_0_interrupt_IRQn + (IRQn_Type)(scb_index)))

/* Disable the SCB interrupt vector. */
#define i2c_scb_vic_int_disable(scb_index) \
    CALL_MAP(__NVIC_DisableIRQ)((IRQn_Type)(scb_0_interrupt_IRQn + (IRQn_Type)(scb_index)))

/* ACK the preamble / data byte on the I2C slave interface. */
#define i2c_slave_ack(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->I2C_S_CMD = SCB_I2C_S_CMD_S_ACK_Msk;

/* NAK the preamble / data byte on the I2C slave interface. */
#define i2c_slave_nak(scb_index) \
    CALL_MAP(SCB_PRT)[(scb_index)]->I2C_S_CMD = SCB_I2C_S_CMD_S_NACK_Msk;

/* Slave transfer enable/disable. */
ATTRIBUTES_SCB_I2C void i2c_slave_ack_ctrl(uint8_t scb_index, bool enable)
{
    CALL_MAP(i2c_ack_disable)[scb_index] = enable;

    /* Send any pending slave ACK so that transfer can continue. */
    if ((!enable) && (CALL_MAP(i2c_ack_pending)[scb_index] != false))
    {
        CALL_MAP(i2c_ack_pending)[scb_index] = false;
        i2c_slave_ack (scb_index);
    }
}

/**
 * Resets the I2C state machine.
 * @param scb_index Index of the I2C block to be reset.
 * @return None
 */
ATTRIBUTES_SCB_I2C void i2c_reset(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
    i2c_scb_config_t *i2c_scb_config_p = &(CALL_MAP(gl_i2c_scb_config)[scb_index]);

    /* Disable the SCB block. */
    scb_p->CTRL &= ~SCB_CTRL_ENABLED_Msk;

    /* Reset the state machine. */
    i2c_scb_config_p->i2c_state = I2C_SCB_STATE_IDLE;
    i2c_scb_config_p->i2c_write_count = 0;

    /* Clear and disable unexpected interrupts. */
    scb_p->INTR_TX_MASK  = 0;
    scb_p->INTR_TX       = 0xFFFFFFFFu;
    scb_p->INTR_S_MASK  &= ~(SCB_INTR_S_I2C_STOP_Msk);
    scb_p->INTR_S        = 0xFFFFFFFFu;
    scb_p->INTR_RX       = 0xFFFFFFFFu;
    scb_p->I2C_S_CMD     = SCB_I2C_S_CMD_DEFAULT;

    /* Re-enable the I2C block. */
    scb_p->CTRL |= SCB_CTRL_ENABLED_Msk;
}

/*
   This function reads from the RX FIFO and append into the scratch buffer
   provided. Data is provided to upper layer through callback when the write
   is completed by the master.
 */
ATTRIBUTES_SCB_I2C static bool i2c_scb_read(uint8_t scb_index, uint8_t size, uint8_t *count)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
    i2c_scb_config_t *i2c_scb_config_p = &(CALL_MAP(gl_i2c_scb_config)[scb_index]);
    uint16_t fifoCount;
    uint16_t idx = i2c_scb_config_p->i2c_write_count;

#if (CCG_BOOT == 0)
    /* Make sure we have a buffer to read data into. */
    if (i2c_scb_config_p->buffer == NULL)
    {
        *count = 0;
        return false;
    }
#endif /* (CCG_BOOT == 0) */

    /* Identify the maximum read size. */
    fifoCount = ((uint16_t)scb_p->RX_FIFO_STATUS & SCB_RX_FIFO_STATUS_USED_Msk);
    fifoCount = CY_USBPD_GET_MIN (fifoCount, size);

    /* Offsetting fifo count by current index of scratch buffer. */
    *count     = (uint8_t)fifoCount;
    fifoCount += idx;

#if (CCG_BOOT == 0)
    /* Ensure that scratch buffer does not overflow because of this read.
     * If yes, Reset I2C block. */
    if (fifoCount > i2c_scb_config_p->buf_size)
    {
        return false;
    }
#endif /* (CCG_BOOT == 0) */

    /* Read data into the scratch buffer. */
    for (; idx < fifoCount; idx++)
    {
        i2c_scb_config_p->buffer[idx] = (uint8_t)scb_p->RX_FIFO_RD;
    }

    /* Update the current pointer of scratch buffer. */
    i2c_scb_config_p->i2c_write_count = idx;
    return true;
}

/* Check whether the I2C interface is idle. */
ATTRIBUTES_SCB_I2C bool i2c_scb_is_idle(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
    i2c_scb_config_t *i2c_scb_config_p = &(CALL_MAP(gl_i2c_scb_config)[scb_index]);

    /* Return true if:
     * 1) I2C state is IDLE
     * 2) I2C Bus is idle
     */
    if (((scb_p->I2C_STATUS & SCB_I2C_STATUS_BUS_BUSY_Msk) == 0u) &&
            (i2c_scb_config_p->i2c_state == I2C_SCB_STATE_IDLE)
       )
    {
        return true;
    }

    return false;
}

/*
   Enable the interrupt due to I2C address match, so that device can
   go to sleep waiting for a new master access.
 */
ATTRIBUTES_SCB_I2C void i2c_scb_enable_wakeup(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];

    /* Configure I2C as wakeup source. */
    scb_p->INTR_I2C_EC_MASK |= SCB_INTR_I2C_EC_WAKE_UP_Msk;
}

#if (CCG_BOOT == 0)
/**
 * @brief Timer callback function used to abort I2C transactions that take too
 * much time.
 * @param id Timer ID that is used to specify the SCB block.
 * @param callbackContext
 * @return None.
 */
ATTRIBUTES_SCB_I2C void i2c_timer_cb(cy_timer_id_t id, void *callbackContext)
{
    uint8_t scb_index;
    (void)callbackContext;

    scb_index = id - (uint8_t)I2C_SLAVE_TIMER_BASE;

    CALL_MAP(i2c_reset)(scb_index);
    CALL_MAP(gl_i2c_scb_config)[scb_index].cb_fun_ptr (
            I2C_CB_CMD_TIMEOUT,
            CALL_MAP(gl_i2c_scb_config)[scb_index].i2c_state,
            0);
}
#endif /* (CCG_BOOT == 0) */

/**
 * @brief I2C interrupt handler.
 * @param scb_index The SCB block ID.
 * @return None.
 */
ATTRIBUTES_SCB_I2C static void i2c_scb_intr_handler(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
    i2c_scb_config_t *i2c_scb_config_p = &(CALL_MAP(gl_i2c_scb_config)[scb_index]);
    bool     status = true;
    uint32_t regVal;
    uint8_t  count = 0;

#if (CCG_BOOT == 0)
    bool     peek_last = false;

    /* Handle Wake-Up interrupt. */
    if ((scb_p->INTR_I2C_EC_MASKED & SCB_INTR_I2C_EC_WAKE_UP_Msk) != 0u)
    {
        /* Disable and clear the interrupt. */
        scb_p->INTR_I2C_EC_MASK &= ~SCB_INTR_I2C_EC_MASK_WAKE_UP_Msk;
        scb_p->INTR_I2C_EC       = SCB_INTR_I2C_EC_WAKE_UP_Msk;
    }
#endif /* (CCG_BOOT == 0) */

    /* Handle I2C Slave interrupt. */
    regVal = scb_p->INTR_S_MASKED;
    if ((regVal & (SCB_INTR_S_I2C_ARB_LOST_Msk | SCB_INTR_S_I2C_BUS_ERROR_Msk)) != 0u)
    {
        /*
           In case of Bus error or arbitration error, we need to reset and
           restore the interface. We should not handle any other interrupts.
           The intr_s register will get cleared in i2c_reset function call.
         */
        status = false;
        regVal = 0;

#if (CCG_BOOT == 0)
        CALL_MAP(cy_sw_timer_stop)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index);

#endif /* (CCG_BOOT == 0) */
    }

    /* First handle STOP interrupt. */
    if ((regVal & SCB_INTR_S_I2C_STOP_Msk) != 0u)
    {
        uint8_t read_size = I2C_SCB_RX_FIFO_SIZE;

#if (CCG_BOOT == 0)
#if (!I2C_SLAVE_SINGLE_ADDR)
        /*
         * If the preamble byte gets loaded into the RX FIFO, we need to do
         * special handling. If there are back to back requests, we cannot
         * distinguish this and we need to selectively copy without the address
         * byte. If the address match is in progress, we need to leave this
         * in the FIFO so that the subsequent handlers can take care of it.
         */
        if (i2c_scb_config_p->slave_mask != I2C_SLAVE_ADDR_MASK_DEFAULT)
        {
            read_size = ((uint8_t)scb_p->RX_FIFO_STATUS &
                    SCB_RX_FIFO_STATUS_USED_Msk);
            /*
             * If the bus is busy, it could be due to other communication or due
             * to subsequent preamble byte. If this is preamble, then the address
             * match should happen. The hardware loads the data into RX FIFO and
             * updates count on the 8th SCL rising edge. It sets the ADDR_MATCH
             * interrupt only on the subsequent SCL falling edge. So, we need to
             * wait for one bit slowest bus clock cycle (assuming here as 100KHz).
             * If within this time, an address match is not seen, then we can be
             * sure that the FIFO count read out before this does not contain the
             * preamble byte. We can read out the content. If the address match
             * happens subsequently, the preamble byte will be handled from the
             * subsequent handling. If the address match has happened within this
             * time, then we should leave one byte in the FIFO as this is the
             * preamble byte for the next transfer. We need to read the fifo count
             * again to avoid race conditions.
             */
            if ((scb_p->I2C_STATUS & SCB_I2C_STATUS_BUS_BUSY_Msk) != 0u)
            {
                /* Wait for 20us (expect this to have happened within 5us). */
                CALL_MAP(Cy_SysLib_DelayUs)(20u);
                if ((scb_p->INTR_S & SCB_INTR_S_I2C_ADDR_MATCH_Msk) != 0u)
                {
                    read_size = (((uint8_t)scb_p->RX_FIFO_STATUS &
                                SCB_RX_FIFO_STATUS_USED_Msk) - 1u);
                    peek_last = true;
                }
            }
        }
#endif /* (!I2C_SLAVE_SINGLE_ADDR) */
#endif /* (CCG_BOOT == 0) */

        /* This could be a re-start to initiate a read operation. */
        if (i2c_scb_config_p->i2c_state != I2C_SCB_STATE_READ)
        {
            /* Read all data into the scratch buffer and initiate a callback. */
            /* QAC suppression 2982: Status assignment is redundant in bootloader application but is required
             * for other applications. */
            status = i2c_scb_read(scb_index, read_size, &count); /* PRQA S 2982 */
#if (CCG_BOOT == 0)
            /* status returned by i2c_scb_read is always true if CCG_BOOT is 1. */
            if (status)
#endif /* (CCG_BOOT == 0) */
            {
#if (CCG_BOOT == 0)
                if (peek_last)
                {
                    /* Ensure that we have left only one byte of data in the FIFO. */
                    CALL_MAP(Cy_SysLib_DelayUs)(20);
                    read_size = ((uint8_t)scb_p->RX_FIFO_STATUS &
                            SCB_RX_FIFO_STATUS_USED_Msk);
                    if (read_size > 1u)
                    {
                        (void)i2c_scb_read(scb_index, read_size - 1u, &count);
                    }
                }
#endif /* (CCG_BOOT == 0) */

                i2c_scb_config_p->i2c_state = I2C_SCB_STATE_WRITE;
                status = i2c_scb_config_p->cb_fun_ptr (I2C_CB_CMD_WRITE,
                        i2c_scb_config_p->i2c_state,
                        i2c_scb_config_p->i2c_write_count);
            }

            /* Clear the RX FIFO Trigger as we have drained the data from the buffer. */
            scb_p->INTR_RX = SCB_INTR_RX_TRIGGER_Msk;
        }
#if (CCG_BOOT == 0)
        else
        {
            /* No meaningful status return expected in this case. */
            i2c_scb_config_p->cb_fun_ptr(
                    I2C_CB_CMD_XFER_END,
                    i2c_scb_config_p->i2c_state,
                    0u);
        }
#endif /* (CCG_BOOT == 0) */

        /* Go back to IDLE state. */
        i2c_scb_config_p->i2c_state = I2C_SCB_STATE_IDLE;
        i2c_scb_config_p->i2c_write_count = 0;

        /* Clear the TX FIFO. */
        i2c_scb_tx_disable (scb_index);
        i2c_scb_tx_enable (scb_index);

        /* Disable the TX trigger interrupt. */
        scb_p->INTR_TX_MASK &= ~SCB_INTR_TX_MASK_TRIGGER_Msk;
        scb_p->INTR_TX       = SCB_INTR_TX_TRIGGER_Msk;

        /* Clear the STOP interrupt and disable STOP bit detection. */
        scb_p->INTR_S_MASK &= ~(SCB_INTR_S_I2C_STOP_Msk);
        scb_p->INTR_S       = (SCB_INTR_S_I2C_STOP_Msk | SCB_INTR_S_I2C_START_Msk);

#if (CCG_BOOT == 0)
        CALL_MAP(cy_sw_timer_stop)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index);
#endif /* (CCG_BOOT == 0) */
    }

    if ((regVal & SCB_INTR_S_I2C_ADDR_MATCH_Msk) != 0u)
    {
#if (CCG_BOOT == 0)
        (void)CALL_MAP(cy_sw_timer_start)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, CALL_MAP(Get_PdStack_Context)(0), (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index, I2C_SLAVE_TIMER_PERIOD,
                CALL_MAP(i2c_timer_cb));
        /* Clear the externally clocked I2C address match interrupt. */
        scb_p->INTR_I2C_EC |= SCB_INTR_I2C_EC_WAKE_UP_Msk;
#endif /* (CCG_BOOT == 0) */

        switch (i2c_scb_config_p->i2c_state)
        {
            case I2C_SCB_STATE_IDLE: /* start. */
                /*
                   Handle read request only if Master has first sent
                   a preamble (register address). In that case state will
                   be PREAMBLE. So, this is an error; NAK the preamble.
                 */
                if ((scb_p->I2C_STATUS & SCB_I2C_STATUS_S_READ_Msk) != 0u)
                {
#if (CCG_BOOT == 0)
#if (!I2C_SLAVE_SINGLE_ADDR)
                    /* If the preamble byte is loaded into the FIFO handle it. */
                    if (i2c_scb_config_p->slave_mask !=
                            I2C_SLAVE_ADDR_MASK_DEFAULT)
                    {
                        /*
                         * Currently, the slave address cannot be changed at the
                         * read preamble. The slave address for read preamble
                         * has to match that of the write preamble with the
                         * write data call. So for now just discard the data.
                         */
                        i2c_scb_rx_disable (scb_index);
                        i2c_scb_rx_enable (scb_index);
                    }
#endif /* (!I2C_SLAVE_SINGLE_ADDR) */
#endif /* (CCG_BOOT == 0) */

                    i2c_scb_config_p->i2c_state = I2C_SCB_STATE_READ;

                    /*
                       CCG cannot distinguish between STOP and RESTART.
                       This means that firmware will have to support read
                       without address specified. However, this is not a
                       valid usage scenario and its response behaviour will be
                       defined in the module which initializes the scb.
                     */
                    status = i2c_scb_config_p->cb_fun_ptr(
                        I2C_CB_CMD_READ,
                        i2c_scb_config_p->i2c_state,
                        0);

                    if (status)
                    {
                        /* Enable the TX FIFO Trigger interrupt. */
                        scb_p->INTR_TX      = SCB_INTR_TX_TRIGGER_Msk;
                        scb_p->INTR_TX_MASK = SCB_INTR_TX_MASK_TRIGGER_Msk;
                    }
                }
                else
                {
                    /* Move to write state. */
                    i2c_scb_config_p->i2c_state = I2C_SCB_STATE_WRITE;
                }
                break;

            default:
                /* A preamble at any other state means an error. */
                status = false;
                break;
        }

        /* ACK preamble on success and NAK on failure. */
        if (status == true)
        {
            /* Enable STOP bit detection. */
            scb_p->INTR_S = (SCB_INTR_S_I2C_STOP_Msk | SCB_INTR_S_I2C_START_Msk);
            scb_p->INTR_S_MASK |= (SCB_INTR_S_I2C_STOP_Msk);

            /* ACK the slave request if allowed by the handlers. */
            if (!(CALL_MAP(i2c_ack_disable)[scb_index]))
            {
                i2c_slave_ack(scb_index);
            }
            else
            {
                CALL_MAP(i2c_ack_pending)[scb_index] = true;
            }
        }
        else
        {
            i2c_slave_nak(scb_index);

#if (CCG_BOOT == 0)
            CALL_MAP(cy_sw_timer_stop)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index);
#endif /* (CCG_BOOT == 0) */
        }
    }

    /* Clear the slave interrupts. */
    scb_p->INTR_S = regVal;

    if (status == true)
    {
        /* INTR_TX register. */
        regVal = scb_p->INTR_TX_MASKED;
        if ((regVal & SCB_INTR_TX_TRIGGER_Msk) != 0u)
        {
            /* Call the read callback to add more data to the TX FIFO. */
            status = i2c_scb_config_p->cb_fun_ptr(
                    I2C_CB_CMD_READ, I2C_SCB_STATE_READ, 0);

            if (!status)
            {
                /* Turn off the trigger interrupt. */
                scb_p->INTR_TX_MASK &= ~SCB_INTR_TX_MASK_TRIGGER_Msk;
                scb_p->INTR_TX       = SCB_INTR_TX_TRIGGER_Msk;

                /* Clear the TX FIFO. */
                i2c_scb_tx_disable(scb_index);
                i2c_scb_tx_enable(scb_index);

                i2c_slave_nak(scb_index);

#if (CCG_BOOT == 0)
                CALL_MAP(cy_sw_timer_stop)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index);
#endif /* (CCG_BOOT == 0) */
            }
        }
        scb_p->INTR_TX = regVal;

        /* INTR_RX register. */
        regVal = scb_p->INTR_RX_MASKED;
        if ((regVal & SCB_INTR_RX_TRIGGER_Msk) != 0u)
        {
            /* Read the data from RX FIFO into scratch buffer. */
            status = i2c_scb_read(scb_index, I2C_SCB_RX_FIFO_SIZE, &count);
        }
        scb_p->INTR_RX = regVal;
    }

    /* Error occurred. Reset the block and state machine. */
    if(!status)
    {
        /* Free the bus. */
        CALL_MAP(i2c_reset)(scb_index);

#if (CCG_BOOT == 0)
        CALL_MAP(cy_sw_timer_stop)(CALL_MAP(Get_PdStack_Context)(0)->ptrTimerContext, (uint8_t)I2C_SLAVE_TIMER_BASE + scb_index);
#endif /* (CCG_BOOT == 0) */
    }
}

/* Interrupt handler for SCB0. */
ATTRIBUTES_SCB_I2C void i2c_scb_0_intr_handler(void)
{
    i2c_scb_intr_handler(0);
}

#if (I2C_BLOCK_COUNT > 1)
/* Interrupt handler for SCB1. */
ATTRIBUTES_SCB_I2C void i2c_scb_1_intr_handler(void)
{
    i2c_scb_intr_handler(1);
}
#endif /* (I2C_BLOCK_COUNT > 1) */

#if (I2C_BLOCK_COUNT > 2)
/* Interrupt handler for SCB2. */
ATTRIBUTES_SCB_I2C void i2c_scb_2_intr_handler(void)
{
    i2c_scb_intr_handler(2);
}
#endif /* (I2C_BLOCK_COUNT > 2) */

#if (I2C_BLOCK_COUNT > 3)
/* Interrupt handler for SCB3. */
ATTRIBUTES_SCB_I2C void i2c_scb_3_intr_handler(void)
{
    i2c_scb_intr_handler(3);
}
#endif /* (I2C_BLOCK_COUNT > 3) */

/**
 * Disables and clear all interrupt of the scb block.
 * @param scb_index
 * @return None
 */
ATTRIBUTES_SCB_I2C static void i2c_clear_all_intr(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];

    /* Clear and disable all interrupts. It is easier to
     * clear all interrupts than to do it selectively. */
    scb_p->INTR_S_MASK  = 0;
    scb_p->INTR_S       = 0xFFFFFFFFu;
    scb_p->INTR_TX_MASK = 0;
    scb_p->INTR_TX      = 0xFFFFFFFFu;
    scb_p->INTR_RX_MASK = 0;
    scb_p->INTR_RX      = 0xFFFFFFFFu;
}

/**
 * @brief Configure one of the I2C blocks as required.
 *
 * This API is used to enable and configure one of the I2C blocks for driver operation.
 * Only I2C slave operation is currently supported by the driver as of now.
 
 * The I2C driver is agnostic of the actual data transfer protocol. It reads all data
 * written by the master into a receive buffer provided by the protocol layer. A callback
 * function is used to notify the protocol layer when the write is complete. The receive
 * buffer provided should be big enough to hold the maximum amount of data that the master
 * may provide in a write operation. If the write contains more data than the buffer can
 * hold, the I2C driver will NAK the transaction.
 *
 * Read requests from the I2C master are automatically delayed by clock stretching. A
 * callback function is used to notify the protocol layer that the master is waiting
 * for data. The i2c_scb_write function can be used by the protocol layer to write data
 * into the transmit FIFO in response to the read request.
 *
 * All I2C driver events are generated from interrupt context, and are expected to be
 * handled with care. The protocol layer should defer any long operations to a non-interrupt
 * context.
 * @param ptrPdStackContext Pointer to pdstack context.
 * @param scb_index SCB index being configured for I2C operation.
 * @param mode Desired mode of operation.
 * @param clock_freq Desired I2C clock frequency.
 * @param slave_addr Device address to be used in case of slave operation.
 * @param slave_mask Mask to be applied on for slave address matching.
 * @param cb_fun_ptr Callback function to be called for event notification.
 * @param scratch_buffer Receive buffer used to hold written by master.
 * @param scratch_buffer_size Size of the receive buffer in bytes.
 *
 * @return None
 */
ATTRIBUTES_SCB_I2C void i2c_scb_init(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t scb_index, i2c_scb_mode_t mode,
        i2c_scb_clock_freq_t clock_freq, uint8_t slave_addr,
        uint8_t slave_mask, i2c_cb_fun_t cb_fun_ptr,
        uint8_t *scratch_buffer, uint16_t scratch_buffer_size)
{
    (void)ptrPdStackContext;
    if(scb_index < I2C_BLOCK_COUNT)
    {
        CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
        i2c_scb_config_t *i2c_scb_config_p = &(CALL_MAP(gl_i2c_scb_config)[scb_index]);
        uint32_t regVal;

        scb_p->CTRL = 0;

        /* Configure the I2C block. */
        regVal = (SCB_I2C_CTRL_DEFAULT & (SCB_I2C_CTRL_HIGH_PHASE_OVS_Msk
                    | SCB_I2C_CTRL_LOW_PHASE_OVS_Msk));
        regVal |= SCB_I2C_CTRL_S_GENERAL_IGNORE_Msk | SCB_I2C_CTRL_S_READY_DATA_ACK_Msk |
            SCB_I2C_CTRL_SLAVE_MODE_Msk;

        scb_p->I2C_CTRL = regVal;
        scb_p->I2C_S_CMD = 0u;

         /* Clear the TX and RX FIFO. */
        scb_p->TX_FIFO_CTRL = SCB_TX_FIFO_CTRL_CLEAR_Msk;
        scb_p->RX_FIFO_CTRL = SCB_RX_FIFO_CTRL_CLEAR_Msk;

        /* Clear all interrupts and enable required interrupts. */
        i2c_clear_all_intr(scb_index);

        /* Read the SCB control register value. */
        regVal = scb_p->CTRL;

#if ((CCG_BOOT == 0) && (I2C_SLAVE_SINGLE_ADDR == 0))
        /* Set I2C Slave address. */
        scb_p->RX_MATCH = (((uint32_t)slave_mask << SCB_RX_MATCH_MASK_Pos) | ((uint32_t)slave_addr << 1));

        /* If the mask is not default, we need to store the address also in the FIFO. */
        if (slave_mask != I2C_SLAVE_ADDR_MASK_DEFAULT)
        {
            regVal |= SCB_CTRL_ADDR_ACCEPT_Msk;
        }

        /* Enable externally clocked mode so that wakeup on I2C address match can work. */
        regVal |= SCB_CTRL_EC_AM_MODE_Msk;
#else
        /* Only single slave address supported in Boot configuration. */
        scb_p->RX_MATCH = (((uint32_t)I2C_SLAVE_ADDR_MASK_DEFAULT << SCB_RX_MATCH_MASK_Pos) | ((uint32_t)slave_addr << 1));
#endif /* ((CCG_BOOT == 0) && (I2C_SLAVE_SINGLE_ADDR == 0)) */

        /* Configure the data width and bit order for I2C. */
        scb_p->TX_CTRL = SCB_TX_CTRL_DEFAULT;
        scb_p->RX_CTRL = SCB_RX_CTRL_DEFAULT;

        /* Enable the SCB. */
        scb_p->CTRL = regVal | SCB_CTRL_ENABLED_Msk;

        /* Reset the state machine and store configuration parameters. */
        i2c_scb_config_p->mode            = mode;
        i2c_scb_config_p->clock_freq      = clock_freq;
        i2c_scb_config_p->i2c_state       = I2C_SCB_STATE_IDLE;
        i2c_scb_config_p->i2c_write_count = 0;
        i2c_scb_config_p->buffer          = scratch_buffer;
        i2c_scb_config_p->buf_size        = scratch_buffer_size;
        i2c_scb_config_p->cb_fun_ptr      = cb_fun_ptr;
        i2c_scb_config_p->slave_address   = slave_addr;
        i2c_scb_config_p->slave_mask      = slave_mask;

        /* Update the trigger levels. */
        i2c_update_tx_trig_level(scb_index, I2C_SCB_TX_TRIGGER);
        i2c_update_rx_trig_level(scb_index, I2C_SCB_RX_TRIGGER);

        /* Enable RX and TX blocks. */
        i2c_scb_rx_enable(scb_index);
        i2c_scb_tx_enable(scb_index);

        /* Enable the I2C interrupts. Transmit trigger is enabled as and when needed. */
        scb_p->INTR_S_MASK = (SCB_INTR_S_I2C_ADDR_MATCH_Msk |
                SCB_INTR_S_I2C_ARB_LOST_Msk | SCB_INTR_S_I2C_BUS_ERROR_Msk);
        scb_p->INTR_RX_MASK = SCB_INTR_RX_MASK_TRIGGER_Msk;

        /* Setting interrupt handler for SCB interrupts on all SCBs. */
        switch (scb_index)
        {
            case 0:
                (void)CALL_MAP(Cy_SysInt_SetVector)(scb_0_interrupt_IRQn, CALL_MAP(i2c_scb_0_intr_handler));
                break;

#if (I2C_BLOCK_COUNT > 1)
            case 1:
                (void)CALL_MAP(Cy_SysInt_SetVector)(scb_1_interrupt_IRQn, CALL_MAP(i2c_scb_1_intr_handler));
                break;
#endif /* (I2C_BLOCK_COUNT > 1) */

#if (I2C_BLOCK_COUNT > 2)
            case 2:
                (void)CALL_MAP(Cy_SysInt_SetVector)(scb_2_interrupt_IRQn, CALL_MAP(i2c_scb_2_intr_handler));
                break;
#endif /* (I2C_BLOCK_COUNT > 2) */

#if (I2C_BLOCK_COUNT > 3)
            case 3:
                (void)CALL_MAP(Cy_SysInt_SetVector)(scb_3_interrupt_IRQn, CALL_MAP(i2c_scb_3_intr_handler));
                break;
#endif /* (I2C_BLOCK_COUNT > 3) */

            default:
                /* Do nothing. */
                break;
        }

        /* Enable the interrupt vector only for the relevant SCB. */
        i2c_scb_vic_int_enable(scb_index);
    }
}

ATTRIBUTES_SCB_I2C void i2c_scb_deinit(uint8_t scb_index)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];

    /* Disable the SCB interrupt. */
    i2c_scb_vic_int_disable(scb_index);

    /* Clear the TX and RX FIFOs. */
    scb_p->TX_FIFO_CTRL = SCB_TX_FIFO_CTRL_CLEAR_Msk;
    scb_p->RX_FIFO_CTRL = SCB_RX_FIFO_CTRL_CLEAR_Msk;

    /* Disable the RX and TX blocks. */
    i2c_scb_rx_disable(scb_index);
    i2c_scb_tx_disable(scb_index);

    /* Clear and disable all interrupts. */
    scb_p->INTR_TX_MASK  = 0;
    scb_p->INTR_TX       = 0xFFFFFFFFu;
    scb_p->INTR_S_MASK   = 0;
    scb_p->INTR_S        = 0xFFFFFFFFu;
    scb_p->INTR_RX_MASK  = 0;
    scb_p->INTR_RX       = 0xFFFFFFFFu;

    /* Disable the I2C slave. */
    scb_p->CTRL &= ~SCB_CTRL_ENABLED_Msk;
}

/* Write data into the SCB TX FIFO. */
ATTRIBUTES_SCB_I2C void i2c_scb_write(uint8_t  scb_index, uint8_t *source_ptr, uint8_t  size,
        uint8_t *count)
{
    CySCB_Type* scb_p = CALL_MAP(SCB_PRT)[scb_index];
    uint8_t fifoCount, idx;

    /* Identify the maximum write size. */
    fifoCount = I2C_SCB_FIFO_SIZE - ((uint8_t)scb_p->TX_FIFO_STATUS & SCB_TX_FIFO_STATUS_USED_Msk);

    /* We are transferring less data than can fit in the buffer. */
    if (fifoCount > size)
    {
        i2c_update_tx_trig_level (scb_index, 1u);
        fifoCount = size;
    }
    else
    {
        i2c_update_tx_trig_level (scb_index, I2C_SCB_TX_TRIGGER);
    }

    for (idx = 0; idx < fifoCount; idx++)
    {
        scb_p->TX_FIFO_WR = source_ptr[idx];
    }

    *count = idx;
}
/* [] END OF FILE */

