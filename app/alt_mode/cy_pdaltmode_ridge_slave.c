/******************************************************************************
* File Name:   cy_pdaltmode_ridge_slave.c
* \version 2.0
*
* Description: Alpine/Titan Ridge I2C slave interface source file
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdaltmode_defines.h"
#include "cy_pdutils.h"
#include "cy_pdstack_dpm.h"
#include <cy_pdutils_sw_timer.h>

#include "cy_scb_i2c.h"

#include "cy_pdaltmode_ridge_slave.h"
#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_intel_ridge.h"
#include "cy_pdaltmode_intel_ridge_internal.h"

#if STORE_DETAILS_OF_HOST
#include "cy_pdaltmode_host_details.h"
#endif

#if DEBUG_LOG
#include "debug.h"
#endif

#if BB_RETIMER_ENABLE
#include "bb_retimer.h"
#endif /* BB_RETIMER_ENABLE */

#if RIDGE_SLAVE_ENABLE

#if BB_RETIMER_ENABLE
/* Debug Mode data from retimer for each PD port. */
static volatile uint32_t  ridge_slave_debug_reg[NO_OF_TYPEC_PORTS];

/* NIDnT overlay number for each PD port. */
static volatile uint8_t   ridge_slave_overlay_num[NO_OF_TYPEC_PORTS];
#endif /* BB_RETIMER_ENABLE */

#if ICL_SLAVE_ENABLE
/* Flag indicating that RIDGE soc alert for TBT controller should be delayed. */
static volatile bool      ridge_delay_soc_alert = false;

/* Bit map flag indicating if any port status updates are pending */
static volatile uint8_t pmc_slave_update_pending = 0;
    
/* Function to return pending status */
uint8_t ridge_update_is_pending()
{
    return pmc_slave_update_pending;
}

/* Base address of the slave connected to Ice Lake's PMC */
static uint8_t pmc_base_addr[NO_OF_TYPEC_PORTS] = {
    PMC_SLAVE_ADDR_PORT0
#if PMG1_PD_DUALPORT_ENABLE
    ,
    PMC_SLAVE_ADDR_PORT1
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

#if (!CCG_BACKUP_FIRMWARE)
/* Whether HPI event for SOC Ack Timeout is disabled. */
static bool gl_soc_timeout_evt_disabled = true;
#endif /* CCG_BACKUP_FIRMWARE */ 
#endif /* ICL_SLAVE_ENABLE */

#if BB_RETIMER_ENABLE

/* Function to set the overlay number in the status register */
void ridge_slave_set_ovnum(uint8_t port, uint8_t overlay_num)
{
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        ridge_slave_overlay_num[port] = overlay_num;
    }
}

/* Function to clear the Write_To_Retimer bit in the ridge slave cmd register */
void ridge_slave_clear_write_to_retimer_bit(uint8_t port)
{
   /* Clear the Write_to_Retimer bit */
    ridge_slave_cmd_reg[port]   &= ~(1 << 12);  
}

/* Function called by Alt. Mode layer to update the debug mode 
 * field in the status register. */
void ridge_slave_set_debug(uint8_t port, uint32_t debug)
{
    if(PD_GET_PTR_ICL_TGL_CFG_TBL(port)->icl_dual_retimer_enable != 0)
    {
        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ridge_slave_debug_reg[port] != debug)
        {
            ridge_slave_debug_reg[port]  = debug;
        }
    }    
}
#endif /* BB_RETIMER_ENABLE */

#if (!CCG_BACKUP_FIRMWARE)

#if ICL_SLAVE_ENABLE

#include <hpi.h>
#define HPI_EVENT_SOC_TIMEOUT           (0xC0u)         /* Should match definition in hpi_internal.h */

static void icl_soc_ack_timeout_cb(uint8_t port, uint8_t id)
{
    if (!gl_soc_timeout_evt_disabled)
    {
#if CCG_HPI_ENABLE
        /* Notify EC that there has been a ACK timeout on the SoC side. */
        CALL_MAP(hpi_reg_enqueue_event)(port + 1, HPI_EVENT_SOC_TIMEOUT, 0, 0);
#endif /* CCG_HPI_ENABLE */        
    }
}

void Cy_PdAltMode_RidgeSlave_SocTimeoutEventControl (bool disable)
{
    (void)disable;

#if ICL_SLAVE_ENABLE
    gl_soc_timeout_evt_disabled = disable;
#endif /* ICL_SLAVE_ENABLE */
}
#endif /* ICL_SLAVE_ENABLE */

#endif /* (!CCG_BACKUP_FIRMWARE) */

#if VPRO_WITH_USB4_MODE
void Cy_PdAltMode_RidgeSlave_VproStatusUpdateEnable(cy_stc_pdaltmode_context_t* ptrAltModeContext, bool vpro_value)
{
    /* Only US supports it */
    if ((ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX) &&
            (ptrAltModeContext ->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP))
    {
        cy_stc_pdaltmode_ridge_reg_t temp_val;
        temp_val.val = ptrAltModeContext->ridge.ridge_slave_stat_reg;
        if(vpro_value == true)
        {
            temp_val.ridge_stat.pro_dock_detect = true;
        }
        else
        {
            temp_val.ridge_stat.pro_dock_detect = false;
        }

        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ptrAltModeContext->ridge.ridge_slave_stat_reg != temp_val.val)
        {
            ptrAltModeContext->ridge.ridge_slave_stat_reg  = temp_val.val;
            ptrAltModeContext->ridge.soc_intr_write(1u);
        }
    }
}
#endif

/* Function called by Alt. Mode layer to update the status register. */
void Cy_PdAltMode_RidgeSlave_StatusUpdate(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status, bool rewrite)
{
#if ICL_SLAVE_ENABLE
    uint32_t ridge_status = ridge_slave_stat_reg[port];
    if ((ridge_status != status) || (rewrite))
    {
        ridge_slave_cmd_reg[port]  &= ~RIDGE_CMD_INT_CLEAR;
        /*
         * Update ridge status register before asserting the interrupt signal to
         * avoid false read by SOC.
         */
        ridge_slave_stat_reg[port] = status;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        /* If only HPD IRQ bit has changed then not assert interrupt */
        if (((ridge_status ^ status) != CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) || ((status & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)))
        {
            if (ridge_delay_soc_alert == true)
            {
                if (port == 1)
                {
                    cyhal_gpio_write(glRidgeHwConfig->intrPortA, 0u);
                }
            }
            else
            {
                cyhal_gpio_write(glRidgeHwConfig->intrPortA, 0u);
            }
            pmc_slave_update_pending |= (1 << port);

#if (!CCG_BACKUP_FIRMWARE)
            if (!gl_soc_timeout_evt_disabled)
            {
                if(timer_is_running(port, ICL_SOC_TIMEOUT_TIMER) == false)
                {
                    timer_start(port, ICL_SOC_TIMEOUT_TIMER, ICL_SOC_ACK_TIMEOUT_PERIOD,
                            icl_soc_ack_timeout_cb);
                }
            }
#endif /* (!CCG_BACKUP_FIRMWARE) */
        }
        /* Clear IRQ ACK bit after TR read IRQ high from Status register */
        else if (ridge_status & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)
        {
            ridge_slave_cmd_reg[port] &= (~CY_PDALTMODE_RIDGE_IRQ_ACK_MASK);
        }
#else
        TR_INT_P1_Write (0);
        pmc_slave_update_pending |= (1 << port);
#if (!CCG_BACKUP_FIRMWARE)
        if (!gl_soc_timeout_evt_disabled)
        {
            if(timer_is_running(port, ICL_SOC_TIMEOUT_TIMER) == false)
            {
                timer_start(port, ICL_SOC_TIMEOUT_TIMER, ICL_SOC_ACK_TIMEOUT_PERIOD,
                        icl_soc_ack_timeout_cb);
            }
        }
#endif /* (!CCG_BACKUP_FIRMWARE) */
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

        ridge_slave_stat_reg[port] = status;
    }
#else

#if STORE_DETAILS_OF_HOST
    ptrAltModeContext->hostDetails.back_up_ridge_status = status;

    if(Cy_PdAltMode_HostDetails_CheckIfRidgeNeedsToBeUpdated(ptrAltModeContext, status) == false)
    {
        return;
    }
#endif /* STORE_DETAILS_OF_HOST */

    /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
    if ((ptrAltModeContext->ridge.ridge_slave_stat_reg != status) || (rewrite))
    {
        ptrAltModeContext->ridge.ridge_slave_stat_reg = status;
        ptrAltModeContext->ridge.soc_intr_write(0u);
    }
#endif /* ICL_SLAVE_ENABLE */
}

void Cy_PdAltMode_RidgeSlave_SetOcpStatus (cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    if (ptrAltModeContext->pdStackContext->port < NO_OF_TYPEC_PORTS)
    {
        /* Set only the Data Connection Present and OCP status bits. */
        Cy_PdAltMode_RidgeSlave_StatusUpdate (ptrAltModeContext, (0x09 | (ptrAltModeContext->pdStackContext->dpmConfig.polarity << 1)), false);
    }
}

void Cy_PdAltMode_RidgeSlave_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext,  uint8_t *write_buf)
{
#if (!CCG_BACKUP_FIRMWARE)
    uint32_t cmd;
    uint8_t  regaddrloc = 0;
    uint8_t  intr_state;

    intr_state = Cy_SysLib_EnterCriticalSection ();

    if (ptrAltModeContext->ridge.ridge_slave_task_pending)
    {
        /* Valid write request. The only write-able register is the command register. */
        if (write_buf[regaddrloc] == RIDGE_REG_CCG_COMMAND)
        {

#if BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP
            if (retimer_is_present (gl_ridge_port))
            {
                /* Handle Write to Retimer command. */
                if ((ridge_slave_write_buf[regaddrloc + 3] & RIDGE_CMD_WRITE_TO_RETIMER) != 0)
                {
                    uint32_t wr_data;

                    memcpy ((uint8_t *)&wr_data, &ridge_slave_write_buf[regaddrloc + 4], 4);
                    retimer_start_debug_poll (gl_ridge_port, wr_data, BB_DEBUGMODE_POLL_COUNT, false);
                }
            }
#endif /* BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP */

            /* Handle interrupt clear command. */
            if ((write_buf[regaddrloc + 2u] & RIDGE_CMD_INT_CLEAR) != 0)
            {
#if ICL_SLAVE_ENABLE
                timer_stop(port, ICL_SOC_TIMEOUT_TIMER);
                pmc_slave_update_pending &= ~(1 << gl_ridge_port);

                /* If no interrupts are pending, clear all interrupts */
                if (pmc_slave_update_pending == 0)
                {
                    TR_INT_P1_Write (1);
                }

#else /* !ICL_SLAVE_ENABLE */
                ptrAltModeContext->ridge.soc_intr_write(1u);
#endif /* ICL_SLAVE_ENABLE */

#if VIRTUAL_HPD_ENABLE
                /* Check if clear interrupt is the response to DP event */
#if (VIRTUAL_HPD_DOCK == 0)
                if (
                        (Cy_PdAltMode_Ridge_IsHpdChange(ptrAltModeContext) != false) &&
                        (ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP)
#if (!ICL_ENABLE)
                        &&
                        (ptrAltModeContext->ridge.virtual_hpd_enable)
#endif /* (!ICL_ENABLE) */
                   )
                {
#if BB_RETIMER_ENABLE
                    if (retimer_is_present (gl_ridge_port))
                    {
                        /* Clear IRQ on retimer */
                        if (ridge_slave_stat_reg[port] & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)
                        {
                            retimer_status_update (port, (ridge_slave_stat_reg[port] & (~CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)), true);
                        }
                    }
#endif /* BB_RETIMER_ENABLE */

                    /*
                     * If IRQ_ACKfmTR bit is set then set IRQ_HPDStickyfmTR
                     * and alert Titan Ridge that CCG was notified about successful
                     * Titan Ridge DP IRQ retransmission
                     */
                    if (write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8))
                    {
                        /* Set IRQ_HPDStickyfmTR bit to zero and alert TR */
                        Cy_PdAltMode_RidgeSlave_StatusUpdate(ptrAltModeContext, (ptrAltModeContext->ridge.ridge_slave_stat_reg & (~CY_PDALTMODE_RIDGE_IRQ_ACK_MASK)), false);
                    }
                    else
                    {
                        /* HPD event processed successfully - goto HPD queue */
                        ptrAltModeContext->ridge.hpd_run = true;
                    }
                }
#endif /* (VIRTUAL_HPD_DOCK == 0) */
#endif /* VIRTUAL_HPD_ENABLE */

#if VIRTUAL_HPD_ENABLE
#if BB_RETIMER_ENABLE
                if (retimer_is_present (gl_ridge_port))
                {
                    /* Clear IRQ on retimer */
                    if (ridge_slave_stat_reg[port] & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)
                    {
                        retimer_status_update (port, (ridge_slave_stat_reg[port] & (~CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)), true);
                    }
                }
#endif /* BB_RETIMER_ENABLE */

                /* Set the HPD IRQ state machine state to default upon device disconnect or DP Mode exit*/
                if((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_DP_MODE_MASK) != CY_PDALTMODE_RIDGE_DP_MODE_MASK)
                {
                    ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_DEFAULT_STATE;
                }

                /* Check if clear interrupt is the response to DP event */
                if (
                        (Cy_PdAltMode_Ridge_IsHpdChange(ptrAltModeContext) != false) &&
                        (ptrAltModeContext->pdStackContext->dpmConfig.curPortType != CY_PD_PRT_TYPE_UFP) &&
                        ((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_DP_MODE_MASK) == CY_PDALTMODE_RIDGE_DP_MODE_MASK)  /* Check whether DP mode is active */
                   )
                {
                    /*
                     * If IRQ_ACKfmTR bit is set then set IRQ_HPDStickyfmTR
                     * and alert Titan Ridge that CCG was notified about successful
                     * Titan Ridge DP IRQ retransmission
                     */
                    if (write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8))
                    {
                        if(
                            (ptrAltModeContext->ridge.handling_irq_for_dfp == DFP_IRQ_CLR_BY_CCG)                      &&
                            ((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) == 0)                   &&
                            ((write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8)) != 0) &&
                            ((write_buf[regaddrloc + 2] & RIDGE_CMD_INT_CLEAR) != 0)
                        )
                        {
                            ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_CLR_ACK_BY_TR;
                        }

                        /* Set IRQ_HPDStickyfmTR bit to zero and alert TR */
                        Cy_PdAltMode_RidgeSlave_StatusUpdate(ptrAltModeContext,
                                (ptrAltModeContext->ridge.ridge_slave_stat_reg & (~CY_PDALTMODE_RIDGE_IRQ_ACK_MASK)), false);
                    }
                    else
                    {
                        /* HPD event processed successfully - goto HPD queue */
                        if(ptrAltModeContext->ridge.handling_irq_for_dfp == DFP_IRQ_DEFAULT_STATE)
                        {
                            ptrAltModeContext->ridge.hpd_run = true;
                        }

                        if(
                            ((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) == CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)     &&
                            ((write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8)) == 0) &&
                            ((write_buf[regaddrloc + 2] & RIDGE_CMD_INT_CLEAR) == RIDGE_CMD_INT_CLEAR)
                        )
                        {
                            ptrAltModeContext->ridge.hpd_run = false;
                            ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_SENT_TO_TR;
                        }
                    }
                }
#endif /* VIRTUAL_HPD_ENABLE */
            }
#if VIRTUAL_HPD_DOCK
            else
#endif /* VIRTUAL_HPD_DOCK */
            {
                /* Check if TR command changed from the previous time */
                cmd = CY_PDUTILS_MAKE_DWORD (0, write_buf[regaddrloc + 4], write_buf[regaddrloc + 3], write_buf[regaddrloc + 2]);
                cmd &= ~RIDGE_CMD_INT_CLEAR;

#if VIRTUAL_HPD_ENABLE
#if VIRTUAL_HPD_DOCK
            /* Set global HPD state (CDT 304326) */
                if (cmd & CY_PDALTMODE_RIDGE_HPD_LVL_MASK)
                {
                    /* Set HPD High */
                    ptrAltModeContext->ridge.ridge_hpd_state = true;
                }
                else
                {
                    /* Set HPD Low */
                    ptrAltModeContext->ridge.ridge_hpd_state = false;
                }
#endif /* VIRTUAL_HPD_DOCK */
#endif /* VIRTUAL_HPD_ENABLE */

                if (ptrAltModeContext->ridge.ridge_slave_cmd_reg != cmd)
                {
                    /* Create variable to hold mask of changed command bits */
                    uint32_t tmp_mask = 0;
#if VIRTUAL_HPD_ENABLE
                    if ( (ptrAltModeContext->ridge.ridge_slave_cmd_reg & 0xEF00) != (cmd & 0xEF00) )
                    {
                        if (ptrAltModeContext->pdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP)
                        {
                           if(
                                (ptrAltModeContext->ridge.handling_irq_for_dfp == DFP_IRQ_SENT_TO_TR) &&
                                ((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) == CY_PDALTMODE_RIDGE_HPD_IRQ_MASK)     &&
                                ((write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8)) != 0) &&
                                ((write_buf[regaddrloc + 2] & (RIDGE_CMD_INT_CLEAR)) == 0)
                            )
                            {
#if !ICL_SLAVE_ENABLE
                               ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_CLR_BY_CCG;
#else
                            /* The TigerLake RVP does not send second ACK after the HPD IRQ is cleared */
                               ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_DEFAULT_STATE;
                               ptrAltModeContext->ridge.hpd_run = true;
#endif /* ICL_SLAVE_ENABLE */
                            }
                            else if(
                                (ptrAltModeContext->ridge.handling_irq_for_dfp == DFP_IRQ_CLR_ACK_BY_TR)                   &&
                                ((ptrAltModeContext->ridge.ridge_slave_stat_reg & CY_PDALTMODE_RIDGE_HPD_IRQ_MASK) == 0)                   &&
                                ((write_buf[regaddrloc + 3] & (CY_PDALTMODE_RIDGE_IRQ_ACK_MASK >> 8)) == 0) &&
                                ((write_buf[regaddrloc + 2] & (RIDGE_CMD_INT_CLEAR)) == 0)
                            )
                            {
                                ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_DEFAULT_STATE;
                                ptrAltModeContext->ridge.hpd_run = true;
                            }
                        }
                    }
#endif /* VIRTUAL_HPD_ENABLE */

                    /* Create bit mask of changed command bits */
                    tmp_mask = (ptrAltModeContext->ridge.ridge_slave_cmd_reg ^ cmd);

                    /* Save received Titan Ridge command */
                    ptrAltModeContext->ridge.ridge_slave_cmd_reg = cmd;

                    /* Workaround to not set clear read bit if there is no connection on US */
                    if(ptrAltModeContext->pdStackContext->port == TYPEC_PORT_0_IDX)
                    {
                        /* Do not set the Ridge RDY bit if there is NO host on US */
                        if((ptrAltModeContext->ridge.ridge_slave_stat_reg & RIDGE_SAFE_STATE_MASK) == 0)
                        {
                            ptrAltModeContext->ridge.ridge_slave_cmd_reg &= (~CY_PDALTMODE_RIDGE_RDY_BIT_MASK);
                        }
                    }

                    /* Evaluate received command register update. */
                    Cy_PdAltMode_Ridge_EvalCmd(ptrAltModeContext, ptrAltModeContext->ridge.ridge_slave_cmd_reg, tmp_mask);
                }
            }
        }

        /* Configure write buffer for the next write */
        ptrAltModeContext->ridge.soc_i2c_wr_reconfigure();
        ptrAltModeContext->ridge.ridge_slave_task_pending = false;
    }

    Cy_SysLib_ExitCriticalSection(intr_state);

#if VIRTUAL_HPD_ENABLE
    if (
#if (!ICL_ENABLE)
            (ptrAltModeContext->ridge.virtual_hpd_enable) &&
#endif /* (!ICL_ENABLE) */
            (ptrAltModeContext->ridge.hpd_run != false)
       )
    {
        Cy_PdAltMode_Ridge_HpdSendEvt(ptrAltModeContext, CY_USBPD_HPD_COMMAND_DONE);
        ptrAltModeContext->ridge.hpd_run = false;
    }
#endif /* VIRTUAL_HPD_ENABLE */
#endif /* (!CCG_BACKUP_FIRMWARE) */
}

void Cy_PdAltMode_RidgeSlave_I2cCmdCbk(cy_stc_pdaltmode_context_t* ptrAltModeContext, uint32_t count, uint8_t *write_buf, uint8_t *read_buf)
{
    uint8_t regaddrloc = 0;

    count++;

    /* If we get a single byte only, allow the existing data in the read buf to be read. */
    switch (count)
    {
        case 0:
        case 1:
            /* This will never happen. */
            break;
        case 2:
        case 3:
            /* Only address has been set. Set the read pointer and return. */
            {
                /* Start with an invalid address. */
                /* Clear the read data to start with. */
                memset ((void *)read_buf, 0, sizeof (read_buf));
                switch (write_buf[regaddrloc])
                {
#if ICL_SLAVE_ENABLE
                    case RIDGE_REG_CONTROLLER_STATE:
                        read_buf[0] = 0x04;
                        read_buf[1] = 'A';
                        read_buf[2] = 'P';
                        read_buf[3] = 'P';
                        read_buf[4] = ' ';
                        break;
#endif /* ICL_SLAVE_ENABLE */

                    case RIDGE_REG_CCG_COMMAND:
                        /* Copy in the relevant command register value. */
                        read_buf[0] = RIDGE_SLAVE_READ_BUFFER_SIZE;
                        read_buf[1] = DWORD_GET_BYTE0 (ptrAltModeContext->ridge.ridge_slave_cmd_reg);
                        read_buf[2] = DWORD_GET_BYTE1 (ptrAltModeContext->ridge.ridge_slave_cmd_reg);
                        read_buf[3] = DWORD_GET_BYTE2 (ptrAltModeContext->ridge.ridge_slave_cmd_reg);
                        read_buf[4] = DWORD_GET_BYTE3 (ptrAltModeContext->ridge.ridge_slave_cmd_reg);
                        break;

                    case RIDGE_REG_CCG_STATUS:
                        /* Copy in the relevant command register value. */
                        read_buf[0] = RIDGE_SLAVE_READ_BUFFER_SIZE;
                        read_buf[1] = DWORD_GET_BYTE0 (ptrAltModeContext->ridge.ridge_slave_stat_reg);
                        read_buf[2] = DWORD_GET_BYTE1 (ptrAltModeContext->ridge.ridge_slave_stat_reg);
                        read_buf[3] = DWORD_GET_BYTE2 (ptrAltModeContext->ridge.ridge_slave_stat_reg);
                        read_buf[4] = DWORD_GET_BYTE3 (ptrAltModeContext->ridge.ridge_slave_stat_reg);
                        break;

#if BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP
                case RIDGE_REG_RETIMER_DEBUG:
                    if (pf_conf->icl_dual_retimer_enable != 0)
                    {
                        ridge_slave_read_buf[0] = RIDGE_SLAVE_READ_BUFFER_SIZE;
#if ICL_SLAVE_ENABLE
                        memcpy(&ridge_slave_read_buf[1], (uint8_t*)&ridge_slave_debug_reg[gl_port], 4);
#else
                        read_buf[1] = DWORD_GET_BYTE0 (ridge_slave_debug_reg[gl_port]);
                        read_buf[2] = DWORD_GET_BYTE1 (ridge_slave_debug_reg[gl_port]);
                        read_buf[3] = DWORD_GET_BYTE2 (ridge_slave_debug_reg[gl_port]);
                        read_buf[4] = DWORD_GET_BYTE3 (ridge_slave_debug_reg[gl_port]);
#endif /* ICL_SLAVE_ENABLE */
                    }
                    break;
#endif /* BB_RETIMER_ENABLE && BB_RETIMER_DEBUG_MODE_SUPP */
                    case RIDGE_UUID_REG:
                        read_buf[0] = 0x10;
                        memcpy(&read_buf[1], ptrAltModeContext->ridge.ridge_reg_uuid, 16);
                        break;

                    default:
                        break;
                }
            }
            break;

        default:
            /* Write command should be handles in ridge_slave_task */
            ptrAltModeContext->ridge.ridge_slave_task_pending = true;
            break;
    }
}

#if VIRTUAL_HPD_ENABLE
void Cy_PdAltMode_RidgeSlave_ResetVirtualHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_DEFAULT_STATE;
    ptrAltModeContext->ridge.hpd_run = false;
}
#endif /* VIRTUAL_HPD_ENABLE */

void Cy_PdAltMode_RidgeSlave_DataReset(cy_stc_pdaltmode_context_t* ptrAltModeContext)
{
    if(ptrAltModeContext == NULL)
    {
        return;
    }
#if VIRTUAL_HPD_ENABLE
    ptrAltModeContext->ridge.handling_irq_for_dfp = DFP_IRQ_DEFAULT_STATE;
#endif /* VIRTUAL_HPD_ENABLE */

    /* Clean ridge status and command registers */
    ptrAltModeContext->ridge.ridge_slave_stat_reg   = CY_PDALTMODE_NO_DATA;
    ptrAltModeContext->ridge.ridge_slave_cmd_reg    = CY_PDALTMODE_NO_DATA;

    /* De-assert the interrupt at start-up. */
    ptrAltModeContext->ridge.soc_intr_write(1u);

#if VIRTUAL_HPD_ENABLE
    Cy_PdAltMode_RidgeSlave_ResetVirtualHpd(ptrAltModeContext);

#if (!ICL_ENABLE)
    if (Cy_PdAltMode_HW_IsHostHpdVirtual(ptrAltModeContext))
    {
        ptrAltModeContext->ridge.virtual_hpd_enable = true;
    }
#endif /* (!ICL_ENABLE) */
#endif /* VIRTUAL_HPD_ENABLE */
}

#if ICL_SLAVE_ENABLE
void Cy_PdAltMode_RidgeSlave_DelayedSocAlert(bool enable)
{
    ridge_delay_soc_alert = enable;
}
#endif

bool Cy_PdAltMode_RidgeSlave_IsHostConnected(cy_stc_pdaltmode_context_t *ptrAltModeContext)
{
    bool ret = false;

    /* Return whether the TBT_Host_Connected bit has been set. */
    if ((ptrAltModeContext->ridge.ridge_slave_cmd_reg & CY_PDALTMODE_TBT_HOST_CONN_MASK) != 0)
    {
        ret = true;
    }

    return ret;
}
#endif /* RIDGE_SLAVE_ENABLE */

/* [] END OF FILE */

