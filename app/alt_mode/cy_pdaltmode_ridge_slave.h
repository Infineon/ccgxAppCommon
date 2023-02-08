/******************************************************************************
* File Name:   cy_pdaltmode_ridge_slave.h
* \version 2.0
*
* Description: This header file defines the data structures and function
*              prototypes associated with the Intel Alpine/Titan Ridge control
*              interface.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#ifndef CY_PDALTMODE_RIDGE_SLAVE_H
#define CY_PDALTMODE_RIDGE_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdaltmode_defines.h"

/**
* \addtogroup group_pdaltmode_macros
* \{
*/

#define RIDGE_SLAVE_SCB_INDEX                      (0x01u)
/**< Default SCB index used for the Ridge/SoC Slave interface. */

#define RIDGE_SLAVE_SCB_CLOCK_FREQ                 (I2C_SCB_CLOCK_FREQ_1_MHZ)
/**< Maximum I2C clock frequency used on the Ridge/SoC slave interface. */

#define RIDGE_SLAVE_MIN_WRITE_SIZE                 (2u)
/**< Minimum slave write size: Corresponds to slave address + register address. */

#define RIDGE_SLAVE_MAX_WRITE_SIZE                 (16u)
/**< Maximum slave write size: We should never get writes longer than 16 bytes. */

#define RIDGE_SLAVE_MAX_READ_SIZE                  (20u)
/**< Maximum slave read size. */
    
#define RIDGE_SLAVE_READ_BUFFER_SIZE               (4u)
/**< Ridge Slave buffer read size. */

#if ICL_SLAVE_ENABLE

#define PMC_SLAVE_ADDR_PORT0                            (0x50)
/**< Port 0 slave default slave address. */

#define PMC_SLAVE_ADDR_PORT1                            (0x51)
/**< Port 1 slave default slave address. */
    
#define ICL_SOC_ACK_TIMEOUT_PERIOD                      (500u)
/**< t_SOCAckTimeout period defined in PD controller specs from Intel. */

#define RIDGE_SLAVE_ADDR_MASK_GENERAL                   (0x80)
/**< @brief I2C slave address mask to be applied on the incoming slave address. */

#else    

#define RIDGE_SLAVE_ADDR_P0                        (0x38)
/**< Slave address associated with USB-PD port number 0. This is defined by Intel. */

#define RIDGE_SLAVE_ADDR_P1                        (0x3F)
/**< Slave address associated with USB-PD port number 1. This is defined by Intel. */

#define RIDGE_SLAVE_ADDR_MASK                      (0xF0)
/**< @brief I2C slave address mask to be applied on the incoming slave address. */
    
#endif /* ICL_SLAVE_ENABLE */

#define RIDGE_CMD_CCG_RESET                        (0x02)
/**< Alpine/Titan Ridge command value that requests a CCG device reset. */

#define RIDGE_CMD_INT_CLEAR                        (0x04)
/**< Alpine/Titan Ridge command value to clear the interrupt from CCG. */

#define RIDGE_IRQ_ACK                              (0x2000)
/**< Alpine/Titan Ridge command IRQ ACK PD Controller to Titan Ridge. */

#if BB_RETIMER_ENABLE
    
#define RIDGE_CMD_WRITE_TO_RETIMER                 (0x10)
/**< Ice Lake command value to request to write to the retimer */    
    
#define RIDGE_STS_RETIMER_DATA_VALID               (0x400000u)
/**< Ice Lake status bit to indicate that retimer data is valid */
    
#endif /* BB_RETIMER_ENABLE */    

/** \} group_pdaltmode_macros */

/**
* \addtogroup group_pdaltmode_enums
* \{
*/


/** \} group_pdaltmode_enums */

/**
* \addtogroup group_pdaltmode_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_I2cCmdCbk
****************************************************************************//**
*
* I2C command callback function that implements the actual Alpine/Titan Ridge
* interface logic. This is called from SCB interrupt handler. Since the work
* to be done is limited, it is completely handled from the callback.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param count
* Count of bytes written to I2C buffer.
*
* \param write_buf
* Pointer to I2C Write buffer.
*
* \param read_buf
* Pointer to I2C Read buffer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_I2cCmdCbk(cy_stc_pdaltmode_context_t* ptrAltModeContext, uint32_t count,
        uint8_t *write_buf, uint8_t *read_buf);


/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_StatusUpdate
****************************************************************************//**
*
* Update the AR/TR status register and send an event to the Alpine/Titan Ridge.
*
* This function is used by the application layer to update the content of the Alpine/Titan
* Ridge status register. If the content of the register is changing, CCG asserts
* the corresponding interrupt to notify Alpine/Titan Ridge about the status change.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param status
* Value to be written into the status register.
*
* \param rewrite
* Flag to enable updating of the status register even it remains the same.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_StatusUpdate(cy_stc_pdaltmode_context_t *ptrAltModeContext, uint32_t status, bool rewrite);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_DataReset
****************************************************************************//**
*
* Clean the AR/TR status register the Alpine/Titan Ridge.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_DataReset(cy_stc_pdaltmode_context_t* ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_Task
****************************************************************************//**
*
* Handler for pending Ridge Slave interface tasks.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \param write_buf
* Pointer to the write buffer
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_Task(cy_stc_pdaltmode_context_t *ptrAltModeContext,  uint8_t *write_buf);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_UpdateIsPending
****************************************************************************//**
*
* Checks if any PMC/Ridge update is pending
*
* \return
* Mask of updates pending
*
*******************************************************************************/
uint8_t Cy_PdAltMode_RidgeSlave_UpdateIsPending(void);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_IsHostConnected
****************************************************************************//**
*
* Check whether a host is connected to the specified PD port.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* true if a host is connected, false otherwise.
*
*******************************************************************************/
bool Cy_PdAltMode_RidgeSlave_IsHostConnected(cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_GetRidgeHpdState
****************************************************************************//**
*
* Return global HPD status reported by Titan Ridge for the specified PD port.
*
* \param port
* PD port index.
*
* \return
* HPD status HIGH or LOW.
*
*******************************************************************************/
uint8_t Cy_PdAltMode_RidgeSlave_GetRidgeHpdState(uint8_t port);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_SetOcpStatus
****************************************************************************//**
*
* Update the data status register to indicate OverCurrent fault condition.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_SetOcpStatus (cy_stc_pdaltmode_context_t *ptrAltModeContext);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_SocTimeoutEventControl
****************************************************************************//**
*
* Enable or disable HPI notifications about SOC Ack timeout condition. This API is only valid
* on Ice Lake / Tiger Lake platforms and will be a NOP otherwise.
*
* \param disable
* Whether HPI notification on SOC Ack timeout should be disabled.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_SocTimeoutEventControl(bool disable);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_ResetVirtualHpd
****************************************************************************//**
*
* Resets virtual HPD related variables.
*
* \param ptrAltModeContext
* PdAltMode Library Context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_ResetVirtualHpd(cy_stc_pdaltmode_context_t *ptrAltModeContext);

#if BB_RETIMER_ENABLE

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_SetOvnum
****************************************************************************//**
*
* Update the NIDnT overlay number
*
* \param port
* SB-PD port index corresponding to the status update.
*
* \param overlay_num
*  Overlay number
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_SetOvnum(uint8_t port, uint8_t overlay_num);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_SetOvnum_SetDebug
****************************************************************************//**
*
* Update the AR/TR status register with debug info and send an event to the Alpine/Titan Ridge.
*
* \param port
* USB-PD port index corresponding to the status update.
*
* \param debug
* Debug Mode data.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_SetOvnum_SetDebug(uint8_t port, uint32_t debug);


/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_ClearWriteToRetimerBit
****************************************************************************//**
*
*  Clear the Write_To_Retimer bit in the ridge slave cmd register.
*
* \param port
*  USB-PD port index corresponding to the status update.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_ClearWriteToRetimerBit(uint8_t port);

/*******************************************************************************
* Function Name: Cy_PdAltMode_RidgeSlave_DelayedSocAlert
****************************************************************************//**
*
* Enable/disable delayed soc alter notification to the TBT controller.
*
* \param enable
* Whether notifications should be delayed.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_RidgeSlave_DelayedSocAlert(bool enable);

#endif /* BB_RETIMER_ENABLE */

#if STORE_DETAILS_OF_HOST

/*******************************************************************************
* Function Name: Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect
****************************************************************************//**
*
* This function Clear RidgeRdy bit on disconnect.
*
* \param port
* PD Port index.
*
* \return
* None
*
*******************************************************************************/
void Cy_PdAltMode_HostDetails_ClearRidgeRdyBitOnDisconnect(cy_stc_pdaltmode_context_t *ptrAltModeContext);
#endif /* STORE_DETAILS_OF_HOST */

/** \} group_pdaltmode_functions */

#endif /* CY_PDALTMODE_RIDGE_SLAVE_H */

/* [] END OF FILE */
