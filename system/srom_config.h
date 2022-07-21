/******************************************************************************
* File Name: srom_config.h
*
* Description: SROM Code configuration file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/

#ifndef SROM_CONFIG_H_
#define SROM_CONFIG_H_

#include "config.h"
#include <stdio.h>
#include <string.h>

/************************************************************************
* Device specific SROM configuration.
* Configurations in this file can be used across modules for ROM function
* pointer or ROM variable declaration.
* Or any device specific ROM workarounds.
************************************************************************/
#if CCG_SROM_CODE_ENABLE
/** @cond DOXYGEN_HIDE */
#ifdef CCG6
#define CCG6_SROM_CODE_ENABLE   (1u)
#define SROM_CODE_SCB_I2C       (1u)
#define SROM_CODE_APP_PDO       (1u)
#define SROM_CODE_PD_PD         (1u)
#define SROM_CODE_PD_PE         (1u)
#define SROM_CODE_PD_PROT       (1u)
#define SROM_CODE_SYS_BOOT      (1u)
#define SROM_CODE_SYS_GPIO      (1u)
#define SROM_CODE_SYS_SYS       (1u)
#define SROM_CODE_CRYPTO        (0u)
#define SROM_CODE_SYS_FLASH     (0u)
#elif defined(CY_DEVICE_CCG7S)
#define CCG7S_SROM_CODE_ENABLE  (1u)
#define SROM_CODE_PD_PD         (0u)
#if GENERATE_SROM_CODE
#define SROM_CODE_APP_PDO       (0u)
#define SROM_CODE_PD_PD         (0u)
#define SROM_CODE_PD_PE         (0u)
#define SROM_CODE_PD_PROT       (0u)
#define SROM_CODE_SYS_BOOT      (0u)
#define SROM_CODE_SYS_SYS       (0u)
#define SROM_CODE_SCB_I2C       (1u)
#define SROM_CODE_SYS_GPIO      (0u)
#define SROM_CODE_SYS_FLASH     (0u)
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_HPISS_HPI     (1u)
#else
#if !CCG_BOOT
#define SROM_CODE_APP_PDO       (0u)
#define SROM_CODE_PD_PD         (0u)
#define SROM_CODE_PD_PE         (0u)
#define SROM_CODE_PD_PROT       (0u)
#define SROM_CODE_SYS_BOOT      (0u)
#define SROM_CODE_SYS_SYS       (0u)
#define SROM_CODE_SCB_I2C       (1u)
#define SROM_CODE_SYS_GPIO      (0u)
#define SROM_CODE_SYS_FLASH     (0u)
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_HPISS_HPI     (1u)
#else
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_SYS_GPIO      (0u)
#define SROM_CODE_SYS_FLASH     (0u)
#define SROM_CODE_HPISS_HPI     (1u)
#if (!CC_BOOT || I2C_BOOT)
#define SROM_CODE_SCB_I2C       (1u)
#endif /* (!CC_BOOT || I2C_BOOT) */
#endif /* !CCG_BOOT */
#endif /* GENERATE_SROM_CODE */
#elif defined(CY_DEVICE_CCG7D)  
#define CCG7D_SROM_CODE_ENABLE  (1u)
#define SROM_CODE_PD_PD         (0u)
#if GENERATE_SROM_CODE
#define SROM_CODE_APP_PDO       (0u)
#define SROM_CODE_PD_PD         (0u)
#define SROM_CODE_PD_PE         (0u)
#define SROM_CODE_PD_PROT       (0u)
#define SROM_CODE_SYS_BOOT      (0u)
#define SROM_CODE_SYS_SYS       (0u)
#define SROM_CODE_SCB_I2C       (1u)
#define SROM_CODE_SYS_GPIO      (1u)    
#define SROM_CODE_SYS_FLASH     (0u)        
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_HPISS_HPI     (1u)
#else
#if !CCG_BOOT
#define SROM_CODE_APP_PDO       (0u)
#define SROM_CODE_PD_PD         (0u)
#define SROM_CODE_PD_PE         (0u)
#define SROM_CODE_PD_PROT       (0u)
#define SROM_CODE_SYS_BOOT      (0u)
#define SROM_CODE_SYS_SYS       (0u)
#define SROM_CODE_SCB_I2C       (1u)
#define SROM_CODE_SYS_GPIO      (1u)    
#define SROM_CODE_SYS_FLASH     (0u)
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_HPISS_HPI     (1u)
#else /* CCG_BOOT */    
#define SROM_CODE_CRYPTO        (1u)
#define SROM_CODE_SYS_GPIO      (1u)
#define SROM_CODE_SYS_FLASH     (0u)
#define SROM_CODE_HPISS_HPI     (1u)
/*
 * This is a temporary hack to allow the CC bootloader to build for the 
 * RSC project and allow the RSE project to build and use the CC + I2C Bootloader.   
 */    
#if (!CC_BOOT || I2C_BOOT)    
#define SROM_CODE_SCB_I2C       (1u)
#endif /* (!CC_BOOT || I2C_BOOT) */    
#endif /* !CCG_BOOT */
#endif /* GENERATE_SROM_CODE */

#else /* ! (CY_DEVICE_CCG7S || CY_DEVICE_CCG7D || CCG6) */
#error "Selected Device does not support SROM code."
#endif

#else /* !CCG_SROM_CODE_ENABLE */
#define SROM_CODE_SCB_I2C       (0u)
#define SROM_CODE_APP_PDO       (0u)
#define SROM_CODE_PD_PD         (0u)
#define SROM_CODE_PD_PE         (0u)
#define SROM_CODE_PD_PROT       (0u)
#define SROM_CODE_SYS_BOOT      (0u)
#define SROM_CODE_SYS_GPIO      (0u)
#define SROM_CODE_SYS_SYS       (0u)
#define SROM_CODE_CRYPTO        (0u)
#define SROM_CODE_SYS_FLASH     (0u)    
#define SROM_CODE_HPISS_HPI     (0u)
#endif /* CCG_SROM_CODE_ENABLE */

#endif /* SROM_CONFIG_H_ */

