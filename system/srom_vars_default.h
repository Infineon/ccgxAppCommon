/******************************************************************************
* File Name: srom_vars_default.h
*
* Description: SROM Code default header file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/

#ifndef SROM_VARS_DEFAULT_H_
#define SROM_VARS_DEFAULT_H_

#include "config.h"

#define ATTRIBUTES
#define CALL_OUT_FUNCTION(func_name) func_name
#define CALL_IN_FUNCTION(func_name) func_name
#define GET_IN_VARIABLE(var_name) var_name

#define ATTRIBUTES_SCB_I2C      ATTRIBUTES
#define ATTRIBUTES_APP_PDO      ATTRIBUTES
#define ATTRIBUTES_PD_PROT      ATTRIBUTES
#define ATTRIBUTES_SYS_GPIO     ATTRIBUTES
#define ATTRIBUTES_SYS_SYS      ATTRIBUTES
#define ATTRIBUTES_SYS_FLASH    ATTRIBUTES
#define ATTRIBUTES_HPISS_HPI    ATTRIBUTES
#define ROM_STATIC_ATTRIBUTE    

#define HPI_GLOBAL_VAR
#define HPI_CONST
#define CRYPTO_STATIC_ATTRIBUTE  
#define CRYPTO_ATTRIBUTE 
#define CRYPTO_VAR_ATTRIBUTE 
#define CRYPTO_RSA_VAR_ATTRIBUTE 
#define ROM_CONSTANT    
#endif /* SROM_VARS_DEFAULT_H_ */
