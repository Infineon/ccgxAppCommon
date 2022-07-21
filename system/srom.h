/******************************************************************************
* File Name: srom.h
*
* Description: SROM Code file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon Common source files
* \{
*/
#ifndef SROM_H_
#define SROM_H_

#include "config.h"
#include <stdio.h>
#include <string.h>
#include "srom_config.h"

#if CCG_SROM_CODE_ENABLE
#ifdef CCG6
#include "srom_vars_ccg6.h"
#elif defined(CY_DEVICE_CCG7D)
#include "srom_vars_ccg7d.h"
#elif defined(CY_DEVICE_CCG7S)
#include "srom_vars_ccg7s.h"
#endif /* CCG6 */
#else /* !CCG_SROM_CODE_ENABLE */
#include "srom_vars_default.h"
#endif /* CCG_SROM_CODE_ENABLE */

#if CCG_SROM_CODE_ENABLE
/**
* ROM/Flash function and variable access macro redirection.
* Macro mapping section in each srom_vars_<device>.h file need to be 
* updated when there is any change in ROM code base.
* Macro mapping sections must include all the CALL_MAP functions of
* the code base. Each call mapping function can be mapped to
* either direct function or variable call 
* or through call_in/call_out function or get_in variable as per the ROM
* code definition of the device. 
*/
#define CALL_MAP(str)  call_##str(str)
#else /* !CCG_SROM_CODE_ENABLE */
/**
* ROM/Flash function and variable access macro redirection.
* Macro mapping section in each srom_vars_<device>.h file need to be 
* updated when there is any change in ROM code base.
* Macro mapping sections must include all the CALL_MAP functions of
* the code base. Each call mapping function can be mapped to
* either direct function or variable call 
* or through call_in/call_out function or get_in variable as per the ROM
* code definition of the device. 
*/
#define CALL_MAP(str)  (str)
#endif /* CCG_SROM_CODE_ENABLE */

#endif /* SROM_H_ */

/** \} group_ccgxAppCommon */
