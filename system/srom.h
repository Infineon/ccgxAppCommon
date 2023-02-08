/******************************************************************************
* File Name: srom.h
* \version 2.0
*
* Description: SROM Code file.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/
/**
* \addtogroup group_ccgxAppCommon App Common Middleware
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
#elif (defined(CY_DEVICE_CCG7D) && !defined(CY_DEVICE_SERIES_WLC1))
#include "srom_vars_ccg7d.h"
#elif defined(CY_DEVICE_SERIES_WLC1)
#include "srom_vars_wlc1.h"
#elif defined(CY_DEVICE_CCG7S)
#include "srom_vars_ccg7s.h"
#endif /* CCGx */
#else /* !CCG_SROM_CODE_ENABLE */
#include "srom_vars_default.h"
#endif /* CCG_SROM_CODE_ENABLE */

#if defined(CY_DEVICE_CCG7D)
/**
* Macro for CCG7D ROM Compatibility. CCG7D call in functions
* take port as paramter while new silicons (CCG7S onwards) take 
* pdstack context as poramter.
*/
    #define ROM_IN_MACRO(x)      (x->port)
#else
/**
* Macro for CCG7D ROM Compatibility. CCG7D call in functions
* take port as paramter while new silicons (CCG7S onwards) take 
* pdstack context as poramter.
*/
    #define ROM_IN_MACRO(x)      (x)
#endif /* defined(CY_DEVICE_CCG7D) */


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
#ifndef CALL_MAP
#define CALL_MAP(str)  (str)
#endif /* CALL_MAP */
#endif /* CCG_SROM_CODE_ENABLE */

#endif /* SROM_H_ */

/** \} group_ccgxAppCommon */
