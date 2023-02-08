/******************************************************************************
* File Name: ccg7s_rom_wrapper.h
* \version 2.0
*
* Description: This file is an extension of srom_vars_ccg7s.h. This was needed to make new timer ids compatible with genertated ROM.
*              This file is recommended to stay intact.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/


#ifndef CCG7S_ROM_WRAPPER_H_
#define CCG7S_ROM_WRAPPER_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_syslib.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_defines.h"
#include "config.h"
#include "battery_charging.h"
#if CCG_HPI_ENABLE
#include "hpi.h"
#endif /* CCG_HPI_ENABLE */
#if (defined(CY_DEVICE_CCG7S) && (CCG_SROM_CODE_ENABLE))

bool Cy_PdUtils_SwTimer_Start_rom(cy_stc_pdutils_sw_timer_t *context, void *callbackContext,
        cy_timer_id_t id, uint16_t period, cy_cb_timer_t cb);

void Cy_PdUtils_SwTimer_Stop_rom(cy_stc_pdutils_sw_timer_t *context, cy_timer_id_t id);

bool Cy_PdUtils_SwTimer_IsRunning_rom (cy_stc_pdutils_sw_timer_t *context, cy_timer_id_t id);

void Cy_PdUtils_SwTimer_StopRange_rom(cy_stc_pdutils_sw_timer_t *context,
        cy_timer_id_t start, cy_timer_id_t end);

#endif /* (defined(CY_DEVICE_CCG7S) && (CCG_SROM_CODE_ENABLE)) */
#endif /* CCG7D_ROM_WRAPPER_H_ */
