/******************************************************************************
* File Name: ccg7s_rom_wrapper.c
* \version 2.0
*
* Description: This file is an extension of srom_vars_ccg7s.c. This was needed to make new timer ids compatible with genertated ROM.
*              This file is recommended to stay intact.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2022-2023 Cypress Semiconductor $
*******************************************************************************/

#include "ccg7s_rom_wrapper.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_common.h"
#include "app.h"
#include "config.h"
#include "sensor_check.h"
#if (defined(CY_DEVICE_CCG7S) && (CCG_SROM_CODE_ENABLE))

bool Cy_PdUtils_SwTimer_Start_rom(cy_stc_pdutils_sw_timer_t *context, void *callbackContext,
        cy_timer_id_t id, uint16_t period, cy_cb_timer_t cb)
{
    /* Mapping old timer ids with new ones. ROM was generated with timer ids same as creator. */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    return(Cy_PdUtils_SwTimer_Start(context, callbackContext, temp_id, period, cb));

}

void Cy_PdUtils_SwTimer_Stop_rom(cy_stc_pdutils_sw_timer_t *context, cy_timer_id_t id)
{
    /* Mapping old timer ids with new ones. ROM was generated with timer ids same as creator. */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    Cy_PdUtils_SwTimer_Stop(context, temp_id);
}

bool Cy_PdUtils_SwTimer_IsRunning_rom (cy_stc_pdutils_sw_timer_t *context, cy_timer_id_t id)
{
    /* Mapping old timer ids with new ones. ROM was generated with timer ids same as creator. */
    cy_timer_id_t temp_id = cy_sw_psoc_modus_id(true, (cy_timer_id_t)id);
    return(Cy_PdUtils_SwTimer_IsRunning(context, temp_id));
}
void Cy_PdUtils_SwTimer_StopRange_rom(cy_stc_pdutils_sw_timer_t *context,
        cy_timer_id_t start, cy_timer_id_t end)
{
    /* Mapping old timer ids with new ones. ROM was generated with timer ids same as creator. */
    cy_timer_id_t temp_id_start = cy_sw_psoc_modus_id(true, (cy_timer_id_t)start);
    cy_timer_id_t temp_id_end = cy_sw_psoc_modus_id(true, (cy_timer_id_t)end);
    Cy_PdUtils_SwTimer_StopRange(context, temp_id_start,temp_id_end);

}
#endif /* (defined(CY_DEVICE_CCG7S) && (CCG_SROM_CODE_ENABLE)) */
