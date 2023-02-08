#ifndef _VBUS_IDAC_CTRL_H_
#define _VBUS_IDAC_CTRL_H_

#include <stdbool.h>
#include <stdint.h>
#include "cy_pdstack_common.h"
#include "config.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
 
/**
* \addtogroup group_ccgxAppCommon App Common Middleware
* \{
*/

/**
* \addtogroup group_ccgxAppCommon_functions
* \{
*/

/**
 * @brief The function computes PWM duty cycle needed to generate requested VBUS.
 *
 * @param vbus The voltage to set to in mV units
 * @param min_volt Minimum VBUS as per design in mv
 * @param max_volt Maximum VBUS as per design in mv
 * @param pwm_period Period of the PWM signal.
 *
 * @return PWM duty cycle in terms of PWM on time.
 */
uint16_t pwm_get_duty_cycle(uint16_t vbus, uint16_t min_volt, uint16_t max_volt,
    uint16_t pwm_period);

/**
 * @brief The function enables the PWM signal that turns VBus ON.
 * @param context PD Stack Context
 * @return None
 */
void vbus_ctrl_pwm_turn_on(cy_stc_pdstack_context_t * context);

/**
 * @brief The function disables the PWM signal thus turning VBus OFF.
 * @param context PD Stack Context
 * @return None
 */
void vbus_ctrl_pwm_turn_off(cy_stc_pdstack_context_t * context);

/**
 * @brief The function sets the Type-C VBUS voltage to requested level using PWM.
 *
 * @param context PD Stack Context
 * @param volt_mV The voltage to set to in mV units.
 * @return None
 */
void vbus_ctrl_pwm_set_volt(cy_stc_pdstack_context_t * context, uint16_t volt_mV);

/**
 * @brief The function enables the Type-C VBUS voltage feedback logic.
 *
 * @param context PD Stack Context
 * @return None
 */
void vbus_ctrl_fb_enable(cy_stc_pdstack_context_t * context);

/**
 * @brief The function disables the Type-C VBUS voltage feedback logic.
 *
 * @param context PD Stack Context
 * @return None
 */
void vbus_ctrl_fb_disable(cy_stc_pdstack_context_t * context);
/**
 * @brief The function sets the Type-C VBUS voltage to requested level using FB (feedback).
 *
 * @param context PD Stack Context
 * @param volt_mV The voltage to set to in mV units.
 * @return None
 */
void vbus_ctrl_fb_set_volt(cy_stc_pdstack_context_t * context, uint16_t volt_mV);
/**
  * @brief The function returns whether the vbus voltage transition is over or not.
  *
  * @param context PD Stack Context
  *
  * @return True if vbus transition is over, otherwise False.
  */
bool vbus_ctrl_set_is_idle(cy_stc_pdstack_context_t * context);

/**
 * @brief The function returns trimmed IDAC value for the required voltage.
 *
 * @param context PD Stack Context
 * @param volt_mv Required source voltage in mV
 * @return idac Signed IDAC value for the requested voltage
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, PAG1S, CCG7D, CCG7S.
 */
int16_t vbus_ctrl_get_trim_idac(cy_stc_usbpd_context_t * context, uint16_t volt_mv);
/** \} group_ccgxAppCommon_functions */
#endif /* _VBUS_IDAC_CTRL_H_ */

/** \} group_ccgxAppCommon */

/* End of File */
