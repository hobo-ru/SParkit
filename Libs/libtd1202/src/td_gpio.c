/***************************************************************************//**
 * @file
 * @brief General Purpose IO (GPIO) peripheral API for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include "td1202.h"
#include <em_cmu.h>
#include <stdbool.h>

#include <em_gpio.h>
#include "td_gpio.h"

/***************************************************************************//**
 * @addtogroup GPIO
 * @brief General Purpose IO (GPIO) peripheral API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup GPIO_PUBLIC_VARIABLES Public Variables
 * @{ */

/** Array of GPIO IRQ hooks for system/user/odd/even IRQs */
TD_GPIO_hook_t TD_GPIO_Hooks[TD_GPIO_MAX_HOOKS];

/** @} */

#ifdef __ICCARM__
#pragma optimize = speed
#else
#pragma GCC optimize ("O3")
#endif

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup GPIO_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   GPIO odd interrupt handler
 *
 * @details
 *   This function is automatically called by the hardware upon reception of an
 *   interrupt generated on an odd-numbered GPIO pin.
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	uint32_t flag, mask;
	uint32_t system_mask = TD_GPIO_Hooks[TD_GPIO_SYSTEM_ODD].mask;
	uint32_t user_mask =TD_GPIO_Hooks[TD_GPIO_USER_ODD].mask;

    // Get interrupt bits
	flag = GPIO_IntGet() & TD_GPIO_ODD_MASK;

    // Save it
	mask = flag;

    // System hook set on odd IRQ
	if (TD_GPIO_Hooks[TD_GPIO_SYSTEM_ODD].callback &&
			(mask & system_mask)) {

		// Remember that this interrupt source has been processed
		mask &= ~system_mask;

        // Call the IRQ hook
		TD_GPIO_Hooks[TD_GPIO_SYSTEM_ODD].callback();
    }

    // User hook on odd IRQ
	if (TD_GPIO_Hooks[TD_GPIO_USER_ODD].callback &&
			(mask & user_mask)) {

        // Call the IRQ hook
		TD_GPIO_Hooks[TD_GPIO_USER_ODD].callback();
    }

	// Acknowledge IRQ
    GPIO_IntClear(flag & (system_mask | user_mask));
}

/***************************************************************************//**
 * @brief
 *   GPIO even interrupt handler
 *
 * @details
 *   This function is automatically called by the hardware upon reception of an
 *   interrupt generated on an even-numbered GPIO pin.
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	uint32_t flag, mask;
	uint32_t system_mask = TD_GPIO_Hooks[TD_GPIO_SYSTEM_EVEN].mask;
	uint32_t user_mask =TD_GPIO_Hooks[TD_GPIO_USER_EVEN].mask;

    // Get interrupt bits
	flag = GPIO_IntGet() & TD_GPIO_EVEN_MASK;

    // Save it
	mask = flag;

    // System hook set on odd IRQ
	if (TD_GPIO_Hooks[TD_GPIO_SYSTEM_EVEN].callback &&
			(mask & system_mask)) {

		// Remember that this interrupt source has been processed
		mask &= ~system_mask;

        // Call the IRQ hook
		TD_GPIO_Hooks[TD_GPIO_SYSTEM_EVEN].callback();
    }

    // User hook on even IRQ
	if (TD_GPIO_Hooks[TD_GPIO_USER_EVEN].callback &&
			(mask & user_mask)) {

        // Call the IRQ hook
		TD_GPIO_Hooks[TD_GPIO_USER_EVEN].callback();
    }

	// Acknowledge IRQ
    GPIO_IntClear(flag & (system_mask | user_mask));
}

/***************************************************************************//**
 * @brief
 *   Initialize the GPIO peripheral for the TD1202 module.
 ******************************************************************************/
void TD_GPIO_Init(void)
{
	int i;

	// Enable the GPIO module clock
	CMU_ClockEnable(cmuClock_GPIO, true);

    // Start LFXO and wait until it is stable
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	// Initialize IRQ hooks
	for (i = 0; i < TD_GPIO_MAX_HOOKS; i++) {
		TD_GPIO_Hooks[i].callback = 0;
		TD_GPIO_Hooks[i].mask = 0;
	}
}

/** @} */

/** @} (end addtogroup GPIO) */
