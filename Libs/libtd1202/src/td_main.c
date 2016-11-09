/***************************************************************************//**
 * @file
 * @brief Main program for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#include <stdint.h>
#include <stdbool.h>
#include <em_chip.h>
#include <td_gpio.h>
#include <td_rtc.h>
#include <td_cmu.h>
#include <td_uart.h>
#include <td_timer.h>
#include <td_printf.h>
//#include <td_boot.h>
//#include <td_sigfox.h>
#include <td_rtc.h>

/***************************************************************************//**
 * @addtogroup MAIN Main
 * @brief Main program for the TD1202 module
 * @{
 ******************************************************************************/

/** @addtogroup MAIN_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup MAIN_PROTOTYPES Extern Declarations
 * @{ */

/** User setup function called once */
extern void TD_USER_Setup(void);

/** User function called at each wake-up event */
extern void TD_USER_Loop(void);

/** @} */
/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MAIN_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Main function for the TD1202 module.
 *
 * @details
 *   This function performs all the required initialization for the TD1202
 *   module before calling a user supplied function to perform the user
 *   initialization.
 *   This function then enter an infinite loop going into sleep mode waiting
 *   for an interrupt, and calls a user supplied function upon wake-up.
 *
 * @note
 *   If you use this main() function, you must provide 2 functions
 *   - TD_USER_Setup(), which will be called once at startup
 *   - TD_SUER_Loop(), which will be called after waking up by an event
 *   If you provide your own main() function, it will take precedence over this
 *   one, which is completely possible. However, using the library main()
 *   function as a template for your implementation is recommended.
 *
 * @note
 *   This function never returns.
 ******************************************************************************/
void main(void)
{
	// Workarounds for chip errata
	CHIP_Init();

	// Call Radio Loader. Will return after 200ms if no upgrade frames received.
	//TD_BOOT_Init(false, 0);

	// Initialize the clock Management Unit
	TD_CMU_Init(true);

	// Initialize the RTC clock
	TD_RTC_Init(0);

	// Initialize GPIOs
	TD_GPIO_Init();

    // Initialize SigFox
    //TD_SIGFOX_Init(true);

    // Call user setup function
    TD_USER_Setup();

    // Main idle loop
    while (1) 
	{
    	// Go into EM2 sleep mode until an event occurs
		TD_RTC_Sleep();

		// Call user loop function
		TD_USER_Loop();

		// Handle keep-alive periodic events
		/*
		if (TD_RTC_KeepAlive) {
			TD_RTC_KeepAlive = false;
			if (TD_RTC_KeepAliveHandler) {
				TD_RTC_KeepAliveHandler();
			}
		}
		*/
    }
}

/** @} */

/** @} (end addtogroup MAIN) */
