
/***************************************************************************//**
 * @file
 * @brief Real-Time Clock (RTC) peripheral API for the TD1202 module.
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

#include "td1202.h"

#include <time.h>
#include <em_cmu.h>
#include <em_rtc.h>
#include <em_emu.h>
#include <stdbool.h>
#include "td_rtc.h"

/***************************************************************************//**
 * @addtogroup RTC
 * @brief Real-Time Clock (RTC) peripheral API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup RTC_DEFINES Defines
 * @{ */

/* Clock Defines */

/* These variables must reflect the RTC frequency */

/** Number of ticks per second */
#define TD_RTC_TICKS_PER_SECOND 32768

/** Overflow period based on counter width and frequency */
#define TD_RTC_OVERFLOW_PERIOD ((0x00FFFFFF + 1) / TD_RTC_TICKS_PER_SECOND)

/** Overflow flag based on counter width and frequency */
#define TD_RTC_OVERFLOWED ((0x00FFFFFF + 1) % TD_RTC_TICKS_PER_SECOND)


/** Use keepalive */
#define TD1202_KEEPALIVE

/** @} */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RTC_PUBLIC_VARIABLES Public Variables
 * @{ */

/** End of delay flag */
volatile bool TD_RTC_AbortDelay = false;

#ifdef TD1202_KEEPALIVE
/** Keep-alive flag */
volatile bool TD_RTC_KeepAlive = false;
#endif

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RTC_PRIVATE_VARIABLES Private Variables
 * @{ */

/** RTC initialization structure */
static RTC_Init_TypeDef RTCInit = {
	.debugRun = false,
	.comp0Top = false,
	.enable   = false,
};

/** End of delay flag */
static volatile bool TD_RTC_EndDelay = false;

/** The system RTC interrupt handler */
static TD_RTC_handler_t TD_RTC_SystemHandler  = 0;

/** The user RTC interrupt handler */
static TD_RTC_handler_t TD_RTC_UserHandler  = 0;

static uint32_t rtcOverflowCounter = 0;
static int   rtcOffsetTime = 0;

#ifdef TD1202_KEEPALIVE
/** The user keep-alive interrupt handler */
TD_RTC_handler_t TD_RTC_KeepAliveHandler = 0;

/** Keep-alive counter in 512 s resolution */
static uint32_t TD_RTC_KeepAliveCounter = 0;

/** Keep-alive limit in 512 s resolution */
static uint32_t TD_RTC_KeepAliveLimit = 0;
#endif

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup RTC_PRIVATE_FUNCTIONS Private Functions
 * @{ */

#ifdef __ICCARM__
// #pragma optimize = speed
#else
#pragma GCC optimize ("O3")
#endif

/***************************************************************************//**
 * @brief
 *   Default RTC interrupt callback.
 ******************************************************************************/
static void TD_RTC_DelayHandler(void)
{
    TD_RTC_EndDelay = true;
}

#ifdef TD1202_KEEPALIVE
/***************************************************************************//**
 * @brief
 *   Default keep-alive interrupt callback.
 ******************************************************************************/
static void TD_RTC_DefaultKeepAliveHandler(void)
{
    TD_RTC_KeepAlive = true;
}
#endif

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup RTC_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   RTC Interrupt Handler. Clears interrupt flag.
 *   The interrupt table is located in the assembly startup file startup_efm32.s.
 *
 * @details
 *   The purpose of the RTC interrupt is to wake the CPU from deep sleep mode EM2
 *   at given intervals. This is a way to save energy, while ensuring that the
 *   CPU often enough checks if there are any other instructions ready to executed.
 ******************************************************************************/
#if (MODULE_REVISION == REVISION_VBC4)
#else
void RTC_IRQHandler(void)
{
	if (RTC->IF & RTC_IF_COMP0)
	{

		// System RTC interrupt
		if ((RTC->IEN & RTC_IF_COMP0) && (TD_RTC_SystemHandler != 0))
		{
			TD_RTC_SystemHandler();
		}

		// Clear interrupt source
		RTC_IntClear(RTC_IFC_COMP0);
	}

	if (RTC->IF & RTC_IF_COMP1)
	{

    	// User RTC interrupt
        if ((RTC->IEN & RTC_IF_COMP1) && (TD_RTC_UserHandler != 0))
		{
            TD_RTC_UserHandler();
        }

    	// Clear interrupt source
    	RTC_IntClear(RTC_IFC_COMP1);
    }

    if (RTC->IF & RTC_IF_OF)
	{
    	rtcOverflowCounter++;
		rtcOverflowCounter &= 0xFF;

    	// Keep-alive interrupt
    	RTC_IntClear(RTC_IF_OF);

#ifdef TD1202_KEEPALIVE
    	if ((RTC->IEN & RTC_IF_OF) && (TD_RTC_KeepAliveLimit != 0))
		{
    		if (++TD_RTC_KeepAliveCounter >= (TD_RTC_KeepAliveLimit >> 9))
			{
    			TD_RTC_KeepAliveCounter -= TD_RTC_KeepAliveLimit >> 9;
    			TD_RTC_DefaultKeepAliveHandler();
    		}
    	}
    }
#endif

}
#endif

/***************************************************************************//**
 * @brief
 *   Initialize the RTC timer.
 *
 * @param[in] function
 *   Pointer to function to handle RTC interrupts.
 ******************************************************************************/
void TD_RTC_Init(TD_RTC_handler_t function)
{
    // Input RTC init struct in initialize function
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

	// Enable the RTC clock
	CMU_ClockEnable(cmuClock_RTC, true);

    // Set system handler
    TD_RTC_SetSystemHandler(function);

    // Use internal crystal
    RTC_Reset();
	RTC_Init(&RTCInit);

	// No start if > T8mn just Tick timer 8mn
	RTC_CompareSet(0, 0xFFFFFF);

	// No start if > T8mn just Tick timer 8mn
	//RTC_CompareSet(1, 0xFFFFFF);
	RTC_CompareSet(1, 32768/10); // 200ms
	// Enable RTC interrupt vector in NVIC
	NVIC_EnableIRQ(RTC_IRQn);

	// Enable the RTC timer
	RTC_Enable(true);

    TD_RTC_EndDelay = false;

	// Enable RTC overflow interrupt for keep-alive
#ifdef TD1202_KEEPALIVE
    TD_RTC_KeepAlive = false;
    TD_RTC_KeepAliveLimit = 0;
	TD_RTC_KeepAliveCounter = 0;
    RTC_IntClear(RTC_IF_OF);
    RTC_IntEnable(RTC_IF_OF);
#endif
}

/***************************************************************************//**
 * @brief
 *   Wait for a delay period.
 *
 * @param[in] duration
 *   The delay duration in 32768 Hz units.
 *
 * @return
 *   Returns true if the false completed, true if it was aborted.
 ******************************************************************************/
#if (MODULE_REVISION == REVISION_VBC4)
bool TD_RTC_Delay(uint32_t duration)
{
	volatile unsigned int counter;

	for (counter = 0; counter < 1500 * duration / 32; counter++) {
		;
	}
	return true;
}
#else
bool TD_RTC_Delay(uint32_t duration)
{
	TD_RTC_handler_t old = TD_RTC_SystemHandler;
    uint32_t top = 0xFFFFFF; // RTC->COMP0;

	if (duration > 0)
	{
		RTC_FreezeEnable(true);
        if (RTC_CounterGet() + duration > top)
		{
        	// Overflow
        	RTC_CompareSet(0, duration - (top - RTC_CounterGet()));
        }
		else
		{
        	RTC_CompareSet(0, RTC_CounterGet() + duration);
        }
        RTC_FreezeEnable(false);

        TD_RTC_AbortDelay = false;
        TD_RTC_EndDelay = false;

        TD_RTC_SystemHandler = TD_RTC_DelayHandler;
        RTC_IntClear(RTC_IFC_COMP0);
        RTC_IntEnable(RTC_IF_COMP0);

        while (!TD_RTC_EndDelay && !TD_RTC_AbortDelay)
        {
            EMU_EnterEM2(false);
        }

        TD_RTC_EnableSystemInterrupts(false);
        TD_RTC_SystemHandler = old;

        return TD_RTC_EndDelay;
    }
    return true;
}
#endif

/***************************************************************************//**
 * @brief
 *   Enter into sleep mode (EM2).
 ******************************************************************************/
void TD_RTC_Sleep(void)
{
	RTC_IntClear(RTC_IFC_COMP0);
	RTC_IntClear(RTC_IFC_COMP1);
	RTC_IntEnable(RTC_IF_COMP0);
	RTC_IntEnable(RTC_IF_COMP1);
	EMU_EnterEM2(false);
	RTC_IntDisable(RTC_IF_COMP0);
	RTC_IntDisable(RTC_IF_COMP1);
}

/***************************************************************************//**
 * @brief
 *   Set RTC system interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle RTC system interrupts.
 ******************************************************************************/
TD_RTC_handler_t TD_RTC_SetSystemHandler(TD_RTC_handler_t function)
{
	TD_RTC_handler_t old = TD_RTC_SystemHandler;

    TD_RTC_SystemHandler = function;
    return old;
}

/***************************************************************************//**
 * @brief
 *   Set RTC user interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle RTC user interrupts.
 ******************************************************************************/
TD_RTC_handler_t TD_RTC_SetUserHandler(TD_RTC_handler_t function)
{
	TD_RTC_handler_t old = TD_RTC_UserHandler;

    TD_RTC_UserHandler = function;
    return old;
}

#ifdef TD1202_KEEPALIVE
/***************************************************************************//**
 * @brief
 *   Set keep-alive interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle keep-alive interrupts.
 *
 * @param[in] period
 *   keep-alive period in seconds.
 ******************************************************************************/
void TD_RTC_SetKeepAliveHandler(TD_RTC_handler_t function, uint32_t period)
{
	TD_RTC_KeepAliveLimit = period;
    TD_RTC_KeepAliveHandler = (function == 0) ? TD_RTC_DefaultKeepAliveHandler : function;
}
#endif

/***************************************************************************//**
 * @brief
 *   Perform a delay calibration.
 *
 * @param[in] udelay
 *   The calibration delay in µs.
 ******************************************************************************/
void TD_RTC_CalibratedDelay(uint32_t udelay)
{
    static uint32_t clock = 0;
    long long delay;

    if (clock == 0) {
    	clock  = CMU_ClockFreqGet(cmuClock_HFPER);
    	clock /= CMU_ClockDivGet(cmuClock_HFPER);
    }
    delay  = clock;
    delay *= udelay;
    delay /= 1000000;
    CMU_Calibrate(delay, cmuOsc_HFRCO);
}

/***************************************************************************//**
 * @brief
 *   Returns the difference between current time and a reference time.
 *
 * @param[in] reference
 *   The reference time to compare to.
 *
 * @return
 *   Returns the absolute difference with the reference time.
 ******************************************************************************/
uint32_t TD_RTC_TimeDiff(uint32_t reference)
{
    uint32_t difference, now = RTC_CounterGet();

    if (now < reference) {
    	difference = 0xFFFFFF - reference + now;
    } else {
    	difference = now - reference;
    }
    return difference;
}

/***************************************************************************//**
 * @brief Set a time offset to clock timer.
 *
 * @param[in] delta
 *   The offset to apply to the clock timer.
 ******************************************************************************/
void TD_RTC_SetOffsetTime(int  delta)
{
  rtcOffsetTime = delta;
}

/***************************************************************************//**
 * @brief Returns the number of overflows that occurred since start time.
 *
 * @return
 *  Returns the number of overflows that occurred since start time.
 ******************************************************************************/
uint32_t TD_RTC_GetOverflowCounter(void)
{
  return rtcOverflowCounter;
}

/***************************************************************************//**
 * @brief Returns the current system time
 *
 * @param[in] timer
 *   Pointer to copy the current system time to, if not NULL.
 *
 * @return
 *   Returns the current system time.
 *
 * @note
 *   it should return -1 if system time is not available, but it currently doesn't.
 *
 *****************************************************************************/
time_t __time32(time_t *timer)
{
	time_t t;

	// Add time based on number of counter overflows
	t = rtcOverflowCounter * TD_RTC_OVERFLOW_PERIOD;

	// Correct if overflow interval is not an integer
	if (TD_RTC_OVERFLOWED != 0) {
		t += rtcOverflowCounter * TD_RTC_OVERFLOWED / TD_RTC_TICKS_PER_SECOND;
	}

	// Add the number of seconds for RTC
	t += RTC->CNT / TD_RTC_TICKS_PER_SECOND;
	t += rtcOffsetTime;

	// Copy system time to timer if not NULL
	if (!timer) {
		timer = &t;
	}
	return t;
}

/** @} */

/** @} (end addtogroup RTC) */
