/*
 * time.c
 *
 *  Created on: 14 марта 2014 г.
 *      Author: vdubikhin
 */

#include <stdlib.h>
#include <stdint.h>

#include <efm32.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_timer.h>
#include <em_rtc.h>
#include <em_gpio.h>

#include <td_module.h>

#include "timer.h"

#define INITIAL_DELAY 3276
#define MAIN_DELAY 32768
#define DELTA_DELAY 100
#define TRANSMIT_DELAY 3276//8)*1 //T1S*5.8
#define NUM_DEVICES 50

//Timer0 counter base value
//For 14Mhz clock and 1024 prescale ~ 1ms
#define MILLI_SEC 14

/** Special factory page (512 bytes) */
#define E2P_FACTORY     0x0FE00000

#define PORT_PWM SDA_PORT
#define PIN_PWM  SDA_BIT


static RTC_Init_TypeDef RTCInit = {
    .debugRun = false,
    .comp0Top = true,
    .enable   = true,
};

static int disableCount;

uint32_t deviceSerial;

//initialize timers interrupt
void initTimer() {

    // Start LFXO, and use LFXO for low-energy modules
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    // Input RTC init struct in initialize function
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    // Enable the RTC clock
    CMU_ClockEnable(cmuClock_RTC, true);

    // Enable clock to low energy modules
    CMU_ClockEnable(cmuClock_CORELE, true);

    // Use internal crystal
    RTC_Reset();
    RTC_Init(&RTCInit);

    // Enable RTC interrupt vector in NVIC
    NVIC_EnableIRQ(RTC_IRQn);

	//get device data from flash mem
    deviceSerial = *((uint32_t*)(E2P_FACTORY+2)) >> 16;
	//initialize random with device serial number
	srand(deviceSerial);

	//set int timer
	RTC_IntClear(RTC_IFC_COMP0);
	RTC_IntClear(RTC_IFC_COMP1);

	RTC_CompareSet(0, (uint32_t) MAIN_DELAY);

	RTC_CounterReset();
	RTC_IntEnable(RTC_IF_COMP0);

    // Enable the RTC timer
    RTC_Enable(true);
}

void TIMER0_IRQHandler(void)
{

    /* Clear flag for TIMER0 overflow interrupt */
    TIMER_IntClear(TIMER0, TIMER_IF_OF);
    if (disableCount)
        TIMER_Enable(TIMER0, false);
}

void initTimer0PWM() {
    TIMER_Reset(TIMER0);
    CMU_ClockEnable(cmuClock_TIMER0, true);


    TIMER_Init_TypeDef timerInit =
    {
      .enable     = true,
      .debugRun   = false,
      .prescale   = timerPrescale1,
      .clkSel     = timerClkSelHFPerClk,
      .fallAction = timerInputActionNone,
      .riseAction = timerInputActionNone,
      .mode       = timerModeUp,
      .dmaClrAct  = false,
      .quadModeX4 = false,
      .oneShot    = false,
      .sync       = false,
    };

    /* Select CC channel parameters */
     TIMER_InitCC_TypeDef timerCCInit =
     {
       .eventCtrl  = timerEventEveryEdge,
       .edge       = timerEdgeBoth,
       .prsSel     = timerPRSSELCh0,
       .cufoa      = timerOutputActionNone,
       .cofoa      = timerOutputActionNone,
       .cmoa       = timerOutputActionToggle,
       .mode       = timerCCModePWM,
       .filter     = false,
       .prsInput   = false,
       .coist      = false,
       .outInvert  = false,
     };

     /* Configure CC channel 0 */
     TIMER_InitCC(TIMER0, 0, &timerCCInit);

     //Enable PWM pin
     GPIO_PinModeSet(PORT_PWM, PIN_PWM, gpioModePushPull, 0);

     /* Route CC0 to location 4 (PD6) and enable pin */
     TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC4);


    /* Enable overflow interrupt */
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);

    /* Enable TIMER0 interrupt vector in NVIC */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Set TIMER Top value and PWM to 50%*/
    TIMER_TopSet(TIMER0, 140);
    TIMER_CompareBufSet(TIMER0, 0, 70);

    /* Configure TIMER */
    TIMER_Init(TIMER0, &timerInit);
    disableCount = 0;
}

void disableTimer0PWM() {
    TIMER0->ROUTE &= ~(TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC4);
    TIMER_Reset(TIMER0);
    //Disable PWM pin
    GPIO_PinModeSet(PORT_PWM, PIN_PWM, gpioModeDisabled, 0);
}

void initTimer0 (uint32_t delay) {
    CMU_ClockEnable(cmuClock_TIMER0, true);
    TIMER_Reset(TIMER0);

    TIMER_Init_TypeDef timerInit =
    {
      .enable     = true,
      .debugRun   = false,
      .prescale   = timerPrescale1024,
      .clkSel     = timerClkSelHFPerClk,
      .fallAction = timerInputActionNone,
      .riseAction = timerInputActionNone,
      .mode       = timerModeUp,
      .dmaClrAct  = false,
      .quadModeX4 = false,
      .oneShot    = false,
      .sync       = false,
    };

    /* Enable overflow interrupt */
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);

    /* Enable TIMER0 interrupt vector in NVIC */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Set TIMER Top value */
    TIMER_TopSet(TIMER0, delay * MILLI_SEC);

    /* Configure TIMER */
    TIMER_Init(TIMER0, &timerInit);
    disableCount = 1;
}

void delayVariableTimer0(uint32_t delay) {

    initTimer0(delay);
    while (TIMER0->STATUS & 1)
        EMU_EnterEM1();
}

void delayTimer0() {
    initTimer0(DELTA_DELAY*(rand() % NUM_DEVICES + 1));
    while (TIMER0->STATUS & 1)
        EMU_EnterEM1();
}

//set timer cmp value
void setTimer() {
    //RTC_CompareSet(0, 0xFFFFFF);
    RTC->CNT = 0;
	RTC_CompareSet(1, (uint32_t) MAIN_DELAY);
}
