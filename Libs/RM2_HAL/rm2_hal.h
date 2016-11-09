//
//  RM2 Hardware Abstraction Layer
//  01.07.2015 S.Omelchenko
//  somelchenko@strij.net
//

#ifndef NWAVE_COMMON_H_
#define NWAVE_COMMON_H_

#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "time.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_wdog.h"
#include "em_rmu.h"
#include "em_device.h"
#include "em_gpio.h"
#include "em_opamp.h"
#include "em_adc.h"
#include "em_cmu.h"

#include "td1202.h"
#include "td_gpio.h"
#include "td_rtc.h"
#include "td_flash.h"
#include "td_measure.h"
#include "td_flash.h"


#include "UNB.h"


/** RM2 device descriptor */
typedef struct _RM2_DEVICE {
    uint32_t   ID;            	/* ID  */
    uint32_t   Key[8];			/*Key*/
    uint32_t   Reserved[7];	        /*Reserved*/
} RM2_DEVICE;

/*
typedef  struct _TWO_DEV
{
  TD_DEVICE     td;
  RM2_DEVICE    rm2;
} TWO_DEV;
*/
    
#define RM2_TRANSFER    (0x8000 - 256)
#define E2P_FACTORY     0x0FE00000

#define DEP_DURATION    24*60*60   //duration of depassivation cycle
#define DEP_PERIOD      60         //minimum period time betwen dep. cycles 

extern uint32_t serial;

extern uint8_t Voltage;
extern uint8_t PowLev;

uint32_t 	RM2_GetSerial();
RM2_DEVICE*     RM2_GetDevice(RM2_DEVICE* device);
bool            RM2_Trasfer_ID();
void battery_start_measure();
uint8_t battery_read_measure();
void power_man_run_after_send();
void power_man_run_every_sec();
void run_depassivation(bool);



#endif