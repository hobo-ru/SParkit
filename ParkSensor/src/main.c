/*
* main.c
*
*  Created on: 06 рту. 2014 у.
*      Author: vdubikhin
*/

#include <stdlib.h>
#include <stdint.h>

#include "bootloader_inapp.h"
#include <efm32.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_timer.h>
#include <em_gpio.h>
#include <em_chip.h>
#include <em_wdog.h>
#include <em_adc.h>

#include <td_module.h>
#include <td1202.h>
#include <td_rtc.h>
#include <td_cmu.h>
#include <td_gpio.h>
#include <td_flash.h>
#include <td_measure.h>
#include <td_uart.h>
#include <td_lan.h>

#include <Protocol.h>
#include <commands.h>
#include <math.h>
#include <UNB.h>
#include <mag_stats.h>
#include <i2cEEPROM.h>

#include "mag_sensor.h"
#include "rm2_hal.h"
#include "defines.h"

#define  HARDWARE_REV 0
#define  SOFTWARE_REV 0



// define where output led port and pin located
#define PORT_LED TIM1_PORT
#define PIN_LED TIM1_BIT

#define PORT_LED2 TIM2_PORT
#define PIN_LED2 TIM2_BIT

// define where output led port and pin located
#define PORT_BUTTON DAC0_PORT
#define PIN_BUTTON DAC0_BIT
#define MASK_BUTTON DAC0_MASK

// define time constant
#define SLEEP_TIME 32768/4

#define BATTERY_NORMAL_VALUE 140 // ~3.2V

#define INT16_HIGH_BYTE(x)		(((uint16_t)(x)) >> 8)
#define INT16_LOW_BYTE(x)		(((uint16_t)(x)) & 0xFF)
#define ABS(x) (((x)>0)?(x):(-(x)))



TD_DEVICE       device;
RM2_DEVICE      rm2_device;

uint32_t serial;
#define DEV_SID			0x00010000			// additional value for device serial id

#define N_CAL 1200
//#define N_CAL 200

uint16_t moving_average_length = 0;
uint16_t sigma2 = 0;
//bool selfTestFault = false;

typedef struct regression_struct
{
	// z = x*ax + y*ay + b
	double b;
	double ax;
	double ay;
} regression_t;

#define IIR_N_COEF 2
const float ACoef[IIR_N_COEF+1] = {
	0.00093397146457995048,
	0.00186794292915990100,
	0.00093397146457995048
};

const float BCoef[IIR_N_COEF+1] = {
	1.00000000000000000000,
	-1.89310959312122780000,
	0.89685259842946785000
};

typedef struct iir_struct
{
	float y[IIR_N_COEF+1]; //output samples
        float x[IIR_N_COEF+1]; //input samples
	float previous_input;
	int threshold;
	int out;
	bool result;
	double average;
}iir_filter_t;


//int16_t dataX, dataY, dataZ;
magnetdata_t magnetdata;

double sum_x = 0;
double sum_y = 0;
double sum_z = 0;
//double sum_xz = 0;
//double sum_yz = 0;
//double sum_x2 = 0;
//double sum_y2 = 0;
//double sum_z2 = 0;
//double var_x = 0;
//double var_y = 0;
//double var_z = 0;
//double cov_xz = 0;
//double cov_yz = 0;
//double corr_xz2 = 0;
//double corr_yz2 = 0;
//double var_diff = 0;
//regression_t r;
//m_average_t filter, filter2;//, filter3;
iir_filter_t iir_x, iir_y, iir_z;


float angle = 0;
#define THRESHOLD_HIGH  50
#define THRESHOLD_LOW  40
#define THRESHOLD_ZERO  15
#define REQIRED_TIME_ON    60
#define REQIRED_TIME_OFF   30


uint32_t N = 0;
uint32_t errors = 0;

double diff = 0;
double filter_diff = 0;

uint8_t TxData[20];

float iir(iir_filter_t* f, float NewSample);

uint8_t Calibrate(magnetdata_t magnetdata)
{
	if(N == 0)
	{
		// clear variables for new calibration round
		sum_x = 0;
		sum_y = 0;
		sum_z = 0;
//		sum_xz = 0;
//		sum_yz = 0;
//		sum_x2 = 0;
//		sum_y2 = 0;
//		sum_z2 = 0;
	}

	if(N < N_CAL)
	{
		N++;
		sum_x += magnetdata.x;
		sum_y += magnetdata.y;
		sum_z += magnetdata.z;
                iir(&iir_x,(magnetdata.x));
                iir(&iir_y,(magnetdata.y));
                iir(&iir_z,(magnetdata.z));
//		sum_xz += magnetdata.x * magnetdata.z;
//		sum_yz += magnetdata.y * magnetdata.z;
//		sum_x2 += magnetdata.x * magnetdata.x;
//		sum_y2 += magnetdata.y * magnetdata.y;
//		sum_z2 += magnetdata.z * magnetdata.z;
		return 0;
	}
	else
	{
		iir_x.average = sum_x / N_CAL;
		iir_y.average = sum_y / N_CAL;
		iir_z.average = sum_z / N_CAL;
                
              
//		var_x = (sum_x2 / n) - (sum_x / n) * (sum_x / n);
//		var_y = (sum_y2 / n) - (sum_y / n) * (sum_y / n);
//		var_z = (sum_z2 / n) - (sum_z / n) * (sum_z / n);
//		cov_xz = (sum_xz / n) - (sum_x * sum_z / (n*n));
//		cov_yz = (sum_yz / n) - (sum_y * sum_z / (n*n));
//		corr_xz2 = (cov_xz * cov_xz) / (var_x * var_z);
//		corr_yz2 = (cov_yz * cov_yz) / (var_y * var_z);
//		if(corr_xz2 > corr_yz2)
//		{
//			r.ax = cov_xz / var_x;
//			r.ay = 0.0;
//			r.b = (sum_z / N_CAL) - (r.ax * sum_x / N_CAL);	// n or N_CAL?
//		}
//		else
//		{
//			r.ax = 0.0;
//			r.ay = cov_yz / var_y;
//			r.b = (sum_z / N_CAL) - (r.ay * sum_y / N_CAL);	// n or N_CAL?
//		}
//
//		var_diff = (corr_xz2 > corr_yz2) ?
//			(r.ax * r.ax * var_x + var_z - 2 * r.ax * cov_xz):
//			(r.ay * r.ay * var_y + var_z - 2 * r.ay * cov_yz);

//		if(var_diff < 30) var_diff = 30; // TODO define

//		sigma2 = 35;	//DEBUG!!!

		return 1;
	}
}

void iir_init(iir_filter_t* f)
{
	int n;
	f->previous_input = 0;
	f->result = 0;
	f->threshold = 0;
	f->average = 0;
	for(n = 0; n < (IIR_N_COEF+1); n++)
	{
		f->x[n] = 0;
		f->y[n] = 0;
	}
}

float iir(iir_filter_t* f, float NewSample)
{
    int n;

	// Slow correction of average. TODO: place it somewhere else?
//	if(f->result == 0)
//	{
//		f->average += (NewSample > 0)?(0.0015):(-0.0015);	// corrects approximately 10 pts per hour
//	}

       // if(angle < THRESHOLD_ZERO)  f->average += (NewSample > 0)?(0.00005):(-0.00005);	// corrects approximately 0.3 pts per hour
       
    
	/*
        NewSample = ABS(NewSample);

	//limit climb and fall speed
	
        if((ABS(NewSample) - ABS(f->previous_input)) > 80)
	{
		NewSample = f->y[0] + 80;
	}
	else if((ABS(NewSample) - ABS(f->previous_input)) < -80)
	{
		NewSample = f->y[0] - 320;
	}
	if(NewSample < 0) NewSample = 0;*/
        
        
	f->previous_input = NewSample;

    //shift the old samples
    for(n=IIR_N_COEF; n>0; n--)
    {
       f->x[n] = f->x[n-1];
       f->y[n] = f->y[n-1];
    }

    //Calculate the new output
    f->x[0] = NewSample;
    f->y[0] = ACoef[0] * f->x[0];
    for(n=1; n<=IIR_N_COEF; n++)
	{
        f->y[0] += ACoef[n] * f->x[n] - BCoef[n] * f->y[n];
	}

	f->out = f->y[0];
    return f->out;
}


/*
uint8_t Compare_xyz(magnetdata_t magnetdata)
{
	iir(&iir_x,(magnetdata.x - iir_x.average));
	iir(&iir_y,(magnetdata.y - iir_y.average));
	iir(&iir_z,(magnetdata.z - iir_z.average));

	iir_x.threshold = (iir_x.result)?(90):(100);
	iir_y.threshold = (iir_y.result)?(90):(100);
	iir_z.threshold = (iir_z.result)?(80):(120);

	iir_x.result = (iir_x.out > iir_x.threshold);
	iir_y.result = (iir_y.out > iir_y.threshold);
	iir_z.result = (iir_z.out > iir_z.threshold);

        if((iir_x.out < 30)&&(iir_y.out < 30)&&(iir_z.out < 50))
        {
          iir_x.average += ((magnetdata.x - iir_x.average) > 0)?(0.00005):(-0.00005);
          iir_y.average += ((magnetdata.y - iir_y.average) > 0)?(0.00005):(-0.00005);
          iir_z.average += ((magnetdata.z - iir_z.average) > 0)?(0.00005):(-0.00005);
        }
        
	return (iir_x.result | iir_y.result | iir_z.result);
}*/




uint8_t Compare_xyz(magnetdata_t magnetdata)
{
	static bool result = false;
        static uint16_t result_timer = 0;
        
        
        iir(&iir_x,(magnetdata.x));
	iir(&iir_y,(magnetdata.y));
	iir(&iir_z,(magnetdata.z));

        
        //angle = (100.0 *acos((magnetdata.x*iir_x.average + magnetdata.y*iir_y.average + magnetdata.z*iir_z.average)  /  (sqrt(magnetdata.x*magnetdata.x + magnetdata.y*magnetdata.y + magnetdata.z*magnetdata.z)  * (sqrt(iir_x.average*iir_x.average + iir_y.average*iir_y.average + iir_z.average*iir_z.average)))));

        float delta_x = iir_x.out - iir_x.average;
        float delta_y = iir_y.out - iir_y.average;
        float delta_z = iir_z.out - iir_z.average;
        
        
        angle = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
        
        if(!result)
        {
          if(angle > THRESHOLD_HIGH)
          {
              if(++result_timer > REQIRED_TIME_ON)
              {
                result_timer = 0;
                result = true;
              }
          }
          else result_timer = 0;
        }
        else
        {
          if(angle < THRESHOLD_LOW)
          {
              if(++result_timer > REQIRED_TIME_OFF)
              {
                result_timer = 0;
                result = false;
              }
          }
          else result_timer = 0;
        }
        
        if(angle < THRESHOLD_ZERO)
        {
          iir_x.average += ((iir_x.out - iir_x.average) > 0)?(0.00005):(-0.00005);
          iir_y.average += ((iir_y.out - iir_y.average) > 0)?(0.00005):(-0.00005);
          iir_z.average += ((iir_z.out - iir_z.average) > 0)?(0.00005):(-0.00005);
        }     
	return result;
}




void Send(uint8_t data)
{
//	static uint8_t packet_iter = 0;

	TD_RTC_Delay(T10MS);
	TxData[0] = 0x41;
	TxData[1] = data;

#ifdef PROTOCOL_C                   
		UNB_ProtocolC_Send(TxData, serial);
                power_man_run_after_send();
#else
		UNBsend (TxData, 2, serial);
                power_man_run_after_send();
#endif
}

void SendSpanish(uint8_t res) //spain sensors format
{
	static uint16_t count = ((240*15)-10), res_old = 0;
	static uint8_t packet_iter = 0;

	if((res != res_old) || ((count++) == (240*15)))
	{
		if (res != res_old) packet_iter++;
		res_old = res;
		count = 0;

		uint16_t id = 0xC000 + (serial & 0x3FFF);
		TD_RTC_Delay(T10MS);
		TxData[0] = 0x88;
		TxData[1] = INT16_HIGH_BYTE(id);
		TxData[2] = INT16_LOW_BYTE(id);
		TxData[3] = packet_iter;
		TxData[4] = (res == 0x01)?(1):(0);
		TxData[5] = packet_iter;
//		double temp = ((filter_diff * filter_diff) > (sigma2 * var_diff)) ?
//			((filter_diff * filter_diff) / (sigma2 * var_diff)) :
//			((sigma2 * var_diff) / (filter_diff * filter_diff)) ;
		TxData[6] = 0;
		TxData[7] = 0;

#ifdef PROTOCOL_C                   
		UNB_ProtocolC_Send(TxData, serial);
                power_man_run_after_send();
#else
		UNBsend (TxData, 8, serial);
                power_man_run_after_send();
#endif
	}
}

uint8_t CompVersion()
{
    const char CompTime[] = __TIME__;
    const char* ptr;
    uint8_t ver = 0;
    ptr = &CompTime[0];
    while(*ptr) ver += ((*(ptr++)) - 0x30) + (uint8_t)ptr;
    return ver;
}


void SendStrij(uint8_t res) // our own optimised format
{
#define RESEND_COUNT (3600 * 12)	// ~ 12 hours
#define SEND_INFO_COUNT (3600 * 24 * 7)     // ~ one week - period of info message send
	static uint32_t count = 0; // counter for regular repeated messages
	static uint8_t res_old = 0;
	static uint16_t event_iter = 0;
        static uint8_t send_again = 0;
	if((res != res_old) || (send_again >= 60) || !(count % RESEND_COUNT))     //send regular message
	{
          
                if (res != res_old) event_iter++;
		if(event_iter >= 0x4000) event_iter = 0; 
                res_old = res;
                
                TxData[0] = (event_iter >> 8) | (res << 7);
		TxData[1] = event_iter&0xff;
                TxData[2] = INT16_HIGH_BYTE(angle);
                TxData[3] = INT16_LOW_BYTE(angle);
                TxData[4] = TD_MEASURE_Temperature();
                TxData[5] = INT16_LOW_BYTE(iir_x.average);
		TxData[6] = INT16_LOW_BYTE(iir_y.average);
                TxData[7] = INT16_LOW_BYTE(iir_z.average);
                
		//TxData[5] = battery_read_measure();//TD_MEASURE_Voltage();

#ifdef PROTOCOL_C                   
		UNB_ProtocolC_Send(TxData, serial);
                power_man_run_after_send();
#else
		UNBsend (TxData, 6, serial);
                power_man_run_after_send();
#endif
                if(send_again) send_again = 0;
                else send_again = 1;            //send again 60 sec later
	}
        
        if(!(count % SEND_INFO_COUNT))       //send info message
        {
                TxData[0] = 0x40;
		TxData[1] = HARDWARE_REV;
		TxData[2] = SOFTWARE_REV;
		TxData[3] = CompVersion();
		TxData[4] = TD_MEASURE_Temperature();
                TxData[5] = Voltage;
		TxData[6] = PowLev*3 + 8;
		TxData[7] = 0xaa;   //reserved
                
#ifdef PROTOCOL_C                   
		UNB_ProtocolC_Send(TxData, serial);
                power_man_run_after_send();
#else
		UNBsend (TxData, 6, serial);
                power_man_run_after_send();
#endif
        
        }           
        if(send_again) send_again++;  
        count++;
}

#define MAG_FIELD_THRESHOLD 243//300

bool doSelfTest()
{
    uint8_t i, data[8] = {0xC0, 0x01, 0, 0, 0, 0, 0, 0};
    bool res = true;
    for(i=0;i<10;i++)
    {
      if(getSelfTest(&magnetdata, 1) == 0) 
      {
          data[0] = 0xBA;
          data[1] = 0xAD; 
          res = false;
          break;
      }
      if ((magnetdata.x < MAG_FIELD_THRESHOLD)||(magnetdata.y < MAG_FIELD_THRESHOLD)||(magnetdata.z < MAG_FIELD_THRESHOLD))
      {  
          data[0] = 0xBA;
          data[1] = 0xAD;
          res = false;
          break;
      }
      TD_RTC_Delay(3276);
    }
    
    for(i=0;i<10;i++)
    {
      if(getSelfTest(&magnetdata, 0) == 0)
      {
          data[0] = 0xBA;
          data[1] = 0xAD;
          res = false;
          break;
      }
      if ((magnetdata.x > -MAG_FIELD_THRESHOLD)||(magnetdata.y > -MAG_FIELD_THRESHOLD)||(magnetdata.z > -MAG_FIELD_THRESHOLD))
      {
          data[0] = 0xBA;
          data[1] = 0xAD;
          res = false;
          break;
      }
      TD_RTC_Delay(3276);
    }

    if(data[0] == 0xBA)
    {
      *((int16_t*)(&data[2])) = magnetdata.x;
      *((int16_t*)(&data[4])) = magnetdata.y;
      *((int16_t*)(&data[6])) = magnetdata.z;
    }
    
#ifdef PROTOCOL_C                   
    UNB_ProtocolC_Send(data, serial);
    power_man_run_after_send();
#else
    UNBsend (data, 2, serial);
    power_man_run_after_send();
#endif
    return res;
}


uint8_t reed_switch_flag = 0;

static void ButtonInterrupt(void)
{
	reed_switch_flag = 1;
	return;
}
volatile uint32_t crc=0;



extern uint8_t iter;
extern uint32_t N;
void TD_USER_Setup(void)
{
	WDOG_Init_TypeDef initWD =
    {
      .enable     = true,               /* Start watchdog when init done */
      .debugRun   = false,              /* WDOG not counting during debug halt */
      .em2Run     = false,               /* WDOG counting when in EM2 */
      .em3Run     = false,               /* WDOG counting when in EM3 */
      .em4Block   = false,              /* EM4 can be entered */
      .swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
      .lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
      .clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
      .perSel     = wdogPeriod_32k,      /* Set the watchdog period to 16k clock periods (ie ~16 seconds)*/
    };
	WDOG_Init(&initWD);
	WDOG_Enable(true);
	//WDOG_Enable(false);
	WDOG_Feed();

#ifdef PROTOCOL_C        
        RM2_Trasfer_ID();
        RM2_GetDevice(&rm2_device);
        serial = rm2_device.ID;
        UNB_Set_KEY_ptr(rm2_device.Key);
        random_seed = serial;
#else
        serial = NWave_GetSerial() | DEV_SID;
#endif        

        UNB_setPower(NormalPower_m6);
        UNB_FrequencyConf(UL_CENTER_FREQ - 25000, UL_CENTER_FREQ + 25000, 100);
	//UNB_FrequencyConf (868775000, 868825000, 100);
	//UNB_FrequencyConf (916475000, 916525000, 100);
	//UNB_FrequencyConf (866475000, 866525000, 100);
 
        // LED
	GPIO_PinModeSet(PORT_LED, PIN_LED, gpioModePushPull, 0);
	//set button
        GPIO_PinModeSet(PORT_BUTTON, PIN_BUTTON, gpioModeInputPullFilter, 1);
	// Set up a user hook on button pin interrupt
	int type = (MASK_BUTTON & TD_GPIO_ODD_MASK) ? TD_GPIO_USER_ODD : TD_GPIO_USER_EVEN;
	TD_GPIO_SetCallback(type, ButtonInterrupt, MASK_BUTTON);
	// Enable falling edge interrupts on button pin
	GPIO_IntConfig(PORT_BUTTON, PIN_BUTTON, false, true, true);
	// Clear and enable the corresponding interrupt in the CPU's Nested Vector
	// Interrupt Controller
	IRQn_Type irq_parity = (MASK_BUTTON & TD_GPIO_ODD_MASK) ? GPIO_ODD_IRQn : GPIO_EVEN_IRQn;
	NVIC_ClearPendingIRQ(irq_parity);
	NVIC_EnableIRQ(irq_parity);

	//TD_UART_Init(9600, false, false);

	iir_init(&iir_x);
	iir_init(&iir_y);
	iir_init(&iir_z);
        
        uint16_t i = 0;         
        
       initSensor();      

       TD_RTC_Delay(T5S);
//#ifndef RAWSEND	
	Send(0x0A);
//#endif
	WDOG_Feed();
	//TD_RTC_Delay(T10S);
	WDOG_Feed();
//#ifndef RAWSEND
//	UNB_FrequencyConf (868775000, 868825000, 100);//DEBUG
//	UNB_setPower (HighPower);//DEBUG
	//Send(0x0B);
       if(!doSelfTest())
       {
          iter = 8;
       }
       /*if(!doSelfTest()) 
       {
          while(1) { WDOG_Feed(); if(reed_switch_flag) BOOT_boot(0);}
       }*/
        
//#endif
	uint8_t calibration_finished = 0;

	while(!calibration_finished)
	{
		if(!getData(&magnetdata))       // 1st data fetch gets corrupted sometimes
		{
                     if(reed_switch_flag) BOOT_boot(0);
                     TD_RTC_Delay(T50MS);
                    getData(&magnetdata);
                }
                
                		
		calibration_finished = Calibrate(magnetdata);

#ifdef RAWSEND
		TD_LAN_frame_t Tx;
                
                //sprintf(Tx.payload, "x:%i,y:%i,z:%i,T:%i,Af:%i,R:%i,a:%i\n", 0, 0, 0, 0, 0, 0, 0 );
		Tx.payload[0] = INT16_HIGH_BYTE(magnetdata.x);
		Tx.payload[1] = INT16_LOW_BYTE(magnetdata.x);
		Tx.payload[2] = INT16_HIGH_BYTE(magnetdata.y);
		Tx.payload[3] = INT16_LOW_BYTE(magnetdata.y);
		Tx.payload[4] = INT16_HIGH_BYTE(magnetdata.z);
		Tx.payload[5] = INT16_LOW_BYTE(magnetdata.z);
                Tx.payload[6] = INT16_HIGH_BYTE(N/10);//INT16_HIGH_BYTE(iir_x.out);
                Tx.payload[7] = INT16_LOW_BYTE(N/10);
                Tx.payload[8] = 0;
                Tx.payload[9] = 0;
                Tx.payload[10] = 0;
                Tx.payload[11] = 0;
                Tx.payload[12] = 0;
                Tx.payload[13] = 0;
/*                float angle = acos((magnetdata.x*50 + magnetdata.y*50 + magnetdata.z*50)  /  (sqrt(magnetdata.x*magnetdata.x + magnetdata.y*magnetdata.y + magnetdata.z*magnetdata.z)  * (sqrt(50*50 + 50*50 + 50*50))));
                Tx.payload[14] = INT16_HIGH_BYTE(angle * 100);           //INT16_HIGH_BYTE(result*100);
                Tx.payload[15] = INT16_LOW_BYTE(angle * 100);//INT16_LOW_BYTE(result*100);*/
                Tx.payload[14] = 0;
                Tx.payload[15] = 0;
		Tx.header = 0;
		TD_LAN_Init(true, 0, 0);
		TD_LAN_SendFrame(1, &Tx, 0);
		TD_LAN_Release();
#endif

		TD_RTC_Delay(T50MS);
		WDOG_Feed();
	}
        
	SaveTempCorr();
	UpdateTempCorr();

        reed_switch_flag = 0;
        
//#ifndef RAWSEND
	//UNB_FrequencyConf (868775000, 868825000, 100);//DEBUG
	//UNB_setPower (HighPower);//DEBUG
	Send(0x0C);
//#endif
	//TD_RTC_Delay(T2S);
	RTC_CounterReset ();

	RTC_CompareSet(1, 32768); // 1000ms
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 *   every 250ms   void TD_RTC_Init(TD_RTC_handler_t function)
 ******************************************************************************/
uint8_t result = 0;
#define CORRECTION_TIMEOUT 60
uint32_t correction_counter = CORRECTION_TIMEOUT + 1;

extern magnetdata_t magnetDataCalibrate, magnetDataCorrection;

void TD_USER_Loop(void)
{
     //   static uint8_t from_200msec_to_one_sec = 0;
	WDOG_Feed();
	if(reed_switch_flag)	// Reed switch
	{
		reed_switch_flag = 0;
		BOOT_boot(0);
	}

	if((correction_counter++) > CORRECTION_TIMEOUT)
	{
		correction_counter = 0;
		UpdateTempCorr();
                TD_RTC_Delay(T10MS);
	}

	getData(&magnetdata);	// if reading is invalid, data is not updated and error counter increments

/*	TD_LAN_frame_t Tx;
	
        Tx.payload[0] = INT16_HIGH_BYTE(magnetdata.z);
        Tx.payload[1] = INT16_LOW_BYTE(magnetdata.z);*/
        
        
        ApplyTempCorr(&magnetdata);
	result = Compare_xyz(magnetdata);

#ifdef RAWSEND
        TD_LAN_frame_t Tx;
	//Tx.payload[0] = INT16_HIGH_BYTE(magnetdata.x - (int)iir_x.average);
	//Tx.payload[1] = INT16_LOW_BYTE(magnetdata.x - (int)iir_x.average);
	//Tx.payload[2] = INT16_HIGH_BYTE(magnetdata.y - (int)iir_y.average);
	//Tx.payload[3] = INT16_LOW_BYTE(magnetdata.y - (int)iir_y.average);
	//Tx.payload[4] = INT16_HIGH_BYTE(magnetdata.z - (int)iir_z.average);
	//Tx.payload[5] = INT16_LOW_BYTE(magnetdata.z - (int)iir_z.average);
	
        Tx.payload[2] = INT16_HIGH_BYTE(magnetdata.z);
        Tx.payload[3] = INT16_LOW_BYTE(magnetdata.z);
        Tx.payload[4] = INT16_HIGH_BYTE(iir_z.average);
        Tx.payload[5] = INT16_LOW_BYTE(iir_z.average);
        
        Tx.payload[6] = INT16_HIGH_BYTE(magnetDataCorrection.z);
        Tx.payload[7] = INT16_LOW_BYTE(magnetDataCorrection.z);
        //Tx.payload[2] = INT16_HIGH_BYTE(magnetdata.y);
        //Tx.payload[3] = INT16_LOW_BYTE(magnetdata.y);
        //Tx.payload[4] = INT16_HIGH_BYTE(magnetdata.z);
        //Tx.payload[5] = INT16_LOW_BYTE(magnetdata.z);
        
        //Tx.payload[6] = INT16_HIGH_BYTE(result*100);//INT16_HIGH_BYTE(iir_x.out);
	//Tx.payload[7] = INT16_LOW_BYTE(result*100);//TD_MEASURE_Temperature();//INT16_LOW_BYTE(iir_x.out);
	Tx.payload[8] = 0;//INT16_HIGH_BYTE(iir_x.out);//INT16_HIGH_BYTE(angle);//INT16_HIGH_BYTE(magnetDataCalibrate.x - magnetDataCorrection.x);//INT16_HIGH_BYTE(iir_y.out);
	Tx.payload[9] = TD_MEASURE_Temperature();//INT16_LOW_BYTE(iir_x.out);//INT16_LOW_BYTE(angle);//INT16_LOW_BYTE(magnetDataCalibrate.x - magnetDataCorrection.x);//INT16_LOW_BYTE(iir_y.out);
	Tx.payload[10] = 0;//INT16_HIGH_BYTE(Voltage);//INT16_HIGH_BYTE(iir_y.out);//INT16_HIGH_BYTE(result*100);//INT16_HIGH_BYTE(magnetDataCalibrate.y - magnetDataCorrection.y);//INT16_HIGH_BYTE(iir_z.out);
	Tx.payload[11] = 0;//INT16_LOW_BYTE(iir_y.out);//INT16_LOW_BYTE(result*100);//INT16_LOW_BYTE(magnetDataCalibrate.y - magnetDataCorrection.y);//INT16_LOW_BYTE(iir_z.out);
	Tx.payload[12] = INT16_HIGH_BYTE(iir_z.out);//INT16_HIGH_BYTE(magnetDataCalibrate.z - magnetDataCorrection.z);
	Tx.payload[13] = INT16_LOW_BYTE(iir_z.out);//INT16_LOW_BYTE(magnetDataCalibrate.z - magnetDataCorrection.z);
        Tx.payload[14] = INT16_HIGH_BYTE(angle);           //INT16_HIGH_BYTE(result*100);
	Tx.payload[15] = INT16_LOW_BYTE(angle);//INT16_LOW_BYTE(result*100);
	Tx.header = 0;
	TD_LAN_Init(true, 0, 0);
	TD_LAN_SendFrame(1, &Tx, 0);
	TD_LAN_Release();
//#else
#endif

	SendStrij(result);
        power_man_run_every_sec();

	//GPIO_PinOutClear(PORT_LED, PIN_LED);
	RTC_CounterReset();

       // GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0); //debug

}
