//
//  RM2 Hardware Abstraction Layer
//  01.07.2015 S.Omelchenko
//  somelchenko@strij.net
//

#include "rm2_hal.h"

uint8_t Voltage;
uint8_t PowLev = 0;

uint32_t RM2_GetSerial()
{
        

        RM2_DEVICE dev;
   
        if(!TD_FLASH_ReadRegion(E2P_FACTORY + 0x20, &dev, sizeof(RM2_DEVICE))) return 0xffffffff; //default ID

        return dev.ID;
        
        
}

RM2_DEVICE* RM2_GetDevice(RM2_DEVICE* device)
{

         if(!TD_FLASH_ReadRegion(E2P_FACTORY + 0x20, device, sizeof(RM2_DEVICE))) return 0; 
        
         return device;
  
}


bool RM2_Trasfer_ID()
{
        
    RM2_DEVICE        rm2_device;
    TD_DEVICE         td_device;
    //  TWO_DEV           two_dev;
    
    if(!TD_FLASH_ReadRegion(RM2_TRANSFER, &rm2_device, sizeof(RM2_DEVICE))) return false;
    
      
    if(rm2_device.ID != 0)
    {
    
      TD_FLASH_ReadRegion(E2P_FACTORY, &td_device, sizeof(TD_DEVICE));
      TD_FLASH_ErasePage(E2P_FACTORY);
      TD_FLASH_WriteRegion_noErase(E2P_FACTORY, &td_device, sizeof(TD_DEVICE));
      TD_FLASH_WriteRegion_noErase(E2P_FACTORY + 0x20, &rm2_device, sizeof(RM2_DEVICE));      
      
    
    }
   // memset((void*)&device, 0xff , sizeof(RM2_DEVICE));
    
 //   TD_FLASH_ErasePage(RM2_TRANSFER);
    
    uint32_t buf[64];
    memset((void*)&buf[0], 0xaa , 256);
    
//    TD_FLASH_ErasePage(RM2_TRANSFER);
    TD_FLASH_WriteRegion(RM2_TRANSFER, &buf[0], 255);
        
    return true;
     
}


void battery_start_measure()
{
	//mode 0 - v

	/* Base the ADC configuration on the default setup. */
	ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
	ADC_Init_TypeDef init  = ADC_INIT_DEFAULT;

	/* Initialize timebase */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(40000, 0);
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init(ADC0, &init);

	/* Set input to temperature sensor. Reference must be 1.25V */
	single_init.reference = adcRef1V25;
	//  single_init.resolution = adcRes8Bit;
	single_init.input   	= adcSingleInpVDDDiv3;
	ADC_InitSingle(ADC0, &single_init);

	// Start one ADC sample
	ADC_Start(ADC0, adcStartSingle);

}

uint8_t battery_read_measure()
{
	uint32_t setpoint;
	setpoint = ADC_DataSingleGet(ADC0);
	setpoint = (setpoint * (125000 * 3 / 4096));	
        if (setpoint >= 300000) {
    	  setpoint = 0x80 + ((setpoint - 300000) / 1000);        				// msb = 1 -> 3V + n*(10mv)
      } else {
    	  setpoint = 0x00 + ((setpoint - 200000) / 1000);                       // msb = 0 -> 2V + n*(10mv)
      }
	CMU_ClockEnable(cmuClock_ADC0, false);
	return setpoint;
}

void power_man_run_after_send()
{
         Voltage = battery_read_measure();
         if(Voltage > 0x94) PowLev = 3;         //if Ubat>3.2V then Power  = NormalPower_m3
              else if(Voltage > 0x80) if(++PowLev > 3) PowLev = 3;       ////if Ubat>3V then PowerLevel Up
              else if(Voltage < 25) { PowLev = 0;}
              else if(Voltage < 80)
              {
                  if(--PowLev < 0) PowLev = 0;     //if Ubat<2.8V then PowerLevel down
              }
          switch(PowLev)
          {
                case 0:
                  UNB_setPower(NormalPower_m6);
                break;
                case 1:
                  UNB_setPower(NormalPower_m3);
                break;  
                case 2:
                  UNB_setPower(NormalPower);
                break;
                case 3:
                  UNB_setPower(HighPower);
                break;
          }
}


void power_man_run_every_sec()
{
    static uint32_t seconds = 0;
    static uint16_t DaysAfterDepas = 59;
    static uint16_t dep_timer;
    
    seconds++;

    if(!(DaysAfterDepas%DEP_PERIOD)&&(Voltage < 0x8A)) {dep_timer = 0; run_depassivation(true);}
    
    if(++dep_timer%DEP_DURATION) run_depassivation(false);
    
    if(!(seconds%(60*60*24))) DaysAfterDepas++;

}

void run_depassivation(bool a)
{
  // on/off depassivation switch
}
