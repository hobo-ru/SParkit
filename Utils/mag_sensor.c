/*
 * mag_sensor.c
 *
 *  Created on: 06 рту. 2014 у.
 *      Author: vdubikhin
 */


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <em_i2c.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <td1202.h>

#include <td_rtc.h>

#include "mag_sensor.h"

#define SENSOR_ADDR 0x3C

extern uint16_t errors;



int initSensor()
{

      //initialiaze i2c
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    //turn on device
    GPIO_DriveModeSet(IO1_PORT, gpioDriveModeHigh);
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 1);

    TD_RTC_Delay(T50MS);

    //set SDA and SCL location
    GPIO_PinModeSet(SDA_PORT, SDA_BIT, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SCL_PORT, SCL_BIT, gpioModeWiredAnd, 1);

    /* Enable pins at location 3 (which is used on the DVK) */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                  I2C_ROUTE_SCLPEN |
                  (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);
	
    uint8_t initdata[3][2] = {{0x00, 0x70},{0x01, 0xA0},{0x02, 0x01/*0x00*/}};

    //uint8_t sensorSingle = 0;//0x03;
	//uint8_t a[2] = {0x02, 0x01};

    I2C_TransferSeq_TypeDef i2cFrame;
	
    i2cFrame.addr = SENSOR_ADDR;
	    
    int retCode;
        
    for(int i=0; i<3; i++)
    {
      i2cFrame.flags = I2C_FLAG_WRITE;
      i2cFrame.buf[0].data = &initdata[i][0];
      i2cFrame.buf[0].len = 2;		
      //initiate transfer
      I2C_TransferInit(I2C0, &i2cFrame);		
      //use polled transfer
      while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
      TD_RTC_Delay(T10MS);
    }
    
   
    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);

    return retCode;
 
}

int getData(magnetdata_t *data) {
    //initialiaze i2c
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    //turn on device
//    GPIO_DriveModeSet(IO1_PORT, gpioDriveModeHigh);
//    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 1);

//    TD_RTC_Delay(T50MS);

    //set SDA and SCL location
    GPIO_PinModeSet(SDA_PORT, SDA_BIT, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SCL_PORT, SCL_BIT, gpioModeWiredAnd, 1);

    /* Enable pins at location 3 (which is used on the DVK) */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                  I2C_ROUTE_SCLPEN |
                  (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);
	
    uint8_t sensorData[15];
    uint8_t sensorSingle = 0;//0x03;
    
    uint8_t a[2] = {0x02, 0x01};

    I2C_TransferSeq_TypeDef i2cFrame;
	
	i2cFrame.addr = SENSOR_ADDR;
	
	int retCode;
	
	
    i2cFrame.flags = I2C_FLAG_WRITE;
    i2cFrame.buf[0].data = a;
    i2cFrame.buf[0].len = 2;
	
	//initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
	
    TD_RTC_Delay(T10MS);
	
    i2cFrame.flags = I2C_FLAG_WRITE_READ;

    i2cFrame.buf[0].data = &sensorSingle;
    i2cFrame.buf[0].len = 1;

    i2cFrame.buf[1].data = sensorData;
    i2cFrame.buf[1].len = 13;//6;

    //initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

    //turn off device
//    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0); //debug

    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);

    data->x = (sensorData[3] << 8) + sensorData[4];
    data->z = (sensorData[5] << 8) + sensorData[6];
    data->y = (sensorData[7] << 8) + sensorData[8];
    
    if (retCode == 0) 
	{	
		if	(	(((sensorData[3] << 8) + sensorData[4]) == (uint16_t)(-4096))||
		   		(((sensorData[5] << 8) + sensorData[6]) == (uint16_t)(-4096))||
				(((sensorData[7] << 8) + sensorData[8]) == (uint16_t)(-4096))	)
		{
			errors++; 
			return 0;
		}

        return 1;
    } 
	else 
	{
		errors++;
//        *dataX = 0;
//        *dataY = 0;
//        *dataZ = 0;
    }

    return 0;
}

int getSelfTest(magnetdata_t *data, uint8_t direction) 
{
    //initialiaze i2c
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    //turn on device
    //GPIO_DriveModeSet(IO1_PORT, gpioDriveModeHigh);
    //GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 1);

    //TD_RTC_Delay(T50MS);

    //set SDA and SCL location
    GPIO_PinModeSet(SDA_PORT, SDA_BIT, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SCL_PORT, SCL_BIT, gpioModeWiredAnd, 1);

    /* Enable pins at location 3 (which is used on the DVK) */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                  I2C_ROUTE_SCLPEN |
                  (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);
	
    uint8_t sensorData[15];
    uint8_t sensorSingle = 0;//0x03;

    uint8_t selftest[3][2] = {{0x00, (direction)?0x71:0x72},{0x01, 0xA0},{0x02, 0x01/*0x00*/}};

    I2C_TransferSeq_TypeDef i2cFrame;
	
	i2cFrame.addr = SENSOR_ADDR;
	
	int retCode;
	
	for(int n=0; n<2; n++)
	{	
		for(int i=0; i<3; i++)
		{
			i2cFrame.flags = I2C_FLAG_WRITE;
			i2cFrame.buf[0].data = &selftest[i][0];
			i2cFrame.buf[0].len = 2;
			
			//initiate transfer
			I2C_TransferInit(I2C0, &i2cFrame);
			
			//use polled transfer
			while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
			TD_RTC_Delay(T100MS);
                        //TD_RTC_Delay(T10MS);
		}
		
		i2cFrame.flags = I2C_FLAG_WRITE_READ;
		
		i2cFrame.buf[0].data = &sensorSingle;
		i2cFrame.buf[0].len = 1;
		
		i2cFrame.buf[1].data = sensorData;
		i2cFrame.buf[1].len = 13;//6;
		
		//initiate transfer
		I2C_TransferInit(I2C0, &i2cFrame);
		
		//use polled transfer
		while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
	}
	//turn off device
//	GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0); //debug

        
    uint8_t initdata[2] = {0x00, 0x70};

      
    i2cFrame.flags = I2C_FLAG_WRITE;
    i2cFrame.buf[0].data = &initdata[0];
    i2cFrame.buf[0].len = 2;		
    //initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);		
    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
  	
	//initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
    
        
        
    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);

    if (retCode == 0) 
	{	
		
          if((((sensorData[3] << 8) + sensorData[4]) == -4096)||(((sensorData[5] << 8) + sensorData[6]) == -4096)||(((sensorData[7] << 8) + sensorData[8]) == -4096))
          {
            errors++; 
            return 0;
          }
                
        data->x = (sensorData[3] << 8) + sensorData[4];
        data->z = (sensorData[5] << 8) + sensorData[6];
        data->y = (sensorData[7] << 8) + sensorData[8];
        return 1;
    } 
	else 
	{
            errors++;
//        *dataX = 0;
//        *dataY = 0;
//        *dataZ = 0;
    }

    return 0;
}

magnetdata_t magnetDataCalibrate, magnetDataCorrection;

void SaveTempCorr(void)
{
	getSelfTest(&magnetDataCalibrate, 1);
}

void UpdateTempCorr(void)
{
	getSelfTest(&magnetDataCorrection, 1);
}

#define ABS(x) (((x)>0)?(x):(-(x)))
float tmp;
void ApplyTempCorr(magnetdata_t * data_in)
{
        
        tmp = (float)data_in->x/magnetDataCalibrate.x*(magnetDataCorrection.x - magnetDataCalibrate.x);
	data_in->x = ((int32_t)data_in->x) - (int32_t)tmp;//(magnetDataCalibrate.x - magnetDataCorrection.x);
        tmp = (float)data_in->y/magnetDataCalibrate.y*(magnetDataCorrection.y - magnetDataCalibrate.y);
	data_in->y = ((int32_t)data_in->y)  - (int32_t)tmp;// (magnetDataCalibrate.y - magnetDataCorrection.y);
        tmp = (float)data_in->z/magnetDataCalibrate.z*(magnetDataCorrection.z - magnetDataCalibrate.z);
	data_in->z = ((int32_t)data_in->z) - (int32_t)tmp; //(magnetDataCalibrate.z - magnetDataCorrection.z);
}