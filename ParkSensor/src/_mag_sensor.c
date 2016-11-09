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

int getData(int16_t* dataX, int16_t* dataY, int16_t* dataZ) {
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
	
    uint8_t sensorData[15];
    uint8_t sensorSingle = 0;//0x03;
	//uint8_t a[2] = {0x02, 0x01};

    I2C_TransferSeq_TypeDef i2cFrame;
	
	i2cFrame.addr = SENSOR_ADDR;
	
	int retCode;
	
	/*
	i2cFrame.flags = I2C_FLAG_WRITE;
	i2cFrame.buf[0].data = a;
    i2cFrame.buf[0].len = 2;
	
	//initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);
	TD_RTC_Delay(T200MS);
	*/
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
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0); //debug

    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);

    if (retCode == 0) 
	{	
		if	(	(((sensorData[3] << 8) + sensorData[4]) == -4096)||
		   		(((sensorData[5] << 8) + sensorData[6]) == -4096)||
				(((sensorData[7] << 8) + sensorData[8]) == -4096)	)
		{
			errors++;
			return 0;
		}
        *dataX = (sensorData[3] << 8) + sensorData[4];
        *dataZ = (sensorData[5] << 8) + sensorData[6];
        *dataY = (sensorData[7] << 8) + sensorData[8];
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
