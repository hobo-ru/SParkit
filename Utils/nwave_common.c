/*
 * nwave_common.c
 *
 *  Created on: Nov 6, 2014 ã.
 *      Author: S.Omelchenko
 */

#include "td1202.h"
#include "td_flash.h"
#include "efm32.h"
#include "nwave_common.h"

uint32_t NWave_GetSerial(void)
{
	TD_DEVICE device;
	const uint32_t* SerialPtr;
	
	while(TD_FLASH_DeviceRead(&device) == 0)
	{
		SerialPtr = (uint32_t*)0xfe0003c;
		device.Serial = *SerialPtr;
		device.Key1 = 0x7ef7cff5;
		device.Key2 = 0xb956e1c68842e550;
		device.ModResult = 0xf;
		device.ProdResult = 0x4;
		device.LedMask1 = 0x00;
		device.LedMask2 = 0x00;
		TD_FLASH_DeviceWrite(&device);
	}
	
	if(device.Serial == 0xFFFFFFFF)
	{
		SerialPtr = (uint32_t*)0xfe081f0;// unique_0, not full!
		device.Serial = *SerialPtr;
		TD_FLASH_DeviceWrite(&device);
	}
	
	return device.Serial;
}

