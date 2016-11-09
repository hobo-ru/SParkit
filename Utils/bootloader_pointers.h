/*
 * bootloader_pointers.h
 *
 *  Created on: Dec 11, 2014 ã.
 *      Author: S.Omelchenko
 */

#ifndef BOOTLOADER_POINTERS_H_
#define BOOTLOADER_POINTERS_H_

#define FIRWMARE_STORAGE 0x3800
#define BOOTLOADER_SIZE FIRWMARE_STORAGE	//TODO remove these twin defines (see bootloader's main.c)

#define TD_CMU_INIT_PTR 		(BOOTLOADER_SIZE - (1*sizeof(uint32_t)))
#define BOOT_BOOT_PTR 			(BOOTLOADER_SIZE - (2*sizeof(uint32_t)))
#define RTC_INIT_PTR 			(BOOTLOADER_SIZE - (3*sizeof(uint32_t)))
#define TD_GPIO_INIT_PTR 		(BOOTLOADER_SIZE - (4*sizeof(uint32_t)))
#define TD_LAN_INIT_PTR 		(BOOTLOADER_SIZE - (5*sizeof(uint32_t)))
#define TD_LAN_RECEIVEFRAME_PTR (BOOTLOADER_SIZE - (6*sizeof(uint32_t)))
#define TD_LAN_SENDFRAME_PTR 	(BOOTLOADER_SIZE - (7*sizeof(uint32_t)))
#define TD_LAN_RELEASE_PTR 		(BOOTLOADER_SIZE - (8*sizeof(uint32_t)))

// How to use:
//ptr = (void(*)(const RTC_Init_TypeDef *init))(*((uint32_t*)RTC_INIT_PTR));
// ptr(0);

void BootloaderPointers_Save(void);
void BootloaderPointers_Retreive(void);

#endif