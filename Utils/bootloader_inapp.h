/*
 * bootloader_inapp.h
 *
 *  Created on: 7.10.2014
 *      Author: S.Omelchenko
 */

#ifndef BOOTLOADER_INAPP_H
#define BOOTLOADER_INAPP_H

#include <efm32.h>
#include <td1202.h>
#include <td_module.h>

void BOOT_boot(uint32_t address);

#endif