/*
 * i2cEEPROM.h
 *
 *  Created on: 17 июля 2014 г.
 *      Author: vdubikhin
 */

#ifndef I2CEEPROM_H_
#define I2CEEPROM_H_

#include <stdlib.h>
#include <stdint.h>

/*
 * Copy image from flash to eeprom
 * return number of copied bytes
 */
int FlashToEEPROM(uint32_t flashAddress);

/*
 * Restore image in flash from eeprom
 * return number of copied bytes
 */
int EEPROMToFlash(uint32_t flashAddress);


/*
 * Fill EEPROM with 0
 */
int ClearEEPROM();

// Check that EEPROM works
int CheckEEPROM();

#endif /* I2CEEPROM_H_ */
