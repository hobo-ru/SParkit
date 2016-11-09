/*
 * flash.c
 *
 *  Created on: 12 июля 2014 г.
 *      Author: vdubikhin
 */

#ifndef FLASH_C_
#define FLASH_C_

//a little reference to SOTS II
#define END_OF_FLESH 0x8000

//Set start address and clear memory
void InitFlash(uint32_t address);

//Set new address
void SetAddress(uint32_t address);

//write data from buffer to flash
uint32_t WriteBuffer(uint8_t* buff, uint32_t size);

//write single word at specified address
//dont erase mem
void WriteWordFlash(uint32_t word, uint32_t address);

//read uint32_t from memory
//won't read more than has been written
uint32_t ReadMemory(uint8_t* buff, uint32_t size, uint32_t offset);


//read data from specified address
void ReadMemoryAddress(uint8_t* buff, uint32_t size, uint32_t address);

//Write all data to mem from internal buffer
uint32_t FlushBuffer();

#endif /* FLASH_C_ */
