/*
 * flash.c
 *
 *  Created on: 12 июля 2014 г.
 *      Author: vdubikhin
 */
#include <stdlib.h>
#include <stdint.h>
#include <em_msc.h>
#include <em_dma.h>
#include <efm32.h>
#include <em_int.h>

#include "flash.h"

static uint32_t startAddress;
static uint32_t curAddress;
static uint32_t bytePointer; //how many bytes has been written
static uint32_t wordBuffer; //write bytes into this buffer 1st


static __ramfunc void ErasePage(uint32_t blockStart)
{
   MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

   // Load address
   MSC->ADDRB    = blockStart;
   MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

   // Send Erase Page command
   MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

   // Wait for erase to complete
   while ((MSC->STATUS & MSC_STATUS_BUSY)) {
       ;
   }
}

__ramfunc void WriteWord(uint32_t address, uint32_t data)
{
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Load address
    MSC->ADDRB    = address;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    // Load data
    MSC->WDATA    = data;

    // Trigger write once
    MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

    // Wait for the write to complete
    while ((MSC->STATUS & MSC_STATUS_BUSY)) {
        ;
    }
}

static void FlashInit(void)
{
    // Enable writing to the MSC
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Unlock the MSC
    MSC->LOCK = MSC_UNLOCK_CODE;

    // Disable writing to the MSC
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

/***************************************************************************//**
 * @brief
 *   Disables the flash controller for writing.
 ******************************************************************************/
static void FlashDeinit(void)
{
    // Enable writing to the MSC
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Lock the MSC
    MSC->LOCK = 0;

    // Disable writing to the MSC
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

//Set start adress and reinitialize internal variables
void InitFlash(uint32_t address) {
    startAddress = address;
    curAddress = address;
    bytePointer = 0;
    wordBuffer = 0;
    INT_Disable();
    FlashInit();

    //Erase memory to enable writing
    for (uint32_t i = address; i < END_OF_FLESH; i += 512)
        ErasePage(i);

    INT_Enable();
}

void SetAddress(uint32_t address) {
    startAddress = address;
    curAddress = address;
}

void WriteWordFlash(uint32_t word, uint32_t address) {
    WriteWord(address, word);
}

//write data from buffer to flash
uint32_t WriteBuffer(uint8_t* buff, uint32_t size) {
    INT_Disable();
    for (uint32_t i = 0; i < size; i++) {
        //shift word buffer by one byte and write new byte
        wordBuffer |= *(buff+i) << (bytePointer++ % 4 * 8) ;
        //wordBuffer = (wordBuffer << (bytePointer++ % 4 ? 8:0) ) | *(buff+i);
        if (bytePointer % 4 == 0) {
            if (curAddress > END_OF_FLESH)
                return END_OF_FLESH;
            WriteWord(curAddress, wordBuffer);
            curAddress += 4;
            wordBuffer = 0;
        }
    }

    INT_Enable();
    return curAddress;
}

//read uint32_t from memory
//won't read more than has been written
uint32_t ReadMemory(uint8_t* buff, uint32_t size, uint32_t offset) {
    INT_Disable();
    uint32_t address = startAddress + offset;
    uint8_t* curReadAddr = (uint8_t *) address;
    uint32_t i;

    for (i = 0; i < size; i++) {
        if (curReadAddr + i > (uint8_t *)(curAddress - 4))
            break;
        *(buff + i) = *(curReadAddr + i);
    }

    INT_Enable();
    return i;
}


//read uint32_t from memory
//won't read more than has been written
void ReadMemoryAddress(uint8_t* buff, uint32_t size, uint32_t address) {
    INT_Disable();
    uint8_t* curReadAddr = (uint8_t *) address;
    uint32_t i;

    for (i = 0; i < size; i++) {
        *(buff + i) = *(curReadAddr + i);
    }

    INT_Enable();
}

//Write all data to mem from internal buffer
//return total written size
uint32_t FlushBuffer() {
    INT_Disable();

    if (bytePointer % 4 != 0) {
        //fill the rest of wordbuffer with FF to ensure nondestructive write
        wordBuffer = 0xFFFFFFFF << (bytePointer % 4 * 8) | wordBuffer;
        WriteWord(curAddress, wordBuffer);
        curAddress += 4;
    }

    wordBuffer = 0;

    FlashDeinit();
    INT_Enable();
    return curAddress - startAddress;
}


