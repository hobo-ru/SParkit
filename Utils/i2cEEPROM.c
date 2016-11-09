/*
 * i2cEEPROM.c
 *
 *  Created on: 17 июля 2014 г.
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

#include "i2cEEPROM.h"
#include "flash.h"

#define EEPROM_ADDR 0xA0
#define PAGE_SIZE 64

// define where output led port and pin located
#define PORT_LED TIM1_PORT
#define PIN_LED TIM1_BIT

#define MAX_TRIES 10000

static void InitDevice() {
    //initialiaze i2c
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    //turn on device
    GPIO_DriveModeSet(IO1_PORT, gpioDriveModeHigh);
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 1);
    TD_RTC_Delay(T1S);

    //set SDA and SCL location
    GPIO_PinModeSet(SDA_PORT, SDA_BIT, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SCL_PORT, SCL_BIT, gpioModeWiredAnd, 1);

    /* Enable pins at location 3 (which is used on the DVK) */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                  I2C_ROUTE_SCLPEN |
                  (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);
}

static void DeinitDevice() {
    //turn off device
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0);

    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);
}

int FlashToEEPROM(uint32_t address) {
    InitDevice();

    //determine total write size
    uint32_t totalSize;
    totalSize = END_OF_FLESH - address;

    //check size, it must not be all 0(no firmware) or 1(page erased)
    if (totalSize == 0 || totalSize == 0xFFFFFFFF)
        return -10;

    uint32_t curSize;
    uint8_t tempData[PAGE_SIZE + 2];
    int retCode = -7;

    for (uint32_t i = 0; i < totalSize; i += PAGE_SIZE) {
        //determine read size based on the amount of left data
        curSize = totalSize - i > PAGE_SIZE ? PAGE_SIZE : totalSize - i;

        //write data to eeprom
        I2C_TransferSeq_TypeDef i2cFrame;
        i2cFrame.addr = EEPROM_ADDR;
        i2cFrame.flags = I2C_FLAG_WRITE;

        //read data from memory
        ReadMemoryAddress(&tempData[2], curSize, address + i);

        tempData[0] = i >> 8; //High addr
        tempData[1] = i & 0xFF; //low addr

        i2cFrame.buf[0].data = tempData;
        i2cFrame.buf[0].len = curSize + 2;

        //initiate transfer
        I2C_TransferInit(I2C0, &i2cFrame);

        //use polled transfer
        while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

        TD_RTC_Delay(T100MS);
    }

    //wait some time to ensure all write operations completed
    TD_RTC_Delay(T1S);

    DeinitDevice();

    return retCode;
}

int EEPROMToFlash(uint32_t flashAddress) {
    InitDevice();

    /*
     * Determine data size to copy from eeprom
     */


    //write data to eeprom
    I2C_TransferSeq_TypeDef i2cFrame;
    i2cFrame.addr = EEPROM_ADDR;
    i2cFrame.flags = I2C_FLAG_WRITE_READ;

    //send static data for now
    uint8_t addr[2] = {0, 0};
    uint8_t dataRcvd[PAGE_SIZE];
    uint32_t totalSize = 0;

    //determine total data to copy
    i2cFrame.buf[0].data = addr;
    i2cFrame.buf[0].len = 2;

    i2cFrame.buf[1].data = (uint8_t*) &totalSize;
    i2cFrame.buf[1].len = 4;


    int retCode;
    int numTries = 0;
    //initiate transfer
    retCode = I2C_TransferInit(I2C0, &i2cFrame);


    //use polled transfer
    while ( retCode == i2cTransferInProgress ) {
        retCode = I2C_Transfer(I2C0);
        if (numTries++ >= MAX_TRIES) {
            retCode = -7;
            break;
        }
    }

    //if some error during transfer happened
    if (retCode != 0) {
        DeinitDevice();
        return retCode;
    }


    /*
     * Copy data from eeprom to flash
     */
    //check that firmware is available in eeprom
    if (totalSize == 0 || totalSize == 0xFFFFFFFF)
        return -8;

    totalSize = END_OF_FLESH - flashAddress;
    //prepare flash
    InitFlash(flashAddress);
    uint32_t curSize;
    //read from eeprom data and data size(4 bytes)
    for (uint32_t i = 0; i < totalSize; i += PAGE_SIZE) {
        //determine current read size
        curSize = totalSize - i > PAGE_SIZE ? PAGE_SIZE : totalSize - i;

        addr[0] = i >> 8; //High addr
        addr[1] = i & 0xFF; //low addr

        //addr to read from
        i2cFrame.buf[0].data = addr;
        i2cFrame.buf[0].len = 2;

        i2cFrame.buf[1].data = dataRcvd;
        i2cFrame.buf[1].len = curSize;

        //initiate transfer
        I2C_TransferInit(I2C0, &i2cFrame);

        //use polled transfer
        while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

        //write received data to flash
        WriteBuffer(dataRcvd, curSize);
        TD_RTC_Delay(T100MS);

    }
    //flush internal buffer to accomplish data transfer
    FlushBuffer();

    DeinitDevice();

    return retCode;
}

int ClearEEPROM() {
    //initialiaze i2c
    InitDevice();

    uint8_t tempData[PAGE_SIZE + 2];
    int retCode = -7;

    //write data to eeprom
    I2C_TransferSeq_TypeDef i2cFrame;
    i2cFrame.addr = EEPROM_ADDR;
    i2cFrame.flags = I2C_FLAG_WRITE;

    //fill 1st page with zeroes for now
    for (int i = 0; i < PAGE_SIZE + 2; i++)
        tempData[i] = 0;

    i2cFrame.buf[0].data = tempData;
    i2cFrame.buf[0].len = PAGE_SIZE + 2;

    //initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

    //wait some time to ensure all write operations completed
    TD_RTC_Delay(T1S);

    DeinitDevice();

    return retCode;
}

int CheckEEPROM() {
    //initialiaze i2c
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    //turn on device
    GPIO_DriveModeSet(IO1_PORT, gpioDriveModeHigh);
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 1);
    TD_RTC_Delay(T1S);

    //set SDA and SCL location
    GPIO_PinModeSet(SDA_PORT, SDA_BIT, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SCL_PORT, SCL_BIT, gpioModeWiredAnd, 1);

    /* Enable pins at location 3 (which is used on the DVK) */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                  I2C_ROUTE_SCLPEN |
                  (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);

    uint8_t tempData[PAGE_SIZE + 2];
    int retCode = -7;

    //write data to eeprom
    I2C_TransferSeq_TypeDef i2cFrame;
    i2cFrame.addr = EEPROM_ADDR;
    i2cFrame.flags = I2C_FLAG_WRITE;

    //fill 1st page
    for (int i = 0; i < PAGE_SIZE + 2; i++)
        tempData[i + 2] = i;

    tempData[0] = 0;
    tempData[1] = 0;

    i2cFrame.buf[0].data = tempData;
    i2cFrame.buf[0].len = PAGE_SIZE + 2;

    //initiate transfer
    I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

    //wait some time to ensure all write operations completed
    TD_RTC_Delay(T1S);

    //read data to eeprom
    i2cFrame.addr = EEPROM_ADDR;
    i2cFrame.flags = I2C_FLAG_WRITE_READ;

    //send static data for now
    uint8_t addr[2] = {0, 0};

    //determine total data to copy
    i2cFrame.buf[0].data = addr;
    i2cFrame.buf[0].len = 2;

    i2cFrame.buf[1].data = tempData;
    i2cFrame.buf[1].len = PAGE_SIZE;

    //initiate transfer
    retCode = I2C_TransferInit(I2C0, &i2cFrame);

    //use polled transfer
    while ((retCode = I2C_Transfer(I2C0)) == i2cTransferInProgress);

    //if some error during transfer happened
    if (retCode != 0) {
        DeinitDevice();
        return retCode;
    }

    // Check returned data
    for (int i = 0; i < PAGE_SIZE; i++) {
        if (tempData[i] != i) {
            retCode = -8;
            break;
        }
    }

    //turn off device
    GPIO_PinModeSet(IO1_PORT, IO1_BIT, gpioModePushPull, 0);

    //deinit i2c after transfer
    I2C_Enable(I2C0, false);
    CMU_ClockEnable(cmuClock_I2C0, false);

    return retCode;
}



