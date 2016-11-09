/*
 * Protocol.c
 *
 *  Created on: 30 θώνÿ 2014 γ.
 *      Author: vdubikhin
 */

#include "Protocol.h"
#include "Frame.h"
#include "flash.h"

#include <stdlib.h>
#include <stdint.h>
#include <em_int.h>
#include <em_emu.h>
#include <core_cm3.h>
#include <em_wdog.h>

#include "td_rf_Si4461.h"
#include "td_rf.h"
#include <td_lan.h>
#include <td_rtc.h>
#include <td_uart.h>

void (*ProtocolLEDOn) (void);
void (*ProtocolLEDOff) (void);
void (*ProtocolLEDToggle) (void);

char* itoa(int n, char s[], int base);//TODO hide to .h

//number of ack messages to send
#define REPEAT_MESG 3

//time for slave to clear out memory
#define SLAVE_MEM_INIT 20000
//Maximum slave listen delay multiplier
#define MAX_DELAY_MULT 20
//Max number of tries for slave to try register on the network
#define MAX_TRIES 200
//Basic delay for slave bus listening
//Multiplied by random number to calculate total delay
#define TIME_LISTEN_MIN 210


//Time to initialize bus
#define TIME_INIT 32768

//Time for master to send discovery init frame
#define TIME_DISCOVERY_INIT 610
//Time for master to timeout on slave discovery
#define TIME_DISCOVERY_TIMEOUT (TIME_LISTEN_MIN*MAX_DELAY_MULT*2)
//Time for master to timeout on command response
#define TIME_COMMAND_TIMEOUT 6000 //20s
//Time for master to initiate data transfer
#define TIME_SENDDATA_INIT 2100
//Time master sends single data frame
#define TIME_SENDDATA_FRAME 410

//Time for slave to timeout on discovery start frame
#define TIME_START_TIMEOUT 32768 //20s
//Time for slave to timeout on command frame
#define TIME_COMMAND_RCV_TIMEOUT (TIME_DISCOVERY_TIMEOUT + 19000)
//Time for slave to timeout on data start frame
#define TIME_SENDDATA_RCV_INIT 10000
//Time for slave to timeout on data receive frame
#define TIME_SENDDATA_RCV_FRAME 200000

extern char stringbuf[];

//Time for slave to listen to bus init message
#define TIME_LISTEN 1900 //listen_time*delay time

//Basic delay before any communication(send/receive)
#define FRAME_DELAY 2000 // in ms/32768
//Small frame delay for data transfer
#define FRAME_DELAY_SMALL 200

enum FunctionType {
    Init = 1,
    Discover = 2,
    Req = 3,
    Ack = 4,
    Data = 5,
    Release = 6,
    Command = 7,
    CommandAck = 8,
    DataStart = 9,
    DataStop = 10,
    DataChunk = 11
};

TD_RF_param_t serviceFreq = {
	869800000, // Frequency (Hz)
	65000, // Baudrate (bps)
	20000, // Deviation (Hz)
	TD_RF_GFSK, // Modulation type
	10, // Power level (dBm)
	TD_RF_LOCAL // Data provider
};

static uint32_t timer = 0;
static AddrType masterDest = {0, -1};

static void RTCIntHandler() {
    timer++;
}

static void setRTCTimer(uint32_t delay) {
    RTC_CompareSet(1, delay);
    RTC_CompareSet(0, 0xFFFFFF);
    TD_RTC_SetUserHandler(RTCIntHandler);
    RTC_IntEnable(7);
    RTC_CounterReset();
}



//send frame over time
static void sendFrame(void* frame,TD_LAN_frame_t (*createFrame)(void*), uint32_t interval, uint32_t delay) {
    TD_LAN_frame_t TX;
    TX = (*createFrame)(frame);

    //Turn on LED
    if (ProtocolLEDOn)
        (*ProtocolLEDOn)();

    //Set RTC and TD RTC counter
    timer = 0;

    //Send frame for specified time
    uint32_t timeElapsed;
    timeElapsed = 0;
	
	//Turn on LAN module
    //TD_LAN_Init(true, 0, 0);

    while(timeElapsed < interval) 
	{
        //setRTCTimer(delay);
        //EMU_EnterEM2(false);
		TD_RTC_Delay(delay);

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);
		if (ProtocolLEDOn)
            (*ProtocolLEDOn)();

        TD_LAN_SendFrame(1, &TX, 0);

        //Turn off LAN module
        TD_LAN_Release();

        timeElapsed += delay;

        if (ProtocolLEDOff)
            (*ProtocolLEDOff)();

    }
	//Turn off LAN module
    //TD_LAN_Release();
	
    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

}

void InitBus(AddrType destination, uint32_t listenTime) {
    //Create INIT frame
    FrameUtility initFrame;
    initFrame.address = destination.address;
    initFrame.mask = destination.mask;
    initFrame.function = Init;

    masterDest.address = destination.address;
    masterDest.mask = destination.mask;

    sendFrame(&initFrame, &CreateFrameUtility, listenTime*TIME_INIT, FRAME_DELAY);
}

uint32_t deviceIDList[MAX_SLAVES];
static int numDevices;

int Discovery(uint8_t max_slaves) {

    //Check that bus is initialized
    if (masterDest.mask == -1)
        return -1;

    numDevices = 0;
	if((max_slaves == 0) || (max_slaves > MAX_SLAVES)) max_slaves = MAX_SLAVES;
    //Send Start frame
    FrameUtility frame;
    frame.address = masterDest.address;
    frame.mask = masterDest.mask;
    frame.function = Discover;

    sendFrame(&frame, &CreateFrameUtility, TIME_DISCOVERY_INIT, FRAME_DELAY_SMALL);

    //Set overflow time
    uint32_t timeElapsed = 0;

    //Turn on LED
    if (ProtocolLEDOn)
        (*ProtocolLEDOn)();

    uint8_t deviceExist;
    TD_LAN_frame_t RX, TX;
    //while not timeout and discovered devices < max
    while (numDevices < max_slaves && timeElapsed < TIME_DISCOVERY_TIMEOUT && TD_UART_AvailableChars() == 0) {
        //Configure RTC delay
        //setRTCTimer(FRAME_DELAY_SMALL);
        //EMU_EnterEM2(false);
		TD_RTC_Delay(FRAME_DELAY_SMALL);

        timeElapsed += FRAME_DELAY_SMALL;

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);

        // Listen for request
        if (TD_LAN_ReceiveFrame(0,0,&RX)) {
            frame = ParseFrameUtility(&RX);

            // if valid frame received
            if (frame.crc8 != 0) {
                // if request
                if (frame.function == Req) {
                    //reset timeout
                    timeElapsed = 0;

                    //send ack
                    frame.function = Ack;
                    TX = CreateFrameUtility(&frame);

                    for (int i = 0; i < REPEAT_MESG; i ++)
                        TD_LAN_SendFrame(1, &TX, 0);

                    //check if device exists
                    deviceExist = 0;
                    for (int i = 0; i < numDevices; i++) {
                        if (deviceIDList[i] == frame.address)
                            deviceExist = 1;
                    }
                    //add new device
                    if (!deviceExist) {
                        //save device address
                        deviceIDList[numDevices] = frame.address;
						TD_UART_SendString("\n");
						TD_UART_SendString(itoa(numDevices+1, stringbuf, 16));
						TD_UART_SendString(": 0x");
						TD_UART_SendString(itoa(deviceIDList[numDevices], stringbuf, 16));
                        //increment num of disc dev
                        numDevices += 1;
                    }

                }

            }
        }

        //Turn off LAN module
        TD_LAN_Release();

    }

    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();



    masterDest.mask = -1;

    return numDevices;
}

static int SendRcvCommand (AddrType address, uint8_t* payload, uint8_t* payloadRcv);

int SendCommand(AddrType address, uint8_t* payload, FailedDevice* deviceFail) {

    int devicesFailed = 0;

    if (numDevices == 0)
        return -1;

    //Turn on LED
    if (ProtocolLEDOn)
        (*ProtocolLEDOn)();

    AddrType destination;

    //iterate over discovered devices
    for (int i = 0; i < numDevices; i++) {

        //check device address against send address with mask
        if ( (deviceIDList[i] & address.mask) != (address.address & address.mask) )
            continue;

        destination.address = deviceIDList[i];
        destination.mask = address.mask;
        //if timeout
        if (SendRcvCommand(destination, payload, 0) == 0)
		{
            (deviceFail + devicesFailed)->address = deviceIDList[i];
            (deviceFail + devicesFailed)->error = -1;
            devicesFailed += 1;
        }
    }

    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

    return devicesFailed;

}

//Send command to slave with acknowledge
//Copy returned payload if buffer provided
static int SendRcvCommand (AddrType address, uint8_t* payload, uint8_t* payloadRcv) {
    FrameUtility frame;
    uint32_t timeElapsed = 0;
    TD_LAN_frame_t TX, RX;

    //create frame
    frame.address = address.address;
    frame.mask = address.mask;

    //copy data
    for (int j = 0; j < UTILITY_PAYLOAD; j++)
        frame.data[j] = *(payload + j);

    frame.function = Command;
    TX = CreateFrameUtility(&frame);
    timeElapsed = 0;

    TD_RTC_Delay(T100MS);

    while(timeElapsed < TIME_COMMAND_TIMEOUT) {
        //Configure RTC
        setRTCTimer(FRAME_DELAY);
        EMU_EnterEM2(false);

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);

        TD_LAN_SendFrame(1, &TX, 0);

        //wait for ack or timeout
        if (TD_LAN_ReceiveFrame(0,0,&RX)) 
		{
            frame = ParseFrameUtility(&RX);

            //check crc
            if (frame.crc8 != 0) 
			{
                //check slave return ack
                if ( frame.function == CommandAck &&
                     address.address == frame.address  ) 
				{

                    //Turn off LAN module
                    TD_LAN_Release();

                    //Toggle LED
                    if (ProtocolLEDToggle)
                        (*ProtocolLEDToggle)();

                    //copy returned data
                    if (payloadRcv) 
					{
                        for (int j = 0; j < UTILITY_PAYLOAD; j++)
                            *(payloadRcv + j) = frame.data[j];
                    }

                    return 1;
                }

            }
        }
        //Turn off LAN module
        TD_LAN_Release();

        timeElapsed += FRAME_DELAY;

    }

    // no response received
    return 0;
}


int SendData(AddrType address, uint32_t size, FailedDevice* deviceFail) {
    int devicesFailed = 0;

    if (numDevices == 0)
        return -1;
    FrameUtility frame;

    //create frame
    frame.address = address.address;
    frame.mask = address.mask;
    frame.function = DataStart;

    //send data start
    sendFrame(&frame, &CreateFrameUtility, TIME_SENDDATA_INIT, FRAME_DELAY_SMALL);

    uint32_t dataSize, totalSize = 0;
    uint32_t timeElapsed;
    TD_LAN_frame_t TX;
    FrameData frameData;

    frameData.iter = 0;

    TD_RTC_Delay(SLAVE_MEM_INIT);
	
	TD_LAN_Init(true, 0, 0);
    //read from memory till the end
    while ( ( dataSize = ReadMemory(frameData.data, DATA_PAYLOAD, totalSize) ) != 0) 
	{
        //initialize RTC
        //setRTCTimer(FRAME_DELAY_SMALL);

        frameData.dataSize = dataSize;

        timeElapsed = 0;
        //ovfCur = timer;
        TX = CreateFrameData(&frameData);
        //Send same data several times
        while(timeElapsed < TIME_SENDDATA_FRAME) 
		{
            //if(ovfCur != timer) 
			{
                //Turn on LAN module
                //TD_LAN_Init(true, 0, 0);

                TD_LAN_SendFrame(1, &TX, 0);

                //Turn off LAN module
                //TD_LAN_Release();
				
				RTC_CounterReset();
				TD_RTC_Delay(FRAME_DELAY_SMALL);

                // Reset timer
                timeElapsed += FRAME_DELAY_SMALL;
                //ovfCur = timer;
                //RTC_CounterReset();

                //Toggle LED
                if (ProtocolLEDToggle)
                    (*ProtocolLEDToggle)();
            }
        }

        frameData.iter += 1;
        totalSize += dataSize;
    }
	TD_LAN_Release();

    //initialize RTC
    setRTCTimer(FRAME_DELAY_SMALL);

    timeElapsed = 0;
    timer = 0;

    frameData.dataSize = 0;

    //send data stop
    sendFrame(&frameData, &CreateFrameData, TIME_SENDDATA_INIT, FRAME_DELAY_SMALL);

    uint8_t payloadCheck[UTILITY_PAYLOAD];
    uint8_t payloadRcv[UTILITY_PAYLOAD];

    //use checksum for now
    //switch to CRC32 later for transfered data check
    payloadCheck[0] = totalSize >> 24;
    payloadCheck[1] = totalSize >> 16;
    payloadCheck[2] = totalSize >> 8;
    payloadCheck[3] = totalSize;

    AddrType destination;

    for (int i = 0; i < numDevices; i++) {

        //check device address against send address with mask
        if ( (deviceIDList[i] & address.mask) != (address.address & address.mask) )
            continue;

        destination.address = deviceIDList[i];
        destination.mask = address.mask;

        //check received payload
        if ( SendRcvCommand(destination, payloadCheck, payloadRcv) ) {
            for (int j = 0; j < 4; j++) {
                if (payloadRcv[j] != payloadCheck[j]) {
                    (deviceFail + devicesFailed)->address = deviceIDList[i];
                    (deviceFail + devicesFailed)->error = -2;
                    devicesFailed += 1;
                    break;
                }
            }
        } else {
            //if no response received
            (deviceFail + devicesFailed)->address = deviceIDList[i];
            (deviceFail + devicesFailed)->error = -1;
            devicesFailed += 1;
        }
    }
    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

    return devicesFailed;
}



//Slave functions

uint32_t slaveAddr;

uint8_t Listen() 
{
    WDOG_Enable(true);

    //Listen for specified time
    uint32_t timeElapsed;
    timeElapsed = 0;

    TD_LAN_frame_t RX;
    FrameUtility frame;
    while(timeElapsed < TIME_LISTEN)
	{
        //set RTC and TD RTC counter
		TD_RTC_Delay(FRAME_DELAY);
        //setRTCTimer(FRAME_DELAY);
        //EMU_EnterEM2(false);
        WDOG_Feed();
        timeElapsed += FRAME_DELAY;
        if (ProtocolLEDOn)
            (*ProtocolLEDOn)();
        //turn on LAN module
        TD_LAN_Init(true, 0, 0);

        //If any frame received
        if (TD_LAN_ReceiveFrame(0,0,&RX))
		{
            //Check that its init frame
            frame = ParseFrameUtility(&RX);

            if (frame.crc8 != 0) 
			{
                if (frame.function == Init) 
				{
                    //Turn off LAN module
                    TD_LAN_Release();

                    //Turn off LED
                    if (ProtocolLEDOff)
                        (*ProtocolLEDOff)();

                    WDOG_Enable(false);

                    if ( (frame.address & frame.mask) == (slaveAddr & frame.mask) )
                        return 1;
                    else
                        return 0;
                }
            }
        }
        //Turn off LAN module
        TD_LAN_Release();
    }

    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

    WDOG_Enable(false);

    return 0;
}

uint8_t Request() {
    uint8_t ackRcvd = 0;

    //Turn on LED
    if (ProtocolLEDOn)
        (*ProtocolLEDOn)();

    WDOG_Enable(true);

    //Set overflow time
    uint32_t timeElapsed;
    timeElapsed = 0;
    timer = 0;

    TD_LAN_frame_t TX, RX;
    FrameUtility frame;

    uint8_t startReceived = 0;

    //Listen start frame
    while(timeElapsed < TIME_START_TIMEOUT && startReceived == 0) {
        //Configure RTC
        //setRTCTimer(FRAME_DELAY);
        //EMU_EnterEM2(false);
		TD_RTC_Delay(FRAME_DELAY);

        WDOG_Feed();

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);

        if (TD_LAN_ReceiveFrame(0,0,&RX)) {
            frame = ParseFrameUtility(&RX);

            if (frame.crc8 != 0) {
                if (frame.function == Init) {
                    timeElapsed = 0;
                }

                if (frame.function == Discover) {
                    startReceived = 1;
                    //Turn off LED
                      if (ProtocolLEDOff)
                          (*ProtocolLEDOff)();
                }
            }
        }
        //Turn off LAN module
        TD_LAN_Release();
        // Reset timer
        timeElapsed += FRAME_DELAY;
    }

    if (startReceived) {
        uint8_t numTries = 0;
        uint8_t numListen;
        uint8_t busBusy;
        uint16_t delayRTC;
        timeElapsed = 0;

        //while not max tries
        while (numTries < MAX_TRIES && !ackRcvd) {
            //calculate delay
            srand(numTries + slaveAddr + timeElapsed);
            numListen = rand() % MAX_DELAY_MULT;
            delayRTC = FRAME_DELAY_SMALL + (FRAME_DELAY_SMALL/2)*(numListen/MAX_DELAY_MULT);


            timeElapsed = 0;
            timer = 0;
            busBusy = 0;

            //listen
            while(timeElapsed < TIME_LISTEN_MIN*numListen && !busBusy) {
                //Configure RTC
                //setRTCTimer(delayRTC);
                //EMU_EnterEM2(false);
				TD_RTC_Delay(delayRTC);

                WDOG_Feed();

                //Turn on LAN module
                TD_LAN_Init(true, 0, 0);

                //listen
                busBusy = TD_LAN_ReceiveFrame(0,0,&RX);

                //Toggle LED
                if (ProtocolLEDToggle)
                    (*ProtocolLEDToggle)();

                //Turn off LAN module
                TD_LAN_Release();

                // Reset timer
                timeElapsed += delayRTC;
           }

            //if no frames received
            if (!busBusy) {

                WDOG_Feed();

                //Turn on LED
                if (ProtocolLEDOn)
                    (*ProtocolLEDOn)();

                //send request
                frame.address = slaveAddr;
                frame.function = Req;

                //Turn on LAN module
                TD_LAN_Init(true, 0, 0);

                TX = CreateFrameUtility(&frame);
                TD_LAN_SendFrame(1, &TX, 0);

                //wait for ack or timeout
                if (TD_LAN_ReceiveFrame(0,0,&RX)) {

                    frame = ParseFrameUtility(&RX);
                    if (frame.crc8 != 0) {
                        //if received ack for our address
                        if (frame.function == Ack && frame.address == slaveAddr)
                            ackRcvd = 1;
                    }
                }
                //Turn off LAN module
                TD_LAN_Release();
            }

            numTries++;
        }
    }

    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

    WDOG_Enable(false);

    return ackRcvd;
}

//Change payloadSend to callback function which fills reply buffer based in received command
int ReceiveCommand(uint8_t* payload, uint8_t* payloadSend) {

    WDOG_Enable(true);

    //Turn on LED
    if (ProtocolLEDOn)
       (*ProtocolLEDOn)();

    //Set overflow time
    uint32_t timeElapsed;
    timeElapsed = 0;
    timer = 0;

    TD_LAN_frame_t TX, RX;
    FrameUtility frame;

    while(timeElapsed < TIME_COMMAND_RCV_TIMEOUT) 
	{
        //Configure RTC
        //setRTCTimer(FRAME_DELAY);
        //EMU_EnterEM2(false);
		TD_RTC_Delay(FRAME_DELAY);

        WDOG_Feed();

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);

        //wait for command or timeout
        if (TD_LAN_ReceiveFrame(0,0,&RX)) {
            frame = ParseFrameUtility(&RX);

            if (frame.crc8 != 0) {
                //if some activity on the bus
                if (frame.function >= Req)
                    timeElapsed = 0;

                //if Command received
                if (frame.function == Command) {
                    timeElapsed = 0; //bus is busy

                    //if received command for our address
                    //for now address is 16 bit
                    if ( slaveAddr == frame.address ) {
                        //send ack to master
                        frame.address = slaveAddr;
                        frame.function = CommandAck;

                        //copy command payload
                        for (int i = 0; i < PAYLOAD_SIZE - 2; i++)
                            *(payload + i) = frame.data[i];

                        //copy reply if any exits
                        if (payloadSend) {
                            for (int i = 0; i < PAYLOAD_SIZE - 2; i++)
                                frame.data[i] = *(payloadSend + i);
                        } else {
                            for (int i = 0; i < PAYLOAD_SIZE - 2; i++)
                                frame.data[i] = 0;
                        }

                        TX = CreateFrameUtility(&frame);

                        for (int i = 0; i < REPEAT_MESG; i ++)
                            TD_LAN_SendFrame(1, &TX, 0);

                        if (ProtocolLEDToggle)
                            (*ProtocolLEDToggle)();

                        //Turn off LAN module
                        TD_LAN_Release();

                        WDOG_Enable(false);

                        return 1;
                    }
                }
            }
        }

        //Turn off LAN module
        TD_LAN_Release();

        timeElapsed += FRAME_DELAY;
    }

    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();
 
    WDOG_Enable(false);

    return 0;
}

int ReceiveData(uint32_t address) {
    //Turn on LED
    if (ProtocolLEDOn)
        (*ProtocolLEDOn)();

    WDOG_Enable(true);

    //Set overflow time
    uint32_t timeElapsed, ovfCur;
    timeElapsed = 0;
    timer = 0;
    ovfCur = timer;

    TD_LAN_frame_t RX;
    FrameUtility frame;

    //send start packet
    while(timeElapsed < TIME_SENDDATA_RCV_INIT) {
        //Configure RTC
        //setRTCTimer(FRAME_DELAY_SMALL);
        //EMU_EnterEM2(false);
		TD_RTC_Delay(FRAME_DELAY_SMALL);

        WDOG_Feed();

        //Turn on LAN module
        TD_LAN_Init(true, 0, 0);

        if (TD_LAN_ReceiveFrame(0,0,&RX)) 
		{
            frame = ParseFrameUtility(&RX);

            if (frame.crc8 != 0) 
			{
                if (frame.function == Command)
                    timeElapsed = 0;

                //if data start received
                if (frame.function == DataStart) 
				{
                    //if received data start for our address
                    if ( (frame.address & frame.mask) == (slaveAddr & frame.mask) ) 
					{
                        //Turn off LAN module
                        TD_LAN_Release();
                        break;
                    }

                }
            }
        }

        //Turn off LAN module
        TD_LAN_Release();

        // Reset timer
        timeElapsed += FRAME_DELAY_SMALL;
    }

    WDOG_Enable(false);

    //timeout
    if (timeElapsed >= TIME_SENDDATA_RCV_INIT)
        return -1;

    WDOG_Enable(true);

    //Configure RTC
    setRTCTimer(FRAME_DELAY_SMALL);

    //Set overflow time
    timeElapsed = 0;
    timer = 0;
    ovfCur = timer;

    //init mem
    InitFlash(address);

    uint8_t curIter = 0;
    uint32_t totalData = 0;
    FrameData frameData;
	
	//debug vars
	uint8_t crc_err = 0;

    while(timeElapsed < TIME_SENDDATA_RCV_FRAME) {
        WDOG_Feed();
        if(ovfCur != timer) {
           timeElapsed += FRAME_DELAY_SMALL;
           ovfCur = timer;

           //Turn on LAN module
           TD_LAN_Init(true, 0, 0);

           //if data received
           if (TD_LAN_ReceiveFrame(0,0,&RX)) 
		   {
               frameData = ParseFrameData(&RX);

               if (frameData.crc8 != 0) 
			   {
                   //if data chunk - write to memory
                   if (frameData.dataSize != 0) 
				   {
                       //if curent iter is the same as of chunk
                       if (curIter == frameData.iter) 
					   {
                           curIter += 1;

                           WriteBuffer(frameData.data, frameData.dataSize);
                           totalData += frameData.dataSize;

                           timeElapsed = 0;
                           //Toggle LED
                           if (ProtocolLEDToggle)
                               (*ProtocolLEDToggle)();

                       }
					   else 
					   {
                           //lost packets - exit
                           if ( (curIter & 0x0F) - frameData.iter > 1 ) 
						   {
                               //Turn off LAN module
                              TD_LAN_Release();

                              return -1;
                           }
                       }
                   } 
				   else 
				   {
                       timeElapsed = 0;
                       FlushBuffer();
                       //Turn off LAN module
                       TD_LAN_Release();
                       break;
                   }

               }
			   else
			   {
				   //crc fail
				   crc_err++;
			   }
           }

           //Turn off LAN module
           TD_LAN_Release();

           setRTCTimer(FRAME_DELAY_SMALL);
        }
    }

    WDOG_Enable(false);

    if (timeElapsed >= TIME_SENDDATA_RCV_FRAME)
        return -1;

    uint8_t dataSizeSlave[UTILITY_PAYLOAD];
    uint8_t dataSizeMaster[UTILITY_PAYLOAD];

    dataSizeSlave[0] = totalData >> 24;
    dataSizeSlave[1] = totalData >> 16;
    dataSizeSlave[2] = totalData >> 8;
    dataSizeSlave[3] = totalData;

    ReceiveCommand(dataSizeMaster, dataSizeSlave);

    for (int j = 0; j < 4; j++) {
        if (dataSizeSlave[j] != dataSizeMaster[j]) {
            //data check failed
            return -1;
        }
    }

    //write data size to flash and restore address
    WriteWordFlash(totalData, END_OF_FLESH - sizeof(totalData));
    SetAddress(address);
    //Turn off LED
    if (ProtocolLEDOff)
        (*ProtocolLEDOff)();

    return totalData;
}

