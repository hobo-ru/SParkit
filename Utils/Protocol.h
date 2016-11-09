/*
 * Protocol.h
 *
 *  Created on: 30 θώνÿ 2014 γ.
 *      Author: vdubikhin
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "Frame.h"
#include <stdlib.h>
#include <stdint.h>

#define MAX_SLAVES 20

typedef struct AddrType {
    uint32_t address;
    int32_t mask;
} AddrType;

typedef struct FailedDevice {
    uint32_t address;
    signed char error;
} FailedDevice;

extern void (*ProtocolLEDOn) (void);
extern void (*ProtocolLEDOff) (void);
extern void (*ProtocolLEDToggle) (void);

#ifdef MASTER

extern uint32_t deviceIDList[MAX_SLAVES];

/*
 * Initialize RF bus with address from destination
 */
void InitBus(AddrType destination, uint32_t listenTime);

/*
 * Obtain IDs of slaves connected to the bus
 * Return number of discovered devices
 */
int Discovery(uint8_t max_slaves);

/*
 * Send single command to slave and receive acknowledge
 * address - slave address
 * mask - send mask
 * payload - pointer to command
 * deviceFail - pointer to store IDs of devices failed to respond
 * return number of failed devices
 */

int SendCommand(AddrType address, uint8_t* payload, FailedDevice* deviceFail);

/*
 * Send data from flash. Calling command is responsible for initializing flash module and
 * writing data to flash
 * address - slave address
 * mask - send mask
 * size - size of data to read from flash
 * deviceFail - pointer to store IDs of devices failed to respond
 * return number of failed devices
 */
int SendData(AddrType address, uint32_t size, FailedDevice* deviceFail);

#endif

#ifdef SLAVE

extern uint32_t slaveAddr;

/*
 * Listen RF bus for init message
 * Return 1 if init frame received, 0 otherwise
 */
uint8_t Listen();

/*
 * Send request to register on the bus
 */
uint8_t Request();

/*
 * Receive command from master and store it in payload
 * return -1 in case of error, 0 in case of no command, 1 in case of command
 */
int ReceiveCommand(uint8_t* payload, uint8_t* payloadSend);

/*
 * Receive data from master and store it in flash
 * return -1 in case of error, 0 in case of no data, data size in bytes
 */
int ReceiveData(uint32_t address);


#endif

#endif /* PROTOCOL_H_ */
