/*
 * packet.c
 *
 *  Created on: 13 марта 2014 г.
 *      Author: vdubikhin
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "packet.h"

static uint8_t bytePointer; //Current byte to write in sigfox array
static uint8_t bitsLeft; //Number of bits left in current byte

void initSegfoxData(SigfoxData* dataOut) {
    uint8_t i;

    for (i = 0; i < SIGFOX_MAX_BYTES; i ++)
        dataOut->array[i] = 0;

    bitsLeft = 8;
    bytePointer = 0;
}

//write data into segfox
//returns number of bytes written
uint8_t writeSegfoxData(uint8_t data, uint8_t size, SigfoxData* dataOut) {
    if (size > 8 )  return bytePointer + 1;

    uint8_t mask;
    mask = 0xFF >> (8 - size); //create mask for data
    if ( (bitsLeft - size) >= 0) {

        dataOut->array[bytePointer] |= (data & mask) << (bitsLeft - size); //data can be completely written into current byte

        if ((bitsLeft - size) == 0) {//if current byte is fully written
            //iterate to next sigfox byte
            bytePointer = bytePointer + 1;
            bitsLeft = 8;
            return bytePointer;
        } else {
            //else - calculate number of bits left
            bitsLeft = bitsLeft - size;
        }
    } else {

        dataOut->array[bytePointer] |= (data & mask) >> (size - bitsLeft); //write high portion in current byte

        if (bytePointer + 1 >= SIGFOX_MAX_BYTES) return 0; //should never happen

        dataOut->array[bytePointer+1] = data << (8 - (size - bitsLeft)); //write lower portion in next byte

        bitsLeft = 8 - (size - bitsLeft);
        bytePointer = bytePointer + 1;
    }

    return bytePointer + 1;
}

/*
 * Create packet of type 0 and place it in segfoxArray
 * Return number of bytes to send or -1 in case of invalid parameters
 *
 * Type 0 packet(MSB 1st):
 * |Iter|Type|Data0|Data1|....|DataN|
 */
int8_t createPacket0 (Packet* packet, SigfoxData* dataOut) {

    int8_t i, packetSize;

    //Check packet parameters
    if (6 + packet->numData * packet->dataSize > SIGFOX_MAX_BYTES * 8)
        return -1;

    //Initialize array 1st
    initSegfoxData(dataOut);

    //Write header info
    packetSize = writeSegfoxData(packet->iterator, 4, dataOut);
    packetSize = writeSegfoxData(TYPE0, 2, dataOut);

    for (i = 0; i < packet->numData; i++){
        if ( packet->dataSize > 8 ) {
            packetSize = writeSegfoxData(packet->data[2 * i], packet->dataSize - 8, dataOut); //write high byte
            packetSize = writeSegfoxData(packet->data[2 * i + 1], 8, dataOut); //write low byte
        }
        else {
            packetSize = writeSegfoxData(packet->data[i], packet->dataSize, dataOut);
        }
    }

    if (packetSize % 2 != 0)
        dataOut->size = packetSize + 1;
    else
        dataOut->size = packetSize;

    return dataOut->size;
}

/*
 * Create packet of type 1 and place it in segfoxArray
 * Return number of bytes to send or -1 in case of invalid parameters
 *
 * Type 1 packet(MSB 1st):
 * |Iter|Type|Mask|Datak0|Datak1|....|DatakN|
 * Data bundle is skipped if corresponding bit in mask is set to 0
 */
int8_t createPacket1 (Packet* packet, SigfoxData* dataOut) {
    int8_t i, packetSize;

    //Check packet parameters

    uint32_t comparison0 = 6 + packet->dataSize + packet->numData * packet->dataSize;
    uint32_t comparison1 = SIGFOX_MAX_BYTES * 8;

    if (comparison0 > comparison1 || packet->numData > 8)
        return -1;

//wierd compiler behaviour
//    if (6 + packet->dataSize + packet->numData * packet->dataSize > SIGFOX_MAX_BYTES * 8) {
//            //|| (packet->numData > 8))
//        return -1;
//    }

    //Initialize array 1st
    initSegfoxData(dataOut);

    //Write header info
    packetSize = writeSegfoxData(packet->iterator, 4, dataOut);
    packetSize = writeSegfoxData(TYPE1, 2, dataOut);
    packetSize = writeSegfoxData(packet->mask, packet->numData, dataOut);

    for (i = 0; i < packet->numData; i ++) {
        // if ith bit in mask is not 0 - send data
        if ( packet->mask & 1 << i ) {
            if ( packet->dataSize > 8 ) {
                packetSize = writeSegfoxData(packet->data[2 * i], packet->dataSize - 8, dataOut); //write high byte
                packetSize = writeSegfoxData(packet->data[2 * i + 1], 8, dataOut); //write low byte
            }
            else {
                packetSize = writeSegfoxData(packet->data[i], packet->dataSize, dataOut);
            }
        }
    }

    if (packetSize % 2 != 0)
        dataOut->size = packetSize + 1;
    else
        dataOut->size = packetSize;

    return dataOut->size;
}
