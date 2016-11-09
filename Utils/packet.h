/*
 * packet.h
 *
 *  Created on: 13 марта 2014 г.
 *      Author: vdubikhin
 */

#ifndef PACKET_H_
#define PACKET_H_

#define SIGFOX_MAX_BYTES 12


typedef struct Packet {
    uint8_t iterator; //4 bits

#define TYPE0 2 // 10 in bin
#define TYPE1 0 // 00 in bin

    uint8_t mask; // 2-8 bits to show data presence
    uint8_t data[SIGFOX_MAX_BYTES]; //variable size data

    uint8_t numData; //Number of data bundles in a packet
    uint8_t dataSize; //Data size in bits
} Packet;

typedef struct SigfoxDataType {
    uint8_t array[SIGFOX_MAX_BYTES];
    int8_t  size;
} SigfoxData;


/*
 * Create packet of type 0 and place it in segfoxArray
 * Return number of bytes to send or -1 in case of invalid parameters
 *
 * Type 0 packet(MSB 1st):
 * |Iter|Type|Data0|Data1|....|DataN|
 */
int8_t createPacket0 (Packet* packet, SigfoxData* dataOut);

/*
 * Create packet of type 1 and place it in segfoxArray
 * Return number of bytes to send or -1 in case of invalid parameters
 *
 * Type 1 packet(MSB 1st):
 * |Iter|Type|Mask|Datak0|Datak1|....|DatakN|
 * Data bundle is skipped if corresponding bit in mask is set to 0
 */
int8_t createPacket1 (Packet* packet, SigfoxData* dataOut);

#endif /* PACKET_H_ */
