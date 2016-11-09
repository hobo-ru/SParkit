/*
 * Frame.h
 *
 *  Created on: 30 θώνÿ 2014 γ.
 *      Author: vdubikhin
 */

#ifndef FRAME_H_
#define FRAME_H_

#define PAYLOAD_SIZE 17

#include <stdlib.h>
#include <stdint.h>

#include <td_lan.h>

#define UTILITY_PAYLOAD (PAYLOAD_SIZE - 7)
typedef __packed struct FrameUtility {
    uint32_t address;
    uint32_t mask;
    uint8_t function;
    uint8_t crc8;
    uint8_t data[UTILITY_PAYLOAD];
} FrameUtility;

#define DATA_PAYLOAD (PAYLOAD_SIZE - 0)
typedef __packed struct FrameData {
    uint8_t dataSize;
    uint8_t iter;
    uint8_t crc8;
    uint8_t data[DATA_PAYLOAD];
} FrameData;

TD_LAN_frame_t CreateFrameUtility(void* frame);

FrameUtility ParseFrameUtility (TD_LAN_frame_t* tdFrame);

TD_LAN_frame_t CreateFrameData(void* frame);

FrameData ParseFrameData (TD_LAN_frame_t* tdFrame);

#endif /* FRAME_H_ */
