/*
 * Frame.c
 *
 *  Created on: 30 θώνÿ 2014 γ.
 *      Author: vdubikhin
 */

#include "Frame.h"

/**
 * Return CRC-8 of the data, using x^8 + x^2 + x + 1 polynomial.  A table-based
 * algorithm would be faster, but for only a few bytes it isn't worth the code
 * size. */
static uint8_t Crc8(const void *vptr, int len)
{
    const uint8_t *data = (uint8_t *)vptr;
    unsigned crc = 0;
    int i, j;
    for (j = len; j; j--, data++) {
        crc ^= (*data << 8);
        for(i = 8; i; i--) {
            if (crc & 0x8000)
                crc ^= (0x1070 << 3);
            crc <<= 1;
        }
    }
    return (uint8_t)(crc >> 8);
}


TD_LAN_frame_t CreateFrameUtility(void* frameAddr) 
{
    TD_LAN_frame_t tdFrame;
    FrameUtility* frame = (FrameUtility*) frameAddr;

    tdFrame.header = frame->mask >> 8; //high mask portion
    tdFrame.payload[0] = frame->mask; // low mask portion
    tdFrame.payload[1] = ( frame->address & 0xFF000000 ) >> 24;
    tdFrame.payload[2] = ( frame->address & 0x00FF0000 ) >> 16;
    tdFrame.payload[3] = ( frame->address & 0x0000FF00 ) >> 8;
    tdFrame.payload[4] = ( frame->address & 0x000000FF );
    tdFrame.payload[5] = frame->function;

    frame->crc8 = 0;

    for (int i = (PAYLOAD_SIZE - UTILITY_PAYLOAD); i < PAYLOAD_SIZE; i++)
        tdFrame.payload[i] = frame->data[i - (PAYLOAD_SIZE - UTILITY_PAYLOAD)];

    tdFrame.payload[6] = Crc8(frame, sizeof(FrameUtility));

    return tdFrame;
}

FrameUtility ParseFrameUtility (TD_LAN_frame_t* tdFrame) 
{
    FrameUtility frame;
    frame.mask = ((tdFrame->header & 0x00FFFFFF) << 8) + tdFrame->payload[0]; // first byte is TD garbage
    frame.address = (tdFrame->payload[1] << 24) + (tdFrame->payload[2] << 16) +
            (tdFrame->payload[3] << 8) + (tdFrame->payload[4]);
    frame.function = tdFrame->payload[5];

    for (int i = (PAYLOAD_SIZE - UTILITY_PAYLOAD); i <PAYLOAD_SIZE; i++)
        frame.data[i - (PAYLOAD_SIZE - UTILITY_PAYLOAD)] = tdFrame->payload[i];

    frame.crc8 = 0;
    uint8_t crc = Crc8(&frame, sizeof(FrameUtility));

    if (crc == tdFrame->payload[6])
        frame.crc8 = 1;

    return frame;
}

TD_LAN_frame_t CreateFrameData(void* frameAddr) {
    TD_LAN_frame_t tdFrame;
    FrameData* frame = (FrameData*) frameAddr;

    frame->crc8 = 0;
    frame->crc8 = Crc8(frame, sizeof(FrameData));

    tdFrame.header = (frame->dataSize << 16) +
            (frame->iter << 8) + frame->crc8;

    for (int i = (PAYLOAD_SIZE - DATA_PAYLOAD); i <PAYLOAD_SIZE; i++)
        tdFrame.payload[i] = frame->data[i - (PAYLOAD_SIZE - DATA_PAYLOAD)];

    return tdFrame;
}

FrameData ParseFrameData (TD_LAN_frame_t* tdFrame) {
    FrameData frame;
    uint32_t tdHeader = tdFrame->header & 0x00FFFFFF; // first byte is TD garbage

    frame.dataSize = tdFrame->header >> 16;
    frame.iter = tdFrame->header >> 8;

    for (int i = (PAYLOAD_SIZE - DATA_PAYLOAD); i <PAYLOAD_SIZE; i++)
        frame.data[i - (PAYLOAD_SIZE - DATA_PAYLOAD)] = tdFrame->payload[i];

    frame.crc8 = 0;
    uint8_t crc = Crc8(&frame, sizeof(FrameData));

    uint8_t crcTD = tdHeader & 0xFF;

    if (crc == crcTD)
        frame.crc8 = 1;

    return frame;
}


