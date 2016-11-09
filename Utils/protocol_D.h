/*
* Protocol_D.c
*
*  Created on: 22.09.2014
*      Author: S.Omelchenko
*
*/

#ifndef PROTOCOL_D_H_
#define PROTOCOL_D_H_

#define FRAME_SIZE 20	//20
#define FRAME_SIZE_bits (FRAME_SIZE*8)
#define FRAME_HEADER_SIZE 1
#define FRAME_HEADER_SIZE_bits (FRAME_HEADER_SIZE*8)
#define FRAME_PAYLOAD (FRAME_SIZE - FRAME_HEADER_SIZE)
#define FRAME_PAYLOAD_bits (FRAME_SIZE_bits - FRAME_HEADER_SIZE_bits)

#define PACKET_ITER_SIZE_bits 3
#define FRAME_ITER_SIZE_bits 2

#define PACKET_SIZE (FRAME_PAYLOAD * (1<<FRAME_ITER_SIZE_bits))

typedef struct
{
	uint8_t length;
	uint8_t payload[FRAME_SIZE]; //not precise length, but who cares
}frame_t;

uint8_t protocol_D_Send(uint8_t *message, uint32_t length, uint32_t serial, uint32_t interframe_time);

#endif //PROTOCOL_D_H_