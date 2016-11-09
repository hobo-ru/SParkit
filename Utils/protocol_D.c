/*
* Protocol_D.c
*
*  Created on: 22.09.2014
*      Author: S.Omelchenko
*
*/

#include <efm32tg210f32.h>
#include <td1202.h>
#include <td_rtc.h>

#include <limits.h>
#include <string.h>
#include <stddef.h>

#include "protocol_D.h"
#include "UNB.h"

#define INCREMENT_PACKET_ITER(x)		((++x) >= (1 << PACKET_ITER_SIZE_bits))?((x) = 0):(x)
#define INCREMENT_FRAME_ITER(x) 		((++x) >= (1 <<  FRAME_ITER_SIZE_bits))?((x) = 0):(x)

#define PREPARE_FIRST_COPY()                                      \
do {                                                          \
	if (src_len >= (CHAR_BIT - dst_offset_modulo)) {              \
		*dst     &= reverse_mask[dst_offset_modulo];              \
			src_len -= CHAR_BIT - dst_offset_modulo;                  \
	} else {                                                      \
		*dst     &= reverse_mask[dst_offset_modulo]               \
			| reverse_mask_xor[dst_offset_modulo + src_len];    \
				c       &= reverse_mask[dst_offset_modulo + src_len];    \
					src_len = 0;                                              \
	} } while (0)
		

// Copies the number of bits from src array to dst array with offset. Found on stackoverflow
static void bitarray_copy(const unsigned char *src_org, int src_offset, int src_len,
						  unsigned char *dst_org, int dst_offset)
{
	static const unsigned char reverse_mask[] =
	{ 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };
	static const unsigned char reverse_mask_xor[] =
	{ 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00 };
	
	if (src_len) {
		const unsigned char *src;
		unsigned char *dst;
		int                  src_offset_modulo,
		dst_offset_modulo;
		
		src = src_org + (src_offset / CHAR_BIT);
		dst = dst_org + (dst_offset / CHAR_BIT);
		
		src_offset_modulo = src_offset % CHAR_BIT;
		dst_offset_modulo = dst_offset % CHAR_BIT;
		
		if (src_offset_modulo == dst_offset_modulo) {
			int              byte_len;
			int              src_len_modulo;
			if (src_offset_modulo) {
				unsigned char   c;
				
				c = reverse_mask_xor[dst_offset_modulo]     & *src++;
				
				PREPARE_FIRST_COPY();
				*dst++ |= c;
			}
			
			byte_len = src_len / CHAR_BIT;
			src_len_modulo = src_len % CHAR_BIT;
			
			if (byte_len) {
				memcpy(dst, src, byte_len);
				src += byte_len;
				dst += byte_len;
			}
			if (src_len_modulo) {
				*dst     &= reverse_mask_xor[src_len_modulo];
				*dst |= reverse_mask[src_len_modulo]     & *src;
			}
		} else {
			int             bit_diff_ls,
			bit_diff_rs;
			int             byte_len;
			int             src_len_modulo;
			unsigned char   c;
			/*
			* Begin: Line things up on destination. 
			*/
			if (src_offset_modulo > dst_offset_modulo) {
				bit_diff_ls = src_offset_modulo - dst_offset_modulo;
				bit_diff_rs = CHAR_BIT - bit_diff_ls;
				
				c = *src++ << bit_diff_ls;
				c |= *src >> bit_diff_rs;
				c     &= reverse_mask_xor[dst_offset_modulo];
			} else {
				bit_diff_rs = dst_offset_modulo - src_offset_modulo;
				bit_diff_ls = CHAR_BIT - bit_diff_rs;
				
				c = *src >> bit_diff_rs     &
					reverse_mask_xor[dst_offset_modulo];
			}
			PREPARE_FIRST_COPY();
			*dst++ |= c;
			
			/*
			* Middle: copy with only shifting the source. 
			*/
			byte_len = src_len / CHAR_BIT;
			
			while (--byte_len >= 0) {
				c = *src++ << bit_diff_ls;
				c |= *src >> bit_diff_rs;
				*dst++ = c;
			}
			
			/*
			* End: copy the remaing bits; 
			*/
			src_len_modulo = src_len % CHAR_BIT;
			if (src_len_modulo) {
				c = *src++ << bit_diff_ls;
				c |= *src >> bit_diff_rs;
				c     &= reverse_mask[src_len_modulo];
				
				*dst     &= reverse_mask_xor[src_len_modulo];
				*dst |= c;
			}
		}
	}
}
/*----------------------------------------------------------------------------------------------------
// Protocol D Send:
// Split the packet in short frames and send over UNB with an interval between frames
// Parameters are the same as in UNBsend(), except
// interframe_time - time in 32768 ticks between frames transmit
//----------------------------------------------------------------------------------------------------*/

#define PORT_LED TIM2_PORT
#define PIN_LED TIM2_BIT
#define PORT_BUTTON USR1_PORT
#define PIN_BUTTON USR1_BIT
#define MASK_BUTTON USR1_MASK
#include <efm32.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_timer.h>
#include <em_gpio.h>
#include <em_chip.h>
#include <em_wdog.h>
#include <em_adc.h>
#include <td_module.h>
#include <td1202.h>
#include <td_rtc.h>
#include <td_cmu.h>
#include <td_gpio.h>

uint8_t protocol_D_Send(uint8_t* message, uint32_t length, uint32_t serial, uint32_t interframe_time)
{
	uint8_t ret = 1;
	uint8_t frame_iter = 0;
	uint8_t stop, ext_mode = 0;// TODO ext mode
	uint8_t num_of_frames = 0;
	static uint8_t packet_iter = 0;
	uint32_t length_bits = length * 8;
	uint32_t last_frame_size = 0;
	uint32_t sent_bits = 0; // number of bits sent
	frame_t frame;
	
	if(length > PACKET_SIZE) return 0; // max size check
	
	if(length % FRAME_PAYLOAD)
	{
		last_frame_size = (length % FRAME_PAYLOAD) + ((FRAME_HEADER_SIZE_bits+7)/8) ;
		num_of_frames = (length_bits / FRAME_PAYLOAD_bits) + ((last_frame_size) ? (1) : (0));
	}
	else
	{
		num_of_frames = (length_bits / FRAME_PAYLOAD_bits);
		last_frame_size = FRAME_SIZE;
	}
	//num_of_frames = (length_bits / FRAME_PAYLOAD_bits) + ((last_frame_size) ? (1) : (0));
	//if(last_frame_size == 0) last_frame_size = FRAME_SIZE;
	
	while(sent_bits < length_bits)
	{
		GPIO_PinOutSet(PORT_LED, PIN_LED);//debug!!!
		// Compose frame
		stop = ((length_bits - sent_bits) > FRAME_PAYLOAD_bits) ? 0 : 1;	// check if the frame is last one
		frame.length = (stop) ? (last_frame_size) : (FRAME_SIZE);
		memset(frame.payload, 0x00, FRAME_SIZE);
		frame.payload[0] = (packet_iter << 5) | ((frame_iter & ((1<<FRAME_ITER_SIZE_bits)-1)) << 3) | (((num_of_frames-1) & ((1<<FRAME_ITER_SIZE_bits)-1)) << 1) | (ext_mode & 1);
		bitarray_copy(message, sent_bits, (stop) ? ((length_bits - sent_bits)) : (FRAME_PAYLOAD_bits), frame.payload, FRAME_HEADER_SIZE_bits);
		// Send frame
		ret &= UNBsend((uint8_t*)frame.payload, frame.length, serial);	// if a single transmit fails, ret sets to zero
		// Increment counters
		INCREMENT_FRAME_ITER(frame_iter);
		
		sent_bits += FRAME_PAYLOAD_bits;
		// Go to sleep
		TD_RTC_Delay(interframe_time);
		
		
		GPIO_PinModeSet(PORT_BUTTON, PIN_BUTTON, gpioModeInput, 1);//debug!!!
		GPIO_PinOutClear(PORT_LED, PIN_LED);//debug!!!
		while(GPIO_PinInGet(PORT_BUTTON, PIN_BUTTON))//debug!!!
		{
			TD_RTC_Delay(100);
			if(stop) break;
		}
	}
	INCREMENT_PACKET_ITER(packet_iter);
	return ret;
}