/*
 * uart.h
 *
 *  Created on: 23 апр. 2014 г.
 *      Author: vdubikhin
 */

#ifndef UART_H_
#define UART_H_

void uartStart(void);
void uartStop(void);
void    uartPutChar(uint8_t charPtr);
uint8_t uartGetChar(void);
void resetBuffers();

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

typedef struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} circularBuffer;

extern const circularBuffer *const rxBuffer;
extern const circularBuffer *const txBuffer;

#endif /* UART_H_ */
