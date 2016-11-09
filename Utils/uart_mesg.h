/*
 * uart_mesg.h
 *
 *  Created on: 14 рту. 2014 у.
 *      Author: vdubikhin
 */

#ifndef UART_MESG_H_
#define UART_MESG_H_

#include <stdint.h>

int getUartInt(uint8_t base);
void SendUartMessage(const char* mesg, unsigned char lr);
void itoa(int n, char s[], int base);
int getCharUart();
void sendCharUart(unsigned char symbol);

#endif /* UART_MESG_H_ */
