/*
 * uart_mesg.c
 *
 *  Created on: 14 авг. 2014 г.
 *      Author: vdubikhin
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <td1202.h>
#include <efm32.h>
#include <td_uart.h>

#include "uart_mesg.h"

int getUartInt(uint8_t base) {
    char inputStr[33];
    int uartChar, curChar = 0;

    uartChar = TD_UART_GetChar();

    while (uartChar != '\n') {
        if (uartChar != -1) {
            TD_UART_Putc(LEUART0, uartChar);
            inputStr[curChar++] = uartChar;
        }

        uartChar = TD_UART_GetChar();
    }

    inputStr[curChar] = '\0';

    return strtol(inputStr, 0, base);
}

void SendUartMessage(const char* mesg, unsigned char lr) {
    if (mesg) {
        uint32_t i = 0;
        while (mesg[i]) {
            TD_UART_Putc(LEUART0,mesg[i]);
            i++;
        }
        if (lr)
            TD_UART_Putc(LEUART0,'\n');
        else
            TD_UART_Putc(LEUART0,' ');
    }
}



/* reverse:  переворачиваем строку s на месте */
static void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}
/* itoa:  конвертируем n в символы в s */
void itoa(int n, char s[], int base)
{
    int i, sign;

    if ((sign = n) < 0)  /* записываем знак */
        n = -n;          /* делаем n положительным числом */
    i = 0;
    do {       /* генерируем цифры в обратном порядке */
        if (n % base <= 9)
            s[i++] = n % base + '0';   /* берем следующую цифру */
        else
            s[i++] = n % base - 10 + 'A';
    } while ((n /= base) > 0);     /* удаляем */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}

int getCharUart() {
    return TD_UART_GetChar();
}

void sendCharUart(unsigned char symbol) {
    TD_UART_Putc(LEUART0, symbol);
}
