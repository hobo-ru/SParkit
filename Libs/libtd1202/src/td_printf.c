/***************************************************************************//**
 * @file
 * @brief printf utility  for the TD1202 module.
 * @author Kustaa Nyholm
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2004 Kustaa Nyholm</b>
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
 *******************************************************************************
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  ******************************************************************************/

#include "td1202.h"

#include <em_usart.h>

#include "td_printf.h"

/***************************************************************************//**
 * @addtogroup PRINTF
 * @brief printf utility  for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

 /** @addtogroup PRINTF_DEFINES Defines
  * @{ */

/** Definition for long support in printf */
#define PRINTF_LONG_SUPPORT

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup PRINTF_TYPEDEFS Typedefs
 * @{ */

/** Pointer to function for flushing characters out */
typedef void    (*putcf)(void *, char);

/** Pointer to function for starting to use the output stream */
typedef void    (*start)(void *);

/** Pointer to function for stopping to use the output stream */
typedef void    (*stop)(void *);

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_PRIVATE_VARIABLES Private Variables
 * @{ */

/** Pointer to opaque context for outputting characters */
static void *   stdout_putp;

/** Pointer to function for flushing characters out */
static putcf    stdout_putf;

/** Pointer to function for starting to use the output stream */
static start    stdout_start;

/** Pointer to function for stopping to use the output stream */
static stop     stdout_stop;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_PRIVATE_FUNCTIONS Private Functions
 * @{ */

#ifdef PRINTF_LONG_SUPPORT

/***************************************************************************//**
 * @brief
 *   Convert an unsigned long integer to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[in] base
 *   Arithmetic base to convert to.
 *
 * @param[in] uc
 *   Convert digits to uppercase if base > 10 and uc is different from 0.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
static void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
    int             n = 0;
    unsigned int    d = 1;

    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        int    dgt = num / d;
        num %= d;
        d   /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

/***************************************************************************//**
 * @brief
 *   Convert a long integer to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
static void li2a(long num, char *bf)
{
    if (num < 0) {
        num   = -num;
        *bf++ = '-';
    }
    uli2a(num, 10, 0, bf);
}

#endif

/***************************************************************************//**
 * @brief
 *   Convert an unsigned integer to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[in] base
 *   Arithmetic base to convert to.
 *
 * @param[in] uc
 *   Convert digits to uppercase if base > 10 and uc is different from 0.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
static void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
    int             n = 0;
    unsigned int    d = 1;

    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        int    dgt = num / d;
        num %= d;
        d   /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

/***************************************************************************//**
 * @brief
 *   Convert an integer to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
static void i2a(int num, char *bf)
{
    if (num < 0) {
        num   = -num;
        *bf++ = '-';
    }
    ui2a(num, 10, 0, bf);
}

/***************************************************************************//**
 * @brief
 *   Convert an ASCII character to decimal.
 *
 * @param[in] ch
 *   Character to convert.
 *
 * @return
 *   Arithmetic base to convert to.
 ******************************************************************************/
static int a2d(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return(ch - '0');
    } else if (ch >= 'a' && ch <= 'f') {
        return(ch - 'a' + 10);
    } else if (ch >= 'A' && ch <= 'F') {
        return(ch - 'A' + 10);
    } else {
        return(-1);
    }
}

/***************************************************************************//**
 * @brief
 *   Convert an ASCII-coded number to binary.
 *
 * @param[in/out] ch
 *   Overhead character.
 *
 * @param[in/out] src
 *   Handle to the string to convert from.
 *
 * @param[in] base
 *   Arithmetic base to convert from.
 *
 * @param[out] nump
 *   Pointer to the output integer number buffer.
 ******************************************************************************/
static char a2i(char ch, char **src, int base, int *nump)
{
    char *  p   = *src;
    int     num = 0;
    int     digit;

    while ((digit = a2d(ch)) >= 0) {
        if (digit > base) {
            break;
        }
        num = num * base + digit;
        ch  = *p++;
    }
    *src  = p;
    *nump = num;
    return(ch);
}

/***************************************************************************//**
 * @brief
 *   Output an argument with left padding.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] n
 *   Number of left-padding characters.
 *
 * @param[in] z
 *   Padding with zero characters if not null, with blank caracters otherwise.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
 ******************************************************************************/
static void putchw(void *putp, putcf putf, int n, char z, char *bf)
{
    char    fc = z ? '0' : ' ';
    char    ch;
    char *  p = bf;

    while (*p++ && n > 0) {
        n--;
    }
    while (n-- > 0) {
        putf(putp, fc);
    }
    while ((ch = *bf++)) {
        putf(putp, ch);
    }
}

/***************************************************************************//**
 * @brief
 *   Function to output a character to a string.
 *
 * @param[out] p
 *   Pointer to the output buffer.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] c
 *   Input character.
******************************************************************************/
static void putcp(void *p, char c)
{
    *(*((char **)p))++ = c;
}

/***************************************************************************//**
 * @brief
 *   Function to output a string.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
******************************************************************************/
static void putcs(void *putp, putcf putf, char *bf)
{
    char ch;
    while ((ch = *bf++)) {
        putf(putp, ch);
    }
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_format(void *putp, putcf putf, char *fmt, va_list va)
{
    char    bf[12];

    char    ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch);
        } else {
            char    lz = 0;
#ifdef  PRINTF_LONG_SUPPORT
            char    lng = 0;
#endif
            int     w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#ifdef  PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch  = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
                case 0:
                    goto abort;
                case 'u': {
#ifdef  PRINTF_LONG_SUPPORT
                    if (lng) {
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    } else
#endif
                    ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    putchw(putp, putf, w, lz, bf);
                    break;
                }
                case 'd': {
#ifdef  PRINTF_LONG_SUPPORT
                    if (lng) {
                        li2a(va_arg(va, unsigned long int), bf);
                    } else
#endif
                    i2a(va_arg(va, int), bf);
                    putchw(putp, putf, w, lz, bf);
                    break;
                }
                case 'x': case 'X':
#ifdef  PRINTF_LONG_SUPPORT
                    if (lng) {
                        uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                    } else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                    putchw(putp, putf, w, lz, bf);
                    break;
                case 'c':
                    putf(putp, (char)(va_arg(va, int)));
                    break;
                case 's':
                    putchw(putp, putf, w, 0, va_arg(va, char *));
                    break;
                case '%':
                    putf(putp, ch);
                    break;
                default:
                    break;
            }
        }
    }
abort:;
}

/***************************************************************************//**
 * @brief
 *   Printf initialization function.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] start
 *   Pointer to function for starting to use the output stream.
 *
 * @param[in] stop
 *   Pointer to function for stopping to use the output stream.
 ******************************************************************************/
void init_printf(void *putp, void (*putf)(void *, char), void (*start)(void *), void (*stop)(void *))
{
    stdout_putp     = putp;
    stdout_putf     = putf;
    stdout_start    = start;
    stdout_stop     = stop;
}

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Number and types of arguments according to the format string.
 ******************************************************************************/
void tfp_printf(char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    if (stdout_start) {
    	stdout_start(stdout_putp);
    }
    tfp_format(stdout_putp, stdout_putf, fmt, va);
    if (stdout_stop) {
    	stdout_stop(stdout_putp);
    }
    va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Output a string wit hvaraible argument list according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_vprintf(char *fmt, va_list va)
{
    if (stdout_start) {
    	stdout_start(stdout_putp);
    }
    tfp_format(stdout_putp, stdout_putf, fmt, va);
    if (stdout_stop) {
    	stdout_stop(stdout_putp);
    }
}

/***************************************************************************//**
 * @brief
 *   Output a string into another string according to a format.
 *
 * @param[out] s
 *   Pointer to the output string buffer.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_sprintf(char *s, char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    tfp_format(&s, putcp, fmt, va);
    putcp(&s, 0);
    va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Dump a buffer in hexadecimal.
 *
 * @param[in] text
 *   Pointer to a prompt string to ouptut in front of the hexadecimal dump.
 *
 * @param[in] s
 *   Pointer to the input buffer.
 *
 * @param[in] sz
 *   Size of the input buffer in characters.
 ******************************************************************************/
void tfp_dump(char *text, unsigned char *s, unsigned char sz)
{
    static char dump[(3 * 16) + 3];
    unsigned char i, j, k;
    char hex[4];

    if (stdout_start) {
        stdout_start(stdout_putp);
    }

    putcs(stdout_putp, stdout_putf, text);

    for (i = 0, k = 0; i < sz; i++) {
        tfp_sprintf(hex, "%02x ", s[i]);
        for (j = 0; hex[j]; j++) {
            dump[k++] = hex[j];
        }
        if (i && ((i % 16) == 0)) {
            dump[k++] = '\r';
            dump[k++] = '\n';
            dump[k]   = '\0';
            putcs(stdout_putp, stdout_putf, dump);
            k = 0;
        }
    }

    if (k > 0) {
        dump[k++] = '\r';
        dump[k++] = '\n';
        dump[k]   = '\0';
        putcs(stdout_putp, stdout_putf, dump);
    }

    if (stdout_stop) {
        stdout_stop(stdout_putp);
    }
}

/** @} */

/** @} (end addtogroup PRINTF) */
