/***************************************************************************//**
 * @file
 * @brief Utility functions for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "td1202.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>

/***************************************************************************//**
 * @addtogroup UTILS
 * @brief Utility functions for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UTILS_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert a string to long long integer.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @return
 *  The converted long long integer value.
 ******************************************************************************/
long long atoll(char *instr)
{
    long long retval = 0;

    for (; *instr; instr++) {
        retval = (10 * retval) + (*instr - '0');
    }

    return retval;
}

/***************************************************************************//**
 * @brief
 *   Convert a string to integer.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @return
 *  The converted integer value.
 ******************************************************************************/
int atoi(char *instr)
{
    int retval = 0;

    for (; *instr; instr++) {
        retval = (10 * retval) + (*instr - '0');
    }

    return retval;
}

#ifndef __ICCARM__
/***************************************************************************//**
 * @brief
 *   Copy a byte into a memory region.
 *
 * @param[out] s
 *  Pointer to the output memory region.
 *
 * @param[in] c
 *  The byte value to copy.
 *
 * @param[in] n
 *  The memory region size in bytes.
 *
 * @return
 *  Returns a pointer to the output buffer.
 ******************************************************************************/
void *memset(void *s, int c, size_t n)
{
    char *s8 = (char *)s;
    int i;

    for (i = 0; i < n; i++) {
        s8[i] = c;
    }

    return s;
}

/***************************************************************************//**
 * @brief
 *   Copy a region of memory into a another.
 *
 * @param[out] d
 *  Pointer to the destination memory region.
 *
 * @param[in] s
 *  Pointer to the source memory region.
 *
 * @param[in] n
 *  The source memory region size in bytes.
 *
 * @return
 *  Returns a pointer to the output buffer.
 ******************************************************************************/
void *memcpy(void *d, const void *s, size_t n)
{
    const char *s8 = (const char *)s;
    char *d8 = (char *)d;
    int i;

    for (i = 0; i < n; i++) {
        d8[i] = s8[i];
    }

    return d;
}

/***************************************************************************//**
 * @brief
 *   Compare two memory regions.
 *
 * @param[in] s1
 *  Pointer to the first memory region.
 *
 * @param[in] s2
 *  Pointer to the second memory region.
 *
 * @param[in] n
 *  The source memory region size in bytes.
 *
 * @return
 *  Returns 0 if memory regions are equal, 1 if first memory region is greater
 *  and -1 if first memoryregion is lighter than the second one.
 ******************************************************************************/
int memcmp(const void *s1, const void *s2, int n)
{
    const uint8_t *sc1 = (const uint8_t *)s1;
    const uint8_t *sc2 = (const uint8_t *)s2;
    int i;

    for (i = 0; i < n; i++) {
        if (sc1[i] > sc2[i]) {
            return 1;
        }
        if (sc1[i] < sc2[i]) {
            return -1;
        }
    }

    return 0;
}

#endif

/** @} */

/** @} (end addtogroup UTILS) */
