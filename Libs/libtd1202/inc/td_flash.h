/***************************************************************************//**
 * @file
 * @brief Flash controller (MSC) peripheral API for the TD1202 module.
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

#ifndef __TD_FLASH_H
#define __TD_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup FLASH
 * @brief Flash controller (MSC) Peripheral API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup FLASH_DEFINES Defines
 * @{ */

/** Size of one flash page */
#define FLASH_PAGE_SIZE            	0x200

/** Number of words in a flash page */
#define FLASH_E2P_SIZE_WORD        	(FLASH_PAGE_SIZE / sizeof(uint32_t))

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup FLASH_TYPEDEFS Typedefs
 * @{ */

/** TD1202 device descriptor */
typedef struct _TD_DEVICE {
    uint32_t    Serial;            	/**< Serial number (SIGFOX ID). */
    uint32_t    Key1;				/**< Reserved. */
    uint64_t    Key2;				/**< Reserved. */
    uint8_t     ModResult;			/**< Module manufacturing test result. */
    uint8_t     ProdResult;			/**< Product manufacturing test result. */
    uint8_t     LedMask1;			/**< Obsolete. */
    uint8_t     LedMask2;			/**< Obsolete. */
} TD_DEVICE;

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup FLASH_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup FLASH_PROTOTYPES Prototypes
 * @{ */

void TD_FLASH_DeviceWrite(TD_DEVICE *device);
void TD_FLASH_Write(void *buffer, uint32_t count);
bool TD_FLASH_Read(void *buffer, uint32_t count);
bool TD_FLASH_DeviceRead(TD_DEVICE *device);
bool TD_FLASH_ReadRegion(uint32_t start, void *buffer, uint32_t count);
void TD_FLASH_WriteRegion(uint32_t start, void *buffer, uint32_t count);
void TD_FLASH_WriteRegion_noErase(uint32_t start, void *buffer, uint32_t count);
void TD_FLASH_ErasePage(uint32_t blockStart);
 

/** @} */
/** @} */

/** @} (end addtogroup FLASH) */

#ifdef __cplusplus
}
#endif

#endif // __TD_FLASH_H
