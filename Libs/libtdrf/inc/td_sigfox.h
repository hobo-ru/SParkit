/***************************************************************************//**
 * @file
 * @brief SIGFOX API for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.fr</b>
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

#ifndef __TD_SIGFOX_H
#define __TD_SIGFOX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup SIGFOX SIGFOX
 * @brief SIGFOX API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SIGFOX_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SIGFOX_PROTOTYPES Prototypes
 * @{ */

bool TD_SIGFOX_Init(bool init);
bool TD_SIGFOX_Send(uint8_t *, uint8_t, uint8_t);
bool TD_SIGFOX_SendTest(uint16_t count,uint8_t time,uint16_t Slot,uint8_t retry);
bool TD_SIGFOX_SendOOB(uint8_t *, uint8_t, uint8_t);
bool TD_SIGFOX_RfPower(uint8_t);
bool TD_SIGFOX_PinConf(uint8_t config);
bool TD_SIGFOX_SetKeepAliveRetries(uint8_t count);
uint8_t TD_SIGFOX_PowerVoltage(void);
bool TD_SIGFOX_FrequencyConf(uint32_t,uint16_t, uint16_t, uint16_t);
void TD_SIGFOX_FrequencySet(uint16_t Slot);
void TD_SIGFOX_KeepAlive(void);
void TD_SIGFOX_SendTestPA(uint16_t Slot,uint8_t mode,uint32_t xx);
uint8_t TD_SIGFOX_MainRfPower(uint8_t power);
char const *TD_SIGFOX_VersionLib(void);


/** @} */
/** @} */

/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_H
