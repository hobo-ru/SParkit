/***************************************************************************//**
 * @file
 * @brief Public header for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.fr</b>
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/

#ifndef __TD1202_H
#define __TD1202_H

#include "td_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup COMPILATION_OTPIONS Compilation Options
 * @brief Public compilation options for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** libtd1202 v1.0.1 version number */
#define LIBTD1202_V1_0_1			((1 * 0x10000) + (0 * 0x100) + 1)

/** libtd1202 current version number */
#define LIBTD1202_VERSION			LIBTD1202_V1_0_1

/** libtd1202 current version number string */
#define LIBTD1202_VERSION_STRING	"v1.0.1"

/** Flag to include printf() support */
#define USE_PRINTF

/** Flag to include UART support */
#define USE_UART

/** @} (end addtogroup COMPILATION_OTPIONS) */

#ifdef __cplusplus
}
#endif

#endif // __TD1202_H

