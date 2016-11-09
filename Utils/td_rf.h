/** @cond TD_PRIVATE */
/***************************************************************************//**
 * @file
 * @brief Si4461 RF peripheral chip parameterized API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2013 Telecom Design S.A., http://www.telecom-design.fr</b>
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/

#ifndef __TD_RF_H
#define __TD_RF_H

#include <stdint.h>
#include <stdbool.h>
#include "td_rf_Si4461.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup RF Low/Medium/High-Level RF
 * @brief Si4461 RF peripheral chip parameterized API for the TD1202 module
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MID_LEVEL_RF
 * @brief Mid-level RF API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup RF_ENUMERATIONS Enumerations
 * @{ */

/** Test mode */
typedef enum {
	TD_RF_LOW = 0,			/**< Continuous low frequency */
	TD_RF_HIGH,				/**< Continuous high frequency */
	TD_RF_DIRECT,			/**< Direct input pin drive */
	TD_RF_MFG,				/**< Manufacturing radio protocol */
	TD_RF_LOCAL,			/**< Local radio protocol */
	TD_RF_PN9 = 9,			/**< PN9 pseudo-random generator sequence */
	TD_RF_SQUARE = 50		/**< 50% duty cycle square wave */
} TD_RF_test_mode_t;

/** Test modulation mode */
typedef enum {
	TD_RF_CW = 0,			/**< Continuous wave */
	TD_RF_FSK,				/**< Frequency Shift Keying Modulation */
	TD_RF_GFSK,				/**< Gaussian-filtered Frequency Shift Keying Modulation */
	TD_RF_OOK,				/**< On/Off Keying Modulation */
	TD_RF_PSK				/**< Phase Shift Keying Modulation */
} TD_RF_modulation_t;

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup RF_TYPEDEFS Typedefs
 * @{ */

/** RF parameter type */
typedef struct {
	uint32_t frequency;		/**< Frequency in Hz */
	uint16_t data_rate;		/**< Data bit rate in bps */
	uint16_t deviation;		/**< Deviation in Hz */
	uint8_t modulation;		/**< Modulation mode */
	int8_t tx_level;		/**< Transmit level in dBm */
	uint8_t tx_data_message; /**< Obsolete */
} TD_RF_param_t;

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup RF_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup RF_PROTOTYPES Prototypes
 * @{ */

bool TD_RF_Parameters(TD_RF_param_t *params);

/** @} */
/** @} */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RF_PUBLIC_VARIABLES Public Variables
 * @{ */
/** @addtogroup RF_EXTERN Extern Declarations
 * @{ */

/** Default RF parameters */
extern TD_RF_param_t TD_RF_DefaultRFParams;

/** Default RF configuration */
extern uint8_t TD_RF_DefaultConfig[];

/** @} */
/** @} */

/** @} (end addtogroup MID_LEVEL_RF) */
/** @} (end addtogroup RF) */

#ifdef __cplusplus
}
#endif

#endif // __TD_RF_H
/** @endcond */
