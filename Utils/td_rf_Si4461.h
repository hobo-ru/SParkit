/** @cond TD_PRIVATE */
/***************************************************************************//**
 * @file
 * @brief Si4461 RF peripheral chip API for the TDxxxx RF modules.
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

#ifndef __TD_RF_SI4461_H
#define __TD_RF_SI4461_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup RF Low-Level Si4461 RF
 * @brief Si4461 RF peripheral chip implementation for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup RF_SI4461_DEFINES Defines
 * @{ */

/** Packet status flags */
#define PACKET_RX_PEND		0x10				///< Received packet pending flag
#define PACKET_SENT_PEND	0x20				///< Sent packet pending flag

/** RF chip mode */
#define TD_RF_NO_CHANGE_STATE         (0 << 4)	///< Don't change RF mode
#define TD_RF_SLEEP_STATE             (1 << 4)	///< Sleep mode
#define TD_RF_SPI_ACTIVE_STATE        (3 << 4)	///< Active SPI mode
#define TD_RF_READY_STATE             (4 << 4)	///< RF ready mode
#define TD_RF_TUNE_TX_STATE           (5 << 4)	///< RF tuning for TX mode
#define TD_RF_TUNE_RX_STATE           (6 << 4)	///< RF tuning for RX mode
#define TD_RF_TX_STATE                (7 << 4)	///< RF TX mode
#define TD_RF_RX_STATE                (8 << 4)	///< RF RX mode

/** RF priority */
#define TD_RF_DEFAULT_PRIORITY			0

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup RF_SI4461_TYPEDEFS Typedefs
 * @{ */

/** Radio Information type */
typedef struct {
	uint16_t part;		/**< Part Number (e.g. Si4461 will be 0x4461) */
	uint16_t id;		/**< ID */
	uint8_t chip_rev;	/**< Chip Mask Revision */
	uint8_t pbuild;		/**< Part Build */
	uint8_t customer;	/**< Customer ID */
	uint8_t rom_id;		/**< ROM ID */
} TD_RF_info_t;

/** I/O mode for RF chip GPIO pin */
typedef enum {
	TD_RF_GPIO_DISABLED = 0,
	TD_RF_GPIO_INPUT,
	TD_RF_GPIO_OUTPUT,
	TD_RF_GPIO_CUSTOM,
	TD_RF_GPIO_READ
} TD_RF_gpio_mode_t;

/** User RF priority change callback */
typedef void (*TD_RF_priority_callback_t)(int previous_priority, int new_priority);

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup LOW_LEVEL_RF
 * @brief Low-level RF API for the TD1202 module
 * @{
 ******************************************************************************/

/** @addtogroup LOW_LEVEL_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup LOW_LEVEL_PROTOTYPES Prototypes
 * @{ */

bool TD_RF_WaitforCTS(void);
bool TD_RF_GetFastResponseRegister(uint8_t start_register, uint8_t count, uint8_t *buffer);
bool TD_RF_GetResponse(uint8_t count, uint8_t *buffer);
void TD_RF_ReadFIFO(uint8_t count, uint8_t *buffer);
void TD_RF_WriteFIFO(uint8_t count, uint8_t *buffer);
bool TD_RF_SendCommand(uint8_t *buffer);

/** @} */
/** @} */

/** @} (end addtogroup LOW_LEVEL_RF) */

/***************************************************************************//**
 * @addtogroup MID_LEVEL_RF
 * @brief Mid-level RF API for the TD1202 module
 * @{
 ******************************************************************************/

/** @addtogroup MID_LEVEL_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup MID_LEVEL_PROTOTYPES Prototypes
 * @{ */

bool TD_RF_Init(uint8_t *config, bool Irq);
void TD_RF_Start(bool irq);
void TD_RF_Stop(bool irq);
bool TD_RF_Configure(uint8_t *config, bool Irq);
void TD_RF_ClearInterrupts(void);
uint8_t TD_RF_InfoFifo(uint8_t mode);
void TD_RF_MaskNode(uint8_t node,uint8_t mask, uint8_t control);
void TD_RF_MaskNode32(uint32_t node, uint32_t mask, uint32_t control);
bool TD_RF_ReadVersion(void);
bool TD_RF_Sleep();
void TD_RF_SleepCrystal();
void TD_RF_Wakeup(void);
void TD_RF_TX(void);
bool TD_RF_StartRx(uint8_t channel, uint16_t count);
bool TD_RF_StartTx(uint8_t channel, uint16_t count,uint8_t mode);
int TD_RF_ReadRSSI(void);
int TD_RF_ReadLatchedRSSI(void);
uint8_t TD_RF_ReadStatus(void);
bool TD_RF_GPIO_PinConfigure(uint8_t pin, TD_RF_gpio_mode_t mode, uint8_t *value);
int TD_RF_Lock(int new_priority);
void TD_RF_Unlock(int previous_priority);
void TD_RF_SetPriorityCallback(TD_RF_priority_callback_t callback);
void TD_RF_Ramp(uint8_t mode,uint8_t start,uint8_t end);
void TD_RF_RampTime(uint8_t mode);
uint8_t TD_RF_PeekPoke(uint8_t mode,uint16_t addr, uint8_t * value);
bool TD_RF_Patch(const uint8_t *patch,uint16_t nb);
void TD_RF_RampTable(uint8_t *table, uint8_t off);
void TD_RF_Reset(void);
void TD_RF_SetProperty(uint8_t group,uint8_t prop,uint8_t val);
void TD_RF_EnableTCXO(bool enable);

/** @} */
/** @} */

/** @} (end addtogroup MID_LEVEL_RF) */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RF_SI4461_PUBLIC_VARIABLES Public Variables
 * @{ */
/** @addtogroup RF_SI4461_EXTERN Extern Declarations
 * @{ */

/** Radio Information */
extern TD_RF_info_t TD_RF_Info;

/** Radio write buffer */
extern uint8_t  TD_RF_WriteBuffer[];

/** Radio read buffer */
extern uint8_t  TD_RF_ReadBuffer[];

/** @} (end addtogroup RF) */

/** @} */
/** @} */

#ifdef __cplusplus
}
#endif

#endif // __TD_RF_SI4461_H
/** @endcond */
