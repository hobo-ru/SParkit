/***************************************************************************//**
 * @file
 * @brief Local RF for the TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#ifndef __TD_LAN_H
#define __TD_LAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup TD_LAN TD LAN
 * @brief Local RF for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup LAN_DEFINES Defines
 * @{ */

/** Macro to get a bit field */
#define GET(b, f)				((b >> f ## _SHIFT) & MASK(f ## _WIDTH))

/** Macro to set a bit field */
#define SET(b, f, v)			b &= ~(MASK(f ## _WIDTH) << f ## _SHIFT); \
	b |= ((v & MASK(f ## _WIDTH))  << f ## _SHIFT)

#define ACK_WIDTH				1				/**< Ack flag field width */
#define ACK_SHIFT				31				/**< Ack flag field shift */
#define FRAME_NUMBER_WIDTH		7				/**< Frame number field width */
#define FRAME_NUMBER_SHIFT		24				/**< Frame number field shift */
#define ADDRESS_WIDTH			24				/**< Address field width */
#define ADDRESS_SHIFT			0				/**< Address field shift */

/** Macro to compute bit mask given its width */
#define MASK(w)					((1 << w) - 1)

#define GET_ACK(b)				GET(b, ACK)				/**< Get the ack flag value */
#define SET_ACK(b, v)			SET(b, ACK, v)			/**< Set the ack flag value */
#define GET_FRAME_NUMBER(b)		GET(b, FRAME_NUMBER)	/**< Get the frame number value */
#define SET_FRAME_NUMBER(b, v)	SET(b, FRAME_NUMBER, v)	/**< Set the frame number value */
#define GET_ADDRESS(b)			GET(b, ADDRESS)			/**< Get the address value */
#define SET_ADDRESS(b, v)		SET(b, ADDRESS, v)		/**< Set the address value */

/** TD LAN frame size */
#define TD_LAN_FRAME_SIZE        sizeof (TD_LAN_frame_t)

/** TD LAN payload size */
#define TD_LAN_PAYLOAD_SIZE		 (TD_LAN_FRAME_SIZE - sizeof (uint32_t))

/** Negative acknowledgment */
#define TD_LAN_NACK				0U

/** Positive acknowledgment */
#define TD_LAN_ACK				1U

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup LAN_ENUMERATIONS Enumerations
 * @{ */

/** LAN RF transaction modes */
typedef enum {
	MODE_SLEEP,					/**< Idle mode */
	MODE_RECEIVER,				/**< Periodic windowed receive mode */
	MODE_TRANSCEIVER,			/**< Transmit mode, then switch to immediate receive mode */
	MODE_ACK,					/**< Immediate receive mode */
	MODE_TRANSMITTER			/**< Transmit mode */
} TD_LAN_mode_t;

/** LAN result states */
typedef enum {
    RESULT_PENDING,				/**< Pending radio transaction */
    RESULT_COMPLETE,			/**< Radio transaction completed */
    RESULT_ABORT				/**< No or aborted radio transaction */
} TD_LAN_result_t;

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup LAN_TYPEDEFS Typedefs
 * @{ */

/** LAN RF frame structure
 *
 * @details
 *   The TD_LAN frame consists in a a header and a user payload field:
 *
 *   | Header  | Payload  |
 *   | :-----: | :------: |
 *   | 4 bytes | 17 bytes |
 *
 *   The header field is further made up of several fields:
 *
 *   | Addr Low | Addr Med | Addr High | Ack   | Frame Counter |
 *   | :------: | :------: | :-------: | :---: | :-----------: |
 *   | 8 bits   | 8 bits   | 8 bits    | 1 bit | 7 bits        |
 *
 *   The 23-bit address field contains the slave address source of a message
 *   or destination of an answer.
 *
 *   The Ack flag is set by the receiver to acknowledge a received message.
 *
 *   The frame counter is a monotonic upwards counter that is incremented for
 *   each retransmission of a message frame during a single retry. This counter
 *   is used by network devices to compute the delay until the end of the
 *   current message.
  */
typedef struct {
    uint32_t    header;			/**< Header */
    uint8_t     payload[17];	/**< User payload */
} TD_LAN_frame_t;

/** User LAN callback */
typedef bool (*TD_LAN_callback_t)(void);

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup LAN_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup LAN_PROTOTYPES Prototypes
 * @{ */

bool TD_LAN_Init(bool init, uint32_t address, uint32_t mask);
bool TD_LAN_Release(void);
bool TD_LAN_Start(TD_LAN_mode_t transfer_mode, uint32_t count, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
void TD_LAN_Stop(TD_LAN_result_t result);
void TD_LAN_Abort(void);
void TD_LAN_Task(void);
void TD_LAN_Abort(void);
void TD_LAN_SetUserCallback(TD_LAN_callback_t callback);
uint32_t TD_LAN_ComputeAddress(uint32_t id);
bool TD_LAN_SendReceive(int count, uint8_t retries, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
bool TD_LAN_SendFrame(int32_t txcount, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
bool TD_LAN_ReceiveFrame(uint32_t rx_period, uint32_t limit, TD_LAN_frame_t *rx_frame);
bool TD_LAN_ReceiveFrameSync(TD_LAN_frame_t *rx_frame);

/** @} */
/** @} */

/** @} (end addtogroup TD_LAN) */

#ifdef __cplusplus
}
#endif

#endif // __TD_LAN_H
