/*
 * uart.c
 *
 *  Created on: 23 апр. 2014 г.
 *      Author: vdubikhin
 */


#include <stdint.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_usart.h>

#include <td_module.h>

#include "uart.h"

static circularBuffer rxBuf, txBuf = { {0}, 0, 0, 0, false };

extern const circularBuffer *const rxBuffer = &rxBuf;
extern const circularBuffer *const txBuffer = &txBuf;

/* Setup UART1 in async mode for RS232*/
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


void resetBuffers() {
    rxBuf.overflow = false;
    rxBuf.pendingBytes = 0;
    rxBuf.rdI = 0;
    rxBuf.wrI = 0;

    txBuf.overflow = false;
    txBuf.pendingBytes = 0;
    txBuf.rdI = 0;
    txBuf.wrI = 0;
}

/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void uartStart(void)
{
    //Initialize soft buffers
    resetBuffers();

    /* Enable clock for GPIO module (required for pin configuration) */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Enable clock for USART module */
    CMU_ClockEnable(cmuClock_USART1, true);

    /* Configure GPIO pins */
    GPIO_PinModeSet(TIM1_PORT, TIM1_BIT, gpioModeInput, 1); //PD6 pin as RX

    /* Prepare struct for initializing UART in asynchronous mode*/
    uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
    uartInit.baudrate     = 1200;         /* Baud rate */
    uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
    uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
    uartInit.parity       = usartNoParity;  /* Parity mode */
    uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
    uartInit.mvdis        = false;          /* Disable majority voting */
    uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
    uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

    /* Initialize USART with uartInit struct */
    USART_InitAsync(USART1, &uartInit);

    /* Prepare UART Rx and Tx interrupts */
    USART_IntClear(USART1, _USART_IF_MASK);
    USART_IntEnable(USART1, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ(USART1_RX_IRQn);
    NVIC_ClearPendingIRQ(USART1_TX_IRQn);
    NVIC_EnableIRQ(USART1_RX_IRQn);
    NVIC_EnableIRQ(USART1_TX_IRQn);

    /* Enable I/O pins at USART1 location #2 */
    USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC2;

    /* Enable UART */
    USART_Enable(USART1, usartEnable);
}

// Stop uarts
void uartStop() {
    //stop transmissions
    USART_Enable(USART1, usartDisable);

    /* clear UART Rx and Tx interrupts */
    USART_IntClear(USART1, _USART_IF_MASK);
    NVIC_ClearPendingIRQ(USART1_RX_IRQn);
    NVIC_ClearPendingIRQ(USART1_TX_IRQn);

    /* Disable clock for USART module */
    CMU_ClockEnable(cmuClock_USART1, false);
}

/**************************************************************************//**
 * @brief UART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
    if (USART1->STATUS & USART_STATUS_RXDATAV) {
        if (rxBuf.pendingBytes >= BUFFERSIZE) {
          /* Flag Rx overflow */
          rxBuf.overflow = true;
        } else {
        /* Copy data into RX Buffer */
            uint8_t rxData = USART_Rx(USART1);
            rxBuf.data[rxBuf.wrI] = rxData;
            rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
            rxBuf.pendingBytes++;
        }

        /* Clear RXDATAV interrupt */
        USART_IntClear(USART1, USART_IF_RXDATAV);
  }
}

/**************************************************************************//**
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  /* Check TX buffer level status */
  if (USART1->STATUS & USART_STATUS_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(USART1, txBuf.data[txBuf.rdI]);
      txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(USART1, USART_IF_TXBL);
    }
  }
}


/******************************************************************************
 * @brief  uartPutChar function
 *
 *****************************************************************************/
void uartPutChar(uint8_t ch)
{
  /* Check if Tx queue has room for new data */
  if ((txBuf.pendingBytes + 1) > BUFFERSIZE)
  {
    /* Wait until there is room in queue */
    while ((txBuf.pendingBytes + 1) > BUFFERSIZE) ;
  }

  /* Copy ch into txBuffer */
  txBuf.data[txBuf.wrI] = ch;
  txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;

  /* Increment pending byte counter */
  txBuf.pendingBytes++;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(USART1, USART_IF_TXBL);
}


/******************************************************************************
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will hang until a character is received.
 *
 *****************************************************************************/
uint8_t uartGetChar( )
{
  uint8_t ch;

  /* Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data */
  if (rxBuf.pendingBytes < 1)
  {
    while (rxBuf.pendingBytes < 1) ;
  }

  /* Copy data from buffer */
  ch        = rxBuf.data[rxBuf.rdI];
  rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

  /* Decrement pending byte counter */
  rxBuf.pendingBytes--;

  return ch;
}

