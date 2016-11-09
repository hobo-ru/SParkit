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

#include "td1202.h"

#include <stdbool.h>
#include <efm32.h>

#include "td_flash.h"

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

/** Special user config page (512 bytes) */
#define E2P_USER        (0x8000 - FLASH_PAGE_SIZE)

/** Special factory page (512 bytes) */
#define E2P_FACTORY     0x0FE00000

/** DWORD size */
#define DWORDSZ         sizeof (uint32_t)

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup FLASH_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Enables the flash controller for writing.
 ******************************************************************************/
static void TD_FLASH_Init(void)
{
    // Enable writing to the MSC
	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Unlock the MSC
	MSC->LOCK = MSC_UNLOCK_CODE;

    // Disable writing to the MSC
	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

/***************************************************************************//**
 * @brief
 *   Disables the flash controller for writing.
 ******************************************************************************/
static void TD_Flash_Deinit(void)
{
    // Enable writing to the MSC
	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Lock the MSC
	MSC->LOCK = 0;

    // Disable writing to the MSC
	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

/***************************************************************************//**
 * @brief
 *   Programs a single word into flash.
 *
 * @note
 *   The flash must be erased prior to writing a new word.
 *   This function must be run from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 *
 * @note
 *   This function will not return until the data has been programmed.
 *
 * @details
 *   This function will program one word into the on-chip flash.
 *   Programming consists of ANDing the new data with the existing data; in
 *   other words bits that contain 1 can remain 1 or be changed to 0, but bits
 *   that are 0 can not be changed to 1.  Therefore, a word can be programmed
 *   multiple times so long as these rules are followed; if a program operation
 *   attempts to change a 0 bit to a 1 bit, that bit will not have its value
 *   changed.
 *
 * @param[in] address
 *   Pointer to the flash word to write to. Must be aligned to long words.
 * @param[in] data
 *   Data to write to flash.
 ******************************************************************************/
static __RAMFUNCTION void TD_FLASH_WriteWord(uint32_t address, uint32_t data)
{
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Load address
    MSC->ADDRB    = address;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    // Load data
    MSC->WDATA    = data;

    // Trigger write once
    MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

    // Wait for the write to complete
    while ((MSC->STATUS & MSC_STATUS_BUSY)) {
    	;
    }
}

/***************************************************************************//**
 * @brief
 *   Erases a block of flash.
 *
 * @note
 *   This function MUST be executed from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 *
 * @note
 *   This function will not return until the data has been erased.
 *
 * @details
 *   This function will erase one blocks on the on-chip flash.  After erasing,
 *   the block will be filled with 0xff bytes.  Read-only and execute-only
 *   blocks can not be erased.
 *
 * @param[in] blockStart
 *   Pointer to the flash page to erase. Must be aligned to beginning of page
 *   boundary.
 ******************************************************************************/
 __RAMFUNCTION void TD_FLASH_ErasePage(uint32_t blockStart)
{
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Load address
    MSC->ADDRB    = blockStart;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    // Send Erase Page command
    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

    // Wait for erase to complete
    while ((MSC->STATUS & MSC_STATUS_BUSY)) {
    	;
    }
}

 /***************************************************************************//**
 * @brief
 *   Computes CCITT CRC32.
 *
 * @param[in] buffer
 *   Pointer to the buffer containing the data.
 *
 * @param[in] size
 *   Size of the buffer in bytes.
 *
 * @return
 *   Returns the computed CRC32 of the buffer.
 ******************************************************************************/
 static uint32_t TD_FLASH_CRC32(uint32_t *buffer, uint32_t size)
 {
     uint32_t i, crc = 0, data;

     while (size--) {
         data = *buffer++;
         for (i = 32; i; i--) {
             if ((crc ^ data) & 1) {
                 crc = (crc >> 1) ^ 0x8408;
             } else {
                 crc = (crc >> 1);
             }
             data >>= 1;
         }
     }

     return(crc);
 }

 /***************************************************************************//**
 * @brief
 *   Writes a buffer to a given Flash region.
 *
 * @param[in] start
 *   Pointer to the flash region to write to.
 *
 * @param[in] buffer
 *   Pointer to the source buffer.
 *
 * @param[in] count
 *   Buffer size in bytes.
 ******************************************************************************/
void TD_FLASH_WriteRegion(uint32_t start, void *buffer, uint32_t count)
{
    uint32_t *pw = (uint32_t *) buffer;
    uint32_t i, acc = 0xFFFFFFFF;
    uint32_t crc;

    if (count % DWORDSZ) {

    	// Round count to DWORD
    	count += DWORDSZ;
    }
    count /= DWORDSZ;

    // Compute buffer CRC
    crc = TD_FLASH_CRC32(pw, count);

    // Optimization - check if block is already erased. This will typically happen when the chip is new
    for (i = start; i < (start + count + 1); i++) {
        acc &= *((int32_t *) i);
    }

    __disable_irq();
    TD_FLASH_Init();

    // If the accumulator is unchanged, there is no need to do an erase
    if (acc != 0xFFFFFFFF) {
    	TD_FLASH_ErasePage(start);
    }

    // Write CRC first
    TD_FLASH_WriteWord(start, crc);

    for (i = 0; i < count; i++) {
        start += DWORDSZ;
        TD_FLASH_WriteWord(start, *pw++);
    }

    // Disable writing to the MSC
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    TD_Flash_Deinit();

    // Interrupts can be enabled whenever not writing to or erasing flash
    __enable_irq();
}


void TD_FLASH_WriteRegion_noErase(uint32_t start, void *buffer, uint32_t count)
{
    uint32_t *pw = (uint32_t *) buffer;
    uint32_t i;//, acc = 0xFFFFFFFF;
    uint32_t crc;

    if (count % DWORDSZ) {

    	// Round count to DWORD
    	count += DWORDSZ;
    }
    count /= DWORDSZ;

    // Compute buffer CRC
    crc = TD_FLASH_CRC32(pw, count);

  /*  // Optimization - check if block is already erased. This will typically happen when the chip is new
    for (i = start; i < (start + count + 1); i++) {
        acc &= *((int32_t *) i);
    }
*/
    __disable_irq();
    TD_FLASH_Init();

  /*  // If the accumulator is unchanged, there is no need to do an erase
    if (acc != 0xFFFFFFFF) {
    	TD_FLASH_ErasePage(start);
    }*/

    // Write CRC first
    TD_FLASH_WriteWord(start, crc);

    for (i = 0; i < count; i++) {
        start += DWORDSZ;
        TD_FLASH_WriteWord(start, *pw++);
    }

    // Disable writing to the MSC
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    TD_Flash_Deinit();

    // Interrupts can be enabled whenever not writing to or erasing flash
    __enable_irq();
}

/***************************************************************************//**
* @brief
*   Reads a buffer from a given Flash region.
*
* @param[in] start
*   Pointer to the flash region to read from.
*
* @param[in] buffer
*   Pointer to the destination buffer.
*
* @param[in] count
*   Buffer size in bytes.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_ReadRegion(uint32_t start, void *buffer, uint32_t count)
{
    uint32_t i, *pr = (uint32_t *)start;
    uint32_t *pw = (uint32_t *)buffer;
    uint32_t crc;

    if (count % DWORDSZ) {

    	// Round count to DWORD
    	count += DWORDSZ;
    }
    count /= DWORDSZ;

    // Read the CRC first
    crc = *pr++;

    for (i = 0; i < count; i++) {
        *pw++ = *pr++;
    }
    if (crc != TD_FLASH_CRC32(buffer, count)) {
        return false;
    }
    return true;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup FLASH_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
* @brief
*   Writes a buffer to Flash memory.
*
* @param[in] buffer
*   Pointer to the source buffer.
*
* @param[in] count
*   Buffer size in bytes.
******************************************************************************/
void TD_FLASH_Write(void *buffer, uint32_t count)
{
    TD_FLASH_WriteRegion(E2P_USER, buffer, count);
}

/***************************************************************************//**
* @brief
*   Reads a buffer from Flash memory.
*
* @param[in] buffer
*   Pointer to the destination buffer.
*
* @param[in] count
*   Buffer size in bytes.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_Read(void *buffer, uint32_t count)
{
    return(TD_FLASH_ReadRegion(E2P_USER, buffer, count));
}

/** @cond RESTRICTED */
/***************************************************************************//**
* @brief
*   Writes a buffer to factory Flash memory.
*
* @param[in] device
*   Pointer to the source DEVICE buffer.
*
* @warning
*   Using this function destroys the unique module ID!
*
******************************************************************************/
void TD_FLASH_DeviceWrite(TD_DEVICE *device)
{
	TD_FLASH_WriteRegion(E2P_FACTORY, device, sizeof (TD_DEVICE));
}
/** @endcond */

/***************************************************************************//**
* @brief
*   Reads a buffer from factory Flash memory.
*
* @param[in] device
*   Pointer to the destination DEVICE buffer.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_DeviceRead(TD_DEVICE *device)
{
    return(TD_FLASH_ReadRegion(E2P_FACTORY, device, sizeof (TD_DEVICE)));
}

/** @} */

/** @} (end addtogroup FLASH) */
