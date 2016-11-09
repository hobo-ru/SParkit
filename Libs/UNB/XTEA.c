/*********************************************************************
 *
 *              Module for encrypting/decrypting data using 
 *              the second revision of the Tiny Encryption Algorigthm
 *				(commonly called XTEA, TEAN, and TEA-n).
 *
 *********************************************************************
 * FileName:        XTEA.c
 * Dependencies:    none
 * Processor:       18F452
 * Assembler:       MPASMWIN 02.70.02 or higher
 * Linker:          MPLINK 2.33.00 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") for its PICmicro(r) Microcontroller is intended and
 * supplied to you, the Company's customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * Microchip Technology Inc. ("Microchip") licenses this software to 
 * you solely for use with Microchip products.  The software is owned 
 * by Microchip and is protected under applicable copyright laws.  
 * All rights reserved.
 *
 * You may not export or re-export Software, technical data, direct 
 * products thereof or any other items which would violate any applicable
 * export control laws and regulations including, but not limited to, 
 * those of the United States or United Kingdom.  You agree that it is
 * your responsibility to obtain copies of and to familiarize yourself
 * fully with these laws and regulations to avoid violation.
 *
 * SOFTWARE IS PROVIDED "AS IS."  MICROCHIP EXPRESSLY DISCLAIM ANY 
 * WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
 * BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES,
 * LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF PROCUREMENT
 * OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS BY THIRD PARTIES
 * (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR 
 * INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS. 
 *
 *
 *
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Flowers        05/27/04    Initial Revision
 ********************************************************************/

/****************************** Headers *****************************/
#include "XTEA.h"

/****************************** Constants ***************************/
const unsigned long DELTA = 0x9E3779B9;

/****************************** Variables ***************************/
//unsigned long KEY[8]={1,2,3,4,5,6,7,8};

/****************************** Macros ******************************/

/****************************** Function Prototypes *****************/

/****************************** Functions ***************************/

/*********************************************************************
 * Function:        void Encode(unsigned long* data, unsigned int dataLength, unsigned char iterations)
 *
 * PreCondition:    None
 *
 * Input:           data - array of data to be encoded (must be even length)
 *					dataLength - length of data array (must be even)
 *					interations - number of Feistel cycles to iterate on the data 
 *					(min suggested is 16, 64 is standard for high security projects
 * 					highest levels cracked so far are around 10-12 cycles)
 *
 * Output:          None
 *
 * Side Effects:    _data values change
 *
 * Stack Requirements: None (3 unsigned long, 1 unsigned int, 1 unsigned char)
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/

void Encode(unsigned long* data, unsigned char dataLength, unsigned long* key)
{
	unsigned char i=0;
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterationCount;
	
	while(i<dataLength)
	{
		sum = 0;
		x1=*data;
		x2=*(data+1);
		iterationCount = NUM_ITERATIONS;

		while(iterationCount > 0)
		{
			x1 += ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
			sum+=DELTA;
			x2 += ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));

			iterationCount--;
		}
		*(data++)=x1;
		*(data++)=x2;
		i+=2;
	}	
}

/*********************************************************************
 * Function:        void Decode(unsigned long* data, unsigned int dataLength, unsigned char iterations)
 *
 * PreCondition:    None
 *
 * Input:           data - array of data to be decoded (must be even length)
 *					dataLength - length of data array (must be even)
 *					interations - number of Feistel cycles to iterate on the data 
 *					(min suggested is 16, 64 is standard for high security projects
 * 					highest levels cracked so far are around 10 cycles)
 *
 * Output:          None
 *
 * Side Effects:    _data values change
 *
 * Stack Requirements: None (3 unsigned long, 1 unsigned int)
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/

void Decode(unsigned long* data, unsigned char dataLength, unsigned long* key)
{
	unsigned char i=0;
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterations;

	iterations = NUM_ITERATIONS;
	
	while(i<dataLength)
	{
		sum = DELTA*iterations;
		x1=*data;
		x2=*(data+1);

		while(sum != 0)
		{
			x2 -= ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));
			sum-=DELTA;
			x1 -= ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
		}
		*(data++)=x1;
		*(data++)=x2;
		i+=2;
	}	
}

void XTEA_Encode(uint8_t *buf, unsigned long* key)
{
  Encode((unsigned long*)buf,2, &key[0]);
  Encode((unsigned long*)buf,2, &key[4]);
}

void XTEA_Decode(uint8_t *buf, unsigned long* key)
{
  Decode((unsigned long*)buf,2, &key[4]);
  Decode((unsigned long*)buf,2, &key[0]);
}

