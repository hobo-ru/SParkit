/*
* UNB.c
*
*  Created on: 06.11.2013
*      Author: Alexey Kabanov
*
*      Sending ITS message
*		(protokol 1_1)
*      This program sends ITS message 3 times every 250 * 4 ms
*      1.1 - remove excessive arrays
* 		1.2 - crc lookup table is optional to save rom space (S.Om)
*/


#include <efm32tg210f32.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <td1202.h>

#include <em_device.h>
#include <em_msc.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_bitband.h>
#include <em_usart.h>
#include <em_leuart.h>
#include <float.h>
#include <math.h>

//#include <td_flash.h>
//#include <td_uart.h>
//#include <td_rtc.h>

#include "UNB.h"
#include "XTEA.h"


#define  ENCRYPT


void battery_start_measure();

uint32_t* UNB_KEY;


#define NO_CRC_TABLE

#define HIGH_SPEED_CLOCK        7000000                                     // 6M
#define USART                   USART0                                      /* USART used for Radio  module access */
#define USART_ROUTE_LOCATION    USART_ROUTE_LOCATION_LOC0

#define GPIO_TIM2_BIT_DOUTSET   (0x7C)
#define GPIO_TIM2_BIT_DOUTCLR	(0x80)
#define MODULE

#define GPIO_CFG_SDI            (gpioModePushPull << ((SDI_RF_BIT - 8) * 4))
#define GPIO_CFG_SDO            (gpioModeInput << ((SDO_RF_BIT - 8) * 4))
#define GPIO_CFG_CLK            (gpioModePushPull << ((SCLK_RF_BIT - 8) * 4))
#define GPIO_CFG_CS             (gpioModePushPull << ((NSEL_RF_BIT - 8) * 4))
#define GPIO_CFG_SHTD           (gpioModePushPull << ((SHTD_BIT - 8) * 4))

//#define GPIO_CFG_CTS          (gpioModeInput << ((GP1_BIT - 8) * 4))
#define GPIO_CFG_TCXO           (gpioModePushPull << ((POWER_CRYSTAL_BIT) * 4))
#define GPIO_CFG_IRQ            (gpioModeInput << ((NIRQ_RF_BIT) * 4))
#define GPIO_CFG_LED            (gpioModePushPull << ((TIM2_BIT) * 4))

#define	SHTD_PORT               gpioPortC           // RF GPIO Pin 1
#define	SHTD_BIT                13
#define	SHTD_MASK               (1 << SHTD_BIT)

#define CMD_START_RX 		    0x32
#define CMD_FIFO_READ		    0x000077
#define CMD_CLEAR_INT		    0x000020

#define WRITE_RAM 		     	0x00400000
#define WRITE_FLASH		     	0x00500000
#define ERASE_ANDWRITEFLASH		0x00600000
#define REBOOT			     	0x00700000
#define JUMP			     	0x00800000
#define LENGTH_FRAME            64

#define MAX_RX_LENGTH           60

#define XO_TUNE                 0x11, 0x00, 0x01, 0x00, 0x52
#define CAP_BANK_VALUE          0x48  // Capacitor bank value for adjusting the XTAL frequency

#define MODEM_DATA_RATE_2_14  0x11, 0x20, 0x0A, 0x03, 0x00, 0x07, 0xd0, 0x00, 0x27, 0xac, 0x40, 0x00, 0x00, 0x0a
// data rate = 1 kHz 00, 0x03, 0xef // dev = 250Hz 0x00, 0x00, 0x0a for FSK
// 4k = 0x0f, 0xa0
//2k = 0x07,0xd0
#define MODEM_TX_RAMP_DELAY_5   0x11, 0x20, 0x01, 0x18, 0x01

#define MODEM_MOD_TYPE_7        0x11, 0x20, 0x03, 0x00, 0x4A, 0x00, 0x07
//direct TX FSK
#define MODEM_CLKGEN_BAND_5     0x11, 0x20, 0x01, 0x51, 0x08
#define SYNTH_PFDCP_CPFF_11     0x11, 0x23, 0x07, 0x00, 0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03
#define FREQ_CONTROL_INTE_12    0x11, 0x40, 0x08, 0x00, 0x41,0x0e,0xa5,0x6a, 0x27, 0x62, 0x20,  0xFF


#define MODEM_MDM_CTRL_11       0x11, 0x20, 0x07, 0x19, 0x00, 0x08, 0x03, 0xC0, 0x00, 0x10, 0x10
#define MODEM_BCR_OSR_1_14      0x11, 0x20, 0x0A, 0x22, 0x00, 0x64, 0x05, 0x1E, 0xB8, 0x05, 0x1F, 0x02, 0x00, 0x00
#define MODEM_AFC_GEAR_12       0x11, 0x20, 0x08, 0x2C, 0x00, 0x12, 0x80, 0xDA, 0x01, 0xEB, 0xE0, 0x00
#define MODEM_AGC_CONTRL_5      0x11, 0x20, 0x01, 0x35, 0xE0
#define MODEM_AGC_WINDOW_SIZE_7 0x11, 0x20, 0x03, 0x38, 0x11, 0x16, 0x16
#define MODEM_FSK4_GAIN1_9      0x11, 0x20, 0x05, 0x3B, 0x0B, 0x1C, 0x40, 0x00, 0xE1
#define MODEM_OOK_PDTC_8        0x11, 0x20, 0x04, 0x40, 0x28, 0x0C, 0xA4, 0x03
#define MODEM_RAW_SEARCH_8      0x11, 0x20, 0x04, 0x44, 0xD6, 0x02, 0x00, 0x7B
#define MODEM_ANT_DIV_MODE_6    0x11, 0x20, 0x02, 0x48, 0x02, 0x80
#define MODEM_RSSI_COMP_5       0x11, 0x20, 0x01, 0x4E, 0x38
#define MODEM_CHFLT_RX1_CHFLT_COE13_7_0_13 0x11, 0x21, 0x09, 0x00,  0xA2, 0x81, 0x26, 0xAF, 0x3F, 0xEE, 0xC8, 0xC7, 0xDB
#define MODEM_CHFLT_RX1_CHFLT_COE4_7_0_13  0x11, 0x21, 0x09, 0x09,  0xF2, 0x02, 0x08, 0x07, 0x03, 0x15, 0xFC, 0x0F, 0x00
#define MODEM_CHFLT_RX2_CHFLT_COE13_7_0_13 0x11, 0x21, 0x09, 0x12,  0xA2, 0x81, 0x26, 0xAF, 0x3F, 0xEE, 0xC8, 0xC7, 0xDB
#define MODEM_CHFLT_RX2_CHFLT_COE4_7_0_13  0x11, 0x21, 0x09, 0x1B,  0xF2, 0x02, 0x08, 0x07, 0x03, 0x15, 0xFC, 0x0F, 0x00
#define PA_MODE                 0x10, 0x00, 0x00, 0x11, 0x22, 0x01, 0x00, 0x21
#define PA_PWR_LVL              0x10, 0x00, 0x00, 0x11, 0x22, 0x01, 0x01, 0x7f
#define PA_BIAS_CLKDUTY         0x10, 0x00, 0x00, 0x11, 0x22, 0x01, 0x02, 0x22

static const uint8_t UNB_Si4461_Loader[] = {
	7, CMD_POWER_UP, 0x01, 0x01, 0x01, 0x8C, 0xBA, 0x80,
	4, CMD_GET_INT_STATUS, 0x00, 0x00, 0x00,
	
	7, MODEM_MOD_TYPE_7,
	
	5, MODEM_CLKGEN_BAND_5,
	11, SYNTH_PFDCP_CPFF_11,
	12, FREQ_CONTROL_INTE_12,
	11, MODEM_MDM_CTRL_11,
	14, MODEM_BCR_OSR_1_14,
	12, MODEM_AFC_GEAR_12,
	5, MODEM_AGC_CONTRL_5,
	7, MODEM_AGC_WINDOW_SIZE_7,
	9, MODEM_FSK4_GAIN1_9,
	8, MODEM_OOK_PDTC_8,
	8, MODEM_RAW_SEARCH_8,
	6, MODEM_ANT_DIV_MODE_6,
	14, MODEM_DATA_RATE_2_14,
	5, MODEM_TX_RAMP_DELAY_5,
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x20, 0x7f, 0x2C, 0x01, //65mA sf+5
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x08, 0x7f, 0x2C, 0x01, //21mA
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x04, 0x7f, 0x2C, 0x01, //18mA
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x21, 0x7f, 0x2C, 0x01, //43mA -51db
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0C, 0x7f, 0x2C, 0x01, //75mA sf + 6db
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0D, 0x7f, 0x2C, 0x01, //49mA  -47db
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0D, 0x7f, 0x6C, 0x01, //38mA
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0D, 0x7f, 0xAC, 0x01, //47.5mA
	//8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0D, 0x7f, 0xEC, 0x01, //37mA
	8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0C, 0x7f, 0xEC, 0x01, //48mA = sf
	
	8, CMD_SET_PROPERTY,PROP_FRR_CTL_GROUP, 4, PROP_FRR_CTL_A_MODE, 0x04, 0x06, 0x0A, 0x00,
	5, CMD_SET_PROPERTY,PROP_MODEM_GROUP, 1, PROP_MODEM_RSSI_CONTROL, 0x12,
	5, CMD_SET_PROPERTY,PROP_GLOBAL_GROUP, 1, PROP_GLOBAL_XO_TUNE, 0,
	6,CMD_GPIO_PIN_CFG, 0x00, 0x00, 0x04, 0x00, 0x10,
	2, CMD_FIFO_INFO, 3,
	8, CMD_START_RX, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0
};

/** Command table for restoring the RF chip to its original state */
static const uint8_t  UNB_Si4461_Clean[] = {
	
	7, CMD_SET_PROPERTY,PROP_PKT_GROUP, 3, PROP_PKT_LEN, 0x00, 0x00, 0x00,
	2, CMD_CHANGE_STATE, 0x01,
	0
};

static const uint8_t  UNB_ModemMode[] =             {MODEM_MOD_TYPE_7};//{0x07, 0x11, 0x20, 0x03, 0x00, 0x03, 0x00, 0x07};
static const uint8_t  UNB_StateModemSleep[] =       {2, CMD_CHANGE_STATE, 0x01};
static const uint8_t  UNB_StateModemRx[] =          {8, CMD_START_RX, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t UNB_SetPowerTx[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0C, 0x7f, 0xEC, 0x01};

static const uint8_t UNB_LowPower[]    = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x08, 20, 0xEC, 0x01};

static const uint8_t UNB_NormalPower[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0, 0x7f, 0xC0, 0x01};  //NormalPower
static const uint8_t UNB_NormalPower_m3[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0, 0x56, 0xC0, 0x01};  //NormalPower -3dBm(-8לְ)
static const uint8_t UNB_NormalPower_m6[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0, 0x3B, 0xC0, 0x01};  //NormalPower -6dBm(-14לְ)
static const uint8_t UNB_NormalPower_m9[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0, 0x2A, 0xC0, 0x01};  //NormalPower -9dBm(-17לְ)
static const uint8_t UNB_NormalPower_m12[] = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0, 0x1D, 0xC0, 0x01};  //NormalPower -12dBm(-20לְ)




static const uint8_t UNB_HighPower[]   = {8, CMD_SET_PROPERTY,PROP_PA_GROUP, 4, PROP_PA_MODE, 0x0C, 0x7f, 0x2C, 0x01,};

//CMU_Clock_TypeDef clock;
//uint32_t freq;
//uint32_t TD_RandomValue;

void UNB_Tx_setup();
void UNB_Tx_shutdoun();
void UNB_sendgetradio(char * buffer,unsigned int txlength,char * rxbuffer,unsigned int rxlength);
uint8_t UNB_Tx_send(uint8_t *message, uint8_t length,uint32_t serial);

/** @} */
void UNB_Tx_setup(){
	uint32_t Cpt,i;
	volatile unsigned int count;
	
	
	BITBAND_Peripheral(&(CMU->HFPERCLKDIV), (cmuClock_HFPER >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);
	BITBAND_Peripheral(&(CMU->HFPERCLKEN0), ((cmuClock_USART0)  >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);
	BITBAND_Peripheral(&(CMU->HFPERCLKEN0), ((cmuClock_GPIO)  >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);
	
	//	    if ((((TD_DEVICE *)(E2P_FACTORY))->ModResult) != 0x0E)
	//	    {
	GPIO->P[SHTD_PORT].DOUTSET |= 1 << SHTD_BIT;               				// Shutdown
	GPIO->P[SHTD_PORT].MODEH |= GPIO_CFG_SHTD;                          	// Shutdown
	GPIO->P[POWER_CRYSTAL_PORT].DOUTSET |= 1 << POWER_CRYSTAL_BIT;           // set TXCO ON
	GPIO->P[POWER_CRYSTAL_PORT].MODEH |= GPIO_CFG_TCXO;                      // set TCXO mode IO
	for (count = 0; count < 50000; count++); 								// 20 ms...
	GPIO->P[SHTD_PORT].DOUTCLR |= 1 << SHTD_BIT;               				// Wake up
	//	    }
	
	GPIO->P[POWER_CRYSTAL_PORT].DOUTSET |= 1 << POWER_CRYSTAL_BIT;               // Set TXCO ON
	GPIO->P[POWER_CRYSTAL_PORT].MODEL |= GPIO_CFG_TCXO;                         // Set TCXO mode IO
	GPIO->P[NSEL_RF_PORT].DOUTSET |= 1 << NSEL_RF_BIT;                           // Set radio chip select
	
	GPIO->P[NSEL_RF_PORT].MODEH |= GPIO_CFG_SDI|GPIO_CFG_SDO|                   // Set SPI mode IO
		GPIO_CFG_CLK|GPIO_CFG_CS ;
	//    GPIO->P[GP1_PORT].MODEH     |= GPIO_CFG_CTS;                              // Set CTS mode IO
	GPIO->P[NIRQ_RF_PORT].MODEL |= GPIO_CFG_IRQ;                                // Set IRQ mode IO (TBC to pullup mode)
	
	USART->CTRL = (USART_CTRL_SYNC |USART_CTRL_MSBF) ;                          // UART INIT from reset state
	USART->CLKDIV = 0;                                                          // 0 = max
	USART->CMD = USART_CMD_MASTEREN | (USART_CMD_RXEN | USART_CMD_TXEN);
	USART0->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
		USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION;
	
	// Enable writing to the MSC
	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
	
	// Unlock the MSC
	MSC->LOCK = MSC_UNLOCK_CODE;
	
	// Disable writing to the MSC
	MSC->TIMEBASE=0x10;
	
	// Configure MSC->TIMEBASE according to selected frequency
	// GPIO->P[TIM2_PORT].MODEL |= GPIO_CFG_LED;                                   // Set LED mode IO
	
	for (count = 0; count < 50000; count++);									// Required delay, do not remove!
	for (i = 0, Cpt = 0; UNB_Si4461_Loader[i] != 0; i += Cpt+1) {
		Cpt = UNB_Si4461_Loader[i];
		UNB_sendgetradio((char *) &UNB_Si4461_Loader[i + 1], Cpt,(char *) &UNB_Si4461_Loader[i + 1],0);
	}
	UNB_sendgetradio((char *) &UNB_SetPowerTx[1], UNB_SetPowerTx[0],(char *) &UNB_SetPowerTx[1],0); //set Tx power
	
	//GPIO_PinModeSet(GP2_PORT, GP2_BIT, gpioModeInput, 0);                                  // on counter
	//Cpt = 0;
	//for(i = 0;i<50000;i++)
	//{
	//if (GPIO_PinInGet(GP2_PORT, GP2_BIT)) ++Cpt;
	//}
	//GPIO_PinModeSet(GP2_PORT, GP2_BIT, gpioModeDisabled, 0);                               // off counter
	// modem off
	UNB_sendgetradio((char *) &UNB_StateModemSleep[1], UNB_StateModemSleep[0],(char *) &UNB_StateModemSleep[1],0);
	// restore right value
	UNB_sendgetradio((char *) &UNB_ModemMode[1], UNB_ModemMode[0],(char *) &UNB_ModemMode[1],0);
	// start RX in normal FSK
	UNB_sendgetradio((char *) &UNB_StateModemRx[1], UNB_StateModemRx[0],(char *) &UNB_StateModemRx[1],0);
	//TD_RandomValue = Cpt;
	
	
	
}

void UNB_Tx_shutdoun(){
	uint32_t Cpt,i;
	volatile unsigned int count;
	for (i = 0, Cpt = 0; UNB_Si4461_Clean[i] != 0; i += Cpt + 1) {
		Cpt = UNB_Si4461_Clean[i];
		UNB_sendgetradio((char *) &UNB_Si4461_Clean[i + 1], Cpt,(char *) &UNB_Si4461_Clean[i + 1], 0);
	}
	//if ((((TD_DEVICE *)(E2P_FACTORY))->ModResult) != 0x0E)
	// {
	GPIO->P[SHTD_PORT].DOUTSET |= 1 << SHTD_BIT;               				// Shutdown
	GPIO->P[SHTD_PORT].MODEH |= GPIO_CFG_SHTD;                          		// Shutdown
	GPIO->P[POWER_CRYSTAL_PORT].DOUTCLR |= 1 << POWER_CRYSTAL_BIT;           // Set TXCO off
	//for (count = 0;count < 50000; count++);									// Few ms... you need not =)
	// }
	
}

void UNB_Set_KEY_ptr(uint32_t* ptr)
{
  UNB_KEY = ptr;
}


uint32_t setFreq;
#define FREQ868_775 0x410ea17b
#define FREQ868_825 0x410ea95b

uint32_t lowFreq  = FREQ868_775;
uint32_t highFreq =  FREQ868_825;
int numChannel = 100;

char commandBufFreq[8];
void UNB_setFreq(uint32_t serial)
{
	uint32_t freqReg;
	setFreq += serial%(numChannel-1);
	if(setFreq > (numChannel-2)){
		setFreq = setFreq%(numChannel-1);
	}
	//freqReg = freq915[setFreq];
	freqReg = lowFreq + ((highFreq - lowFreq)/numChannel)*(setFreq+1);
	commandBufFreq[0] = CMD_SET_PROPERTY;
	commandBufFreq[1] = PROP_FREQ_CONTROL_GROUP;
	commandBufFreq[2] = 0x04;//0x30;//
	commandBufFreq[3] = 0x00;//0;  0x01E0 = 480 bit
	commandBufFreq[4] = (char)(freqReg>>24);//
	commandBufFreq[5] = (char)(freqReg>>16);//
	commandBufFreq[6] = (char)(freqReg>>8);//
	commandBufFreq[7] = (char)(freqReg);//
	UNB_sendgetradio(commandBufFreq, 8, commandBufFreq, 0);
}

uint8_t length_header[21] = {0,0,0,1,2,3,4,5,6,7,8,9,10,11,11,12,12,13,13,14,14};
uint32_t digital_crc32(const uint8_t *buf, uint8_t len);
uint8_t txBuf[32];

uint8_t UNB_Tx_send(uint8_t *message, uint8_t length, uint32_t serial ){
	uint8_t txBuf[32];
	//uint8_t dataBuf[21];
	char commandBuf[6];
	uint8_t messLength,normLength;
	uint8_t byteCounter = 0;
	uint32_t crc = 0;
	
	
	if((!length)||(length > 20))return 0;   // UNB message no more than 12bytes
	
	//normalize length
	if(length<3)normLength = 2;
	else if(length<13)normLength = length;
	else if(length<15)normLength = 14;
	else if(length<17)normLength = 16;
	else if(length<19)normLength = 18;
	else normLength = 20;
	//Form message
	//preamble AA F0 99
	txBuf[0] = 0xAA;
	txBuf[1] = 0xF0;
	txBuf[2] = 0x99;
	txBuf[3] = length_header[normLength];
	txBuf[4] = (uint8_t)(serial >> 24);; // 4bytes serial number
	txBuf[5] = (uint8_t)(serial >> 16);;
	txBuf[6] = (uint8_t)(serial >> 8);
	txBuf[7] = (uint8_t)(serial);
	
	for(byteCounter = 8;byteCounter < (length + 8);byteCounter++){ //payload
		txBuf[byteCounter] = message[byteCounter - 8];
	}
	
	if(length > 12){
		if(length%2){ //odd
			txBuf[byteCounter] = 0;
			byteCounter++;
		}
	}
	
	
	crc = digital_crc32(&txBuf[3], normLength + 5);
	
	txBuf[byteCounter] = (uint8_t)(crc>>24);
	txBuf[byteCounter + 1] = (uint8_t)(crc>>16);
	txBuf[byteCounter + 2] = (uint8_t)(crc>>8);
	txBuf[byteCounter + 3] = (uint8_t)crc;
	
	messLength = normLength + 3 + 1 + 4 + 4; // payload + preamb + counter + serial + crc
	
	
	
	GPIO_PinModeSet(GP2_PORT, GP2_BIT, gpioModePushPull, 0);
	GPIO_PinOutClear(GP2_PORT, GP2_BIT);
	
	
	for(uint8_t repeat = 0; repeat < 3; repeat++)  //three messages to send
	{
		
		UNB_setFreq(serial);		//change frequency within band
		
		commandBuf[0] = CMD_CHANGE_STATE;
		commandBuf[1] = 0x05;//3;
		UNB_sendgetradio(commandBuf, 2, commandBuf, 0);
		
		commandBuf[0] = CMD_START_TX;
		commandBuf[1] = 0;
		commandBuf[2] = 0x50;//0x30;//
		commandBuf[3] = 0x00;//0;  0x01E0 = 480 bit
		commandBuf[4] = 0;//
		UNB_sendgetradio(commandBuf, 5, commandBuf, 0);
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//send two bits 1,0;
		//send 1
		for(uint8_t tackts = 0;tackts<20; tackts++)
		{
			// Wait for packet sent interrupt
			while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
			while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is high
			
		}
		//send 0
		for(uint8_t tackts = 0;tackts<20; tackts++)
		{
			// Wait for packet sent interrupt
			while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
			
			if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
			else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
			//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
			//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
			while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is high
			
		}
		
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		for(uint32_t i = 0;i<3;i++)  //send preambula
		{
			uint8_t simbol = txBuf[i];
			for(uint8_t bit = 0; bit<8;bit++)
			{
				
				uint8_t digit = simbol & (0x80 >> bit);
				for(uint8_t tackts = 0;tackts<20; tackts++)
				{
					// Wait for packet sent interrupt
					while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
					if(!digit)
					{
						
						if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
						else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
						//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
						//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
					}
					
					
					while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
					
				}
				
			}
		}
                battery_start_measure();
		uint8_t simbol = txBuf[3]; //send length
		for(uint8_t bit = 0; bit < 4;bit++)
		{
			
			uint8_t digit = simbol & (0x08 >> bit);
			for(uint8_t tackts = 0;tackts<20; tackts++)
			{
				// Wait for packet sent interrupt
				while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
				if(!digit)
				{
					
					if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
					//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
				}
				while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
			}
			
		}
		for(uint32_t i = 4; i < messLength ;i++)  //send the rest of the message
		{
			uint8_t simbol = txBuf[i];
			for(uint8_t bit = 0; bit<8;bit++)
			{
				uint8_t digit = simbol & (0x80 >> bit);
				
				for(uint8_t tackts = 0;tackts<20; tackts++)
				{
					
					// Wait for packet sent interrupt
					while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
					if(!digit)
					{
						
						if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
						else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
						//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
						//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
					}
					while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
				}
			}
		}
		
		commandBuf[0] = CMD_CHANGE_STATE;
		commandBuf[1] = 0x01;//3;
		UNB_sendgetradio(commandBuf, 2,commandBuf, 0);
		
		//for(uint32_t j = 0;j<100000;j++); //delay
		
	}//end three messages sending routine
	return 1;
}

#include "td_rtc.h"
void UNB_Tx_send_carrier(uint32_t ticks)
{
	UNB_Tx_setup();
	char commandBuf[6];
	
	GPIO_PinModeSet(GP2_PORT, GP2_BIT, gpioModePushPull, 0);
	GPIO_PinOutClear(GP2_PORT, GP2_BIT);
	
	UNB_setFreq(0);//change frequency within band
	
	commandBuf[0] = CMD_CHANGE_STATE;
	commandBuf[1] = 0x05;//3;
	UNB_sendgetradio(commandBuf, 2, commandBuf, 0);
	
	commandBuf[0] = CMD_START_TX;
	commandBuf[1] = 0;
	commandBuf[2] = 0x50;//0x30;//
	commandBuf[3] = 0x00;//0;  0x01E0 = 480 bit
	commandBuf[4] = 0;//
	UNB_sendgetradio(commandBuf, 5, commandBuf, 0);
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	if(ticks)
	{
		TD_RTC_Delay(ticks);
		
		commandBuf[0] = CMD_CHANGE_STATE;
		commandBuf[1] = 0x01;//3;
		UNB_sendgetradio(commandBuf, 2,commandBuf, 0);
		UNB_Tx_shutdoun();
	}
	else return;
}

void UNB_Tx_stop_send_carrier()
{
	char commandBuf[6];
	commandBuf[0] = CMD_CHANGE_STATE;
	commandBuf[1] = 0x01;//3;
	UNB_sendgetradio(commandBuf, 2,commandBuf, 0);
	UNB_Tx_shutdoun();
}

uint8_t UNBsend(uint8_t *message, uint8_t length,uint32_t serial)
{
	uint8_t complete;
	UNB_Tx_setup();
	complete = UNB_Tx_send(message, length, serial);
	UNB_Tx_shutdoun();
	return complete;
}

// Zigzag code permutation table
const uint8_t n[4][128] =
{
	{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127},
	{104,52,43,96,31,7,71,78,58,37,93,25,125,85,42,111,6,95,72,117,27,51,63,84,91,35,120,26,97,45,110,70,1,28,86,114,53,67,12,127,40,101,73,94,115,61,20,126,3,46,92,116,9,56,87,77,109,44,65,54,100,118,2,34,21,41,76,14,69,124,90,18,103,48,113,36,0,81,13,62,24,38,105,68,15,75,88,50,122,29,83,102,8,16,108,23,32,49,99,112,19,55,89,11,107,82,47,98,22,30,60,80,66,121,10,57,17,39,79,4,64,123,33,59,106,74,5,119},
	{26,10,105,48,38,84,76,57,23,125,115,3,106,33,77,99,71,113,22,1,44,87,8,31,111,96,2,42,70,81,13,93,122,37,114,88,63,107,50,40,82,116,68,6,127,16,51,73,61,83,46,0,126,104,78,67,41,119,28,11,56,47,4,21,52,66,15,98,24,7,30,91,112,35,55,124,64,5,95,32,49,9,85,65,43,18,92,36,12,86,118,60,25,72,53,80,123,45,58,102,110,120,89,34,17,75,94,27,100,62,20,39,108,90,69,117,97,59,79,109,101,19,121,54,29,14,74,103},
	{0,93,104,36,87,125,23,97,44,107,11,3,70,35,60,77,29,84,6,91,126,15,76,56,4,89,115,99,43,22,122,16,105,55,2,113,78,51,63,14,120,102,8,19,68,111,86,47,64,32,121,72,59,108,96,80,25,67,118,12,58,127,20,90,9,37,103,53,62,69,85,10,110,34,100,119,39,73,1,83,48,112,30,54,65,45,5,123,101,26,88,18,46,95,40,109,7,27,57,66,116,38,75,92,21,52,61,28,106,114,94,33,17,79,42,71,124,50,82,13,31,41,117,74,98,81,24,49}
};

static void CalculateZigzag(uint8_t* src_buf, uint8_t* dst_buf, bool parity)
{
	uint8_t b0;     // source bit 0
	uint8_t b1;     // source bit 0
	uint8_t bprev;  // previous result bit
	uint8_t res;    // result bit
	
	// code[N] is zigzag code for Nth permutation
	uint8_t code[4][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	
	// calculate zigzag code for each permutation
	for(int j=0; j<4; j++)
	{
		for(uint8_t i=0; i<64; i++) //payload 16bytes, each zigzag code 8bytes
		{
			b0      = (src_buf[ n[j][ i ]  /8] << ( n[j][ i ]  %8)) & 0x80;
			b1      = (src_buf[ n[j][64+i] /8] << ( n[j][64+i] %8)) & 0x80;
			bprev   = (code[j][   (i-1)    /8] << (   (i-1)    %8)) & 0x80;
			res     =  b0 ^ b1 ^ bprev;
			code[j][i/8] |= res >> (i%8);
		}
	}
	
	// append result to dst_buf
	for(int i=0; i<8;i++)
	{
		// depending on parity, use either odd or even parts of zigzag code
		dst_buf[i]      = (code[0][i] & (parity ? 0xAA : 0x55)) | (code[1][i] & (parity ? 0x55 : 0xAA));
		dst_buf[i+8]    = (code[2][i] & (parity ? 0xAA : 0x55)) | (code[3][i] & (parity ? 0x55 : 0xAA));
	}
}

uint16_t random_seed;
uint16_t random(void)
{
	random_seed = 28629U * random_seed + 157U;
	return random_seed;
}

uint8_t iter = 0;
uint8_t channelC = 0;
uint32_t setFreqC = 0;

uint8_t UNB_GetCiter() {return iter;}


void UNB_setFreqC(uint32_t parity)
{
	uint32_t freqReg;
//	setFreq += serial%(numChannel-1);
//	if(setFreq > (numChannel-2)){
//		setFreq = setFreq%(numChannel-1);
//	}
	
	setFreqC = channelC + ((parity)?150:0);
	
	freqReg = lowFreq + ((highFreq - lowFreq)/251)*(setFreqC);
	commandBufFreq[0] = CMD_SET_PROPERTY;
	commandBufFreq[1] = PROP_FREQ_CONTROL_GROUP;
	commandBufFreq[2] = 0x04;//0x30;//
	commandBufFreq[3] = 0x00;//0;  0x01E0 = 480 bit
	commandBufFreq[4] = (char)(freqReg>>24);//
	commandBufFreq[5] = (char)(freqReg>>16);//
	commandBufFreq[6] = (char)(freqReg>>8);//
	commandBufFreq[7] = (char)(freqReg);//
	UNB_sendgetradio(commandBufFreq, 8, commandBufFreq, 0);
}

void HopChannel()
{
    channelC = random() % 101;
}

static void UNB_ProtocolC_SendOneMsg(uint8_t * buf, uint32_t serial, uint8_t parity)
{
	static uint8_t tx_packet_encoded[255];
	static uint8_t tx_packet[64];
	uint8_t preambula[] = {0x97, 0x15, 0x7A};
	uint8_t ptr = 0, ptr_enc = 0, i, j;
	uint32_t crc = 0xFFFFFFFF;
	char commandBuf[6];
	
	// clear tx_packet buffer
	for(i=0;i<sizeof(tx_packet);i++)    tx_packet[i]=0;
	
	// form UNB protocol B packet
	tx_packet[ptr++] = 0x6F;//header
	tx_packet[ptr++] = (uint8_t)(serial >> 8);
	tx_packet[ptr++] = (uint8_t)(serial);
	tx_packet[ptr++] = (iter)&0xF;
	for(i = 0; i < 8; i++)
	{
		tx_packet[ptr++] = buf[i];
	}
        #ifdef ENCRYPT
        for(i = 0; i < 8; i++)  if(UNB_KEY[i]) break;
        if(i != 8) XTEA_Encode(&tx_packet[ptr-8], UNB_KEY);       
        #endif
	crc = digital_crc32(tx_packet, 8+4);
	tx_packet[ptr++] = (uint8_t)(crc >> 24);
	tx_packet[ptr++] = (uint8_t)(crc >> 16);
	tx_packet[ptr++] = (uint8_t)(crc >> 8);
	tx_packet[ptr++] = (uint8_t)(crc);
	        
        CalculateZigzag(tx_packet, tx_packet + ptr, parity);

	ptr += 16;
	
		
	GPIO_PinModeSet(GP2_PORT, GP2_BIT, gpioModePushPull, 0);
	GPIO_PinOutClear(GP2_PORT, GP2_BIT);
		
	UNB_setFreqC(parity);		//change frequency
	
	commandBuf[0] = CMD_CHANGE_STATE;
	commandBuf[1] = 0x05;//3;
	UNB_sendgetradio(commandBuf, 2, commandBuf, 0);
	
	commandBuf[0] = CMD_START_TX;
	commandBuf[1] = 0;
	commandBuf[2] = 0x50;//0x30;//
	commandBuf[3] = 0x00;//0;  0x01E0 = 480 bit
	commandBuf[4] = 0;//
	UNB_sendgetradio(commandBuf, 5, commandBuf, 0);
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//send two bits 1,0;
	//send 1
	for(uint8_t tackts = 0;tackts<40; tackts++)
	{
		// Wait for packet sent interrupt
		while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
		while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is high
		
	}
	//send 0
	for(uint8_t tackts = 0;tackts<40; tackts++)
	{
		// Wait for packet sent interrupt
		while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
		
		if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
		else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
		//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
		//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
		while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is high
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(uint32_t i = 0;i<3;i++)  //send preambula
	{
		uint8_t simbol = preambula[i];
		for(uint8_t bit = 0; bit<8;bit++)
		{
			
			uint8_t digit = simbol & (0x80 >> bit);
			for(uint8_t tackts = 0;tackts<40; tackts++)
			{
				// Wait for packet sent interrupt
				while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
				if(!digit)
				{
					
					if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
					//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
				}
								
				while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
				
			}
		}
	}
	battery_start_measure();
	for(uint32_t i = 0; i < ptr ;i++)  //send the rest of the message
	{
		uint8_t simbol = tx_packet[i];
		for(uint8_t bit = 0; bit<8;bit++)
		{
			uint8_t digit = simbol & (0x80 >> bit);
			
			for(uint8_t tackts = 0;tackts<40; tackts++)
			{
				
				// Wait for packet sent interrupt
				while (!((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
				if(!digit)
				{
					
					if((tackts > 2)&&(tackts < 5))GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
					//if(tackts < 2)GPIO_PinOutSet(GP2_PORT, GP2_BIT);
					//else GPIO_PinOutClear(GP2_PORT, GP2_BIT);
				}
				while (((GPIO->P[NIRQ_RF_PORT].DIN) & NIRQ_RF_MASK)); //NIRQ is low
			}
		}
	}
	
	commandBuf[0] = CMD_CHANGE_STATE;
	commandBuf[1] = 0x01;//3;
	UNB_sendgetradio(commandBuf, 2,commandBuf, 0);
	
	//for(uint32_t j = 0;j<100000;j++); //delay
	
}

uint8_t UNB_ProtocolC_Send(uint8_t *message, uint32_t serial)
{
	uint8_t complete=1;
	UNB_Tx_setup();
	UNB_ProtocolC_SendOneMsg(message, serial, 0);
	UNB_ProtocolC_SendOneMsg(message, serial, 1);
	iter++;
	UNB_Tx_shutdoun();
	HopChannel();
	return complete;
}

/*
* Send message to si4461 via SPI
* @param
* buffer - data buffer to send
* txlength - number of bites to send
*/
void UNB_sendgetradio(char * buffer,unsigned int txlength,char * rxbuffer,unsigned int rxlength)
{
	unsigned int i;
	char CTS;
	
	// Use interrupt status command
	GPIO->P[NSEL_RF_PORT].DOUTCLR = 1 << NSEL_RF_BIT;                                       // Select radio IC by pulling its nSEL pin low
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	for (i = 0; i < txlength; i++)
	{
		USART->TXDATA = buffer[i];
		while (!(USART->STATUS & USART_STATUS_TXC)) ;
	}
	while (!(USART->STATUS & USART_STATUS_TXC)) ;
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	
	if (*buffer != CMD_RX_FIFO_READ)
	{
		GPIO->P[NSEL_RF_PORT].DOUTSET = 1 << NSEL_RF_BIT;                                     // De-select radio IC by putting its nSEL pin high
		
		// Wait for CTS
		CTS = 0;
		USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
		while (CTS != 0xFF)                                                       			// Wait until radio IC is ready with the data
		{
			GPIO->P[NSEL_RF_PORT].DOUTCLR = 1 << NSEL_RF_BIT;                               // Select radio IC by pulling its nSEL pin low
			USART->TXDATA = CMD_CTS_READ;                                            		// Read command buffer; send command byte
			while (!(USART->STATUS & USART_STATUS_TXC)) ;
			USART->RXDATA;
			USART->TXDATA = 0;                                            					// Read command buffer; send command byte
			while (!(USART->STATUS & USART_STATUS_TXC)) ;
			CTS = USART->RXDATA;
			if (CTS != 0xFF)
			{
				GPIO->P[NSEL_RF_PORT].DOUTSET = 1 << NSEL_RF_BIT;                           // De-select radio IC by putting its nSEL pin high
			}
		}
	}
	
	//  Read
	for (i = 0; i < rxlength; i++)
	{
		USART->TXDATA = 0;
		while (!(USART->STATUS & USART_STATUS_TXC)) ;
		rxbuffer[i] = USART->RXDATA;
	}
	GPIO->P[NSEL_RF_PORT].DOUTSET = 1 << NSEL_RF_BIT;                                       // De-select radio IC by putting its nSEL pin high
}

#ifndef CRC_TABLE
#define WIDTH (8*4)
#define TOPBIT (1 << (WIDTH-1))
#define POLYNOMIAL (0x104C11DB7)
uint32_t crc_table(uint8_t n)
{
	uint32_t c;
	int k;
	c=((uint32_t)n) << (WIDTH - 8);
	for(k=8;k>0;k--)
	{
		if(c & TOPBIT)
		{
			c = (c<<1) ^ POLYNOMIAL;
		}
		else
		{
			c=c<<1;
		}
	}
	return c;
}
#endif

// Automatically generated CRC function
// polynomial: 0x104C11DB7
uint32_t digital_update_crc32( uint32_t crc, const uint8_t *data, uint8_t len )
{
#ifdef CRC_TABLE
	static const uint32_t crc_table[256] = {
		0x00000000U,0x04C11DB7U,0x09823B6EU,0x0D4326D9U,
		0x130476DCU,0x17C56B6BU,0x1A864DB2U,0x1E475005U,
		0x2608EDB8U,0x22C9F00FU,0x2F8AD6D6U,0x2B4BCB61U,
		0x350C9B64U,0x31CD86D3U,0x3C8EA00AU,0x384FBDBDU,
		0x4C11DB70U,0x48D0C6C7U,0x4593E01EU,0x4152FDA9U,
		0x5F15ADACU,0x5BD4B01BU,0x569796C2U,0x52568B75U,
		0x6A1936C8U,0x6ED82B7FU,0x639B0DA6U,0x675A1011U,
		0x791D4014U,0x7DDC5DA3U,0x709F7B7AU,0x745E66CDU,
		0x9823B6E0U,0x9CE2AB57U,0x91A18D8EU,0x95609039U,
		0x8B27C03CU,0x8FE6DD8BU,0x82A5FB52U,0x8664E6E5U,
		0xBE2B5B58U,0xBAEA46EFU,0xB7A96036U,0xB3687D81U,
		0xAD2F2D84U,0xA9EE3033U,0xA4AD16EAU,0xA06C0B5DU,
		0xD4326D90U,0xD0F37027U,0xDDB056FEU,0xD9714B49U,
		0xC7361B4CU,0xC3F706FBU,0xCEB42022U,0xCA753D95U,
		0xF23A8028U,0xF6FB9D9FU,0xFBB8BB46U,0xFF79A6F1U,
		0xE13EF6F4U,0xE5FFEB43U,0xE8BCCD9AU,0xEC7DD02DU,
		0x34867077U,0x30476DC0U,0x3D044B19U,0x39C556AEU,
		0x278206ABU,0x23431B1CU,0x2E003DC5U,0x2AC12072U,
		0x128E9DCFU,0x164F8078U,0x1B0CA6A1U,0x1FCDBB16U,
		0x018AEB13U,0x054BF6A4U,0x0808D07DU,0x0CC9CDCAU,
		0x7897AB07U,0x7C56B6B0U,0x71159069U,0x75D48DDEU,
		0x6B93DDDBU,0x6F52C06CU,0x6211E6B5U,0x66D0FB02U,
		0x5E9F46BFU,0x5A5E5B08U,0x571D7DD1U,0x53DC6066U,
		0x4D9B3063U,0x495A2DD4U,0x44190B0DU,0x40D816BAU,
		0xACA5C697U,0xA864DB20U,0xA527FDF9U,0xA1E6E04EU,
		0xBFA1B04BU,0xBB60ADFCU,0xB6238B25U,0xB2E29692U,
		0x8AAD2B2FU,0x8E6C3698U,0x832F1041U,0x87EE0DF6U,
		0x99A95DF3U,0x9D684044U,0x902B669DU,0x94EA7B2AU,
		0xE0B41DE7U,0xE4750050U,0xE9362689U,0xEDF73B3EU,
		0xF3B06B3BU,0xF771768CU,0xFA325055U,0xFEF34DE2U,
		0xC6BCF05FU,0xC27DEDE8U,0xCF3ECB31U,0xCBFFD686U,
		0xD5B88683U,0xD1799B34U,0xDC3ABDEDU,0xD8FBA05AU,
		0x690CE0EEU,0x6DCDFD59U,0x608EDB80U,0x644FC637U,
		0x7A089632U,0x7EC98B85U,0x738AAD5CU,0x774BB0EBU,
		0x4F040D56U,0x4BC510E1U,0x46863638U,0x42472B8FU,
		0x5C007B8AU,0x58C1663DU,0x558240E4U,0x51435D53U,
		0x251D3B9EU,0x21DC2629U,0x2C9F00F0U,0x285E1D47U,
		0x36194D42U,0x32D850F5U,0x3F9B762CU,0x3B5A6B9BU,
		0x0315D626U,0x07D4CB91U,0x0A97ED48U,0x0E56F0FFU,
		0x1011A0FAU,0x14D0BD4DU,0x19939B94U,0x1D528623U,
		0xF12F560EU,0xF5EE4BB9U,0xF8AD6D60U,0xFC6C70D7U,
		0xE22B20D2U,0xE6EA3D65U,0xEBA91BBCU,0xEF68060BU,
		0xD727BBB6U,0xD3E6A601U,0xDEA580D8U,0xDA649D6FU,
		0xC423CD6AU,0xC0E2D0DDU,0xCDA1F604U,0xC960EBB3U,
		0xBD3E8D7EU,0xB9FF90C9U,0xB4BCB610U,0xB07DABA7U,
		0xAE3AFBA2U,0xAAFBE615U,0xA7B8C0CCU,0xA379DD7BU,
		0x9B3660C6U,0x9FF77D71U,0x92B45BA8U,0x9675461FU,
		0x8832161AU,0x8CF30BADU,0x81B02D74U,0x857130C3U,
		0x5D8A9099U,0x594B8D2EU,0x5408ABF7U,0x50C9B640U,
		0x4E8EE645U,0x4A4FFBF2U,0x470CDD2BU,0x43CDC09CU,
		0x7B827D21U,0x7F436096U,0x7200464FU,0x76C15BF8U,
		0x68860BFDU,0x6C47164AU,0x61043093U,0x65C52D24U,
		0x119B4BE9U,0x155A565EU,0x18197087U,0x1CD86D30U,
		0x029F3D35U,0x065E2082U,0x0B1D065BU,0x0FDC1BECU,
		0x3793A651U,0x3352BBE6U,0x3E119D3FU,0x3AD08088U,
		0x2497D08DU,0x2056CD3AU,0x2D15EBE3U,0x29D4F654U,
		0xC5A92679U,0xC1683BCEU,0xCC2B1D17U,0xC8EA00A0U,
		0xD6AD50A5U,0xD26C4D12U,0xDF2F6BCBU,0xDBEE767CU,
		0xE3A1CBC1U,0xE760D676U,0xEA23F0AFU,0xEEE2ED18U,
		0xF0A5BD1DU,0xF464A0AAU,0xF9278673U,0xFDE69BC4U,
		0x89B8FD09U,0x8D79E0BEU,0x803AC667U,0x84FBDBD0U,
		0x9ABC8BD5U,0x9E7D9662U,0x933EB0BBU,0x97FFAD0CU,
		0xAFB010B1U,0xAB710D06U,0xA6322BDFU,0xA2F33668U,
		0xBCB4666DU,0xB8757BDAU,0xB5365D03U,0xB1F740B4U,
	};
#endif
	
	while (len > 0)
	{
#ifdef CRC_TABLE
		crc = crc_table[*data ^ ((crc >> 24) & 0xff)] ^ (crc << 8);
#else
		crc = crc_table(*data ^ ((crc >> 24) & 0xff)) ^ (crc << 8);
#endif
		data++;
		len--;
	}
	return crc;
}

uint32_t digital_crc32(const uint8_t *buf, uint8_t len)
{
	return digital_update_crc32(0xffffffff, buf, len) ^ 0xffffffff;
}


/*
* Setup the power of UNB transmition
*/

void UNB_setPower(UNB_power_typedef power){
	if(power == LowPower)for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_LowPower[i];
        else if(power == NormalPower)for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_NormalPower[i];
	else if(power == HighPower)for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_HighPower[i];
        else if(power == NormalPower_m3) for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_NormalPower_m3[i];
        else if(power == NormalPower_m6) for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_NormalPower_m6[i];
        else if(power == NormalPower_m9) for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_NormalPower_m9[i];
        else if(power == NormalPower_m12) for(int i = 0;i < UNB_SetPowerTx[0];i++)UNB_SetPowerTx[i] = UNB_NormalPower_m12[i];
}
/*
* return 1 if frequency set
* return 0 if not correct parameters
*/
#define MINIMUM_FREQUENCY 858500000
#define MAXIMUM_FREQUENCY 916700000
//#define MAXIMUM_FREQUENCY 870600000


int UNB_FrequencyConf(uint32_t freqLow, uint32_t freqHigh, int channels){
	double frequency;
	uint32_t integerPart = 0,fractionPart = 0;
	
	if((freqLow >= freqHigh)||(freqLow < MINIMUM_FREQUENCY)||(freqHigh > MAXIMUM_FREQUENCY))
	{
		//default initialization
		lowFreq  = FREQ868_775;
		highFreq =  FREQ868_825;
		numChannel = 100;
		return 0;
	}
	numChannel = channels;
	if(channels < 50)numChannel = 50;
	if(channels > 400)numChannel = 400;
	
	//low freq
	integerPart = (freqLow/13000000) - 1;
	frequency = (double)freqLow * (524288);
	frequency /=13000000;
	fractionPart = ((uint32_t)frequency - ((integerPart)*(524288)));
	lowFreq = (integerPart << 24) + (fractionPart & 0x00FFFFFF);
	
	//high freq
	integerPart = (freqHigh/13000000) - 1;
	frequency = (double)freqHigh * (524288);
	frequency /=13000000;
	fractionPart = (uint32_t)frequency - integerPart*(524288);
	highFreq = (integerPart << 24) + (fractionPart & 0x00FFFFFF);
	
	return 1;
}



