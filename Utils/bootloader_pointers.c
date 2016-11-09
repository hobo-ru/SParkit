
#include <efm32.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_timer.h>
#include <em_gpio.h>
#include <em_chip.h>
#include <em_wdog.h>
#include <em_rmu.h>

#include <td_flash.h>
#include <td1202.h>
#include <td_rtc.h>
#include <td_gpio.h>
#include <td_cmu.h>
#include <td_lan.h>
#include "td_rf_Si4461.h"
#include "td_rf.h"

#include <Protocol.h>
#include <commands.h>
#include <i2cEEPROM.h>
#include <flash.h>
#include "nwave_common.h"
#include "bootloader_pointers.h"

__ramfunc __noreturn void BOOT_boot(void);

void BootloaderPointers_Save(void)
{
	InitFlash(TD_LAN_RELEASE_PTR);

    INT_Disable();
    FlashInit();

    //Erase memory to enable writing
    ErasePage(TD_LAN_RELEASE_PTR);

    INT_Enable();
	
	WriteWordFlash((uint32_t)TD_CMU_Init,			TD_CMU_INIT_PTR);
	WriteWordFlash((uint32_t)BOOT_boot,				BOOT_BOOT_PTR);
	WriteWordFlash((uint32_t)RTC_Init,				RTC_INIT_PTR);
	WriteWordFlash((uint32_t)TD_GPIO_Init,			TD_GPIO_INIT_PTR);
	WriteWordFlash((uint32_t)TD_LAN_Init,			TD_LAN_INIT_PTR);
	WriteWordFlash((uint32_t)TD_LAN_ReceiveFrame,	TD_LAN_RECEIVEFRAME_PTR);
	WriteWordFlash((uint32_t)TD_LAN_SendFrame,		TD_LAN_SENDFRAME_PTR);
	WriteWordFlash((uint32_t)TD_LAN_Release,		TD_LAN_RELEASE_PTR);
	FlushBuffer();
}