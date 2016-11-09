/*
 * bootloader_inapp.c
 *
 *  Created on: 7.10.2014
 *      Author: S.Omelchenko
 */

#include "bootloader_inapp.h"
#include <efm32.h>

/**************************************************************************//**
 * @brief This function sets up the Cortex M-3 with a new SP and PC.
 *****************************************************************************/
#if defined ( __CC_ARM   )
__asm void BOOT_jump(uint32_t sp, uint32_t pc)
{
  /* Set new MSP, PSP based on SP (r0)*/
  msr msp, r0
  msr psp, r0

  /* Jump to PC (r1)*/
  mov pc, r1
}
#else
__ramfunc __noreturn void BOOT_jump(uint32_t sp, uint32_t pc)
{
  (void) sp;
  (void) pc;
  /* Set new MSP, PSP based on SP (r0)*/
  __asm("msr msp, r0");
  __asm("msr psp, r0");

  /* Jump to PC (r1)*/
  __asm("mov pc, r1");
}
#endif

/**************************************************************************//**
 * @brief Boots the application
 *****************************************************************************/
__ramfunc __noreturn void BOOT_boot(uint32_t address)
{
  uint32_t pc, sp;

  /* Reset registers */
  /* Clear all interrupts set. */
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;
  RTC->CTRL  = _RTC_CTRL_RESETVALUE;
  RTC->COMP0 = _RTC_COMP0_RESETVALUE;
  RTC->IEN   = _RTC_IEN_RESETVALUE;
  /* Reset GPIO settings */
  GPIO->P[5].MODEL = _GPIO_P_MODEL_RESETVALUE;
  /* Disable RTC clock */
  CMU->LFACLKEN0 = _CMU_LFACLKEN0_RESETVALUE;
  CMU->LFCLKSEL  = _CMU_LFCLKSEL_RESETVALUE;
  /* Disable LFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_LFRCODIS;
  /* Disable LE interface */
  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
  /* Reset clocks */
  CMU->HFPERCLKDIV = _CMU_HFPERCLKDIV_RESETVALUE;
  CMU->HFPERCLKEN0 = _CMU_HFPERCLKEN0_RESETVALUE;

  /* Set new vector table */
  SCB->VTOR = (uint32_t) (address);//set vector offset, SP and PC back to bootloader memory
  /* Read new SP and PC from vector table */
  sp = *((uint32_t *) address);
  pc = *((uint32_t *) address + 1);

  BOOT_jump(sp, pc);
}
