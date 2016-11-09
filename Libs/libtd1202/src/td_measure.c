/***************************************************************************//**
 * @file
 * @brief Temperature/Supply Voltage measure API for the TD1202 module.
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

#include <td1202.h>

#include <em_chip.h>
#include <em_emu.h>
#include <em_vcmp.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_adc.h>

#include "td_rtc.h"
#include "td_measure.h"

/***************************************************************************//**
 * @addtogroup MEASURE
 * @brief Temperature/Supply Voltage measure API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert ADC sample values to Celsius.
 *
 * @note
 *   See section 2.3.4 in the reference manual for details on this calculation.
 *
 * @param adcSample
 *   Raw value from ADC to be converted to Celsius.
 *
 * @return
 *   The temperature in degrees Celsius.
 ******************************************************************************/
static float convertToCelsius(int32_t adcSample)
{
  /* Factory calibration temperature from device information page. */
  int32_t cal_temp_0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);

  /* Factory calibration value from device information page. */
  int32_t cal_value_0 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;
  float temp;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));

  return temp;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Measure Power Supply voltage or the temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply voltage.
 *
 * @return
 *   The measured temperature in degrees Celsius, or the power supply voltage
 *   in 10s of mV plus 2 V if MSB is 0, plus 3 V if MSB is 1.
 */
uint8_t TD_MEASURE_VoltageTemperature(bool mode)
{
  uint32_t setpoint;

  /* Base the ADC configuration on the default setup. */
  ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
  ADC_Init_TypeDef init  = ADC_INIT_DEFAULT;

  /* Initialize timebase */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(40000, 0);
  CMU_ClockEnable(cmuClock_ADC0, true);
  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  single_init.reference = adcRef1V25;
//  single_init.resolution = adcRes8Bit;
  single_init.input   	= mode? adcSingleInpTemp: adcSingleInpVDDDiv3;
  ADC_InitSingle(ADC0, &single_init);

  // Start one ADC sample
  ADC_Start(ADC0, adcStartSingle);

  // Active wait for ADC to complete
  while ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0) {
	  ;
  }

  setpoint = ADC_DataSingleGet(ADC0);

  if (mode) {
      setpoint = (uint32_t) convertToCelsius(setpoint);
  } else {
	  setpoint = (setpoint * (125000 * 3 / 4096));                              // based in uV
      if (setpoint >= 300000) {
    	  setpoint = 0x80 + ((setpoint - 300000) / 1000);        				// msb = 1 -> 3V + n*(10mv)
      } else {
    	  setpoint = 0x00 + ((setpoint - 200000) / 1000);                       // msb = 0 -> 2V + n*(10mv)
      }
  }

  CMU_ClockEnable(cmuClock_ADC0, false);
  return(setpoint);
}

/** @} */

/** @} (end addtogroup MEASURE) */
