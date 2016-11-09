/*
 * timer.h
 *
 *  Created on: 14 марта 2014 г.
 *      Author: vdubikhin
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdlib.h>
#include <stdint.h>

void initTimer(); //initialize timers interrupt
void setTimer(); //set timer cmp value
void delayTimer0(); //delay via Timer0
void delayVariableTimer0(uint32_t delay); //variable delay via Timer0
void initTimer0PWM();
void disableTimer0PWM();


extern uint32_t deviceSerial;

#endif /* TIMER_H_ */
