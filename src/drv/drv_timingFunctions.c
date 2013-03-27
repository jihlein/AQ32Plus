/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Timing Functions Initialization
///////////////////////////////////////////////////////////////////////////////

void timingFunctionsInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,  ENABLE);

    // Output timers

    TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;              // Just roll over counter at max value
    TIM_TimeBaseStructure.TIM_Prescaler         = 84 - 1;              // 84 MHz / 84 = 1 MHz = 1 uSec Tick
  //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepititionCounter = 0x0000;

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM6, ENABLE);

    ///////////////////////////////////

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

    TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;              // Just roll over counter at max value
    TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;              // 84 MHz / 42 = 2 MHz = 0.5 uSec Tick
  //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

    TIM_SetCounter(TIM10, 0);
    TIM_SetCounter(TIM11, 0);
}

///////////////////////////////////////////////////////////////////////////////
// delayTimerMicroseconds
///////////////////////////////////////////////////////////////////////////////

void delayTimerMicroseconds(uint16_t usec)
{
	TIM6->CNT = 0;

	while (TIM6->CNT <= usec) { }
}

///////////////////////////////////////////////////////////////////////////////
// delayTimerMilliseconds
///////////////////////////////////////////////////////////////////////////////

void delayTimerMilliseconds(uint32_t msec)
{
	while (msec--)
		delayTimerMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////
