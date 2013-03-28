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
// PWM Servo Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SERVO_PULSE_1p5MS           3000  // 1.5 ms pulse width

static volatile uint32_t *OutputChannels[] = { &(TIM5->CCR3),
	                                           &(TIM5->CCR2),
	                                           &(TIM5->CCR1), };

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Initialization
///////////////////////////////////////////////////////////////////////////////

void pwmServoInit(uint16_t servoPwmRate)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);

    // Outputs
    // SERVO PWM0 TIM5 CH3 PA2
    // SERVO PWM1 TIM5 CH2 PA1
    // SERVO PWM2 TIM5 CH1 PA0

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

    // Output timers

    TIM_TimeBaseStructure.TIM_Period            = (uint16_t)(2000000 / servoPwmRate) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;
  //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepititionCounter = 0x0000;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIMOutputNState  = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = SERVO_PULSE_1p5MS;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
  //TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);

    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);

}

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Write
///////////////////////////////////////////////////////////////////////////////

void pwmServoWrite(uint8_t channel, uint16_t value)
{
    *OutputChannels[channel] = value;
}

///////////////////////////////////////////////////////////////////////////////
