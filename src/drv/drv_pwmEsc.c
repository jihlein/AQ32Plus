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
// PWM ESC Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define ESC_PULSE_1MS    2000  // 1ms pulse width
#define ESC_PULSE_PERIOD 5000  // pulse period (400Hz)

static volatile uint32_t *OutputChannels[] = { &(TIM8->CCR4),
	                                           &(TIM8->CCR3),
	                                           &(TIM8->CCR2),
	                                           &(TIM8->CCR1),
                                               &(TIM2->CCR2),
                                               &(TIM3->CCR1),
                                               &(TIM3->CCR2),
                                               &(TIM2->CCR1), };

///////////////////////////////////////////////////////////////////////////////
// PWM ESC Initialization
///////////////////////////////////////////////////////////////////////////////

void pwmEscInit(uint16_t escPwmRate)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);

    // Outputs
    // ESC PWM1 TIM8_CH4 PC9
    // ESC PWM2 TIM8_CH3 PC8
    // ESC PWM3 TIM8_CH2 PC7
    // ESC PWM4 TIM8_CH1 PC6
    // ESC PWM5 TIM2_CH2 PB3
    // ESC PWM6 TIM3_CH1 PB4
    // ESC PWM7 TIM3_CH2 PB5
    // ESC PWM8 TIM2_CH1 PA15

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,  ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

 	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

 	GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

 	GPIO_Init(GPIOC, &GPIO_InitStructure);

 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,  GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_TIM3);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,  GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,  GPIO_AF_TIM8);

    // Output timers

    TIM_TimeBaseStructure.TIM_Period            = (uint16_t)(2000000 / escPwmRate) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;
  //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
  //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepititionCounter = 0x0000;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = ESC_PULSE_1MS;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
  //TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

}

///////////////////////////////////////////////////////////////////////////////
// PWM ESC Write
///////////////////////////////////////////////////////////////////////////////

void pwmEscWrite(uint8_t channel, uint16_t value)
{
    *OutputChannels[channel] = value;
}

///////////////////////////////////////////////////////////////////////////////
