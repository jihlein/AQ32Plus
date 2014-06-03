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
// Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define RX_PULSE_1p5MS 3000  // 1.5 ms pulse width

///////////////////////////////////////////////////////////////////////////////
// PWM Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

static struct TIM_Channel { TIM_TypeDef *tim;
                            uint16_t    channel;
                            uint16_t    cc;
                          } Channels[] = { { TIM4, TIM_Channel_1, TIM_IT_CC1 },
                                           { TIM4, TIM_Channel_2, TIM_IT_CC2 },
                                           { TIM4, TIM_Channel_3, TIM_IT_CC3 },
                                           { TIM4, TIM_Channel_4, TIM_IT_CC4 },
                                           { TIM1, TIM_Channel_1, TIM_IT_CC1 },
                                           { TIM1, TIM_Channel_2, TIM_IT_CC2 },
                                           { TIM1, TIM_Channel_3, TIM_IT_CC3 },
                                           { TIM1, TIM_Channel_4, TIM_IT_CC4 }, };

pwmState_t Inputs[12] = { { 0, } };

static TIM_ICInitTypeDef  TIM_ICInitStructure;

///////////////////////////////////////////////////////////////////////////////
// PPM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

static void ppmRX_IRQHandler(TIM_TypeDef *tim)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t  chan = 0;

    if (TIM_GetITStatus(tim, TIM_IT_CC4) == SET)
    {
        last = now;
        now = TIM_GetCapture4(tim);
        rcActive = true;
        watchDogReset(rcDataLostCnt);
    }

    TIM_ClearITPendingBit(tim, TIM_IT_CC4);

    diff = now - last;

    if (diff > 2700 * 2)   // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
    {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
        chan = 0;          // you will be fine. I use 2.7ms just to be safe."
    }
    else
    {
        if (diff > 750 * 2 && diff < 2250 * 2 && chan < eepromConfig.ppmChannels)    // 750 to 2250 ms is our 'valid' channel range
        {
            Inputs[chan].pulseWidth = diff;
        }
        chan++;
    }
}

///////////////////////////////////////////////////////////////////////////////
// PWM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

static void pwmRX_IRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint32_t inputCaptureValue = 0;

    for (i = 0; i < 8; i++)  // don't use NUMCHANNELS here as AQ32 board is restricted to 8 parallel PWM inputs
    {
        struct TIM_Channel channel = Channels[i];
        struct pwmState    *state  = &Inputs[i];

        if (tim == channel.tim && (TIM_GetITStatus(channel.tim, channel.cc) == SET))
        {
            TIM_ClearITPendingBit(channel.tim, channel.cc);
            if (i == 0)
            {
            	rcActive = true;
            	watchDogReset(rcDataLostCnt);
            }

            switch (channel.channel)
            {
                case TIM_Channel_1:
                    inputCaptureValue = (uint16_t)TIM_GetCapture1(channel.tim);
                    break;
                case TIM_Channel_2:
                    inputCaptureValue = (uint16_t)TIM_GetCapture2(channel.tim);
                    break;
                case TIM_Channel_3:
                    inputCaptureValue = (uint16_t)TIM_GetCapture3(channel.tim);
                    break;
                case TIM_Channel_4:
                    inputCaptureValue = (uint16_t)TIM_GetCapture4(channel.tim);
                    break;
            }

            if (state->state == 0)
            {
                state->riseTime = inputCaptureValue;

                // switch states
                state->state = 1;

                TIM_ICInitStructure.TIM_Channel     = channel.channel;
                TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Falling;
              //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
              //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
              //TIM_ICInitStructure.TIM_ICFilter    = 0x00;

                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
            else
            {
                // inputCaptureValue has falling edge timer value

                // compute capture
                if (inputCaptureValue > state->riseTime)
                    state->pulseWidth = (inputCaptureValue - state->riseTime);
                else
                    state->pulseWidth = ((0xFFFF - state->riseTime) + inputCaptureValue);

                // switch state
                state->state = 0;

                TIM_ICInitStructure.TIM_Channel     = channel.channel;
                TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
              //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
              //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
              //TIM_ICInitStructure.TIM_ICFilter    = 0x00;

                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// PWM Receiver Interrupt Handlers
///////////////////////////////////////////////////////////////////////////////

void TIM4_IRQHandler(void)
{
    if (eepromConfig.receiverType == PPM)
        ppmRX_IRQHandler(TIM4);
    else
        pwmRX_IRQHandler(TIM4);
}

void TIM1_CC_IRQHandler(void)
{
    pwmRX_IRQHandler(TIM1);
}

///////////////////////////////////////////////////////////////////////////////
// Receiver Initialization
///////////////////////////////////////////////////////////////////////////////

void rxInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    USART_InitTypeDef        USART_InitStructure;

    uint8_t i;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_ICStructInit(&TIM_ICInitStructure);
    USART_StructInit(&USART_InitStructure);

    ///////////////////////////////////

    if (eepromConfig.receiverType == PPM)
    {
        // Serial PWM Input
    	// TIM4_CH4 PD15

        // preset channels to center
		for (i = 0; i < 12; i++)
		    Inputs[i].pulseWidth = RX_PULSE_1p5MS;

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

        GPIO_Init(GPIOD, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

        NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;
	  //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
	  //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

        TIM_ICInitStructure.TIM_Channel     = TIM_Channel_4;
      //TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
      //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
      //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
      //TIM_ICInitStructure.TIM_ICFilter    = 0x00;

        TIM_ICInit(TIM4, &TIM_ICInitStructure);

        TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
        TIM_Cmd(TIM4, ENABLE);
    }

    ///////////////////////////////////

    else if (eepromConfig.receiverType == PWM)
    {
        // Parallel PWM Inputs
    	// RX1  TIM4_CH1 PD12
    	// RX2  TIM4_CH2 PD13
    	// RX3  TIM4_CH3 PD14
    	// RX4  TIM4_CH4 PD15
    	// RX5  TIM1_CH1 PE9
    	// RX6  TIM1_CH2 PE11
    	// RX7  TIM1_CH3 PE13
    	// RX8  TIM1_CH4 PE14

		// preset channels to center
        for (i = 0; i < 8; i++)
            Inputs[i].pulseWidth = RX_PULSE_1p5MS;

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	    GPIO_Init(GPIOD, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;

        GPIO_Init(GPIOE, &GPIO_InitStructure);

    	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

        GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

        // Input timers on TIM4 and TIM1 for PWM
        NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	    //NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        //NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        // TIM4 and TIM1 timebase
        TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;
      //TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;
      //TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
      //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

        // Parallel PWM Input capture
      //TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
      //TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
      //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
      //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
      //TIM_ICInitStructure.TIM_ICFilter    = 0x00;

        for (i = 0; i < 8; i++)
        {
            TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
            TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
        }

        TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
        TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

        TIM_Cmd(TIM4, ENABLE);
        TIM_Cmd(TIM1, ENABLE);
	}

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Receiver Read
///////////////////////////////////////////////////////////////////////////////

float rxRead(uint8_t channel)
{
    return (float)Inputs[channel].pulseWidth;
}

///////////////////////////////////////////////////////////////////////////////


