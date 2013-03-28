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

uint8_t rcActive = false;

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

static struct PWM_State { uint8_t  state;          // 0 = looking for rising edge, 1 = looking for falling edge
                          uint16_t riseTime;       // Timer value at rising edge of pulse
                          uint16_t pulseWidth;     // Computed pulse width
                        } Inputs[8] = { { 0, } };

static TIM_ICInitTypeDef  TIM_ICInitStructure;

///////////////////////////////////////////////////////////////////////////////
// Spektrum Satellite Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SPEKTRUM_UART_PIN        GPIO_Pin_9
#define SPEKTRUM_UART_GPIO       GPIOD
#define SPEKTRUM_UART_PINSOURCE  GPIO_PinSource9
#define SPEKTRUM_BIND_PIN        GPIO_Pin_14
#define SPEKTRUM_BIND_GPIO       GPIOE

#define SPEKTRUM_FRAME_SIZE 16

uint8_t  i;
uint8_t  spektrumBindCount;

uint32_t spektrumChannelData[SPEKTRUM_MAX_CHANNEL];
uint8_t  spektrumChannelMask;
uint8_t  spektrumChannelShift;

uint8_t  spektrumFrame[SPEKTRUM_FRAME_SIZE];
bool     spektrumFrameComplete = false;
uint8_t  spektrumFramePosition;

uint32_t spektrumTimeInterval;
uint32_t spektrumTimeLast;

///////////////////////////////////////////////////////////////////////////////
// Serial PWM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

static void serialPWM_IRQHandler(TIM_TypeDef *tim)
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
    }

    TIM_ClearITPendingBit(tim, TIM_IT_CC4);

    diff = now - last;

    if (diff > 2700 * 2)   // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
    {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
        chan = 0;          // you will be fine. I use 2.7ms just to be safe."
    }
    else
    {
        if (diff > 750 * 2 && diff < 2250 * 2 && chan < 8)    // 750 to 2250 ms is our 'valid' channel range
        {
            Inputs[chan].pulseWidth = diff;
        }
        chan++;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Parallel PWM Receiver Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

static void parallelPWM_IRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint32_t inputCaptureValue = 0;

    for (i = 0; i < 8; i++) {
        struct TIM_Channel channel = Channels[i];
        struct PWM_State   *state  = &Inputs[i];

        if (tim == channel.tim && (TIM_GetITStatus(channel.tim, channel.cc) == SET))
        {
            TIM_ClearITPendingBit(channel.tim, channel.cc);
            if (i == 0)
                rcActive = true;

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
    if (eepromConfig.receiverType == SERIAL_PWM)
        serialPWM_IRQHandler(TIM4);
    else
        parallelPWM_IRQHandler(TIM4);
}

void TIM1_CC_IRQHandler(void)
{
    parallelPWM_IRQHandler(TIM1);
}

///////////////////////////////////////////////////////////////////////////////
//  Spektrum Satellite Receiver UART Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void USART3_IRQHandler(void)
{
    uint8_t  b;
    uint8_t  spektrumChannel;
    uint32_t spektrumTime;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        rcActive             = true;
        spektrumTime         = micros();
        spektrumTimeInterval = spektrumTime - spektrumTimeLast;
        spektrumTimeLast     = spektrumTime;

        if (spektrumTimeInterval > 5000)
            spektrumFramePosition = 0;

        spektrumFrame[spektrumFramePosition] = USART_ReceiveData(USART3);

        if (spektrumFramePosition == SPEKTRUM_FRAME_SIZE - 1)
        {
            spektrumFrameComplete = true;
            //failsafeCnt = 0;
        }
        else
        {
            spektrumFramePosition++;
        }

        if (spektrumFrameComplete)
		{
		    for (b = 3; b < SPEKTRUM_FRAME_SIZE; b += 2)
		    {
		        spektrumChannel = 0x0F & (spektrumFrame[b - 1] >> spektrumChannelShift);
		        if (spektrumChannel < eepromConfig.spektrumChannels)
		            spektrumChannelData[spektrumChannel] = ((uint32_t)(spektrumFrame[b - 1] & spektrumChannelMask) << 8) + spektrumFrame[b];
		    }

		    spektrumFrameComplete = false;
		}
    }
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

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_ICStructInit(&TIM_ICInitStructure);
    USART_StructInit(&USART_InitStructure);

    ///////////////////////////////////

    if (eepromConfig.receiverType == SERIAL_PWM)
    {
        // Serial PWM Input
    	// TIM4_CH4 PD15

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE);

		// preset channels to center
		for (i = 0; i < 8; i++)
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

    else if (eepromConfig.receiverType == PARALLEL_PWM)
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

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE);

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

	else if (eepromConfig.receiverType == SPEKTRUM)
	{
        // Spektrum Satellite RX Input
    	// USART3 RX P9

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

        NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_UART_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

        GPIO_Init(SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

    	GPIO_PinAFConfig(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PINSOURCE, GPIO_AF_USART3);

        USART_InitStructure.USART_BaudRate            = 115200;
      //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      //USART_InitStructure.USART_Parity              = USART_Parity_No;
        USART_InitStructure.USART_Mode                = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

        USART_Init(USART3, &USART_InitStructure);

        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART3, ENABLE);

        ///////////////////////////////

        if (eepromConfig.spektrumHires)
        {
		    // 11 bit frames
		    spektrumChannelShift = 3;
		    spektrumChannelMask  = 0x07;
		}
		else
		{
		    // 10 bit frames
		    spektrumChannelShift = 2;
		    spektrumChannelMask  = 0x03;
		}

        ///////////////////////////////
	}

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Receiver Read
///////////////////////////////////////////////////////////////////////////////

uint16_t rxRead(uint8_t channel)
{
    uint16_t data;

    if (eepromConfig.receiverType == SPEKTRUM)
    {
        if (channel >= eepromConfig.spektrumChannels)
    	{
    	    data = MINCOMMAND;
    	}
       	else
       	{
       	    if (eepromConfig.spektrumHires)
       	        data = 1000 + spektrumChannelData[channel];         // 2048 mode
       	    else
       	        data = (1000 + spektrumChannelData[channel]) << 1;  // 1024 mode
       	}
        return data;
    }
    else
    {
        return Inputs[channel].pulseWidth;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Check Spektrum Bind
///////////////////////////////////////////////////////////////////////////////

void checkSpektrumBind()
{
    // Spektrum Satellite RX Input
  	// USART1 RX PA10
	// Spektrum Satellite Bind Input
	// PE14

	GPIO_InitTypeDef  GPIO_InitStructure;

	uint8_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    ///////////////////////////////

    // Configure bind pin as input
    GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_BIND_PIN;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(SPEKTRUM_BIND_GPIO, &GPIO_InitStructure);

    // Check bind pin state, if high (true), return without binding
    if (GPIO_ReadInputDataBit(SPEKTRUM_BIND_GPIO, SPEKTRUM_BIND_PIN) == true)
    	return;

    if (eepromConfig.spektrumChannels <= 7)
        spektrumBindCount = 3;  // Master receiver with 7 or less channels
    else
        spektrumBindCount = 5;  // Master receiver with 8 or more channels

    // Configure UART pin as output
    GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_UART_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

    GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_SET);

    delay(60);

    for (i = 0; i < spektrumBindCount; i++)
    {
	    GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_RESET);
	    delayMicroseconds(120);
		GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_SET  );
        delayMicroseconds(120);
	}
}

///////////////////////////////////////////////////////////////////////////////


