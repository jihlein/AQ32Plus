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
// Spektrum Satellite Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define MASTER_SPEKTRUM_UART_PIN       GPIO_Pin_9
#define MASTER_SPEKTRUM_UART_GPIO      GPIOD
#define MASTER_SPEKTRUM_UART_PINSOURCE GPIO_PinSource9

#define SLAVE_SPEKTRUM_UART_PIN        GPIO_Pin_1       // Dummy value for AQ32Plus
#define SLAVE_SPEKTRUM_UART_GPIO       GPIOA            // Dummy value for AQ32Plus
#define SLAVE_SPEKTRUM_UART_PINSOURCE  GPIO_PinSource1  // Dummy value for AQ32Plus

#define MASTER_BIND_COUNT   5
#define SLAVE_BIND_COUNT    6

#define SPEKTRUM_BIND_PIN          GPIO_Pin_14
#define SPEKTRUM_BIND_GPIO         GPIOE

#define MASTER_SPEKTRUM_BIND_PIN   GPIO_Pin_13
#define MASTER_SPEKTRUM_BIND_GPIO  GPIOA

///////////////////////////////////////

#define MAX_SPEKTRUM_CHANNELS       16

#define SPEKTRUM_FRAME_SIZE         16

///////////////////////////////////////

#define MIN_FRAME_SPACE 70  // 7.0 mSec
#define MAX_BYTE_SPACE   3  // 0.3 mSec

///////////////////////////////////////

spektrumStateType primarySpektrumState = {1,0,0,0,0,0,0,0,0,{0}};

spektrumStateType slaveSpektrumState   = {1,0,0,0,0,0,0,0,0,{0}};

int16_t spektrumBuf[SPEKTRUM_CHANNELS_PER_FRAME * MAX_SPEKTRUM_FRAMES];

static uint8_t encodingType   = 0;
static uint8_t expectedFrames = 0;

uint8_t  bestReceiver = 0;
uint8_t  channelCnt;
uint8_t  channelNum;
uint16_t channelData;
uint8_t  i;
uint8_t  maxChannelNum = 0;

uint8_t rcActive = false;

///////////////////////////////////////

uint32_t primarySpektrumFrameLostCnt;
uint32_t slaveSpektrumFrameLostCnt;

uint32_t rcDataLostCnt;

enum frameWatchDogConsts {
  rcDataLostTime         = 1000, // 1 second
  spektrumFrameLostTime  = 1000, // 1 second
  };

///////////////////////////////////////////////////////////////////////////////
// Decode Channels
//////////////////////////////////////////////////////////////////////////////

void decodeChannels(void)
{
	channelCnt = 0;

    // For every piece of channel data we have received

    for (i = 0; (i < SPEKTRUM_CHANNELS_PER_FRAME * expectedFrames); i++)
    {
        if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    	   	channelData = (!bestReceiver) ? primarySpektrumState.values[i] : slaveSpektrumState.values[i];
    	else
    	   	channelData = primarySpektrumState.values[i];

    	// Find out the channel number and its value by
        // using the EncodingType which is only received
        // from the main receiver

        switch(encodingType)
        {
            case(0) :  // 10 bit
                channelNum = (channelData >> 10) & 0x0F;

                // Don't bother decoding unused channels

                if (channelNum < 12)  //HJI RADIO_CONTROL_NB_CHANNEL)
                {
                    spektrumBuf[channelNum]  = channelData & 0x3FF;
                    spektrumBuf[channelNum]  = (spektrumBuf[channelNum] + 1000) << 1;
                    channelCnt++;
                }
                break;

            case(1) :  // 11 bit
                channelNum = (channelData >> 11) & 0x0F;

                // Don't bother decoding unused channels

                if (channelNum < 12) //HJI RADIO_CONTROL_NB_CHANNEL)
                {
                    spektrumBuf[channelNum]  = channelData & 0x7FF;
                    spektrumBuf[channelNum]  = spektrumBuf[channelNum] + 2000;
                    channelCnt++;
                }
                break;

            default :  // Never going to get here
            	channelNum = 0x0F;
            	break;
        }

        // Store the value of the highest valid channel

        if ((channelNum != 0x0F) && (channelNum > maxChannelNum))
            maxChannelNum = channelNum;
    }

    // Indicate valid RC data

    if (channelCnt >= (maxChannelNum + 1))
    {
        rcActive = true;
        watchDogReset(rcDataLostCnt);
    }
    else
    {
		rcActive = false;
	}
}

///////////////////////////////////////////////////////////////////////////////
//  Process SpektrumData
///////////////////////////////////////////////////////////////////////////////

void processSpektrumData(void)
{
    if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    {
    	// We have two receivers and at least one of them has new data

        // If both receivers have new data select the one
        // that has had the least number of frames lost

        if ((primarySpektrumState.rcAvailable) && (slaveSpektrumState.rcAvailable))
        {
            bestReceiver  = (primarySpektrumState.lostFrameCnt <= slaveSpektrumState.lostFrameCnt) ? 0 : 1;
        }
        else
        {
            // If only one of the receivers has new data use it
            bestReceiver  = (primarySpektrumState.rcAvailable) ? 0 : 1;
        }

        //  Clear the data ready flags
        primarySpektrumState.rcAvailable = 0;
        slaveSpektrumState.rcAvailable   = 0;

        decodeChannels();
    }
    else
    {
    	// We have one receiver and it has new data

        //  Clear the data ready flag
        primarySpektrumState.rcAvailable = 0;

        decodeChannels();
    }
}

///////////////////////////////////////////////////////////////////////////////
//  Spektrum Parser captures frame data by using time between frames to sync on
///////////////////////////////////////////////////////////////////////////////

static inline void spektrumParser(uint8_t c, spektrumStateType* spektrumState, bool slaveReceiver)
{
    uint16_t channelData;
	uint8_t  timedOut;
	static uint8_t tmpEncType   = 0;  // 0 = 10bit, 1 = 11 bit
	static uint8_t tmpExpFrames = 0;  // # of frames for channel data

    timedOut = (!spektrumState->spektrumTimer) ? 1 : 0;

    // If we have just started the resync process or
    // if we have recieved a character before our
    // 7ms wait has finished

    if ((spektrumState->reSync == 1) || ((spektrumState->sync == 0) && (!timedOut)))
    {
	    spektrumState->reSync = 0;
	    spektrumState->spektrumTimer = MIN_FRAME_SPACE;
	    spektrumState->sync = 0;
	    spektrumState->channelCnt = 0;
	    spektrumState->frameCnt = 0;
	    spektrumState->secondFrame = 0;
	    return;
    }

    // The first byte of a new frame. It was received
	// more than 7ms after the last received byte.
	// It represents the number of lost frames so far.

    if (spektrumState->sync == 0)
	{
	    spektrumState->lostFrameCnt = c;

        if (slaveReceiver)  // slave receiver
	        spektrumState->lostFrameCnt = spektrumState->lostFrameCnt << 8;

        spektrumState->sync = 1;
        spektrumState->spektrumTimer = MAX_BYTE_SPACE;
        return;
    }

    // All other bytes should be received within
	// MAX_BYTE_SPACE time of the last byte received
	// otherwise something went wrong resynchronise

    if (timedOut)
	{
	    spektrumState->reSync = 1;

	    // Next frame not expected sooner than 7ms

	    spektrumState->spektrumTimer = MIN_FRAME_SPACE;
	    return;
	}

    // Second character determines resolution and frame rate for main
	// receiver or low byte of LostFrameCount for slave receiver

	if (spektrumState->sync == 1)
	{
	    if (slaveReceiver)
	    {
	        spektrumState->lostFrameCnt +=c;
	        tmpExpFrames = expectedFrames;
	    }
	    else
	    {
	      // @todo collect more data. I suspect that there is a low res
	      // protocol that is still 10 bit but without using the full range.

	      tmpEncType   = (c & 0x10)>>4;  // 0 = 10bit, 1 = 11 bit
	      tmpExpFrames =  c & 0x03;      // 1 = 1 frame contains all channels
	                                     // 2 = 2 channel data in 2 frames
	    }

	    spektrumState->sync = 2;
	    spektrumState->spektrumTimer = MAX_BYTE_SPACE;
	    return;
	}

	// High byte of channel data if this is the first byte
	// of channel data and the most significant bit is set
	// then this is the second frame of channel data.

	if (spektrumState->sync == 2)
	{
	    spektrumState->highByte = c;

	    if (spektrumState->channelCnt == 0)
	    {
	        spektrumState->secondFrame = (spektrumState->highByte & 0x80) ? 1 : 0;
	    }

	    spektrumState->sync = 3;
	    spektrumState->spektrumTimer = MAX_BYTE_SPACE;
	    return;
	}

    // Low byte of channel data

    if (spektrumState->sync == 3)
    {
	    spektrumState->sync = 2;
	    spektrumState->spektrumTimer = MAX_BYTE_SPACE;

	    // We overwrite the buffer now so rc data is not available now

	    spektrumState->rcAvailable = 0;

	    channelData = ((uint16_t)spektrumState->highByte << 8) | c;

	    spektrumState->values[spektrumState->channelCnt + (spektrumState->secondFrame * 7)] = channelData;

	    spektrumState->channelCnt++;
    }

    // If we have a whole frame

    if(spektrumState->channelCnt >= SPEKTRUM_CHANNELS_PER_FRAME)
	{
        // How many frames did we expect ?
	    ++spektrumState->frameCnt;

	    if (spektrumState->frameCnt == tmpExpFrames)
	    {
	        // Set the rc_available_flag
	        spektrumState->rcAvailable = 1;
	        spektrumState->frameCnt    = 0;
	    }

	    if (!slaveReceiver)
	    {
		    // Main receiver
	        encodingType   = tmpEncType;    // Only update on a good
	        expectedFrames = tmpExpFrames;  // main receiver frame

	        watchDogReset(primarySpektrumFrameLostCnt);
	    }
	    else
	    {
			watchDogReset(slaveSpektrumFrameLostCnt);
		}

	    spektrumState->sync          = 0;
	    spektrumState->channelCnt    = 0;
	    spektrumState->secondFrame   = 0;
	    spektrumState->spektrumTimer = MIN_FRAME_SPACE;
    }

    if ((primarySpektrumState.rcAvailable) || (slaveSpektrumState.rcAvailable))
        processSpektrumData();
}

///////////////////////////////////////////////////////////////////////////////
//  RC Data Lost Handler
///////////////////////////////////////////////////////////////////////////////

void rcDataLost(void)
{
    evrPush(EVR_rcDataLost,0);

    // Maybe do something more interesting like auto-descent or hover-hold.
    // armed = false;
}

///////////////////////////////////////////////////////////////////////////////
//  Spektrum Frame Lost Handlers
///////////////////////////////////////////////////////////////////////////////

void primarySpektrumFrameLost(void)
{
    evrPush(EVR_primarySpektrumFrameLost,0);
}

void slaveSpektrumFrameLost(void)
{
    evrPush(EVR_slaveSpektrumFrameLost,0);
}

///////////////////////////////////////////////////////////////////////////////
//  Primary Spektrum Satellite Receiver UART Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void USART3_IRQHandler(void)
{
    if (((USART3->CR1 & USART_CR1_RXNEIE) != 0) && ((USART3->SR & USART_SR_RXNE) != 0))
    {
        uint8_t b = USART_ReceiveData(USART3);

        spektrumParser(b, &primarySpektrumState, false);
    }
}

///////////////////////////////////////////////////////////////////////////////
//  Slave Spektrum Satellite Receiver USART Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void UART4_IRQHandler(void)
{
    if (((UART4->CR1 & USART_CR1_RXNEIE) != 0) && ((UART4->SR & USART_SR_RXNE) != 0))
    {
        uint8_t b = USART_ReceiveData(UART4);

        spektrumParser(b, &slaveSpektrumState, true);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Timer 6 Interrupt Handler - Updates times used by Spektrum Parser
///////////////////////////////////////////////////////////////////////////////

void TIM6_DAC_IRQHandler(void)
{
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    if (primarySpektrumState.spektrumTimer)
        --primarySpektrumState.spektrumTimer;

    if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    {
	    if (slaveSpektrumState.spektrumTimer)
            --slaveSpektrumState.spektrumTimer;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Spektrum Initialization
///////////////////////////////////////////////////////////////////////////////

void spektrumInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    USART_InitTypeDef        USART_InitStructure;

    ///////////////////////////////////

	primarySpektrumState.reSync = 1;
    slaveSpektrumState.reSync   = 1;

    ///////////////////////////////////

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Period            = 100 - 1;             // 100 1 uSec Ticks
    TIM_TimeBaseStructure.TIM_Prescaler         = 84 - 1;              // 84 MHz / 84 = 1 MHz = 1 uSec Tick
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    TIM_Cmd(TIM6, ENABLE);

    ///////////////////////////////////

    // Turn off USART3 features used by DMA receive circular buffer

    USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);

    DMA_Cmd(DMA1_Stream1, DISABLE);

    DMA_DeInit(DMA1_Stream1);

    ///////////////////////////////////

    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(MASTER_SPEKTRUM_UART_GPIO, MASTER_SPEKTRUM_UART_PINSOURCE, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin   = MASTER_SPEKTRUM_UART_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(MASTER_SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  // Don't mistakenly clear TX mode for UART3
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);

    watchDogRegister(&primarySpektrumFrameLostCnt, spektrumFrameLostTime, primarySpektrumFrameLost, true );

    ///////////////////////////////////

    if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    {
		NVIC_InitStructure.NVIC_IRQChannel                   = UART4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        GPIO_PinAFConfig(SLAVE_SPEKTRUM_UART_GPIO, SLAVE_SPEKTRUM_UART_PINSOURCE, GPIO_AF_UART4);

        GPIO_InitStructure.GPIO_Pin   = SLAVE_SPEKTRUM_UART_PIN;
      //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

        GPIO_Init(SLAVE_SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

      //USART_InitStructure.USART_BaudRate            = 115200;
      //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      //USART_InitStructure.USART_Parity              = USART_Parity_No;
      //USART_InitStructure.USART_Mode                = USART_Mode_Rx;
      //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

        USART_Init(UART4, &USART_InitStructure);

        USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

        USART_Cmd(UART4, ENABLE);

        watchDogRegister(&slaveSpektrumFrameLostCnt, spektrumFrameLostTime, slaveSpektrumFrameLost, true );
	}

	///////////////////////////////////

	watchDogRegister(&rcDataLostCnt, rcDataLostTime, rcDataLost, true );
}

///////////////////////////////////////////////////////////////////////////////
// Spektrum Read
///////////////////////////////////////////////////////////////////////////////

uint16_t spektrumRead(uint8_t channel)
{
    return spektrumBuf[channel];
}

///////////////////////////////////////////////////////////////////////////////
// Check Spektrum Bind
///////////////////////////////////////////////////////////////////////////////

void checkSpektrumBind(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	uint8_t i;

	///////////////////////////////////

	// Configure bind pin as input
    GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_BIND_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(SPEKTRUM_BIND_GPIO, &GPIO_InitStructure);

    // Check bind pin state, if high (true), return without binding
    if (GPIO_ReadInputDataBit(SPEKTRUM_BIND_GPIO, SPEKTRUM_BIND_PIN) == true)
    	return;

    // Configure master Spektrum bind pin as output
    GPIO_InitStructure.GPIO_Pin   = MASTER_SPEKTRUM_BIND_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(MASTER_SPEKTRUM_BIND_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(MASTER_SPEKTRUM_BIND_GPIO, MASTER_SPEKTRUM_BIND_PIN);

    ///////////////////////////////////

    if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    {
        // Configure Slave UART pin as output
        GPIO_InitStructure.GPIO_Pin   = SLAVE_SPEKTRUM_UART_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(SLAVE_SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

        GPIO_SetBits(SLAVE_SPEKTRUM_UART_GPIO, SLAVE_SPEKTRUM_UART_PIN);
    }

    ///////////////////////////////////

    delay(60);

    for (i = 0; i < MASTER_BIND_COUNT; i++)
    {
    	GPIO_ResetBits(MASTER_SPEKTRUM_BIND_GPIO, MASTER_SPEKTRUM_BIND_PIN);
	    delayMicroseconds(120);
	    GPIO_SetBits(MASTER_SPEKTRUM_BIND_GPIO, MASTER_SPEKTRUM_BIND_PIN);
        delayMicroseconds(120);
	}

    ///////////////////////////////////

    if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on AQ32Plus
    {
        for (i = 0; i < SLAVE_BIND_COUNT; i++)
        {
	        GPIO_ResetBits(SLAVE_SPEKTRUM_UART_GPIO, SLAVE_SPEKTRUM_UART_PIN);
	        delayMicroseconds(120);
	        GPIO_SetBits(SLAVE_SPEKTRUM_UART_GPIO, SLAVE_SPEKTRUM_UART_PIN);
            delayMicroseconds(120);
	    }
    }

    ///////////////////////////////////

    // Configure master Spektrum bind pin as input
    GPIO_InitStructure.GPIO_Pin   = MASTER_SPEKTRUM_BIND_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(MASTER_SPEKTRUM_BIND_GPIO, &GPIO_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////


