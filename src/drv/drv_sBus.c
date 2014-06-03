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
// SBUS Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SBUS_UART_PIN       GPIO_Pin_9
#define SBUS_UART_GPIO      GPIOD
#define SBUS_UART_PINSOURCE GPIO_PinSource9

#define SBUS_INVERTER_PIN   GPIO_Pin_12
#define SBUS_INVERTER_GPIO  GPIOB

#define SBUS_SYNCBYTE 0x0F
#define SBUS_ENDBYTE  0x00

///////////////////////////////////////

uint16_t sBusChannel[12];

uint8_t sbusIndex        =  0;
uint8_t sbusPacketLength = 24;

uint8_t sbus[25];

///////////////////////////////////////

uint32_t sBusFrameLostCnt;

///////////////////////////////////////////////////////////////////////////////
//  SBUS Parser captures frame data by using time between frames to sync on
///////////////////////////////////////////////////////////////////////////////

void sBusParser(uint8_t c)
{
	uint8_t i;
	uint8_t shiftIndex;

	if(sbusIndex == 0 && c != SBUS_SYNCBYTE) return; // search for the beginning of a packet

	sbus[sbusIndex] = c;

	if (sbusIndex == sbusPacketLength)  // we have a full packet
	{
		if (c != SBUS_ENDBYTE)  // out of sync incorrect end byte
		{
			shiftIndex = 0;

			for(i = 1; i <= sbusIndex; i++)  // start at array pos 2 because we already know the byte at pos 1 is a syncByte
			{
				if(sbus[i] == SBUS_SYNCBYTE)
				{
					shiftIndex = i;
					break; //we have the location of the next SYNCBYTE
				}
			}

			if (shiftIndex != 0)   // the start of a packet was found in the middle of the bad packet
			{
				// shift everything by the value of -shiftIndex

				for( i = 0; i <= sbusPacketLength-shiftIndex; i++)
				    sbus[i] = sbus[i + shiftIndex];

				// reset the sbusIndex to the next location

				sbusIndex = sbusIndex - shiftIndex;
				sbusIndex++;
			}
			else  // no packet start was found in the middle of the bad packet
			{
			    sbusIndex = 0; // clear the packet buffer
			}
		}
		else  // everything is OK as my end byte and sync byte are correct
		{
			sBusChannel[XAXIS]	  = ((sbus[ 1]      | sbus[ 2] << 8)                   & 0x07FF);
			sBusChannel[YAXIS]	  = ((sbus[ 2] >> 3 | sbus[ 3] << 5)                   & 0x07FF);
			sBusChannel[THROTTLE] = ((sbus[ 3] >> 6 | sbus[ 4] << 2  | sbus[ 5] << 10) & 0x07FF);
			sBusChannel[ZAXIS]	  = ((sbus[ 5] >> 1 | sbus[ 6] << 7)                   & 0x07FF);
			sBusChannel[AUX1]	  = ((sbus[ 6] >> 4 | sbus[ 7] << 4)                   & 0x07FF);
			sBusChannel[AUX2]	  = ((sbus[ 7] >> 7 | sbus[ 8] << 1  | sbus[ 9] <<  9) & 0x07FF);
			sBusChannel[AUX3]	  = ((sbus[ 9] >> 2 | sbus[10] << 6)                   & 0x07FF);
			sBusChannel[AUX4]	  = ((sbus[10] >> 5 | sbus[11] << 3)                   & 0x07FF);
		  //sbusChannel[AUX5]	  = ((sbus[12]      | sbus[13] << 8)                   & 0x07FF);
		  //sbusChannel[AUX6]	  = ((sbus[13] >> 3 | sbus[14] << 5)                   & 0x07FF);
		  //sbusChannel[AUX7]	  = ((sbus[14] >> 6 | sbus[15] << 2  | sbus[16] << 10) & 0x07FF);
		  //sbusChannel[AUX8]	  = ((sbus[16] >> 1 | sbus[17] << 7)                   & 0x07FF);

		    sbusIndex = 0; // clear the packet buffer

		    rcActive = true;
		    watchDogReset(rcDataLostCnt);
		}
	}
	else  // we have a partial packet - keep calm and carry on
	{
		sbusIndex++;
	}
}

///////////////////////////////////////////////////////////////////////////////
//  SBUS Frame Lost Handler
///////////////////////////////////////////////////////////////////////////////

void sBusFrameLost(void)
{
    evrPush(EVR_sBusFrameLost,0);
}

///////////////////////////////////////////////////////////////////////////////
// SBUS Initialization
///////////////////////////////////////////////////////////////////////////////

void sBusInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    USART_InitTypeDef        USART_InitStructure;

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

    ///////////////////////////////////

    GPIO_InitStructure.GPIO_Pin   = SBUS_INVERTER_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(SBUS_INVERTER_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(SBUS_INVERTER_GPIO, SBUS_INVERTER_PIN);

    ///////////////////////////////////

    GPIO_PinAFConfig(SBUS_UART_GPIO, SBUS_UART_PINSOURCE, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin   = SBUS_UART_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(SBUS_UART_GPIO, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 100000;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  // Don't mistakenly clear TX mode for UART3
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);

    watchDogRegister(&sBusFrameLostCnt, rcFrameLostTime, sBusFrameLost, true );

	watchDogRegister(&rcDataLostCnt, rcDataLostTime, rcDataLost, true );
}

///////////////////////////////////////////////////////////////////////////////
// SBUS Read
///////////////////////////////////////////////////////////////////////////////

float sBusRead(uint8_t channel)
{
    return (float)sBusChannel[channel];
}

///////////////////////////////////////////////////////////////////////////////


