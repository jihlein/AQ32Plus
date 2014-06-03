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

uint8_t rcActive = false;

uint32_t rcDataLostCnt;

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
//  Primary Spektrum Satellite Receiver UART Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void USART3_IRQHandler(void)
{
	uint8_t b;

	if (((USART3->CR1 & USART_CR1_RXNEIE) != 0) && ((USART3->SR & USART_SR_RXNE) != 0))
    {
    	b = USART_ReceiveData(USART3);

        if (eepromConfig.receiverType == SPEKTRUM)
        	spektrumParser(b, &primarySpektrumState, false);
        else
        	sBusParser(b);
    }
}

///////////////////////////////////////////////////////////////////////////////
