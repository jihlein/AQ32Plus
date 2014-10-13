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

uint16_t RSSIRaw;
uint8_t RSSI = 0;

///////////////////////////////////////////////////////////////////////////////
//  RSSI Initialization
///////////////////////////////////////////////////////////////////////////////

void rssiInit(void)
{
    // nothing for now
}

///////////////////////////////////////////////////////////////////////////////
//  Measure RSSI
///////////////////////////////////////////////////////////////////////////////

void rssiMeasure(void)
{
	if (eepromConfig.rssiPPM)
	{
		RSSIRaw = (uint16_t)rxCommand[eepromConfig.rssiPin - 1]; // RX channels index from 0, ADC channels index from 1
	}
	else
	{
		RSSIRaw = (uint16_t)adcValue(eepromConfig.rssiPin);
	}

    if (RSSIRaw < eepromConfig.rssiMin)
	{
	    RSSI = 0;
	}
	else if (RSSIRaw > eepromConfig.rssiMax)
	{
	    RSSI = 100;
	}
	else
	{
	    RSSI = (float)(RSSIRaw - eepromConfig.rssiMin) / (float)(eepromConfig.rssiMax - eepromConfig.rssiMin) * 100.0f;
    }
}

///////////////////////////////////////////////////////////////////////////////
