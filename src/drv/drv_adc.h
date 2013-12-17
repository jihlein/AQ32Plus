/*
  March 2013

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

#pragma once

///////////////////////////////////////////////////////////////////////////////

extern float accelSum100HzMXR[3];

extern float accelSum500HzMXR[3];

extern float accelSummedSamples100HzMXR[3];

extern float accelSummedSamples500HzMXR[3];

///////////////////////////////////////

#define ADC_PIN_1    0
#define ADC_PIN_2    1
#define ADC_PIN_3    2
#define ADC_PIN_4    3
#define ADC_PIN_5    4
#define ADC_PIN_6    5
#define ADC_PIN_7    6
#define ADC_PIN_NONE -1

///////////////////////////////////////

#define VOLTS_PER_BIT   (3.3f / 4096.0f)

///////////////////////////////////////////////////////////////////////////////
//  ADC Initialization
///////////////////////////////////////////////////////////////////////////////

void adcInit(void);

///////////////////////////////////////////////////////////////////////////////
//  Compute and return an ADC pin value
///////////////////////////////////////////////////////////////////////////////

float adcValue(uint8_t pin);

///////////////////////////////////////////////////////////////////////////////
