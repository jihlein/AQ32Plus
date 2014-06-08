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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

struct pwmState
{
	uint8_t  state;          // 0 = looking for rising edge, 1 = looking for falling edge
    uint16_t riseTime;       // Timer value at rising edge of pulse
    uint16_t pulseWidth;     // Computed pulse width
};

typedef struct pwmState pwmState_t;

extern pwmState_t Inputs[12];

///////////////////////////////////////////////////////////////////////////////
// Receiver Initialization
///////////////////////////////////////////////////////////////////////////////

void rxInit(void);

///////////////////////////////////////////////////////////////////////////////
// Receiver Read
///////////////////////////////////////////////////////////////////////////////

float rxRead(uint8_t channel);

///////////////////////////////////////////////////////////////////////////////
