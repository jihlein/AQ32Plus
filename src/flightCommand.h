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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define DEADBAND       24
#define DEADBAND_SLOPE 1000/(1000-DEADBAND)

extern float rxCommand[8];

extern uint8_t commandInDetent[3];
extern uint8_t previousCommandInDetent[3];

extern uint8_t channelOrder[8];

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t flightMode;

extern uint8_t headingHoldEngaged;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t armed;
extern uint8_t armingTimer;
extern uint8_t disarmingTimer;

///////////////////////////////////////////////////////////////////////////////
// Altitude Hold State Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t altitudeHoldState;

///////////////////////////////////////////////////////////////////////////////
// Process Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void);

///////////////////////////////////////////////////////////////////////////////
