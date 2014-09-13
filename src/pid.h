/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

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

// PID Variables
typedef struct PIDdata {
  float   P, I, D, Limit;
  float   integratorState;
  float   filterState;
  uint8_t prevResetState;
} PIDdata_t;

extern uint8_t pidReset;

///////////////////////////////////////////////////////////////////////////////

void initPID(void);

///////////////////////////////////////////////////////////////////////////////

float updatePID(float error, float deltaT, uint8_t reset, struct PIDdata *PIDparameters);

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value);

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void);

///////////////////////////////////////////////////////////////////////////////



