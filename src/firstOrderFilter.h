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

#define NUMBER_OF_FIRST_ORDER_FILTERS 10

#define ACCEL500HZ_X_LOWPASS 0
#define ACCEL500HZ_Y_LOWPASS 1
#define ACCEL500HZ_Z_LOWPASS 2

#define ACCEL100HZ_X_LOWPASS 3
#define ACCEL100HZ_Y_LOWPASS 4
#define ACCEL100HZ_Z_LOWPASS 5

#define PRESSURE_ALT_LOWPASS 6

#define EARTH_AXIS_ACCEL_X_HIGHPASS 7
#define EARTH_AXIS_ACCEL_Y_HIGHPASS 8
#define EARTH_AXIS_ACCEL_Z_HIGHPASS 9

///////////////////////////////////////////////////////////////////////////////

typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter();

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters);

///////////////////////////////////////////////////////////////////////////////



