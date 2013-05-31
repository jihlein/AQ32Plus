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

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////

#define ACCEL500HZ_X_LOWPASS_TAU         0.05f
#define ACCEL500HZ_X_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL500HZ_X_LOWPASS_A           (2.0f * ACCEL500HZ_X_LOWPASS_TAU / ACCEL500HZ_X_LOWPASS_SAMPLE_TIME)
#define ACCEL500HZ_X_LOWPASS_GX1         (1.0f / (1.0f + ACCEL500HZ_X_LOWPASS_A))
#define ACCEL500HZ_X_LOWPASS_GX2         (1.0f / (1.0f + ACCEL500HZ_X_LOWPASS_A))
#define ACCEL500HZ_X_LOWPASS_GX3         ((1.0f - ACCEL500HZ_X_LOWPASS_A) / (1.0f + ACCEL500HZ_X_LOWPASS_A))

///////////////////////////////////////

#define ACCEL500HZ_Y_LOWPASS_TAU         0.05f
#define ACCEL500HZ_Y_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL500HZ_Y_LOWPASS_A           (2.0f * ACCEL500HZ_Y_LOWPASS_TAU / ACCEL500HZ_Y_LOWPASS_SAMPLE_TIME)
#define ACCEL500HZ_Y_LOWPASS_GX1         (1.0f / (1.0f + ACCEL500HZ_Y_LOWPASS_A))
#define ACCEL500HZ_Y_LOWPASS_GX2         (1.0f / (1.0f + ACCEL500HZ_Y_LOWPASS_A))
#define ACCEL500HZ_Y_LOWPASS_GX3         ((1.0f - ACCEL500HZ_Y_LOWPASS_A) / (1.0f + ACCEL500HZ_Y_LOWPASS_A))

///////////////////////////////////////

#define ACCEL500HZ_Z_LOWPASS_TAU         0.05f
#define ACCEL500HZ_Z_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL500HZ_Z_LOWPASS_A           (2.0f * ACCEL500HZ_Z_LOWPASS_TAU / ACCEL500HZ_Z_LOWPASS_SAMPLE_TIME)
#define ACCEL500HZ_Z_LOWPASS_GX1         (1.0f / (1.0f + ACCEL500HZ_Z_LOWPASS_A))
#define ACCEL500HZ_Z_LOWPASS_GX2         (1.0f / (1.0f + ACCEL500HZ_Z_LOWPASS_A))
#define ACCEL500HZ_Z_LOWPASS_GX3         ((1.0f - ACCEL500HZ_Z_LOWPASS_A) / (1.0f + ACCEL500HZ_Z_LOWPASS_A))

///////////////////////////////////////

#define ACCEL100HZ_X_LOWPASS_TAU         0.05f
#define ACCEL100HZ_X_LOWPASS_SAMPLE_TIME 0.01f
#define ACCEL100HZ_X_LOWPASS_A           (2.0f * ACCEL100HZ_X_LOWPASS_TAU / ACCEL100HZ_X_LOWPASS_SAMPLE_TIME)
#define ACCEL100HZ_X_LOWPASS_GX1         (1.0f / (1.0f + ACCEL100HZ_X_LOWPASS_A))
#define ACCEL100HZ_X_LOWPASS_GX2         (1.0f / (1.0f + ACCEL100HZ_X_LOWPASS_A))
#define ACCEL100HZ_X_LOWPASS_GX3         ((1.0f - ACCEL100HZ_X_LOWPASS_A) / (1.0f + ACCEL100HZ_X_LOWPASS_A))

///////////////////////////////////////

#define ACCEL100HZ_Y_LOWPASS_TAU         0.05f
#define ACCEL100HZ_Y_LOWPASS_SAMPLE_TIME 0.01f
#define ACCEL100HZ_Y_LOWPASS_A           (2.0f * ACCEL100HZ_Y_LOWPASS_TAU / ACCEL100HZ_Y_LOWPASS_SAMPLE_TIME)
#define ACCEL100HZ_Y_LOWPASS_GX1         (1.0f / (1.0f + ACCEL100HZ_Y_LOWPASS_A))
#define ACCEL100HZ_Y_LOWPASS_GX2         (1.0f / (1.0f + ACCEL100HZ_Y_LOWPASS_A))
#define ACCEL100HZ_Y_LOWPASS_GX3         ((1.0f - ACCEL100HZ_Y_LOWPASS_A) / (1.0f + ACCEL100HZ_Y_LOWPASS_A))

///////////////////////////////////////

#define ACCEL100HZ_Z_LOWPASS_TAU         0.05f
#define ACCEL100HZ_Z_LOWPASS_SAMPLE_TIME 0.01f
#define ACCEL100HZ_Z_LOWPASS_A           (2.0f * ACCEL100HZ_Z_LOWPASS_TAU / ACCEL100HZ_Z_LOWPASS_SAMPLE_TIME)
#define ACCEL100HZ_Z_LOWPASS_GX1         (1.0f / (1.0f + ACCEL100HZ_Z_LOWPASS_A))
#define ACCEL100HZ_Z_LOWPASS_GX2         (1.0f / (1.0f + ACCEL100HZ_Z_LOWPASS_A))
#define ACCEL100HZ_Z_LOWPASS_GX3         ((1.0f - ACCEL100HZ_Z_LOWPASS_A) / (1.0f + ACCEL100HZ_Z_LOWPASS_A))

///////////////////////////////////////

#define PRESSURE_ALT_LOWPASS_TAU         0.05f
#define PRESSURE_ALT_LOWPASS_SAMPLE_TIME 0.02f
#define PRESSURE_ALT_LOWPASS_A           (2.0f * PRESSURE_ALT_LOWPASS_TAU / PRESSURE_ALT_LOWPASS_SAMPLE_TIME)
#define PRESSURE_ALT_LOWPASS_GX1         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX2         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX3         ((1.0f - PRESSURE_ALT_LOWPASS_A) / (1.0f + PRESSURE_ALT_LOWPASS_A))

///////////////////////////////////////

#define EARTH_AXIS_ACCEL_X_HIGHPASS_TAU         4.00f
#define EARTH_AXIS_ACCEL_X_HIGHPASS_SAMPLE_TIME 0.01f
#define EARTH_AXIS_ACCEL_X_HIGHPASS_A           (2.0f * EARTH_AXIS_ACCEL_X_HIGHPASS_TAU / EARTH_AXIS_ACCEL_X_HIGHPASS_SAMPLE_TIME)
#define EARTH_AXIS_ACCEL_X_HIGHPASS_GX1         ( EARTH_AXIS_ACCEL_X_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_X_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_X_HIGHPASS_GX2         (-EARTH_AXIS_ACCEL_X_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_X_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_X_HIGHPASS_GX3         ((1.0f - EARTH_AXIS_ACCEL_X_HIGHPASS_A) / (1.0f + EARTH_AXIS_ACCEL_X_HIGHPASS_A))

///////////////////////////////////////

#define EARTH_AXIS_ACCEL_Y_HIGHPASS_TAU         4.00f
#define EARTH_AXIS_ACCEL_Y_HIGHPASS_SAMPLE_TIME 0.01f
#define EARTH_AXIS_ACCEL_Y_HIGHPASS_A           (2.0f * EARTH_AXIS_ACCEL_Y_HIGHPASS_TAU / EARTH_AXIS_ACCEL_Y_HIGHPASS_SAMPLE_TIME)
#define EARTH_AXIS_ACCEL_Y_HIGHPASS_GX1         ( EARTH_AXIS_ACCEL_Y_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_Y_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_Y_HIGHPASS_GX2         (-EARTH_AXIS_ACCEL_Y_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_Y_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_Y_HIGHPASS_GX3         ((1.0f - EARTH_AXIS_ACCEL_Y_HIGHPASS_A) / (1.0f + EARTH_AXIS_ACCEL_Y_HIGHPASS_A))

///////////////////////////////////////

#define EARTH_AXIS_ACCEL_Z_HIGHPASS_TAU         4.00f
#define EARTH_AXIS_ACCEL_Z_HIGHPASS_SAMPLE_TIME 0.01f
#define EARTH_AXIS_ACCEL_Z_HIGHPASS_A           (2.0f * EARTH_AXIS_ACCEL_Z_HIGHPASS_TAU / EARTH_AXIS_ACCEL_Z_HIGHPASS_SAMPLE_TIME)
#define EARTH_AXIS_ACCEL_Z_HIGHPASS_GX1         ( EARTH_AXIS_ACCEL_Z_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_Z_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_Z_HIGHPASS_GX2         (-EARTH_AXIS_ACCEL_Z_HIGHPASS_A / (1.0f + EARTH_AXIS_ACCEL_Z_HIGHPASS_A))
#define EARTH_AXIS_ACCEL_Z_HIGHPASS_GX3         ((1.0f - EARTH_AXIS_ACCEL_Z_HIGHPASS_A) / (1.0f + EARTH_AXIS_ACCEL_Z_HIGHPASS_A))

///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter()
{
    firstOrderFilters[ACCEL500HZ_X_LOWPASS].gx1 = ACCEL500HZ_X_LOWPASS_GX1;
    firstOrderFilters[ACCEL500HZ_X_LOWPASS].gx2 = ACCEL500HZ_X_LOWPASS_GX2;
    firstOrderFilters[ACCEL500HZ_X_LOWPASS].gx3 = ACCEL500HZ_X_LOWPASS_GX3;
    firstOrderFilters[ACCEL500HZ_X_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL500HZ_X_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL500HZ_Y_LOWPASS].gx1 = ACCEL500HZ_Y_LOWPASS_GX1;
	firstOrderFilters[ACCEL500HZ_Y_LOWPASS].gx2 = ACCEL500HZ_Y_LOWPASS_GX2;
	firstOrderFilters[ACCEL500HZ_Y_LOWPASS].gx3 = ACCEL500HZ_Y_LOWPASS_GX3;
	firstOrderFilters[ACCEL500HZ_Y_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL500HZ_Y_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL500HZ_Z_LOWPASS].gx1 = ACCEL500HZ_Z_LOWPASS_GX1;
	firstOrderFilters[ACCEL500HZ_Z_LOWPASS].gx2 = ACCEL500HZ_Z_LOWPASS_GX2;
	firstOrderFilters[ACCEL500HZ_Z_LOWPASS].gx3 = ACCEL500HZ_Z_LOWPASS_GX3;
	firstOrderFilters[ACCEL500HZ_Z_LOWPASS].previousInput  = -accelOneG;
    firstOrderFilters[ACCEL500HZ_Z_LOWPASS].previousOutput = -accelOneG;

    ///////////////////////////////////

    firstOrderFilters[ACCEL100HZ_X_LOWPASS].gx1 = ACCEL100HZ_X_LOWPASS_GX1;
    firstOrderFilters[ACCEL100HZ_X_LOWPASS].gx2 = ACCEL100HZ_X_LOWPASS_GX2;
    firstOrderFilters[ACCEL100HZ_X_LOWPASS].gx3 = ACCEL100HZ_X_LOWPASS_GX3;
    firstOrderFilters[ACCEL100HZ_X_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL100HZ_X_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL100HZ_Y_LOWPASS].gx1 = ACCEL100HZ_Y_LOWPASS_GX1;
	firstOrderFilters[ACCEL100HZ_Y_LOWPASS].gx2 = ACCEL100HZ_Y_LOWPASS_GX2;
	firstOrderFilters[ACCEL100HZ_Y_LOWPASS].gx3 = ACCEL100HZ_Y_LOWPASS_GX3;
	firstOrderFilters[ACCEL100HZ_Y_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL100HZ_Y_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL100HZ_Z_LOWPASS].gx1 = ACCEL100HZ_Z_LOWPASS_GX1;
	firstOrderFilters[ACCEL100HZ_Z_LOWPASS].gx2 = ACCEL100HZ_Z_LOWPASS_GX2;
	firstOrderFilters[ACCEL100HZ_Z_LOWPASS].gx3 = ACCEL100HZ_Z_LOWPASS_GX3;
	firstOrderFilters[ACCEL100HZ_Z_LOWPASS].previousInput  = -accelOneG;
    firstOrderFilters[ACCEL100HZ_Z_LOWPASS].previousOutput = -accelOneG;

    ///////////////////////////////////

    firstOrderFilters[PRESSURE_ALT_LOWPASS].gx1 = PRESSURE_ALT_LOWPASS_GX1;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].gx2 = PRESSURE_ALT_LOWPASS_GX2;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].gx3 = PRESSURE_ALT_LOWPASS_GX3;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].previousInput  = sensors.pressureAlt50Hz;
    firstOrderFilters[PRESSURE_ALT_LOWPASS].previousOutput = sensors.pressureAlt50Hz;

    ///////////////////////////////////

    firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS].gx1 = EARTH_AXIS_ACCEL_X_HIGHPASS_GX1;
	firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS].gx2 = EARTH_AXIS_ACCEL_X_HIGHPASS_GX2;
	firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS].gx3 = EARTH_AXIS_ACCEL_X_HIGHPASS_GX3;
	firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS].previousInput  = 0.0f;
    firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS].gx1 = EARTH_AXIS_ACCEL_Y_HIGHPASS_GX1;
	firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS].gx2 = EARTH_AXIS_ACCEL_Y_HIGHPASS_GX2;
	firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS].gx3 = EARTH_AXIS_ACCEL_Y_HIGHPASS_GX3;
	firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS].previousInput  = 0.0f;
    firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS].gx1 = EARTH_AXIS_ACCEL_Z_HIGHPASS_GX1;
	firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS].gx2 = EARTH_AXIS_ACCEL_Z_HIGHPASS_GX2;
	firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS].gx3 = EARTH_AXIS_ACCEL_Z_HIGHPASS_GX3;
	firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS].previousInput  = 0.0f;
    firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS].previousOutput = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////


