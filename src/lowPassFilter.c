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

fourthOrderData_t fourthOrder100Hz[3];
fourthOrderData_t fourthOrder500Hz[3];

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 100 Hz Data
///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder100Hz(float currentInput, fourthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)
    #define B0_100HZ  0.001893594048567f
    #define B1_100HZ -0.002220262954039f
    #define B2_100HZ  0.003389066536478f
    #define B3_100HZ -0.002220262954039f
    #define B4_100HZ  0.001893594048567f

    #define A1_100HZ -3.362256889209355f
    #define A2_100HZ  4.282608240117919f
    #define A3_100HZ -2.444765517272841f
    #define A4_100HZ  0.527149895089809f

    float output;

    output = B0_100HZ * currentInput +
             B1_100HZ * filterParameters->inputTm1  +
             B2_100HZ * filterParameters->inputTm2  +
             B3_100HZ * filterParameters->inputTm3  +
             B4_100HZ * filterParameters->inputTm4  -
             A1_100HZ * filterParameters->outputTm1 -
             A2_100HZ * filterParameters->outputTm2 -
             A3_100HZ * filterParameters->outputTm3 -
             A4_100HZ * filterParameters->outputTm4;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder100Hz(void)
{
    fourthOrder100Hz[AX_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[AX_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder100Hz[AY_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[AY_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder100Hz[AZ_FILTER].inputTm1 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].inputTm2 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].inputTm3 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].inputTm4 = -9.8065f;

    fourthOrder100Hz[AZ_FILTER].outputTm1 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].outputTm2 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].outputTm3 = -9.8065f;
    fourthOrder100Hz[AZ_FILTER].outputTm4 = -9.8065f;
}

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 500 Hz Data
///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder500Hz(float currentInput, fourthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/250)
    #define B0_500HZ  0.000987786751039f
    #define B1_500HZ -0.003762348901931f
    #define B2_500HZ  0.005553744695291f
    #define B3_500HZ -0.003762348901931f
    #define B4_500HZ  0.000987786751039f

    #define A1_500HZ -3.878129734998889f
    #define A2_500HZ  5.641762572815879f
    #define A3_500HZ -3.648875955419102f
    #define A4_500HZ  0.885247737995618f

    float output;

    output = B0_500HZ * currentInput +
    		 B1_500HZ * filterParameters->inputTm1  +
    		 B2_500HZ * filterParameters->inputTm2  +
    		 B3_500HZ * filterParameters->inputTm3  +
    		 B4_500HZ * filterParameters->inputTm4  -
    		 A1_500HZ * filterParameters->outputTm1 -
    		 A2_500HZ * filterParameters->outputTm2 -
    		 A3_500HZ * filterParameters->outputTm3 -
    		 A4_500HZ * filterParameters->outputTm4;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder500Hz(void)
{
    fourthOrder500Hz[AX_FILTER].inputTm1 = 0.0f;
    fourthOrder500Hz[AX_FILTER].inputTm2 = 0.0f;
    fourthOrder500Hz[AX_FILTER].inputTm3 = 0.0f;
    fourthOrder500Hz[AX_FILTER].inputTm4 = 0.0f;

    fourthOrder500Hz[AX_FILTER].outputTm1 = 0.0f;
    fourthOrder500Hz[AX_FILTER].outputTm2 = 0.0f;
    fourthOrder500Hz[AX_FILTER].outputTm3 = 0.0f;
    fourthOrder500Hz[AX_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder500Hz[AY_FILTER].inputTm1 = 0.0f;
    fourthOrder500Hz[AY_FILTER].inputTm2 = 0.0f;
    fourthOrder500Hz[AY_FILTER].inputTm3 = 0.0f;
    fourthOrder500Hz[AY_FILTER].inputTm4 = 0.0f;

    fourthOrder500Hz[AY_FILTER].outputTm1 = 0.0f;
    fourthOrder500Hz[AY_FILTER].outputTm2 = 0.0f;
    fourthOrder500Hz[AY_FILTER].outputTm3 = 0.0f;
    fourthOrder500Hz[AY_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder500Hz[AZ_FILTER].inputTm1 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].inputTm2 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].inputTm3 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].inputTm4 = -9.8065f;

    fourthOrder500Hz[AZ_FILTER].outputTm1 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].outputTm2 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].outputTm3 = -9.8065f;
    fourthOrder500Hz[AZ_FILTER].outputTm4 = -9.8065f;
}

///////////////////////////////////////////////////////////////////////////////
