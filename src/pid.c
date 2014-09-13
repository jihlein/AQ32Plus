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

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t pidReset = true;

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
    	eepromConfig.PID[index].integratorState = 0.0f;
    	eepromConfig.PID[index].filterState     = 0.0f;
    	eepromConfig.PID[index].prevResetState  = false;
    }
}

///////////////////////////////////////////////////////////////////////////////

float updatePID(float error, float deltaT, uint8_t reset, struct PIDdata *PIDparameters)
{
    float dTerm;
    float pidSum;
    float pidLimited;

    if ((reset == true) || (PIDparameters->prevResetState == true))
    {
        PIDparameters->integratorState = 0.0f;
        PIDparameters->filterState     = 0.0f;
    }

    dTerm = ((error * PIDparameters->D) - PIDparameters->filterState) * 100.0f;

    pidSum = (error * PIDparameters->P) + PIDparameters->integratorState + dTerm;

    if (pidSum > PIDparameters->Limit)
    {
        pidLimited = PIDparameters->Limit;
    }
    else
    {
        pidLimited = -PIDparameters->Limit;

        if (!(pidSum < (-PIDparameters->Limit)))
        {
            pidLimited = pidSum;
        }
    }

    PIDparameters->integratorState += ((error * PIDparameters->I) + 100.0f * (pidLimited - pidSum)) * deltaT;

    PIDparameters->filterState += deltaT * dTerm;

    if (reset == true)
        PIDparameters->prevResetState = true;
    else
        PIDparameters->prevResetState = false;

    return pidLimited;
}

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value)
{
    eepromConfig.PID[IDPid].integratorState = value;
    eepromConfig.PID[IDPid].filterState     = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDstates(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////
