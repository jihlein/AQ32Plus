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

uint8_t holdIntegrators = true;

#define F_CUT 20.0f
float rc;

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
    uint8_t index;

    rc = 1.0f / ( TWO_PI * F_CUT );

    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
    	eepromConfig.PID[index].iTerm          = 0.0f;
    	eepromConfig.PID[index].lastDcalcValue = 0.0f;
    	eepromConfig.PID[index].lastDterm      = 0.0f;
    	eepromConfig.PID[index].lastLastDterm  = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////

float updatePID(float command, float state, float deltaT, uint8_t iHold, struct PIDdata *PIDparameters)
{
    float error;
    float dTerm;
    float dTermFiltered;
    float dAverage;

    ///////////////////////////////////

    error = command - state;

    if (PIDparameters->type == ANGULAR)
        error = standardRadianFormat(error);

    ///////////////////////////////////

    if (iHold == false)
    {
    	PIDparameters->iTerm += error * deltaT;
    	PIDparameters->iTerm = constrain(PIDparameters->iTerm, -PIDparameters->windupGuard, PIDparameters->windupGuard);
    }

    ///////////////////////////////////

    if (PIDparameters->dErrorCalc == D_ERROR)  // Calculate D term from error
    {
		dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
        PIDparameters->lastDcalcValue = error;
	}
	else                                       // Calculate D term from state
	{
		dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;

		if (PIDparameters->type == ANGULAR)
		    dTerm = standardRadianFormat(dTerm);

		PIDparameters->lastDcalcValue = state;
	}

    ///////////////////////////////////

    dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);

    dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

    PIDparameters->lastLastDterm = PIDparameters->lastDterm;
    PIDparameters->lastDterm = dTermFiltered;

    ///////////////////////////////////

    if (PIDparameters->type == ANGULAR)
        return(PIDparameters->P * error                +
	           PIDparameters->I * PIDparameters->iTerm +
	           PIDparameters->D * dAverage);
    else
        return(PIDparameters->P * PIDparameters->B * command +
               PIDparameters->I * PIDparameters->iTerm       +
               PIDparameters->D * dAverage                   -
               PIDparameters->P * state);

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void setPIDintegralError(uint8_t IDPid, float value)
{
	eepromConfig.PID[IDPid].iTerm = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDintegralError(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDintegralError(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value)
{
    eepromConfig.PID[IDPid].lastDcalcValue = value;
    eepromConfig.PID[IDPid].lastDterm      = value;
    eepromConfig.PID[IDPid].lastLastDterm  = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDstates(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////


