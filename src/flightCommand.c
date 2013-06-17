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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float    rxCommand[8] = { 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };

uint8_t  commandInDetent[3]         = { true, true, true };
uint8_t  previousCommandInDetent[3] = { true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t flightMode = RATE;

uint8_t headingHoldEngaged     = false;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

semaphore_t armed          = false;
uint8_t     armingTimer    = 0;
uint8_t     disarmingTimer = 0;

///////////////////////////////////////////////////////////////////////////////
// Altitude Hold State Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t  altitudeModeState    = DISENGAGED;
uint8_t  altitudeHold         = DISENGAGED;
uint8_t  verticalVelocityHold = DISENGAGED;

uint16_t previousAUX2State = MINCOMMAND;

float    altitudeHoldThrottleValue = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Read Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void)
{
    uint8_t channel;

    if ( rcActive == true )
    {
		// Read receiver commands
        for (channel = 0; channel < 8; channel++)
            rxCommand[channel] = (float)rxRead(eepromConfig.rcMap[channel]);

        rxCommand[ROLL]  -= eepromConfig.midCommand;                  // Roll Range    -1000:1000
        rxCommand[PITCH] -= eepromConfig.midCommand;                  // Pitch Range   -1000:1000
        rxCommand[YAW]   -= eepromConfig.midCommand;                  // Yaw Range     -1000:1000

        rxCommand[THROTTLE] -= eepromConfig.midCommand - MIDCOMMAND;  // Throttle Range 2000:4000
        rxCommand[AUX1]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux1 Range     2000:4000
        rxCommand[AUX2]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux2 Range     2000:4000
        rxCommand[AUX3]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux3 Range     2000:4000
        rxCommand[AUX4]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux4 Range     2000:4000
    }

    // Set past command in detent values
    for (channel = 0; channel < 3; channel++)
    	previousCommandInDetent[channel] = commandInDetent[channel];

    // Apply deadbands and set detent discretes'
    for (channel = 0; channel < 3; channel++)
    {
    	if ((rxCommand[channel] <= DEADBAND) && (rxCommand[channel] >= -DEADBAND))
        {
            rxCommand[channel] = 0;
  	        commandInDetent[channel] = true;
  	    }
        else
  	    {
  	        commandInDetent[channel] = false;
  	        if (rxCommand[channel] > 0)
  	        {
  		        rxCommand[channel] = (rxCommand[channel] - DEADBAND) * DEADBAND_SLOPE;
  	        }
  	        else
  	        {
  	            rxCommand[channel] = (rxCommand[channel] + DEADBAND) * DEADBAND_SLOPE;
  	        }
        }
    }

    ///////////////////////////////////

    // Check for low throttle
    if ( rxCommand[THROTTLE] < eepromConfig.minCheck )
    {
		// Check for disarm command ( low throttle, left yaw )
		if ( (rxCommand[YAW] < (eepromConfig.minCheck - MIDCOMMAND)) && (armed == true) )
		{
			disarmingTimer++;

			if (disarmingTimer > eepromConfig.disarmCount)
			{
				zeroPIDintegralError();
			    zeroPIDstates();
			    armed = false;
			    disarmingTimer = 0;
			}
		}
		else
		{
			disarmingTimer = 0;
		}

		// Check for gyro bias command ( low throttle, left yaw, aft pitch, right roll )
		if ( (rxCommand[YAW  ] < (eepromConfig.minCheck - MIDCOMMAND)) &&
		     (rxCommand[ROLL ] > (eepromConfig.maxCheck - MIDCOMMAND)) &&
		     (rxCommand[PITCH] < (eepromConfig.minCheck - MIDCOMMAND)) )
		{
			computeMPU6000RTData();
			pulseMotors(3);
		}

		// Check for arm command ( low throttle, right yaw)
		if ((rxCommand[YAW] > (eepromConfig.maxCheck - MIDCOMMAND) ) && (armed == false) && (execUp == true))
		{
			armingTimer++;

			if (armingTimer > eepromConfig.armCount)
			{
				zeroPIDintegralError();
				zeroPIDstates();
				armed = true;
				armingTimer = 0;
			}
		}
		else
		{
			armingTimer = 0;
		}
	}

	///////////////////////////////////

	// Check for armed true and throttle command > minThrottle

    if ((armed == true) && (rxCommand[THROTTLE] > eepromConfig.minThrottle))
    	holdIntegrators = false;
    else
    	holdIntegrators = true;

    ///////////////////////////////////

    // Check AUX1 for rate, attitude, or GPS mode (3 Position Switch) NOT COMPLETE YET....

	if ((rxCommand[AUX1] > MIDCOMMAND) && (flightMode == RATE))
	{
		flightMode = ATTITUDE;
		setPIDintegralError(ROLL_ATT_PID,  0.0f);
		setPIDintegralError(PITCH_ATT_PID, 0.0f);
		setPIDintegralError(HEADING_PID,   0.0f);
		setPIDstates(ROLL_ATT_PID,  0.0f);
		setPIDstates(PITCH_ATT_PID, 0.0f);
		setPIDstates(HEADING_PID,   0.0f);
	}
	else if ((rxCommand[AUX1] <= MIDCOMMAND) && (flightMode == ATTITUDE))
	{
		flightMode = RATE;
		setPIDintegralError(ROLL_RATE_PID,  0.0f);
		setPIDintegralError(PITCH_RATE_PID, 0.0f);
		setPIDintegralError(YAW_RATE_PID,   0.0f);
		setPIDstates(ROLL_RATE_PID,  0.0f);
		setPIDstates(PITCH_RATE_PID, 0.0f);
		setPIDstates(YAW_RATE_PID,   0.0f);
	}

	///////////////////////////////////

	// Check yaw in detent and flight mode to determine hdg hold engaged state

	if ((commandInDetent[YAW] == true) && (flightMode == ATTITUDE))
	    headingHoldEngaged = true;
	else
	    headingHoldEngaged = false;

	///////////////////////////////////

	// Check AUX2 for altitude hold mode (2 Position Switch)

	if ((rxCommand[AUX2] > MIDCOMMAND) && (previousAUX2State <= MIDCOMMAND))           // Rising edge detection
	{
		altitudeModeState = ENGAGED;
		altitudeHoldThrottleValue = rxCommand[THROTTLE];
	}
	else if (((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND)) ||  // Falling edge detection
			 ((rxCommand[THROTTLE] < 2333.3f) || (rxCommand[THROTTLE] > 3666.6f)))     // Reference out of range
	{
		altitudeModeState    = DISENGAGED;
		altitudeHold         = DISENGAGED;
		verticalVelocityHold = DISENGAGED;
	}

	if (altitudeModeState == ENGAGED)
	{
		if (fabs(altitudeHoldThrottleValue - rxCommand[THROTTLE]) < 100.0f)
    	{
            altitudeHold         = ENGAGED;
	        verticalVelocityHold = DISENGAGED;
	    }
	    else
	    {
		    altitudeHold         = DISENGAGED;
		    verticalVelocityHold = ENGAGED;
	    }
	}

	previousAUX2State = rxCommand[AUX2];



	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////




