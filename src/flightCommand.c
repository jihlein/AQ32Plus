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

float    rxCommand[12] = { 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };

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
// Vertical Mode State Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t  verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

uint16_t previousAUX2State = MINCOMMAND;
uint16_t previousAUX4State = MINCOMMAND;

uint8_t  vertRefCmdInDetent         = true;
uint8_t  previousVertRefCmdInDetent = true;

float    verticalReferenceCommand;

///////////////////////////////////////////////////////////////////////////////
// Read Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void)
{
    uint8_t channel;
    uint8_t channelsToRead = 8;

    float hdgDelta, simpleX, simpleY;

    if ( rcActive == true )
    {
		// Read receiver commands
    	if (eepromConfig.receiverType == PPM)
    		channelsToRead = eepromConfig.ppmChannels;

		for (channel = 0; channel < channelsToRead; channel++)
		{
			if (eepromConfig.receiverType == SPEKTRUM)
			    rxCommand[channel] = (float)spektrumRead(eepromConfig.rcMap[channel]);
			else
			    rxCommand[channel] = (float)rxRead(eepromConfig.rcMap[channel]);
		}

        rxCommand[ROLL]  -= eepromConfig.midCommand;  // Roll Range  -1000:1000
        rxCommand[PITCH] -= eepromConfig.midCommand;  // Pitch Range -1000:1000
        rxCommand[YAW]   -= eepromConfig.midCommand;  // Yaw Range   -1000:1000

		for (channel = 3; channel < channelsToRead; channel++)
			rxCommand[channel] -= eepromConfig.midCommand - MIDCOMMAND;  // Range 2000:4000
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

		if ((commandInDetent[YAW] == true) && (flightMode == ATTITUDE) && (headingHoldEngaged == false))
		{
			headingHoldEngaged = true;
		    setPIDintegralError(HEADING_PID, 0.0f);
	        setPIDstates(YAW_RATE_PID,       0.0f);
	        headingReference = heading.mag;
		}

		if (((commandInDetent[YAW] == false) || (flightMode != ATTITUDE)) && (headingHoldEngaged == true))
		{
		    headingHoldEngaged = false;
		}

	///////////////////////////////////

	// Simple Mode Command Processing

	if (rxCommand[AUX3] > MIDCOMMAND)
	{
        hdgDelta = sensors.attitude500Hz[YAW] - homeData.magHeading;

        hdgDelta = standardRadianFormat(hdgDelta);

        simpleX = cosf(hdgDelta) * rxCommand[PITCH] + sinf(hdgDelta) * rxCommand[ROLL ];

        simpleY = cosf(hdgDelta) * rxCommand[ROLL ] - sinf(hdgDelta) * rxCommand[PITCH];

        rxCommand[ROLL ] = simpleY;

        rxCommand[PITCH] = simpleX;
	}

	///////////////////////////////////

	// Vertical Mode Command Processing

	verticalReferenceCommand = rxCommand[THROTTLE] - eepromConfig.midCommand;

    // Set past altitude reference in detent value
    previousVertRefCmdInDetent = vertRefCmdInDetent;

    // Apply deadband and set detent discrete'
    if ((verticalReferenceCommand <= ALT_DEADBAND) && (verticalReferenceCommand >= -ALT_DEADBAND))
    {
        verticalReferenceCommand = 0;
  	    vertRefCmdInDetent = true;
  	}
    else
  	{
  	    vertRefCmdInDetent = false;
  	    if (verticalReferenceCommand > 0)
  	    {
  		    verticalReferenceCommand = (verticalReferenceCommand - ALT_DEADBAND) * ALT_DEADBAND_SLOPE;
  	    }
  	    else
  	    {
  	        verticalReferenceCommand = (verticalReferenceCommand + ALT_DEADBAND) * ALT_DEADBAND_SLOPE;
  	    }
    }

    ///////////////////////////////////

    // Vertical Mode State Machine

    switch (verticalModeState)
	{
		case ALT_DISENGAGED_THROTTLE_ACTIVE:
		    if ((rxCommand[AUX2] > MIDCOMMAND) && (previousAUX2State <= MIDCOMMAND))  // AUX2 Rising edge detection
		    {
				verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
				setPIDintegralError(HDOT_PID, 0.0f);
				setPIDintegralError(H_PID,    0.0f);
				setPIDstates(HDOT_PID,        0.0f);
				setPIDstates(H_PID,           0.0f);
                altitudeHoldReference = hEstimate;
                throttleReference     = rxCommand[THROTTLE];
		    }

		    break;

		///////////////////////////////

		case ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT:
		    if ((vertRefCmdInDetent == true) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		        verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;

		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case ALT_HOLD_AT_REFERENCE_ALTITUDE:
		    if ((vertRefCmdInDetent == false) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY;

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		        verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;

		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY:
		    if ((vertRefCmdInDetent == true) && !eepromConfig.verticalVelocityHoldOnly)
		    {
				verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;
				altitudeHoldReference = hEstimate;
			}

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		    {
				verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;
				altitudeHoldReference = hEstimate;
			}


		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case ALT_DISENGAGED_THROTTLE_INACTIVE:
			if (((rxCommand[THROTTLE] < throttleCmd + THROTTLE_WINDOW) && (rxCommand[THROTTLE] > throttleCmd - THROTTLE_WINDOW)) ||
			    eepromConfig.verticalVelocityHoldOnly)
			    verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

			if ((rxCommand[AUX2] > MIDCOMMAND) && (previousAUX2State <= MIDCOMMAND))  // AUX2 Rising edge detection
		        verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;

			if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
			    verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;
    }

	previousAUX2State = rxCommand[AUX2];
	previousAUX4State = rxCommand[AUX4];

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////




