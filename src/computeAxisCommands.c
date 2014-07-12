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

float   attCmd[3];

float   attPID[3];

float   ratePID[3];

float   rateCmd[3];

float   headingReference;

float   altitudeHoldReference;

float   throttleReference;

float   verticalVelocityCmd;

///////////////////////////////////////////////////////////////////////////////
// Compute Axis Commands
///////////////////////////////////////////////////////////////////////////////

void computeAxisCommands(float dt)
{
    float error;
    float tempAttCompensation;

    if (flightMode == ATTITUDE)
    {
        attCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.attitudeScaling;
        attCmd[PITCH] = rxCommand[PITCH] * eepromConfig.attitudeScaling;
    }

    if (flightMode >= ATTITUDE)
    {
        error = standardRadianFormat(attCmd[ROLL] - sensors.attitude500Hz[ROLL]);
        attPID[ROLL]  = updatePID(error, dt, eepromConfig.attitudeScaling, pidReset, &eepromConfig.PID[ROLL_ATT_PID ]);

        error = standardRadianFormat(attCmd[PITCH] + sensors.attitude500Hz[PITCH]);
        attPID[PITCH] = updatePID(error, dt, eepromConfig.attitudeScaling, pidReset, &eepromConfig.PID[PITCH_ATT_PID]);
    }


    if (flightMode == RATE)
    {
        rateCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.rollAndPitchRateScaling;
        rateCmd[PITCH] = rxCommand[PITCH] * eepromConfig.rollAndPitchRateScaling;
    }
    else
    {
        rateCmd[ROLL ] = attPID[ROLL ];
        rateCmd[PITCH] = attPID[PITCH];
    }

    ///////////////////////////////////

    if (headingHoldEngaged == true)  // Heading Hold is ON
    {
    	error = standardRadianFormat(headingReference - heading.mag);
        rateCmd[YAW] = updatePID(error, dt, eepromConfig.attitudeScaling, pidReset, &eepromConfig.PID[HEADING_PID]);
    }
    else                             // Heading Hold is OFF
	    rateCmd[YAW] = rxCommand[YAW] * eepromConfig.yawRateScaling;

	///////////////////////////////////

    error = rateCmd[ROLL] - sensors.gyro500Hz[ROLL];
    ratePID[ROLL] = updatePID(error, dt, eepromConfig.rollAndPitchRateScaling, pidReset, &eepromConfig.PID[ROLL_RATE_PID ]);

    error = rateCmd[PITCH] + sensors.gyro500Hz[PITCH];
    ratePID[PITCH] = updatePID(error, dt, eepromConfig.rollAndPitchRateScaling, pidReset, &eepromConfig.PID[PITCH_RATE_PID]);

    error = rateCmd[YAW] - sensors.gyro500Hz[YAW];
    ratePID[YAW] = updatePID(error, dt, eepromConfig.yawRateScaling, pidReset, &eepromConfig.PID[YAW_RATE_PID  ]);

	///////////////////////////////////

	if (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE)            // Manual Mode is ON
	    throttleCmd = rxCommand[THROTTLE];
	else
	{
	    if ((verticalModeState == ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT) ||  // Altitude Hold is ON
	        (verticalModeState == ALT_HOLD_AT_REFERENCE_ALTITUDE)   ||
	        (verticalModeState == ALT_DISENGAGED_THROTTLE_INACTIVE))
        {
            error = altitudeHoldReference - hEstimate;
			verticalVelocityCmd = updatePID(error, dt, eepromConfig.hDotScaling, pidReset, &eepromConfig.PID[H_PID]);
		}
	    else                                                            // Vertical Velocity Hold is ON
	    {
	        verticalVelocityCmd = verticalReferenceCommand * eepromConfig.hDotScaling;
	    }

    	error = verticalVelocityCmd - hDotEstimate;
		throttleCmd = throttleReference + updatePID(error, dt, eepromConfig.hDotScaling, pidReset, &eepromConfig.PID[HDOT_PID]);

	     // Get Roll Angle, Constrain to +/-20 degrees (default)
	    tempAttCompensation = constrain(sensors.attitude500Hz[ROLL ], eepromConfig.rollAttAltCompensationLimit,  -eepromConfig.rollAttAltCompensationLimit);

	    // Compute Cosine of Roll Angle and Multiply by Att-Alt Gain
	    tempAttCompensation = eepromConfig.rollAttAltCompensationGain / cosf(tempAttCompensation);

	    // Apply Roll Att Compensation to Throttle Command
	    throttleCmd *= tempAttCompensation;

	    // Get Pitch Angle, Constrain to +/-20 degrees (default)
	    tempAttCompensation = constrain(sensors.attitude500Hz[PITCH], eepromConfig.pitchAttAltCompensationLimit,  -eepromConfig.pitchAttAltCompensationLimit);

	    // Compute Cosine of Pitch Angle and Multiply by Att-Alt Gain
	    tempAttCompensation = eepromConfig.pitchAttAltCompensationGain / cosf(tempAttCompensation);

	    // Apply Pitch Att Compensation to Throttle Command
	    throttleCmd *= tempAttCompensation;
	}
}

///////////////////////////////////////////////////////////////////////////////
