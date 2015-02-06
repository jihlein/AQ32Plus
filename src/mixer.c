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

uint8_t numberMotor;

float throttleCmd = 2000.0f;

float motor[8] = { 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, };

float servo[3] = { 3000.0f, 3000.0f, 3000.0f, };

///////////////////////////////////////////////////////////////////////////////
// Initialize Mixer
///////////////////////////////////////////////////////////////////////////////

void initMixer(void)
{
    switch (eepromConfig.mixerConfiguration)
    {
        case MIXERTYPE_TRI:
            numberMotor = 3;
            motor[7] = eepromConfig.triYawServoMid;
            break;

        case MIXERTYPE_QUADX:
            numberMotor = 4;
            break;

        case MIXERTYPE_HEX6X:
        case MIXERTYPE_Y6:
            numberMotor = 6;
            break;

        case MIXERTYPE_FREE:
		    numberMotor = eepromConfig.freeMixMotors;
        	break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Write to Servos
///////////////////////////////////////////////////////////////////////////////

void writeServos(void)
{
    pwmServoWrite(0, (uint16_t)servo[0]);
    pwmServoWrite(1, (uint16_t)servo[1]);
    pwmServoWrite(2, (uint16_t)servo[2]);
}

///////////////////////////////////////////////////////////////////////////////
// Write to Motors
///////////////////////////////////////////////////////////////////////////////

void writeMotors(void)
{
    uint8_t i;

    if (eepromConfig.oneShot125 == true)
    {
        for (i = 0; i < numberMotor; i++)
            pwmEscWrite(i, (uint16_t)(motor[i] * 1.05f / 2.0f));

        TIM8->EGR |= TIM_EGR_UG;

        if ((numberMotor > 3) && (numberMotor <= 5))
        	TIM2->EGR |= TIM_EGR_UG;

        if (numberMotor > 5)
        {
        	TIM2->EGR |= TIM_EGR_UG;
        	TIM3->EGR |= TIM_EGR_UG;
        }

        for (i = 0; i < numberMotor; i++)
    	    pwmEscWrite(i, (uint16_t)0x0000);
    }
    else
    {
	    for (i = 0; i < numberMotor; i++)
		    pwmEscWrite(i, (uint16_t)motor[i]);
	}

    if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
        pwmEscWrite(7, (uint16_t)motor[7]);
}

///////////////////////////////////////////////////////////////////////////////
// Write to All Motors
///////////////////////////////////////////////////////////////////////////////

void writeAllMotors(float mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

///////////////////////////////////////////////////////////////////////////////
// Pulse Motors
///////////////////////////////////////////////////////////////////////////////

void pulseMotors(uint8_t quantity)
{
    uint8_t i;

    for ( i = 0; i < quantity; i++ )
    {
        writeAllMotors( eepromConfig.minThrottle );
        delay(250);
        writeAllMotors( (float)MINCOMMAND );
        delay(250);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Mixer
///////////////////////////////////////////////////////////////////////////////

#define PIDMIX(X,Y,Z,T) (ratePID[ROLL] * (X) + ratePID[PITCH] * (Y) + eepromConfig.yawDirection * ratePID[YAW] * (Z) + throttleCmd * (T))

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i;

    ///////////////////////////////////

    switch ( eepromConfig.mixerConfiguration )
    {
        ///////////////////////////////

        case MIXERTYPE_TRI:
            motor[0] = PIDMIX(  1.0f, -0.666667f, 0.0f, 1.0f );  // Left  CW
            motor[1] = PIDMIX( -1.0f, -0.666667f, 0.0f, 1.0f );  // Right CCW
            motor[2] = PIDMIX(  0.0f,  1.333333f, 0.0f, 1.0f );  // Rear  CW or CCW

            motor[7] = eepromConfig.triYawServoMid + eepromConfig.yawDirection * ratePID[YAW];

            motor[7] = firstOrderFilter(motor[7], &firstOrderFilters[TRICOPTER_YAW_LOWPASS]);

            motor[7] = constrain(motor[7], eepromConfig.triYawServoMin, eepromConfig.triYawServoMax );

            break;

        ///////////////////////////////

        case MIXERTYPE_QUADX:
            motor[0] = PIDMIX(  1.0f, -1.0f, -1.0f, 1.0f );      // Front Left  CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  1.0f, 1.0f );      // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  1.0f, -1.0f, 1.0f );      // Rear Right  CW
            motor[3] = PIDMIX(  1.0f,  1.0f,  1.0f, 1.0f );      // Rear Left   CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_HEX6X:
            motor[0] = PIDMIX(  0.866025f, -1.0f, -1.0f, 1.0f ); // Front Left  CW
            motor[1] = PIDMIX( -0.866025f, -1.0f,  1.0f, 1.0f ); // Front Right CCW
            motor[2] = PIDMIX( -0.866025f,  0.0f, -1.0f, 1.0f ); // Right       CW
            motor[3] = PIDMIX( -0.866025f,  1.0f,  1.0f, 1.0f ); // Rear Right  CCW
            motor[4] = PIDMIX(  0.866025f,  1.0f, -1.0f, 1.0f ); // Rear Left   CW
            motor[5] = PIDMIX(  0.866025f,  0.0f,  1.0f, 1.0f ); // Left        CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_Y6:
            motor[0] = PIDMIX(  1.0f, -0.666667, -1.0f, 1.0f );  // Top Left     CW
            motor[1] = PIDMIX( -1.0f, -0.666667,  1.0f, 1.0f );  // Top Right    CCW
            motor[2] = PIDMIX(  0.0f,  1.333333,  1.0f, 1.0f );  // Top Rear     CCW
            motor[3] = PIDMIX(  1.0f, -0.666667,  1.0f, 1.0f );  // Bottom Left  CCW
            motor[4] = PIDMIX( -1.0f, -0.666667, -1.0f, 1.0f );  // Bottom Right CW
            motor[5] = PIDMIX(  0.0f,  1.333333, -1.0f, 1.0f );  // Bottom Rear  CW
            break;

        ///////////////////////////////

		case MIXERTYPE_FREE:
		    for ( i = 0; i < numberMotor; i++ )
		        motor[i] = PIDMIX ( eepromConfig.freeMix[i][ROLL], eepromConfig.freeMix[i][PITCH], eepromConfig.freeMix[i][YAW], eepromConfig.freeMix[i][THROTTLE] );

        	break;

        ///////////////////////////////
    }

    ///////////////////////////////////

    // this is a way to still have good gyro corrections if any motor reaches its max.

    maxMotor = motor[0];

    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];

    for (i = 0; i < numberMotor; i++)
    {
        if (maxMotor > eepromConfig.maxThrottle)
            motor[i] -= maxMotor - eepromConfig.maxThrottle;

        motor[i] = constrain(motor[i], eepromConfig.minThrottle, eepromConfig.maxThrottle);

        if ((rxCommand[THROTTLE]) < eepromConfig.minCheck)
        {
            motor[i] = eepromConfig.minThrottle;
        }

        if ( armed == false )
            motor[i] = (float)MINCOMMAND;
    }
}

///////////////////////////////////////////////////////////////////////////////
