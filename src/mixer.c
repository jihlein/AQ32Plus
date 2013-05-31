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

float motor[8] = { 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, };

float servo[3] = { 3000.0f, 3000.0f, 3000.0f, };

///////////////////////////////////////////////////////////////////////////////
// Initialize Mixer
///////////////////////////////////////////////////////////////////////////////

void initMixer(void)
{
    switch (eepromConfig.mixerConfiguration)
    {
        case MIXERTYPE_GIMBAL:
            numberMotor = 0;
            break;

        case MIXERTYPE_FLYING_WING:
	        numberMotor = 1;
	        break;

        case MIXERTYPE_BI:
            numberMotor = 2;
            break;

        case MIXERTYPE_TRI:
            numberMotor = 3;
            break;

        case MIXERTYPE_QUADP:
        case MIXERTYPE_QUADX:
        case MIXERTYPE_VTAIL4_NO_COMP:
        case MIXERTYPE_VTAIL4_Y_COMP:
		case MIXERTYPE_VTAIL4_RY_COMP:
		case MIXERTYPE_VTAIL4_PY_COMP:
		case MIXERTYPE_VTAIL4_RP_COMP:
        case MIXERTYPE_VTAIL4_RPY_COMP:
        case MIXERTYPE_Y4:
            numberMotor = 4;
            break;

        case MIXERTYPE_HEX6P:
        case MIXERTYPE_HEX6X:
        case MIXERTYPE_Y6:
            numberMotor = 6;
            break;

        case MIXERTYPE_OCTOF8P:
        case MIXERTYPE_OCTOF8X:
        case MIXERTYPE_OCTOX8P:
        case MIXERTYPE_OCTOX8X:
            numberMotor = 8;
            break;

        case MIXERTYPE_FREEMIX:
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

    for (i = 0; i < numberMotor; i++)
        pwmEscWrite(i, (uint16_t)motor[i]);
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

#define PIDMIX(X,Y,Z) rxCommand[THROTTLE] + axisPID[ROLL] * (X) + axisPID[PITCH] * (Y) + eepromConfig.yawDirection * axisPID[YAW] * (Z)

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i;

    ///////////////////////////////////
    /*
    if ( numberMotor > 3 )
    {
        //prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - fabs(rcCommand[YAW]), +100 + fabs(rcCommand[YAW]));
    }
    */
    ///////////////////////////////////

    switch ( eepromConfig.mixerConfiguration )
    {
        case MIXERTYPE_GIMBAL:
            servo[0] = constrain( eepromConfig.gimbalRollServoMid + eepromConfig.gimbalRollServoGain * sensors.attitude500Hz[ROLL] + rxCommand[ROLL],
                                  eepromConfig.gimbalRollServoMin, eepromConfig.gimbalRollServoMax );

            servo[1] = constrain( eepromConfig.gimbalPitchServoMid + eepromConfig.gimbalPitchServoGain * sensors.attitude500Hz[PITCH] + rxCommand[PITCH],
                                  eepromConfig.gimbalPitchServoMin, eepromConfig.gimbalPitchServoMax );
            break;

        ///////////////////////////////

        case MIXERTYPE_FLYING_WING:
            motor[0] = rxCommand[THROTTLE];
            if (flightMode != ATTITUDE)
            {   // do not use sensors for correction, simple 2 channel mixing
            	servo[0] = eepromConfig.pitchDirectionLeft  * (rxCommand[PITCH] - eepromConfig.midCommand) +
            			   eepromConfig.rollDirectionLeft   * (rxCommand[ROLL ] - eepromConfig.midCommand);

            	servo[1] = eepromConfig.pitchDirectionRight * (rxCommand[PITCH] - eepromConfig.midCommand) +
            			   eepromConfig.rollDirectionRight  * (rxCommand[ROLL]  - eepromConfig.midCommand);
            }
            else
            {   // use sensors to correct (attitude only)
            	servo[0] = eepromConfig.pitchDirectionLeft  * axisPID[PITCH] +
            			   eepromConfig.rollDirectionLeft   * axisPID[ROLL];

            	servo[1] = eepromConfig.pitchDirectionRight * axisPID[PITCH] +
            			   eepromConfig.rollDirectionRight  * axisPID[ROLL];
            }

            servo[0] = constrain(servo[0] + eepromConfig.midCommand, eepromConfig.wingLeftMinimum,
            		                                                 eepromConfig.wingLeftMaximum);

            servo[1] = constrain(servo[1] + eepromConfig.midCommand, eepromConfig.wingRightMinimum,
            		                                                 eepromConfig.wingRightMaximum);
            break;

        ///////////////////////////////

        case MIXERTYPE_BI:
            motor[0] = PIDMIX(  1.0f, 0.0f, 0.0f );        // Left Motor
            motor[1] = PIDMIX( -1.0f, 0.0f, 0.0f );        // Right Motor

            servo[0] = constrain( eepromConfig.biLeftServoMid + (eepromConfig.yawDirection * axisPID[YAW]) + axisPID[PITCH],
                                  eepromConfig.biLeftServoMin, eepromConfig.biLeftServoMax );   // Left Servo

            servo[1] = constrain( eepromConfig.biRightServoMid + (eepromConfig.yawDirection * axisPID[YAW]) - axisPID[PITCH],
                                  eepromConfig.biRightServoMin, eepromConfig.biRightServoMax );   // Right Servo
            break;

        ///////////////////////////////

        case MIXERTYPE_TRI:
            motor[0] = PIDMIX(  1.0f, -0.666667f, 0.0f );  // Left  CW
            motor[1] = PIDMIX( -1.0f, -0.666667f, 0.0f );  // Right CCW
            motor[2] = PIDMIX(  0.0f,  1.333333f, 0.0f );  // Rear  CW or CCW

            servo[0] = constrain( eepromConfig.triYawServoMid + eepromConfig.yawDirection * axisPID[YAW],
                                  eepromConfig.triYawServoMin, eepromConfig.triYawServoMax ); // Tail Servo
            break;

        ///////////////////////////////

        case MIXERTYPE_QUADP:
            motor[0] = PIDMIX(  0.0f, -1.0f, -1.0f );      // Front CW
            motor[1] = PIDMIX( -1.0f,  0.0f,  1.0f );      // Right CCW
            motor[2] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Rear  CW
            motor[3] = PIDMIX(  1.0f,  0.0f,  1.0f );      // Left  CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_QUADX:
            motor[0] = PIDMIX(  1.0f, -1.0f, -1.0f );      // Front Left  CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  1.0f );      // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  1.0f, -1.0f );      // Rear Right  CW
            motor[3] = PIDMIX(  1.0f,  1.0f,  1.0f );      // Rear Left   CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_NO_COMP:
            motor[0] = PIDMIX(  1.0f, -1.0f,  0.0f );      // Front Left  CCW - NOTE rotation difference for vtail configurations
            motor[1] = PIDMIX( -1.0f, -1.0f,  0.0f );      // Front Right CW  - NOTE rotation difference for vtail configurations
            motor[2] = PIDMIX(  0.0f,  1.0f,  1.0f );      // Rear Right  CCW - NOTE rotation difference for vtail configurations
            motor[3] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Rear Left   CW  - NOTE rotation difference for vtail configurations
            break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_Y_COMP:
		    motor[0] = PIDMIX(  1.0f, -1.0f,  vTailThrust ); // Front Left  CCW - NOTE rotation difference for vtail configurations
		    motor[1] = PIDMIX( -1.0f, -1.0f, -vTailThrust ); // Front Right CW  - NOTE rotation difference for vtail configurations
		    motor[2] = PIDMIX(  0.0f,  1.0f,  1.0f        ); // Rear Right  CCW - NOTE rotation difference for vtail configurations
		    motor[3] = PIDMIX(  0.0f,  1.0f, -1.0f        ); // Rear Left   CW  - NOTE rotation difference for vtail configurations
		    break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_RY_COMP:
        	motor[0] = PIDMIX(  1.0f, -vTailThrust,  vTailThrust ); // Front Left  CCW - NOTE rotation difference for vtail configurations
			motor[1] = PIDMIX( -1.0f, -vTailThrust, -vTailThrust ); // Front Right CW  - NOTE rotation difference for vtail configurations
			motor[2] = PIDMIX(  0.0f,  1.0f,          1.0f       ); // Rear Right  CCW - NOTE rotation difference for vtail configurations
			motor[3] = PIDMIX(  0.0f,  1.0f,         -1.0f       ); // Rear Left   CW  - NOTE rotation difference for vtail configurations
			break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_PY_COMP:
            motor[0] = PIDMIX(  vTailThrust, -1.0f,  vTailThrust ); // Front Left  CCW - NOTE rotation difference for vtail configurations
			motor[1] = PIDMIX( -vTailThrust, -1.0f, -vTailThrust ); // Front Right CW  - NOTE rotation difference for vtail configurations
			motor[2] = PIDMIX( -1.0f,         1.0f,  1.0f        ); // Rear Right  CCW - NOTE rotation difference for vtail configurations
			motor[3] = PIDMIX(  1.0f,         1.0f, -1.0f        ); // Rear Left   CW  - NOTE rotation difference for vtail configurations
			break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_RP_COMP:
            motor[0] = PIDMIX(  vTailThrust, -vTailThrust,  0.0f ); // Front Left  CCW - NOTE rotation difference for vtail configurations
			motor[1] = PIDMIX( -vTailThrust, -vTailThrust, -0.0f ); // Front Right CW  - NOTE rotation difference for vtail configurations
			motor[2] = PIDMIX( -1.0f,         1.0f,         1.0f ); // Rear Right  CCW - NOTE rotation difference for vtail configurations
			motor[3] = PIDMIX(  1.0f,         1.0f,        -1.0f ); // Rear Left   CW  - NOTE rotation difference for vtail configurations
			break;

        ///////////////////////////////

        case MIXERTYPE_VTAIL4_RPY_COMP:
        	motor[0] = PIDMIX(  vTailThrust, -vTailThrust,  vTailThrust ); // Front Left  CCW - NOTE rotation difference for vtail configurations
			motor[1] = PIDMIX( -vTailThrust, -vTailThrust, -vTailThrust ); // Front Right CW  - NOTE rotation difference for vtail configurations
			motor[2] = PIDMIX( -1.0f,         1.0f,         1.0f        ); // Rear Right  CCW - NOTE rotation difference for vtail configurations
			motor[3] = PIDMIX(  1.0f,         1.0f,        -1.0f        ); // Rear Left   CW  - NOTE rotation difference for vtail configurations
			break;

        ///////////////////////////////

        case MIXERTYPE_Y4:
            motor[0] = PIDMIX(  1.0f, -1.0f,  0.0f );      // Front Left  CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  0.0f );      // Front Right CCW
            motor[2] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Top Rear    CW
            motor[3] = PIDMIX(  0.0f,  1.0f,  1.0f );      // Bottom Rear CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_HEX6P:
            motor[0] = PIDMIX(  0.0f, -0.866025f, -1.0f ); // Front       CW
            motor[1] = PIDMIX( -1.0f, -0.866025f,  1.0f ); // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  0.866025f, -1.0f ); // Rear Right  CW
            motor[3] = PIDMIX(  0.0f,  0.866025f,  1.0f ); // Rear        CCW
            motor[4] = PIDMIX(  1.0f,  0.866025f, -1.0f ); // Rear Left   CW
            motor[5] = PIDMIX(  1.0f, -0.866025f,  1.0f ); // Front Left  CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_HEX6X:
            motor[0] = PIDMIX(  0.866025f, -1.0f, -1.0f ); // Front Left  CW
            motor[1] = PIDMIX( -0.866025f, -1.0f,  1.0f ); // Front Right CCW
            motor[2] = PIDMIX( -0.866025f,  0.0f, -1.0f ); // Right       CW
            motor[3] = PIDMIX( -0.866025f,  1.0f,  1.0f ); // Rear Right  CCW
            motor[4] = PIDMIX(  0.866025f,  1.0f, -1.0f ); // Rear Left   CW
            motor[5] = PIDMIX(  0.866025f,  0.0f,  1.0f ); // Left        CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_Y6:
            motor[0] = PIDMIX(  1.0f, -0.666667, -1.0f );  // Top Left     CW
            motor[1] = PIDMIX( -1.0f, -0.666667, -1.0f );  // Top Right    CW
            motor[2] = PIDMIX(  0.0f,  1.333333,  1.0f );  // Top Rear     CCW
            motor[3] = PIDMIX(  1.0f, -0.666667,  1.0f );  // Bottom Left  CCW
            motor[4] = PIDMIX( -1.0f, -0.666667,  1.0f );  // Bottom Right CCW
            motor[5] = PIDMIX(  0.0f,  1.333333, -1.0f );  // Bottom Rear  CW
            break;

        ///////////////////////////////

        case MIXERTYPE_OCTOF8P:
            motor[0] = PIDMIX(  0.0f, -1.0f, -1.0f );      // Front       CW
            motor[1] = PIDMIX( -0.7f, -0.7f,  1.0f );      // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  0.0f, -1.0f );      // Right       CW
            motor[3] = PIDMIX( -0.7f,  0.7f,  1.0f );      // Rear Right  CCW
            motor[4] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Rear        CW
            motor[5] = PIDMIX(  0.7f,  0.7f,  1.0f );      // Rear left   CCW
            motor[6] = PIDMIX(  1.0f,  0.0f, -1.0f );      // Left        CW
            motor[7] = PIDMIX(  0.7f, -0.7f,  1.0f );      // Front Left  CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_OCTOF8X:
            motor[0] = PIDMIX(  0.5f, -1.0f, -1.0f );      // Front Left      CW
            motor[1] = PIDMIX( -0.5f, -1.0f,  1.0f );      // Front Right     CCW
            motor[2] = PIDMIX( -1.0f, -0.5f, -1.0f );      // Mid Front Right CW
            motor[3] = PIDMIX( -1.0f,  0.5f,  1.0f );      // Mid Rear Right  CCW
            motor[4] = PIDMIX( -0.5f,  1.0f, -1.0f );      // Rear Right      CW
            motor[5] = PIDMIX(  0.5f,  1.0f,  1.0f );      // Rear Left       CCW
            motor[6] = PIDMIX(  1.0f,  0.5f, -1.0f );      // Mid Rear left   CW
            motor[7] = PIDMIX(  1.0f, -0.5f,  1.0f );      // Mid Front Left  CCW
            break;

        ///////////////////////////////

        case MIXERTYPE_OCTOX8P:
            motor[0] = PIDMIX(  0.0f, -1.0f, -1.0f );      // Top Front    CW
            motor[1] = PIDMIX( -1.0f,  0.0f,  1.0f );      // Top Right    CCW
            motor[2] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Top Rear     CW
            motor[3] = PIDMIX(  1.0f,  0.0f,  1.0f );      // Top Left     CCW
            motor[4] = PIDMIX(  0.0f, -1.0f, -1.0f );      // Bottom Front CCW
            motor[5] = PIDMIX( -1.0f,  0.0f,  1.0f );      // Bottom Right CW
            motor[6] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Bottom Rear  CCW
            motor[7] = PIDMIX(  1.0f,  0.0f,  1.0f );      // Bottom Left  CW
            break;

        ///////////////////////////////

        case MIXERTYPE_OCTOX8X:
            motor[0] = PIDMIX(  1.0f, -1.0f, -1.0f );      // Top Front Left     CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  1.0f );      // Top Front Right    CCW
            motor[2] = PIDMIX( -1.0f,  1.0f, -1.0f );      // Top Rear Right     CW
            motor[3] = PIDMIX(  1.0f,  1.0f,  1.0f );      // Top Rear Left      CCW
            motor[4] = PIDMIX(  1.0f, -1.0f, -1.0f );      // Bottom Front Left  CCW
            motor[5] = PIDMIX( -1.0f, -1.0f,  1.0f );      // Bottom Front Right CW
            motor[6] = PIDMIX( -1.0f,  1.0f, -1.0f );      // Bottom Rear Right  CCW
            motor[7] = PIDMIX(  1.0f,  1.0f,  1.0f );      // Bottom Rear Left   CW
            break;

        ///////////////////////////////

        case MIXERTYPE_FREEMIX:
        	for ( i = 0; i < numberMotor; i++ )
        	{
        		motor[i] = PIDMIX ( eepromConfig.freeMix[i][ROLL],
        				            eepromConfig.freeMix[i][PITCH],
        				            eepromConfig.freeMix[i][YAW] );
        	}

        	break;
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
