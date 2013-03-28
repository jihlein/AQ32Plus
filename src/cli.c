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

uint8_t cliBusy = false;

static volatile uint8_t queryType;
static volatile uint8_t validCommand = false;

uint8_t highSpeedTelem1Enabled = false;
uint8_t highSpeedTelem2Enabled = false;
uint8_t highSpeedTelem3Enabled = false;
uint8_t highSpeedTelem4Enabled = false;
uint8_t highSpeedTelem5Enabled = false;
uint8_t highSpeedTelem6Enabled = false;
uint8_t highSpeedTelem7Enabled = false;
uint8_t highSpeedTelem8Enabled = false;
uint8_t highSpeedTelem9Enabled = false;

///////////////////////////////////////////////////////////////////////////////
// High Speed Telem Disable
///////////////////////////////////////////////////////////////////////////////

void highSpeedTelemDisable(void)
{
	highSpeedTelem1Enabled = false;
	highSpeedTelem2Enabled = false;
	highSpeedTelem3Enabled = false;
	highSpeedTelem4Enabled = false;
	highSpeedTelem5Enabled = false;
	highSpeedTelem6Enabled = false;
	highSpeedTelem7Enabled = false;
	highSpeedTelem8Enabled = false;
	highSpeedTelem9Enabled = false;
}

///////////////////////////////////////////////////////////////////////////////
// Read Character String from USB Comm
///////////////////////////////////////////////////////////////////////////////

char *readStringUsb(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if ((data[index] = usbRead()) == 0)
        {
            delay(10);
            timeout++;
        }
        else
        {
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from USB Comm
///////////////////////////////////////////////////////////////////////////////

float readFloatUsb(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if ((data[index] = usbRead()) == 0)
        {
            delay(10);
            timeout++;
        }
        else
        {
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from USB Comm
///////////////////////////////////////////////////////////////////////////////

void readUsbPID(unsigned char PIDid)
{
  struct PIDdata* pid = &eepromConfig.PID[PIDid];

  pid->B              = readFloatUsb();
  pid->P              = readFloatUsb();
  pid->I              = readFloatUsb();
  pid->D              = readFloatUsb();
  pid->windupGuard    = readFloatUsb();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     =(uint8_t)readFloatUsb();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	uint8_t  index;

	if ((usbAvailable() && !validCommand))
    	queryType = usbRead();

    switch (queryType)
    {
        ///////////////////////////////

        case 'a': // Rate PIDs
            usbPrint("\n");

            usbPrint("Roll Rate PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_RATE_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_RATE_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_RATE_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_RATE_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_RATE_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[ROLL_RATE_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("Pitch Rate PID: ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_RATE_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_RATE_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_RATE_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_RATE_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_RATE_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[PITCH_RATE_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("Yaw Rate PID:   ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[YAW_RATE_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[YAW_RATE_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[YAW_RATE_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[YAW_RATE_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[YAW_RATE_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[YAW_RATE_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'b': // Attitude PIDs
            usbPrint("\n");

            usbPrint("Roll Attitude PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_ATT_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_ATT_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_ATT_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_ATT_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[ROLL_ATT_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[ROLL_ATT_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("Pitch Attitude PID: ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_ATT_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_ATT_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_ATT_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_ATT_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[PITCH_ATT_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[PITCH_ATT_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("Heading PID:        ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HEADING_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HEADING_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HEADING_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HEADING_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HEADING_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[HEADING_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'c': // Velocity PIDs
            usbPrint("\n");

            usbPrint("nDot PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[NDOT_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[NDOT_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[NDOT_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[NDOT_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[NDOT_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[NDOT_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("eDot PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[EDOT_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[EDOT_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[EDOT_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[EDOT_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[EDOT_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[EDOT_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("hDot PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HDOT_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HDOT_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HDOT_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HDOT_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[HDOT_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[HDOT_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            queryType = 'x';
            validCommand = false;
        	break;

        ///////////////////////////////

        case 'd': // Position PIDs
            usbPrint("\n");

            usbPrint("n PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[N_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[N_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[N_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[N_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[N_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[N_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("e PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[E_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[E_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[E_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[E_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[E_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[E_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            usbPrint("h PID:  ");
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[H_PID].B);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[H_PID].P);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[H_PID].I);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[H_PID].D);           usbPrint(numberString);
            snprintf(numberString, 16, "%8.4f, ", eepromConfig.PID[H_PID].windupGuard); usbPrint(numberString);
            if  (eepromConfig.PID[H_PID].dErrorCalc)
                usbPrint("Error\n");
            else
                usbPrint("State\n");

            queryType = 'x';
            validCommand = false;
        	break;

         ///////////////////////////////

        case 'e': // Loop Delta Times
        	snprintf(numberString, 16, "%7ld, ", deltaTime1000Hz); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", deltaTime500Hz ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", deltaTime100Hz ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", deltaTime50Hz  ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", deltaTime10Hz  ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", deltaTime5Hz   ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld\n", deltaTime1Hz   ); usbPrint(numberString);

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'f': // Loop Execution Times
        	snprintf(numberString, 16, "%7ld, ", executionTime1000Hz); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", executionTime500Hz ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", executionTime100Hz ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", executionTime50Hz  ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", executionTime10Hz  ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld, ", executionTime5Hz   ); usbPrint(numberString);
        	snprintf(numberString, 16, "%7ld\n", executionTime1Hz   ); usbPrint(numberString);

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'g': // 500 Hz Accels
        	ftoa(sensors.accel500Hz[XAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.accel500Hz[YAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.accel500Hz[ZAXIS], numberString); usbPrint(numberString); usbPrint("\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'h': // 100 hz Earth Axis Accels
        	ftoa(earthAxisAccels[XAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(earthAxisAccels[YAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(earthAxisAccels[ZAXIS], numberString); usbPrint(numberString); usbPrint("\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'i': // 500 Hz Gyros
        	ftoa(sensors.gyro500Hz[ROLL ] * R2D, numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.gyro500Hz[PITCH] * R2D, numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.gyro500Hz[YAW  ] * R2D, numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(mpu6000Temperature,             numberString); usbPrint(numberString); usbPrint("\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'j': // 10 Hz Mag Data
        	ftoa(sensors.mag10Hz[XAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.mag10Hz[YAXIS], numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.mag10Hz[ZAXIS], numberString); usbPrint(numberString); usbPrint("\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'k': // Vertical Axis Variables
        	ftoa(earthAxisAccels[ZAXIS],  numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.pressureAlt10Hz, numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(hDotEstimate,            numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(hEstimate,               numberString); usbPrint(numberString); usbPrint("\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'l': // Attitudes
        	snprintf(numberString, 16, "%9.4f, ", sensors.attitude500Hz[ROLL ] * R2D); usbPrint(numberString);
        	snprintf(numberString, 16, "%9.4f, ", sensors.attitude500Hz[PITCH] * R2D); usbPrint(numberString);
        	snprintf(numberString, 16, "%9.4f, ", heading.mag * R2D);                  usbPrint(numberString);
        	snprintf(numberString, 16, "%9.4f\n", heading.tru * R2D);                  usbPrint(numberString);

        	validCommand = false;
        	break;

       ///////////////////////////////

        case 'm': // GPS Data
        	snprintf(numberString, 16, "%12.7f, ", sensors.gpsLatitude  * R2D); usbPrint(numberString);
        	snprintf(numberString, 16, "%12.7f, ", sensors.gpsLongitude * R2D); usbPrint(numberString);

        	ftoa(sensors.gpsAltitude,          numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.gpsGroundSpeed,       numberString); usbPrint(numberString); usbPrint(", ");
        	ftoa(sensors.gpsGroundTrack * R2D, numberString); usbPrint(numberString); usbPrint("\n");

            validCommand = false;
            break;

        ///////////////////////////////

        case 'n': // GPS Stats
            if (sensors.gpsFix == FIX_2D)
                usbPrint("2D Fix,  ");
            else if (sensors.gpsFix == FIX_3D)
                usbPrint("3D Fix,  ");
            else if (sensors.gpsFix == FIX_2D_SBAS)
            	usbPrint("2D SBAS, ");
            else if (sensors.gpsFix == FIX_3D_SBAS)
            	usbPrint("3D SBAS, ");
            else
                usbPrint("No Fix, ");

            itoa(sensors.gpsNumSats, numberString, 10); usbPrint(numberString); usbPrint(", ");
        	itoa(sensors.gpsDate,    numberString, 10); usbPrint(numberString); usbPrint(" ");
        	ftoa(sensors.gpsTime,    numberString);     usbPrint(numberString); usbPrint(", ");
            ftoa(sensors.gpsHdop,    numberString);     usbPrint(numberString); usbPrint("\n");
            validCommand = false;
        	break;

        ///////////////////////////////

        case 'o': // Not Used
            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'p': // Not Used
            queryType = 'x';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'q': // Not Used
            queryType = 'x';
           	validCommand = false;
           	break;

        ///////////////////////////////

        case 'r':
        	if (flightMode == RATE)
        		usbPrint("Flight Mode = RATE      ");
        	else if (flightMode == ATTITUDE)
        		usbPrint("Flight Mode = ATTITUDE  ");
        	else if (flightMode == GPS)
        		usbPrint("Flight Mode = GPS       ");

        	if (headingHoldEngaged == true)
        	    usbPrint("Heading Hold = ENGAGED     ");
        	else
        	    usbPrint("Heading Hold = DISENGAGED  ");

        	if (altitudeHoldState == DISENGAGED)
        		usbPrint("Altitude Hold = DISENAGED\n");
            else if (altitudeHoldState == ENGAGED)
            	usbPrint("Altitude Hold = ENGAGED\n");
            else if (altitudeHoldState == PANIC)
            	usbPrint("Altitude Hold = PANIC\n");

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 's': // Raw Receiver Commands
            if (eepromConfig.receiverType == SPEKTRUM)
            {
				for (index = 0; index < eepromConfig.spektrumChannels - 1; index++)
                {
    		    	snprintf(numberString, 16, "%4ld, ", spektrumChannelData[index]);
    		    	usbPrint(numberString);
    		    }

                snprintf(numberString, 16, "%4ld\n", spektrumChannelData[eepromConfig.spektrumChannels - 1]);
                usbPrint(numberString);
		    }
		    else
		    {
				for (index = 0; index < 7; index++)
                {
    		    	snprintf(numberString, 16, "%4i, ", rxRead(index));
    		    	usbPrint(numberString);
    		    }

                snprintf(numberString, 16, "%4i, ", rxRead(7));
                usbPrint(numberString);
			}

        	validCommand = false;
        	break;

        ///////////////////////////////

        case 't': // Processed Receiver Commands
            for (index = 0; index < 7; index++)
            {
    			snprintf(numberString, 16, "%8.2f, ", rxCommand[index]);
    			usbPrint(numberString);
    		}

            snprintf(numberString, 16, "%8.2f\n", rxCommand[7]);
            usbPrint(numberString);

            validCommand = false;
            break;

        ///////////////////////////////

        case 'u': // Command in Detent Discretes
            if ( commandInDetent[ROLL] == true )
                usbPrint( "true" );
            else
                usbPrint( "false" );
            usbPrint(", ");

            if ( commandInDetent[PITCH] == true )
                usbPrint( "true" );
            else
                usbPrint( "false" );
            usbPrint(", ");

            if ( commandInDetent[YAW] == true )
                usbPrint( "true" );
            else
                usbPrint( "false" );
            usbPrint("\n");

            validCommand = false;
            break;

        ///////////////////////////////

        case 'v': // ESC PWM Outputs
            itoa(TIM8->CCR4, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM8->CCR3, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM8->CCR2, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM8->CCR1, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM2->CCR2, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM3->CCR1, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM3->CCR2, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM2->CCR1, numberString, 10); usbPrint(numberString); usbPrint("\n");

            validCommand = false;
            break;

        ///////////////////////////////

        case 'w': // Servo PWM Outputs
            itoa(TIM5->CCR3, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM5->CCR2, numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(TIM5->CCR1, numberString, 10); usbPrint(numberString); usbPrint("\n");

            validCommand = false;
            break;

        ///////////////////////////////

        case 'x':
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'y': // ESC Calibration
        	escCalibration();

        	queryType = 'x';
        	break;

        ///////////////////////////////

        case 'z':
            ftoa(batteryVoltage(), numberString);     usbPrint(numberString); usbPrint(", ");
            itoa(convertedADC2(),  numberString, 10); usbPrint(numberString); usbPrint(", ");
            itoa(convertedADC4(),  numberString, 10); usbPrint(numberString); usbPrint("\n");

            break;

        ///////////////////////////////

        case '1': // Turn high speed telemetry 1 on
        	highSpeedTelemDisable();
          	highSpeedTelem1Enabled = true;

        	queryType = 'x';
            break;

        ///////////////////////////////

        case '2': // Turn high speed telemetry 2 on
           	highSpeedTelemDisable();
           	highSpeedTelem2Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '3': // Turn high speed telemetry 3 on
           	highSpeedTelemDisable();
           	highSpeedTelem3Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '4': // Turn high speed telemetry 4 on
           	highSpeedTelemDisable();
           	highSpeedTelem4Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '5': // Turn high speed telemetry 5 on
           	highSpeedTelemDisable();
           	highSpeedTelem5Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '6': // Turn high speed telemetry 6 on
           	highSpeedTelemDisable();
           	highSpeedTelem6Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '7': // Turn high speed telemetry 7 on
           	highSpeedTelemDisable();
           	highSpeedTelem7Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '8': // Turn high speed telemetry 8 on
           	highSpeedTelemDisable();
           	highSpeedTelem8Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '9': // Turn high speed telemetry 9 on
           	highSpeedTelemDisable();
           	highSpeedTelem9Enabled = true;

            queryType = 'x';
           	break;

        ///////////////////////////////

        case '0': // Disable high speed telemetry
           	highSpeedTelemDisable();

            queryType = 'x';
           	break;

        ///////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////

        case 'A': // Read Roll Rate PID Values
            readUsbPID(ROLL_RATE_PID);
            usbPrint( "\nRoll Rate PID Received....\n" );

        	queryType = 'a';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch Rate PID Values
            readUsbPID(PITCH_RATE_PID);
            usbPrint( "\nPitch Rate PID Received....\n" );

        	queryType = 'a';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw Rate PID Values
            readUsbPID(YAW_RATE_PID);
            usbPrint( "\nYaw Rate PID Received....\n" );

        	queryType = 'a';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'D': // Read Roll Attitude PID Values
            readUsbPID(ROLL_ATT_PID);
            usbPrint( "\nRoll Attitude PID Received....\n" );

        	queryType = 'b';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'E': // Read Pitch Attitude PID Values
            readUsbPID(PITCH_ATT_PID);
            usbPrint( "\nPitch Attitude PID Received....\n" );

        	queryType = 'b';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'F': // Read Heading Hold PID Values
            readUsbPID(HEADING_PID);
            usbPrint( "\nHeading PID Received....\n" );

        	queryType = 'b';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'G': // Read nDot PID Values
            readUsbPID(NDOT_PID);
            usbPrint( "\nnDot PID Received....\n" );

        	queryType = 'c';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'H': // Read eDot PID Values
            readUsbPID(EDOT_PID);
            usbPrint( "\neDot PID Received....\n" );

            queryType = 'c';
          	validCommand = false;
          	break;

        ///////////////////////////////

        case 'I': // Read hDot PID Values
            readUsbPID(HDOT_PID);
            usbPrint( "\nhDot PID Received....\n" );

          	queryType = 'c';
          	validCommand = false;
          	break;

       	///////////////////////////////

        case 'J': // Read n PID Values
            readUsbPID(N_PID);
            usbPrint( "\nn PID Received....\n" );

            queryType = 'd';
            validCommand = false;
        	break;

        ///////////////////////////////

        case 'K': // Read e PID Values
            readUsbPID(E_PID);
            usbPrint( "\ne PID Received....\n" );

            queryType = 'd';
            validCommand = false;
        	break;

        ///////////////////////////////

        case 'L': // Read h PID Values
            readUsbPID(H_PID);
            usbPrint( "\nh PID Received....\n" );

            queryType = 'd';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'M': // MAX7456 CLI
           	max7456CLI();

           	queryType = 'x';
        	validCommand = false;
        	break;

        ///////////////////////////////

        case 'N': // Mixer CLI
            mixerCLI();

            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'O': // Receiver CLI
            receiverCLI();

            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'P': // Sensor CLI
           	sensorCLI();

           	queryType = 'x';
           	validCommand = false;
           	break;

        ///////////////////////////////

        case 'Q': // GPS CLI
            gpsCLI();

            queryType = 'x';
           	validCommand = false;
           	break;

        ///////////////////////////////

        case 'R': // Reset to Bootloader
        	usbPrint("Entering Bootloader....\n\n");
        	delay(100);
        	systemReset(true);
        	break;

        ///////////////////////////////

        case 'S': // Reset System
        	usbPrint("\nSystem Reseting....\n\n");
        	delay(100);
        	systemReset(false);
        	break;

        ///////////////////////////////

        case 'T': // Not Used
            queryType = 'x';
           	validCommand = false;
           	break;

        ///////////////////////////////

        case 'U': // Not Used
            queryType = 'x';
         	validCommand = false;
         	break;

        ///////////////////////////////

        case 'V': // Reset EEPROM Parameters
            usbPrint( "\nEEPROM Parameters Reset....\n" );
            checkFirstTime(true);
            usbPrint("\nSystem Resetting....\n\n");
            delay(100);
            systemReset(false);
            break;

        ///////////////////////////////

        case 'W': // Write EEPROM Parameters
            usbPrint("\nWriting EEPROM Parameters....\n");
            writeEEPROM();

            queryType = 'x';
         	validCommand = false;
         	break;

        ///////////////////////////////

        case 'X': // Not Used
            queryType = 'x';
            validCommand = false;
            break;

        ///////////////////////////////

        case 'Y': // Not Used
            queryType = 'x';
            break;

        ///////////////////////////////

        case 'Z': // Not Used
            queryType = 'x';
            break;

        ///////////////////////////////

        case '?': // Command Summary
        	cliBusy = true;

        	usbPrint("\n");
   		    usbPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'g' 500 Hz Accels                          'G' Set nDot PID Data        GB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'h' 100 Hz Earth Axis Accels               'H' Set eDot PID Data        HB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'j' 10 hz Mag Data                         'J' Set n PID Data           JB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'k' Vertical Axis Variable                 'K' Set e PID Data           KB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("'l' Attitudes                              'L' Set h PID Data           LB;P;I;D;windupGuard;dErrorCalc\n");
   		    usbPrint("\n");

   		    usbPrint("Press space bar for more, or enter a command....\n");
   		    while (usbAvailable() == false);
   		    queryType = usbRead();
   		    if (queryType != ' ')
   		    {
   		        validCommand = true;
   		        cliBusy = false;
   		    	return;
   		    }

   		    usbPrint("\n");
   		    usbPrint("'m' GPS Data                               'M' MAX7456 CLI\n");
   		    usbPrint("'n' GPS Stats                              'N' Mixer CLI\n");
   		    usbPrint("'o' Not Used                               'O' Receiver CLI\n");
   		    usbPrint("'p' Not Used                               'P' Sensor CLI\n");
   		    usbPrint("'q' Not Used                               'Q' GPS CLI\n");
   		    usbPrint("'r' Mode States                            'R' Reset and Enter Bootloader\n");
   		    usbPrint("'s' Raw Receiver Commands                  'S' Reset\n");
   		    usbPrint("'t' Processed Receiver Commands            'T' Not Used\n");
   		    usbPrint("'u' Command In Detent Discretes            'U' Not Used\n");
   		    usbPrint("'v' Motor PWM Outputs                      'V' Reset EEPROM Parameters\n");
   		    usbPrint("'w' Servo PWM Outputs                      'W' Write EEPROM Parameters\n");
   		    usbPrint("'x' Terminate Serial Communication         'X' Not Used\n");
   		    usbPrint("\n");

   		    usbPrint("Press space bar for more, or enter a command....\n");
   		    while (usbAvailable() == false);
   		    queryType = usbRead();
   		    if (queryType != ' ')
   		    {
   		    	validCommand = true;
   		    	cliBusy = false;
   		    	return;
   		    }

   		    usbPrint("\n");
   		    usbPrint("'y' ESC Calibration                        'Y' Not Used\n");
   		    usbPrint("'z' ADC Values                             'Z' Not Used\n");
   		    usbPrint("'1' High Speed Telemetry 1 Enable\n");
   		    usbPrint("'2' High Speed Telemetry 2 Enable\n");
   		    usbPrint("'3' High Speed Telemetry 3 Enable\n");
   		    usbPrint("'4' High Speed Telemetry 4 Enable\n");
   		    usbPrint("'5' High Speed Telemetry 5 Enable\n");
   		    usbPrint("'6' High Speed Telemetry 6 Enable\n");
   		    usbPrint("'7' High Speed Telemetry 7 Enable\n");
   		    usbPrint("'8' High Speed Telemetry 8 Enable\n");
   		    usbPrint("'9' High Speed Telemetry 9 Enable\n");
   		    usbPrint("'0' High Speed Telemetry Disable           '?' Command Summary\n");
   		    usbPrint("\n");

  		    queryType = 'x';
  		    cliBusy = false;
   		    break;

            ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
