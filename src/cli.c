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

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

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
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if ((data[index] = cliRead()) == 0)
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
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if ((data[index] = cliRead()) == 0)
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
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

void readCliPID(unsigned char PIDid)
{
  struct PIDdata* pid = &eepromConfig.PID[PIDid];

  pid->B              = readFloatCLI();
  pid->P              = readFloatCLI();
  pid->I              = readFloatCLI();
  pid->D              = readFloatCLI();
  pid->windupGuard    = readFloatCLI();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     =(uint8_t)readFloatCLI();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	uint8_t  index;

	if ((cliAvailable() && !validCliCommand))
    	cliQuery = cliRead();

    switch (cliQuery)
    {
        ///////////////////////////////

        case 'a': // Rate PIDs
            cliPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_RATE_PID].B,
                            		                                               eepromConfig.PID[ROLL_RATE_PID].P,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].I,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].D,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_RATE_PID].B,
                            		                                               eepromConfig.PID[PITCH_RATE_PID].P,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].I,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].D,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[YAW_RATE_PID].B,
                             		                                               eepromConfig.PID[YAW_RATE_PID].P,
                		                                                           eepromConfig.PID[YAW_RATE_PID].I,
                		                                                           eepromConfig.PID[YAW_RATE_PID].D,
                		                                                           eepromConfig.PID[YAW_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[YAW_RATE_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'b': // Attitude PIDs
            cliPrintF("\nRoll Attitude PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_ATT_PID].B,
              		                                                                   eepromConfig.PID[ROLL_ATT_PID].P,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].I,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].D,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].windupGuard,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Pitch Attitude PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_ATT_PID].B,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].P,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].I,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].D,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].windupGuard,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Heading PID:        %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[HEADING_PID].B,
               		                                                                   eepromConfig.PID[HEADING_PID].P,
               		                                                                   eepromConfig.PID[HEADING_PID].I,
               		                                                                   eepromConfig.PID[HEADING_PID].D,
               		                                                                   eepromConfig.PID[HEADING_PID].windupGuard,
               		                                                                   eepromConfig.PID[HEADING_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;
        ///////////////////////////////

        case 'c': // Velocity PIDs
            cliPrintF("\nnDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[NDOT_PID].B,
               		                                                          eepromConfig.PID[NDOT_PID].P,
               		                                                          eepromConfig.PID[NDOT_PID].I,
               		                                                          eepromConfig.PID[NDOT_PID].D,
               		                                                          eepromConfig.PID[NDOT_PID].windupGuard,
               		                                                          eepromConfig.PID[NDOT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("eDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[EDOT_PID].B,
               		                                                          eepromConfig.PID[EDOT_PID].P,
               		                                                          eepromConfig.PID[EDOT_PID].I,
               		                                                          eepromConfig.PID[EDOT_PID].D,
               		                                                          eepromConfig.PID[EDOT_PID].windupGuard,
               		                                                          eepromConfig.PID[EDOT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("hDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[HDOT_PID].B,
               		                                                          eepromConfig.PID[HDOT_PID].P,
               		                                                          eepromConfig.PID[HDOT_PID].I,
               		                                                          eepromConfig.PID[HDOT_PID].D,
               		                                                          eepromConfig.PID[HDOT_PID].windupGuard,
               		                                                          eepromConfig.PID[HDOT_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'd': // Position PIDs
            cliPrintF("\nN PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[N_PID].B,
               		                                                       eepromConfig.PID[N_PID].P,
               		                                                       eepromConfig.PID[N_PID].I,
               		                                                       eepromConfig.PID[N_PID].D,
               		                                                       eepromConfig.PID[N_PID].windupGuard,
               		                                                       eepromConfig.PID[N_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("E PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[E_PID].B,
               		                                                       eepromConfig.PID[E_PID].P,
               		                                                       eepromConfig.PID[E_PID].I,
               		                                                       eepromConfig.PID[E_PID].D,
               		                                                       eepromConfig.PID[E_PID].windupGuard,
               		                                                       eepromConfig.PID[E_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("h PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[H_PID].B,
               		                                                       eepromConfig.PID[H_PID].P,
               		                                                       eepromConfig.PID[H_PID].I,
               		                                                       eepromConfig.PID[H_PID].D,
               		                                                       eepromConfig.PID[H_PID].windupGuard,
               		                                                       eepromConfig.PID[H_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
          	break;

         ///////////////////////////////

        case 'e': // Loop Delta Times
           	cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", deltaTime1000Hz,
               		                                                deltaTime500Hz,
               		                                                deltaTime100Hz,
               		                                                deltaTime50Hz,
               		                                                deltaTime10Hz,
               		                                                deltaTime5Hz,
               		                                                deltaTime1Hz);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'f': // Loop Execution Times
           	cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", executionTime1000Hz,
           	        			                                    executionTime500Hz,
           	        			                                    executionTime100Hz,
           	        			                                    executionTime50Hz,
           	        			                                    executionTime10Hz,
           	        			                                    executionTime5Hz,
           	        			                                    executionTime1Hz);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'g': // 100 Hz Accels
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", sensors.accel100Hz[XAXIS],
        			                                                sensors.accel100Hz[YAXIS],
        			                                                sensors.accel100Hz[ZAXIS],
        			                                                sensors.accel100HzMXR[XAXIS],
        			                                                sensors.accel100HzMXR[YAXIS],
        			                                                sensors.accel100HzMXR[ZAXIS]);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'h': // 100 hz Earth Axis Accels
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", earthAxisAccels[XAXIS],
        			                           earthAxisAccels[YAXIS],
        			                           earthAxisAccels[ZAXIS]);
        	validCliCommand = false;
        	break;
        ///////////////////////////////

        case 'i': // 500 hz Gyros
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ] * R2D,
        			                                  sensors.gyro500Hz[PITCH] * R2D,
        					                          sensors.gyro500Hz[YAW  ] * R2D,
        					                          mpu6000Temperature);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'j': // 10 Hz Mag Data
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
        			                           sensors.mag10Hz[YAXIS],
        			                           sensors.mag10Hz[ZAXIS]);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'k': // Vertical Axis Variables
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %4ld\n", earthAxisAccels[ZAXIS],
        			                                        sensors.pressureAlt50Hz,
        					                                hDotEstimate,
        					                                hEstimate,
        					                                ms5611Temperature);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'l': // Attitudes
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ] * R2D,
        			                           sensors.attitude500Hz[PITCH] * R2D,
        			                           sensors.attitude500Hz[YAW  ] * R2D);
        	validCliCommand = false;
        	break;

       ///////////////////////////////

        case 'm': // GPS Data
        	cliPrintF("%12.7f, %12.7f, %7.2f, %6.2f, %6.2f\n", sensors.gpsLatitude  * R2D,
        			                                           sensors.gpsLongitude * R2D,
        			                                           sensors.gpsAltitude,
        			                                           sensors.gpsGroundSpeed,
        			                                           sensors.gpsGroundTrack * R2D);
        	validCliCommand = false;
            break;

        ///////////////////////////////

        case 'n': // GPS Stats
            if (sensors.gpsFix == FIX_2D)
                cliPrint(" 2D Fix, ");
            else if (sensors.gpsFix == FIX_3D)
                cliPrint(" 3D Fix, ");
            else if (sensors.gpsFix == FIX_2D_SBAS)
            	cliPrint("2D SBAS, ");
            else if (sensors.gpsFix == FIX_3D_SBAS)
            	cliPrint("3D SBAS, ");
            else
                cliPrint(" No Fix, ");

            cliPrintF("%2ld, %8ld, %9.2f, %5.2f\n", sensors.gpsNumSats,
            		                                sensors.gpsDate,
            		                                sensors.gpsTime,
            		                                sensors.gpsHdop);
            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'o': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'p': // Not Used
            cliQuery = 'x';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'q': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'r':
        	if (flightMode == RATE)
        		cliPrint("Flight Mode = RATE      ");
        	else if (flightMode == ATTITUDE)
        		cliPrint("Flight Mode = ATTITUDE  ");
        	else if (flightMode == GPS)
        		cliPrint("Flight Mode = GPS       ");

        	if (headingHoldEngaged == true)
        	    cliPrint("Heading Hold = ENGAGED     ");
        	else
        	    cliPrint("Heading Hold = DISENGAGED  ");

        	if (altitudeHold == true)
        	    cliPrint("Alt Hold = ENGAGED     ");
        	else
        	    cliPrint("Alt Hold = DISENGAGED  ");

        	if (verticalVelocityHold == true)
        	    cliPrint("Vert Vel Hold = ENGAGED\n");
        	else
        	    cliPrint("Vert Vel Hold = DISENGAGED\n");

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 's': // Raw Receiver Commands
            if (eepromConfig.receiverType == SPEKTRUM)
            {
				for (index = 0; index < eepromConfig.spektrumChannels - 1; index++)
                     cliPrintF("%4ld, ", spektrumChannelData[index]);

                cliPrintF("%4ld\n", spektrumChannelData[eepromConfig.spektrumChannels - 1]);
            }
		    else
		    {
				for (index = 0; index < 7; index++)
                    cliPrintF("%4i, ", rxRead(index));

                cliPrintF("%4i\n", rxRead(7));
            }

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 't': // Processed Receiver Commands
            for (index = 0; index < 7; index++)
                cliPrintF("%8.2f, ", rxCommand[index]);

            cliPrintF("%8.2f\n", rxCommand[7]);

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'u': // Command in Detent Discretes
        	cliPrintF("%s, ", commandInDetent[ROLL ] ? " true" : "false");
        	cliPrintF("%s, ", commandInDetent[PITCH] ? " true" : "false");
        	cliPrintF("%s\n", commandInDetent[YAW  ] ? " true" : "false");

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'v': // ESC PWM Outputs
        	cliPrintF("%4ld, ", TIM8->CCR4);
        	cliPrintF("%4ld, ", TIM8->CCR3);
        	cliPrintF("%4ld, ", TIM8->CCR2);
        	cliPrintF("%4ld, ", TIM8->CCR1);
        	cliPrintF("%4ld, ", TIM2->CCR2);
        	cliPrintF("%4ld, ", TIM3->CCR1);
        	cliPrintF("%4ld, ", TIM3->CCR2);
        	cliPrintF("%4ld\n", TIM2->CCR1);

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'w': // Servo PWM Outputs
        	cliPrintF("%4ld, ", TIM5->CCR3);
        	cliPrintF("%4ld, ", TIM5->CCR2);
        	cliPrintF("%4ld\n", TIM5->CCR1);

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'x':
        	//logSync();
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'y': // ESC Calibration
        	escCalibration();

        	cliQuery = 'x';
        	break;

        ///////////////////////////////

        case 'z':
            cliPrintF("%8.4f, %8.4f, %8.4f, %8.4f\n", mxr9150XAxis(),
            		                                  mxr9150YAxis(),
                             	                      mxr9150ZAxis(),
                            	                      voltageMonitor);
            break;

        ///////////////////////////////

        case '1': // Turn high speed telemetry 1 on
        	highSpeedTelemDisable();
          	highSpeedTelem1Enabled = true;

        	cliQuery = 'x';
            break;

        ///////////////////////////////

        case '2': // Turn high speed telemetry 2 on
           	highSpeedTelemDisable();
           	highSpeedTelem2Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '3': // Turn high speed telemetry 3 on
           	highSpeedTelemDisable();
           	highSpeedTelem3Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '4': // Turn high speed telemetry 4 on
           	highSpeedTelemDisable();
           	highSpeedTelem4Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '5': // Turn high speed telemetry 5 on
           	highSpeedTelemDisable();
           	highSpeedTelem5Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '6': // Turn high speed telemetry 6 on
           	highSpeedTelemDisable();
           	highSpeedTelem6Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '7': // Turn high speed telemetry 7 on
           	highSpeedTelemDisable();
           	highSpeedTelem7Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '8': // Turn high speed telemetry 8 on
           	highSpeedTelemDisable();
           	highSpeedTelem8Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '9': // Turn high speed telemetry 9 on
           	highSpeedTelemDisable();
           	highSpeedTelem9Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '0': // Disable high speed telemetry
           	highSpeedTelemDisable();

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////

        case 'A': // Read Roll Rate PID Values
            readCliPID(ROLL_RATE_PID);
            cliPrint( "\nRoll Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch Rate PID Values
            readCliPID(PITCH_RATE_PID);
            cliPrint( "\nPitch Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw Rate PID Values
            readCliPID(YAW_RATE_PID);
            cliPrint( "\nYaw Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'D': // Read Roll Attitude PID Values
            readCliPID(ROLL_ATT_PID);
            cliPrint( "\nRoll Attitude PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'E': // Read Pitch Attitude PID Values
            readCliPID(PITCH_ATT_PID);
            cliPrint( "\nPitch Attitude PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'F': // Read Heading Hold PID Values
            readCliPID(HEADING_PID);
            cliPrint( "\nHeading PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'G': // Read nDot PID Values
            readCliPID(NDOT_PID);
            cliPrint( "\nnDot PID Received....\n" );

        	cliQuery = 'c';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'H': // Read eDot PID Values
            readCliPID(EDOT_PID);
            cliPrint( "\neDot PID Received....\n" );

            cliQuery = 'c';
          	validCliCommand = false;
          	break;

        ///////////////////////////////

        case 'I': // Read hDot PID Values
            readCliPID(HDOT_PID);
            cliPrint( "\nhDot PID Received....\n" );

          	cliQuery = 'c';
          	validCliCommand = false;
          	break;

       	///////////////////////////////

        case 'J': // Read n PID Values
            readCliPID(N_PID);
            cliPrint( "\nn PID Received....\n" );

            cliQuery = 'd';
            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'K': // Read e PID Values
            readCliPID(E_PID);
            cliPrint( "\ne PID Received....\n" );

            cliQuery = 'd';
            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'L': // Read h PID Values
            readCliPID(H_PID);
            cliPrint( "\nh PID Received....\n" );

            cliQuery = 'd';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'M': // MAX7456 CLI
           	max7456CLI();

           	cliQuery = 'x';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'N': // Mixer CLI
            mixerCLI();

            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'O': // Receiver CLI
            receiverCLI();

            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'P': // Sensor CLI
           	sensorCLI();

           	cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'Q': // GPS CLI
            gpsCLI();

            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'R': // Reset to Bootloader
        	cliPrint("Entering Bootloader....\n\n");
        	delay(100);
        	systemReset(true);
        	break;

        ///////////////////////////////

        case 'S': // Reset System
        	cliPrint("\nSystem Reseting....\n\n");
        	delay(100);
        	systemReset(false);
        	break;

        ///////////////////////////////

        case 'T': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'U': // EEPROM CLI
            eepromCLI();

            cliQuery = 'x';
         	validCliCommand = false;
         	break;

        ///////////////////////////////

        case 'V': // Reset EEPROM Parameters
            cliPrint( "\nEEPROM Parameters Reset....\n" );
            checkFirstTime(true);
            cliPrint("\nSystem Resetting....\n\n");
            delay(100);
            systemReset(false);
            break;

        ///////////////////////////////

        case 'W': // Write EEPROM Parameters
            cliPrint("\nWriting EEPROM Parameters....\n");
            writeEEPROM();

            cliQuery = 'x';
         	validCliCommand = false;
         	break;

        ///////////////////////////////

        case 'X': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'Y': // Not Used
            cliQuery = 'x';
            break;

        ///////////////////////////////

        case 'Z': // Not Used
            cliQuery = 'x';
            break;

        ///////////////////////////////

        case '?': // Command Summary
        	cliBusy = true;

        	cliPrint("\n");
   		    cliPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'g' 500 Hz Accels                          'G' Set nDot PID Data        GB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'h' 100 Hz Earth Axis Accels               'H' Set eDot PID Data        HB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'j' 10 hz Mag Data                         'J' Set n PID Data           JB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'k' Vertical Axis Variable                 'K' Set e PID Data           KB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'l' Attitudes                              'L' Set h PID Data           LB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		        validCliCommand = true;
   		        cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'m' GPS Data                               'M' MAX7456 CLI\n");
   		    cliPrint("'n' GPS Stats                              'N' Mixer CLI\n");
   		    cliPrint("'o' Not Used                               'O' Receiver CLI\n");
   		    cliPrint("'p' Not Used                               'P' Sensor CLI\n");
   		    cliPrint("'q' Not Used                               'Q' GPS CLI\n");
   		    cliPrint("'r' Mode States                            'R' Reset and Enter Bootloader\n");
   		    cliPrint("'s' Raw Receiver Commands                  'S' Reset\n");
   		    cliPrint("'t' Processed Receiver Commands            'T' Not Used\n");
   		    cliPrint("'u' Command In Detent Discretes            'U' EEPROM CLI\n");
   		    cliPrint("'v' Motor PWM Outputs                      'V' Reset EEPROM Parameters\n");
   		    cliPrint("'w' Servo PWM Outputs                      'W' Write EEPROM Parameters\n");
   		    cliPrint("'x' Terminate Serial Communication         'X' Not Used\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		    	validCliCommand = true;
   		    	cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'y' ESC Calibration                        'Y' Not Used\n");
   		    cliPrint("'z' ADC Values                             'Z' Not Used\n");
   		    cliPrint("'1' High Speed Telemetry 1 Enable\n");
   		    cliPrint("'2' High Speed Telemetry 2 Enable\n");
   		    cliPrint("'3' High Speed Telemetry 3 Enable\n");
   		    cliPrint("'4' High Speed Telemetry 4 Enable\n");
   		    cliPrint("'5' High Speed Telemetry 5 Enable\n");
   		    cliPrint("'6' High Speed Telemetry 6 Enable\n");
   		    cliPrint("'7' High Speed Telemetry 7 Enable\n");
   		    cliPrint("'8' High Speed Telemetry 8 Enable\n");
   		    cliPrint("'9' High Speed Telemetry 9 Enable\n");
   		    cliPrint("'0' High Speed Telemetry Disable           '?' Command Summary\n");
   		    cliPrint("\n");

  		    cliQuery = 'x';
  		    cliBusy = false;
   		    break;

            ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
