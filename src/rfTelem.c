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

uint8_t rfBusy = false;

static volatile uint8_t rfQueryType;
static volatile uint8_t validRFCommand = false;

///////////////////////////////////////////////////////////////////////////////
// Read Float from RF Comm
///////////////////////////////////////////////////////////////////////////////

float readFloatRF(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if (telemetryAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = telemetryRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from RF Comm
///////////////////////////////////////////////////////////////////////////////

void readRFPID(unsigned char PIDid)
{
  struct PIDdata* pid = &eepromConfig.PID[PIDid];

  pid->B              = readFloatRF();
  pid->P              = readFloatRF();
  pid->I              = readFloatRF();
  pid->D              = readFloatRF();
  pid->windupGuard    = readFloatRF();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     = (uint8_t)readFloatRF();
}

///////////////////////////////////////////////////////////////////////////////
// RF Telem Communication
///////////////////////////////////////////////////////////////////////////////

void rfCom(void)
{
    if (telemetryAvailable() && !validRFCommand)
    	rfQueryType = telemetryRead();

    switch (rfQueryType)
    {
        ///////////////////////////////

    case 'a': // Rate PIDs
        telemetryPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_RATE_PID].B,
                                                                                     eepromConfig.PID[ROLL_RATE_PID].P,
            		                                                                 eepromConfig.PID[ROLL_RATE_PID].I,
            		                                                                 eepromConfig.PID[ROLL_RATE_PID].D,
            		                                                                 eepromConfig.PID[ROLL_RATE_PID].windupGuard,
            		                                                                 eepromConfig.PID[ROLL_RATE_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_RATE_PID].B,
                        		                                                     eepromConfig.PID[PITCH_RATE_PID].P,
            		                                                                 eepromConfig.PID[PITCH_RATE_PID].I,
            		                                                                 eepromConfig.PID[PITCH_RATE_PID].D,
            		                                                                 eepromConfig.PID[PITCH_RATE_PID].windupGuard,
            		                                                                 eepromConfig.PID[PITCH_RATE_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[YAW_RATE_PID].B,
                         		                                                     eepromConfig.PID[YAW_RATE_PID].P,
            		                                                                 eepromConfig.PID[YAW_RATE_PID].I,
            		                                                                 eepromConfig.PID[YAW_RATE_PID].D,
            		                                                                 eepromConfig.PID[YAW_RATE_PID].windupGuard,
            		                                                                 eepromConfig.PID[YAW_RATE_PID].dErrorCalc ? "Error" : "State");
        rfQueryType = 'x';
        validRFCommand = false;
        break;

    ///////////////////////////////

    case 'b': // Attitude PIDs
        telemetryPrintF("\nRoll Attitude PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_ATT_PID].B,
          		                                                                         eepromConfig.PID[ROLL_ATT_PID].P,
           		                                                                         eepromConfig.PID[ROLL_ATT_PID].I,
           		                                                                         eepromConfig.PID[ROLL_ATT_PID].D,
           		                                                                         eepromConfig.PID[ROLL_ATT_PID].windupGuard,
           		                                                                         eepromConfig.PID[ROLL_ATT_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("Pitch Attitude PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_ATT_PID].B,
           		                                                                         eepromConfig.PID[PITCH_ATT_PID].P,
           		                                                                         eepromConfig.PID[PITCH_ATT_PID].I,
           		                                                                         eepromConfig.PID[PITCH_ATT_PID].D,
           		                                                                         eepromConfig.PID[PITCH_ATT_PID].windupGuard,
           		                                                                         eepromConfig.PID[PITCH_ATT_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("Heading PID:        %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[HEADING_PID].B,
           		                                                                         eepromConfig.PID[HEADING_PID].P,
           		                                                                         eepromConfig.PID[HEADING_PID].I,
           		                                                                         eepromConfig.PID[HEADING_PID].D,
           		                                                                         eepromConfig.PID[HEADING_PID].windupGuard,
           		                                                                         eepromConfig.PID[HEADING_PID].dErrorCalc ? "Error" : "State");
        rfQueryType = 'x';
        validRFCommand = false;
        break;

    ///////////////////////////////

    case 'c': // Velocity PIDs
        telemetryPrintF("\nnDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[NDOT_PID].B,
           		                                                                eepromConfig.PID[NDOT_PID].P,
           		                                                                eepromConfig.PID[NDOT_PID].I,
           		                                                                eepromConfig.PID[NDOT_PID].D,
           		                                                                eepromConfig.PID[NDOT_PID].windupGuard,
           		                                                                eepromConfig.PID[NDOT_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("eDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[EDOT_PID].B,
           		                                                                eepromConfig.PID[EDOT_PID].P,
           		                                                                eepromConfig.PID[EDOT_PID].I,
           		                                                                eepromConfig.PID[EDOT_PID].D,
           		                                                                eepromConfig.PID[EDOT_PID].windupGuard,
           		                                                                eepromConfig.PID[EDOT_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("hDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[HDOT_PID].B,
           		                                                                eepromConfig.PID[HDOT_PID].P,
           		                                                                eepromConfig.PID[HDOT_PID].I,
           		                                                                eepromConfig.PID[HDOT_PID].D,
           		                                                                eepromConfig.PID[HDOT_PID].windupGuard,
           		                                                                eepromConfig.PID[HDOT_PID].dErrorCalc ? "Error" : "State");
        rfQueryType = 'x';
        validRFCommand = false;
        break;

    ///////////////////////////////

    case 'd': // Position PIDs
        telemetryPrintF("\nN PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[N_PID].B,
           		                                                             eepromConfig.PID[N_PID].P,
           		                                                             eepromConfig.PID[N_PID].I,
           		                                                             eepromConfig.PID[N_PID].D,
           		                                                             eepromConfig.PID[N_PID].windupGuard,
           		                                                             eepromConfig.PID[N_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("E PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[E_PID].B,
           		                                                             eepromConfig.PID[E_PID].P,
           		                                                             eepromConfig.PID[E_PID].I,
           		                                                             eepromConfig.PID[E_PID].D,
           		                                                             eepromConfig.PID[E_PID].windupGuard,
           		                                                             eepromConfig.PID[E_PID].dErrorCalc ? "Error" : "State");

        telemetryPrintF("h PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[H_PID].B,
           		                                                             eepromConfig.PID[H_PID].P,
           		                                                             eepromConfig.PID[H_PID].I,
           		                                                             eepromConfig.PID[H_PID].D,
           		                                                             eepromConfig.PID[H_PID].windupGuard,
           		                                                             eepromConfig.PID[H_PID].dErrorCalc ? "Error" : "State");
        rfQueryType = 'x';
        validRFCommand = false;
        break;

        ///////////////////////////////

        case 'x':
        	break;

        ///////////////////////////////

        case '1': // Turn high speed telemetry 1 on
        	highSpeedTelemDisable();
          	highSpeedTelem1Enabled = true;

        	rfQueryType = 'x';
            break;

        ///////////////////////////////

        case '2': // Turn high speed telemetry 2 on
           	highSpeedTelemDisable();
           	highSpeedTelem2Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '3': // Turn high speed telemetry 3 on
           	highSpeedTelemDisable();
           	highSpeedTelem3Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '4': // Turn high speed telemetry 4 on
           	highSpeedTelemDisable();
           	highSpeedTelem4Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '5': // Turn high speed telemetry 5 on
           	highSpeedTelemDisable();
           	highSpeedTelem5Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '6': // Turn high speed telemetry 6 on
           	highSpeedTelemDisable();
           	highSpeedTelem6Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '7': // Turn high speed telemetry 7 on
           	highSpeedTelemDisable();
           	highSpeedTelem7Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '8': // Turn high speed telemetry 8 on
           	highSpeedTelemDisable();
           	highSpeedTelem8Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '9': // Turn high speed telemetry 9 on
           	highSpeedTelemDisable();
           	highSpeedTelem9Enabled = true;

            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        case '0': // Disable high speed telemetry 0
           	highSpeedTelemDisable();
            
            rfQueryType = 'x';
           	break;

        ///////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////

        case 'A': // Read Roll Rate PID Values
            readRFPID(ROLL_RATE_PID);
            telemetryPrint( "\nRoll Rate PID Received....\n" );

        	rfQueryType = 'a';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch Rate PID Values
            readRFPID(PITCH_RATE_PID);
            telemetryPrint( "\nPitch Rate PID Received....\n" );

        	rfQueryType = 'a';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw Rate PID Values
            readRFPID(YAW_RATE_PID);
            telemetryPrint( "\nYaw Rate PID Received....\n" );

        	rfQueryType = 'a';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'D': // Read Roll Attitude PID Values
            readRFPID(ROLL_ATT_PID);
            telemetryPrint( "\nRoll Attitude PID Received....\n" );

        	rfQueryType = 'b';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'E': // Read Pitch Attitude PID Values
            readRFPID(PITCH_ATT_PID);
            telemetryPrint( "\nPitch Attitude PID Received....\n" );

        	rfQueryType = 'b';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'F': // Read Heading Hold PID Values
            readRFPID(HEADING_PID);
            telemetryPrint( "\nHeading PID Received....\n" );

        	rfQueryType = 'b';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'G': // Read nDot PID Values
            readRFPID(NDOT_PID);
            telemetryPrint( "\nnDot PID Received....\n" );

        	rfQueryType = 'c';
        	validRFCommand = true;
        	break;

        ///////////////////////////////

        case 'H': // Read eDot PID Values
            readRFPID(EDOT_PID);
            telemetryPrint( "\neDot PID Received....\n" );

            rfQueryType = 'c';
            validRFCommand = true;
            break;

        ///////////////////////////////

        case 'I': // Read hDot PID Values
            readRFPID(HDOT_PID);
            telemetryPrint( "\nhDot PID Received....\n" );

          	rfQueryType = 'c';
          	validRFCommand = true;
          	break;

       	///////////////////////////////

        case 'J': // Read n PID Values
            readRFPID(N_PID);
            telemetryPrint( "\nn PID Received....\n" );

            rfQueryType = 'd';
            validRFCommand = true;
            break;

        ///////////////////////////////

        case 'K': // Read e PID Values
            readRFPID(E_PID);
            telemetryPrint( "\ne PID Received....\n" );

            rfQueryType = 'd';
            validRFCommand = true;
            break;

        ///////////////////////////////

        case 'L': // Read h PID Values
            readRFPID(H_PID);
            telemetryPrint( "\nh PID Received....\n" );

            rfQueryType = 'd';
            validRFCommand = true;
            break;

        ///////////////////////////////

        case 'W': // Write EEPROM Parameters
            telemetryPrint("\nWriting EEPROM Parameters....\n");
            writeEEPROM();

            rfQueryType = 'x';
         	break;

        ///////////////////////////////

        case '?': // Command Summary

        	telemetryPrint("\n");
        	telemetryPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'x' Terminate Serial Communication         'E' Set Pitch Att PID Data   EB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'1' High Speed Telemetry 1 Enable          'F' Set Hdg Hold PID Data    FB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'2' High Speed Telemetry 2 Enable          'G' Set nDot PID Data        GB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'3' High Speed Telemetry 3 Enable          'H' Set eDot PID Data        HB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'4' High Speed Telemetry 4 Enable          'I' Set hDot PID Data        IB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'5' High Speed Telemetry 5 Enable          'J' Set n PID Data           JB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'6' High Speed Telemetry 6 Enable          'K' Set e PID Data           KB;P;I;D;windupGuard;dErrorCalc\n");
   		    telemetryPrint("'7' High Speed Telemetry 7 Enable          'L' Set h PID Data           LB;P;I;D;windupGuard;dErrorCalc\n");
		   	telemetryPrint("'8' High Speed Telemetry 8 Enable          'W' Write EEPROM Parameters\n");
		   	telemetryPrint("'9' High Speed Telemetry 9 Enable\n");
		   	telemetryPrint("'0' High Speed Telemetry Disable           '?' Command Summary\n");
   		    telemetryPrint("\n");

  		    rfQueryType = 'x';
  		    break;

            ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
