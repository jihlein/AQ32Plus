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
// Telemetry CLI
///////////////////////////////////////////////////////////////////////////////

void telemetryCLI()
{
    uint8_t  telemetryQuery = 'x';
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering Telemetry CLI....\n\n");

    while(true)
    {
        cliPrint("Telemetry CLI -> ");

            while ((cliAvailable() == false) && (validQuery == false));

	    if (validQuery == false)
		telemetryQuery = cliRead();

	    cliPrint("\n");

	    switch(telemetryQuery)

	    {
            ///////////////////////////

            case 'a': // Telemetry Configuration
                cliPrint("\nTelemetry Configuration:\n");

                cliPrint("    Telemetry Set 1: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 1 ?   "  Active" : "Inactive");

                cliPrint("    Telemetry Set 2: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 2 ?   "  Active" : "Inactive");

                cliPrint("    Telemetry Set 3: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 4 ?   "  Active" : "Inactive");

                cliPrint("    Telemetry Set 4: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 8 ?   "  Active" : "Inactive");

                cliPrint("    Telemetry Set 5: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 16 ?  "  Active" : "Inactive");

                cliPrint("    Telemetry Set 6: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 32 ?  "  Active" : "Inactive");

                cliPrint("    Telemetry Set 7: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 64 ?  "  Active" : "Inactive");

                cliPrint("    Telemetry Set 8: ");
                cliPrintF("%s\n", eepromConfig.activeTelemetry == 128 ? "  Active" : "Inactive");

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Turn all Telemetry Off
                eepromConfig.activeTelemetry = 0;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'c': // Toggle Telemetry Set 1 State
                eepromConfig.activeTelemetry = 1;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'd': // Toggle Telemetry Set 2 State
                eepromConfig.activeTelemetry = 2;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'e': // Toggle Telemetry Set 3 State
                eepromConfig.activeTelemetry = 4;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'f': // Toggle Telemetry Set 4 State
                eepromConfig.activeTelemetry = 8;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'g': // Toggle Telemetry Set 5 State
                eepromConfig.activeTelemetry = 16;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'h': // Toggle Telemetry Set 6 State
                eepromConfig.activeTelemetry = 32;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'i': // Toggle Telemetry Set 7 State
                eepromConfig.activeTelemetry = 64;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'j': // Toggle Telemetry Set 8 State
                eepromConfig.activeTelemetry = 128;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting Telemetry CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

            ///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Telemetry Configuration Data\n");
   		        cliPrint("'b' Turn all Telemetry Off\n");
			   	cliPrint("'c' Toggle Telemetry Set 1 State\n");
			   	cliPrint("'d' Toggle Telemetry Set 2 State\n");
			   	cliPrint("'e' Toggle Telemetry Set 3 State\n");
			   	cliPrint("'f' Toggle Telemetry Set 4 State\n");
   		        cliPrint("'g' Toggle Telemetry Set 5 State\n");
   		        cliPrint("'h' Toggle Telemetry Set 6 State\n");
   		        cliPrint("'i' Toggle Telemetry Set 7 State\n");
   		        cliPrint("'j' Toggle Telemetry Set 8 State\n");
   		        cliPrint("                                           'W' Write EEPROM Parameters\n");
   		        cliPrint("'x' Exit Telemetry CLI                     '?' Command Summary\n\n");
   		        break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
