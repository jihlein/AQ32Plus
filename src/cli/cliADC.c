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
// ADC CLI
///////////////////////////////////////////////////////////////////////////////

void adcCLI()
{
    uint8_t  adcQuery = 'x';
    uint8_t  validQuery = false;

    uint8_t  tempPin      = 0;
	uint16_t tempMin      = 0;
	uint16_t tempMax      = 0;
	uint8_t  tempWarn     = 0;
	uint8_t	 tempCells	  = 0;
    uint8_t  tempE		  = 2;
	uint8_t  tempVPin	  = 0;
	uint8_t  tempCPin	  = 0;
	float	 tempVScale	  = 0.0f;
	float 	 tempCScale	  = 0.0f;
	float 	 tempVBias	  = 0.0f;
	float	 tempCBias	  = 0.0f;
	float    tempVWarning = 0.0f;

    cliBusy = true;

    cliPrint("\nEntering ADC CLI....\n\n");

    while(true)
    {
        cliPrint("ADC CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    adcQuery = cliRead();

		cliPrint("\n");

		switch(adcQuery)
		{
            ///////////////////////////

            case 'a': // ADC Data
            	cliPrintF("\n");
            	cliPrintF("Battery Cells:               %2d\n",    eepromConfig.batteryCells);
                cliPrintF("Battery Voltage Pin:----------%d\n",    eepromConfig.batteryVPin);
				cliPrintF("Battery Voltage Divider:   %9.4f\n",    eepromConfig.batteryVScale);
				cliPrintF("Battery Voltage Bias:------%9.4f\n",    eepromConfig.batteryVBias);
				cliPrintF("Battery Volt/Cell Warning     %3.1f\n", eepromConfig.batteryVWarning);
				cliPrintF("Battery Extended:-------------%d\n",    eepromConfig.batteryExtended);
				cliPrintF("Battery Current Pin:          %d\n",    eepromConfig.batteryCPin);
				cliPrintF("Battery Current Divider:   %9.4f\n",    eepromConfig.batteryCScale);
				cliPrintF("Battery Current Bias:------%9.4f\n",    eepromConfig.batteryCBias);
				cliPrintF("RSSI Pin:                     %1d\n",   eepromConfig.RSSIPin);
				cliPrintF("RSSI Min:------------------%4d\n",      eepromConfig.RSSIMin);
				cliPrintF("RSSI Max:                  %4d\n",      eepromConfig.RSSIMax);
				cliPrintF("RSSI Warning %:---------------%2d\n\n", eepromConfig.RSSIWarning);

                cliPrintF("Battery Low Setpoint:      %4.2f volts\n",   eepromConfig.batteryLow);
                cliPrintF("Battery Very Low Setpoint: %4.2f volts\n",   eepromConfig.batteryVeryLow);
                cliPrintF("Battery Max Low Setpoint:  %4.2f volts\n\n", eepromConfig.batteryMaxLow);

                validQuery = false;
                break;

            ///////////////////////////

        	case 'x':
			    cliPrint("\nExiting ADC CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

			case 'A': // RSSI pin/min/max/warning
				tempPin	 = readFloatCLI();
				tempMin  = readFloatCLI();
				tempMax  = readFloatCLI();
				tempWarn = readFloatCLI();

				if ((tempPin < 2) || (tempPin > 6))
				{
					cliPrintF("Invalid RSSI Pin number, valid numbers are 2-6\n");
					cliPrintF("You entered %2d, please try again\n", tempPin);
					adcQuery = '?';
					validQuery = false;
					break;
				}

				eepromConfig.RSSIPin     = tempPin;
				eepromConfig.RSSIMin     = tempMin;
				eepromConfig.RSSIMax     = tempMax;
				eepromConfig.RSSIWarning = tempWarn;

				adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'B': // Set battery Cells and Voltage Warning Level
				tempCells = (uint8_t)readFloatCLI();
				tempVWarning = readFloatCLI();

				if ((tempVWarning <= 0) || (tempVWarning > 4.2) || (tempCells < 1) || (tempCells > 12))
				{
					cliPrintF("\nCells or Voltage Warning set incorrectly\n");
					cliPrintF("You entered %2d, %2d\n", tempCells, tempVWarning);
					cliPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = true;
					break;
				}

				eepromConfig.batteryCells = tempCells;
				eepromConfig.batteryVWarning = tempVWarning;

				adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'C': // Set Battery monitoring extended configuration
				tempE		= (uint8_t)readFloatCLI();
				tempCPin	= (uint8_t)readFloatCLI();
				tempCScale	= readFloatCLI();
				tempCBias 	= readFloatCLI();

				if (((tempE != 0) && (tempE != 1)) || (tempCPin < 2) || (tempCPin > 6) || (tempCScale = 0.0f))
				{
					cliPrintF("\nbatteryExtended, CPin entered incorrectly, or CScale not set\n");
					cliPrintF("%d, %d, %3.2f, %2.2f\n", tempE, tempCPin, tempCScale, tempCBias);
					cliPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = true;
					break;
				}
				eepromConfig.batteryExtended	= tempE;
				eepromConfig.batteryCPin		= tempCPin;
				eepromConfig.batteryCScale		= tempCScale;
				eepromConfig.batteryCBias		= tempCBias;

				adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'D': // Set Battery monitoring configuration
				tempVPin	= (uint8_t)readFloatCLI();
				tempVScale	= readFloatCLI();
				tempVBias 	= readFloatCLI();
				tempCells	= (uint8_t)readFloatCLI();

				if ((tempVPin < 2) || (tempVPin > 7) || (tempVScale == 0.0f))
				{
					cliPrintF("\nVPin, VScale, or Cells entered incorrectly\n");
					cliPrintF("%3d, %9.4f, %9.4f, %3d", tempVPin, tempVScale, tempVBias, tempCells);
					cliPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = false;
					break;
				}

				eepromConfig.batteryVPin   = tempVPin;
				eepromConfig.batteryVScale = tempVScale;
				eepromConfig.batteryVBias  = tempVBias;
				eepromConfig.batteryCells  = tempCells;

				adcQuery = 'a';
				validQuery = true;
				break;

            ///////////////////////////

            case 'M': // Set Voltage Monitor Trip Points
                eepromConfig.batteryLow     = readFloatCLI();
                eepromConfig.batteryVeryLow = readFloatCLI();
                eepromConfig.batteryMaxLow  = readFloatCLI();

                thresholds[BATTERY_LOW].value      = eepromConfig.batteryLow;
                thresholds[BATTERY_VERY_LOW].value = eepromConfig.batteryVeryLow;
                thresholds[BATTRY_MAX_LOW].value   = eepromConfig.batteryMaxLow;

                adcQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Display ADC Data                       'A' Set RSSI Pin/Min/Max/Warning         RPin;Min;Max;Warning\n");
			   	cliPrint("                                           'B' Set Battery Cells/Volt Warning       TCells;Warning\n");
			   	cliPrint("                                           'C' Set Battery Current Config           UExtended;CPin;CScale;CBias\n");
			   	cliPrint("                                               where extended = 1 (on) or 0 (off).  Must be 1 to measure current\n");
				cliPrint("                                           'D' Set Battery Voltage Config           VVPin;VScale;VBias\n");
			   	cliPrint("                                           'M' Set Voltage Monitor Trip Points      Mlow;veryLow;maxLow\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Sensor CLI                        '?' Command Summary\n\n");

			   	validQuery = false;
	    	    break;

	    	///////////////////////////
	    }
	}

}

