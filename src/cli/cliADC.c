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

    uint8_t  tempCells    = 0;
    uint8_t  tempCM		  = 2;
    uint16_t tempMax      = 0;
    uint16_t tempMin      = 0;
    uint8_t  tempPin      = 0;
	uint8_t  tempWarn     = 0;

	float 	 tempBias	  = 0.0f;
    float	 tempScale	  = 0.0f;

	float    tempLow      = 0.0f;
	float    tempVeryLow  = 0.0f;
	float    tempMaxLow   = 0.0f;

    cliBusy = true;

    cliPortPrint("\nEntering ADC CLI....\n\n");

    while(true)
    {
        cliPortPrint("ADC CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    adcQuery = cliPortRead();

		cliPortPrint("\n");

		switch(adcQuery)
		{
            ///////////////////////////

            case 'a': // ADC Data
            	cliPortPrintF("\n");
            	cliPortPrintF("AGL Pin:-------------------    %d\n",            eepromConfig.aglPin);
            	cliPortPrintF("AGL Scale:                  %9.4f\n",            eepromConfig.aglScale);
            	cliPortPrintF("AGL Bias:------------------ %9.4f\n",            eepromConfig.aglBias);

            	cliPortPrintF("Current Monitoring:            %s\n", eepromConfig.currentMonitoring ? "Enabled " : "Disabled");

				cliPortPrintF("Current Monitor Pin:-------    %d\n",            eepromConfig.currentMonitorPin);
				cliPortPrintF("Current MonitorDivider:     %9.4f\n",            eepromConfig.currentMonitorScale);
				cliPortPrintF("Current Monitor Bias:------ %9.4f\n",            eepromConfig.currentMonitorBias);
				cliPortPrintF("RSSI Pin:                      %1d\n",           eepromConfig.rssiPin);
				cliPortPrintF("RSSI Min:-----------------  %4d\n",              eepromConfig.rssiMin);
				cliPortPrintF("RSSI Max:                   %4d\n",              eepromConfig.rssiMax);
				cliPortPrintF("RSSI Warning %%:------------   %2d\n",           eepromConfig.rssiWarning);
            	cliPortPrintF("Voltage Monitor Pin:           %d\n",            eepromConfig.voltageMonitorPin);
				cliPortPrintF("Voltage Monitor Scale:----- %9.4f\n",            eepromConfig.voltageMonitorScale);
				cliPortPrintF("Voltage Monitor Bias:       %9.4f\n",            eepromConfig.voltageMonitorBias);

				cliPortPrintF("Battery Cells:-------------   %2d\n",            eepromConfig.batteryCells);
				cliPortPrintF("Battery Low Setpoint:          %4.2f volts\n",   eepromConfig.batteryLow);
                cliPortPrintF("Battery Very Low Setpoint:-    %4.2f volts\n",   eepromConfig.batteryVeryLow);
                cliPortPrintF("Battery Max Low Setpoint:      %4.2f volts\n\n", eepromConfig.batteryMaxLow);

                validQuery = false;
                break;

            ///////////////////////////

        	case 'x':
			    cliPortPrint("\nExiting ADC CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

			///////////////////////////

			case 'A': // Set AGL Configuration
				tempPin  	= (uint8_t)readFloatCLI();
				tempScale	= readFloatCLI();
				tempBias 	= readFloatCLI();

				if ((tempPin < 1) || (tempPin > 6) || (tempScale == 0.0f))
				{
					cliPortPrintF("\nAGL Pin or AGL Scale entered incorrectly\n");
					cliPortPrintF("%3d, %9.4f, %9.4f\n", tempPin, tempScale, tempBias);
					cliPortPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = false;
					break;
				}

				eepromConfig.aglPin   = tempPin;
				eepromConfig.aglScale = tempScale;
				eepromConfig.aglBias  = tempBias;

				adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'B': // Set battery Cells and Monitor Trip Points
				tempCells   = (uint8_t)readFloatCLI();
				tempLow     = readFloatCLI();
			    tempVeryLow = readFloatCLI();
			    tempMaxLow  = readFloatCLI();

				if ((tempCells < 1) || (tempCells > 12))
				{
					cliPortPrintF("\nNumber of Cells set incorrectly\n");
					cliPortPrintF("You entered %2d\n", tempCells);
					cliPortPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = true;
					break;
				}

				eepromConfig.batteryCells = tempCells;

				eepromConfig.batteryLow     = tempLow;
				eepromConfig.batteryVeryLow = tempVeryLow;;
				eepromConfig.batteryMaxLow  = tempMaxLow;

				thresholds[BATTERY_LOW].value      = eepromConfig.batteryLow;
                thresholds[BATTERY_VERY_LOW].value = eepromConfig.batteryVeryLow;
                thresholds[BATTRY_MAX_LOW].value   = eepromConfig.batteryMaxLow;

                adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'C': // Set current monitoring extended configuration
				tempCM    = (uint8_t)readFloatCLI();
				tempPin	  = (uint8_t)readFloatCLI();
				tempScale = readFloatCLI();
				tempBias  = readFloatCLI();

				if (((tempCM != 0) && (tempCM != 1)) || (tempPin < 1) || (tempPin > 6) || (tempScale = 0.0f))
				{
					cliPortPrintF("\nbatteryExtended, CPin entered incorrectly, or CScale not set\n");
					cliPortPrintF("%d, %d, %3.2f, %2.2f\n", tempCM, tempPin, tempScale, tempBias);
					cliPortPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = true;
					break;
				}
				eepromConfig.currentMonitoring   = tempCM;
				eepromConfig.currentMonitorPin	 = tempPin;
				eepromConfig.currentMonitorScale = tempScale;
				eepromConfig.currentMonitorBias	 = tempBias;

				adcQuery = 'a';
				validQuery = true;
				break;

	        ///////////////////////////

			case 'R': // RSSI pin/min/max/warning
				tempPin	 = readFloatCLI();
				tempMin  = readFloatCLI();
				tempMax  = readFloatCLI();
				tempWarn = readFloatCLI();

				if ((tempPin < 1) || (tempPin > 6))
				{
					cliPortPrintF("Invalid RSSI Pin number, valid numbers are 1-6\n");
					cliPortPrintF("You entered %2d, please try again\n", tempPin);
					adcQuery = '?';
					validQuery = false;
					break;
				}

				eepromConfig.rssiPin     = tempPin;
				eepromConfig.rssiMin     = tempMin;
				eepromConfig.rssiMax     = tempMax;
				eepromConfig.rssiWarning = tempWarn;

				adcQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'V': // Set voltage monitoring configuration
				tempPin  	= (uint8_t)readFloatCLI();
				tempScale	= readFloatCLI();
				tempBias 	= readFloatCLI();

				if ((tempPin < 1) || (tempPin > 7) || (tempScale == 0.0f))
				{
					cliPortPrintF("\nVPin or VScale entered incorrectly\n");
					cliPortPrintF("%3d, %9.4f, %9.4f\n", tempPin, tempScale, tempBias);
					cliPortPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n\n");
					adcQuery = '?';
					validQuery = false;
					break;
				}

				eepromConfig.voltageMonitorPin   = tempPin;
				eepromConfig.voltageMonitorScale = tempScale;
				eepromConfig.voltageMonitorBias  = tempBias;

				adcQuery = 'a';
				validQuery = true;
				break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPortPrint("\n");
			   	cliPortPrint("'a' Display ADC Data                       'A' Set AGL Config                        APin;Min;Max;Warning\n");
			   	cliPortPrint("                                           'B' Set Battery Cells/Warnings            BCells;Low;VeryLow;MaxLow\n");
			   	cliPortPrint("                                           'C' Set Current Monitor Config            CCM;Pin;Scale;Bias\n");
			   	cliPortPrint("                                               where CM = 1 (Enabled) or 0 (Disabled)\n");
				cliPortPrint("                                           'R' Set RSSI Config                       RPin;Min;Max;Warning\n");
				cliPortPrint("                                           'V' Set Voltage Monitor Config            VPin;Scale;Bias\n");
				cliPortPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Sensor CLI                        '?' Command Summary\n\n");

			   	validQuery = false;
	    	    break;

	    	///////////////////////////
	    }
	}

}

