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
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
	char     rcOrderString[13];
    float    tempFloat;
    uint8_t  tempChannels = 0;
    uint16_t tempMax      = 0;
    uint16_t tempMin      = 0;
    uint8_t  tempPin      = 0;
	uint8_t  tempWarn     = 0;

    uint8_t  index;
    uint8_t  receiverQuery = 'x';
    uint8_t  validQuery    = false;

    cliBusy = true;

    cliPortPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPortPrint("Receiver CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliPortRead();

		cliPortPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPortPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PARALLEL_PWM:
                        cliPortPrint("Parallel\n");
                        break;
                    case SERIAL_PWM:
                        cliPortPrint("Serial\n");
                        break;
                    case SPEKTRUM:
                        cliPortPrint("Spektrum\n");
                        break;
		        }

            	if (eepromConfig.receiverType == SERIAL_PWM)
            		tempChannels = eepromConfig.serialChannels;
            	else
            		tempChannels = 8;

            	cliPortPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < tempChannels; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPortPrint(rcOrderString);  cliPortPrint("\n");
                cliPortPrintF("Number of serial PWM channels   %2d\n",    eepromConfig.serialChannels);
                cliPortPrintF("Spektrum Resolution:            %s\n",     eepromConfig.spektrumHires ? "11 Bit Mode" : "10 Bit Mode");
                cliPortPrintF("Number of Spektrum Channels:    %2d\n",    eepromConfig.spektrumChannels);
                cliPortPrintF("Mid Command:                    %4ld\n",   (uint16_t)eepromConfig.midCommand);
				cliPortPrintF("Min Check:                      %4ld\n",   (uint16_t)eepromConfig.minCheck);
				cliPortPrintF("Max Check:                      %4ld\n",   (uint16_t)eepromConfig.maxCheck);
				cliPortPrintF("Min Throttle:                   %4ld\n",   (uint16_t)eepromConfig.minThrottle);
				cliPortPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)eepromConfig.maxThrottle);

				tempFloat = eepromConfig.rollAndPitchRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Roll and Pitch Rate Cmd:    %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.yawRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Yaw Rate Cmd:               %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
                cliPortPrintF("Max Attitude Cmd:               %6.2f Degrees\n\n", tempFloat);

				cliPortPrintF("Arm Delay Count:                %3d Frames\n",   eepromConfig.armCount);
				cliPortPrintF("Disarm Delay Count:             %3d Frames\n\n", eepromConfig.disarmCount);

				cliPortPrintF("RSSI via PPM or ADC:            %s",             eepromConfig.rssiPPM ? "PPM\n" : "ADC\n");
				cliPortPrintF("RSSI Pin:                       %1d\n",          eepromConfig.rssiPin);
				cliPortPrintF("RSSI Min:-----------------   %4d\n",             eepromConfig.rssiMin);
				cliPortPrintF("RSSI Max:                    %4d\n",             eepromConfig.rssiMax);
				cliPortPrintF("RSSI Warning %%:------------   %2d\n",           eepromConfig.rssiWarning);

				validQuery = false;
				break;

            ///////////////////////////

            case 'b': // Read Max Rate Values
                eepromConfig.rollAndPitchRateScaling = readFloatCLI() / 180000.0f * PI;
                eepromConfig.yawRateScaling          = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatCLI() / 180000 * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            case 'r': // Toggle RSSI between ADC and PPM
				eepromConfig.rssiPPM = !eepromConfig.rssiPPM;
				if (eepromConfig.rssiPPM)
				{									// automatically adjust the settings
					eepromConfig.rssiPin = 9;
					eepromConfig.rssiMin = eepromConfig.minCheck;
					eepromConfig.rssiMax = eepromConfig.maxCheck;
				}
				else
				{
					eepromConfig.rssiPin = 5; // default from config.c
					eepromConfig.rssiMin = 10;
					eepromConfig.rssiMax = 3450;
				}
				eepromConfig.rssiWarning = 25;

				receiverQuery = 'a';
				validQuery = true;
				break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read RX Input Type
                eepromConfig.receiverType = (uint8_t)readFloatCLI();
			    cliPortPrint( "\nReceiver Type Changed....\n");

			    cliPortPrint("\nSystem Resetting....\n");
			    delay(100);
			    writeEEPROM();
			    systemReset(false);

		        break;

            ///////////////////////////

            case 'B': // Read RC Control Order
				readStringCLI( rcOrderString, 12 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Read Spektrum Resolution
                eepromConfig.spektrumHires = (uint8_t)readFloatCLI();

                if (eepromConfig.spektrumHires)
                {
        		    // 11 bit frames
        		    spektrumChannelShift = 3;
        		    spektrumChannelMask  = 0x07;
        		}
        		else
        		{
        		    // 10 bit frames
        		    spektrumChannelShift = 2;
        		    spektrumChannelMask  = 0x03;
        		}

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read Number of Spektrum Channels
                eepromConfig.spektrumChannels = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read RC Control Points
                eepromConfig.midCommand   = readFloatCLI();
    	        eepromConfig.minCheck     = readFloatCLI();
    		    eepromConfig.maxCheck     = readFloatCLI();
    		    eepromConfig.minThrottle  = readFloatCLI();
    		    eepromConfig.maxThrottle  = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Arm/Disarm Counts
                eepromConfig.armCount    = (uint8_t)readFloatCLI();
    	        eepromConfig.disarmCount = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read number of PPM Channels
            	tempChannels = (uint8_t)readFloatCLI();
            	if ((tempChannels < 8) || (tempChannels > 12))
            	{
					cliPortPrintF("\nValid number of channels are 8 to 12\n");
					cliPortPrintF("You entered %2d\n\n", tempChannels);
            	}
            	else
            		eepromConfig.serialChannels = tempChannels;

				receiverQuery = 'a';
            	validQuery = true;
            	break;

			///////////////////////////

			case 'R': // RSSI pin/min/max/warning
				tempPin	 = readFloatCLI();
				tempMin  = readFloatCLI();
				tempMax  = readFloatCLI();
				tempWarn = readFloatCLI();

				if (eepromConfig.rssiPPM)
				{
					if ((tempPin < 0) || (tempPin > eepromConfig.serialChannels))
					{
						cliPortPrintF("Invalid RSSI PPM channel number, valid numbers are 0-%2d\n", eepromConfig.serialChannels);
						cliPortPrintF("You entered %2d, please try again\n", tempPin);
						receiverQuery = '?';
						validQuery = false;
						break;
					}
				}
				else
				{
					if ((tempPin < 1) || (tempPin > 6))
					{
						cliPortPrintF("Invalid RSSI Pin number, valid numbers are 1-6\n");
						cliPortPrintF("You entered %2d, please try again\n", tempPin);
						receiverQuery = '?';
						validQuery = false;
						break;
					}
				}

				eepromConfig.rssiPin     = tempPin;
				eepromConfig.rssiMin     = tempMin;
				eepromConfig.rssiMax     = tempMax;
				eepromConfig.rssiWarning = tempWarn;

				receiverQuery = 'a';
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
			   	cliPortPrint("'a' Receiver Configuration Data            'A' Set RX Input Type                    AX, 1=Parallel, 2=Serial, 3=Spektrum\n");
   		        cliPortPrint("'b' Set Maximum Rate Commands              'B' Set RC Control Order                 BTAER12345678\n");
			   	cliPortPrint("'c' Set Maximum Attitude Command           'C' Set Spektrum Resolution              C0 or C1\n");
			   	cliPortPrint("                                           'D' Set Number of Spektrum Channels      D6 thru D12\n");
			   	cliPortPrint("                                           'E' Set RC Control Points                EmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPortPrint("                                           'F' Set Arm/Disarm Counts                FarmCount;disarmCount\n");
			   	cliPortPrint("                                           'G' Set number of serial PWM channels    GnumChannels\n");
			   	cliPortPrint("'r' Toggle RSSI between PPM/ADC            'R' Set RSSI Config                      RPin;Min;Max;Warning\n");
			   	cliPortPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Receiver CLI                      '?' Command Summary\n\n");
			   	break;

	    	///////////////////////////
	    }
	}

}

