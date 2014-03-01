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
// GPS CLI
///////////////////////////////////////////////////////////////////////////////

void gpsCLI()
{
	USART_InitTypeDef USART_InitStructure;

	uint8_t  gpsQuery   = 'x';
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering GPS CLI....\n\n");

    while(true)
    {
        cliPrint("GPS CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    gpsQuery = cliRead();

		cliPrint("\n");

		switch(gpsQuery)
		{
            ///////////////////////////

            case 'a': // GPS Installation Data
                cliPrint("\n");

				switch(eepromConfig.gpsType)
				{
					///////////////

					case NO_GPS:
					    cliPrint("No GPS Installed....\n\n");
					    break;

					///////////////

					case MEDIATEK_3329_BINARY:
					    cliPrint("MediaTek 3329 GPS installed, Binary Mode....\n\n");
					    break;

					///////////////

					case MEDIATEK_3329_NMEA:
					    cliPrint("MediaTek 3329 GPS Installed, NMEA Mode....\n\n");
					    break;

					///////////////

					case UBLOX:
					    cliPrint("UBLOX GPS Installed, Binary Mode....\n\n");
					    break;

					///////////////
				}

                cliPrintF("GPS Baud Rate: %6ld\n\n", eepromConfig.gpsBaudRate);

                validQuery = false;
                break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting GPS CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set GPS Installed State to False
                eepromConfig.gpsType = NO_GPS;

                gpsQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Set GPS Type to MediaTek 3329 Binary
                eepromConfig.gpsType = MEDIATEK_3329_BINARY;

                initGPS();

                gpsQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Set GPS Type to MediaTek 3329 NMEA
                eepromConfig.gpsType = MEDIATEK_3329_NMEA;

                initGPS();

                gpsQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Set GPS Type to UBLOX Binary
                eepromConfig.gpsType = UBLOX;

                initGPS();

                gpsQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'S': // Read GPS Baud Rate
                eepromConfig.gpsBaudRate = (uint16_t)readFloatCLI();

                USART_StructInit(&USART_InitStructure);

                USART_InitStructure.USART_BaudRate = eepromConfig.gpsBaudRate;

                USART_Init(USART2, &USART_InitStructure);

                gpsQuery = 'a';
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
			   	cliPrint("'a' Display GPS Installation Data          'A' Set GPS Type to No GPS\n");
			   	cliPrint("                                           'B' Set GPS Type to MediaTek 3329 Binary\n");
			   	cliPrint("                                           'C' Set GPS Type to MediaTek 3329 NMEA\n");
			   	cliPrint("                                           'D' Set GPS Type to UBLOX\n");
			   	cliPrint("                                           'S' Set GPS Baud Rate\n");
			    cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit GPS CLI                           '?' Command Summary\n\n");
			    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
