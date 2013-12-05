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
// MAX7456 CLI
///////////////////////////////////////////////////////////////////////////////

void max7456CLI()
{
    uint8_t  max7456query = 'x' ;
    uint8_t  validQuery   = false;

    cliBusy = true;

    cliPrint("\nEntering MAX7456 CLI....\n\n");

   	resetMax7456();

    while(true)
    {
		if (!validQuery) cliPrint("MAX7456 CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    max7456query = cliRead();

		if (!validQuery) cliPrint("\n");

		switch(max7456query)
		{
            ///////////////////////

            case 'a': // OSD Configuration
                cliPrint("\nMAX7456 OSD Status:              ");
                if (eepromConfig.osdEnabled)
                	cliPrint("Enabled\n");
                else
               	    cliPrint("Disabled\n");

                cliPrint("OSD Default Video Standard:      ");
                if (eepromConfig.defaultVideoStandard)
                    cliPrint("PAL\n");
                else
                    cliPrint("NTSC\n");

                cliPrint("OSD Display Units:               ");
                if (eepromConfig.metricUnits)
                    cliPrint("Metric\n");
                else
                    cliPrint("English\n");

                cliPrint("OSD Altitude Display:            ");
                if (eepromConfig.osdDisplayAlt)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrintF("OSD Altitude Row:                %3d\n", eepromConfig.osdDisplayAltRow);
				cliPrintF("OSD Altitude Column:             %3d\n", eepromConfig.osdDisplayAltCol);

				cliPrint("OSD Altitude Hold State Display: ");
                if (eepromConfig.osdDisplayAltHoldState)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("OSD Artificial Horizon Display:  ");
                if (eepromConfig.osdDisplayAH)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("OSD Attitude Display:            ");
                if (eepromConfig.osdDisplayAtt)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("OSD Heading Display:             ");
                if (eepromConfig.osdDisplayHdg)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrintF("OSD Heading Row:                 %3d\n", eepromConfig.osdDisplayHdgRow);
				cliPrintF("OSD Heading Column:              %3d\n", eepromConfig.osdDisplayHdgCol);

				cliPrint("\n");
                validQuery = false;
                break;

            ///////////////////////

   		    case 'b': // Toggle OSD Altitude Display
   			    if (eepromConfig.osdDisplayAlt)
   			        eepromConfig.osdDisplayAlt = false;
   			    else
   			        eepromConfig.osdDisplayAlt = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			case 'c': // Toggle OSD Altitude State Display
   			    if (eepromConfig.osdDisplayAltHoldState)
   			        eepromConfig.osdDisplayAltHoldState = false;
   			    else
   			        eepromConfig.osdDisplayAltHoldState = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			case 'd': // Toggle OSD Artificial Horizon Display
   			    if (eepromConfig.osdDisplayAH)
   			        eepromConfig.osdDisplayAH = false;
   			    else
   			    {
   			        eepromConfig.osdDisplayAH  = true;
   			        eepromConfig.osdDisplayAtt = false;
   			    }

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

           case 'e': // Toggle OSD Attitude Horizon Display
   			    if (eepromConfig.osdDisplayAtt)
   			        eepromConfig.osdDisplayAtt = false;
   			    else
   			    {
   			        eepromConfig.osdDisplayAtt = true;
   			        eepromConfig.osdDisplayAH  = false;
   			    }

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

             case 'f': // Toggle OSD Heading Display
   			    if (eepromConfig.osdDisplayHdg)
   			        eepromConfig.osdDisplayHdg = false;
   			    else
   			        eepromConfig.osdDisplayHdg = true;

                max7456query = 'a';
                validQuery = true;
   				break;

            ///////////////////////

            case 'q': // Toggle English/Metric Display Units
                if (eepromConfig.metricUnits)
                	eepromConfig.metricUnits = false;
                else
                    eepromConfig.metricUnits = true;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'r': // Reset MAX7456
                resetMax7456();
                cliPrint("\nMAX7456 Reset....\n\n");
                break;

            ///////////////////////

            case 's': // Show character set
                showMax7456Font();
                cliPrint("\nMAX7456 Character Set Displayed....\n\n");
                break;

            ///////////////////////

            case 't': // Download font
                downloadMax7456Font();
                break;

            ///////////////////////

            case 'u': // Toggle OSD enabled status
   			    if (eepromConfig.osdEnabled)                   // If  Enabled
   			        eepromConfig.osdEnabled = false;           // Set Disabled
   			    else
   			    {                                              // If  Disabled
   			        eepromConfig.osdEnabled = true;            // Set Enabled
                    initMax7456();                             // and call init procedure
				}

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

   		    case 'v': // Toggle default video standard
   			    if (eepromConfig.defaultVideoStandard)         // If  PAL
   			        eepromConfig.defaultVideoStandard = NTSC;  // Set NTSC
   			    else                                           // If  NTSC
   			        eepromConfig.defaultVideoStandard = PAL;   // Set PAL

                max7456query = 'a';
                validQuery = true;
   				break;

   		    ///////////////////////

   			case 'x':
   			    cliPrint("\nExiting MAX7456 CLI....\n\n");
   			    cliBusy = false;
   			    return;
   			    break;

            ///////////////////////

			case 'B': // Change OSD Altitude Display Location
                eepromConfig.osdDisplayAltRow = readFloatCLI();
				eepromConfig.osdDisplayAltCol = readFloatCLI();

				max7456query = 'a';
				validQuery = true;
				break;

			///////////////////////

			case 'C': // Disable OSD Altitude State Display
				if (eepromConfig.osdDisplayAltHoldState)
					eepromConfig.osdDisplayAltHoldState = false;
				else
					eepromConfig.osdDisplayAltHoldState = true;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'F': // Change OSD Heading Display Location
                eepromConfig.osdDisplayHdgRow = readFloatCLI();
				eepromConfig.osdDisplayHdgCol = readFloatCLI();

				max7456query = 'a';
				validQuery = true;
				break;

            ///////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

    		///////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' OSD Configuration\n");
				cliPrint("'b' Toggle OSD Alt Display                 'B' Change OSD Alt Display Location              Brow;column\n");
				cliPrint("'c' Toggle OSD Alt Hold State Display      'C' Change OSD Alt Hold State Display Location   Crow;column\n");
			   	cliPrint("'d' Toggle OSD Artificial Horizon Display\n");
			   	cliPrint("'e' Toggle OSD Attitude Display\n");
			   	cliPrint("'f' Toggle OSD Heading Display             'F' Change OSD Heading Display Location          Frow;column\n");
				cliPrint("'q' Toggle English/Metric Display Units\n");
				cliPrint("'r' Reset MAX7456\n");
			   	cliPrint("'s' Display MAX7456 Character Set\n");
			   	cliPrint("'t' Download Font to MAX7456\n");
			   	cliPrint("'u' Toggle OSD Enabled State\n");
			   	cliPrint("'v' Toggle Default Video Standard          'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit MAX7456 CLI                       '?' Command Summary\n");
			   	cliPrint("\n");
	    	    break;

	    	///////////////////////
	    }
    }
}

///////////////////////////////////////////////////////////////////////////////
