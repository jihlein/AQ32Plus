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
	uint8_t  cliOSDReset  = false;

	uint8_t  tempRow = 0;
    uint8_t  tempCol = 0;
    uint8_t	 maxRows = 0;
    uint8_t  maxCols = 30;

	if (eepromConfig.defaultVideoStandard == PAL)
    	maxRows = 16;
    else
    	maxRows = 13;

    cliBusy = true;

    cliPrint("\nEntering MAX7456 CLI....\n\n");

   	//resetMax7456();

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
                cliPrint("\nMAX7456 OSD Status:                    ");
                if (eepromConfig.osdEnabled)
                	cliPrint("Enabled\n");
                else
               	    cliPrint("Disabled\n");

                cliPrint("OSD Default Video Standard:------------");
                if (eepromConfig.defaultVideoStandard == PAL)
                    cliPrint("PAL\n");
                else if (eepromConfig.defaultVideoStandard == NTSC)
					cliPrint("NTSC\n");
                else if (eepromConfig.defaultVideoStandard == AUTO)
					cliPrint("AUTO\n");

                cliPrint("'q' OSD Display Units:                 ");
				if (eepromConfig.metricUnits)
					cliPrint("Metric\n");
				else
					cliPrint("English\n");

				cliPrint("'b' OSD Display AH/Attitude:-----------");
				if (eepromConfig.osdDisplayAH)
					cliPrint("Artificial Horizon\n");
				else if (eepromConfig.osdDisplayAtt)
					cliPrint("Attitude\n");
				else
					cliPrint("Off\n");

				cliPrint("'d' OSD Display Altitude:--------------");
				if (eepromConfig.osdDisplayAlt)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'e' OSD Display Altitude Hold State:   ");
				if (eepromConfig.osdDisplayAltHoldState)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'f' OSD Display Heading:---------------");
				if (eepromConfig.osdDisplayHdg)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'g' OSD Display Heading Bar:           ");
				if (eepromConfig.osdDisplayHdgBar)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'h' OSD Display Voltage:---------------");
				if (eepromConfig.osdDisplayVoltage)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'i' OSD Display Current:               ");
				if (eepromConfig.osdDisplayCurrent)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'j' OSD Display RSSI:------------------");
				if (eepromConfig.osdDisplayRSSI)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'k' OSD Display Throttle:              ");
				if (eepromConfig.osdDisplayThrot)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'l' OSD Display Motors Armed Timer:----");
				if (eepromConfig.osdDisplayTimer)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrintF("'D' OSD Altitude Row, Column:-----------%2d, %2d\n", eepromConfig.osdDisplayAltRow, eepromConfig.osdDisplayAltCol);

				cliPrintF("'F' OSD Heading  Row, Column:           %2d, %2d\n", eepromConfig.osdDisplayHdgRow, eepromConfig.osdDisplayHdgCol);

				cliPrintF("'G' OSD Hdg Bar  Row, Column:-----------%2d, %2d\n", eepromConfig.osdDisplayHdgBarRow, eepromConfig.osdDisplayHdgBarCol);

				cliPrintF("'H' OSD Voltage  Row, Column:           %2d, %2d\n", eepromConfig.osdDisplayVoltageRow, eepromConfig.osdDisplayVoltageCol);

				cliPrintF("'I' OSD Current  Row, Column:-----------%2d, %2d\n", eepromConfig.osdDisplayCurrentRow, eepromConfig.osdDisplayCurrentCol);

				cliPrintF("'J' OSD RSSI     Row, Column:           %2d, %2d\n", eepromConfig.osdDisplayRSSIRow, eepromConfig.osdDisplayRSSICol);

				cliPrintF("'K' OSD Throttle Row, Column:-----------%2d, %2d\n", eepromConfig.osdDisplayThrotRow, eepromConfig.osdDisplayThrotCol);

				cliPrintF("'L' OSD Motors Armed Timer Row, Column: %2d, %2d\n", eepromConfig.osdDisplayTimerRow, eepromConfig.osdDisplayTimerCol);

				cliPrint("\n");
				validQuery = false;
				break;

            case 'b': // Toggle OSD Artificial Horizon/Attitude Display
   			    if (eepromConfig.osdDisplayAH)
   			    {
   			        eepromConfig.osdDisplayAH = false;
   			    	eepromConfig.osdDisplayAtt = true;
   			    }
   			    else if (eepromConfig.osdDisplayAtt)
   			    {
   			        eepromConfig.osdDisplayAH = false;
   			        eepromConfig.osdDisplayAtt = false;
   			    }
   			    else
   			    {
   			    	eepromConfig.osdDisplayAH = true;
   			    	eepromConfig.osdDisplayAtt = false;
   			    }

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

  		    case 'd': // Toggle OSD Altitude Display
  			    if (eepromConfig.osdDisplayAlt)
  			        eepromConfig.osdDisplayAlt = false;
  			    else
  			        eepromConfig.osdDisplayAlt = true;

               max7456query = 'a';
               validQuery = true;
               cliOSDReset = true;
  				break;

			///////////////////////

			case 'e': // Toggle OSD Altitude State Display
  			    if (eepromConfig.osdDisplayAltHoldState)
  			        eepromConfig.osdDisplayAltHoldState = false;
  			    else
  			        eepromConfig.osdDisplayAltHoldState = true;

               max7456query = 'a';
               validQuery = true;
               cliOSDReset = true;
  				break;

			///////////////////////

             case 'f': // Toggle OSD Heading Display
   			    if (eepromConfig.osdDisplayHdg)
   			        eepromConfig.osdDisplayHdg = false;
   			    else
   			        eepromConfig.osdDisplayHdg = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

            ///////////////////////

            case 'g': // Toggle OSD Heading Bar Display
   			    if (eepromConfig.osdDisplayHdgBar)
   			        eepromConfig.osdDisplayHdgBar = false;
   			    else
   			        eepromConfig.osdDisplayHdgBar = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

			 case 'h': // Toggle OSD Voltage Display
   			    if (eepromConfig.osdDisplayVoltage)
   			        eepromConfig.osdDisplayVoltage = false;
   			    else
   			        eepromConfig.osdDisplayVoltage = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

			case 'i': // Toggle OSD Current Display
   			    if (eepromConfig.osdDisplayCurrent)
   			        eepromConfig.osdDisplayCurrent = false;
   			    else
   			        eepromConfig.osdDisplayCurrent = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

			case 'j': // Toggle OSD RSSI Display
   			    if (eepromConfig.osdDisplayRSSI)
   			        eepromConfig.osdDisplayRSSI = false;
   			    else
   			        eepromConfig.osdDisplayRSSI = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

			case 'k': // Toggle OSD Throttle Display
   			    if (eepromConfig.osdDisplayThrot)
   			        eepromConfig.osdDisplayThrot = false;
   			    else
   			        eepromConfig.osdDisplayThrot = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

			case 'l': // Toggle OSD Motors Armed Timer Display
				if (eepromConfig.osdDisplayTimer)
					eepromConfig.osdDisplayTimer = false;
				else
					eepromConfig.osdDisplayTimer = true;

				max7456query = 'a';
				validQuery = true;
				cliOSDReset = true;
				break;

			///////////////////////

            case 'q': // Toggle english/metric units
            	if (eepromConfig.metricUnits)
            		eepromConfig.metricUnits = false;
            	else
            		eepromConfig.metricUnits = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
                break;

            ///////////////////////

            case 'r': // Reset MAX7456
                resetMax7456();
				updateMax7456(0, 1);
                cliPrint("\nMAX7456 Reset....\n\n");
                max7456query = 'a';
                validQuery = true;
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
					if (eepromConfig.defaultVideoStandard == PAL)
						maxRows = 16;
					else
						maxRows = 13;
					updateMax7456(0, 1);
				}

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
   				break;

			///////////////////////

   		    case 'v': // Toggle default video standard
   			    if (eepromConfig.defaultVideoStandard == PAL)       // If  PAL
				{
   			        eepromConfig.defaultVideoStandard = NTSC;       // Set NTSC
					maxRows = 13;
				}
   			    else if (eepromConfig.defaultVideoStandard == NTSC) // If  NTSC
				{
   			        eepromConfig.defaultVideoStandard = AUTO;       // Set AUTO
				}
   			    else if (eepromConfig.defaultVideoStandard == AUTO) // If  AUTO
				{
					eepromConfig.defaultVideoStandard = PAL;        // Set PAL
					maxRows = 16;
				}

                max7456query = 'u';
                validQuery = true;
   				break;

   		    ///////////////////////

   			case 'x':
   			    cliPrint("\nExiting MAX7456 CLI....\n\n");
   			    cliBusy = false;
   			    return;
   			    break;

            ///////////////////////

			case 'D': // Change OSD Altitude Display Location
				tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayAltRow = tempRow;
					eepromConfig.osdDisplayAltCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

			///////////////////////

			case 'E': // Disable OSD Altitude State Display
				if (eepromConfig.osdDisplayAltHoldState)
					eepromConfig.osdDisplayAltHoldState = false;
				else
					eepromConfig.osdDisplayAltHoldState = true;

                max7456query = 'a';
                validQuery = true;
                cliOSDReset = true;
                break;

            ///////////////////////

            case 'F': // Change OSD Heading Display Location
            	tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayHdgRow = tempRow;
					eepromConfig.osdDisplayHdgCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

            ///////////////////////

            case 'G': // Change OSD Heading Bar Display Location
            	tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayHdgBarRow = tempRow;
					eepromConfig.osdDisplayHdgBarCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

           ///////////////////////

		   case 'H': // Change OSD Voltage Display Location
			    tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayVoltageRow = tempRow;
					eepromConfig.osdDisplayVoltageCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

			///////////////////////

		    case 'I': // Change OSD Current Display Location
		    	tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayCurrentRow = tempRow;
					eepromConfig.osdDisplayCurrentCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

           ///////////////////////

		   case 'J': // Change OSD RSSI Display Location
			    tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayRSSIRow = tempRow;
					eepromConfig.osdDisplayRSSICol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

           ///////////////////////

		   case 'K': // Change OSD Throttle Display Location
			    tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayThrotRow = tempRow;
					eepromConfig.osdDisplayThrotCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}

				max7456query = 'a';
				validQuery = true;
                cliOSDReset = true;
				break;

            ///////////////////////

		   case 'L': // Change OSD Timer Display Location
				tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayTimerRow = tempRow;
					eepromConfig.osdDisplayTimerCol = tempCol;
				}
				else
				{
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);
					cliPrintF("You entered %2d, %2d\n\n", tempRow, tempCol);
				}


				max7456query = 'a';
				validQuery = true;
				cliOSDReset = true;
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
			   	cliPrint("'b' Toggle OSD Artificial Horizon/Attitude Display\n");
				cliPrint("'d' Toggle OSD Alt Display                   'D' Change OSD Alt Display Location              Drow;column\n");
				cliPrint("'e' Toggle OSD Alt Hold State Display\n");
			   	cliPrint("'f' Toggle OSD Heading Display               'F' Change OSD Heading Display Location          Frow;column\n");
				cliPrint("'g' Toggle OSD Heading Bar Display           'G' Change OSD Heading Bar Display Location      Grow;column\n");
				cliPrint("'h' Toggle OSD Voltage Display               'H' Change OSD Voltage Display Location          Hrow;column\n");
				cliPrint("'i' Toggle OSD Current Display               'I' Change OSD Current Display Location          Irow;column\n");
				cliPrint("'j' Toggle OSD RSSI Display                  'J' Change OSD RSSI Display Location             Jrow;column\n");
				cliPrint("'k' Toggle OSD Throttle Display              'K' Change OSD Throttle Display Location         Krow;column\n");
				cliPrint("'l' Toggle OSD Motors Armed Timer Display    'L' Change OSD Armed Timer Display Location      Lrow;column\n");
			   	cliPrint("'q' Toggle English/Metric Display Units\n");
				cliPrint("'r' Reset MAX7456\n");
			   	cliPrint("'s' Display MAX7456 Character Set\n");
			   	cliPrint("'t' Download Font to MAX7456\n");
			   	cliPrint("'u' Toggle OSD Enabled State\n");
			   	cliPrint("'v' Toggle Default Video Standard            'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit MAX7456 CLI                         '?' Command Summary\n\n");
			   	break;

	    	///////////////////////
	    }
		if (cliOSDReset)
		{
			resetMax7456();
			updateMax7456(0, 1);
			cliOSDReset = false;
		}
    }
}

///////////////////////////////////////////////////////////////////////////////
