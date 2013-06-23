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
    uint8_t  max7456query;
    uint8_t  validQuery = false;

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
                cliPrint("\n'u' MAX7456 OSD Status:              ");
                if (eepromConfig.osdEnabled)
                	cliPrint("Enabled\n");
                else
               	    cliPrint("Disabled\n");

                cliPrint("'v' OSD Default Video Standard:------");
                if (eepromConfig.defaultVideoStandard)
                    cliPrint("PAL\n");
                else
                    cliPrint("NTSC\n");

                cliPrint("'q' OSD Display Units:               ");
                if (eepromConfig.metricUnits)
                    cliPrint("Metric\n");
                else
                    cliPrint("English\n");

                cliPrint("'b' OSD Display Artificial Horizon:--");
                if (eepromConfig.osdDisplayAH)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("'c' OSD Display Attitude:            ");
                if (eepromConfig.osdDisplayAtt)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("'d' OSD Display Altitude:------------");
                if (eepromConfig.osdDisplayAlt)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrint("'e' OSD Display Altitude Hold State: ");
                if (eepromConfig.osdDisplayAltHoldState)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrint("'f' OSD Display Heading:-------------");
                if (eepromConfig.osdDisplayHdg)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrint("'g' OSD Display Heading Bar:         ");
                if (eepromConfig.osdDisplayHdgBar)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrint("'h' OSD Display Voltage:-------------");
                if (eepromConfig.osdDisplayVoltage)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

				cliPrint("'i' OSD Display Current:             ");
                if (eepromConfig.osdDisplayCurrent)
                    cliPrint(" On\n");
                else
					cliPrint("Off\n");

                cliPrint("'j' OSD Display RSSI:----------------");
				if (eepromConfig.osdDisplayRSSI)
					cliPrint(" On\n");
				else
					cliPrint("Off\n");

				cliPrint("'k' OSD Display Throttle:            ");
                if (eepromConfig.osdDisplayThrot)
                    cliPrint(" On\n");
                else
                    cliPrint("Off\n");

                cliPrintF("'D' OSD Altitude Row, Column:---------%2d, %2d\n", eepromConfig.osdDisplayAltRow, eepromConfig.osdDisplayAltCol);

				cliPrintF("'F' OSD Heading  Row, Column:         %2d, %2d\n", eepromConfig.osdDisplayHdgRow, eepromConfig.osdDisplayHdgCol);

				cliPrintF("'G' OSD Hdg Bar  Row, Column:---------%2d, %2d\n", eepromConfig.osdDisplayHdgBarRow, eepromConfig.osdDisplayHdgBarCol);

				cliPrintF("'H' OSD Voltage  Row, Column:         %2d, %2d\n", eepromConfig.osdDisplayVoltageRow, eepromConfig.osdDisplayVoltageCol);

				cliPrintF("'I' OSD Current  Row, Column:---------%2d, %2d\n", eepromConfig.osdDisplayCurrentRow, eepromConfig.osdDisplayCurrentCol);

				cliPrintF("'J' OSD RSSI     Row, Column:         %2d, %2d\n", eepromConfig.osdDisplayRSSIRow, eepromConfig.osdDisplayRSSICol);

				cliPrintF("'K' OSD Throttle Row, Column:---------%2d, %2d\n", eepromConfig.osdDisplayThrotRow, eepromConfig.osdDisplayThrotCol);

                cliPrint("\n");
                validQuery = false;
                break;

            ///////////////////////

			case 'b': // Toggle OSD Aritificial Horizon Display
   			    if (eepromConfig.osdDisplayAH)
   			        eepromConfig.osdDisplayAH = false;
   			    else
   			        eepromConfig.osdDisplayAH = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

           case 'c': // Toggle OSD Attitude Horizon Display
   			    if (eepromConfig.osdDisplayAtt)
   			        eepromConfig.osdDisplayAtt = false;
   			    else
   			        eepromConfig.osdDisplayAtt = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

  		    case 'd': // Toggle OSD Altitude Display
  			    if (eepromConfig.osdDisplayAlt)
  			        eepromConfig.osdDisplayAlt = false;
  			    else
  			        eepromConfig.osdDisplayAlt = true;

               max7456query = 'a';
               validQuery = true;
  				break;

			///////////////////////

			case 'e': // Toggle OSD Altitude State Display
  			    if (eepromConfig.osdDisplayAltHoldState)
  			        eepromConfig.osdDisplayAltHoldState = false;
  			    else
  			        eepromConfig.osdDisplayAltHoldState = true;

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

            case 'g': // Toggle OSD Heading Bar Display
   			    if (eepromConfig.osdDisplayHdgBar)
   			        eepromConfig.osdDisplayHdgBar = false;
   			    else
   			        eepromConfig.osdDisplayHdgBar = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			 case 'h': // Toggle OSD Voltage Display
   			    if (eepromConfig.osdDisplayVoltage)
   			        eepromConfig.osdDisplayVoltage = false;
   			    else
   			        eepromConfig.osdDisplayVoltage = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			case 'i': // Toggle OSD Current Display
   			    if (eepromConfig.osdDisplayCurrent)
   			        eepromConfig.osdDisplayCurrent = false;
   			    else
   			        eepromConfig.osdDisplayCurrent = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			case 'j': // Toggle OSD RSSI Display
   			    if (eepromConfig.osdDisplayRSSI)
   			        eepromConfig.osdDisplayRSSI = false;
   			    else
   			        eepromConfig.osdDisplayRSSI = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

			case 'k': // Toggle OSD Throttle Display
   			    if (eepromConfig.osdDisplayThrot)
   			        eepromConfig.osdDisplayThrot = false;
   			    else
   			        eepromConfig.osdDisplayThrot = true;

                max7456query = 'a';
                validQuery = true;
   				break;

			///////////////////////

            case 'q': // Toggle english/metric units
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

			case 'D': // Change OSD Altitude Display Location
				tempRow = (uint8_t)readFloatCLI();
				tempCol = (uint8_t)readFloatCLI();
				if ((tempRow >= 0) && (tempRow < maxRows) && (tempCol >=0) && (tempCol < maxCols))
				{
					eepromConfig.osdDisplayAltRow = tempRow;
					eepromConfig.osdDisplayAltCol = tempCol;
				}
				else
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
				break;

			///////////////////////

			case 'E': // Disable OSD Altitude State Display
				if (eepromConfig.osdDisplayAltHoldState)
					eepromConfig.osdDisplayAltHoldState = false;
				else
					eepromConfig.osdDisplayAltHoldState = true;

                max7456query = 'a';
                validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

				max7456query = 'a';
				validQuery = true;
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
					cliPrintF("\nValid rows are 0 to %2d, valid columns are 0 to %2d\n", maxRows, maxCols);

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
			   	cliPrint("'b' Toggle OSD Artificial Horizon Display\n");
			   	cliPrint("'c' Toggle OSD Attitude Display\n");
				cliPrint("'d' Toggle OSD Alt Display                 'D' Change OSD Alt Display Location              Drow;column\n");
				cliPrint("'e' Toggle OSD Alt Hold State Display      \n");
			   	cliPrint("'f' Toggle OSD Heading Display             'F' Change OSD Heading Display Location          Frow;column\n");
				cliPrint("'g' Toggle OSD Heading Bar Display         'G' Change OSD Heading Bar Display Location      Grow;column\n");
				cliPrint("'h' Toggle OSD Voltage Display             'H' Change OSD Voltage Display Location          Hrow;column\n");
				cliPrint("'i' Toggle OSD Current Display             'I' Change OSD Current Display Location          Irow;column\n");
				cliPrint("'j' Toggle OSD RSSI Display                'J' Change OSD RSSI Display Location             Jrow;column\n");
				cliPrint("'k' Toggle OSD Throttle Display            'K' Change OSD Throttle Display Location         Krow;column\n");
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
// Mixer CLI
///////////////////////////////////////////////////////////////////////////////

void mixerCLI()
{
    float    tempFloat;

    uint8_t  index;
    uint8_t  rows, columns;

    uint8_t  mixerQuery;
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering Mixer CLI....\n\n");

    while(true)
    {
        cliPrint("Mixer CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    mixerQuery = cliRead();

		cliPrint("\n");

		switch(mixerQuery)
		{
            ///////////////////////////

            case 'a': // Mixer Configuration
                cliPrint("\nMixer Configuration:            ");
                switch (eepromConfig.mixerConfiguration)
                {
                    case MIXERTYPE_GIMBAL:
                    	cliPrint("MIXERTYPE GIMBAL\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_FLYING_WING:
                    	cliPrint("MIXERTYPE FLYING WING\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_BI:
                        cliPrint("MIXERTYPE BICOPTER\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_TRI:
                        cliPrint("MIXERTYPE TRICOPTER\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_QUADP:
                        cliPrint("MIXERTYPE QUAD PLUS\n");
                        break;

                    case MIXERTYPE_QUADX:
                        cliPrint("MIXERTYPE QUAD X\n");
                        break;

                    case MIXERTYPE_VTAIL4_NO_COMP:
                    	cliPrint("MULTITYPE VTAIL NO COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_Y_COMP:
                    	cliPrint("MULTITYPE VTAIL Y COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RY_COMP:
                    	cliPrint("MULTITYPE VTAIL RY COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_PY_COMP:
                    	cliPrint("MULTITYPE VTAIL PY COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RP_COMP:
                    	cliPrint("MULTITYPE VTAIL RP COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RPY_COMP:
                    	cliPrint("MULTITYPE VTAIL RPY COMP\n");
                    	break;

                    case MIXERTYPE_Y4:
                    	cliPrint("MIXERTYPE Y4\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_HEX6P:
                        cliPrint("MIXERTYPE HEX PLUS\n");
                        break;

                    case MIXERTYPE_HEX6X:
                        cliPrint("MIXERTYPE HEX X\n");
                        break;

                    case MIXERTYPE_Y6:
                        cliPrint("MIXERTYPE Y6\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_OCTOF8P:
                        cliPrint("MIXERTYPE FLAT OCTO PLUS\n");
                        break;

                    case MIXERTYPE_OCTOF8X:
                        cliPrint("MIXERTYPE FLAT OCTO X\n");
                        break;

                    case MIXERTYPE_OCTOX8P:
                        cliPrint("MIXERTYPE COAXIAL OCTO PLUS\n");
                        break;

                    case MIXERTYPE_OCTOX8X:
                        cliPrint("MIXERTYPE COAXIAL OCTO X\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_FREEMIX:
                    	cliPrint("MIXERTYPE FREE MIX\n");
                    	break;
                }

                cliPrintF("Number of Motors:                  %1d\n",  numberMotor);
                cliPrintF("ESC PWM Rate:                    %3ld\n", eepromConfig.escPwmRate);
                cliPrintF("Servo PWM Rate:                  %3ld\n", eepromConfig.servoPwmRate);

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_BI )
                {
                    cliPrintF("BiCopter Left Servo Min:        %4ld\n", (uint16_t)eepromConfig.biLeftServoMin);
                    cliPrintF("BiCopter Left Servo Mid:        %4ld\n", (uint16_t)eepromConfig.biLeftServoMid);
                    cliPrintF("BiCopter Left Servo Max:        %4ld\n", (uint16_t)eepromConfig.biLeftServoMax);
                    cliPrintF("BiCopter Right Servo Min:       %4ld\n", (uint16_t)eepromConfig.biRightServoMin);
                    cliPrintF("BiCopter Right Servo Mid:       %4ld\n", (uint16_t)eepromConfig.biRightServoMid);
                    cliPrintF("BiCopter Right Servo Max:       %4ld\n", (uint16_t)eepromConfig.biRightServoMax);
                }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_FLYING_WING )
                {
                    cliPrintF("Roll Direction Left:            %4ld\n", (uint16_t)eepromConfig.rollDirectionLeft);
                    cliPrintF("Roll Direction Right:           %4ld\n", (uint16_t)eepromConfig.rollDirectionRight);
                    cliPrintF("Pitch Direction Left:           %4ld\n", (uint16_t)eepromConfig.pitchDirectionLeft);
                    cliPrintF("Pitch Direction Right:          %4ld\n", (uint16_t)eepromConfig.pitchDirectionRight);
                    cliPrintF("Wing Left Minimum:              %4ld\n", (uint16_t)eepromConfig.wingLeftMinimum);
                    cliPrintF("Wing Left Maximum:              %4ld\n", (uint16_t)eepromConfig.wingLeftMaximum);
                    cliPrintF("Wing Right Minimum:             %4ld\n", (uint16_t)eepromConfig.wingRightMinimum);
                    cliPrintF("Wing Right Maximum:             %4ld\n", (uint16_t)eepromConfig.wingRightMaximum);
                }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_GIMBAL )
                {
                    cliPrintF("Gimbal Roll Servo Min:          %4ld\n",   (uint16_t)eepromConfig.gimbalRollServoMin);
                    cliPrintF("Gimbal Roll Servo Mid:          %4ld\n",   (uint16_t)eepromConfig.gimbalRollServoMid);
                    cliPrintF("Gimbal Roll Servo Max:          %4ld\n",   (uint16_t)eepromConfig.gimbalRollServoMax);
                    cliPrintF("Gimbal Roll Servo Gain:          %7.3f\n", eepromConfig.gimbalRollServoGain);
                    cliPrintF("Gimbal Pitch Servo Min:         %4ld\n",   (uint16_t)eepromConfig.gimbalPitchServoMin);
                    cliPrintF("Gimbal Pitch Servo Mid:         %4ld\n",   (uint16_t)eepromConfig.gimbalPitchServoMid);
                    cliPrintF("Gimbal Pitch Servo Max:         %4ld\n",   (uint16_t)eepromConfig.gimbalPitchServoMax);
                    cliPrintF("Gimbal Pitch Servo Gain:         %7.3f\n", eepromConfig.gimbalPitchServoGain);
                 }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_TRI )
                {
                    cliPrintF("TriCopter Yaw Servo Min:        %4ld\n", (uint16_t)eepromConfig.triYawServoMin);
                    cliPrintF("TriCopter Yaw Servo Mid:        %4ld\n", (uint16_t)eepromConfig.triYawServoMid);
                    cliPrintF("TriCopter Yaw Servo Max:        %4ld\n", (uint16_t)eepromConfig.triYawServoMax);
                }

                if (eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_Y_COMP  ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RY_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_PY_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RP_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RPY_COMP)
                {
                    cliPrintF("V Tail Angle                     %6.2f\n", eepromConfig.vTailAngle);
                 }

                cliPrintF("Yaw Direction:                    %2d\n\n", (uint16_t)eepromConfig.yawDirection);

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Free Mix Matrix
        	    cliPrintF("\nNumber of Free Mixer Motors:  %1d\n         Roll    Pitch   Yaw\n", eepromConfig.freeMixMotors);

        	    for ( index = 0; index < eepromConfig.freeMixMotors; index++ )
        	    {
        	    	cliPrintF("Motor%1d  %6.3f  %6.3f  %6.3f\n", index,
        	    			                                     eepromConfig.freeMix[index][ROLL ],
        	    			                                     eepromConfig.freeMix[index][PITCH],
        	    			                                     eepromConfig.freeMix[index][YAW  ]);
        	    }

        	    cliPrint("\n");
        	    validQuery = false;
        	    break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting Mixer CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read Mixer Configuration
                eepromConfig.mixerConfiguration = (uint8_t)readFloatCLI();
                initMixer();

        	    mixerQuery = 'a';
                validQuery = true;
		        break;

            ///////////////////////////

            case 'B': // Read ESC and Servo PWM Update Rates
                eepromConfig.escPwmRate   = (uint16_t)readFloatCLI();
                eepromConfig.servoPwmRate = (uint16_t)readFloatCLI();

                pwmEscInit(eepromConfig.escPwmRate);
                pwmServoInit(eepromConfig.servoPwmRate);

                mixerQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Read BiCopter Left Servo Parameters
           	    eepromConfig.biLeftServoMin = readFloatCLI();
           	    eepromConfig.biLeftServoMid = readFloatCLI();
           	    eepromConfig.biLeftServoMax = readFloatCLI();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read BiCopter Right Servo Parameters
           	    eepromConfig.biRightServoMin = readFloatCLI();
           	    eepromConfig.biRightServoMid = readFloatCLI();
           	    eepromConfig.biRightServoMax = readFloatCLI();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Flying Wing Servo Directions
                eepromConfig.rollDirectionLeft   = readFloatCLI();
                eepromConfig.rollDirectionRight  = readFloatCLI();
                eepromConfig.pitchDirectionLeft  = readFloatCLI();
                eepromConfig.pitchDirectionRight = readFloatCLI();

         	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Flying Wing Servo Limits
           	    eepromConfig.wingLeftMinimum  = readFloatCLI();
           	    eepromConfig.wingLeftMaximum  = readFloatCLI();
           	    eepromConfig.wingRightMinimum = readFloatCLI();
           	    eepromConfig.wingRightMaximum = readFloatCLI();

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Free Mix Motor Number
           	    eepromConfig.freeMixMotors = (uint8_t)readFloatCLI();
           	    initMixer();

           	    mixerQuery = 'b';
                validQuery = true;
                break;

            ///////////////////////////

            case 'H': // Read Free Mix Matrix Element
                rows    = (uint8_t)readFloatCLI();
                columns = (uint8_t)readFloatCLI();
                eepromConfig.freeMix[rows][columns] = readFloatCLI();

                mixerQuery = 'b';
                validQuery = true;
                break;

            ///////////////////////////

            case 'I': // Read Gimbal Roll Servo Parameters
         	    eepromConfig.gimbalRollServoMin  = readFloatCLI();
           	    eepromConfig.gimbalRollServoMid  = readFloatCLI();
           	    eepromConfig.gimbalRollServoMax  = readFloatCLI();
           	    eepromConfig.gimbalRollServoGain = readFloatCLI();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'J': // Read Gimbal Pitch Servo Parameters
           	    eepromConfig.gimbalPitchServoMin  = readFloatCLI();
           	    eepromConfig.gimbalPitchServoMid  = readFloatCLI();
           	    eepromConfig.gimbalPitchServoMax  = readFloatCLI();
           	    eepromConfig.gimbalPitchServoGain = readFloatCLI();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'K': // Read TriCopter YawServo Parameters
        	    eepromConfig.triYawServoMin = readFloatCLI();
           	    eepromConfig.triYawServoMid = readFloatCLI();
           	    eepromConfig.triYawServoMax = readFloatCLI();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'L': // Read V Tail Angle
        	    eepromConfig.vTailAngle = readFloatCLI();

        	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'M': // Read yaw direction
                tempFloat = readFloatCLI();
                if (tempFloat >= 0.0)
                    tempFloat = 1.0;
                else
                	tempFloat = -1.0;

                eepromConfig.yawDirection = tempFloat;

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Mixer Configuration Data               'A' Set Mixer Configuration              A1 thru 21, see aq32Plus.h\n");
   		        cliPrint("'b' Free Mixer Configuration               'B' Set PWM Rates                        BESC;Servo\n");
			   	cliPrint("                                           'C' Set BiCopter Left Servo Parameters   CMin;Mid;Max\n");
			   	cliPrint("                                           'D' Set BiCopter Right Servo Parameters  DMin;Mid;Max\n");
			   	cliPrint("                                           'E' Set Flying Wing Servo Directions     ERollLeft;RollRight;PitchLeft;PitchRight\n");
			   	cliPrint("                                           'F' Set Flying Wing Servo Limits         FLeftMin;LeftMax;RightMin;RightMax\n");
   		        cliPrint("                                           'G' Set Number of FreeMix Motors         GNumber\n");
   		        cliPrint("                                           'H' Set FreeMix Matrix Element           HRow;Column;Element\n");
   		        cliPrint("                                           'I' Set Gimbal Roll Servo Parameters     IMin;Mid;Max;Gain\n");
   		        cliPrint("                                           'J' Set Gimbal Pitch Servo Parameters    JMin;Mid;Max;Gain\n");
   		        cliPrint("                                           'K' Set TriCopter Servo Parameters       KMin;Mid;Max\n");
   		        cliPrint("                                           'L' Set V Tail Angle                     LAngle\n");
   		        cliPrint("                                           'M' Set Yaw Direction                    M1 or M-1\n");
   		        cliPrint("                                           'W' Write EEPROM Parameters\n");
   		        cliPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
   		        cliPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
    char     rcOrderString[9];
    float    tempFloat;
    uint8_t  index;
    uint8_t  receiverQuery;
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPrint("Receiver CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliRead();

		cliPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PARALLEL_PWM:
                        cliPrint("Parallel\n");
                        break;
                    case SERIAL_PWM:
                        cliPrint("Serial\n");
                        break;
                    case SPEKTRUM:
                        cliPrint("Spektrum\n");
                        break;
		        }

                cliPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPrint(rcOrderString);  cliPrint("\n");

                cliPrintF("Spektrum Resolution:            %s\n",     eepromConfig.spektrumHires ? "11 Bit Mode" : "10 Bit Mode");
                cliPrintF("Number of Spektrum Channels:    %2d\n",    eepromConfig.spektrumChannels);
                cliPrintF("Mid Command:                    %4ld\n",   (uint16_t)eepromConfig.midCommand);
				cliPrintF("Min Check:                      %4ld\n",   (uint16_t)eepromConfig.minCheck);
				cliPrintF("Max Check:                      %4ld\n",   (uint16_t)eepromConfig.maxCheck);
				cliPrintF("Min Throttle:                   %4ld\n",   (uint16_t)eepromConfig.minThrottle);
				cliPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)eepromConfig.maxThrottle);

				tempFloat = eepromConfig.rateScaling * 180000.0 / PI;
				cliPrintF("Max Rate Command:               %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
                cliPrintF("Max Attitude Command:           %6.2f Degrees\n\n", tempFloat);

				cliPrintF("Arm Delay Count:                %3d Frames\n",   eepromConfig.armCount);
				cliPrintF("Disarm Delay Count:             %3d Frames\n\n", eepromConfig.disarmCount);

				validQuery = false;
				break;

            ///////////////////////////

            case 'b': // Read Max Rate Value
                eepromConfig.rateScaling = readFloatCLI() / 180000 * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatCLI() / 180000 * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read RX Input Type
                eepromConfig.receiverType = (uint8_t)readFloatCLI();
			    cliPrint( "\nReceiver Type Changed....\n");

			    cliPrint("\nSystem Resetting....\n");
			    delay(100);
			    writeEEPROM();
			    systemReset(false);

		        break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringCLI( rcOrderString, 8 );
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

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Receiver Configuration Data            'A' Set RX Input Type                    AX, 1=Parallel, 2=Serial, 3=Spektrum\n");
   		        cliPrint("'b' Set Maximum Rate Command               'B' Set RC Control Order                 BTAER1234\n");
			   	cliPrint("'c' Set Maximum Attitude Command           'C' Set Spektrum Resolution              C0 or C1\n");
			   	cliPrint("                                           'D' Set Number of Spektrum Channels      D6 thru D12\n");
			   	cliPrint("                                           'E' Set RC Control Points                EmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPrint("                                           'F' Set Arm/Disarm Counts                FarmCount;disarmCount\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	cliPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
    uint8_t  sensorQuery;
    uint8_t  tempInt;
    uint8_t  tempPin;
    uint8_t  validQuery = false;

	uint8_t	 tempCells	= 0;
    uint8_t  tempE		= 2;
	uint8_t  tempVPin	= 0;
	uint8_t  tempCPin	= 0;
	float	 tempVScale	= 0.0f;
	float 	 tempCScale	= 0.0f;
	float 	 tempVBias	= 0.0f;
	float	 tempCBias	= 0.0f;

    cliBusy = true;

    cliPrint("\nEntering Sensor CLI....\n\n");

    while(true)
    {
        cliPrint("Sensor CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = cliRead();

		cliPrint("\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data
                cliPrintF("\nMXR Accel Bias:            %9.4f, %9.4f, %9.4f\n", eepromConfig.accelBiasMXR[XAXIS],
				                                                		        eepromConfig.accelBiasMXR[YAXIS],
				                                                		        eepromConfig.accelBiasMXR[ZAXIS]);
				cliPrintF("MXR Accel Scale Factor:    %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelScaleFactorMXR[XAXIS],
								                                                eepromConfig.accelScaleFactorMXR[YAXIS],
				                                                		        eepromConfig.accelScaleFactorMXR[ZAXIS]);
               cliPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\n",    eepromConfig.accelTCBiasSlope[XAXIS],
                                                		                        eepromConfig.accelTCBiasSlope[YAXIS],
                                                		                        eepromConfig.accelTCBiasSlope[ZAXIS]);
                cliPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelTCBiasIntercept[XAXIS],
                                                		                        eepromConfig.accelTCBiasIntercept[YAXIS],
                                                		                        eepromConfig.accelTCBiasIntercept[ZAXIS]);
                cliPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasSlope[ROLL ],
                                                                		        eepromConfig.gyroTCBiasSlope[PITCH],
                                                                		        eepromConfig.gyroTCBiasSlope[YAW  ]);
                cliPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasIntercept[ROLL ],
                                                                   		        eepromConfig.gyroTCBiasIntercept[PITCH],
                                                                   		        eepromConfig.gyroTCBiasIntercept[YAW  ]);
                cliPrintF("Mag Bias:                  %9.4f, %9.4f, %9.4f\n",   eepromConfig.magBias[XAXIS],
                                                   		                        eepromConfig.magBias[YAXIS],
                                                   		                        eepromConfig.magBias[ZAXIS]);
                cliPrintF("Accel One G:               %9.4f\n",   accelOneG);
                cliPrintF("Accel Cutoff:              %9.4f\n",   eepromConfig.accelCutoff);
                cliPrintF("KpAcc (MARG):              %9.4f\n",   eepromConfig.KpAcc);
                cliPrintF("KiAcc (MARG):              %9.4f\n",   eepromConfig.KiAcc);
                cliPrintF("KpMag (MARG):              %9.4f\n",   eepromConfig.KpMag);
                cliPrintF("KiMag (MARG):              %9.4f\n",   eepromConfig.KiMag);
                cliPrintF("hdot est/h est Comp Fil A: %9.4f\n",   eepromConfig.compFilterA);
                cliPrintF("hdot est/h est Comp Fil B: %9.4f\n",   eepromConfig.compFilterB);

                cliPrint("MPU6000 DLPF:                 ");
                switch(eepromConfig.dlpfSetting)
                {
                    case DLPF_256HZ:
                        cliPrint("256 Hz\n");
                        break;
                    case DLPF_188HZ:
                        cliPrint("188 Hz\n");
                        break;
                    case DLPF_98HZ:
                        cliPrint("98 Hz\n");
                        break;
                    case DLPF_42HZ:
                        cliPrint("42 Hz\n");
                        break;
                }

                cliPrint("Magnetic Variation:           ");
                if (eepromConfig.magVar >= 0.0f)
                  cliPrintF("E%6.4f\n\n",  eepromConfig.magVar * R2D);
                else
                  cliPrintF("W%6.4f\n\n", -eepromConfig.magVar * R2D);

                cliPrintF("Battery Cells:               %2d\n", eepromConfig.batteryCells);
                cliPrintF("Battery Voltage Pin:----------%d\n", eepromConfig.batteryVPin);
                cliPrintF("Battery Current Pin:          %d\n", eepromConfig.batteryCPin);
                cliPrintF("Battery Extended:-------------%d\n", eepromConfig.batteryExtended);
                cliPrintF("Battery Voltage Divider:   %9.4f\n", eepromConfig.batteryVScale);
                cliPrintF("Battery Voltage Bias:------%9.4f\n", eepromConfig.batteryVBias);
                cliPrintF("Battery Current Divider:   %9.4f\n", eepromConfig.batteryCScale);
                cliPrintF("Battery Current Bias:------%9.4f\n", eepromConfig.batteryCBias);
                cliPrintF("RSSI Pin:                    %2d\n", eepromConfig.RSSIPin);
                cliPrintF("RSSI Max:------------------%4d\n", eepromConfig.RSSIMax);
                cliPrintF("RSSI Min:                  %4d\n", eepromConfig.RSSIMin);
                cliPrintF("RSSI Warning %:---------------%2d\n", eepromConfig.RSSIWarning);

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // MPU6000 Calibration
                mpu6000Calibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Magnetometer Calibration
                magCalibration(HMC5883L_I2C);

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'd': // Accel Bias and Scale Factor Calibration
                accelCalibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

			///////////////////////////

        	case 'x':
			    cliPrint("\nExiting Sensor CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6000 Digital Low Pass Filter
                tempInt = (uint8_t)readFloatCLI();

                switch(tempInt)
                {
                    case DLPF_256HZ:
                        eepromConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
                        break;

                    case DLPF_188HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
                    	break;

                    case DLPF_98HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
                    	break;

                    case DLPF_42HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
                     	break;
                }

                setSPIdivisor(MPU6000_SPI, 64);  // 0.65625 MHz SPI clock (within 20 +/- 10%)

                GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);
			    spiTransfer(MPU6000_SPI, MPU6000_CONFIG);
			    spiTransfer(MPU6000_SPI, eepromConfig.dlpfSetting);
			    GPIO_SetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);

                setSPIdivisor(MPU6000_SPI, 2);  // 21 MHz SPI clock (within 20 +/- 10%)

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Accel Cutoff
                eepromConfig.accelCutoff = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc, kiAcc
                eepromConfig.KpAcc = readFloatCLI();
                eepromConfig.KiAcc = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag, kiMag
                eepromConfig.KpMag = readFloatCLI();
                eepromConfig.KiMag = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // h dot est/h est Comp Filter A/B
                eepromConfig.compFilterA = readFloatCLI();
                eepromConfig.compFilterB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

    	    ///////////////////////////

    		case 'F': // RSSI Pin
                tempPin = (uint8_t)readFloatCLI();

    			if ((tempPin > 0) && (tempPin < 7))
    				eepromConfig.RSSIPin = tempPin;

                sensorQuery = 'a';
                validQuery = true;
                break;

    		///////////////////////////

            case 'G': // RSSI max/min
                eepromConfig.RSSIMax = (uint16_t)readFloatCLI();
                eepromConfig.RSSIMin = (uint16_t)readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

    		///////////////////////////

            case 'H': // RSSI warning %
                eepromConfig.RSSIWarning = (uint8_t)readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'M': // Magnetic Variation
                eepromConfig.magVar = readFloatCLI() * D2R;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'U': // Set Battery monitoring extended configuration
            	tempE		= (uint8_t)readFloatCLI();
            	tempCPin	= (uint8_t)readFloatCLI();
            	tempCScale	= readFloatCLI();
            	tempCBias 	= readFloatCLI();
            	if (((tempE != 0) && (tempE != 1)) || (tempCPin < 1) || (tempCPin > 6) || (tempCScale = 0.0f))
            	{
            		cliPrintF("batteryExtended, CPin entered incorrectly, or CScale not set\n");
            		cliPrintF("%d, %d, %3.2f, %2.2f\n", tempE, tempCPin, tempCScale, tempCBias);
					cliPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n");
            		sensorQuery = '?';
            		validQuery = true;
            		break;
            	}
            	eepromConfig.batteryExtended	= tempE;
                eepromConfig.batteryCPin		= tempCPin;
                eepromConfig.batteryCScale		= tempCScale;
                eepromConfig.batteryCBias		= tempCBias;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'V': // Set Battery monitoring configuration
				tempVPin	= (uint8_t)readFloatCLI();
				tempVScale	= readFloatCLI();
				tempVBias 	= readFloatCLI();
				tempCells	= (uint8_t)readFloatCLI();

				if ((tempVPin < 1) || (tempVPin > 7) || (tempVScale == 0.0f) || (tempCells < 1) || (tempCells > 12))
				{
					cliPrintF("VPin, VScale, or Cells entered incorrectly\n");
            		cliPrintF("%3d, %9.4f, %9.4f, %3d", tempVPin, tempVScale, tempVBias, tempCells);
					cliPrintF("Please see CLI documentation in the \"aq32plus\\Documentation\" folder\n");
					sensorQuery = '?';
					validQuery = false;
					break;
				}

                eepromConfig.batteryVPin   = tempVPin;
                eepromConfig.batteryVScale = tempVScale;
                eepromConfig.batteryVBias  = tempVBias;
                eepromConfig.batteryCells  = tempCells;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3, see aq32Plus.h\n");
			   	cliPrint("'b' MPU6000 Temp Calibration               'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	cliPrint("'c' Magnetometer Calibration               'C' Set kpAcc/kiAcc                      CkpAcc;kiAcc\n");
			   	cliPrint("'d' Accel Bias and SF Calibration          'D' Set kpMag/kiMag                      DkpMag;kiMag\n");
			   	cliPrint("                                           'E' Set h dot est/h est Comp Filter A/B  EA;B\n");
			   	cliPrint("    Valid Pins for RSSI/Battery - 1-7      'F' Set RSSI pin                         Fpin\n");
			   	cliPrint("    where 1-6->analog pins 1-6 and         'G' Set RSSI max/min                     Gmax;min\n");
			   	cliPrint("    7->onboard voltage divider (default)   'H' Set RSSI Warning %                   Hwarning\n");
			   	cliPrint("                                           'M' Set Mag Variation (+ East, - West)   MMagVar\n");
			   	cliPrint("                                           'U' Set Battery Current Config           UExtended;CPin;CScale;CBias\n");
			   	cliPrint("                                           'V' Set Battery Voltage Config           VVPin;VScale;VBias;#Cells\n");
			   	cliPrint("                                               where extended = 1 (on) or 0 (off).  Must be 1 to measure current\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
// GPS CLI
///////////////////////////////////////////////////////////////////////////////

void gpsCLI()
{
	USART_InitTypeDef USART_InitStructure;

	uint8_t  gpsQuery;
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

            case 'S': // Read ESC and Servo PWM Update Rates
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
			   	cliPrint("'x' Exit GPS CLI                           '?' Command Summary\n");
			    cliPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
// EEPROM CLI
///////////////////////////////////////////////////////////////////////////////

int min(int a, int b)
{
    return a < b ? a : b;
}

///////////////////////////////////////

int8_t parse_hex(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 0x0A;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 0x0A;
    return -1;
}

///////////////////////////////////////

void cliPrintEEPROM(eepromConfig_t *e)
{
    uint32_t old_crc = e->CRCAtEnd[0];
    enum { line_length = 32, len = sizeof(eepromConfig_t) };
    uint8_t *by = (uint8_t*)e;
    int i, j;

    e->CRCAtEnd[0] = crc32bEEPROM(e, false);

    if (e->CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)len / line_length); i++)
    {
        for (j = 0; j < min(line_length, len - line_length * i); j++)
            cliPrintF("%02X", by[i * line_length + j]);

        cliPrint("\n");
    }

    e->CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void eepromCLI()
{
    uint8_t  eepromQuery;
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPrint("\nEntering EEPROM CLI....\n\n");

    while(true)
    {
        cliPrint("EEPROM CLI -> ");

        while ((cliAvailable() == false) && (validQuery == false));

        if (validQuery == false)
            eepromQuery = cliRead();

        cliPrint("\n");

        switch(eepromQuery)
        {
            // 'a' is the standard "print all the information" character
            case 'a': // config struct data
                ;
                uint32_t c1 = eepromConfig.CRCAtEnd[0],
                         c2 = crc32bEEPROM(&eepromConfig, false);

                cliPrintF("Config structure information:\n");
                cliPrintF("Version          : %d\n", eepromConfig.version );
                cliPrintF("Size             : %d\n", sizeof(eepromConfig) );
                cliPrintF("CRC on last read : %08x\n", c1 );
                cliPrintF("Current CRC      : %08x\n", c2 );
                if ( c1 != c2 )
                    cliPrintF("  CRCs differ. Current Config has not yet been saved.\n");
                cliPrintF("CRC Flags :\n");
                cliPrintF("  History Bad    : %s\n", eepromConfig.CRCFlags & CRC_HistoryBad ? "true" : "false" );
                validQuery = false;
                break;

            ///////////////////////////

            case 'c': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;
                // these change randomly when not in flight and can mistakenly
                // make one think that the in-memory eeprom struct has changed
                zeroPIDintegralError();
                zeroPIDstates();

                cliPrintF("\n");

                cliPrintEEPROM(&eepromConfig);

                cliPrintF("\n");

                if (crcCheckVal != crc32bEEPROM(&eepromConfig, true))
                {
                    cliPrint("NOTE: in-memory config CRC invalid; there have probably been changes to\n");
                    cliPrint("      eepromConfig since the last write to flash/eeprom.\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'H': // clear bad history flag
                cliPrintF("Clearing Bad History flag.\n");
                eepromConfig.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'C': // Read in from Console in hex.  Console -> RAM
                ;
                uint32_t sz = sizeof(eepromConfig);
                eepromConfig_t e;
                uint8_t *p = (uint8_t*)&e;
                uint8_t *end = (uint8_t*)(&e + 1);
                uint32_t t = millis();
                enum { Timeout = 100 }; // timeout is in ms
                int second_nibble = 0; // 0 or 1
                char c;
                uint32_t chars_encountered = 0;

                cliPrintF("Ready to read in config. Expecting %d (0x%03X) bytes as %d\n",
                    sz, sz, sz * 2);
                cliPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\n");
                cliPrintF("Times out if no character is received for %dms\n", Timeout);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliAvailable() && millis() - t < Timeout) {}
                    t = millis();

                    c = cliAvailable() ? cliRead() : '\0';
                    int8_t hex = parse_hex(c);
                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        chars_encountered++;
                    if (ignore)
                        continue;
                    if (hex == -1)
                        break;

                    *p |= second_nibble ? hex : hex << 4;
                    p += second_nibble;
                    second_nibble ^= 1;
                }

                if (c == 0)
                {
                    cliPrintF("Did not receive enough hex chars! (got %d, expected %d)\n",
                        (p - (uint8_t*)&e) * 2 + second_nibble, sz * 2);
                }
                else if (p < end || second_nibble)
                {
                    cliPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        chars_encountered, c, c);
                }
                else if (crcCheckVal != crc32bEEPROM(&e, true))
                {
                    cliPrintF("CRC mismatch! Not writing to in-memory config.\n");
                    cliPrintF("Here's what was received:\n\n");
                    cliPrintEEPROM(&e);
                }
                else
                {
                    // check to see if the newly received eeprom config
                    // actually differs from what's in-memory
                    zeroPIDintegralError();
                    zeroPIDstates();

                    int i;
                    for (i = 0; i < sz; i++)
                        if (((uint8_t*)&e)[i] != ((uint8_t*)&eepromConfig)[i])
                            break;

                    if (i == sz)
                    {
                        cliPrintF("NOTE: uploaded config was identical to in-memory config.\n");
                    }
                    else
                    {
                        eepromConfig = e;
                        cliPrintF("In-memory config updated!\n");
                        cliPrintF("NOTE: config not written to EEPROM; use 'W' to do so.\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something
                t = millis();
                while (millis() - t < Timeout)
                    if (cliAvailable())
                        cliRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'E': // Read in from EEPROM.  (EEPROM -> RAM)
                cliPrint("Re-reading EEPROM.\n");
                readEEPROM();
                validQuery = false;
                break;

            ///////////////////////////

            case 'x': // exit EEPROM CLI
                cliPrint("\nExiting EEPROM CLI....\n\n");
                cliBusy = false;
                return;
                break;

            ///////////////////////////

            case 'W':
            case 'e': // Write out to EEPROM. (RAM -> EEPROM)
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

            ///////////////////////////

            case 'f': // Write out to sdCard FILE. (RAM -> FILE)
                validQuery = false;
                break;

            ///////////////////////////

            case 'F': // Read in from sdCard FILE. (FILE -> RAM)
                validQuery = false;
                break;

            ///////////////////////////

            case 'V': // Reset EEPROM Parameters
                cliPrint( "\nEEPROM Parameters Reset....(not rebooting)\n" );
                checkFirstTime(true);
                validQuery = false;
            break;


            ///////////////////////////

            case '?':
            //                0         1         2         3         4         5         6         7
            //                01234567890123456789012345678901234567890123456789012345678901234567890123456789
                cliPrintF("\n");
                cliPrintF("'a' Display in-RAM config information\n");
                cliPrintF("'c' Write in-RAM -> Console (as Hex)      'C' Read Console (as Hex) -> in-RAM\n");
                cliPrintF("'e' Write in-RAM -> EEPROM                'E' Read EEPROM -> in-RAM\n");
                cliPrintF("'f' Write in-RAM -> sd FILE (Not yet imp) 'F' Read sd FILE -> in-RAM (Not imp)\n");
                cliPrintF("                                          'H' Clear CRC Bad History flag\n");
                cliPrintF("                                          'V' Reset in-RAM config to default.\n");
                cliPrintF("'x' Exit EEPROM CLI                       '?' Command Summary\n");
                cliPrintF("\n");
                cliPrintF("For compatability:                        'W' Write in-RAM -> EEPROM\n");
                cliPrintF("\n");
                break;

            ///////////////////////////
        }
    }

}

///////////////////////////////////////////////////////////////////////////////
