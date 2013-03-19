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

    cliBusy = true;

    usbPrint("\nEntering MAX7456 CLI....\n\n");

   	resetMax7456();

    while(true)
    {
		if (!validQuery) usbPrint("MAX7456 CLI -> ");

		while ((usbAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    max7456query = usbRead();

		if (!validQuery) usbPrint("\n");

		switch(max7456query)
		{
            ///////////////////////

            case 'a': // OSD Configuration
                usbPrint("\nMAX7456 OSD Status:             ");
                if (eepromConfig.osdEnabled)
                	usbPrint("Enabled\n");
                else
               	    usbPrint("Disabled\n");

                usbPrint("OSD Default Video Standard:     ");
                if (eepromConfig.defaultVideoStandard)
                    usbPrint("PAL\n");
                else
                    usbPrint("NTSC\n");

                usbPrint("OSD Display Units:              ");
                if (eepromConfig.metricUnits)
                    usbPrint("Metric\n");
                else
                    usbPrint("English\n");

                usbPrint("OSD Altitude Display:           ");
                if (eepromConfig.osdDisplayAlt)
                    usbPrint("On\n");
                else
                    usbPrint("Off\n");

                usbPrint("OSD Artifical Horizon Display:  ");
                if (eepromConfig.osdDisplayAH)
                    usbPrint("On\n");
                else
                    usbPrint("Off\n");

                usbPrint("OSD Attitude Display:           ");
                if (eepromConfig.osdDisplayAtt)
                    usbPrint("On\n");
                else
                    usbPrint("Off\n");

                usbPrint("OSD Heading Display:            ");
                if (eepromConfig.osdDisplayHdg)
                    usbPrint("On\n");
                else
                    usbPrint("Off\n");

                usbPrint("\n");
                validQuery = false;
                break;

            ///////////////////////

            case 'b': // Enable OSD Altitude Display
                eepromConfig.osdDisplayAlt  = true;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'c': // Enable OSD Artifical Horizon Display
                eepromConfig.osdDisplayAH  = true;
                eepromConfig.osdDisplayAtt = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'd': // Enable OSD Attitude Display
                eepromConfig.osdDisplayAtt = true;
                eepromConfig.osdDisplayAH  = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'e': // Enable OSD Heading Display
                eepromConfig.osdDisplayHdg = true;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'q': // Set English Display Units
                eepromConfig.metricUnits = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'r': // Reset MAX7456
                resetMax7456();
                usbPrint("\nMAX7456 Reset....\n\n");
                break;

            ///////////////////////

            case 's': // Show character set
                showMax7456Font();
                usbPrint("\nMAX7456 Character Set Displayed....\n\n");
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
   			    usbPrint("\nExiting MAX7456 CLI....\n\n");
   			    cliBusy = false;
   			    return;
   			    break;

            ///////////////////////

            case 'B': // Disable OSD Altitude Display
                eepromConfig.osdDisplayAlt = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'C': // Disable OSD Artifical Horizon Display
                eepromConfig.osdDisplayAH = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'D': // Disable OSD Attitude Display
                eepromConfig.osdDisplayAtt = false;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////

            case 'E': // Disable OSD Heading Display
                eepromConfig.osdDisplayHdg = false;

                max7456query = 'a';
                validQuery = true;
                break;

           ///////////////////////

           case 'Q': // Set Metric Display Units
                eepromConfig.metricUnits = true;

                max7456query = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                usbPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

    		///////////////////////

			case '?':
			   	usbPrint("\n");
			   	usbPrint("'a' OSD Configuration\n");
			    usbPrint("'b' Enable OSD Altitude Display            'B' Disable OSD Altitude Display\n");
			   	usbPrint("'c' Enable OSD Artificial Horizon Display  'C' Disable OSD Artificial Horizon Display\n");
			   	usbPrint("'d' Enable OSD Attitude Display            'D' Disable OSD Attitude Display\n");
			   	usbPrint("'e' Enable OSD Heading Display             'E' Disable OSD Heading Display\n");
			   	usbPrint("'q' Set English Display Units              'Q' Set Metric Display Units\n");
			    usbPrint("'r' Reset MAX7456\n");
			   	usbPrint("'s' Display MAX7456 Character Set\n");
			   	usbPrint("'t' Download Font to MAX7456\n");
			   	usbPrint("'u' Toggle OSD Enabled State\n");
			   	usbPrint("'v' Toggle Default Video Standard          'W' Write EEPROM Parameters\n");
			   	usbPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
			   	usbPrint("\n");
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

    usbPrint("\nEntering Mixer CLI....\n\n");

    while(true)
    {
        usbPrint("Mixer CLI -> ");

		while ((usbAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    mixerQuery = usbRead();

		usbPrint("\n");

		switch(mixerQuery)
		{
            ///////////////////////////

            case 'a': // Mixer Configuration
                usbPrint("\nMixer Configuration:            ");
                switch (eepromConfig.mixerConfiguration)
                {
                    case MIXERTYPE_GIMBAL:
                    	usbPrint("MIXERTYPE GIMBAL\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_FLYING_WING:
                    	usbPrint("MIXERTYPE FLYING WING\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_BI:
                        usbPrint("MIXERTYPE BICOPTER\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_TRI:
                        usbPrint("MIXERTYPE TRICOPTER\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_QUADP:
                        usbPrint("MIXERTYPE QUAD PLUS\n");
                        break;

                    case MIXERTYPE_QUADX:
                        usbPrint("MIXERTYPE QUAD X\n");
                        break;

                    case MIXERTYPE_VTAIL4_NO_COMP:
                    	usbPrint("MULTITYPE VTAIL NO COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_Y_COMP:
                    	usbPrint("MULTITYPE VTAIL Y COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RY_COMP:
                    	usbPrint("MULTITYPE VTAIL RY COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_PY_COMP:
                    	usbPrint("MULTITYPE VTAIL PY COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RP_COMP:
                    	usbPrint("MULTITYPE VTAIL RP COMP\n");
                    	break;

                    case MIXERTYPE_VTAIL4_RPY_COMP:
                    	usbPrint("MULTITYPE VTAIL RPY COMP\n");
                    	break;

                    case MIXERTYPE_Y4:
                    	usbPrint("MIXERTYPE Y4\n");
                    	break;

                    ///////////////////////

                    case MIXERTYPE_HEX6P:
                        usbPrint("MIXERTYPE HEX PLUS\n");
                        break;

                    case MIXERTYPE_HEX6X:
                        usbPrint("MIXERTYPE HEX X\n");
                        break;

                    case MIXERTYPE_Y6:
                        usbPrint("MIXERTYPE Y6\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_OCTOF8P:
                        usbPrint("MIXERTYPE FLAT OCTO PLUS\n");
                        break;

                    case MIXERTYPE_OCTOF8X:
                        usbPrint("MIXERTYPE FLAT OCTO X\n");
                        break;

                    case MIXERTYPE_OCTOX8P:
                        usbPrint("MIXERTYPE COAXIAL OCTO PLUS\n");
                        break;

                    case MIXERTYPE_OCTOX8X:
                        usbPrint("MIXERTYPE COAXIAL OCTO X\n");
                        break;

                    ///////////////////////

                    case MIXERTYPE_FREEMIX:
                    	usbPrint("MIXERTYPE FREE MIX\n");
                    	break;
                }

                usbPrint("Number of Motors:               ");
                itoa(numberMotor,                         numberString, 10); usbPrint(numberString); usbPrint("\n");

                usbPrint("ESC PWM Rate:                   ");
                itoa((uint16_t)eepromConfig.escPwmRate,   numberString, 10); usbPrint(numberString); usbPrint("\n");

                usbPrint("Servo PWM Rate:                 ");
                itoa((uint16_t)eepromConfig.servoPwmRate, numberString, 10); usbPrint(numberString); usbPrint("\n");

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_BI )
                {
                    usbPrint("BiCopter Left Servo Min:        ");
                    itoa((uint16_t)eepromConfig.biLeftServoMin,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("BiCopter Left Servo Mid:        ");
                    itoa((uint16_t)eepromConfig.biLeftServoMid,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("BiCopter Left Servo Max:        ");
                    itoa((uint16_t)eepromConfig.biLeftServoMax,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("BiCopter Right Servo Min:       ");
                    itoa((uint16_t)eepromConfig.biRightServoMin, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("BiCopter Right Servo Mid:       ");
                    itoa((uint16_t)eepromConfig.biRightServoMid, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("BiCopter Right Servo Max:       ");
                    itoa((uint16_t)eepromConfig.biRightServoMax, numberString, 10); usbPrint(numberString); usbPrint("\n");
                }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_FLYING_WING )
                {
                    usbPrint("Roll Direction Left:            ");
                    itoa((uint16_t)eepromConfig.rollDirectionLeft,   numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Roll Direction Right:           ");
                    itoa((uint16_t)eepromConfig.rollDirectionRight,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Pitch Direction Left:           ");
                    itoa((uint16_t)eepromConfig.pitchDirectionLeft,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Pitch Direction Right:          ");
                    itoa((uint16_t)eepromConfig.pitchDirectionRight, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Wing Left Minimum:              ");
                    itoa((uint16_t)eepromConfig.wingLeftMinimum,     numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Wing Left Maximum:              ");
                    itoa((uint16_t)eepromConfig.wingLeftMaximum,     numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Wing Right Minimum:             ");
                    itoa((uint16_t)eepromConfig.wingRightMinimum,    numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Wing Right Maximum:             ");
                    itoa((uint16_t)eepromConfig.wingRightMaximum,    numberString, 10); usbPrint(numberString); usbPrint("\n");
                }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_GIMBAL )
                {
                    usbPrint("Gimbal Roll Servo Min:          ");
                    itoa((uint16_t)eepromConfig.gimbalRollServoMin,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Roll Servo Mid:          ");
                    itoa((uint16_t)eepromConfig.gimbalRollServoMid,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Roll Servo Max:          ");
                    itoa((uint16_t)eepromConfig.gimbalRollServoMax,  numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Roll Servo Gain:        ");
                    ftoa(eepromConfig.gimbalRollServoGain, numberString);               usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Pitch Servo Min:         ");
                    itoa((uint16_t)eepromConfig.gimbalPitchServoMin, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Pitch Servo Mid:         ");
                    itoa((uint16_t)eepromConfig.gimbalPitchServoMid, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Pitch Servo Max:         ");
                    itoa((uint16_t)eepromConfig.gimbalPitchServoMax, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("Gimbal Pitch Servo Gain:       ");
                    ftoa(eepromConfig.gimbalPitchServoGain, numberString);              usbPrint(numberString); usbPrint("\n");
                }

                if ( eepromConfig.mixerConfiguration == MIXERTYPE_TRI )
                {
                    usbPrint("TriCopter Yaw Servo Min:        ");
                    itoa((uint16_t)eepromConfig.triYawServoMin, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("TriCopter Yaw Servo Mid:        ");
                    itoa((uint16_t)eepromConfig.triYawServoMid, numberString, 10); usbPrint(numberString); usbPrint("\n");
                    usbPrint("TriCopter Yaw Servo Max:        ");
                    itoa((uint16_t)eepromConfig.triYawServoMax, numberString, 10); usbPrint(numberString); usbPrint("\n");
                }

                if (eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_Y_COMP  ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RY_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_PY_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RP_COMP ||
                    eepromConfig.mixerConfiguration == MIXERTYPE_VTAIL4_RPY_COMP)
                {
                    usbPrint("V Tail Angle                   ");
                    ftoa(eepromConfig.vTailAngle, numberString); usbPrint(numberString); usbPrint("\n");
    			}

                usbPrint("Yaw Direction:                  ");
                itoa((int8_t)eepromConfig.yawDirection,   numberString, 10); usbPrint(numberString); usbPrint("\n\n");

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Free Mix Matrix
        	    usbPrint("\nNumber of Free Mixer Motors:  ");
        	    itoa( eepromConfig.freeMixMotors, numberString, 10 ); usbPrint( numberString ); usbPrint("\n\n");
                usbPrint("         Roll    Pitch   Yaw\n");

        	    for ( index = 0; index < eepromConfig.freeMixMotors; index++ )
        	    {
        	    	usbPrint("Motor"); itoa(index, numberString, 10);       usbPrint(numberString); usbPrint("  ");
        	    	ftoa(eepromConfig.freeMix[index][ROLL ], numberString); usbPrint(numberString); usbPrint("  ");
        	    	ftoa(eepromConfig.freeMix[index][PITCH], numberString); usbPrint(numberString); usbPrint("  ");
        	    	ftoa(eepromConfig.freeMix[index][YAW  ], numberString); usbPrint(numberString); usbPrint("\n");
        	    }

        	    usbPrint("\n");
        	    validQuery = false;
        	    break;

            ///////////////////////////

			case 'x':
			    usbPrint("\nExiting Mixer CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read Mixer Configuration
                eepromConfig.mixerConfiguration = (uint8_t)readFloatUsb();
                initMixer();

        	    mixerQuery = 'a';
                validQuery = true;
		        break;

            ///////////////////////////

            case 'B': // Read ESC and Servo PWM Update Rates
                eepromConfig.escPwmRate   = (uint16_t)readFloatUsb();
                eepromConfig.servoPwmRate = (uint16_t)readFloatUsb();

                pwmEscInit(eepromConfig.escPwmRate);
                pwmServoInit(eepromConfig.servoPwmRate);

                mixerQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Read BiCopter Left Servo Parameters
           	    eepromConfig.biLeftServoMin = readFloatUsb();
           	    eepromConfig.biLeftServoMid = readFloatUsb();
           	    eepromConfig.biLeftServoMax = readFloatUsb();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read BiCopter Right Servo Parameters
           	    eepromConfig.biRightServoMin = readFloatUsb();
           	    eepromConfig.biRightServoMid = readFloatUsb();
           	    eepromConfig.biRightServoMax = readFloatUsb();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Flying Wing Servo Directions
                eepromConfig.rollDirectionLeft   = readFloatUsb();
                eepromConfig.rollDirectionRight  = readFloatUsb();
                eepromConfig.pitchDirectionLeft  = readFloatUsb();
                eepromConfig.pitchDirectionRight = readFloatUsb();

         	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Flying Wing Servo Limits
           	    eepromConfig.wingLeftMinimum  = readFloatUsb();
           	    eepromConfig.wingLeftMaximum  = readFloatUsb();
           	    eepromConfig.wingRightMinimum = readFloatUsb();
           	    eepromConfig.wingRightMaximum = readFloatUsb();

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Free Mix Motor Number
           	    eepromConfig.freeMixMotors = (uint8_t)readFloatUsb();
           	    initMixer();

           	    mixerQuery = 'b';
                validQuery = true;
                break;

            ///////////////////////////

            case 'H': // Read Free Mix Matrix Element
                rows    = (uint8_t)readFloatUsb();
                columns = (uint8_t)readFloatUsb();
                eepromConfig.freeMix[rows][columns] = readFloatUsb();

                mixerQuery = 'b';
                validQuery = true;
                break;

            ///////////////////////////

            case 'I': // Read Gimbal Roll Servo Parameters
         	    eepromConfig.gimbalRollServoMin  = readFloatUsb();
           	    eepromConfig.gimbalRollServoMid  = readFloatUsb();
           	    eepromConfig.gimbalRollServoMax  = readFloatUsb();
           	    eepromConfig.gimbalRollServoGain = readFloatUsb();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'J': // Read Gimbal Pitch Servo Parameters
           	    eepromConfig.gimbalPitchServoMin  = readFloatUsb();
           	    eepromConfig.gimbalPitchServoMid  = readFloatUsb();
           	    eepromConfig.gimbalPitchServoMax  = readFloatUsb();
           	    eepromConfig.gimbalPitchServoGain = readFloatUsb();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'K': // Read TriCopter YawServo Parameters
        	    eepromConfig.triYawServoMin = readFloatUsb();
           	    eepromConfig.triYawServoMid = readFloatUsb();
           	    eepromConfig.triYawServoMax = readFloatUsb();

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'L': // Read V Tail Angle
        	    eepromConfig.vTailAngle = readFloatUsb();

        	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'M': // Read yaw direction
                tempFloat = readFloatUsb();
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
                usbPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	usbPrint("\n");
			   	usbPrint("'a' Mixer Configuration Data               'A' Set Mixer Configuration              A1 thru 21, see aq32Plus.h\n");
   		        usbPrint("'b' Free Mixer Configuration               'B' Set PWM Rates                        BESC;Servo\n");
			   	usbPrint("                                           'C' Set BiCopter Left Servo Parameters   CMin;Mid;Max\n");
			   	usbPrint("                                           'D' Set BiCopter Right Servo Parameters  DMin;Mid;Max\n");
			   	usbPrint("                                           'E' Set Flying Wing Servo Directions     ERollLeft;RollRight;PitchLeft;PitchRight\n");
			   	usbPrint("                                           'F' Set Flying Wing Servo Limits         FLeftMin;LeftMax;RightMin;RightMax\n");
   		        usbPrint("                                           'G' Set Number of FreeMix Motors         GNumber\n");
   		        usbPrint("                                           'H' Set FreeMix Matrix Element           HRow;Column;Element\n");
   		        usbPrint("                                           'I' Set Gimbal Roll Servo Parameters     IMin;Mid;Max;Gain\n");
   		        usbPrint("                                           'J' Set Gimbal Pitch Servo Parameters    JMin;Mid;Max;Gain\n");
   		        usbPrint("                                           'K' Set TriCopter Servo Parameters       KMin;Mid;Max\n");
   		        usbPrint("                                           'L' Set V Tail Angle                     LAngle\n");
   		        usbPrint("                                           'M' Set Yaw Direction                    M1 or M-1\n");
   		        usbPrint("                                           'W' Write EEPROM Parameters\n");
   		        usbPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
   		        usbPrint("\n");
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

    usbPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        usbPrint("Receiver CLI -> ");

		while ((usbAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = usbRead();

		usbPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                usbPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PARALLEL_PWM:
                        usbPrint("Parallel\n");
                        break;
                    case SERIAL_PWM:
                        usbPrint("Serial\n");
                        break;
                    case SPEKTRUM:
                        usbPrint("Spektrum\n");
                        break;
		        }

                usbPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                usbPrint(rcOrderString);  usbPrint("\n");

                usbPrint("Spektrum Resolution:            ");
                if (eepromConfig.spektrumHires)
				    usbPrint("11 Bit Mode\n");
				else
				    usbPrint("10 Bit Mode\n");

				usbPrint("Number of Spektrum Channels:    ");
				snprintf(numberString, 16, "%d\n", eepromConfig.spektrumChannels); usbPrint(numberString);

                usbPrint("Mid Command:                    ");
                snprintf(numberString, 16, "%d\n", (uint16_t)eepromConfig.midCommand); usbPrint(numberString);

				usbPrint("Min Check:                      ");
                snprintf(numberString, 16, "%d\n", (uint16_t)eepromConfig.minCheck); usbPrint(numberString);

				usbPrint("Max Check:                      ");
                snprintf(numberString, 16, "%d\n", (uint16_t)eepromConfig.maxCheck); usbPrint(numberString);

				usbPrint("Min Throttle:                   ");
                snprintf(numberString, 16, "%d\n", (uint16_t)eepromConfig.minThrottle); usbPrint(numberString);

				usbPrint("Max Thottle:                    ");
                snprintf(numberString, 16, "%d\n\n", (uint16_t)eepromConfig.maxThrottle); usbPrint(numberString);

				usbPrint("Max Rate Command:               ");
				tempFloat = eepromConfig.rateScaling * 180000.0 / PI;
				snprintf(numberString, 16, "%6.2f DPS\n", tempFloat); usbPrint(numberString);

				usbPrint("Max Attitude Command:           ");
				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
				snprintf(numberString, 18, "%6.2f Degrees\n\n", tempFloat); usbPrint(numberString);

				validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Read Max Rate Value
                eepromConfig.rateScaling = readFloatUsb() / 180000 * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatUsb() / 180000 * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

			case 'x':
			    usbPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read RX Input Type
                eepromConfig.receiverType = (uint8_t)readFloatUsb();
			    usbPrint( "\nReceiver Type Changed....\n");

			    usbPrint("\nSystem Resetting....\n");
			    delay(100);
			    writeEEPROM();
			    systemReset(false);

		        break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringUsb( rcOrderString, 8 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Read Spektrum Resolution
                eepromConfig.spektrumHires = (uint8_t)readFloatUsb();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read Number of Spektrum Channels
                eepromConfig.spektrumChannels = (uint8_t)readFloatUsb();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read RC Control Points
                eepromConfig.midCommand   = readFloatUsb();
    	        eepromConfig.minCheck     = readFloatUsb();
    		    eepromConfig.maxCheck     = readFloatUsb();
    		    eepromConfig.minThrottle  = readFloatUsb();
    		    eepromConfig.maxThrottle  = readFloatUsb();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                usbPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	usbPrint("\n");
			   	usbPrint("'a' Receiver Configuration Data            'A' Set RX Input Type                    AX, 1=Parallel, 2=Serial, 3=Spektrum\n");
   		        usbPrint("'b' Set Maximum Rate Command               'B' Set RC Control Order                 BTAER1234\n");
			   	usbPrint("'c' Set Maximum Attitude Command           'C' Set Spektrum Resolution              C0 or C1\n");
			   	usbPrint("                                           'D' Set Number of Spektrum Channels      D6 thru D12\n");
			   	usbPrint("                                           'E' Set RC Control Points                EmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	usbPrint("                                           'W' Write EEPROM Parameters\n");
			   	usbPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	usbPrint("\n");
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
    uint8_t  validQuery = false;

    cliBusy = true;

    usbPrint("\nEntering Sensor CLI....\n\n");

    while(true)
    {
        usbPrint("Sensor CLI -> ");

		while ((usbAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = usbRead();

		usbPrint("\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data
                usbPrint("\n");

                usbPrint("Accel One G:               ");
                snprintf(numberString, 16, "%9.4f\n", accelOneG); usbPrint(numberString);

                usbPrint("Accel Temp Comp Slope:     ");
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.accelTCBiasSlope[XAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.accelTCBiasSlope[YAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.accelTCBiasSlope[ZAXIS]); usbPrint(numberString);

                usbPrint("Accel Temp Comp Bias:      ");
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.accelTCBiasIntercept[XAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.accelTCBiasIntercept[XAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.accelTCBiasIntercept[XAXIS]); usbPrint(numberString);

                usbPrint("Gyro Temp Comp Slope:      ");
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.gyroTCBiasSlope[ROLL ]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.gyroTCBiasSlope[PITCH]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.gyroTCBiasSlope[YAW  ]); usbPrint(numberString);

                usbPrint("Gyro Temp Comp Intercept:  ");
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.gyroTCBiasIntercept[ROLL ]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.gyroTCBiasIntercept[PITCH]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.gyroTCBiasIntercept[YAW  ]); usbPrint(numberString);

                usbPrint("Mag Bias:                  ");
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.magBias[XAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f, ", eepromConfig.magBias[YAXIS]); usbPrint(numberString);
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.magBias[ZAXIS]); usbPrint(numberString);

                usbPrint("Accel Cutoff:              ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.accelCutoff); usbPrint(numberString);

                usbPrint("KpAcc (MARG):              ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.KpAcc); usbPrint(numberString);

                usbPrint("KiAcc (MARG):              ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.KiAcc); usbPrint(numberString);

                usbPrint("KpMag (MARG):              ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.KpMag); usbPrint(numberString);

                usbPrint("KiMag (MARG):              ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.KiMag); usbPrint(numberString);

                usbPrint("hdot est/h est Comp Fil A: ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.compFilterA); usbPrint(numberString);

                usbPrint("hdot est/h est Comp Fil B: ");
                snprintf(numberString, 16, "%9.4f\n", eepromConfig.compFilterB); usbPrint(numberString);

                usbPrint("MPU6000 DLPF:                 ");
                switch(eepromConfig.dlpfSetting)
                {
                    case DLPF_256HZ:
                        usbPrint("256 Hz\n");
                        break;
                    case DLPF_188HZ:
                        usbPrint("188 Hz\n");
                        break;
                    case DLPF_98HZ:
                        usbPrint("98 Hz\n");
                        break;
                    case DLPF_42HZ:
                        usbPrint("42 Hz\n");
                        break;
                }

                usbPrint("Magnetic Variation:           ");
                if (eepromConfig.magVar >= 0.0f)
                  snprintf(numberString, 16, "E%6.4f\n\n",  eepromConfig.magVar * R2D);
                else
                  snprintf(numberString, 16, "W%6.4f\n\n", -eepromConfig.magVar * R2D);

                usbPrint(numberString);

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

			case 'x':
			    usbPrint("\nExiting Sensor CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6000 Digital Low Pass Filter
                tempInt = (uint8_t)readFloatUsb();

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
                eepromConfig.accelCutoff = readFloatUsb();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc, kiAcc
                eepromConfig.KpAcc = readFloatUsb();
                eepromConfig.KiAcc = readFloatUsb();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag, kiMag
                eepromConfig.KpMag = readFloatUsb();
                eepromConfig.KiMag = readFloatUsb();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // h dot est/h est Comp Filter A/B
                eepromConfig.compFilterA = readFloatUsb();
                eepromConfig.compFilterB = readFloatUsb();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'M': // Magnetic Variation
                eepromConfig.magVar = readFloatUsb() * D2R;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                usbPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	usbPrint("\n");
			   	usbPrint("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3, see aq32Plus.h\n");
			   	usbPrint("'b' MPU6000 Calibration                    'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	usbPrint("'c' Magnetometer Calibration               'C' Set kpAcc/kiAcc                      CkpAcc;kiAcc\n");
			   	usbPrint("                                           'D' Set kpMag/kiMag                      DkpMag;kiMag\n");
			   	usbPrint("                                           'E' Set h dot est/h est Comp Filter A/B  EA;B\n");
			   	usbPrint("                                           'M' Set Mag Variation (+ East, - West)   MMagVar\n");
			   	usbPrint("                                           'W' Write EEPROM Parameters\n");
			   	usbPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
			    usbPrint("\n");
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
    uint8_t  gpsQuery;
    uint8_t  validQuery = false;

    cliBusy = true;

    usbPrint("\nEntering GPS CLI....\n\n");

    while(true)
    {
        usbPrint("GPS CLI -> ");

		while ((usbAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    gpsQuery = usbRead();

		usbPrint("\n");

		switch(gpsQuery)
		{
            ///////////////////////////

            case 'a': // GPS Installation Data
                usbPrint("\n");

				switch(eepromConfig.gpsType)
				{
					///////////////

					case NO_GPS:
					    usbPrint("No GPS Installed....\n\n");
					    break;

					///////////////

					case MEDIATEK_3329_BINARY:
					    usbPrint("MediaTek 3329 GPS installed, Binary Mode....\n\n");
					    break;

					///////////////

					case MEDIATEK_3329_NMEA:
					    usbPrint("MediaTek 3329 GPS Installed, NMEA Mode....\n\n");
					    break;

					///////////////

					case UBLOX:
					    usbPrint("UBLOX GPS Installed, Binary Mode....\n\n");
					    break;

					///////////////
				}

                validQuery = false;
                break;

            ///////////////////////////

			case 'x':
			    usbPrint("\nExiting GPS CLI....\n\n");
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

            case 'W': // Write EEPROM Parameters
                usbPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	usbPrint("\n");
			   	usbPrint("'a' Display GPS Installation Data          'A' Set GPS Type to No GPS\n");
			   	usbPrint("                                           'B' Set GPS Type to MediaTek 3329 Binary\n");
			   	usbPrint("                                           'C' Set GPS Type to MediaTek 3329 NMEA\n");
			   	usbPrint("                                           'D' Set GPS Type to UBLOX\n");
			   	usbPrint("                                           'W' Write EEPROM Parameters\n");
			   	usbPrint("'x' Exit GPS CLI                           '?' Command Summary\n");
			    usbPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
