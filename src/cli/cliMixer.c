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
// Mixer CLI
///////////////////////////////////////////////////////////////////////////////

void mixerCLI()
{
    float    tempFloat;

    uint8_t  index;
    uint8_t  rows, columns;

    uint8_t  mixerQuery = 'x';
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
                cliPrint("\nMixer Configuration:  ");
                switch (eepromConfig.mixerConfiguration)
                {
                    case MIXERTYPE_TRI:
                        cliPrint("   MIXERTYPE TRI\n");
                        break;

                    case MIXERTYPE_QUADX:
                        cliPrint("MIXERTYPE QUAD X\n");
                        break;

                    case MIXERTYPE_HEX6X:
                        cliPrint(" MIXERTYPE HEX X\n");
                        break;

                    case MIXERTYPE_FREE:
                        cliPrint("  MIXERTYPE FREE\n");
                        break;
                }

                cliPrintF("Number of Motors:                    %1d\n",  numberMotor);
                cliPrintF("ESC PWM Rate:                      %3ld\n",   eepromConfig.escPwmRate);
                cliPrintF("Servo PWM Rate:                    %3ld\n",   eepromConfig.servoPwmRate);

                if (eepromConfig.yawDirection == 1.0f)
                	cliPrintF("Yaw Direction:                  Normal\n\n");
                else if (eepromConfig.yawDirection == -1.0f)
                	cliPrintF("Yaw Direction:                 Reverse\n\n");
                else
                	cliPrintF("Yaw Direction:              Undefined\n\n");

                if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
					cliPrintF("TriCopter Yaw Servo PWM Rate:      %3ld\n",  eepromConfig.triYawServoPwmRate);
                    cliPrintF("TriCopter Yaw Servo Min PWM:      %4ld\n",   (uint16_t)eepromConfig.triYawServoMin);
                    cliPrintF("TriCopter Yaw Servo Mid PWM:      %4ld\n",   (uint16_t)eepromConfig.triYawServoMid);
                    cliPrintF("TriCopter Yaw Servo Max PWM:      %4ld\n\n", (uint16_t)eepromConfig.triYawServoMax);
                    cliPrintF("Tricopter Yaw Cmd Time Constant:  %5.3f\n\n", eepromConfig.triCopterYawCmd500HzLowPassTau);
			    }

        	    if (eepromConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
					cliPrintF("\nNumber of Free Mixer Motors:  %1d\n         Roll    Pitch   Yaw\n\n", eepromConfig.freeMixMotors);

        	        for ( index = 0; index < eepromConfig.freeMixMotors; index++ )
        	        {
        	    	    cliPrintF("Motor%1d  %6.3f  %6.3f  %6.3f\n", index,
        	    			                                         eepromConfig.freeMix[index][ROLL ],
        	    			                                         eepromConfig.freeMix[index][PITCH],
        	    			                                         eepromConfig.freeMix[index][YAW  ]);
        	        }

        	        cliPrint("\n");
			    }

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
                pwmEscInit();

        	    mixerQuery = 'a';
                validQuery = true;
		        break;

            ///////////////////////////

            case 'B': // Read ESC and Servo PWM Update Rates
                eepromConfig.escPwmRate   = (uint16_t)readFloatCLI();
                eepromConfig.servoPwmRate = (uint16_t)readFloatCLI();

                pwmEscInit();
                pwmServoInit();

                mixerQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'D': // Read yaw direction
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

            case 'E': // Read TriCopter Servo PWM Rate
            	if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
               	{
               		eepromConfig.triYawServoPwmRate = (uint16_t)readFloatCLI();

                    pwmEscInit();
               	}
                else
                {
                   	tempFloat = readFloatCLI();

                   	cliPrintF("\nTriCopter Mixing not Selected....\n\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read TriCopter Servo Min Point
               	if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
               	{
               		eepromConfig.triYawServoMin = readFloatCLI();

               		pwmEscWrite(7, (uint16_t)eepromConfig.triYawServoMin);
                }
                else
                {
                   	tempFloat = readFloatCLI();

                    cliPrintF("\nTriCopter Mixing not Selected....\n\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read TriCopter Servo Mid Point
                if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                    eepromConfig.triYawServoMid = readFloatCLI();

                    pwmEscWrite(7, (uint16_t)eepromConfig.triYawServoMid);
                }
                else
                {
                   	tempFloat = readFloatCLI();

                   	cliPrintF("\nTriCopter Mixing not Selected....\n\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'H': // Read TriCopter Servo Max Point
                if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                    eepromConfig.triYawServoMax = readFloatCLI();

                    pwmEscWrite(7, (uint16_t)eepromConfig.triYawServoMax);
                }
                else
                {
                    tempFloat = readFloatCLI();

                    cliPrintF("\nTriCopter Mixing not Selected....\n\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'I': // Read TriCopter Yaw Command Time Constant
                if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                	eepromConfig.triCopterYawCmd500HzLowPassTau = readFloatCLI();

                	initFirstOrderFilter();
                }
                else
                {
                    tempFloat = readFloatCLI();

                    cliPrintF("\nTriCopter Mixing not Selected....\n\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'J': // Read Free Mix Motor Number
           	    if (eepromConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
                	eepromConfig.freeMixMotors = (uint8_t)readFloatCLI();
           	        initMixer();
				}
				else
				{
					tempFloat = readFloatCLI();

					cliPrintF("\nFree Mix not Selected....\n\n");
                }

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'K': // Read Free Mix Matrix Element
                if (eepromConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
                	rows    = (uint8_t)readFloatCLI() - 1;
                    columns = (uint8_t)readFloatCLI() - 1;

                    eepromConfig.freeMix[rows][columns] = readFloatCLI();
				}
				else
				{
					tempFloat = readFloatCLI();
					tempFloat = readFloatCLI();
					tempFloat = readFloatCLI();

					cliPrintF("\nFree Mix not Selected....\n\n");
                }

           	    mixerQuery = 'a';
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
			   	cliPrint("'a' Mixer Configuration Data               'A' Set Mixer Configuration              A0 thru 3, see aq32Plus.h\n");
   		        cliPrint("                                           'B' Set PWM Rates                        BESC;Servo\n");
			   	cliPrint("                                           'D' Set Yaw Direction                    D1 or D-1\n");

   		        if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
   		    	{
   		        	cliPrint("                                           'E' Set TriCopter Servo PWM Rate         ERate\n");
   		        	cliPrint("                                           'F' Set TriCopter Servo Min Point        FMin\n");
			   	    cliPrint("                                           'G' Set TriCopter Servo Mid Point        GMid\n");
			   	    cliPrint("                                           'H' Set TriCopter Servo Max Point        HMax\n");
			   	    cliPrint("                                           'I' Set TriCopter Yaw Cmd Time Constant  ITimeConstant\n");
   		    	}

   		        if (eepromConfig.mixerConfiguration == MIXERTYPE_FREE)
   		    	{
   		        	cliPrint("                                           'J' Set Number of FreeMix Motors         JNumb\n");
   		        	cliPrint("                                           'K' Set FreeMix Matrix Element           KRow;Col;Value\n");
			   	}

   		        cliPrint("                                           'W' Write EEPROM Parameters\n");
   		        cliPrint("'x' Exit Mixer CLI                         '?' Command Summary\n\n");
   		        break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
