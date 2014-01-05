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

uint8_t escCalibrating = false;
char    temp;

///////////////////////////////////////////////////////////////////////////////
// ESC Calibration
///////////////////////////////////////////////////////////////////////////////

void escCalibration(void)
{
    escCalibrating = true;

    armed = false;

    cliPrint("\nESC Calibration:\n\n");
    cliPrint("!!!! CAUTION - Remove all propellers and disconnect !!!!\n");
    cliPrint("!!!! flight battery before proceeding any further   !!!!\n\n");
    cliPrint("Type 'Y' to continue, anything other character exits\n\n");

    while (cliAvailable() == false);
    temp = cliRead();
    if (temp != 'Y')
    {
    	cliPrint("ESC Calibration Canceled!!\n\n");
    	escCalibrating = false;
    	return;
    }

    ///////////////////////////////////

    cliPrint("For ESC Calibration:\n");
    cliPrint("  Enter 'h' for Max Command....\n");
    cliPrint("  Enter 'm' for Mid Command....\n");
    cliPrint("  Enter 'l' for Min Command....\n");
    cliPrint("  Enter 'x' to exit....\n\n");
    cliPrint("For Motor Order Verification:\n");
    cliPrint("  Enter '0' to turn off all motors....\n");
    cliPrint("  Enter '1' to turn on Motor1....\n");
    cliPrint("  Enter '2' to turn on Motor2....\n");
    cliPrint("  Enter '3' to turn on Motor3....\n");
    cliPrint("  Enter '4' to turn on Motor4....\n");
    cliPrint("  Enter '5' to turn on Motor5....\n");
    cliPrint("  Enter '6' to turn on Motor6....\n");
    cliPrint("  Enter '7' to turn on Motor7....\n");
    cliPrint("  Enter '8' to turn on Motor8....\n\n");

    ///////////////////////////////////

    while(true)
    {
		while (cliAvailable() == false);

		temp = cliRead();

		switch (temp)
		{
			case 'h':
			    cliPrint("Applying Max Command....\n\n");
			    writeAllMotors(eepromConfig.maxThrottle);
			    break;

			case 'm':
			    cliPrint("Applying Mid Command....\n\n");
			    writeAllMotors(eepromConfig.midCommand);
			    break;

			case 'l':
			    cliPrint("Applying Min Command....\n\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case 'x':
			    cliPrint("Applying Min Command, Exiting Calibration....\n\n");
			    writeAllMotors(MINCOMMAND);
			    escCalibrating = false;
			    return;
			    break;

			case '0':
			    cliPrint("Motors at Min Command....\n\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case '1':
				cliPrint("Motor1 at Min Throttle....\n\n");
				pwmEscWrite(0, eepromConfig.minThrottle);
				break;

			case '2':
				cliPrint("Motor2 at Min Throttle....\n\n");
				pwmEscWrite(1, eepromConfig.minThrottle);
				break;

			case '3':
				cliPrint("Motor3 at Min Throttle....\n\n");
				pwmEscWrite(2, eepromConfig.minThrottle);
				break;

			case '4':
				cliPrint("Motor4 at Min Throttle....\n\n");
				pwmEscWrite(3, eepromConfig.minThrottle);
				break;

			case '5':
				cliPrint("Motor5 at Min Throttle....\n\n");
				pwmEscWrite(4, eepromConfig.minThrottle);
				break;

			case '6':
				cliPrint("Motor6 at Min Throttle....\n\n");
				pwmEscWrite(5, eepromConfig.minThrottle);
				break;

			case '7':
				cliPrint("Motor7 at Min Throttle....\n\n");
				pwmEscWrite(6, eepromConfig.minThrottle);
				break;

			case '8':
				cliPrint("Motor8 at Min Throttle....\n\n");
				pwmEscWrite(7, eepromConfig.minThrottle);
				break;

			case '?':
			    cliPrint("For ESC Calibration:\n");
			    cliPrint("  Enter 'h' for Max Command....\n");
			    cliPrint("  Enter 'm' for Mid Command....\n");
			    cliPrint("  Enter 'l' for Min Command....\n");
			    cliPrint("  Enter 'x' to exit....\n\n");
			    cliPrint("For Motor Order Verification:\n");
			    cliPrint("  Enter '0' to turn off all motors....\n");
			    cliPrint("  Enter '1' to turn on Motor1....\n");
			    cliPrint("  Enter '2' to turn on Motor2....\n");
			    cliPrint("  Enter '3' to turn on Motor3....\n");
			    cliPrint("  Enter '4' to turn on Motor4....\n");
			    cliPrint("  Enter '5' to turn on Motor5....\n");
			    cliPrint("  Enter '6' to turn on Motor6....\n");
			    cliPrint("  Enter '7' to turn on Motor7....\n");
			    cliPrint("  Enter '8' to turn on Motor8....\n\n");
				break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
