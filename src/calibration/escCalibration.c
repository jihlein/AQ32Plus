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
    uint16_t escCommand = MINCOMMAND;

    escCalibrating = true;

    armed = false;

    cliPortPrint("\nESC Calibration/MotorVerification:\n\n");
    cliPortPrint("!!!! CAUTION - Remove all propellers and disconnect !!!!\n");
    cliPortPrint("!!!! flight battery before proceeding any further   !!!!\n\n");
    cliPortPrint("Type 'Y' to continue, anything other character exits\n\n");

    while (cliPortAvailable() == false);
    temp = cliPortRead();
    if (temp != 'Y')
    {
    	cliPortPrint("ESC Calibration/Motor Verification Canceled!!\n\n");
    	escCalibrating = false;
    	return;
    }

    ///////////////////////////////////

    cliPortPrint("For ESC Calibration:\n");
    cliPortPrint("  Enter 'h' for Max Command....\n");
    cliPortPrint("  Enter 'm' for Mid Command....\n");
    cliPortPrint("  Enter 'l' for Min Command....\n");
    cliPortPrint("  Enter 'x' to exit....\n\n");

    ///////////////////////////////////

    while(true)
    {
		if (cliPortAvailable() == true)
		{
		    temp = cliPortRead();

		    switch (temp)
		    {
			    case 'h':
			        cliPortPrint("Applying Max Command....\n\n");
			        escCommand    = eepromConfig.maxThrottle;
			        break;

			    case 'm':
			        cliPortPrint("Applying Mid Command....\n\n");
			        escCommand    = eepromConfig.midCommand;
			        break;

			    case 'l':
			        cliPortPrint("Applying Min Command....\n\n");
			        escCommand    = MINCOMMAND;
			        break;

			    case 'x':
			        cliPortPrint("Applying Min Command, Exiting Calibration....\n\n");
			        writeAllMotors(MINCOMMAND);
			        escCalibrating = false;
			        return;
			        break;

			    case '?':
			        cliPortPrint("For ESC Calibration:\n");
			        cliPortPrint("  Enter 'h' for Max Command....\n");
			        cliPortPrint("  Enter 'm' for Mid Command....\n");
			        cliPortPrint("  Enter 'l' for Min Command....\n");
			        cliPortPrint("  Enter 'x' to exit....\n\n");
				    break;
		    }
		}

		writeAllMotors(escCommand);
        delayMicroseconds(2500);  // ~400 Hz
	}
}

///////////////////////////////////////////////////////////////////////////////
