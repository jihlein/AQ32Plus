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

    usbPrint("\nESC Calibration:\n\n");
    usbPrint("!!!! CAUTION - Remove all propellers and disconnect !!!!\n");
    usbPrint("!!!! flight battery before proceeding any further   !!!!\n\n");
    usbPrint("Type 'Y' to continue, anything other character exits\n\n");

    while (usbAvailable() == false);
    temp = usbRead();
    if (temp != 'Y')
    {
    	usbPrint("ESC Calibration Canceled!!\n\n");
    	escCalibrating = false;
    	return;
    }

    ///////////////////////////////////

    usbPrint("Enter 'h' for Max Command....\n");
    usbPrint("Enter 'm' for Mid Command....\n");
    usbPrint("Enter 'l' for Min Command....\n");
    usbPrint("Enter 'x' to exit....\n\n");

    while(true)
    {
		while (usbAvailable() == false);

		temp = usbRead();

		switch (temp)
		{
			case 'h':
			    usbPrint("Applying Max Command....\n\n");
			    writeAllMotors(eepromConfig.maxThrottle);
			    break;

			case 'm':
			    usbPrint("Applying Mid Command....\n\n");
			    writeAllMotors(eepromConfig.midCommand);
			    break;

			case 'l':
			    usbPrint("Applying Min Command....\n\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case 'x':
			    usbPrint("Applying Min Command, Exiting Calibration....\n\n");
			    writeAllMotors(MINCOMMAND);
			    escCalibrating = false;
			    return;
			    break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
