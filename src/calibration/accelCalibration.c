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
// Accelerometer Calibration
///////////////////////////////////////////////////////////////////////////////

void accelCalibration(void)
{
    double noseUpMXR        = 0.0f;
	double noseDownMXR      = 0.0f;
	double leftWingDownMXR  = 0.0f;
	double rightWingDownMXR = 0.0f;
	double upSideDownMXR    = 0.0f;
    double rightSideUpMXR   = 0.0f;

    int16_t index;

    accelCalibrating = true;

    cliPrint("\nAccelerometer Calibration:\n\n");

    ///////////////////////////////////

    cliPrint("Place accelerometer right side up\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        rightSideUpMXR += mxr9150ZAxis();;
        delayMicroseconds(1000);
    }

    rightSideUpMXR /= 5000.0;

    cliPrint("Place accelerometer up side down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        upSideDownMXR += mxr9150ZAxis();
        delayMicroseconds(1000);
    }

    upSideDownMXR /= 5000.0;

    eepromConfig.accelBiasMXR[ZAXIS] = (float)((rightSideUpMXR + upSideDownMXR) / 2.0);

    eepromConfig.accelScaleFactorMXR[ZAXIS] = (float)((2.0 * 9.8065) / (abs(rightSideUpMXR - eepromConfig.accelBiasMXR[ZAXIS]) +
    		                                                            abs(upSideDownMXR  - eepromConfig.accelBiasMXR[ZAXIS])));
    ///////////////////////////////////

    cliPrint("Place accelerometer left edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        leftWingDownMXR += mxr9150YAxis();
        delayMicroseconds(1000);
    }

    leftWingDownMXR /= 5000.0;

    cliPrint("Place accelerometer right edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        rightWingDownMXR += mxr9150YAxis();
        delayMicroseconds(1000);
    }

    rightWingDownMXR /= 5000.0;

    eepromConfig.accelBiasMXR[YAXIS] = (float)((leftWingDownMXR + rightWingDownMXR) / 2.0);

    eepromConfig.accelScaleFactorMXR[YAXIS] = (float)((2.0 * 9.8065) / (abs(leftWingDownMXR  - eepromConfig.accelBiasMXR[YAXIS]) +
    		                                                            abs(rightWingDownMXR - eepromConfig.accelBiasMXR[YAXIS])));
    ///////////////////////////////////

    cliPrint("Place accelerometer rear edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseUpMXR += mxr9150XAxis();
        delayMicroseconds(1000);
    }

    noseUpMXR /= 5000.0;

    cliPrint("Place accelerometer front edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseDownMXR += mxr9150XAxis();
        delayMicroseconds(1000);
    }

    noseDownMXR /= 5000.0;

    eepromConfig.accelBiasMXR[XAXIS] = (float)((noseUpMXR + noseDownMXR) / 2.0);

    eepromConfig.accelScaleFactorMXR[XAXIS] = (float)((2.0 * 9.8065) / (abs(noseUpMXR   - eepromConfig.accelBiasMXR[XAXIS]) +
    		                                                            abs(noseDownMXR - eepromConfig.accelBiasMXR[XAXIS])));
    ///////////////////////////////////

    accelCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
