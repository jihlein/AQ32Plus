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
    float noseUpMPU        = 0.0f;
    float noseDownMPU      = 0.0f;
    float leftWingDownMPU  = 0.0f;
    float rightWingDownMPU = 0.0f;
    float upSideDownMPU    = 0.0f;
    float rightSideUpMPU   = 0.0f;

    float noseUpMXR        = 0.0f;
	float noseDownMXR      = 0.0f;
	float leftWingDownMXR  = 0.0f;
	float rightWingDownMXR = 0.0f;
	float upSideDownMXR    = 0.0f;
    float rightSideUpMXR   = 0.0f;

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
        rightSideUpMXR += mxr9150Zaxis();;
        delayMicroseconds(1000);
    }

    rightSideUpMXR /= 5000.0f;

    cliPrint("Place accelerometer up side down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        upSideDownMXR += mxr9150Zaxis();
        delayMicroseconds(1000);
    }

    upSideDownMXR /= 5000.0f;

    eepromConfig.accelBiasMXR[ZAXIS] = (rightSideUpMXR + upSideDownMXR) / 2.0f;

    eepromConfig.accelScaleFactorMXR[ZAXIS] = (2.0f * 9.8065f) / (fabs(rightSideUpMXR) + fabs(upSideDownMXR));

    ///////////////////////////////////

    cliPrint("Place accelerometer left edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        leftWingDownMXR += mxr9150Yaxis();
        delayMicroseconds(1000);
    }

    leftWingDownMXR /= 5000.0f;

    cliPrint("Place accelerometer right edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        rightWingDownMXR += mxr9150Yaxis();
        delayMicroseconds(1000);
    }

    rightWingDownMXR /= 5000.0f;

    eepromConfig.accelBiasMXR[YAXIS] = (leftWingDownMXR + rightWingDownMXR) / 2.0f;

    eepromConfig.accelScaleFactorMXR[YAXIS] = (2.0f * 9.8065f) / (fabs(leftWingDownMXR) + fabs(rightWingDownMXR));

    ///////////////////////////////////

    cliPrint("Place accelerometer rear edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseUpMXR += mxr9150Xaxis();
        delayMicroseconds(1000);
    }

    noseUpMXR /= 5000.0f;

    cliPrint("Place accelerometer front edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseDownMXR += mxr9150Xaxis();
        delayMicroseconds(1000);
    }

    noseDownMXR /= 5000.0f;

    eepromConfig.accelBiasMXR[XAXIS] = (noseUpMXR + noseDownMXR) / 2.0f;

    eepromConfig.accelScaleFactorMXR[XAXIS] = (2.0f * 9.8065f) / (fabs(noseUpMXR) + fabs(noseDownMXR));

    accelCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
