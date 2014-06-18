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

float mpuOrientationMatrix[9];
float hmcOrientationMatrix[9];

///////////////////////////////////////////////////////////////////////////////
// Orient Sensors
///////////////////////////////////////////////////////////////////////////////

void orientSensors(void)
{
    switch (eepromConfig.sensorOrientation)
    {
        case 1: // Normal, no rotation
            mpuOrientationMatrix[0] =  1.0f;
            mpuOrientationMatrix[1] =  0.0f;
            mpuOrientationMatrix[2] =  0.0f;
            mpuOrientationMatrix[3] =  0.0f;
            mpuOrientationMatrix[4] = -1.0f;
            mpuOrientationMatrix[5] =  0.0f;
            mpuOrientationMatrix[6] =  0.0f;
            mpuOrientationMatrix[7] =  0.0f;
            mpuOrientationMatrix[8] = -1.0f;

            hmcOrientationMatrix[0] =  1.0f;
            hmcOrientationMatrix[1] =  0.0f;
            hmcOrientationMatrix[2] =  0.0f;
            hmcOrientationMatrix[3] =  0.0f;
            hmcOrientationMatrix[4] =  1.0f;
            hmcOrientationMatrix[5] =  0.0f;
            hmcOrientationMatrix[6] =  0.0f;
            hmcOrientationMatrix[7] =  0.0f;
            hmcOrientationMatrix[8] = -1.0f;

            break;

        case 2: // 90 degree rotation
            mpuOrientationMatrix[0] =  0.0f;
            mpuOrientationMatrix[1] =  1.0f;
            mpuOrientationMatrix[2] =  0.0f;
            mpuOrientationMatrix[3] =  1.0f;
            mpuOrientationMatrix[4] =  0.0f;
            mpuOrientationMatrix[5] =  0.0f;
            mpuOrientationMatrix[6] =  0.0f;
            mpuOrientationMatrix[7] =  0.0f;
            mpuOrientationMatrix[8] = -1.0f;

            hmcOrientationMatrix[0] =  0.0f;
            hmcOrientationMatrix[1] = -1.0f;
            hmcOrientationMatrix[2] =  0.0f;
            hmcOrientationMatrix[3] =  1.0f;
            hmcOrientationMatrix[4] =  0.0f;
            hmcOrientationMatrix[5] =  0.0f;
            hmcOrientationMatrix[6] =  0.0f;
            hmcOrientationMatrix[7] =  0.0f;
            hmcOrientationMatrix[8] = -1.0f;

            break;

        case 3: // 180 degree rotation
            mpuOrientationMatrix[0] = -1.0f;
            mpuOrientationMatrix[1] =  0.0f;
            mpuOrientationMatrix[2] =  0.0f;
            mpuOrientationMatrix[3] =  0.0f;
            mpuOrientationMatrix[4] =  1.0f;
            mpuOrientationMatrix[5] =  0.0f;
            mpuOrientationMatrix[6] =  0.0f;
            mpuOrientationMatrix[7] =  0.0f;
            mpuOrientationMatrix[8] = -1.0f;

            hmcOrientationMatrix[0] = -1.0f;
            hmcOrientationMatrix[1] =  0.0f;
            hmcOrientationMatrix[2] =  0.0f;
            hmcOrientationMatrix[3] =  0.0f;
            hmcOrientationMatrix[4] = -1.0f;
            hmcOrientationMatrix[5] =  0.0f;
            hmcOrientationMatrix[6] =  0.0f;
            hmcOrientationMatrix[7] =  0.0f;
            hmcOrientationMatrix[8] = -1.0f;

            break;

        case 4: // -90 degree rotation
            mpuOrientationMatrix[0] =  0.0f;
            mpuOrientationMatrix[1] = -1.0f;
            mpuOrientationMatrix[2] =  0.0f;
            mpuOrientationMatrix[3] = -1.0f;
            mpuOrientationMatrix[4] =  0.0f;
            mpuOrientationMatrix[5] =  0.0f;
            mpuOrientationMatrix[6] =  0.0f;
            mpuOrientationMatrix[7] =  0.0f;
            mpuOrientationMatrix[8] = -1.0f;

            hmcOrientationMatrix[0] =  0.0f;
            hmcOrientationMatrix[1] =  1.0f;
            hmcOrientationMatrix[2] =  0.0f;
            hmcOrientationMatrix[3] = -1.0f;
            hmcOrientationMatrix[4] =  0.0f;
            hmcOrientationMatrix[5] =  0.0f;
            hmcOrientationMatrix[6] =  0.0f;
            hmcOrientationMatrix[7] =  0.0f;
            hmcOrientationMatrix[8] = -1.0f;

            break;

        default: // Normal, no rotation
            mpuOrientationMatrix[0] =  1.0f;
            mpuOrientationMatrix[1] =  0.0f;
            mpuOrientationMatrix[2] =  0.0f;
            mpuOrientationMatrix[3] =  0.0f;
            mpuOrientationMatrix[4] = -1.0f;
            mpuOrientationMatrix[5] =  0.0f;
            mpuOrientationMatrix[6] =  0.0f;
            mpuOrientationMatrix[7] =  0.0f;
            mpuOrientationMatrix[8] = -1.0f;

            hmcOrientationMatrix[0] =  1.0f;
            hmcOrientationMatrix[1] =  0.0f;
            hmcOrientationMatrix[2] =  0.0f;
            hmcOrientationMatrix[3] =  0.0f;
            hmcOrientationMatrix[4] =  1.0f;
            hmcOrientationMatrix[5] =  0.0f;
            hmcOrientationMatrix[6] =  0.0f;
            hmcOrientationMatrix[7] =  0.0f;
            hmcOrientationMatrix[8] = -1.0f;

            break;
    }
}

///////////////////////////////////////////////////////////////////////////////

