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

int16_t mpuOrientationMatrix[4];
int16_t hmcOrientationMatrix[4];

///////////////////////////////////////////////////////////////////////////////
// Orient Sensors
///////////////////////////////////////////////////////////////////////////////

void orientSensors(void)
{
    switch (eepromConfig.sensorOrientation)
    {
        case 1: // Normal, no rotation
            mpuOrientationMatrix[0] =  1;
            mpuOrientationMatrix[1] =  0;
            mpuOrientationMatrix[2] =  0;
            mpuOrientationMatrix[3] =  1;

            hmcOrientationMatrix[0] =  1;
            hmcOrientationMatrix[1] =  0;
            hmcOrientationMatrix[2] =  0;
            hmcOrientationMatrix[3] =  1;

            break;

        case 2: // 90 degree rotation
            mpuOrientationMatrix[0] =  0;
            mpuOrientationMatrix[1] = -1;
            mpuOrientationMatrix[2] =  1;
            mpuOrientationMatrix[3] =  0;

            hmcOrientationMatrix[0] =  0;
            hmcOrientationMatrix[1] =  1;
            hmcOrientationMatrix[2] = -1;
            hmcOrientationMatrix[3] =  0;

            break;

        case 3: // 180 degree rotation
            mpuOrientationMatrix[0] = -1;
            mpuOrientationMatrix[1] =  0;
            mpuOrientationMatrix[2] =  0;
            mpuOrientationMatrix[3] = -1;

            hmcOrientationMatrix[0] = -1;
            hmcOrientationMatrix[1] =  0;
            hmcOrientationMatrix[2] =  0;
            hmcOrientationMatrix[3] = -1;

            break;

        case 4: // -90 degree rotation
            mpuOrientationMatrix[0] =  0;
            mpuOrientationMatrix[1] =  1;
            mpuOrientationMatrix[2] = -1;
            mpuOrientationMatrix[3] =  0;

            hmcOrientationMatrix[0] =  0;
            hmcOrientationMatrix[1] = -1;
            hmcOrientationMatrix[2] =  1;
            hmcOrientationMatrix[3] =  0;

            break;

        default: // Normal, no rotation
            mpuOrientationMatrix[0] =  1;
            mpuOrientationMatrix[1] =  0;
            mpuOrientationMatrix[2] =  0;
            mpuOrientationMatrix[3] =  1;

            hmcOrientationMatrix[0] =  1;
            hmcOrientationMatrix[1] =  0;
            hmcOrientationMatrix[2] =  0;
            hmcOrientationMatrix[3] =  1;

            break;
    }
}

///////////////////////////////////////////////////////////////////////////////

