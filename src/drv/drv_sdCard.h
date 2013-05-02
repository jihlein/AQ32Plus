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

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define SDCARD_SPI           SPI1

#define SDCARD_CS_GPIO       GPIOE
#define SDCARD_CS_GPIO_CLOCK RCC_AHB1Periph_GPIOE
#define SDCARD_CS_PIN        GPIO_Pin_10

#define DISABLE_SDCARD       GPIO_SetBits(SDCARD_CS_GPIO,   SDCARD_CS_PIN)
#define ENABLE_SDCARD        GPIO_ResetBits(SDCARD_CS_GPIO, SDCARD_CS_PIN)

///////////////////////////////////////////////////////////////////////////////
// SD Card Variables
///////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Initialize SD Card
///////////////////////////////////////////////////////////////////////////////

void initSDcard();

//////////////////////////////////////////////////////////////////////////////
