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

#define SD_CARD_SPI           SPI1

#define SD_CARD_CS_GPIO       GPIOE
#define SD_CARD_CS_PIN        GPIO_Pin_10

#define DISABLE_SD_CARD       GPIO_SetBits(SD_CARD_CS_GPIO,   SD_CARD_CS_PIN)
#define ENABLE_SD_CARD        GPIO_ResetBits(SD_CARD_CS_GPIO, SD_CARD_CS_PIN)

///////////////////////////////////////

#define CT_UNKNOWN  0x00             // Unknown

// From diskio.h:

#define CT_MMC		0x01		     // MMC ver 3
#define CT_SD1		0x02		     // SD  ver 1
#define CT_SD2		0x04		     // SD  ver 2
#define CT_SDC		(CT_SD1|CT_SD2)	 // SD
#define CT_BLOCK	0x08		     // Block addressing

///////////////////////////////////////////////////////////////////////////////
// SD Card Initialization
///////////////////////////////////////////////////////////////////////////////

uint8_t initSDCard(void);

///////////////////////////////////////////////////////////////////////////////
// SD Card Count down timers
///////////////////////////////////////////////////////////////////////////////

void sdCardCountDown(void);

///////////////////////////////////////////////////////////////////////////////
