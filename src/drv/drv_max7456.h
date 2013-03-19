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

#define MAX7456_SPI         SPI2

#define MAX7456_CS_GPIO     GPIOA
#define MAX7456_CS_PIN      GPIO_Pin_3

#define NTSC                0
#define PAL                 1

//MAX7456 Registers
#define VM0_REG             0x00
#define VM1_REG             0x01
#define DMM_REG             0x04
#define DMAH_REG            0x05
#define DMAL_REG            0x06
#define DMDI_REG            0x07
#define CMM_REG             0x08
#define CMAH_REG            0x09
#define RB0_REG             0x10
#define CMAL_REG            0x0A
#define CMDI_REG            0x0B
#define STAT_REG            0xA0

#define READ_MAX7456_REG    0x80

//MAX7456 Commands
#define CLEAR_DISPLAY       0x04
#define CLEAR_DISPLAY_VERT  0x06
#define END_STRING          0xFF
#define WRITE_NVR           0xA0

#define WHITE_LEVEL_80      0x03
#define WHITE_LEVEL_90      0x02
#define WHITE_LEVEL_100     0x01
#define WHITE_LEVEL_120     0x00

#define MAX_FONT_ROM        0xFF
#define STATUS_REG_NVR_BUSY 0x20
#define NVM_RAM_SIZE        0x36

///////////////////////////////////////////////////////////////////////////////
// MAX7456 Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t  osdDisabled;

extern uint16_t maxScreenSize;
extern uint16_t maxScreenRows;
extern uint8_t  enableDisplay;
extern uint8_t  enableDisplayVert;
extern uint8_t  max7456Reset;
extern uint8_t  disableDisplay;

//////////////////////////////////////////////////////////////////////////////
// Initialize MAX7456
///////////////////////////////////////////////////////////////////////////////

void initMax7456();

//////////////////////////////////////////////////////////////////////////////
// Reset MAX7456
///////////////////////////////////////////////////////////////////////////////

void resetMax7456(void);

///////////////////////////////////////////////////////////////////////////////
// Show MAX7456 Font
///////////////////////////////////////////////////////////////////////////////

void showMax7456Font(void);

///////////////////////////////////////////////////////////////////////////////
// Download MAX7456 Font Data
///////////////////////////////////////////////////////////////////////////////

void downloadMax7456Font(void);

///////////////////////////////////////////////////////////////////////////////
// Write Characters
///////////////////////////////////////////////////////////////////////////////

void writeMax7456Chars( const char* buf, uint8_t len, uint8_t flags, uint8_t y, uint8_t x);

///////////////////////////////////////////////////////////////////////////////
