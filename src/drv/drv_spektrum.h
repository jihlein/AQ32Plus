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
// Receiver Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define MAX_SPEKTRUM_FRAMES         2

#define SPEKTRUM_CHANNELS_PER_FRAME 7

///////////////////////////////////////

struct spektrumStateStruct
{
    uint8_t  reSync;
    uint8_t  spektrumTimer;
    uint8_t  sync;
    uint8_t  channelCnt;
    uint8_t  frameCnt;
    uint8_t  highByte;
    uint8_t  secondFrame;
    uint16_t lostFrameCnt;
    uint8_t  rcAvailable;
    uint16_t values[SPEKTRUM_CHANNELS_PER_FRAME * MAX_SPEKTRUM_FRAMES];
};

typedef struct spektrumStateStruct spektrumStateType;

extern spektrumStateType primarySpektrumState;

extern spektrumStateType slaveSpektrumState;

extern int16_t spektrumBuf[SPEKTRUM_CHANNELS_PER_FRAME * MAX_SPEKTRUM_FRAMES];

extern uint8_t maxChannelNum;

extern uint8_t rcActive;

///////////////////////////////////////////////////////////////////////////////
// Spektrum Initialization
///////////////////////////////////////////////////////////////////////////////

void spektrumInit(void);

///////////////////////////////////////////////////////////////////////////////
// Receiver Read
///////////////////////////////////////////////////////////////////////////////

uint16_t spektrumRead(uint8_t channel);

///////////////////////////////////////////////////////////////////////////////
// Check Spektrum Bind
///////////////////////////////////////////////////////////////////////////////

void checkSpektrumBind(void);

///////////////////////////////////////////////////////////////////////////////
