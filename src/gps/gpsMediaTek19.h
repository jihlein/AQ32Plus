/*
  February 2013

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
// GPS Variables
///////////////////////////////////////////////////////////////////////////////

enum mtk19State { MTK19_WAIT_SYNC1, MTK19_WAIT_SYNC2, MTK19_GET_LEN, MTK19_GET_DATA, MTK19_GET_CKA, MTK19_GET_CKB } mtk19ProcessDataState;

///////////////////////////////////////////////////////////////////////////////
// Decode MediaTek 3329 Binary Message (DIY Drones Binary Protocol)
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeMediaTek3329BinaryMsg(void);

///////////////////////////////////////////////////////////////////////////////
