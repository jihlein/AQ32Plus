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
// AltitudeHold Display
///////////////////////////////////////////////////////////////////////////////

void displayAltitude(float pressureAltitude, float altitudeReference, uint8_t altHoldState);

///////////////////////////////////////////////////////////////////////////////
// Artificial Horizon Display
///////////////////////////////////////////////////////////////////////////////

#define AH_DISPLAY_RECT_HEIGHT  9  // Height of rectangle bounding AI.
                                   // Should be odd so that there is an equal space
                                   // above/below the center reticle

extern uint8_t reticleRow;
extern uint8_t ahTopPixel;
extern uint8_t ahBottomPixel;
extern uint8_t ahCenter;

void displayArtificialHorizon(float roll, float pitch, uint8_t flightMode);

///////////////////////////////////////////////////////////////////////////////
// Attitude Display
///////////////////////////////////////////////////////////////////////////////

#define AI_DISPLAY_RECT_HEIGHT  9  // Height of rectangle bounding AI.
                                   // Should be odd so that there is an equal space
                                   // above/below the center reticle

extern uint8_t aiTopPixel;
extern uint8_t aiBottomPixel;
extern uint8_t aiCenter;

void displayAttitude(float roll, float pitch, uint8_t flightMode);

///////////////////////////////////////////////////////////////////////////////
// Heading Display
///////////////////////////////////////////////////////////////////////////////

void displayHeading(float currentHeading);

///////////////////////////////////////////////////////////////////////////////
