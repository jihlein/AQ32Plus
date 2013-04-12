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
// Constrain
///////////////////////////////////////////////////////////////////////////////

float constrain(float input, float minValue, float maxValue);

///////////////////////////////////////////////////////////////////////////////
// _sbrk
///////////////////////////////////////////////////////////////////////////////

/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */

caddr_t _sbrk(int incr);

///////////////////////////////////////////////////////////////////////////////
//  Least Squares Fit a Sphere to 3D Data
////////////////////////////////////////////////////////////////////////////////

// Least squares fit a sphere to 3D data, ImaginaryZ's blog,
// Miscellaneous banter, Useful mathematics, game programming
// tools and the occasional kink or two.
// 22 April 2011.
// http: imaginaryz.blogspot.com.au/2011/04/least-squares-fit-sphere-to-3d-data.html

// Substantially rewritten for UAVXArm by Prof. G.K. Egan (C) 2012.

// Incorporated into AQ32Plus by J. Ihlein (C) 2012.

uint16_t sphereFit(float    d[][3],
                   uint16_t N,
                   uint16_t MaxIterations,
                   float    Err,
		           uint16_t Population[][3],
		           float    SphereOrigin[],
		           float    * SphereRadius);

///////////////////////////////////////////////////////////////////////////////
//  Standard Radian Format Limiter
///////////////////////////////////////////////////////////////////////////////

float standardRadianFormat(float angle);

///////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////

float stringToFloat(const char *p);

///////////////////////////////////////////////////////////////////////////////
