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

///////////////////////////////////////

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
  #define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

///////////////////////////////////////

uint32_t d1Average;

uint32_t d1Sum = 0;

uint16andUint8_t c1, c2, c3, c4,c5, c6;

uint32andUint8_t d1;

uint32andUint8_t d2;

int32_t dT;
int32_t temp;

int64_t offset;
int64_t sensitivity;

int32_t p;

uint8_t pressureAltValid = false;
///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperatureRequestPressure(I2C_TypeDef *I2Cx)
{
    uint8_t data[3];

    i2cRead(I2Cx, MS5611_ADDRESS, 0x00, 3, data);    // Request temperature read

    d2.bytes[2] = data[0];
    d2.bytes[1] = data[1];
    d2.bytes[0] = data[2];

    #if   (OSR ==  256)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x40);  // Request pressure conversion
	#elif (OSR ==  512)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x42);
	#elif (OSR == 1024)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x44);
	#elif (OSR == 2048)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x46);
	#elif (OSR == 4096)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// ReadPressureRequestPressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(I2C_TypeDef *I2Cx)
{
    uint8_t data[3];

    i2cRead(I2Cx, MS5611_ADDRESS, 0x00, 3, data);    // Request pressure read

    d1.bytes[2] = data[0];
    d1.bytes[1] = data[1];
    d1.bytes[0] = data[2];

    #if   (OSR ==  256)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x40);  // Request pressure conversion
	#elif (OSR ==  512)
		i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x42);
	#elif (OSR == 1024)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x44);
	#elif (OSR == 2048)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x46);
	#elif (OSR == 4096)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(I2C_TypeDef *I2Cx)
{
    uint8_t data[3];

    i2cRead(I2Cx, MS5611_ADDRESS, 0x00, 3, data);    // Request pressure read

    d1.bytes[2] = data[0];
    d1.bytes[1] = data[1];
    d1.bytes[0] = data[2];

    #if   (OSR ==  256)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x50);   // Request temperature converison
	#elif (OSR ==  512)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x52);
	#elif (OSR == 1024)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x54);
	#elif (OSR == 2048)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x56);
	#elif (OSR == 4096)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x58);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void)
{
    dT = d2.value - ((int32_t)c5.value << 8);
    temp = 2000 + (int32_t)(((int64_t)dT * (int64_t)c6.value) >> 23);
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void)
{
    int64_t offset2      = 0;
    int64_t sensitivity2 = 0;

    offset      = ((uint32_t)c2.value << 16) + ((c4.value * (int64_t)dT) >> 7);
	sensitivity = ((uint32_t)c1.value << 15) + ((c3.value * (int64_t)dT) >> 8);

	if (temp < 20)
	{
		offset2 = 5 * SQR(temp - 2000) / 2;
		sensitivity2 = 5 * SQR(temp -2000) / 4;

		if (temp < -15)
		{
			offset2 = offset2 + 7 * SQR(temp + 1500);
			sensitivity2 = sensitivity2 + 11 * SQR(temp + 1500) / 2;
		}
	}

	offset = offset - offset2;
	sensitivity = sensitivity - sensitivity2;

	p = (((d1Average * sensitivity) >> 21) - offset) >> 15;

    sensors.pressureAlt10Hz = (44330.0f * (1.0f - pow((float)p / 101325.0f, 0.190295f)));
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(I2C_TypeDef *I2Cx)
{
    uint8_t data[2];

    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x1E);      // Reset Device

    delay(10);

    i2cRead(I2Cx, MS5611_ADDRESS, 0xA2, 2, data);    // Read Calibration Data C1
    c1.bytes[1] = data[0];
    c1.bytes[0] = data[1];

    i2cRead(I2Cx, MS5611_ADDRESS, 0xA4, 2, data);    // Read Calibration Data C2
    c2.bytes[1] = data[0];
    c2.bytes[0] = data[1];

    i2cRead(I2Cx, MS5611_ADDRESS, 0xA6, 2, data);    // Read Calibration Data C3
	c3.bytes[1] = data[0];
    c3.bytes[0] = data[1];

    i2cRead(I2Cx, MS5611_ADDRESS, 0xA8, 2, data);    // Read Calibration Data C4
	c4.bytes[1] = data[0];
    c4.bytes[0] = data[1];

    i2cRead(I2Cx, MS5611_ADDRESS, 0xAA, 2, data);    // Read Calibration Data C5
	c5.bytes[1] = data[0];
    c5.bytes[0] = data[1];

    i2cRead(I2Cx, MS5611_ADDRESS, 0xAC, 2, data);    // Read Calibration Data C6
	c6.bytes[1] = data[0];
    c6.bytes[0] = data[1];

    #if   (OSR ==  256)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x50);  // Request temperature conversion
	#elif (OSR ==  512)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x52);
	#elif (OSR == 1024)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x54);
	#elif (OSR == 2048)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x56);
	#elif (OSR == 4096)
	    i2cWrite(I2Cx, MS5611_ADDRESS, 0xFF, 0x58);
    #endif

    delay(10);

    readTemperatureRequestPressure(I2Cx);
    delay(10);
}

///////////////////////////////////////////////////////////////////////////////
