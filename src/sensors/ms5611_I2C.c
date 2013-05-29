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

float c1fp, c2fp, c3fp, c4fp, c5fp, c6fp;

volatile uint32andUint8_t d1;  // syncAccess

uint32_t d1Value;              // syncAccess

volatile uint32andUint8_t d2;  // syncAccess

uint32_t d2Value;              // syncAccess

float dT;
float ms5611Temperature;

volatile uint8_t newPressureReading = false;     // syncAccess

volatile uint8_t newTemperatureReading = false;  // syncAccess
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
    dT                = (float)d2Value - c5fp;
    ms5611Temperature = 20.0f + (dT * c6fp);
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void)
{
    float offset, p, sensitivity;

    offset      = c2fp + c4fp * dT;
    sensitivity = c1fp + c3fp * dT;

    if (ms5611Temperature < 20.0)
    {
        offset      -= SQR(ms5611Temperature - 20.0f) * (10000.0f * 5.0f / 2.0f) * ((1.0f / (float)(1 << 15)) / 100.0f);
        sensitivity -= SQR(ms5611Temperature - 20.0f) * (10000.0f * 5.0f / 4.0f) *  (1.0f / (float)(1 << 21)) * ((1.0f / (float)(1 << 15)) / 100.0f);

        if (ms5611Temperature < -15.0)
        {
            offset      -= SQR(ms5611Temperature + 15.0f) * (10000.0f * 7.0f) * ((1.0f / (float)(1 << 15)) / 100.0f);
            sensitivity -= SQR(ms5611Temperature + 15.0f) * (10000.0f * 11.0f / 2.0f) * (1.0f / (float)(1 << 21)) * ((1.0f / (float)(1 << 15)) / 100.0f);
        }
    }

    p = (float)d1Value * sensitivity - offset; // mbar

    sensors.pressureAlt50Hz = 44330.0f * (1.0 - pow((p / 1013.25f), 0.190295f));
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(I2C_TypeDef *I2Cx)
{
    uint16andUint8_t c1, c2, c3, c4, c5, c6;

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

    c1fp = ((float)c1.value * (float)(1 << 15)) * (1.0f / (float)(1 << 21)) * (1.0f / (float)(1 << 15) / 100.0f);

    c2fp = ((float)c2.value * (float)(1 << 16)) * (1.0f / (float)(1 << 15)) / 100.0f;

    c3fp = ((float)c3.value / (float)(1 << 8))  * (1.0f / (float)(1 << 21)) * (1.0f / (float)(1 << 15) / 100.0f);

    c4fp = ((float)c4.value / (float)(1 << 7))  * (1.0f / (float)(1 << 15)) / 100.0f;

    c5fp = ((float)c5.value * (float)(1 << 8));

    c6fp = ((float)c6.value / (float)(1 << 23)) / 100.0f;

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

    readPressureRequestTemperature(I2Cx);
    delay(10);

    d1Value = d1.value;
    d2Value = d2.value;

    calculateTemperature();
    calculatePressureAltitude();
}

///////////////////////////////////////////////////////////////////////////////
