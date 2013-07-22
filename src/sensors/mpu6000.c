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
// MPU6000 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Registers

#define MPU6000_SMPLRT_DIV	    	0x19
#define MPU6000_GYRO_CONFIG	    	0x1B
#define MPU6000_ACCEL_CONFIG  		0x1C
#define MPU6000_FIFO_EN		    	0x23
#define MPU6000_INT_PIN_CFG	    	0x37
#define MPU6000_INT_ENABLE	    	0x38
#define MPU6000_INT_STATUS	    	0x3A
#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48
#define MPU6000_USER_CTRL	    	0x6A
#define MPU6000_PWR_MGMT_1	    	0x6B
#define MPU6000_PWR_MGMT_2	    	0x6C
#define MPU6000_FIFO_COUNTH	    	0x72
#define MPU6000_FIFO_COUNTL	    	0x73
#define MPU6000_FIFO_R_W		   	0x74
#define MPU6000_WHOAMI		    	0x75

// Bits

#define BIT_SLEEP				    0x40
#define BIT_H_RESET				    0x80
#define BITS_CLKSEL				    0x07
#define MPU_CLK_SEL_PLLGYROX	    0x01
#define MPU_CLK_SEL_PLLGYROZ	    0x03
#define MPU_EXT_SYNC_GYROX		    0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN			    0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA		    0x01

///////////////////////////////////////

float accelOneG = 9.8065;

int32_t accelSum100Hz[3] = { 0, 0, 0 };

int32_t accelSum500Hz[3] = { 0, 0, 0 };

int32_t accelSummedSamples100Hz[3];

int32_t accelSummedSamples500Hz[3];

float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };

int16andUint8_t rawAccel[3];

///////////////////////////////////////

float gyroRTBias[3];

int32_t gyroSum500Hz[3] = { 0, 0, 0 };

int32_t gyroSummedSamples500Hz[3];

float gyroTCBias[3];

int16andUint8_t rawGyro[3];

///////////////////////////////////////

uint8_t accelCalibrating = false;

uint8_t mpu6000Calibrating = false;

float   mpu6000Temperature;

int16andUint8_t rawMPU6000Temperature;

///////////////////////////////////////////////////////////////////////////////
// MPU6000 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMPU6000(void)
{
    ///////////////////////////////////

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_PWR_MGMT_1);          // Device Reset
    spiTransfer(MPU6000_SPI, BIT_H_RESET);
    DISABLE_MPU6000;

    delay(150);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_PWR_MGMT_1);          // Clock Source PPL with Z axis gyro reference
    spiTransfer(MPU6000_SPI, MPU_CLK_SEL_PLLGYROZ);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_USER_CTRL);           // Disable Primary I2C Interface
    spiTransfer(MPU6000_SPI, BIT_I2C_IF_DIS);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_PWR_MGMT_2);
    spiTransfer(MPU6000_SPI, 0x00);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_SMPLRT_DIV);          // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    spiTransfer(MPU6000_SPI, 0x00);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_CONFIG);              // Accel and Gyro DLPF Setting
    spiTransfer(MPU6000_SPI, eepromConfig.dlpfSetting);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_ACCEL_CONFIG);        // Accel +/- 4 G Full Scale
    spiTransfer(MPU6000_SPI, BITS_FS_4G);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransfer(MPU6000_SPI, MPU6000_GYRO_CONFIG);         // Gyro +/- 1000 DPS Full Scale
    spiTransfer(MPU6000_SPI, BITS_FS_1000DPS);
    DISABLE_MPU6000;

    ///////////////////////////////////

    setSPIdivisor(MPU6000_SPI, 2);                         // 21 MHz SPI clock (within 20 +/- 10%)

    ///////////////////////////////////

    delay(100);

    computeMPU6000RTData();
}

///////////////////////////////////////////////////////////////////////////////
// Read MPU6000
///////////////////////////////////////////////////////////////////////////////

void readMPU6000(void)
{
    ENABLE_MPU6000;

                                     spiTransfer(MPU6000_SPI, MPU6000_ACCEL_XOUT_H | 0x80);

    rawAccel[XAXIS].bytes[1]       = spiTransfer(MPU6000_SPI, 0x00);
    rawAccel[XAXIS].bytes[0]       = spiTransfer(MPU6000_SPI, 0x00);
    rawAccel[YAXIS].bytes[1]       = spiTransfer(MPU6000_SPI, 0x00);
    rawAccel[YAXIS].bytes[0]       = spiTransfer(MPU6000_SPI, 0x00);
    rawAccel[ZAXIS].bytes[1]       = spiTransfer(MPU6000_SPI, 0x00);
    rawAccel[ZAXIS].bytes[0]       = spiTransfer(MPU6000_SPI, 0x00);

    rawMPU6000Temperature.bytes[1] = spiTransfer(MPU6000_SPI, 0x00);
    rawMPU6000Temperature.bytes[0] = spiTransfer(MPU6000_SPI, 0x00);

    rawGyro[ROLL ].bytes[1]        = spiTransfer(MPU6000_SPI, 0x00);
    rawGyro[ROLL ].bytes[0]        = spiTransfer(MPU6000_SPI, 0x00);
    rawGyro[PITCH].bytes[1]        = spiTransfer(MPU6000_SPI, 0x00);
    rawGyro[PITCH].bytes[0]        = spiTransfer(MPU6000_SPI, 0x00);
    rawGyro[YAW  ].bytes[1]        = spiTransfer(MPU6000_SPI, 0x00);
    rawGyro[YAW  ].bytes[0]        = spiTransfer(MPU6000_SPI, 0x00);

    DISABLE_MPU6000;
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000RTData(void)
{
    uint8_t  axis;
    uint16_t samples;

    double accelSum[3]    = { 0.0f, 0.0f, 0.0f };
    double gyroSum[3]     = { 0.0f, 0.0f, 0.0f };
    double accelSumMXR[3] = { 0.0f, 0.0f, 0.0f };

    mpu6000Calibrating = true;

    for (samples = 0; samples < 5000; samples++)
    {
        readMPU6000();

        computeMPU6000TCBias();

        accelSum[XAXIS] += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
        accelSum[YAXIS] += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
        accelSum[ZAXIS] += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];

        gyroSum[ROLL ]  += (float)rawGyro[ROLL ].value  - gyroTCBias[ROLL ];
        gyroSum[PITCH]  += (float)rawGyro[PITCH].value  - gyroTCBias[PITCH];
        gyroSum[YAW  ]  += (float)rawGyro[YAW  ].value  - gyroTCBias[YAW  ];

        accelSumMXR[XAXIS] += mxr9150XAxis();
        accelSumMXR[YAXIS] += mxr9150YAxis();
        accelSumMXR[ZAXIS] += mxr9150ZAxis();

        delayMicroseconds(1000);
    }

    for (axis = 0; axis < 3; axis++)
    {
        accelSum[axis]   = accelSum[axis] / 5000.0f * ACCEL_SCALE_FACTOR;
        gyroRTBias[axis] = gyroSum[axis]  / 5000.0f;

        accelSumMXR[axis] = (accelSumMXR[axis] / 5000.0f - eepromConfig.accelBiasMXR[axis]) * eepromConfig.accelScaleFactorMXR[axis];
    }

    #if defined(MPU_ACCEL)
        accelOneG = sqrt(SQR(accelSum[XAXIS]) + SQR(accelSum[YAXIS]) + SQR(accelSum[ZAXIS]));
    #endif

    #if defined(MXR_ACCEL)
        accelOneG = sqrt(SQR(accelSumMXR[XAXIS]) + SQR(accelSumMXR[YAXIS]) + SQR(accelSumMXR[ZAXIS]));
    #endif

    mpu6000Calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000TCBias(void)
{
    mpu6000Temperature = (float) (rawMPU6000Temperature.value) / 340.0f + 35.0f;

    accelTCBias[XAXIS] = eepromConfig.accelTCBiasSlope[XAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[XAXIS];
    accelTCBias[YAXIS] = eepromConfig.accelTCBiasSlope[YAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[YAXIS];
    accelTCBias[ZAXIS] = eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[ZAXIS];

    gyroTCBias[ROLL ]  = eepromConfig.gyroTCBiasSlope[ROLL ]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[ROLL ];
    gyroTCBias[PITCH]  = eepromConfig.gyroTCBiasSlope[PITCH]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[PITCH];
    gyroTCBias[YAW  ]  = eepromConfig.gyroTCBiasSlope[YAW  ]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[YAW  ];
}

///////////////////////////////////////////////////////////////////////////////

