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

#define MPU6000_CONFIG		    	0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define MPU6000_SPI           SPI3

#define MPU6000_CS_GPIO       GPIOB
#define MPU6000_CS_GPIO_CLOCK RCC_AHB1Periph_GPIOB
#define MPU6000_CS_PIN        GPIO_Pin_8

#define DISABLE_MPU6000       GPIO_SetBits(MPU6000_CS_GPIO,   MPU6000_CS_PIN)
#define ENABLE_MPU6000        GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);

#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

///////////////////////////////////////////////////////////////////////////////
// MPU6000 Variables
///////////////////////////////////////////////////////////////////////////////

extern float   accelOneG;

extern float   accelTCBias[3];

extern int32_t accelSum100Hz[3];

extern int32_t accelSum500Hz[3];

extern int32_t accelSummedSamples100Hz[3];

extern int32_t accelSummedSamples500Hz[3];

extern int16andUint8_t rawAccel[3];

///////////////////////////////////////

extern float gyroRTBias[3];

extern float gyroTCBias[3];

extern int32_t gyroSum500Hz[3];

extern int32_t gyroSummedSamples500Hz[3];

extern int16andUint8_t rawGyro[3];

///////////////////////////////////////

extern uint8_t accelCalibrating;

extern uint8_t mpu6000Calibrating;

extern float   mpu6000Temperature;

extern int16andUint8_t rawMPU6000Temperature;

///////////////////////////////////////////////////////////////////////////////
// MPU6000 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMPU6000(void);

///////////////////////////////////////////////////////////////////////////////
// Read MPU6000
///////////////////////////////////////////////////////////////////////////////

void readMPU6000(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000RTData(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000TCBias(void);

///////////////////////////////////////////////////////////////////////////////
