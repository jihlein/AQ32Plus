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

__attribute__((__section__(".eeprom"), used)) const int8_t eepromArray[16384];

eepromConfig_t eepromConfig;

uint8_t        execUpCount = 0;

sensors_t      sensors;

heading_t      heading;

gps_t          gps;

homeData_t     homeData;

uint16_t       timerValue;

void           (*openLogPortPrintF)(const char * fmt, ...);

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    ///////////////////////////////////////////////////////////////////////////

    uint32_t currentTime;

	arm_matrix_instance_f32 a;
	arm_matrix_instance_f32 b;
	arm_matrix_instance_f32 x;

    systemReady = false;

    systemInit();

    systemReady = true;

    evrPush(EVR_StartingMain, 0);

    #ifdef _DTIMING

        #define LA1_ENABLE       GPIO_SetBits(GPIOA,   GPIO_Pin_4)
        #define LA1_DISABLE      GPIO_ResetBits(GPIOA, GPIO_Pin_4)
        #define LA4_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_5)
        #define LA4_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_5)
        #define LA2_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_2)
        #define LA2_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_2)
        #define LA3_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_3)
        #define LA3_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_3)

        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,   ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,   ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,   ENABLE);

        GPIO_StructInit(&GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
      //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
      //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOC, &GPIO_InitStructure);

        // PB0_DISABLE;
        LA4_DISABLE;
        LA2_DISABLE;
        LA3_DISABLE;
        LA1_DISABLE;

    #endif

    while (1)
    {
        evrCheck();

        ///////////////////////////////

        if (frame_50Hz)
        {
            #ifdef _DTIMING
                LA2_ENABLE;
            #endif

            frame_50Hz = false;

            currentTime      = micros();
            deltaTime50Hz    = currentTime - previous50HzTime;
            previous50HzTime = currentTime;

            processFlightCommands();

            if (newTemperatureReading && newPressureReading)
            {
                d1Value = d1.value;
                d2Value = d2.value;

                calculateTemperature();
                calculatePressureAltitude();

                newTemperatureReading = false;
                newPressureReading    = false;
            }

            sensors.pressureAlt50Hz = firstOrderFilter(sensors.pressureAlt50Hz, &firstOrderFilters[PRESSURE_ALT_LOWPASS]);

            rssiMeasure();

            updateMax7456(currentTime, 0);

            executionTime50Hz = micros() - currentTime;

            #ifdef _DTIMING
                LA2_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
            #ifdef _DTIMING
                LA4_ENABLE;
            #endif

            frame_10Hz = false;

            currentTime      = micros();
            deltaTime10Hz    = currentTime - previous10HzTime;
            previous10HzTime = currentTime;

            if (newMagData == true)
            {
			    nonRotatedMagData[XAXIS] = (rawMag[XAXIS].value * magScaleFactor[XAXIS]) - eepromConfig.magBias[XAXIS + eepromConfig.externalHMC5883];
			    nonRotatedMagData[YAXIS] = (rawMag[YAXIS].value * magScaleFactor[YAXIS]) - eepromConfig.magBias[YAXIS + eepromConfig.externalHMC5883];
			    nonRotatedMagData[ZAXIS] = (rawMag[ZAXIS].value * magScaleFactor[ZAXIS]) - eepromConfig.magBias[ZAXIS + eepromConfig.externalHMC5883];

			    arm_mat_init_f32(&a, 3, 3, (float *)hmcOrientationMatrix);

			    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedMagData);

			    arm_mat_init_f32(&x, 3, 1,          sensors.mag10Hz);

			    arm_mat_mult_f32(&a, &b, &x);

				newMagData = false;
			    magDataUpdate = true;
            }

            decodeUbloxMsg();

            batMonTick();

            cliCom();

            if (eepromConfig.mavlinkEnabled == true)
            {
				mavlinkSendAttitude();
				mavlinkSendVfrHud();
			}

            executionTime10Hz = micros() - currentTime;

            #ifdef _DTIMING
                LA4_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
            #ifdef _DTIMING
                LA1_ENABLE;
            #endif

            frame_500Hz = false;

            currentTime       = micros();
            deltaTime500Hz    = currentTime - previous500HzTime;
            previous500HzTime = currentTime;

            TIM_Cmd(TIM10, DISABLE);
            timerValue = TIM_GetCounter(TIM10);
            TIM_SetCounter(TIM10, 0);
            TIM_Cmd(TIM10, ENABLE);

            dt500Hz = (float)timerValue * 0.0000005f;  // For integrations in 500 Hz loop

            computeMPU6000TCBias();

       	    nonRotatedAccelData[XAXIS] = ((float)accelSummedSamples500Hz[XAXIS] * 0.5f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[YAXIS] = ((float)accelSummedSamples500Hz[YAXIS] * 0.5f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[ZAXIS] = ((float)accelSummedSamples500Hz[ZAXIS] * 0.5f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedAccelData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.accel500Hz);

		    arm_mat_mult_f32(&a, &b, &x);

            nonRotatedGyroData[ROLL ] = ((float)gyroSummedSamples500Hz[ROLL]  * 0.5f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
            nonRotatedGyroData[PITCH] = ((float)gyroSummedSamples500Hz[PITCH] * 0.5f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            nonRotatedGyroData[YAW  ] = ((float)gyroSummedSamples500Hz[YAW]   * 0.5f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedGyroData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.gyro500Hz);

		    arm_mat_mult_f32(&a, &b, &x);

            MargAHRSupdate(sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                           sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                           sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                           eepromConfig.accelCutoff,
                           magDataUpdate,
                           dt500Hz);

            magDataUpdate = false;

            computeAxisCommands(dt500Hz);
            mixTable();
            writeServos();
            writeMotors();

            executionTime500Hz = micros() - currentTime;

            #ifdef _DTIMING
                LA1_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_100Hz)
        {
            #ifdef _DTIMING
                LA3_ENABLE;
            #endif

            frame_100Hz = false;

            currentTime       = micros();
            deltaTime100Hz    = currentTime - previous100HzTime;
            previous100HzTime = currentTime;

            TIM_Cmd(TIM11, DISABLE);
            timerValue = TIM_GetCounter(TIM11);
            TIM_SetCounter(TIM11, 0);
            TIM_Cmd(TIM11, ENABLE);

            dt100Hz = (float)timerValue * 0.0000005f;  // For integrations in 100 Hz loop

       	    nonRotatedAccelData[XAXIS] = ((float)accelSummedSamples100Hz[XAXIS] * 0.1f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[YAXIS] = ((float)accelSummedSamples100Hz[YAXIS] * 0.1f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[ZAXIS] = ((float)accelSummedSamples100Hz[ZAXIS] * 0.1f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedAccelData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.accel100Hz);

		    arm_mat_mult_f32(&a, &b, &x);

            createRotationMatrix();
            bodyAccelToEarthAccel();
            vertCompFilter(dt100Hz);

            if (true) //(armed == true)
            {
				if ( eepromConfig.activeTelemetry == 1 )
                {
            	    // 500 Hz Accels
					openLogPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
					        			                                            sensors.accel500Hz[YAXIS],
					        			                                            sensors.accel500Hz[ZAXIS]);
                }

                if ( eepromConfig.activeTelemetry == 2 )
                {
            	    // 500 Hz Gyros
            	    openLogPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	            			                       sensors.gyro500Hz[PITCH],
            	            					               sensors.gyro500Hz[YAW  ]);
                }

                if ( eepromConfig.activeTelemetry == 4 )
                {
            	    // 500 Hz Attitudes
            	    openLogPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ],
            	            			                       sensors.attitude500Hz[PITCH],
            	            			                       sensors.attitude500Hz[YAW  ]);
                }

                if ( eepromConfig.activeTelemetry == 8 )
                {
               	    // Vertical Variables
            	    openLogPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %4ld\n", earthAxisAccels[ZAXIS],
            	    		                                                sensors.pressureAlt50Hz,
            	    		                                                hDotEstimate,
            	    		                                                hEstimate,
            	    		                                                ms5611Temperature);
                }

                if ( eepromConfig.activeTelemetry == 16)
                {
               	    // Vertical Variables
            	    openLogPortPrintF("%9.4f, %9.4f, %9.4f, %4ld, %1d, %9.4f, %9.4f\n", verticalVelocityCmd,
            	    		                                                            hDotEstimate,
            	    		                                                            hEstimate,
            	    		                                                            ms5611Temperature,
            	    		                                                            verticalModeState,
            	    		                                                            throttleCmd,
            	    		                                                            eepromConfig.PID[HDOT_PID].iTerm);
                }
		    }

            executionTime100Hz = micros() - currentTime;

            #ifdef _DTIMING
                LA3_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
            frame_5Hz = false;

            currentTime     = micros();
            deltaTime5Hz    = currentTime - previous5HzTime;
            previous5HzTime = currentTime;

            if (gpsValid() == true)
            {

			}

            //if (eepromConfig.mavlinkEnabled == true)
            //{
			//	mavlinkSendGpsRaw();
			//}

			if (batMonVeryLowWarning > 0)
			{
				LED1_TOGGLE;
				batMonVeryLowWarning--;
			}

            if (execUp == true)
                BLUE_LED_TOGGLE;

			executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
            frame_1Hz = false;

            currentTime     = micros();
            deltaTime1Hz    = currentTime - previous1HzTime;
            previous1HzTime = currentTime;

            if (execUp == true)
                GREEN_LED_TOGGLE;

            if (execUp == false)
                execUpCount++;

            if ((execUpCount == 5) && (execUp == false))
            {
				execUp = true;

                pwmEscInit();

                homeData.magHeading = sensors.attitude500Hz[YAW];
			}

            if (batMonLowWarning > 0)
			{
				LED1_TOGGLE;
				batMonLowWarning--;
			}

            if (eepromConfig.mavlinkEnabled == true)
            {
				mavlinkSendHeartbeat();
				mavlinkSendSysStatus();
			}

            executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
