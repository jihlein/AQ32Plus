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

//#define DEBUG
//#define _DTIMING

#define TELEM_PRINT 1
#define TELEM_LOG   0

///////////////////////////////////////////////////////////////////////////////

__attribute__((__section__(".eeprom"), used)) const int8_t eepromArray[16384];

eepromConfig_t eepromConfig;

uint8_t        execUpCount = 0;

sensors_t      sensors;

heading_t      heading;

uint16_t       timerValue;

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
	///////////////////////////////////////////////////////////////////////////

	uint32_t currentTime;

    #ifdef _DTIMING

        #define PA4_ENABLE       GPIO_SetBits(GPIOA,   GPIO_Pin_4)
        #define PA4_DISABLE      GPIO_ResetBits(GPIOA, GPIO_Pin_4)
        #define PB0_ENABLE       GPIO_SetBits(GPIOB,   GPIO_Pin_0)
        #define PB0_DISABLE      GPIO_ResetBits(GPIOB, GPIO_Pin_0)
        #define PB1_ENABLE       GPIO_SetBits(GPIOB,   GPIO_Pin_1)
        #define PB1_DISABLE      GPIO_ResetBits(GPIOB, GPIO_Pin_1)
        #define PC2_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_2)
        #define PC2_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_2)
        #define PC3_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_3)
        #define PC3_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_3)

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
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
      //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
      //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOC, &GPIO_InitStructure);

	    PB0_DISABLE;
	    PB1_DISABLE;
	    PC2_DISABLE;
    	PC3_DISABLE;
	    PA4_DISABLE;

    #endif

	systemInit();

    systemReady = true;

    while (1)
    {
    	///////////////////////////////

        if (frame_50Hz)
        {
            #ifdef _DTIMING
        	    PC2_ENABLE;
            #endif

        	frame_50Hz = false;

        	currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processFlightCommands();

            if (validTemperature && validPressure)
            {
				calculateTemperature();
        	    calculatePressureAltitude();

        	    validPressure    = false;
        	    validTemperature = false;
			}

			if (eepromConfig.osdEnabled)
			{
				if (eepromConfig.osdDisplayAlt)
				    displayAltitude(sensors.pressureAlt10Hz, 0.0f, DISENGAGED);

				if (eepromConfig.osdDisplayAH)
				    displayArtificialHorizon(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode);

				if (eepromConfig.osdDisplayAtt)
				    displayAttitude(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode);

				if (eepromConfig.osdDisplayHdg)
				    displayHeading(heading.mag);
			}

			executionTime50Hz = micros() - currentTime;

			#ifdef _DTIMING
        	    PC2_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
            #ifdef _DTIMING
        	    PB1_ENABLE;
            #endif

        	frame_10Hz = false;

        	currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			if (newMagData == true)
			{
				sensors.mag10Hz[XAXIS] =   (float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS];
			    sensors.mag10Hz[YAXIS] =   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
			    sensors.mag10Hz[ZAXIS] = -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

			    newMagData = false;
			    magDataUpdate = true;
			}

        	// HJI d1Average = d1Sum / 10;
        	// HJI d1Sum = 0;
        	// HJI calculateTemperature();
        	// HJI calculatePressureAltitude();

        	// HJI pressureAltValid = true;

        	switch (eepromConfig.gpsType)
			{
			    ///////////////////////

			    case NO_GPS:                // No GPS installed
			        break;

			    ///////////////////////

			    case MEDIATEK_3329_BINARY:  // MediaTek 3329 in binary mode
			    	decodeMediaTek3329BinaryMsg();
			    	break;

				///////////////////////

				case MEDIATEK_3329_NMEA:    // MediaTek 3329 in NMEA mode
				    decodeNMEAsentence();
	        	    break;

			    ///////////////////////

			    case UBLOX:                 // UBLOX in binary mode
			    	decodeUbloxMsg();
			    	break;

			    ///////////////////////
			}

        	cliCom();

        	rfCom();

        	///////////////////////////

        	if ( highSpeedTelem6Enabled == true )
			{
			   	// Vertical Variables
				#if (TELEM_PRINT == 1)
				telemetryPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
						                                                                           sensors.accel500Hz[YAXIS],
						                                                                           sensors.accel500Hz[ZAXIS],
						                                                                           sensors.gyro500Hz[ROLL ],
						                                                                           sensors.gyro500Hz[PITCH],
						                                                                           sensors.gyro500Hz[YAW  ],
						                                                                           sensors.mag10Hz[XAXIS],
						                                                                           sensors.mag10Hz[YAXIS],
						                                                                           sensors.mag10Hz[ZAXIS]);
			    #endif

			    #if (TELEM_LOG == 1)
				      logPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
						                                                                           sensors.accel500Hz[YAXIS],
						                                                                           sensors.accel500Hz[ZAXIS],
						                                                                           sensors.gyro500Hz[ROLL ],
						                                                                           sensors.gyro500Hz[PITCH],
						                                                                           sensors.gyro500Hz[YAW  ],
						                                                                           sensors.mag10Hz[XAXIS],
						                                                                           sensors.mag10Hz[YAXIS],
						                                                                           sensors.mag10Hz[ZAXIS]);
			    #endif
		    }

		    ///////////////////////////

            executionTime10Hz = micros() - currentTime;

            #ifdef _DTIMING
        	    PB1_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
            #ifdef _DTIMING
        	    PA4_ENABLE;
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
            /*
            sensorTemp1 = computeMPU6000SensorTemp();
            sensorTemp2 = sensorTemp1 * sensorTemp1;
            sensorTemp3 = sensorTemp2 * sensorTemp1;
            */
            sensors.accel500Hz[XAXIS] =  ((float)accelSummedSamples500Hz[XAXIS] / 2.0f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[YAXIS] = -((float)accelSummedSamples500Hz[YAXIS] / 2.0f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[ZAXIS] = -((float)accelSummedSamples500Hz[ZAXIS] / 2.0f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;
            /*
            sensors.accel500Hz[XAXIS] =  ((float)accelSummedSamples500Hz[XAXIS] / 2.0f  +
                                          eepromConfig.accelBiasP0[XAXIS]               +
                                          eepromConfig.accelBiasP1[XAXIS] * sensorTemp1 +
                                          eepromConfig.accelBiasP2[XAXIS] * sensorTemp2 +
                                          eepromConfig.accelBiasP3[XAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;

			sensors.accel500Hz[YAXIS] = -((float)accelSummedSamples500Hz[YAXIS] / 2.0f  +
			                              eepromConfig.accelBiasP0[YAXIS]               +
			                              eepromConfig.accelBiasP1[YAXIS] * sensorTemp1 +
			                              eepromConfig.accelBiasP2[YAXIS] * sensorTemp2 +
			                              eepromConfig.accelBiasP3[YAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;

			sensors.accel500Hz[ZAXIS] = -((float)accelSummedSamples500Hz[ZAXIS] / 2.0f  +
			                              eepromConfig.accelBiasP0[ZAXIS]               +
			                              eepromConfig.accelBiasP1[ZAXIS] * sensorTemp1 +
			                              eepromConfig.accelBiasP2[ZAXIS] * sensorTemp2 +
			                              eepromConfig.accelBiasP3[ZAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;
            */
            sensors.gyro500Hz[ROLL ] =  ((float)gyroSummedSamples500Hz[ROLL]  / 2.0f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
			sensors.gyro500Hz[PITCH] = -((float)gyroSummedSamples500Hz[PITCH] / 2.0f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            sensors.gyro500Hz[YAW  ] = -((float)gyroSummedSamples500Hz[YAW]   / 2.0f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;
            /*
            sensors.gyro500Hz[ROLL ] =  ((float)gyroSummedSamples500Hz[ROLL ] / 2.0f  +
                                         gyroBiasP0[ROLL ]                            +
                                         eepromConfig.gyroBiasP1[ROLL ] * sensorTemp1 +
                                         eepromConfig.gyroBiasP2[ROLL ] * sensorTemp2 +
                                         eepromConfig.gyroBiasP3[ROLL ] * sensorTemp3 ) * GYRO_SCALE_FACTOR;

			sensors.gyro500Hz[PITCH] = -((float)gyroSummedSamples500Hz[PITCH] / 2.0f  +
			                             gyroBiasP0[PITCH]                            +
			                             eepromConfig.gyroBiasP1[PITCH] * sensorTemp1 +
			                             eepromConfig.gyroBiasP2[PITCH] * sensorTemp2 +
			                             eepromConfig.gyroBiasP3[PITCH] * sensorTemp3 ) * GYRO_SCALE_FACTOR;

            sensors.gyro500Hz[YAW  ] = -((float)gyroSummedSamples500Hz[YAW]   / 2.0f  +
                                         gyroBiasP0[YAW  ]                            +
                                         eepromConfig.gyroBiasP1[YAW  ] * sensorTemp1 +
                                         eepromConfig.gyroBiasP2[YAW  ] * sensorTemp2 +
                                         eepromConfig.gyroBiasP3[YAW  ] * sensorTemp3 ) * GYRO_SCALE_FACTOR;
            */
            MargAHRSupdate( sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                            sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                            sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                            eepromConfig.accelCutoff,
                            magDataUpdate,
                            dt500Hz );

            magDataUpdate = false;

            computeAxisCommands(dt500Hz);
            mixTable();
            writeServos();
            writeMotors();

       	    executionTime500Hz = micros() - currentTime;

       	    #ifdef _DTIMING
        	    PA4_DISABLE;
            #endif
		}

        ///////////////////////////////

        if (frame_100Hz)
        {
            #ifdef _DTIMING
        	    PC3_ENABLE;
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

			sensors.accel100Hz[XAXIS] =  ((float)accelSummedSamples100Hz[XAXIS] / 10.0f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel100Hz[YAXIS] = -((float)accelSummedSamples100Hz[YAXIS] / 10.0f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel100Hz[ZAXIS] = -((float)accelSummedSamples100Hz[ZAXIS] / 10.0f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

        	createRotationMatrix();
        	bodyAccelToEarthAccel();
        	vertCompFilter(dt100Hz);

        	if ( highSpeedTelem1Enabled == true )
            {
            	// 500 Hz Accels
            	#if (TELEM_PRINT == 1)
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
            	        			                     sensors.accel500Hz[YAXIS],
            	        			                     sensors.accel500Hz[ZAXIS]);
                #endif

            	#if (TELEM_LOG == 1)
            	      logPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
            	        			                     sensors.accel500Hz[YAXIS],
            	        			                     sensors.accel500Hz[ZAXIS]);
                #endif
            }

            if ( highSpeedTelem2Enabled == true )
            {
            	// 500 Hz Gyros
            	#if (TELEM_PRINT == 1)
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	        			                     sensors.gyro500Hz[PITCH],
            	        					             sensors.gyro500Hz[YAW  ]);
                #endif

            	#if (TELEM_LOG == 1)
            	      logPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	        			                     sensors.gyro500Hz[PITCH],
            	        					             sensors.gyro500Hz[YAW  ]);
                #endif

            }

            if ( highSpeedTelem3Enabled == true )
            {
            	// Earth Axis Accels
            	#if (TELEM_PRINT == 1)
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", earthAxisAccels[XAXIS],
            	        			                     earthAxisAccels[YAXIS],
            	        			                     earthAxisAccels[ZAXIS]);
                #endif

            	#if (TELEM_LOG == 1)
            	      logPrintF("%9.4f, %9.4f, %9.4f\n", earthAxisAccels[XAXIS],
            	        			                     earthAxisAccels[YAXIS],
            	        			                     earthAxisAccels[ZAXIS]);
                #endif

            }

            if ( highSpeedTelem4Enabled == true )
            {
            	// 500 Hz Attitudes
            	#if (TELEM_PRINT == 1)
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ],
            	        			                     sensors.attitude500Hz[PITCH],
            	        			                     sensors.attitude500Hz[YAW  ]);
                #endif

            	#if (TELEM_LOG == 1)
            	      logPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ],
            	        			                     sensors.attitude500Hz[PITCH],
            	        			                     sensors.attitude500Hz[YAW  ]);
                #endif
            }

            if ( highSpeedTelem5Enabled == true )
            {
               	// Vertical Variables
            	#if (TELEM_PRINT == 1)
            	telemetryPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", earthAxisAccels[ZAXIS],
            			                                        sensors.pressureAlt10Hz,
            			                                        hDotEstimate,
            			                                        hEstimate);
                #endif

                #if (TELEM_LOG == 1)
            	      logPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", earthAxisAccels[ZAXIS],
            			                                        sensors.pressureAlt10Hz,
            			                                        hDotEstimate,
            			                                        hEstimate);
                #endif
            }

            executionTime100Hz = micros() - currentTime;

            #ifdef _DTIMING
        	    PC3_DISABLE;
            #endif
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
            #ifdef _DTIMING
        	    PB0_ENABLE;
            #endif

        	frame_5Hz = false;

        	currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

			if (execUp == true)
			    BLUE_LED_TOGGLE;

        	executionTime5Hz = micros() - currentTime;

        	#ifdef _DTIMING
        	    PB0_DISABLE;
            #endif
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
			    execUp = true;

			executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
