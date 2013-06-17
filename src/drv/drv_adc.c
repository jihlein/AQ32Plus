/*
  March 2013

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
//  ADC Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float accelSum100HzMXR[3] = { 0, 0, 0 };

float accelSum500HzMXR[3] = { 0, 0, 0 };

float accelSummedSamples100HzMXR[3];

float accelSummedSamples500HzMXR[3];

///////////////////////////////////////

#define MXR9150_XAXIS_CONVERTED_VALUE  0
#define MXR9150_YAXIS_CONVERTED_VALUE  1
#define MXR9150_ZAXIS_CONVERTED_VALUE  2

///////////////////////////////////////

#define ADC4_PIN               GPIO_Pin_5
#define ADC4_GPIO              GPIOC
#define ADC4_CHANNEL           ADC_Channel_15

#define ADC5_PIN               GPIO_Pin_2
#define ADC5_GPIO              GPIOC
#define ADC5_CHANNEL           ADC_Channel_12

#define ADC6_PIN               GPIO_Pin_3
#define ADC6_GPIO              GPIOC
#define ADC6_CHANNEL           ADC_Channel_13

#define VBATT_PIN              GPIO_Pin_0
#define VBATT_GPIO             GPIOC
#define VBATT_CHANNEL          ADC_Channel_10

///////////////////////////////////////

#define ADC1_PIN               GPIO_Pin_0
#define ADC1_GPIO              GPIOB
#define MXR9150_XAXIS_CHANNEL  ADC_Channel_8

#define ADC2_PIN               GPIO_Pin_4
#define ADC2_GPIO              GPIOC
#define MXR9150_YAXIS_CHANNEL  ADC_Channel_14

#define ADC3_PIN               GPIO_Pin_1
#define ADC3_GPIO              GPIOB
#define MXR9150_ZAXIS_CHANNEL  ADC_Channel_9

///////////////////////////////////////

uint16_t adc1ConvertedValues[16] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t adc2ConvertedValues[15] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

///////////////////////////////////////

#define VOLTS_PER_BIT   (3.3f / 4096.0f)

///////////////////////////////////////////////////////////////////////////////
//  ADC Initialization
///////////////////////////////////////////////////////////////////////////////

void adcInit(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_StructInit(&ADC_InitStructure);
    DMA_StructInit(&DMA_InitStructure);
    GPIO_StructInit(&GPIO_InitStructure);

    ///////////////////////////////////

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,  ENABLE);

    ///////////////////////////////////

    DMA_InitStructure.DMA_Channel            = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)adc1ConvertedValues;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = 16;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream4, ENABLE);

    ///////////////////////////////////

    DMA_InitStructure.DMA_Channel            = DMA_Channel_1;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)adc2ConvertedValues;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = 15;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream2, ENABLE);

    ///////////////////////////////////

    GPIO_InitStructure.GPIO_Pin   = ADC1_PIN | ADC3_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Seed_2MHz;
  //GPIO_InitStructrue.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = ADC2_PIN | ADC4_PIN | ADC5_PIN | ADC6_PIN| VBATT_PIN;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Seed_2MHz;
  //GPIO_InitStructrue.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ///////////////////////////////////

  //ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;            // PCLK2 = 42 MHz, ADCCLK = 10.5 MHz
  //ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
  //ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

    ADC_CommonInit(&ADC_CommonInitStructure);

    ///////////////////////////////////

  //ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;
  //ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  //ADC_InitStructure.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1;
  //ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 16;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_InitStructure.ADC_NbrOfConversion      = 15;

    ADC_Init(ADC2, &ADC_InitStructure);

    ///////////////////////////////////

    ADC_RegularChannelConfig(ADC1, ADC4_CHANNEL,  1,  ADC_SampleTime_480Cycles);   // Tconv = (480 + 12) / 10.5 MHz = 46.86 uSec
    ADC_RegularChannelConfig(ADC1, ADC5_CHANNEL,  2,  ADC_SampleTime_480Cycles);   // 16 Conversions will take 749.71 uSec
    ADC_RegularChannelConfig(ADC1, ADC6_CHANNEL,  3,  ADC_SampleTime_480Cycles);   // 16 Conversions will update at 1333.84 Hz
    ADC_RegularChannelConfig(ADC1, VBATT_CHANNEL, 4,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC4_CHANNEL,  5,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC5_CHANNEL,  6,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC6_CHANNEL,  7,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, VBATT_CHANNEL, 8,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC4_CHANNEL,  9,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC5_CHANNEL,  10, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC6_CHANNEL,  11, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, VBATT_CHANNEL, 12, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC4_CHANNEL,  13, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC5_CHANNEL,  14, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC6_CHANNEL,  15, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, VBATT_CHANNEL, 16, ADC_SampleTime_480Cycles);

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    ADC_SoftwareStartConv(ADC1);

    ///////////////////////////////////

    ADC_RegularChannelConfig(ADC2, MXR9150_XAXIS_CHANNEL, 1,  ADC_SampleTime_480Cycles);   // Tconv = (480 + 12) / 10.5 MHz = 46.86 uSec
    ADC_RegularChannelConfig(ADC2, MXR9150_YAXIS_CHANNEL, 2,  ADC_SampleTime_480Cycles);   // 16 Conversions will take 749.71 uSec
    ADC_RegularChannelConfig(ADC2, MXR9150_ZAXIS_CHANNEL, 3,  ADC_SampleTime_480Cycles);   // 16 Conversions will update at 1333.84 Hz
    ADC_RegularChannelConfig(ADC2, MXR9150_XAXIS_CHANNEL, 4,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_YAXIS_CHANNEL, 5,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_ZAXIS_CHANNEL, 6,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_XAXIS_CHANNEL, 7,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_YAXIS_CHANNEL, 8,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_ZAXIS_CHANNEL, 9,  ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_XAXIS_CHANNEL, 10, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_YAXIS_CHANNEL, 11, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_ZAXIS_CHANNEL, 12, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_XAXIS_CHANNEL, 13, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_YAXIS_CHANNEL, 14, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC2, MXR9150_ZAXIS_CHANNEL, 15, ADC_SampleTime_480Cycles);

    ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

    ADC_DMACmd(ADC2, ENABLE);

    ADC_Cmd(ADC2, ENABLE);

    ADC_SoftwareStartConv(ADC2);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//  Compute and return any ADC1 value (ADC Pins 4, 5, 6, & vBat)
///////////////////////////////////////////////////////////////////////////////

float adcValue(uint8_t pin)
{
    uint8_t i;
    uint16_t adcSum = 0;

    for (i = pin; i < pin + 13; i += 4)
        adcSum += adc1ConvertedValues[i];

    return (float)adcSum / 4.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  Compute and return MXR9150 X axis value
///////////////////////////////////////////////////////////////////////////////

float mxr9150Xaxis(void)
{
	uint8_t  i;
	uint16_t adcSum = 0;

	for (i = MXR9150_XAXIS_CONVERTED_VALUE; i < MXR9150_XAXIS_CONVERTED_VALUE + 13; i += 3)
	    adcSum += adc2ConvertedValues[i];

	return (float)adcSum / 5.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  Compute and return MXR9150 Y axis value
///////////////////////////////////////////////////////////////////////////////

float mxr9150Yaxis(void)
{
	uint8_t  i;
	uint16_t adcSum = 0;

	for (i = MXR9150_YAXIS_CONVERTED_VALUE; i < MXR9150_YAXIS_CONVERTED_VALUE + 13; i += 3)
	    adcSum += adc2ConvertedValues[i];

	return (float)adcSum / 5.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  Compute and return MXR9150 Z axis value
///////////////////////////////////////////////////////////////////////////////

float mxr9150Zaxis(void)
{
	uint8_t  i;
	uint16_t adcSum = 0;

	for (i = MXR9150_ZAXIS_CONVERTED_VALUE; i < MXR9150_ZAXIS_CONVERTED_VALUE + 13; i += 3)
	    adcSum += adc2ConvertedValues[i];

	return (float)adcSum / 5.0f;
}

///////////////////////////////////////////////////////////////////////////////

// test code begin

uint16_t mxr9150X(void)
{
	return adc2ConvertedValues[MXR9150_XAXIS_CONVERTED_VALUE];
}

uint16_t mxr9150Y(void)
{
	return adc2ConvertedValues[MXR9150_YAXIS_CONVERTED_VALUE];
}

uint16_t mxr9150Z(void)
{
	return adc2ConvertedValues[MXR9150_ZAXIS_CONVERTED_VALUE];
}

uint16_t vbatt(void)
{
	return adc1ConvertedValues[VBATT_CONVERTED_VALUE];
}

// test code end

