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

// SPI1
// SCK  PA5
// MISO PA6
// MOSI PA7

// SPI2
// SCK  PB13
// MISO PB14
// MOSI PB15

// SPI3
// SCK  PC10
// MISO PC11
// MOSI PC12

///////////////////////////////////////////////////////////////////////////////
// SPI Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SPI1_GPIO             GPIOA
#define SPI1_SCK_PIN          GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE   GPIO_PinSource5
#define SPI1_MISO_PIN         GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE  GPIO_PinSource6
#define SPI1_MOSI_PIN         GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE  GPIO_PinSource7

#define SPI2_GPIO             GPIOB
#define SPI2_SCK_PIN          GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE   GPIO_PinSource13
#define SPI2_MISO_PIN         GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE  GPIO_PinSource14
#define SPI2_MOSI_PIN         GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE  GPIO_PinSource15

#define SPI3_GPIO             GPIOC
#define SPI3_SCK_PIN          GPIO_Pin_10
#define SPI3_SCK_PIN_SOURCE   GPIO_PinSource10
#define SPI3_MISO_PIN         GPIO_Pin_11
#define SPI3_MISO_PIN_SOURCE  GPIO_PinSource11
#define SPI3_MOSI_PIN         GPIO_Pin_12
#define SPI3_MOSI_PIN_SOURCE  GPIO_PinSource12

///////////////////////////////////////////////////////////////////////////////
// SPI Initialize
///////////////////////////////////////////////////////////////////////////////

void spiInit(SPI_TypeDef *SPI)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    ///////////////////////////////////

    if (SPI == SPI1)
    {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1,  ENABLE);

        GPIO_StructInit(&GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

        GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE,  GPIO_AF_SPI1);
	    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
	    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);
}

    ///////////////////////////////////

    if (SPI == SPI2)
    {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,  ENABLE);

        GPIO_StructInit(&GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

        GPIO_PinAFConfig(SPI2_GPIO, SPI2_SCK_PIN_SOURCE,  GPIO_AF_SPI2);
	    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MISO_PIN_SOURCE, GPIO_AF_SPI2);
	    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MOSI_PIN_SOURCE, GPIO_AF_SPI2);

        GPIO_InitStructure.GPIO_Pin   = MAX7456_CS_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_Init(MAX7456_CS_GPIO, &GPIO_InitStructure);

		GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

		SPI_StructInit(&SPI_InitStructure);

        SPI_I2S_DeInit(SPI2);

      //SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
      //SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
        SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  // 42/4 = 10.5 MHz SPI Clock
      //SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
      //SPI_InitStructure.SPI_CRCPolynomial     = 7;

        SPI_Init(SPI2, &SPI_InitStructure);

        SPI_Cmd(SPI2, ENABLE);
  }

    ///////////////////////////////////

    if (SPI == SPI3)
    {
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE);

        GPIO_StructInit(&GPIO_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(SPI3_GPIO, &GPIO_InitStructure);

        GPIO_PinAFConfig(SPI3_GPIO, SPI3_SCK_PIN_SOURCE,  GPIO_AF_SPI3);
	    GPIO_PinAFConfig(SPI3_GPIO, SPI3_MISO_PIN_SOURCE, GPIO_AF_SPI3);
	    GPIO_PinAFConfig(SPI3_GPIO, SPI3_MOSI_PIN_SOURCE, GPIO_AF_SPI3);

	    GPIO_StructInit(&GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_SetBits(GPIOB, GPIO_Pin_8);

		SPI_StructInit(&SPI_InitStructure);

        SPI_I2S_DeInit(SPI3);

      //SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
      //SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
        SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
      //SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
      //SPI_InitStructure.SPI_CRCPolynomial     = 7;

        SPI_Init(SPI3, &SPI_InitStructure);

        SPI_Cmd(SPI3, ENABLE);
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// SPI Transfer
///////////////////////////////////////////////////////////////////////////////

uint8_t spiTransfer(SPI_TypeDef *SPIx, uint8_t data)
{
    uint16_t spiTimeout;

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
      if ((spiTimeout--) == 0)
          return(0);

    SPI_I2S_SendData(SPIx, data);

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
      if ((spiTimeout--) == 0)
          return(0);

    return((uint8_t)SPI_I2S_ReceiveData(SPIx));
}

///////////////////////////////////////////////////////////////////////////////
// Set SPI Divisor
///////////////////////////////////////////////////////////////////////////////

void setSPIdivisor(SPI_TypeDef *SPIx, uint16_t data)
{
    #define BR_CLEAR_MASK 0xFFC7

	uint16_t tempRegister;

    SPI_Cmd(SPIx, DISABLE);

	tempRegister = SPIx->CR1;

	switch (data)
	{
	case 2:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_2;
	    break;

	case 4:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_4;
	    break;

	case 8:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_8;
	    break;

	case 16:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_16;
	    break;

	case 32:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_32;
	    break;

	case 64:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_64;
	    break;

	case 128:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_128;
	    break;

	case 256:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_256;
	    break;
	}

	SPIx->CR1 = tempRegister;

	SPI_Cmd(SPIx, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////

