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

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART1 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART1_TX_PIN        GPIO_Pin_9
#define UART1_RX_PIN        GPIO_Pin_10
#define UART1_GPIO          GPIOA
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART1_BUFFER_SIZE   2048

// Receive buffer, circular DMA
volatile uint8_t rx1Buffer[UART1_BUFFER_SIZE];
uint32_t rx1DMAPos = 0;

volatile uint8_t tx1Buffer[UART1_BUFFER_SIZE];
volatile uint16_t tx1BufferTail = 0;
volatile uint16_t tx1BufferHead = 0;

volatile uint8_t  tx1DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART1 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart1TxDMA(void)
{
	if ((tx1DmaEnabled == true) || (tx1BufferHead == tx1BufferTail))  // Ignore call if already active or no new data in buffer
        return;

    DMA2_Stream7->M0AR = (uint32_t)&tx1Buffer[tx1BufferTail];

    if (tx1BufferHead > tx1BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, tx1BufferHead - tx1BufferTail);
	    tx1BufferTail = tx1BufferHead;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, UART1_BUFFER_SIZE - tx1BufferTail);
	    tx1BufferTail = 0;
    }

    tx1DmaEnabled = true;

    DMA_Cmd(DMA2_Stream7, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA2_Stream7_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

    tx1DmaEnabled = false;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Initialization
///////////////////////////////////////////////////////////////////////////////

void telemetryInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    USART_StructInit(&USART_InitStructure);
    DMA_StructInit(&DMA_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,   ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = UART1_TX_PIN | UART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PINSOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PINSOURCE, GPIO_AF_USART1);

    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
  //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  //USART_InitStructure.USART_Parity              = USART_Parity_No;
  //USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA2_Stream5);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx1Buffer;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = UART1_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream5, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    rx1DMAPos = DMA_GetCurrDataCounter(DMA2_Stream5);

    // Transmit DMA
    DMA_DeInit(DMA2_Stream7);

  //DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx1Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  //DMA_InitStructure.DMA_BufferSize         = UART_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  //DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA2_Stream7, 0);

    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Available
///////////////////////////////////////////////////////////////////////////////

uint16_t telemetryAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA2_Stream5) != rx1DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Read
///////////////////////////////////////////////////////////////////////////////

uint8_t telemetryRead(void)
{
    uint8_t ch;

    ch = rx1Buffer[UART1_BUFFER_SIZE - rx1DMAPos];
    // go back around the buffer
    if (--rx1DMAPos == 0)
	    rx1DMAPos = UART1_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t telemetryReadPoll(void)
{
    while (!telemetryAvailable()); // wait for some bytes
    return telemetryRead();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Write
///////////////////////////////////////////////////////////////////////////////

void telemetryWrite(uint8_t ch)
{
    tx1Buffer[tx1BufferHead] = ch;
    tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Print
///////////////////////////////////////////////////////////////////////////////

void telemetryPrint(char *str)
{
    while (*str)
    {
    	tx1Buffer[tx1BufferHead] = *str++;
    	tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;
    }

	uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Print Formatted - Print formatted string to Telemetry Port
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void telemetryPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	telemetryPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
