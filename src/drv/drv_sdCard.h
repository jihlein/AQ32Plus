///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define SDCARD_SPI                 SPI1

#define SDCARD_CS_GPIO             GPIOE
#define SDCARD_CS_GPIO_CLOCK       RCC_AHB1Periph_GPIOE
#define SDCARD_CS_PIN              GPIO_Pin_10

#define SDCARD_SPI_RX_DMA_CHANNEL  DMA_Channel_3
#define SDCARD_SPI_TX_DMA_CHANNEL  DMA_Channel_3
#define SDCARD_SPI_RX_DMA_STREAM   DMA2_Stream0
#define SDCARD_SPI_TX_DMA_STREAM   DMA2_Stream3
#define SDCARD_SPI_RX_TC_FLAG      DMA_FLAG_TCIF0
#define SDCARD_SPI_TX_TC_FLAG      DMA_FLAG_TCIF3

#define DISABLE_SDCARD             GPIO_SetBits(SDCARD_CS_GPIO,   SDCARD_CS_PIN)
#define ENABLE_SDCARD              GPIO_ResetBits(SDCARD_CS_GPIO, SDCARD_CS_PIN)

///////////////////////////////////////////////////////////////////////////////
