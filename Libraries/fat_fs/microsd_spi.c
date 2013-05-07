/**
 ********************************************************************************
 **
 **  File        : microsd_spi.c
 **
 **  Abstract    : driver pour la carte sd (avec liaison spi)
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **                STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 **  Author 	 : Ronan Douguet
 **
 **  Date	     : 08-03-2012
 **
 ********************************************************************************
 */

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

//#include "stm32f4xx.h"
//#include "ffconf.h"
//#include "microsd_spi.h"

/* SPIx Communication boards Interface */
#define STM32_SD_USE_DMA 1

#ifdef STM32_SD_USE_DMA
#pragma message "*** Using DMA ***"
#endif

// MOSI PA 7
// SCK PA5
// CS PE10
// MISO PA6

#define STM32_SD_DISK_IOCTRL_FORCE      0

#define CARD_SUPPLY_SWITCHABLE   0
#define SOCKET_WP_CONNECTED      0
#define SPIx_SD                         SPI1
#define SPIx_SD_CLK                     RCC_APB2Periph_SPI1
#define SPIx_SD_CLK_INIT                RCC_APB2PeriphClockCmd
//#define SPIx_SD_IRQn                    SPI1_IRQn
//#define SPIx_SD_IRQHANDLER              SPI1_IRQHandler

#define SPIx_SD_SCK_PIN                 GPIO_Pin_5
#define SPIx_SD_SCK_GPIO_PORT           GPIOA
#define SPIx_SD_SCK_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define SPIx_SD_SCK_SOURCE              GPIO_PinSource5
#define SPIx_SD_SCK_AF                  GPIO_AF_SPI1

#define SPIx_SD_MISO_PIN                GPIO_Pin_6
#define SPIx_SD_MISO_GPIO_PORT          GPIOA
#define SPIx_SD_MISO_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define SPIx_SD_MISO_SOURCE             GPIO_PinSource6
#define SPIx_SD_MISO_AF                 GPIO_AF_SPI1

#define SPIx_SD_MOSI_PIN                GPIO_Pin_7
#define SPIx_SD_MOSI_GPIO_PORT          GPIOA
#define SPIx_SD_MOSI_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define SPIx_SD_MOSI_SOURCE             GPIO_PinSource7
#define SPIx_SD_MOSI_AF                 GPIO_AF_SPI1

#define SPIx_SD_NSS_PIN                	GPIO_Pin_10
#define SPIx_SD_NSS_GPIO_PORT          	GPIOE
#define SPIx_SD_NSS_GPIO_CLK           	RCC_AHB1Periph_GPIOE
#define SPIx_SD_NSS_SOURCE             	GPIO_PinSource10
//#define SPIx_SD_NSS_AF                 	GPIO_AF_SPI1

#define SPIx_SD_BAUDRATE_SLOW  			SPI_BaudRatePrescaler_128
#define SPIx_SD_BAUDRATE_FAST  			SPI_BaudRatePrescaler_4

#define DMA_Channel_SPIx_SD_RX	 DMA_Channel_3
#define DMA_Channel_SPIx_SD_TX	 DMA_Channel_3
#define DMA_Stream_SPIx_SD_RX    DMA2_Stream2
#define DMA_Stream_SPIx_SD_TX    DMA2_Stream3
#define DMA_FLAG_SPI_SD_TC_RX    DMA_FLAG_TCIF2
#define DMA_FLAG_SPI_SD_TC_TX    DMA_FLAG_TCIF3

/* Definitions for SDC command --------------------------------------------*/
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card-Select Controls  ------------------------------------------------------*/
#define SELECT()        GPIO_ResetBits(SPIx_SD_NSS_GPIO_PORT, SPIx_SD_NSS_PIN)    /* MMC CS = L */
#define DESELECT()      GPIO_SetBits(SPIx_SD_NSS_GPIO_PORT, SPIx_SD_NSS_PIN)      /* MMC CS = H */

#if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
#define _USE_IOCTL   1
#else
#define _USE_IOCTL   0
#endif
/*--------------------------------------------------------------------------

 Module Private Functions and Variables

 ---------------------------------------------------------------------------*/

//static const DWORD socket_state_mask_cp = (1 << 0);
//static const DWORD socket_state_mask_wp = (1 << 1);
static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */
static volatile DWORD Timer1, Timer2;			/* 100Hz decrement timers */
static BYTE CardType; /* Card type flags */

enum speed_setting
{
    INTERFACE_SLOW, INTERFACE_FAST
};

/*-----------------------------------------------------------------------*/
/* @descr  INTERFACE SPEED						 						 */
/* @param  speed	INTERFACE_SLOW OR INTERFACE_FAST					 */
/* @retval none					 										 */
/*-----------------------------------------------------------------------*/
static void interface_speed(enum speed_setting speed)
{
    DWORD tmp;

    /* Lecture du registre CR1 ------------------------------- */
    tmp = SPIx_SD->CR1;

    /* Test la configuration --- ----------------------------- */
    if (speed == INTERFACE_SLOW)
    {
        /* Set slow clock (100k-400k) ------------------------ */
        tmp = (tmp | SPIx_SD_BAUDRATE_SLOW);
    }
    else
    {
        /* Set fast clock (depends on the CSD) --------------- */
        tmp = (tmp & ~SPIx_SD_BAUDRATE_SLOW) | SPIx_SD_BAUDRATE_FAST;
    }

    /* Ecriture de la nouvelle config. sur le registre CR1 --- */
    SPIx_SD ->CR1 = tmp;
}

/*-----------------------------------------------------------------------*/
/* @descr  TRANSMIT & RECEIVE BYTE via SPI						 		 */
/* @param  out				octet à transemttre							 */
/* @retval stm32_spi_rw		octet reçu									 */
/*-----------------------------------------------------------------------*/
static BYTE stm32_spi_rw(BYTE out)
{
    /* Send byte through the SPI peripheral ------------------ */
    SPI_I2S_SendData(SPIx_SD, out);

    /* Wait to receive a byte -------------------------------- */
    while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_RXNE) == RESET)
    {
        ;
    }

    /* Return the byte read from the SPI bus ----------------- */
    return SPI_I2S_ReceiveData(SPIx_SD);
}

/*-----------------------------------------------------------------------*/
/* Transmit a byte via SPI  (Platform dependent)                  		 */
/*-----------------------------------------------------------------------*/
#define xmit_spi(dat)  stm32_spi_rw(dat)

/*-----------------------------------------------------------------------*/
/* @descr  RECEIVE A BYTE via SPI										 */
/* @param  None															 */
/* @retval rcvr_spi		octet reçu										 */
/*-----------------------------------------------------------------------*/
static BYTE rcvr_spi(void)
{
    /* Transmet un octet pour recevoir la reponse ------------ */
    return stm32_spi_rw(0xff);
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)

/*-----------------------------------------------------------------------*/
/* @descr  Wait for card ready											 */
/* @param  None															 */
/* @retval wait_ready	octet reçu										 */
/*-----------------------------------------------------------------------*/
static BYTE wait_ready(void)
{
    BYTE res;

    Timer2 = 50; /* Wait for ready in timeout of 500ms */
    rcvr_spi();

    do
        res = rcvr_spi();

    while ((res != 0xFF) && Timer2);

    return res;
}

/*-----------------------------------------------------------------------*/
/* @descr  Deselect the card and release SPI bus						 */
/* @param  None															 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static void release_spi(void)
{
    DESELECT();
    rcvr_spi();
}

#ifdef STM32_SD_USE_DMA
/*-----------------------------------------------------------------------*/
/* Transmit/Receive Block using DMA (Platform dependent. STM32 here)     */
/*-----------------------------------------------------------------------*/
static
void stm32_dma_transfer(
    BOOL receive,		/* FALSE for TX (buff->SPI), TRUE for RX (SPI->buff)     */
    const BYTE *buff,	/* receive TRUE  : 512 byte data block to be transmitted
						   receive FALSE : Data buffer to store received data    */
    UINT btr 			/* receive TRUE  : Byte count (must be multiple of 2)
						   receive FALSE : Byte count (must be 512)              */
)
{
    DMA_InitTypeDef DMA_InitStructure;
    WORD rw_workbyte[] = { 0xffff };

#ifndef NOT_USED
    DMA_DeInit(DMA_Stream_SPIx_SD_RX);
    DMA_DeInit(DMA_Stream_SPIx_SD_TX);

    DMA_StructInit (&DMA_InitStructure);

    /* shared DMA configuration values */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (DWORD)(&(SPIx_SD->DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_BufferSize = btr;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  // AKA CHANGE
#endif

    if (receive)
    {
        /* DMA2 channel2 configuration SPI1 RX ---------------------------------------------*/
#ifndef NOT_USED
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
        DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_RX;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_Init(DMA_Stream_SPIx_SD_RX, &DMA_InitStructure);
#else

        DMA_Stream_SPIx_SD_RX->NDTR = (uint32_t)btr;
        DMA_Stream_SPIx_SD_RX->M0AR = (uint32_t)buff;
#endif

        /* DMA2 channel3 configuration SPI1 TX ---------------------------------------------*/
#ifndef NOT_USED
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
        DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_TX;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        DMA_Init(DMA_Stream_SPIx_SD_TX, &DMA_InitStructure);
#else

        DMA_Stream_SPIx_SD_TX->NDTR = (uint32_t)btr;
        DMA_Stream_SPIx_SD_TX->M0AR = (uint32_t)rw_workbyte;
#endif
    }
    else
    {
#if _FS_READONLY == 0
        /* DMA2 channel2 configuration SPI1 RX ---------------------------------------------*/
#ifndef NOT_USED
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
        DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_RX;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        DMA_Init(DMA_Stream_SPIx_SD_RX, &DMA_InitStructure);
#else

        DMA_Stream_SPIx_SD_RX->NDTR = (uint32_t)btr;
        DMA_Stream_SPIx_SD_RX->M0AR = (uint32_t)rw_workbyte;
#endif
        /* DMA2 channel3 configuration SPI1 TX ---------------------------------------------*/
#ifndef NOT_USED
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
        DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_TX;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_Init(DMA_Stream_SPIx_SD_TX, &DMA_InitStructure);
#else

        DMA_Stream_SPIx_SD_TX->NDTR = (uint32_t)btr;
        DMA_Stream_SPIx_SD_TX->M0AR = (uint32_t)buff;
#endif
#endif

    }

    /* Enable DMA RX Channel */
    DMA_Cmd(DMA_Stream_SPIx_SD_RX, ENABLE);
    /* Enable DMA TX Channel */
    DMA_Cmd(DMA_Stream_SPIx_SD_TX, ENABLE);

    /* Enable SPI TX/RX request */
    SPI_I2S_DMACmd(SPIx_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

    /* Wait until DMA2_Channel 3 Transfer Complete */
    /// not needed: while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_TX) == RESET) { ; }
    /* Wait until DMA2_Channel 2 Receive Complete */
    while (DMA_GetFlagStatus(DMA_Stream_SPIx_SD_RX, DMA_FLAG_SPI_SD_TC_RX) == RESET)
    {
        ;
    }

    // same w/o function-call:
    // while ( ( ( DMA2->ISR ) & DMA_FLAG_SPI_SD_TC_RX ) == RESET ) { ; }

    /* Disable DMA RX Channel */
    DMA_Cmd(DMA_Stream_SPIx_SD_RX, DISABLE);
    /* Disable DMA TX Channel */
    DMA_Cmd(DMA_Stream_SPIx_SD_TX, DISABLE);

    /* Disable SPI RX/TX request */
    SPI_I2S_DMACmd(SPIx_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);

}
#endif /* STM32_SD_USE_DMA */

/*-----------------------------------------------------------------------*/
/* @descr  Power Control and interface-initialization					 */
/* @param  None															 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static void power_on(void)
{
    volatile BYTE dummyread __attribute__((unused));

    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef STM32_SD_USE_DMA
    DMA_InitTypeDef DMA_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;
#endif

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(
        SPIx_SD_SCK_GPIO_CLK | SPIx_SD_MISO_GPIO_CLK | SPIx_SD_MOSI_GPIO_CLK
        | SPIx_SD_NSS_GPIO_CLK, ENABLE);

    /* Enable the SPI clock */
    SPIx_SD_CLK_INIT(SPIx_SD_CLK, ENABLE);

    /* SPI GPIO Configuration --------------------------------------------------*/

    /* Connect SPI pins to AF5 */
    GPIO_PinAFConfig(SPIx_SD_SCK_GPIO_PORT, SPIx_SD_SCK_SOURCE,
                     SPIx_SD_SCK_AF);
    GPIO_PinAFConfig(SPIx_SD_MOSI_GPIO_PORT, SPIx_SD_MOSI_SOURCE,
                     SPIx_SD_MOSI_AF);
    GPIO_PinAFConfig(SPIx_SD_MISO_GPIO_PORT, SPIx_SD_MISO_SOURCE,
                     SPIx_SD_MOSI_AF);

    /* Configure I/O for Flash Chip select */
    GPIO_InitStructure.GPIO_Pin = SPIx_SD_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPIx_SD_NSS_GPIO_PORT, &GPIO_InitStructure);

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPIx_SD_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPIx_SD_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPIx_SD_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPIx_SD_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /* SPI  MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPIx_SD_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPIx_SD_MISO_GPIO_PORT, &GPIO_InitStructure);

    /* SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_SD_BAUDRATE_SLOW; // 42000kHz/128=328kHz < 400kHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx_SD, &SPI_InitStructure);
    SPI_CalculateCRC(SPIx_SD, DISABLE);
    SPI_Cmd(SPIx_SD, ENABLE);

    /* De-select the Card: Chip Select high */
    DESELECT();

    /* drain SPI */
    while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET)
    {
        ;
    }

    dummyread = SPI_I2S_ReceiveData(SPIx_SD);

#ifdef STM32_SD_USE_DMA
    /* enable DMA clock */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // all new coded added below AKA
    /* setup the DMA TX Channel/Stream */
    DMA_DeInit(DMA_Stream_SPIx_SD_TX);

    /*
      * Check if the DMA Stream is disabled before enabling it.
      * Note that this step is useful when the same Stream is used multiple times:
      * enabled, then disabled then re-enabled... In this case, the DMA Stream disable
      * will be effective only at the end of the ongoing data transfer and it will
      * not be possible to re-configure it before making sure that the Enable bit
      * has been cleared by hardware. If the Stream is used only once, this step might
      * be bypassed.
      */

	while (DMA_GetCmdStatus(DMA_Stream_SPIx_SD_TX) != DISABLE)
	{
		;
	}

	DMA_StructInit (&DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPIx_SD->DR);
	DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_TX;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;
	DMA_InitStructure.DMA_BufferSize = 1;

	DMA_Init (DMA_Stream_SPIx_SD_TX, &DMA_InitStructure);

	// Enable dma tx request.
	SPI_I2S_DMACmd (SPIx_SD, SPI_I2S_DMAReq_Tx, ENABLE);

    /* setup the DMA RX Channel/Stream */
    DMA_DeInit(DMA_Stream_SPIx_SD_RX);

    /*
      * Check if the DMA Stream is disabled before enabling it.
      * Note that this step is useful when the same Stream is used multiple times:
      * enabled, then disabled then re-enabled... In this case, the DMA Stream disable
      * will be effective only at the end of the ongoing data transfer and it will
      * not be possible to re-configure it before making sure that the Enable bit
      * has been cleared by hardware. If the Stream is used only once, this step might
      * be bypassed.
      */

	while (DMA_GetCmdStatus(DMA_Stream_SPIx_SD_RX) != DISABLE)
	{
		;
	}

	DMA_StructInit (&DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPIx_SD->DR);
	DMA_InitStructure.DMA_Channel = DMA_Channel_SPIx_SD_RX;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;
	DMA_InitStructure.DMA_BufferSize = 1;

	DMA_Init (DMA_Stream_SPIx_SD_RX, &DMA_InitStructure);

	// Enable dma rx request.
	SPI_I2S_DMACmd (SPIx_SD, SPI_I2S_DMAReq_Rx, ENABLE);

#endif
}

/*-----------------------------------------------------------------------*/
/* @descr  Power Off													 */
/* @param  None															 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static void power_off(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (!(Stat & STA_NOINIT))
    {
        SELECT();
        wait_ready();
        release_spi();
    }

    SPI_I2S_DeInit(SPIx_SD);
    SPI_Cmd(SPIx_SD, DISABLE);
    SPIx_SD_CLK_INIT(SPIx_SD_CLK, DISABLE);

    /* All SPI-Pins to input with weak internal pull-downs */
    GPIO_InitStructure.GPIO_Pin = SPIx_SD_SCK_PIN | SPIx_SD_MISO_PIN
                                  | SPIx_SD_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPIx_SD_SCK_GPIO_PORT, &GPIO_InitStructure);

    Stat |= STA_NOINIT; /* Set STA_NOINIT */
}

/*-----------------------------------------------------------------------*/
/* @descr  Receive a data packet from MMC								 */
/* @param  *buff	Data buffer to store received data   				 */
/* @param  btr		Byte count (must be multiple of 4)   				 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static BOOL rcvr_datablock(BYTE *buff, UINT btr)
{
    BYTE token;

    Timer1 = 10;

    do   /* Wait for data packet in timeout of 100ms */
    {
        token = rcvr_spi();
    }
    while ((token == 0xFF) && Timer1);

    if (token != 0xFE)
        return FALSE; /* If not valid data token, return with error */

#ifdef STM32_SD_USE_DMA
    stm32_dma_transfer(TRUE, buff, btr);
#else

    do   /* Receive the data block into buffer */
    {
        rcvr_spi_m(buff++);
        rcvr_spi_m(buff++);
        rcvr_spi_m(buff++);
        rcvr_spi_m(buff++);
    }
    while (btr -= 4);

#endif /* STM32_SD_USE_DMA */

    rcvr_spi(); /* Discard CRC */
    rcvr_spi();

    return TRUE; /* Return with success */
}

#if _FS_READONLY == 0
/*-----------------------------------------------------------------------*/
/* @descr  end a data packet to MMC     								 */
/* @param  *buff	512 byte data block to be transmitted  				 */
/* @param  token	Data/Stop token 	  				 				 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static BOOL xmit_datablock(const BYTE *buff, BYTE token)
{
    BYTE resp;
#ifndef STM32_SD_USE_DMA
    BYTE wc;
#endif

    if (wait_ready() != 0xFF)
        return FALSE;

    xmit_spi(token); /* transmit data token */

    if (token != 0xFD)   /* Is data token */
    {

#ifdef STM32_SD_USE_DMA
        stm32_dma_transfer(FALSE, buff, 512);
#else
        wc = 0;

        do   /* transmit the 512 byte data block to MMC */
        {
            xmit_spi(*buff++);
            xmit_spi(*buff++);
        }
        while (--wc);

#endif /* STM32_SD_USE_DMA */

        xmit_spi(0xFF); /* CRC (Dummy) */
        xmit_spi(0xFF);
        resp = rcvr_spi(); /* Receive data response */

        if ((resp & 0x1F) != 0x05) /* If not accepted, return with error */
            return FALSE;
    }

    return TRUE;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* @descr  SEND A COMMAND PACKET (6 BYTES) 								 */
/* @param  cmd		Command byte  										 */
/* @param  arg		Argument   				 				 			 */
/* @retval None															 */
/*-----------------------------------------------------------------------*/
static BYTE send_cmd(BYTE cmd, DWORD arg)
{
    /* Declaration des variables ----------------------------- */
    BYTE n, res;

    /* Test si c'est cmd ACDM -------------------------------- */
    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);		 // envoie de la CMD55

        if (res > 1)
            return res;
    }

    /* Select the card --------------------------------------- */
    DESELECT();
    SELECT();

    /* wait for ready ---------------------------------------- */
    if (cmd != CMD0)
    {
        if (wait_ready() != 0xFF)
        {
            return 0xFF;
        }
    }

    /* Send command packet ----------------------------------- */
    xmit_spi(cmd);						// Send command index

    xmit_spi((BYTE)(arg >> 24));		// Send argument[31..24]
    xmit_spi((BYTE)(arg >> 16));		// Send argument[23..16]
    xmit_spi((BYTE)(arg >> 8));			// Send argument[15..8]
    xmit_spi((BYTE)arg);				// Send argument[7..0]

    n = 0x01;							// Stop : Dummy CRC

    if (cmd == CMD0)
        n = 0x95;			// Valid CRC for CMD0(0)

    if (cmd == CMD8)
        n = 0x87;			// Valid CRC for CMD8(0x1AA)

    xmit_spi(n);						// Send CRC

    /* Receive command response ------------------------------ */
    if (cmd == CMD12)
        rcvr_spi();		// Skip a stuff byte when stop reading

    /* Wait for a valid response in timeout of 10 attempts --- */
    n = 10;

    do
        res = rcvr_spi();

    while ((res & 0x80) && --n);

    /* Return with the response value ------------------------ */
    return res;
}

/*-----------------------------------------------------------------------*/
/* @descr  INITIALIZE DISK DRIVE  								 		 */
/* @param  drv		Physical drive numbre (0)							 */
/* @retval DSTATUS	Status of Disk Functions (BYTE)						 */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE drv)
{
    /* Declaration des variables ----------------------------- */
    BYTE n, cmd, ty, ocr[4];

    /* Supports only single drive ---------------------------- */
    if (drv)
        return STA_NOINIT;

    /* No card in the socket --------------------------------- */
    if (Stat & STA_NODISK)
        return Stat;

    /* Configure les broches & alimente la carte si necessaire */
    power_on();
    interface_speed(INTERFACE_SLOW);

    DESELECT();

    /* 80 dummy clocks --------------------------------------- */
    for (n = 10; n; n--)
        rcvr_spi();

    SELECT();

    /* Initialise SDType ------------------------------------- */
    ty = 0;

    /* Initialise la SDCard en mode SPI ---------------------- */
    if (send_cmd(CMD0, 0) == 1)
    {
        /* Verifie les conditions (tension,...) -------------- */
        Timer1 = 100;

        if (send_cmd(CMD8, 0x1AA) == 1)
        {

            /* lecture des trames de réponses ---------------- */
            for (n = 0; n < 4; n++)
                ocr[n] = rcvr_spi();

            /* Verifie les tensions autorisées --------------- */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                /* Attend de quitter l'état repos ------------ */
                while (Timer1 && send_cmd(ACMD41, 1UL << 30))
                    ;

                /* Envoie de la commande OCR ----------------- */
                if (Timer1 && send_cmd(CMD58, 0) == 0)
                {
                    for (n = 0; n < 4; n++)
                        ocr[n] = rcvr_spi();

                    /* Control bit CCS dans le registre OCR -- */
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else  							/* SDSC or MMC */
        {
            if (send_cmd(ACMD41, 0) <= 1)
            {
                ty = CT_SD1;
                cmd = ACMD41;	/* SDSC */
            }
            else
            {
                ty = CT_MMC;
                cmd = CMD1;	/* MMC */
            }

            while (Timer1 && send_cmd(cmd, 0))
            {
                ;			/* Wait for leaving idle state */
            }

            if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
                ty = 0;
        }
    }


    /* Deselectionne la liaison SPI -------------------------- */
    release_spi();

    /* Affecte le type de la carte a CardType ---------------- */
    CardType = ty;

    /* Test si l'initialisation est valide ------------------- */
    if (ty)  			/* Initialization succeeded */
    {
        /* Initialization succeeded -------------------------- */
        Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
        interface_speed(INTERFACE_FAST);
#ifdef DEBUG
        cliPrintF("sd init success\n");
#endif
        //LED3On();

    }
    else
    {
        /* Initialization failed ----------------------------- */
        power_off();
#ifdef DEBUG
        cliPrintF("sd card failure\n");
#endif
    }

    return Stat;

}

/*-----------------------------------------------------------------------*/
/* @descr  GET DISK STATUS      								 		 */
/* @param  drv		Physical drive numbre (0)							 */
/* @retval DSTATUS	Status of Disk Functions (BYTE) 					 */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE drv)
{
    if (drv)
        return STA_NOINIT; /* Supports only single drive */

    return Stat;
}

/*-----------------------------------------------------------------------*/
/* @descr  READ SECTOR(S)          								 		 */
/* @param  drv		Physical drive numbre (0)							 */
/* @param  *buff	Pointer to the data buffer to store read data		 */
/* @param  sector	Start sector number (LBA)							 */
/* @param  count	Sector count (1..255)   							 */
/* @retval DRESULT	Results of Disk Functions  							 */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
    /* Check parameters -------------------------------------- */
    if (drv || !count)
        return RES_PARERR;

    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    /* Convert to byte address if needed --------------------- */
    if (!(CardType & CT_BLOCK))
        sector *= 512;

    /* Test : Single or Multiple block read ------------------ */
    if (count == 1)
    {
        /* Single block read */
        if (send_cmd(CMD17, sector) == 0)
        {
            /* READ_SINGLE_BLOCK */
            if (rcvr_datablock(buff, 512))
            {
                count = 0;
            }
        }
    }
    else
    {
        /* Multiple block read */
        if (send_cmd(CMD18, sector) == 0)
        {
            /* READ_MULTIPLE_BLOCK */
            do
            {
                if (!rcvr_datablock(buff, 512))
                {
                    break;
                }

                buff += 512;
            }
            while (--count);

            /* STOP_TRANSMISSION */
            send_cmd(CMD12, 0);
        }
    }

    release_spi();

    return count ? RES_ERROR : RES_OK;
}

#if _FS_READONLY == 0
/*-----------------------------------------------------------------------*/
/* @descr  WRITE SECTOR(S)         								 		 */
/* @param  drv		Physical drive numbre (0)							 */
/* @param  *buff	Pointer to the data to be written					 */
/* @param  sector	Start sector number (LBA)							 */
/* @param  count	Sector count (1..255)   							 */
/* @retval DRESULT	Results of Disk Functions  							 */
/*-----------------------------------------------------------------------*/
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{
    /* Check parameters -------------------------------------- */
    if (drv || !count)
        return RES_PARERR;

    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    if (Stat & STA_PROTECT)
        return RES_WRPRT;

    /* Convert to byte address if needed --------------------- */
    if (!(CardType & CT_BLOCK))
        sector *= 512;

    /* Test : Single or Multiple block write ----------------- */
    if (count == 1)
    {
        /* Single block write -------------------------------- */
        if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
                && xmit_datablock(buff, 0xFE))
            count = 0;
    }
    else
    {
        /* Multiple block write */
        if (CardType & CT_SDC)
            send_cmd(ACMD23, count);

        if (send_cmd(CMD25, sector) == 0)
        {
            /* WRITE_MULTIPLE_BLOCK */
            do
            {
                if (!xmit_datablock(buff, 0xFC))
                    break;

                buff += 512;
            }
            while (--count);

            /* STOP_TRAN token */
            if (!xmit_datablock(0, 0xFD))
                count = 1;
        }
    }

    release_spi();

    return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY == 0 */

#if (_USE_IOCTL == 1)
/*-----------------------------------------------------------------------*/
/* @descr  GET CURRENT TIME INTO A DWORD VALUE					 		 */
/* @param  drv		Physical drive number (0)							 */
/* @param  ctrl		Control code  							 			 */
/* @param  *buff	Buffer to send/receive control data 				 */
/* @param  DRESULT	Results of Disk Functions	 						 */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
    DRESULT res;
    BYTE n, csd[16], *ptr = buff;
    WORD csize;

    if (drv)
    	return RES_PARERR;

    res = RES_ERROR;

    if (ctrl == CTRL_POWER)
    {
        switch (*ptr)
        {
            case 0:		/* Sub control code == 0 (POWER_OFF) */
                power_off();		/* Power off */
                res = RES_OK;
                break;

            case 1:		/* Sub control code == 1 (POWER_ON) */
                power_on();				/* Power on */
                res = RES_OK;
                break;

            case 2:		/* Sub control code == 2 (POWER_GET) */
                //fix
                *(ptr + 1) = (BYTE)1;
                res = RES_OK;
                break;

            default :
                res = RES_PARERR;
        }
    }
    else
    {
        if (Stat & STA_NOINIT)
        	return RES_NOTRDY;

        switch (ctrl)
        {
            case CTRL_SYNC :		/* Make sure that no pending write process */
                SELECT();

                if (wait_ready() == 0xFF)
                    res = RES_OK;

                break;

            case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
                if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
                {
                    if ((csd[0] >> 6) == 1)  	/* SDC version 2.00 */
                    {
                        csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                        *(DWORD *)buff = (DWORD)csize << 10;
                    }
                    else  					/* SDC version 1.XX or MMC*/
                    {
                        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                        csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                        *(DWORD *)buff = (DWORD)csize << (n - 9);
                    }

                    res = RES_OK;
                }

                break;

            case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
                *(WORD *)buff = 512;
                res = RES_OK;
                break;

            case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
                if (CardType & CT_SD2)  	/* SDC version 2.00 */
                {
                    if (send_cmd(ACMD13, 0) == 0)  	/* Read SD status */
                    {
                        rcvr_spi();

                        if (rcvr_datablock(csd, 16))  				/* Read partial block */
                        {
                            for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */

                            *(DWORD *)buff = 16UL << (csd[10] >> 4);
                            res = RES_OK;
                        }
                    }
                }
                else  					/* SDC version 1.XX or MMC */
                {
                    if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))  	/* Read CSD */
                    {
                        if (CardType & CT_SD1)  	/* SDC version 1.XX */
                        {
                            *(DWORD *)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                        }
                        else  					/* MMC */
                        {
                            *(DWORD *)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                        }

                        res = RES_OK;
                    }
                }

                break;

            case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
                *ptr = CardType;
                res = RES_OK;
                break;

            case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
                if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
                        && rcvr_datablock(ptr, 16))
                    res = RES_OK;

                break;

            case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
                if (send_cmd(CMD10, 0) == 0		/* READ_CID */
                        && rcvr_datablock(ptr, 16))
                    res = RES_OK;

                break;

            case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
                if (send_cmd(CMD58, 0) == 0)  	/* READ_OCR */
                {
                    for (n = 4; n; n--) *ptr++ = rcvr_spi();

                    res = RES_OK;
                }

                break;

            case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
                if (send_cmd(ACMD13, 0) == 0)  	/* SD_STATUS */
                {
                    rcvr_spi();

                    if (rcvr_datablock(ptr, 64))
                        res = RES_OK;
                }

                break;

            default:
                res = RES_PARERR;
        }

        release_spi();
    }

    return res;
}
#endif /* _USE_IOCTL != 0 */

/*-----------------------------------------------------------------------*/
/* @descr  GET CURRENT TIME INTO A DWORD VALUE					 		 */
/* @param  none															 */
/* @retval DWORD	Results of Disk Functions  							 */
/*-----------------------------------------------------------------------*/
DWORD get_fattime(void)
{
    DWORD res;

    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;

    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

    res = (((DWORD) RTC_DateStructure.RTC_Year + 20) << 25)	// bit31:25		Year from 1980 (0..127)
          | ((DWORD) RTC_DateStructure.RTC_Month << 21)// bit24:21		Month (1..12)
          | ((DWORD) RTC_DateStructure.RTC_Date << 16)// bit20:16		Day in month(1..31)
          | (WORD)(RTC_TimeStructure.RTC_Hours << 11) // bit15:11		Hour (0..23)
          | (WORD)(RTC_TimeStructure.RTC_Minutes << 5) // bit10:5		Minute (0..59)
          | (WORD)(RTC_TimeStructure.RTC_Seconds >> 1); // bit4:0		Second / 2 (0..29)

    return res;
}

void disk_timerproc(void)
{
    //	static DWORD pv;
    //	DWORD ns;
    BYTE n;//, s;


    n = Timer1;                /* 100Hz decrement timers */

    if (n)
        Timer1 = --n;

    n = Timer2;

    if (n)
        Timer2 = --n;

#ifdef _USE_DISK_SENSE
    ns = pv;
    pv = socket_is_empty() | socket_is_write_protected();	/* Sample socket switch */

    if (ns == pv)                           /* Have contacts stabled? */
    {
        s = Stat;

        if (pv & socket_state_mask_wp)      /* WP is H (write protected) */
            s |= STA_PROTECT;
        else                                /* WP is L (write enabled) */
            s &= ~STA_PROTECT;

        if (pv & socket_state_mask_cp)      /* INS = H (Socket empty) */
            s |= (STA_NODISK | STA_NOINIT);
        else                                /* INS = L (Card inserted) */
            s &= ~STA_NODISK;

        Stat = s;
    }
#endif
}


