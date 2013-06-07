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

#define STM32_SD_USE_DMA 1

///////////////////////////////////////

#define STM32_SD_DISK_IOCTRL_FORCE 0

///////////////////////////////////////

#define CMD0	(0x40+0)	// GO_IDLE_STATE
#define CMD1	(0x40+1)	// SEND_OP_COND (MMC)
#define ACMD41	(0xC0+41)	// SEND_OP_COND (SDC)
#define CMD8	(0x40+8)	// SEND_IF_COND
#define CMD9	(0x40+9)	// SEND_CSD
#define CMD10	(0x40+10)	// SEND_CID
#define CMD12	(0x40+12)	// STOP_TRANSMISSION
#define ACMD13	(0xC0+13)	// SD_STATUS (SDC)
#define CMD16	(0x40+16)	// SET_BLOCKLEN
#define CMD17	(0x40+17)	// READ_SINGLE_BLOCK
#define CMD18	(0x40+18)	// READ_MULTIPLE_BLOCK
#define CMD23	(0x40+23)	// SET_BLOCK_COUNT (MMC)
#define ACMD23	(0xC0+23)	// SET_WR_BLK_ERASE_COUNT (SDC)
#define CMD24	(0x40+24)	// WRITE_BLOCK
#define CMD25	(0x40+25)	// WRITE_MULTIPLE_BLOCK
#define CMD55	(0x40+55)	// APP_CMD
#define CMD58	(0x40+58)	// READ_OCR

///////////////////////////////////////

#if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
    #define _USE_IOCTL   1
#else
    #define _USE_IOCTL   0
#endif

///////////////////////////////////////////////////////////////////////////////
// Module Private Functions and Variables
///////////////////////////////////////////////////////////////////////////////

static volatile DSTATUS Stat = STA_NOINIT;            // Disk status
static volatile DWORD Timer1, Timer2;			      // 100Hz decrement timers
static BYTE CardType;                                 // Card type flags

///////////////////////////////////////////////////////////////////////////////
// Wait for card ready
///////////////////////////////////////////////////////////////////////////////

BYTE wait_ready(void)
{
    BYTE res;

    Timer2 = 50;                                      // Wait for ready in timeout of 500ms

    spiTransfer(SDCARD_SPI, 0xFF);

    do
        res = spiTransfer(SDCARD_SPI, 0xFF);

    while ((res != 0xFF) && Timer2);

    return res;
}

///////////////////////////////////////////////////////////////////////////////
// Deselect the card and release SPI bus
///////////////////////////////////////////////////////////////////////////////

void release_spi(void)
{
    DISABLE_SDCARD;
    spiTransfer(SDCARD_SPI, 0xFF);
}

#ifdef STM32_SD_USE_DMA
///////////////////////////////////////////////////////////////////////////////
// Transmit/Receive Block using DMA (Platform dependent. STM32 here)
//   receive: FALSE for TX (buff->SPI), TRUE for RX (SPI->buff)
//   *buff:   receive TRUE  : 512 byte data block to be transmitted
//            receive FALSE : Data buffer to store received data
//   btr:     receive TRUE  : Byte count (must be multiple of 2)
//            receive FALSE : Byte count (must be 512)
///////////////////////////////////////////////////////////////////////////////

void stm32_dma_transfer(BOOL receive, const BYTE *buff, UINT btr)
{
    DMA_InitTypeDef DMA_InitStructure;
    WORD rw_workbyte[] = { 0xffff };

    DMA_DeInit(SDCARD_SPI_RX_DMA_STREAM);
    DMA_DeInit(SDCARD_SPI_TX_DMA_STREAM);

    DMA_StructInit (&DMA_InitStructure);

    // shared DMA configuration values
    DMA_InitStructure.DMA_PeripheralBaseAddr = (DWORD)(&(SDCARD_SPI->DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_BufferSize         = btr;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;

    if (receive)
    {
        // DMA configuration SPI1 RX
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
        DMA_InitStructure.DMA_Channel         = SDCARD_SPI_RX_DMA_CHANNEL;
        DMA_InitStructure.DMA_DIR             = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_MemoryInc       = DMA_MemoryInc_Enable;

        DMA_Init(SDCARD_SPI_RX_DMA_STREAM, &DMA_InitStructure);

        // DMA configuration SPI1 TX
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
        DMA_InitStructure.DMA_Channel         = SDCARD_SPI_TX_DMA_CHANNEL;
        DMA_InitStructure.DMA_DIR             = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_MemoryInc       = DMA_MemoryInc_Disable;

        DMA_Init(SDCARD_SPI_TX_DMA_STREAM, &DMA_InitStructure);
    }
    else
    {
    #if _FS_READONLY == 0
        // DMA configuration SPI1 RX
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)rw_workbyte;
        DMA_InitStructure.DMA_Channel         = SDCARD_SPI_RX_DMA_CHANNEL;
        DMA_InitStructure.DMA_DIR             = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_MemoryInc       = DMA_MemoryInc_Disable;

        DMA_Init(SDCARD_SPI_RX_DMA_STREAM, &DMA_InitStructure);

        // DMA configuration SPI1 TX
        DMA_InitStructure.DMA_Memory0BaseAddr = (DWORD)buff;
        DMA_InitStructure.DMA_Channel         = SDCARD_SPI_TX_DMA_CHANNEL;
        DMA_InitStructure.DMA_DIR             = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_MemoryInc       = DMA_MemoryInc_Enable;

        DMA_Init(SDCARD_SPI_TX_DMA_STREAM, &DMA_InitStructure);
    #endif
    }

    // Enable DMA RX Channel
    DMA_Cmd(SDCARD_SPI_RX_DMA_STREAM, ENABLE);

    // Enable DMA TX Channel
    DMA_Cmd(SDCARD_SPI_TX_DMA_STREAM, ENABLE);

    // Enable SPI TX/RX request
    SPI_I2S_DMACmd(SDCARD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

    // Wait until DMA Receive Complete
    while (DMA_GetFlagStatus(SDCARD_SPI_RX_DMA_STREAM, SDCARD_SPI_RX_TC_FLAG) == RESET);

    // Disable DMA RX Channel
    DMA_Cmd(SDCARD_SPI_RX_DMA_STREAM, DISABLE);

    // Disable DMA TX Channel */
    DMA_Cmd(SDCARD_SPI_TX_DMA_STREAM, DISABLE);

    // Disable SPI RX/TX request
    SPI_I2S_DMACmd(SDCARD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
}
#endif /* STM32_SD_USE_DMA */

///////////////////////////////////////////////////////////////////////////////
// Power Control and interface-initialization
///////////////////////////////////////////////////////////////////////////////

void power_on(void)
{
    #ifdef STM32_SD_USE_DMA
        DMA_InitTypeDef DMA_InitStructure;
    #endif

    spiInit(SDCARD_SPI);

    #ifdef STM32_SD_USE_DMA
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

        // all new coded added below AKA
        // setup the DMA TX Channel/Stream
        DMA_DeInit(SDCARD_SPI_TX_DMA_STREAM);

        /*
        * Check if the DMA Stream is disabled before enabling it.
        * Note that this step is useful when the same Stream is used multiple times:
        * enabled, then disabled then re-enabled... In this case, the DMA Stream disable
        * will be effective only at the end of the ongoing data transfer and it will
        * not be possible to re-configure it before making sure that the Enable bit
        * has been cleared by hardware. If the Stream is used only once, this step might
        * be bypassed.
        */

	    while (DMA_GetCmdStatus(SDCARD_SPI_TX_DMA_STREAM) != DISABLE);

	    DMA_StructInit (&DMA_InitStructure);

    	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SDCARD_SPI->DR);
	    DMA_InitStructure.DMA_Channel            = SDCARD_SPI_TX_DMA_CHANNEL;
    	DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
	    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralInc      = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
	    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Enable;
	    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
	    DMA_InitStructure.DMA_BufferSize         = 1;

	    DMA_Init (SDCARD_SPI_TX_DMA_STREAM, &DMA_InitStructure);

	    // Enable dma tx request.
	    SPI_I2S_DMACmd (SDCARD_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

        // setup the DMA RX Channel/Stream
        DMA_DeInit(SDCARD_SPI_RX_DMA_STREAM);

        /*
        * Check if the DMA Stream is disabled before enabling it.
        * Note that this step is useful when the same Stream is used multiple times:
        * enabled, then disabled then re-enabled... In this case, the DMA Stream disable
        * will be effective only at the end of the ongoing data transfer and it will
        * not be possible to re-configure it before making sure that the Enable bit
        * has been cleared by hardware. If the Stream is used only once, this step might
        * be bypassed.
        */

	    while (DMA_GetCmdStatus(SDCARD_SPI_RX_DMA_STREAM) != DISABLE);

        DMA_StructInit (&DMA_InitStructure);

        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SDCARD_SPI->DR);
	    DMA_InitStructure.DMA_Channel            = SDCARD_SPI_RX_DMA_CHANNEL;
	    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralInc      = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
	    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Enable;
	    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
	    DMA_InitStructure.DMA_BufferSize         = 1;

	    DMA_Init (SDCARD_SPI_RX_DMA_STREAM, &DMA_InitStructure);

	    // Enable dma rx request.
	    SPI_I2S_DMACmd (SDCARD_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Power Off
///////////////////////////////////////////////////////////////////////////////

void power_off(void)
{
    if (!(Stat & STA_NOINIT))
    {
        ENABLE_SDCARD;
        wait_ready();
        release_spi();
    }

    spiDeInit(SDCARD_SPI);

    Stat |= STA_NOINIT;                               // Set STA_NOINIT
}

///////////////////////////////////////////////////////////////////////////////
// Receive a data packet from MMC
//   *buff:	Data buffer to store received data
//   btr:	Byte count (must be multiple of 4)
///////////////////////////////////////////////////////////////////////////////

BOOL rcvr_datablock(BYTE *buff, UINT btr)
{
    BYTE token;

    Timer1 = 10;

    do                                                // Wait for data packet in timeout of 100ms
    {
        token = spiTransfer(SDCARD_SPI, 0xFF);
    }
    while ((token == 0xFF) && Timer1);

    if (token != 0xFE)
        return FALSE;                                 // If not valid data token, return with error

    #ifdef STM32_SD_USE_DMA
        stm32_dma_transfer(TRUE, buff, btr);
    #else

        do                                            // Receive the data block into buffer
        {
             *(buff++) = spiTransfer(SDCARD_SPI, 0xFF);
             *(buff++) = spiTransfer(SDCARD_SPI, 0xFF);
             *(buff++) = spiTransfer(SDCARD_SPI, 0xFF);
             *(buff++) = spiTransfer(SDCARD_SPI, 0xFF);
        }
        while (btr -= 4);

    #endif /* STM32_SD_USE_DMA */

    spiTransfer(SDCARD_SPI, 0xFF);                    // Discard CRC
    spiTransfer(SDCARD_SPI, 0xFF);

    return TRUE;                                      // Return with success
}

#if _FS_READONLY == 0
///////////////////////////////////////////////////////////////////////////////
// Send a data packet to MMC
//   *buff:	512 byte data block to be transmitted
//   token:	Data/Stop token
///////////////////////////////////////////////////////////////////////////////

BOOL xmit_datablock(const BYTE *buff, BYTE token)
{
    BYTE resp;

    #ifndef STM32_SD_USE_DMA
        BYTE wc;
    #endif

    if (wait_ready() != 0xFF)
        return FALSE;

    spiTransfer(SDCARD_SPI, token);                   // transmit data token

    if (token != 0xFD)                                // Is data token
    {

    #ifdef STM32_SD_USE_DMA
        stm32_dma_transfer(FALSE, buff, 512);
    #else
        wc = 0;

        do                                            // transmit the 512 byte data block to MMC
        {
            spiTransfer(SDCARD_SPI, *buff++);
            spiTransfer(SDCARD_SPI, *buff++);
        }
        while (--wc);

    #endif /* STM32_SD_USE_DMA */

        spiTransfer(SDCARD_SPI, 0xFF);                // CRC (Dummy)
        spiTransfer(SDCARD_SPI, 0xFF);
        resp = spiTransfer(SDCARD_SPI, 0xFF);         // Receive data response

        if ((resp & 0x1F) != 0x05)                    // If not accepted, return with error
            return FALSE;
    }

    return TRUE;
}
#endif /* _FS_READONLY */

///////////////////////////////////////////////////////////////////////////////
// SEND A COMMAND PACKET (6 BYTES)
//   cmd: Command byte
//   arg: Argument
///////////////////////////////////////////////////////////////////////////////

BYTE send_cmd(BYTE cmd, DWORD arg)
{
    BYTE n, res;

    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);

        if (res > 1)
            return res;
    }

    DISABLE_SDCARD;
    ENABLE_SDCARD;

    if (cmd != CMD0)
    {
        if (wait_ready() != 0xFF)
        {
            return 0xFF;
        }
    }

    spiTransfer(SDCARD_SPI, cmd);				      // Send command index

    spiTransfer(SDCARD_SPI, (BYTE)(arg >> 24));		  // Send argument[31..24]
    spiTransfer(SDCARD_SPI, (BYTE)(arg >> 16));		  // Send argument[23..16]
    spiTransfer(SDCARD_SPI, (BYTE)(arg >> 8));	      // Send argument[15..8]
    spiTransfer(SDCARD_SPI, (BYTE)arg);				  // Send argument[7..0]

    n = 0x01;							              // Stop : Dummy CRC

    if (cmd == CMD0)
        n = 0x95;			                          // Valid CRC for CMD0(0)

    if (cmd == CMD8)
        n = 0x87;			                          // Valid CRC for CMD8(0x1AA)

    spiTransfer(SDCARD_SPI, n);						  // Send CRC

    if (cmd == CMD12)
        spiTransfer(SDCARD_SPI, 0xFF);		          // Skip a stuff byte when stop reading

    n = 10;

    do
        res = spiTransfer(SDCARD_SPI, 0xFF);

    while ((res & 0x80) && --n);

    return res;                                       // Return with the response value
}

///////////////////////////////////////////////////////////////////////////////
// INITIALIZE DISK DRIVE
//   drv: Physical drive number (0)
///////////////////////////////////////////////////////////////////////////////

DSTATUS disk_initialize(BYTE drv)
{
    BYTE n, cmd, ty, ocr[4];

    if (drv)                                          //Supports only single drive
        return STA_NOINIT;

    power_on();                                       // Initialize interface

    DISABLE_SDCARD;

    for (n = 10; n; n--)                              // 80 dummy clocks
        spiTransfer(SDCARD_SPI, 0xFF);

    ENABLE_SDCARD;

    ty = 0;

    if (send_cmd(CMD0, 0) == 1)                       // Enter Idle state
    {
        Timer1 = 100;                                 // Initialization timeout of 1000 milliseconds

        if (send_cmd(CMD8, 0x1AA) == 1)               // SDHC
        {
            for (n = 0; n < 4; n++)                   // Get trailing return value of R7 response
                ocr[n] = spiTransfer(SDCARD_SPI, 0xFF);

            if (ocr[2] == 0x01 && ocr[3] == 0xAA)     // The card can work at VDD range of 2.7 - 3.6 V
            {
                while (Timer1 && send_cmd(ACMD41, 1UL << 30));  // Wait for leaving idle state (ACMD41 with HCS bit)

                if (Timer1 && send_cmd(CMD58, 0) == 0)          // Check CCS bit in the OCR
                {
                    for (n = 0; n < 4; n++)
                        ocr[n] = spiTransfer(SDCARD_SPI, 0xFF);

                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else  							              // SDSC or MMC
        {
            if (send_cmd(ACMD41, 0) <= 1)
            {
                ty = CT_SD1;
                cmd = ACMD41;                         // SDSC
            }
            else
            {
                ty = CT_MMC;
                cmd = CMD1;	                          // MMC
            }

            while (Timer1 && send_cmd(cmd, 0));		  // Wait for leaving idle state

            if (!Timer1 || send_cmd(CMD16, 512) != 0) // Set R/W block length to 512
                ty = 0;
        }
    }


    release_spi();

    CardType = ty;

    if (ty)                                           // Initialization succeeded
    {
        Stat &= ~STA_NOINIT;                          // Clear STA_NOINIT
        setSPIdivisor(SDCARD_SPI, 4);                 // 10.5 MHz SPI clock
        cliPrintF("SD Card Initialized....\n\n");
    }
    else
    {
        power_off();                                  // Initialization failed
        evrPush(EVR_SDCard_Failed, 0);
    }

    return Stat;
}

///////////////////////////////////////////////////////////////////////////////
// GET DISK STATUS
//   drv: Physical drive number (0)
///////////////////////////////////////////////////////////////////////////////

DSTATUS disk_status(BYTE drv)
{
    if (drv)
        return STA_NOINIT;                            // Supports only single drive

    return Stat;
}

///////////////////////////////////////////////////////////////////////////////
// READ SECTOR(S)
//   drv:	 Physical drive number (0)
//   *buff:  Pointer to the data buffer to store read data
//   sector: Start sector number (LBA)
//   count:	 Sector count (1..255)
///////////////////////////////////////////////////////////////////////////////

DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
    if (drv || !count)
        return RES_PARERR;

    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    if (!(CardType & CT_BLOCK))                       // Convert to byte address if needed
        sector *= 512;

    if (count == 1)                                   // Single block read
    {
        if (send_cmd(CMD17, sector) == 0)             // READ_SINGLE_BLOCK
        {
            if (rcvr_datablock(buff, 512))
            {
                count = 0;
            }
        }
    }
    else                                              // Multiple block read
    {
        if (send_cmd(CMD18, sector) == 0)             // READ_MULTIPLE_BLOCK
        {
            do
            {
                if (!rcvr_datablock(buff, 512))
                {
                    break;
                }

                buff += 512;
            }
            while (--count);

            send_cmd(CMD12, 0);                       // STOP_TRANSMISSION
        }
    }

    release_spi();

    return count ? RES_ERROR : RES_OK;
}

#if _FS_READONLY == 0
///////////////////////////////////////////////////////////////////////////////
// WRITE SECTOR(S)
//   drv:	 Physical drive numbre (0)
//   *buff:	 Pointer to the data to be written
//   sector: Start sector number (LBA)
//   count:	 Sector count (1..255)
///////////////////////////////////////////////////////////////////////////////

DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{
    if (drv || !count)
        return RES_PARERR;

    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    if (Stat & STA_PROTECT)
        return RES_WRPRT;

    if (!(CardType & CT_BLOCK))                       // Convert to byte address if needed
        sector *= 512;

    if (count == 1)                                   // Single block write
    {
        if ((send_cmd(CMD24, sector) == 0)            // WRITE_BLOCK
                && xmit_datablock(buff, 0xFE))
            count = 0;
    }
    else                                              // Multiple block write
    {
        if (CardType & CT_SDC)
            send_cmd(ACMD23, count);

        if (send_cmd(CMD25, sector) == 0)             // WRITE_MULTIPLE_BLOCKS
        {
            do
            {
                if (!xmit_datablock(buff, 0xFC))
                    break;

                buff += 512;
            }
            while (--count);

            if (!xmit_datablock(0, 0xFD))             // STO_TRAN token
                count = 1;
        }
    }

    release_spi();

    return count ? RES_ERROR : RES_OK;
}
#endif //_FS_READONLY == 0

#if (_USE_IOCTL == 1)
///////////////////////////////////////////////////////////////////////////////
// GET CURRENT TIME INTO A DWORD VALUE
//   drv:	Physical drive number (0)
//   ctrl:	Control code
//   *buff:	Buffer to send/receive control data
///////////////////////////////////////////////////////////////////////////////

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
            case 0:		                              // Sub control code == 0 (POWER_OFF)
                power_off();		                  // Power off
                res = RES_OK;
                break;

            case 1:		                              // Sub control code == 1 (POWER_ON)
                power_on();				              // Power on
                res = RES_OK;
                break;

            case 2:		                              // Sub control code == 2 (POWER_GET)
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
            case CTRL_SYNC :		                  // Make sure that no pending write process
                ENABLE_SDCARD;

                if (wait_ready() == 0xFF)
                    res = RES_OK;

                break;

            case GET_SECTOR_COUNT :	                  // Get number of sectors on the disk (DWORD)
                if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
                {
                    if ((csd[0] >> 6) == 1)  	      // SDC version 2.00
                    {
                        csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                        *(DWORD *)buff = (DWORD)csize << 10;
                    }
                    else  					          // SDC version 1.XX or MMC
                    {
                        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                        csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                        *(DWORD *)buff = (DWORD)csize << (n - 9);
                    }

                    res = RES_OK;
                }

                break;

            case GET_SECTOR_SIZE :	                  // Get R/W sector size (WORD)
                *(WORD *)buff = 512;
                res = RES_OK;
                break;

            case GET_BLOCK_SIZE :	                  // Get erase block size in unit of sector (DWORD)
                if (CardType & CT_SD2)  	          // SDC version 2.00
                {
                    if (send_cmd(ACMD13, 0) == 0)     // Read SD status
                    {
                        spiTransfer(SDCARD_SPI, 0xFF);

                        if (rcvr_datablock(csd, 16))  // Read partial block
                        {
                            for (n = 64 - 16; n; n--) // Purge trailing data
                                spiTransfer(SDCARD_SPI, 0xFF);

                            *(DWORD *)buff = 16UL << (csd[10] >> 4);
                            res = RES_OK;
                        }
                    }
                }
                else  					              // SDC version 1.XX or MMC
                {
                    if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) // Read CSD
                    {
                        if (CardType & CT_SD1)  	  // SDC version 1.XX
                        {
                            *(DWORD *)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                        }
                        else  					      // MMC
                        {
                            *(DWORD *)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                        }

                        res = RES_OK;
                    }
                }

                break;

            case MMC_GET_TYPE :		                  // Get card type flags (1 byte)
                *ptr = CardType;
                res = RES_OK;
                break;

            case MMC_GET_CSD :		                  // Receive CSD as a data block (16 bytes)
                if (send_cmd(CMD9, 0) == 0 &&		  // READ_CSD
                    rcvr_datablock(ptr, 16))
                    res = RES_OK;

                break;

            case MMC_GET_CID :		                  // Receive CID as a data block (16 bytes)
                if (send_cmd(CMD10, 0) == 0	&&	      // READ_CID
                    rcvr_datablock(ptr, 16))
                    res = RES_OK;

                break;

            case MMC_GET_OCR :		                  // Receive OCR as an R3 resp (4 bytes)
                if (send_cmd(CMD58, 0) == 0)  	      // READ_OCR
                {
                    for (n = 4; n; n--)
                        *ptr++ = spiTransfer(SDCARD_SPI, 0xFF);

                    res = RES_OK;
                }

                break;

            case MMC_GET_SDSTAT :	                  // Receive SD status as a data block (64 bytes)
                if (send_cmd(ACMD13, 0) == 0)  	      // SD_STATUS
                {
                    spiTransfer(SDCARD_SPI, 0xFF);

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

///////////////////////////////////////////////////////////////////////////////
// GET CURRENT TIME INTO A DWORD VALUE
///////////////////////////////////////////////////////////////////////////////

DWORD get_fattime(void)
{
    DWORD res;

    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;

    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

    res = (((DWORD)  RTC_DateStructure.RTC_Year + 20) << 25)  // bit31:25		Year from 1980 (0..127)
          | ((DWORD) RTC_DateStructure.RTC_Month << 21)       // bit24:21		Month          (1..12)
          | ((DWORD) RTC_DateStructure.RTC_Date << 16)        // bit20:16		Day in month   (1..31)
          |   (WORD)(RTC_TimeStructure.RTC_Hours << 11)       // bit15:11		Hour           (0..23)
          |   (WORD)(RTC_TimeStructure.RTC_Minutes << 5)      // bit10:5		Minute         (0..59)
          |   (WORD)(RTC_TimeStructure.RTC_Seconds >> 1);     // bit4:0	    	Second / 2     (0..29)

    return res;
}

///////////////////////////////////////////////////////////////////////////////
// Count down timers
// This function must be called every 10ms
///////////////////////////////////////////////////////////////////////////////

void disk_timerproc(void)
{
    BYTE n;//, s;

    n = Timer1;

    if (n)
        Timer1 = --n;

    n = Timer2;

    if (n)
        Timer2 = --n;
}

///////////////////////////////////////////////////////////////////////////////


