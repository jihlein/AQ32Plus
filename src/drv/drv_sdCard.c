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
// SD Card Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define CMD0	(0x40 +  0)	// GO_IDLE_STATE
#define CMD1	(0x40 +  1)	// SEND_OP_COND (MMC)
#define ACMD41	(0xC0 + 41)	// SEND_OP_COND (SDC)
#define CMD8	(0x40 +  8)	// SEND_IF_COND
#define CMD9	(0x40 +  9)	// SEND_CSD
#define CMD10	(0x40 + 10)	// SEND_CID
#define CMD12	(0x40 + 12)	// STOP_TRANSMISSION
#define ACMD13	(0xC0 + 13)	// SD_STATUS (SDC)
#define CMD16	(0x40 + 16)	// SET_BLOCKLEN
#define CMD17	(0x40 + 17)	// READ_SINGLE_BLOCK
#define CMD18	(0x40 + 18)	// READ_MULTIPLE_BLOCK
#define CMD23	(0x40 + 23)	// SET_BLOCK_COUNT (MMC)
#define ACMD23	(0xC0 + 23)	// SET_WR_BLK_ERASE_COUNT (SDC)
#define CMD24	(0x40 + 24)	// WRITE_BLOCK
#define CMD25	(0x40 + 25)	// WRITE_MULTIPLE_BLOCK
#define CMD55	(0x40 + 55)	// APP_CMD
#define CMD58	(0x40 + 58)	// READ_OCR

///////////////////////////////////////

static uint8_t cardType = CT_UNKNOWN;

static volatile uint8_t timer1, timer2;

///////////////////////////////////////////////////////////////////////////////
// Wait for SD card
///////////////////////////////////////////////////////////////////////////////

uint8_t waitForSDCard(void)
{
    uint8_t result;

    timer2 = 50;                                                                   // Timeout of 500 mSec Waiting for Ready

    spiTransfer(SD_CARD_SPI, 0xFF);

    do
        result = spiTransfer(SD_CARD_SPI, 0xFF);

    while ((result != 0xFF) && timer2);

    return result;
}

///////////////////////////////////////////////////////////////////////////////
// Send CMD
///////////////////////////////////////////////////////////////////////////////

uint8_t sendCmd(uint8_t command, uint32_t commandArguement, uint8_t crc)
{
    uint8_t n;
    uint8_t result;

    if (command & 0x80)
    {
		command &= 0x7F;
		result = sendCmd(CMD55, 0x00000000, 0x00);

		if (result > 1)
		    return result;
	}

    if (command != CMD0)
    {
		if (waitForSDCard() != 0xFF)
		    return 0xFF;
	}

	spiTransfer(SD_CARD_SPI, command);                                             // Send command

    spiTransfer(SD_CARD_SPI, (uint8_t)(commandArguement >> 24));                   // Send argument[31..24]
    spiTransfer(SD_CARD_SPI, (uint8_t)(commandArguement >> 16));                   // Send argument[23..16]
    spiTransfer(SD_CARD_SPI, (uint8_t)(commandArguement >>  8));                   // Send argument[15.. 8]
    spiTransfer(SD_CARD_SPI, (uint8_t)(commandArguement      ));                   // Send argument[ 7.. 0]

    spiTransfer(SD_CARD_SPI, crc);                                                 // CRC

    if (command ==CMD12)                                                           // Skip a byte for stop transmission cmd
        spiTransfer(SD_CARD_SPI, 0xFF);

    n = 10;

    do result = spiTransfer(SD_CARD_SPI, 0xFF);

    while ((result & 0x80) && --n);

    return result;                                                                 // Return with the Response Value
}

///////////////////////////////////////////////////////////////////////////////
// SD Card Initialization
///////////////////////////////////////////////////////////////////////////////

uint8_t initSDCard(void)
{
    uint8_t cmd;
    uint8_t ocr[4];
    uint8_t pulseCount;

    ///////////////////////////////////

    DISABLE_SD_CARD;  // Make sure card is disabled

    for(pulseCount = 0; pulseCount < 10; pulseCount++)                             // Send 80 (74 Minimum) pulses to set SPI mode
        spiTransfer(SD_CARD_SPI, 0xFF);

    ///////////////////////////////////

    ENABLE_SD_CARD;

    cardType = CT_UNKNOWN;

    if (sendCmd(CMD0, 0x00000000, 0x95) == 0x01)                                   // Enter Idle State
    {
		timer1 = 100;                                                              // Initialization Timeout of 1000 mSec

		if (sendCmd(CMD8, 0x000001AA, 0x87) == 1)                                  // SDHC
		{
			ocr[3] = spiTransfer(SD_CARD_SPI, 0xFF);                               // Get Return Value of R7 Response
			ocr[2] = spiTransfer(SD_CARD_SPI, 0xFF);
			ocr[1] = spiTransfer(SD_CARD_SPI, 0xFF);
			ocr[0] = spiTransfer(SD_CARD_SPI, 0xFF);

			if ((ocr[1] == 0x01) && (ocr[0] == 0xAA))                              // Card can Work at VDD Range of 2.7 - 3.6 volts
			{
				while (timer1 && sendCmd(ACMD41, 0x40000000, 0x01));               // Wait to Leave Idle State (ACMD41 with HCS bit set)

				if (timer1 && sendCmd(CMD58, 0x00000000, 0x01) == 0)               // Check CC2 Bit in the OCR
				{
					ocr[3] = spiTransfer(SD_CARD_SPI, 0xFF);
					ocr[2] = spiTransfer(SD_CARD_SPI, 0xFF);
					ocr[1] = spiTransfer(SD_CARD_SPI, 0xFF);
			        ocr[0] = spiTransfer(SD_CARD_SPI, 0xFF);

			        cardType = (ocr[3] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		}
		else                                                                       // SDSC or MMC
		{
			if (sendCmd(ACMD41, 0x00000000, 0x01) <= 1)
			{
				cardType = CT_SD1;                                                 // SDSC
				cmd = ACMD41;
			}
			else
			{
				cardType = CT_MMC;                                                 // MMC
				cmd = CMD1;
			}

			while (timer1 && sendCmd(cmd, 0x00000000, 0x01));                      // Wait to Leave Idle State

			if (!timer1 || sendCmd(CMD16, 0x00000200, 0x01))                       // Set R/W Block Length to 512
			    cardType = CT_UNKNOWN;
		}
	}

	DISABLE_SD_CARD;

    spiTransfer(SD_CARD_SPI, 0xFF);

    if (cardType)                                                                  // Initialization Succeeded
        setSPIdivisor(SD_CARD_SPI, 4);                                             // 10.5 MHz SPI Clock
	else
	    evrPush(EVR_SDCard_Failed, 0);                                             // Initialization Failed

	return cardType;
}

///////////////////////////////////////////////////////////////////////////////
// SD Card Count down timers
///////////////////////////////////////////////////////////////////////////////

void sdCardCountDown(void)
{
    uint8_t n;

    n = timer1;

    if (n)
        timer1 = --n;

    n = timer2;

    if (n)
        timer2 = --n;
}

///////////////////////////////////////////////////////////////////////////////
