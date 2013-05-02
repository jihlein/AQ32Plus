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

// This driver is based on code from:

/*
 * (c) Domen Puncer, Visionect, d.o.o.
 * BSD License
 *
 * v0.2 add support for SDHC
 */

 ///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

uint8_t crc7_one(uint8_t t, uint8_t data)
{
	uint8_t i;
	const uint8_t g = 0x89;

	t ^= data;

	for (i = 0; i < 8; i++)
	{
		if (t & 0x80)
			t ^= g;

		t <<= 1;
	}

	return t;
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void sd_cmd(uint8_t cmd, uint32_t arg)
{
	uint8_t crc = 0;

	spiTransfer(SDCARD_SPI, 0x40 | cmd);
	crc = crc7_one(crc, 0x40 | cmd);

	spiTransfer(SDCARD_SPI, arg >> 24);
	crc = crc7_one(crc, arg >> 24);

	spiTransfer(SDCARD_SPI, arg >> 16);
	crc = crc7_one(crc, arg >> 16);

	spiTransfer(SDCARD_SPI, arg >> 8);
	crc = crc7_one(crc, arg >> 8);

	spiTransfer(SDCARD_SPI, arg);
	crc = crc7_one(crc, arg);

	//spiTransfer(SDCARD_SPI, 0x95);	/* crc7, for cmd0 */

	spiTransfer(SDCARD_SPI, crc | 0x1);	/* crc7, for cmd0 */
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

uint8_t sd_get_r1()
{
	uint16_t tries = 1000;
	uint8_t  r;

	while (tries--)
	{
		r = spiTransfer(SDCARD_SPI, 0xF);

		if ((r & 0x80) == 0)
			return r;
	}

	return 0xFF;
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

/* Nec (=Ncr? which is limited to [0,8]) dummy bytes before lowering CS,
 * as described in sandisk doc, 5.4. */

void sd_nec()
{
	uint8_t i;

	for (i = 0; i < 8; i++)
		spiTransfer(SDCARD_SPI, 0xFF);
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

static const char *r1_strings[7] =
{
	"in idle",
	"erase reset",
	"illegal command",
	"communication crc error",
	"erase sequence error",
	"address error",
	"parameter error"
};

void print_r1(uint8_t r)
{
	uint8_t i;

	cliPrintF("R1: %02x\n", r);

	for (i = 0; i < 7; i++)
		if (r & (1<<i))
			cliPrintF("  %s\n", r1_strings[i]);
}

///////////////////////////////////////////////////////////////////////////////
// Initialize SD Card
///////////////////////////////////////////////////////////////////////////////

void initSDcard()
{
	uint8_t i, r;

	// Force SD Card into SPI mode

	// setSPIdivisor(SDCARD_SPI, 128);  // 328.125 kHz SPI Clock  // This done in SPI Initialization

	// DISABLE_SDCARD;                                            // This done in SPI Initialization

	for (i = 0; i < 10; i++)
		spiTransfer(SDCARD_SPI, 0xFF);

    // Reset

	ENABLE_SDCARD;

	sd_cmd(0, 0);
	r = sd_get_r1();
	sd_nec();

	DISABLE_SDCARD;

	if (r == 0xFF)
		goto err_spi;

	if (r != 0x01)
	{
		cliPrintF("failure\n");
		print_r1(r);
		goto err;
	}

	cliPrintF("sd init success\n");

	setSPIdivisor(SDCARD_SPI, 2);  // 21 MHz SPI clock

	return;

	err_spi:
	    cliPrintF("sd spi failure\n");
		return;

	err:
	    cliPrintF("sd card failure\n");
	    return;
}

//////////////////////////////////////////////////////////////////////////////
