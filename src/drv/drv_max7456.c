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
#include "max7456font.h"

///////////////////////////////////////////////////////////////////////////////
// MAX7456 Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t charbuf[54]; // for NVM_read

uint8_t osdDisabled        = false;

uint16_t maxScreenSize     = 0;
uint16_t maxScreenRows     = 0;
uint8_t  enableDisplay     = 0;
uint8_t  enableDisplayVert = 0;
uint8_t  max7456Reset      = 0;
uint8_t  disableDisplay    = 0;

////////////////////////////////////////////////////////////////////////////////
// SPI Read MAX7456 Register
///////////////////////////////////////////////////////////////////////////////

uint8_t spiReadMax7456Register(uint8_t r)
{
    spiTransfer(MAX7456_SPI, r);
    return spiTransfer(MAX7456_SPI, 0x00);
}

///////////////////////////////////////////////////////////////////////////////
// SPI Write MAX7456 Register
///////////////////////////////////////////////////////////////////////////////

void spiWriteMax7456Register(uint8_t r, uint8_t d)
{
    spiTransfer(MAX7456_SPI, r);
    spiTransfer(MAX7456_SPI, d);
}

///////////////////////////////////////////////////////////////////////////////
// Hide OSD Display
///////////////////////////////////////////////////////////////////////////////

void hideOSD()
{
  if (!osdDisabled)
  {
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(VM0_REG, disableDisplay);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    osdDisabled = true;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Unhide OSD Display
///////////////////////////////////////////////////////////////////////////////

void unhideOSD()
{
  if (osdDisabled)
  {
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(VM0_REG, enableDisplay);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    osdDisabled = false;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Write Characters
///////////////////////////////////////////////////////////////////////////////

// Writes 'len' character address bytes to the display memory corresponding to row y, column x
// - uses autoincrement mode when writing more than one character
// - will wrap around to next row if 'len' is greater than the remaining cols in row y
// - buf=NULL or len>strlen(buf) can be used to write zeroes (clear)
// - flags: 0x01 blink, 0x02 invert (can be combined)

void writeMax7456Chars( const char* buf, uint8_t len, uint8_t flags, uint8_t y, uint8_t x)
{
    uint8_t  i;
    uint16_t offset = y * 30 + x;

    if (flags)
        unhideOSD(); // make sure OSD is visible in case of alarms etc.

    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    // 16bit transfer, transparent BG, autoincrement mode (if len!=1)
    spiWriteMax7456Register(DMM_REG, ((flags & 1) ? 0x10 : 0x00) | ((flags & 2) ? 0x08 : 0x00) | ((len != 1) ? 0x01 : 0x00));

    // send starting display memory address (position of text)
    spiWriteMax7456Register(DMAH_REG, offset >> 8 );
    spiWriteMax7456Register(DMAL_REG, offset & 0xFF );

    // write out data
    for (i = 0; i < len; i++)
        spiWriteMax7456Register(DMDI_REG, (!buf || strlen(buf) < i) ? 0 : buf[i]);


    // Send escape 11111111 to exit autoincrement mode
    if (len != 1)
        spiWriteMax7456Register(DMDI_REG, END_STRING);

    // finished writing

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);
}

///////////////////////////////////////////////////////////////////////////////
// Detect Video Standard
///////////////////////////////////////////////////////////////////////////////

void detectVideoStandard()
{
    // First set the default
    uint8_t pal = false;
    uint8_t stat;

    pal = eepromConfig.defaultVideoStandard;

    // if autodetect enabled modify the default if signal is present on either standard
    // otherwise default is preserved

    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    stat = spiReadMax7456Register(STAT_REG);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    if (stat & 0x01)
        pal = PAL;

    if (stat & 0x02)
        pal = NTSC;

    if (pal)
    {
        maxScreenSize     = 480;
        maxScreenRows     = 16;
        enableDisplay     = 0x48;
        enableDisplayVert = 0x4C;
        max7456Reset      = 0x42;
        disableDisplay    = 0x40;
    }
    else
    {
        maxScreenSize     = 390;
        maxScreenRows     = 13;
        enableDisplay     = 0x08;
        enableDisplayVert = 0x0C;
        max7456Reset      = 0x02;
        disableDisplay    = 0x00;
    }

    // Artificial Horizon Display Parameters

    reticleRow    =  maxScreenRows / 2;
    ahTopPixel    = (reticleRow - AH_DISPLAY_RECT_HEIGHT / 2) * 18;
    ahBottomPixel = (reticleRow + AH_DISPLAY_RECT_HEIGHT / 2) * 18;
    ahCenter      =  reticleRow * 18 + 10;

    // Attitude Display Parameters

    aiTopPixel    = (reticleRow - AI_DISPLAY_RECT_HEIGHT / 2) * 18;
    aiBottomPixel = (reticleRow + AI_DISPLAY_RECT_HEIGHT / 2) * 18;
    aiCenter      =  reticleRow * 18 + 10;
}

///////////////////////////////////////////////////////////////////////////////
// Initialize MAX7456
///////////////////////////////////////////////////////////////////////////////

void initMax7456()
{
    uint8_t i;

    if (eepromConfig.osdEnabled == false) return;

    detectVideoStandard();

    //Soft reset the MAX7456 - clear display memory
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(VM0_REG, max7456Reset);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    delay(500);

    //Set white level to 90% for all rows
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    for(i = 0; i < maxScreenRows; i++ )
        spiWriteMax7456Register(RB0_REG + i, WHITE_LEVEL_90 );

    //ensure device is enabled
    spiWriteMax7456Register(VM0_REG, enableDisplay);

    delay(100);

    //finished writing
    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);
}

//////////////////////////////////////////////////////////////////////////////
// Reset MAX7456
///////////////////////////////////////////////////////////////////////////////

void resetMax7456()
{
    uint8_t x;

    // force soft reset on Max7456
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(VM0_REG, max7456Reset);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    delay(500);

    // set all rows to same character white level, 90%
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    for (x = 0; x < maxScreenRows; x++)
        spiWriteMax7456Register(RB0_REG + x, WHITE_LEVEL_90);

    // make sure the Max7456 is enabled
    spiWriteMax7456Register(VM0_REG, enableDisplay);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);
}

///////////////////////////////////////////////////////////////////////////////
// Show MAX7456 Font
///////////////////////////////////////////////////////////////////////////////

void showMax7456Font(void) //show all chars on 24 wide grid
{
    uint8_t i, x;

    // clear the screen
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(DMM_REG, CLEAR_DISPLAY);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    delay(1); // clearing display takes 20uS so wait some...

    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    // disable display
    //spiWriteRegister(VM0_REG, DISABLE_DISPLAY);

    spiWriteMax7456Register(DMM_REG,  0x01);  // 16 bit trans w/o background, autoincrement
    spiWriteMax7456Register(DMAH_REG,    0);  // set start address high
    spiWriteMax7456Register(DMAL_REG,   33);  // set start address low (line 1 col 3 (0 based)

    // show all characters on screen (actually 0-254)
    for (x = 0; x < 255; x++)
    {
        spiWriteMax7456Register(DMDI_REG, x);

        if ((x%24)==23)
        {
            for (i = 0; i < 6; i++)
                spiWriteMax7456Register(DMDI_REG, 0);
		}
    }

    spiWriteMax7456Register(DMDI_REG, END_STRING);

    //spiWriteMax7456Register(VM0_REG, ENABLE_DISPLAY_VERT);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);
}

///////////////////////////////////////////////////////////////////////////////
// Wait for NVM
///////////////////////////////////////////////////////////////////////////////

void waitNVM()
{
    while (spiReadMax7456Register(STAT_REG) & STATUS_REG_NVR_BUSY) ;
}

///////////////////////////////////////////////////////////////////////////////
// Write NVM Character
///////////////////////////////////////////////////////////////////////////////

void writeNVMcharacter(uint8_t ch, const uint16_t index)
{
    uint8_t x;

    // disable display
    GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);

    spiWriteMax7456Register(VM0_REG, disableDisplay);

    spiWriteMax7456Register(CMAH_REG, ch);  // set start address high

    for(x = 0; x < NVM_RAM_SIZE; x++) // write out 54 (out of 64) bytes of character to shadow ram
    {
        spiWriteMax7456Register(CMAL_REG, x); // set start address low

        spiWriteMax7456Register(CMDI_REG, fontdata[index+x]);
    }

    // transfer a 54 bytes from shadow ram to NVM
    spiWriteMax7456Register(CMM_REG, WRITE_NVR);

    waitNVM(); // NVM should be busy around 12ms

    spiWriteMax7456Register(VM0_REG, enableDisplayVert);

    GPIO_SetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN);
}

///////////////////////////////////////////////////////////////////////////////
// Download MAX7456 Font Data
///////////////////////////////////////////////////////////////////////////////

void downloadMax7456Font(void)
{
    uint16_t ch;

    if (sizeof(fontdata)!=16384)
    {
        usbPrint("\nERROR: fontdata with invalid size, aborting!!!\n\n");
        return;
    }

    usbPrint("\nDownloading font to MAX7456 NVM, this may take a while...\n\n");

    for (ch = 0; ch < 256; ch++)
    {
        itoa(ch, numberString, 10);
        usbPrint(numberString);
        writeNVMcharacter(ch, 64*ch);
        delay(30);
        usbPrint(" Done\n");
    }

    // force soft reset on Max7456
    resetMax7456();

    showMax7456Font();

    usbPrint("\nDone with MAX7456 font download\n\n");
}

///////////////////////////////////////////////////////////////////////////////
