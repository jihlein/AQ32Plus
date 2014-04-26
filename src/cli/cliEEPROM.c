/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

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
// EEPROM CLI
///////////////////////////////////////////////////////////////////////////////

int min(int a, int b)
{
    return a < b ? a : b;
}

///////////////////////////////////////

int8_t parse_hex(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 0x0A;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 0x0A;
    return -1;
}

///////////////////////////////////////

void cliPrintEEPROM(eepromConfig_t *e)
{
    uint32_t old_crc = e->CRCAtEnd[0];
    enum { line_length = 32, len = sizeof(eepromConfig_t) };
    uint8_t *by = (uint8_t*)e;
    int i, j;

    e->CRCAtEnd[0] = crc32bEEPROM(e, false);

    if (e->CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)len / line_length); i++)
    {
        for (j = 0; j < min(line_length, len - line_length * i); j++)
            cliPortPrintF("%02X", by[i * line_length + j]);

        cliPortPrint("\n");
    }

    e->CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void eepromCLI()
{
    uint32_t c1, c2;

    uint8_t  eepromQuery = 'x';
    uint8_t  validQuery  = false;

    cliBusy = true;

    cliPortPrint("\nEntering EEPROM CLI....\n\n");

    while(true)
    {
        cliPortPrint("EEPROM CLI -> ");

        while ((cliPortAvailable() == false) && (validQuery == false));

        if (validQuery == false)
            eepromQuery = cliPortRead();

        cliPortPrint("\n");

        switch(eepromQuery)
        {
            // 'a' is the standard "print all the information" character
            case 'a': // config struct data
                c1 = eepromConfig.CRCAtEnd[0],

                zeroPIDintegralError();
                zeroPIDstates();

                c2 = crc32bEEPROM(&eepromConfig, false);

                cliPortPrintF("Config structure information:\n");
                cliPortPrintF("Version          : %d\n", eepromConfig.version );
                cliPortPrintF("Size             : %d\n", sizeof(eepromConfig) );
                cliPortPrintF("CRC on last read : %08X\n", c1 );
                cliPortPrintF("Current CRC      : %08X\n", c2 );

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current Config has not yet been saved.\n");

                cliPortPrintF("CRC Flags :\n");
                cliPortPrintF("  History Bad    : %s\n", eepromConfig.CRCFlags & CRC_HistoryBad ? "true" : "false" );
                validQuery = false;
                break;

            ///////////////////////////

            case 'c': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;
                // these change randomly when not in flight and can mistakenly
                // make one think that the in-memory eeprom struct has changed
                zeroPIDintegralError();
                zeroPIDstates();

                cliPortPrintF("\n");

                cliPrintEEPROM(&eepromConfig);

                cliPortPrintF("\n");

                if (crcCheckVal != crc32bEEPROM(&eepromConfig, true))
                {
                    cliPortPrint("NOTE: in-memory config CRC invalid; there have probably been changes to\n");
                    cliPortPrint("      eepromConfig since the last write to flash/eeprom.\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'H': // clear bad history flag
                cliPortPrintF("Clearing Bad History flag.\n");
                eepromConfig.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'C': // Read in from Console in hex.  Console -> RAM
                ;
                uint32_t sz = sizeof(eepromConfig);
                eepromConfig_t e;
                uint8_t *p = (uint8_t*)&e;
                uint8_t *end = (uint8_t*)(&e + 1);
                uint32_t t = millis();
                enum { Timeout = 100 }; // timeout is in ms
                int second_nibble = 0; // 0 or 1
                char c;
                uint32_t chars_encountered = 0;

                cliPortPrintF("Ready to read in config. Expecting %d (0x%03X) bytes as %d\n",
                    sz, sz, sz * 2);
                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\n");
                cliPortPrintF("Times out if no character is received for %dms\n", Timeout);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - t < Timeout) {}
                    t = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';
                    int8_t hex = parse_hex(c);
                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        chars_encountered++;
                    if (ignore)
                        continue;
                    if (hex == -1)
                        break;

                    *p |= second_nibble ? hex : hex << 4;
                    p += second_nibble;
                    second_nibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\n",
                        (p - (uint8_t*)&e) * 2 + second_nibble, sz * 2);
                }
                else if (p < end || second_nibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        chars_encountered, c, c);
                }
                else if (crcCheckVal != crc32bEEPROM(&e, true))
                {
                    cliPortPrintF("CRC mismatch! Not writing to in-memory config.\n");
                    cliPortPrintF("Here's what was received:\n\n");
                    cliPrintEEPROM(&e);
                }
                else
                {
                    // check to see if the newly received eeprom config
                    // actually differs from what's in-memory
                    zeroPIDintegralError();
                    zeroPIDstates();

                    int i;
                    for (i = 0; i < sz; i++)
                        if (((uint8_t*)&e)[i] != ((uint8_t*)&eepromConfig)[i])
                            break;

                    if (i == sz)
                    {
                        cliPortPrintF("NOTE: uploaded config was identical to in-memory config.\n");
                    }
                    else
                    {
                        eepromConfig = e;
                        cliPortPrintF("In-memory config updated!\n");
                        cliPortPrintF("NOTE: config not written to EEPROM; use 'W' to do so.\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something
                t = millis();
                while (millis() - t < Timeout)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'E': // Read in from EEPROM.  (EEPROM -> RAM)
                cliPortPrint("Re-reading EEPROM.\n");
                readEEPROM();
                validQuery = false;
                break;

            ///////////////////////////

            case 'x': // exit EEPROM CLI
                cliPortPrint("\nExiting EEPROM CLI....\n\n");
                cliBusy = false;
                return;
                break;

            ///////////////////////////

            case 'W':
            case 'e': // Write out to EEPROM. (RAM -> EEPROM)
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'f': // Write out to sdCard FILE. (RAM -> FILE)
                validQuery = false;
                break;

            ///////////////////////////

            case 'F': // Read in from sdCard FILE. (FILE -> RAM)
                validQuery = false;
                break;

            ///////////////////////////

            case 'V': // Reset EEPROM Parameters
                cliPortPrint( "\nEEPROM Parameters Reset....(not rebooting)\n" );
                checkFirstTime(true);
                validQuery = false;
            break;


            ///////////////////////////

            case '?':
            //                0         1         2         3         4         5         6         7
            //                01234567890123456789012345678901234567890123456789012345678901234567890123456789
                cliPortPrintF("\n");
                cliPortPrintF("'a' Display in-RAM config information\n");
                cliPortPrintF("'c' Write in-RAM -> Console (as Hex)      'C' Read Console (as Hex) -> in-RAM\n");
                cliPortPrintF("'e' Write in-RAM -> EEPROM                'E' Read EEPROM -> in-RAM\n");
                cliPortPrintF("'f' Write in-RAM -> sd FILE (Not yet imp) 'F' Read sd FILE -> in-RAM (Not imp)\n");
                cliPortPrintF("                                          'H' Clear CRC Bad History flag\n");
                cliPortPrintF("                                          'V' Reset in-RAM config to default.\n");
                cliPortPrintF("'x' Exit EEPROM CLI                       '?' Command Summary\n");
                cliPortPrintF("\n");
                cliPortPrintF("For compatability:                        'W' Write in-RAM -> EEPROM\n\n");
                break;

            ///////////////////////////
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
