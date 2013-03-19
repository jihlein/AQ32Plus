/*
  February 2013

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
// GPS Defines
///////////////////////////////////////////////////////////////////////////////

#define SENTENCE_SIZE 80 // Maximum size of sentence (80 in NMEA)

///////////////////////////////////////////////////////////////////////////////
// GPS Variables
///////////////////////////////////////////////////////////////////////////////

static const char nib2hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char    sentenceBuffer[SENTENCE_SIZE + 1];
uint8_t sentenceLength = 0;
uint8_t sentenceCalculatedXOR;
uint8_t sentenceXOR;

///////////////////////////////////////////////////////////////////////////////
// Get Integer from NMEA
///////////////////////////////////////////////////////////////////////////////

// read a optionally decimal number from string
// - value returned is always integer as multiplied to include wanted decimal count
// - this will also consume the leading comma (required)
// - out can be NULL to ignore read value

uint8_t nmeaGetScaledInt(char **s, long *out, uint8_t decimals)
{
    uint8_t ret = 0;
    int32_t val = 0;

    // read whole numbers (prior to dot)
    while (((**s)>='0') && ((**s) <= '9'))
    {
        val *= 10;
        val = val + (*((*s)++) - '0');
        ret = 1;
    }

    if ((**s)=='.')
    {
        // we have decimals
        (*s)++;

        while  (decimals--)
        {
            val *= 10;
            if (((**s)>='0') && ((**s) <= '9'))
            {
                val += (*((*s)++) - '0');
                ret = 1;
            }
        }
    }
    else
    {
        while  (decimals--)
          val *= 10;
    }

    // take off the decimals we did not care about
    while (((**s)>='0') && ((**s) <= '9'))
        (*s)++;

    if ((**s) == ',')
        (*s)++;
    else
        ret = 0; // no comma -> fail

    if (ret && out)
        *out=val;

    return ret;
}

///////////////////////////////////////////////////////////////////////////////
// Get Lat/Long from NMEA
///////////////////////////////////////////////////////////////////////////////

// input: string "[d]ddmm.mmmm,[NESW],"

uint8_t nmeaGetLatLong(char **s, long *outp, uint8_t decimals)
{
    int32_t raw, deg;

    if (nmeaGetScaledInt(s, &raw, decimals))
    {
        deg = raw / 10000000 * 10000000; // whole degrees
        raw = raw - deg;                 // minutes
        raw = raw * 100 / 60;            // minutes to fractional degrees
        deg = deg + raw;

        switch (**s)
        {
            case 'S':
            case 'W':
                deg = -deg;
                (*s)++;
                break;

            case 'N':
            case 'E':
                (*s)++;
                break;

            default:
                break;
        }

        if ((**s) == ',')
        {
            (*s)++;
            if (outp) *outp = deg;
            return 1;
        }
    }

    if ((**s) == ',') (*s)++; // consume the second comma if no number parsed

    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Process NMEA Sentence
///////////////////////////////////////////////////////////////////////////////

void nmeaProcessSentence()
{
    char *p = sentenceBuffer;
    int32_t work;

    ///////////////////////////////////

    if (!strncmp(p,"GPGGA,",6))
    {
        // MediaTek Example: $GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65
        // Ublox Example   : $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B

        p += 6;  // Skip over 'GPGGA,'

        sensors.gpsTime        = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f             : GPS_INVALID_TIME;

        sensors.gpsLatitude    = (nmeaGetLatLong(&p,   &work, 5)) ? (float)work * 0.0000001f * D2R   : GPS_INVALID_ANGLE;
        sensors.gpsLongitude   = (nmeaGetLatLong(&p,   &work, 5)) ? (float)work * 0.0000001f * D2R   : GPS_INVALID_ANGLE;

        p += 2;  // Skip Quality (1 character and ',')  //nmeaGetScaledInt(&p, NULL,  0); // Position Fix Indicator - Not Used

        sensors.gpsNumSats     = (nmeaGetScaledInt(&p, &work, 0)) ? work                             : GPS_INVALID_SATS;
        sensors.gpsHdop        = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f             : GPS_INVALID_HDOP;
        sensors.gpsAltitude    = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f             : GPS_INVALID_ALTITUDE;
    }

    ///////////////////////////////////

    else if (!strncmp(p,"GPGSA,",6))
    {
        // MediaTek Example: $GPGSA,A,3,29,21,26,15,18,09,06,10,,,,,2.32,0.95,2.11*00
        // Ublox Example   : $GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54*0D

        p += 6;  // Skip over 'GPGSA,'

        p += 2;  // Skip opMode (1 character and ',')

        sensors.gpsFix         = (nmeaGetScaledInt(&p, &work, 0)) ? work                             : GPS_INVALID_FIX;
    }

    ///////////////////////////////////

    else if (!strncmp(p,"GPRMC,",6))
    {
        // MediaTek Example: $GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C
        // Ublox Example   : $GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*57

        p += 6;  // Skip over 'GPRMC,'

        sensors.gpsTime        = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f             : GPS_INVALID_TIME;

        p += 2;  // Skip Status (1 character and ',')

        sensors.gpsLatitude    = (nmeaGetLatLong(&p,   &work, 5)) ? (float)work * 0.0000001f * D2R   : GPS_INVALID_ANGLE;
        sensors.gpsLongitude   = (nmeaGetLatLong(&p,   &work, 5)) ? (float)work * 0.0000001f * D2R   : GPS_INVALID_ANGLE;
        sensors.gpsGroundSpeed = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f * KNOTS2MPS : GPS_INVALID_SPEED;
        sensors.gpsGroundTrack = (nmeaGetScaledInt(&p, &work, 3)) ? (float)work * 0.001f * D2R       : GPS_INVALID_ANGLE;
        sensors.gpsDate        = (nmeaGetScaledInt(&p, &work, 0)) ? work                             : GPS_INVALID_DATE;
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Decode a NMEA Sentence
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeNMEAsentence(void)
{
    char     data;
    uint8_t  parsed = false;
    uint16_t i;
    uint16_t numberOfChars;

    numberOfChars = gpsNumCharsAvailable();

    for (i = 0; i < numberOfChars; i++)
    {
		data = gpsRead();

        switch (nmeaProcessDataState)
        {
            case WAIT_START:
                if (data == '$')
                {
                    nmeaProcessDataState  = READ;
                    sentenceLength        = 0;
                    sentenceCalculatedXOR = 0;
                }
                break;

            case READ:
                if (data == '*')
                {
                    sentenceBuffer[sentenceLength] = 0; // ensure NUL at end
                    nmeaProcessDataState           = READ_CS1;
                }
                else if (sentenceLength < SENTENCE_SIZE)
                {
                    sentenceBuffer[sentenceLength++] = data;
                    sentenceCalculatedXOR ^= data;
                }
                else
                {
                    // overrun !!
                    nmeaProcessDataState = WAIT_START;
                }
                break;

            case READ_CS1:
                if (data == nib2hex[sentenceCalculatedXOR>>4])
                    nmeaProcessDataState = READ_CS2;
                else
                    nmeaProcessDataState = WAIT_START;

                break;

            case READ_CS2:
                if (data == nib2hex[sentenceCalculatedXOR & 0xf])
                {
                    parsed = true;
                    nmeaProcessSentence();
                }

                nmeaProcessDataState = WAIT_START;
                break;
        }

    }

    return parsed;
}

///////////////////////////////////////////////////////////////////////////////
