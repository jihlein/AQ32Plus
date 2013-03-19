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
// GPS Variables
///////////////////////////////////////////////////////////////////////////////

// UBLOX binary message definitions
struct ublox_NAV_POSLLH  // 01 02 (28)
{
	uint32_t iTow;
    int32_t  lon;    // 1e-7 degrees
    int32_t  lat;    // 1e-7 degrees
    int32_t  height; // mm
    int32_t  hMSL;   // mm
    uint32_t hAcc;   // mm
    uint32_t vAcc;   // mm
};

struct ublox_NAV_STATUS  // 01 03 (16)
{
    uint32_t iTow;
    uint8_t  gpsFix;
    uint8_t  flags;
    uint8_t  fixStat;
    uint8_t  flags2;
    uint32_t ttfx;
    uint32_t msss;
};

struct ublox_NAV_DOP  // 01 04 (18)
{
	uint32_t iTow;
	uint16_t gDOP;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t vDOP;
	uint16_t hDOP;
	uint16_t nDOP;
	uint16_t eDOP;
};

struct ublox_NAV_SOL  // 01 06 (52)
{
	uint32_t iTow;
    int32_t  fTow;
    int16_t  week;
    uint8_t  gspFix;
    uint8_t  flags;
    int32_t  ecefX;
    int32_t  ecefY;
    int32_t  ecefZ;
    int32_t  pAcc;
    int32_t  ecefVX;
    int32_t  ecefVY;
    int32_t  ecefVZ;
    int32_t  sAcc;
    uint16_t pDOP;
    uint8_t  res1;
    uint8_t  numSV;
    uint32_t res2;
};

struct ublox_NAV_VELNED  // 01 18 (36)
{
	uint32_t iTow;
    int32_t  velN;    // cm/s
    int32_t  velE;    // cm/s
    int32_t  velD;    // cm/s
    uint32_t speed;   // cm/s
    uint32_t gSpeed;  // cm/s
    int32_t  heading; // dev 1e-5
    uint32_t sAcc;    // cm/s
    uint32_t cAcc;    // deg 1e-5
};

struct ublox_NAV_TIMEUTC  // 01 33 (20)
{
    uint32_t iTOW;  // mSec
    uint32_t tACC;  // nSec
    int32_t  nano;  // nSec
    uint16_t year;  // years
    uint8_t  month; // months
    uint8_t  day;   // days
    uint8_t  hour;  // hours
    uint8_t  min;   // minutes
    uint8_t  sec;   // seconds
    uint8_t  valid;
};

union ublox_message
{
    struct ublox_NAV_POSLLH  nav_posllh;
    struct ublox_NAV_STATUS  nav_status;
    struct ublox_NAV_DOP     nav_dop;
    struct ublox_NAV_VELNED  nav_velned;
    struct ublox_NAV_SOL     nav_sol;
    struct ublox_NAV_TIMEUTC nav_timeutc;
    unsigned char raw[52];
}   ubloxMessage;

uint8_t ubloxExpectedDataLength;
uint8_t ubloxDataLength;
uint8_t ubloxClass,ubloxId;
uint8_t ubloxCKA,ubloxCKB;

///////////////////////////////////////////////////////////////////////////////
// Decode Data Parse
///////////////////////////////////////////////////////////////////////////////

void ubloxParseData()
{
    if (ubloxClass == 1)         // NAV
    {
        if (ubloxId == 2)        // NAV:POSLLH
        {
            sensors.gpsLatitude  = (float)ubloxMessage.nav_posllh.lat    * 0.0000001f * D2R; // Radians;
            sensors.gpsLongitude = (float)ubloxMessage.nav_posllh.lon    * 0.0000001f * D2R; // Radians;
            sensors.gpsAltitude  = (float)ubloxMessage.nav_posllh.height * 0.01f;            // Meters
        }
        else if (ubloxId == 3)   // NAV:STATUS
        {
            switch (ubloxMessage.nav_status.gpsFix)
            {
                case 2:
                    sensors.gpsFix = FIX_2D;
                    break;

                case 3:
                    sensors.gpsFix = FIX_3D;
                    break;

                default:
                    sensors.gpsFix = FIX_NONE;
                    break;
            }
        }
        else if (ubloxId == 4)   // NAV:DOP
        {
		    sensors.gpsHdop    = (float)ubloxMessage.nav_dop.hDOP * 0.01f;
		}
		else if (ubloxId == 6)   // NAV:SOL
        {
            sensors.gpsNumSats = ubloxMessage.nav_sol.numSV;
        }
        else if (ubloxId == 18)  // NAV:VELNED
        {
            sensors.gpsGroundTrack = (float)ubloxMessage.nav_velned.heading * 0.01f * D2R;    // Radians
            sensors.gpsGroundSpeed = (float)ubloxMessage.nav_velned.gSpeed  * 0.01f;          // Meters/Sec
        }
        else if (ubloxId == 33)  // NAV:TIMEUTC
        {
			sensors.gpsTime = (float)(ubloxMessage.nav_timeutc.hour * 10000 +
			                          ubloxMessage.nav_timeutc.min  * 100   +
			                          ubloxMessage.nav_timeutc.sec        ) +
			                  (float)(ubloxMessage.nav_timeutc.nano) * 0.000000001f;

			sensors.gpsDate = ubloxMessage.nav_timeutc.day   * 10000 +
			                  ubloxMessage.nav_timeutc.month * 100   +
			                  ubloxMessage.nav_timeutc.year  - 2000;
		}
    }
}

///////////////////////////////////////////////////////////////////////////////
// Decode UBLOX Message
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeUbloxMsg(void)
{
    char     data;
    uint8_t  parsed = false;
    uint16_t i;
    uint16_t numberOfChars;

    numberOfChars = gpsNumCharsAvailable();

    for (i = 0; i < numberOfChars; i++)
    {
		data = gpsRead();

        switch (ubloxProcessDataState)
        {
            ///////////////////////////

            case WAIT_SYNC1:
                if (data == 0xb5)
                    ubloxProcessDataState = WAIT_SYNC2;

                break;

            ///////////////////////////

            case WAIT_SYNC2:
                if (data == 0x62)
                    ubloxProcessDataState = GET_CLASS;
                else
                    ubloxProcessDataState = WAIT_SYNC1;

                break;

            ///////////////////////////

            case GET_CLASS:
                ubloxClass            = data;
                ubloxCKA              = data;
                ubloxCKB              = data;
                ubloxProcessDataState = GET_ID;

                break;

            ///////////////////////////

            case GET_ID:
                ubloxId               = data;
                ubloxCKA             += data;
                ubloxCKB             += ubloxCKA;
                ubloxProcessDataState = GET_LL;

                break;

            ///////////////////////////

            case GET_LL:
                ubloxExpectedDataLength = data;
                ubloxCKA               += data;
                ubloxCKB               += ubloxCKA;
                ubloxProcessDataState   = GET_LH;

                break;

            ///////////////////////////

            case GET_LH:
                ubloxExpectedDataLength += data << 8;
                ubloxDataLength          = 0;
                ubloxCKA                += data;
                ubloxCKB                += ubloxCKA;

                if (ubloxExpectedDataLength <= sizeof(ubloxMessage))
                    ubloxProcessDataState = GET_DATA;
                else
                    // discard overlong message
                    ubloxProcessDataState = WAIT_SYNC1;

                break;

            ///////////////////////////

            case GET_DATA:
                ubloxCKA += data;
                ubloxCKB += ubloxCKA;

                // next will discard data if it exceeds our biggest known msg
                if (ubloxDataLength < sizeof(ubloxMessage))
                    ubloxMessage.raw[ubloxDataLength++] = data;

                if (ubloxDataLength >= ubloxExpectedDataLength)
                    ubloxProcessDataState = GET_CKA;

                break;

            ///////////////////////////

            case GET_CKA:
                if (ubloxCKA != data)
                    ubloxProcessDataState = WAIT_SYNC1;
	            else
                    ubloxProcessDataState = GET_CKB;

                break;

            ///////////////////////////

            case GET_CKB:
                if (ubloxCKB == data)
                {
                    parsed = 1;
                    ubloxParseData();
                }
			    else
			    {
					sensors.gpsLatitude    = GPS_INVALID_ANGLE;
					sensors.gpsLongitude   = GPS_INVALID_ANGLE;
					sensors.gpsAltitude	   = GPS_INVALID_ALTITUDE;
					sensors.gpsGroundSpeed = GPS_INVALID_SPEED;
					sensors.gpsGroundTrack = GPS_INVALID_ANGLE;
					sensors.gpsNumSats     = GPS_INVALID_SATS;
					sensors.gpsFix         = GPS_INVALID_FIX;
					sensors.gpsDate        = GPS_INVALID_DATE;
					sensors.gpsTime        = GPS_INVALID_TIME;
					sensors.gpsHdop        = GPS_INVALID_HDOP;
				}

                ubloxProcessDataState = WAIT_SYNC1;

                break;
        }
    }
    return parsed;
}

///////////////////////////////////////////////////////////////////////////////
