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

static const uint8_t ubx5Hz[14] = {0xb5,0x62,0x06,0x08,0x06,0x00,0xc8,0x00,0x01,0x00,0x01,0x00,0xde,0x6a};  // 06 08 (6)

///////////////////////////////////////////////////////////////////////////////
// Initialize GPS Receiver
///////////////////////////////////////////////////////////////////////////////

void initGPS(void)
{
    uint8_t i;

    switch(eepromConfig.gpsType)
    {
		///////////////////////////////

		case NO_GPS:
		    break;

		///////////////////////////////

		case MEDIATEK_3329_BINARY:     // MediaTek 3329 in binary mode
		    gpsPrint("$PGCMD,16,0,0,0,0,0*6A\r\n");  // Set Binary Output
            gpsPrint("$PMTK220,200*2C\r\n");         // Set 5 Hz Output
            gpsPrint("$PMTK313,1*2E\r\n");           // Set SBAS On - Not sure if this does anything on MTK16 software
            gpsPrint("$PMTK301,2*2E\r\n");           // Set WAAS On - Not sure if this does anything on MTK16 software
            gpsPrint("$PMTK397,0*23\r\n");           // Set Nav Speed Threshold to 0

            mtk19ProcessDataState = MTK19_WAIT_SYNC1;
            break;

        ///////////////////////////////

        case MEDIATEK_3329_NMEA:       // MediaTek 3329 in NMEA mode
            gpsPrint("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");  // GPRMC, GPGGA and GPGSA
			gpsPrint("$PMTK220,200*2C\r\n");                                    // Set 5 Hz Update Rate
			gpsPrint("$PMTK313,1*2E\r\n");                                      // Set SBAS On
            gpsPrint("$PMTK301,2*2E\r\n");                                      // Set WAAS On
            gpsPrint("$PMTK397,0*23\r\n");                                      // Set Nav Speed Threshold to 0

            nmeaProcessDataState = WAIT_START;
            break;

        ///////////////////////////////

        case UBLOX:             // UBLOX in binary mode
        	for (i = 0; i < sizeof(ubx5Hz); i++)                // Set 5 Hz Update Rate
        	   gpsWrite(ubx5Hz[i]);

        	gpsPrint("$PUBX,41,1,0003,0001,38400,0*26\r\n");  // Set Binary Output

        	ubloxProcessDataState = WAIT_SYNC1;
        	break;

        ///////////////////////////////
	}

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

	gpsClearBuffer();
}

///////////////////////////////////////////////////////////////////////////////
