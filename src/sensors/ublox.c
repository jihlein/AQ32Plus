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
// GPS Initialization Variables
///////////////////////////////////////////////////////////////////////////////

void     (*gpsPortClearBuffer)(void);

uint16_t (*gpsPortNumCharsAvailable)(void);

void     (*gpsPortPrintBinary)(uint8_t *buf, uint16_t length);

uint8_t  (*gpsPortRead)(void);

///////////////////////////////////////////////////////////////////////////////

uint32_t initBaudRates[5] = {9600,19200,38400,57600,115200};

///////////////////////////////////////

enum ubloxState { WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState;

///////////////////////////////////////

static uint8_t ubloxPortConfig38p4[] = {0xB5,0x62,            // Header             Setup UBLOX Comm Port
                                        0x06,0x00,            // ID
                                        0x14,0x00,            // Length
                                        0x01,                 // Port ID
                                        0x00,                 // Reserved 0
                                        0x00,0x00,            // TX Ready
                                        0xD0,0x08,0x00,0x00,  // Mode
                                        0x00,0x96,0x00,0x00,  // Baud Rate
                                        0x03,0x00,            // In Proto Mask
                                        0x01,0x00,            // Out Proto Mask
                                        0x00,0x00,            // Reserved 4
                                        0x00,0x00,            // Reserved 5
                                        0x8D,0x64};           // CK_A, CK_B

///////////////////////////////////////

static uint8_t ubloxInitData[] = {0xB5,0x62,            // Header             Turn Off NMEA GGA Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x00,                 // Msg ID       GGA
                                  0x00,                 // Rate         0
                                  0xFA,0x0F,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GLL Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x01,                 // Msg ID       GLL
                                  0x00,                 // Rate         0
                                  0xFB,0x11,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GSA Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x02,                 // Msg ID       GSA
                                  0x00,                 // Rate         0
                                  0xFC,0x13,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GSV Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x03,                 // Msg ID       GSV
                                  0x00,                 // Rate         0
                                  0xFD,0x15,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA RMC Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x04,                 // Msg ID       RMC
                                  0x00,                 // Rate         0
                                  0xFE,0x17,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA VTG Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x05,                 // Msg ID       VTG
                                  0x00,                 // Rate         0
                                  0xFF,0x19,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-POSLLH Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x02,                 // Msg ID       POSLLH
                                  0x01,                 // Rate         1
                                  0x0E,0x47,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-STATUS Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x03,                 // Msg ID       STATUS
                                  0x01,                 // Rate         1
                                  0x0F,0x49,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-DOP Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x04,                 // Msg ID       DOP
                                  0x01,                 // Rate         1
                                  0x10,0x4B,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-SOL Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x06,                 // Msg ID       SOL
                                  0x01,                 // Rate         1
                                  0x12,0x4F,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-VELNED Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x12,                 // Msg ID       VELNED
                                  0x01,                 // Rate         1
                                  0x1E,0x67,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-TIMEUTC Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x21,                 // Msg ID       TIMEUTC
                                  0x01,                 // Rate         1
                                  0x2D,0x85,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-SVINFO Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x30,                 // Msg ID       SVINFO
                                  0x05,                 // Rate         5
                                  0x40,0xA7,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Setup SBAS Mode
                                  0x06,0x16,            // ID
                                  0x08,0x00,            // Length
                                  0x01,                 // Mode
                                  0x07,                 // Usage
                                  0x03,                 // Max SBAS
                                  0x00,                 // Scan Mode 2
                                  0x04,0xE0,0x04,0x00,  // Scan Mode 1
                                  0x17,0x8D,            // CK_A, CK_B

                                  #ifdef AIRBORNE
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x07,                 // DynModel             Airborne with < 2g Acceleration
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x56,0x75,            // CK_A, CK_B
                                  #endif

                                  #ifdef PEDESTRIAN
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x03,                 // DynModel             Pedestrian
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x52,0xED,            // CK_A, CK_B
                                  #endif

                                  #ifdef PORTABLE
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x00,                 // DynModel             Portable
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x4F,0x87,            // CK_A, CK_B
                                  #endif

                                  0xB5,0x62,            // Header             Setup Meaurement Rates, Clock Reference
                                  0x06,0x08,            // ID
                                  0x06,0x00,            // Length
                                  0xC8,0x00,            // Measurement Rate     200 mSec (5 Hz)
                                  0x01,0x00,            // Navigation Rate      1 Measurement Cycle
                                  0x00,0x00,            // Time Reference       UTM Time
                                  0xDD,0x68};           // CK_A, CK_B

///////////////////////////////////////////////////////////////////////////////
// Initialize UBLOX Receiver
///////////////////////////////////////////////////////////////////////////////

void initUBLOX(void)
{
    uint8_t i;

    USART_InitTypeDef USART_InitStructure;

    ///////////////////////////////////

    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    ///////////////////////////////////

    // Going to send the GPS port configuration command at 5 different baud rates,
    // 9600, 19200, 38400, 57600, 115200.  If GPS is not at one of these rates,
    // port configuration will fail and communication between flight control board
    // and GPS board will not be possible.  GPS configuration command sets 38400
    // baud.  NEO-6M defaults to 9600 baud if config pins are left floating.

    for (i = 0; i < (sizeof(initBaudRates) / sizeof(initBaudRates[0])); i++)
    {
    	USART_InitStructure.USART_BaudRate = initBaudRates[i];

    	USART_Init(USART2, &USART_InitStructure);

    	gpsPortPrintBinary(ubloxPortConfig38p4, sizeof(ubloxPortConfig38p4));

    	delay(50);  // Delay so DMA buffer can be completely sent before trying next baud rate
    }

	USART_InitStructure.USART_BaudRate = 38400;

	USART_Init(USART2, &USART_InitStructure);

    //////////////////////////////////

    gpsPortPrintBinary(ubloxInitData, sizeof(ubloxInitData));  // Send UBLOX Initialization Data

    ///////////////////////////////////

    ubloxProcessDataState = WAIT_SYNC1;

    gpsPortClearBuffer();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Decoding Variables
///////////////////////////////////////////////////////////////////////////////

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
    int32_t  heading; // deg 1e-5
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

struct ublox_NAV_SVINFO  // 01 48 (608, 8 + 12 * numCh, numCh = 50)
{
	uint32_t iTOW;        // mSec
	uint8_t  numCh;       // number of channels
	uint8_t  globalFlags; // bitmask
	uint16_t reserved2;   // reserved
    uint8_t  svinfo[600];
};

union ublox_message
{
    struct ublox_NAV_POSLLH  nav_posllh;
    struct ublox_NAV_STATUS  nav_status;
    struct ublox_NAV_DOP     nav_dop;
    struct ublox_NAV_VELNED  nav_velned;
    struct ublox_NAV_SOL     nav_sol;
    struct ublox_NAV_TIMEUTC nav_timeutc;
    struct ublox_NAV_SVINFO  nav_svinfo;
    unsigned char raw[608];
}   ubloxMessage;

uint16_t ubloxExpectedDataLength;
uint16_t ubloxDataLength;
uint8_t  ubloxClass,ubloxId;
uint8_t  ubloxCKA,ubloxCKB;

///////////////////////////////////////////////////////////////////////////////
// UBLOX Parse Data
///////////////////////////////////////////////////////////////////////////////

void ubloxParseData(void)
{
    uint8_t n;

    if (ubloxClass == 1)         // NAV
    {
        ///////////////////////////////

    	if (ubloxId == 2)        // NAV:POSLLH
        {
        	gps.latitude  = ubloxMessage.nav_posllh.lat;
        	gps.longitude = ubloxMessage.nav_posllh.lon;
        	gps.height    = ubloxMessage.nav_posllh.height;
        	gps.hMSL      = ubloxMessage.nav_posllh.hMSL;
        }

    	///////////////////////////////

    	else if (ubloxId == 3)   // NAV:STATUS
        {
            gps.fix         = ubloxMessage.nav_status.gpsFix;
            gps.statusFlags = ubloxMessage.nav_status.flags;
        }

    	///////////////////////////////

    	else if (ubloxId == 4)   // NAV:DOP
        {
        	gps.hDop = ubloxMessage.nav_dop.hDOP;
        	gps.vDop = ubloxMessage.nav_dop.vDOP;
		}

    	///////////////////////////////

    	else if (ubloxId == 6)   // NAV:SOL
        {
			gps.numSats = ubloxMessage.nav_sol.numSV;
        }

    	///////////////////////////////

    	else if (ubloxId == 18)  // NAV:VELNED
        {
    		gps.velN    = ubloxMessage.nav_velned.velN;
    		gps.velE    = ubloxMessage.nav_velned.velE;
    	    gps.velD    = ubloxMessage.nav_velned.velD;
    	    gps.speed   = ubloxMessage.nav_velned.speed;
    	    gps.gSpeed  = ubloxMessage.nav_velned.gSpeed;
    		gps.heading = ubloxMessage.nav_velned.heading;
        }

    	///////////////////////////////

    	else if (ubloxId == 33)  // NAV:TIMEUTC
        {
        	gps.iTOW  = ubloxMessage.nav_timeutc.iTOW;
        	gps.year  = ubloxMessage.nav_timeutc.year;
			gps.month = ubloxMessage.nav_timeutc.month;
            gps.day   = ubloxMessage.nav_timeutc.day;
		}

    	///////////////////////////////

    	else if (ubloxId == 48)  // NAV:SVINFO
    	{
			gps.numCh = ubloxMessage.nav_svinfo.numCh;

			for (n = 0; n < gps.numCh; n++)
			{
				gps.chn[n]  = ubloxMessage.nav_svinfo.svinfo[0 + 12 * n];
				gps.svid[n] = ubloxMessage.nav_svinfo.svinfo[1 + 12 * n];
				gps.cno[n]  = ubloxMessage.nav_svinfo.svinfo[4 + 12 * n];
			}
		}

		///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
// Decode UBLOX Message
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeUbloxMsg(void)
{
    uint8_t  data;
    uint8_t  parsed = false;
    uint16_t i;
    uint16_t numberOfChars;

    numberOfChars = gpsPortNumCharsAvailable();

    for (i = 0; i < numberOfChars; i++)
    {
		data = gpsPortRead();

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

                ubloxProcessDataState = WAIT_SYNC1;

                break;
        }
    }
    return parsed;
}

///////////////////////////////////////////////////////////////////////////////
