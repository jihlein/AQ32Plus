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

#define FLASH_WRITE_EEPROM_ADDR  0x08004000  // FLASH_Sector_1

const char rcChannelLetters[] = "AERT1234";

static uint8_t checkNewEEPROMConf = 19;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            eepromConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

uint32_t crc32bEEPROM(eepromConfig_t *e, int includeCRCAtEnd)
{
    return crc32B((uint32_t*)e, includeCRCAtEnd ? (uint32_t*)(e + 1) : e->CRCAtEnd);
}

///////////////////////////////////////////////////////////////////////////////

enum { eepromConfigNUMWORD =  sizeof(eepromConfig_t)/sizeof(uint32_t) };

void readEEPROM(void)
{
    eepromConfig_t *dst = &eepromConfig;

    *dst = *(eepromConfig_t*)FLASH_WRITE_EEPROM_ADDR ;

    if ( crcCheckVal != crc32bEEPROM(dst, true) )
    {
        evrPush(EVR_FlashCRCFail,0);
        dst->CRCFlags |= CRC_HistoryBad;
    }
    else if ( dst->CRCFlags & CRC_HistoryBad )
      evrPush(EVR_ConfigBadHistory,0);

    accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);

    eepromConfig.yawDirection = constrain(eepromConfig.yawDirection, -1.0f, 1.0f);
}

///////////////////////////////////////////////////////////////////////////////

int writeEEPROM(void)
{
    FLASH_Status status;
    int32_t i;

    eepromConfig_t *src = &eepromConfig;
    uint32_t       *dst = (uint32_t*)FLASH_WRITE_EEPROM_ADDR;

    // there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDintegralError();
    zeroPIDstates();

    if ( src->CRCFlags & CRC_HistoryBad )
        evrPush(EVR_ConfigBadHistory,0);

    src->CRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->CRCAtEnd);

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    status = FLASH_EraseSector(FLASH_Sector_1, VoltageRange_3);

    ///////////////////////////////////

    i = -1;

    while ( FLASH_COMPLETE == status && i++ < eepromConfigNUMWORD )
        status = FLASH_ProgramWord((uint32_t)&dst[i], ((uint32_t*)src)[i]);

    if ( FLASH_COMPLETE != status )
        evrPush( -1 == i ? EVR_FlashEraseFail : EVR_FlashProgramFail, status);

    ///////////////////////////////////

    FLASH_Lock();

    readEEPROM();

    return status;
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool eepromReset)
{
    uint8_t test_val;

    test_val = *(uint8_t *)FLASH_WRITE_EEPROM_ADDR;

    if (eepromReset || test_val != checkNewEEPROMConf)
    {
		// Default settings
        eepromConfig.version = checkNewEEPROMConf;

	    ///////////////////////////////

        eepromConfig.accelBiasMPU[XAXIS] = 0.0f;
        eepromConfig.accelBiasMPU[YAXIS] = 0.0f;
        eepromConfig.accelBiasMPU[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelScaleFactorMPU[XAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        eepromConfig.accelScaleFactorMPU[YAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        eepromConfig.accelScaleFactorMPU[ZAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)

	    ///////////////////////////////

        eepromConfig.accelTCBiasSlope[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelTCBiasIntercept[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
        eepromConfig.gyroTCBiasSlope[PITCH] = 0.0f;
        eepromConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.magBias[XAXIS] = 0.0f;      // Internal Mag Bias
	    eepromConfig.magBias[YAXIS] = 0.0f;      // Internal Mag Bias
	    eepromConfig.magBias[ZAXIS] = 0.0f;      // Internal Mag Bias

	    eepromConfig.magBias[XAXIS + 3] = 0.0f;  // External Mag Bias
	    eepromConfig.magBias[YAXIS + 3] = 0.0f;  // External Mag Bias
	    eepromConfig.magBias[ZAXIS + 3] = 0.0f;  // External Mag Bias

	    ///////////////////////////////

		eepromConfig.accelCutoff = 1.0f;

		///////////////////////////////

	    eepromConfig.KpAcc = 5.0f;    // proportional gain governs rate of convergence to accelerometer
	    eepromConfig.KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
	    eepromConfig.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	    eepromConfig.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    eepromConfig.compFilterA =  2.0f;
		eepromConfig.compFilterB =  1.0f;

	    ///////////////////////////////

	    eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;

	    ///////////////////////////////////

        eepromConfig.rollAndPitchRateScaling = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

	    eepromConfig.yawRateScaling          = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

	    eepromConfig.attitudeScaling         = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

	    eepromConfig.nDotEdotScaling         = 0.009f;                 // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        eepromConfig.hDotScaling             = 0.003f;                 // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////

	    eepromConfig.receiverType  = PARALLEL_PWM;
	    eepromConfig.spektrumChannels = 7;
	    eepromConfig.spektrumHires = 0;

	    parseRcChannels("TAER1234");

	    eepromConfig.escPwmRate   = 450;
        eepromConfig.servoPwmRate = 50;

        eepromConfig.mixerConfiguration = MIXERTYPE_TRI;
        eepromConfig.yawDirection = 1.0f;

        eepromConfig.triYawServoPwmRate = 50;
        eepromConfig.triYawServoMin     = 2000.0f;
        eepromConfig.triYawServoMid     = 3000.0f;
        eepromConfig.triYawServoMax     = 4000.0f;
        eepromConfig.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		eepromConfig.freeMixMotors        = 4;

		eepromConfig.freeMix[0][ROLL ]    =  1.0f;
		eepromConfig.freeMix[0][PITCH]    = -1.0f;
		eepromConfig.freeMix[0][YAW  ]    = -1.0f;

		eepromConfig.freeMix[1][ROLL ]    = -1.0f;
		eepromConfig.freeMix[1][PITCH]    = -1.0f;
		eepromConfig.freeMix[1][YAW  ]    =  1.0f;

		eepromConfig.freeMix[2][ROLL ]    = -1.0f;
		eepromConfig.freeMix[2][PITCH]    =  1.0f;
		eepromConfig.freeMix[2][YAW  ]    = -1.0f;

		eepromConfig.freeMix[3][ROLL ]    =  1.0f;
		eepromConfig.freeMix[3][PITCH]    =  1.0f;
		eepromConfig.freeMix[3][YAW  ]    =  1.0f;

		eepromConfig.freeMix[4][ROLL ]    =  0.0f;
		eepromConfig.freeMix[4][PITCH]    =  0.0f;
		eepromConfig.freeMix[4][YAW  ]    =  0.0f;

		eepromConfig.freeMix[5][ROLL ]    =  0.0f;
		eepromConfig.freeMix[5][PITCH]    =  0.0f;
        eepromConfig.freeMix[5][YAW  ]    =  0.0f;

        eepromConfig.midCommand   = 3000.0f;
        eepromConfig.minCheck     = (float)(MINCOMMAND + 200);
        eepromConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        eepromConfig.minThrottle  = (float)(MINCOMMAND + 200);
        eepromConfig.maxThrottle  = (float)(MAXCOMMAND);

        eepromConfig.PID[ROLL_RATE_PID].B               =   1.0f;
        eepromConfig.PID[ROLL_RATE_PID].P               = 250.0f;
        eepromConfig.PID[ROLL_RATE_PID].I               = 100.0f;
        eepromConfig.PID[ROLL_RATE_PID].D               =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        eepromConfig.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[ROLL_RATE_PID].type            =   OTHER;

        eepromConfig.PID[PITCH_RATE_PID].B              =   1.0f;
        eepromConfig.PID[PITCH_RATE_PID].P              = 250.0f;
        eepromConfig.PID[PITCH_RATE_PID].I              = 100.0f;
        eepromConfig.PID[PITCH_RATE_PID].D              =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        eepromConfig.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        eepromConfig.PID[PITCH_RATE_PID].type           =   OTHER;

        eepromConfig.PID[YAW_RATE_PID].B                =   1.0f;
        eepromConfig.PID[YAW_RATE_PID].P                = 350.0f;
        eepromConfig.PID[YAW_RATE_PID].I                = 100.0f;
        eepromConfig.PID[YAW_RATE_PID].D                =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].iTerm            =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        eepromConfig.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[YAW_RATE_PID].type             =   OTHER;

        eepromConfig.PID[ROLL_ATT_PID].B                =   1.0f;
        eepromConfig.PID[ROLL_ATT_PID].P                =   2.0f;
        eepromConfig.PID[ROLL_ATT_PID].I                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].D                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        eepromConfig.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[ROLL_ATT_PID].type             =   ANGULAR;

        eepromConfig.PID[PITCH_ATT_PID].B               =   1.0f;
        eepromConfig.PID[PITCH_ATT_PID].P               =   2.0f;
        eepromConfig.PID[PITCH_ATT_PID].I               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].D               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        eepromConfig.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[PITCH_ATT_PID].type            =   ANGULAR;

        eepromConfig.PID[HEADING_PID].B                 =   1.0f;
        eepromConfig.PID[HEADING_PID].P                 =   3.0f;
        eepromConfig.PID[HEADING_PID].I                 =   0.0f;
        eepromConfig.PID[HEADING_PID].D                 =   0.0f;
        eepromConfig.PID[HEADING_PID].iTerm             =   0.0f;
        eepromConfig.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        eepromConfig.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        eepromConfig.PID[HEADING_PID].lastDterm         =   0.0f;
        eepromConfig.PID[HEADING_PID].lastLastDterm     =   0.0f;
        eepromConfig.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        eepromConfig.PID[HEADING_PID].type              =   ANGULAR;

        eepromConfig.PID[NDOT_PID].B                    =   1.0f;
        eepromConfig.PID[NDOT_PID].P                    =   3.0f;
        eepromConfig.PID[NDOT_PID].I                    =   0.0f;
        eepromConfig.PID[NDOT_PID].D                    =   0.0f;
        eepromConfig.PID[NDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[NDOT_PID].windupGuard          =   0.5f;
        eepromConfig.PID[NDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[NDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[NDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[NDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[NDOT_PID].type                 =   OTHER;

        eepromConfig.PID[EDOT_PID].B                    =   1.0f;
        eepromConfig.PID[EDOT_PID].P                    =   3.0f;
        eepromConfig.PID[EDOT_PID].I                    =   0.0f;
        eepromConfig.PID[EDOT_PID].D                    =   0.0f;
        eepromConfig.PID[EDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[EDOT_PID].windupGuard          =   0.5f;
        eepromConfig.PID[EDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[EDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[EDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[EDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[EDOT_PID].type                 =   OTHER;

        eepromConfig.PID[HDOT_PID].B                    =   1.0f;
        eepromConfig.PID[HDOT_PID].P                    =   2.0f;
        eepromConfig.PID[HDOT_PID].I                    =   0.0f;
        eepromConfig.PID[HDOT_PID].D                    =   0.0f;
        eepromConfig.PID[HDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[HDOT_PID].windupGuard          =   5.0f;
        eepromConfig.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[HDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[HDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[HDOT_PID].type                 =   OTHER;

        eepromConfig.PID[N_PID].B                       =   1.0f;
        eepromConfig.PID[N_PID].P                       =   3.0f;
        eepromConfig.PID[N_PID].I                       =   0.0f;
        eepromConfig.PID[N_PID].D                       =   0.0f;
        eepromConfig.PID[N_PID].iTerm                   =   0.0f;
        eepromConfig.PID[N_PID].windupGuard             =   0.5f;
        eepromConfig.PID[N_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[N_PID].lastDterm               =   0.0f;
        eepromConfig.PID[N_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[N_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[N_PID].type                    =   OTHER;

        eepromConfig.PID[E_PID].B                       =   1.0f;
        eepromConfig.PID[E_PID].P                       =   3.0f;
        eepromConfig.PID[E_PID].I                       =   0.0f;
        eepromConfig.PID[E_PID].D                       =   0.0f;
        eepromConfig.PID[E_PID].iTerm                   =   0.0f;
        eepromConfig.PID[E_PID].windupGuard             =   0.5f;
        eepromConfig.PID[E_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[E_PID].lastDterm               =   0.0f;
        eepromConfig.PID[E_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[E_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[E_PID].type                    =   OTHER;

        eepromConfig.PID[H_PID].B                       =   1.0f;
        eepromConfig.PID[H_PID].P                       =   2.0f;
        eepromConfig.PID[H_PID].I                       =   0.0f;
        eepromConfig.PID[H_PID].D                       =   0.0f;
        eepromConfig.PID[H_PID].iTerm                   =   0.0f;
        eepromConfig.PID[H_PID].windupGuard             =   5.0f;
        eepromConfig.PID[H_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[H_PID].lastDterm               =   0.0f;
        eepromConfig.PID[H_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[H_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[H_PID].type                    =   OTHER;

        ///////////////////////////////

        eepromConfig.osdEnabled             =  true;
        eepromConfig.defaultVideoStandard   =  NTSC;
        eepromConfig.metricUnits            =  true;

        eepromConfig.osdDisplayAlt          =  true;
        eepromConfig.osdDisplayAltRow       =  12;
        eepromConfig.osdDisplayAltCol       =  1;
        eepromConfig.osdDisplayAltHoldState =  false;

        eepromConfig.osdDisplayAH           =  false; // DO NOT SET BOTH TO TRUE
        eepromConfig.osdDisplayAtt          =  true;

        eepromConfig.osdDisplayHdg          =  false;
        eepromConfig.osdDisplayHdgRow       =  1;
        eepromConfig.osdDisplayHdgCol       =  13;

        eepromConfig.osdDisplayHdgBar		=  true;
		eepromConfig.osdDisplayHdgBarRow	=  11;
		eepromConfig.osdDisplayHdgBarCol	=  10;

		eepromConfig.osdDisplayVoltage		=  true;
		eepromConfig.osdDisplayVoltageRow	=  1;
		eepromConfig.osdDisplayVoltageCol	=  1;

		eepromConfig.osdDisplayCurrent		=  true;
		eepromConfig.osdDisplayCurrentRow	=  1;
		eepromConfig.osdDisplayCurrentCol	=  8;

		eepromConfig.osdDisplayThrot		=  true;
		eepromConfig.osdDisplayThrotRow		=  12;
		eepromConfig.osdDisplayThrotCol		=  25;

		eepromConfig.osdDisplayTimer		=  true;
		eepromConfig.osdDisplayTimerRow		=  0;
		eepromConfig.osdDisplayTimerCol		=  23;

		eepromConfig.osdDisplayRSSI			=  true;
		eepromConfig.osdDisplayRSSIRow		=  1;
		eepromConfig.osdDisplayRSSICol		=  24;

        ///////////////////////////////

        // The default agl pin and scale factor are
        // for the MB1200 ultrasonic ranger.  The
        // MB1200 is connected to the RNG input (ADC1)
        // on the AQ32 V2 hardware and is supplied
        // with 3.3 volts.  If supplied with 5 volts,
        // the analog output of the MB1200 can exceed
        // 3.3 volts by 0.1 volts.  It is not known if
        // this will damage the analog input or not.

        eepromConfig.aglPin                 = 1;
		eepromConfig.aglScale               = 3.125f;  // mV to meters, 3.2 mV = 1 cm
        eepromConfig.aglBias                = 0.0f;

		// Current monitoring defaults to off.
		// The default scale factor of 27.322404
		// is for the 90 amp AttoPilot sensor. It
		// is a nominal value and slight adjustment
		// may be required.

		eepromConfig.currentMonitoring      = false;
		eepromConfig.currentMonitorPin      = 6;
		eepromConfig.currentMonitorScale    = 27.322404f;  // For 90 amp AttoPilot Sensor
		eepromConfig.currentMonitorBias     =  0.0f;

		eepromConfig.rssiPPM                = false;
	    eepromConfig.rssiPin		    	= 5;
        eepromConfig.rssiMax			    = 3450;
		eepromConfig.rssiMin		    	= 10;
		eepromConfig.rssiWarning		    = 25;

	    // The default voltage monitor pin and scale factor
	    // are for the AQ32 onboard resistor divider.  To use
	    // an external voltage monitor, the ADC pin and scale
	    // factor must be changed.  The onboard resistor
	    // divider requires a 7.666667 scale factor, the 90
	    // amp AttoPilot sensor requires a 15.701052 scale
	    // factor.  These are nominal values, and slight
	    // adjustment may be required.

	    eepromConfig.voltageMonitorPin      = 7;
	    eepromConfig.voltageMonitorScale    = 7.666667f;
	    eepromConfig.voltageMonitorBias     = 0.0f;

		eepromConfig.batteryCells           = 3;

		eepromConfig.batteryLow             = 3.30f;
        eepromConfig.batteryVeryLow         = 3.20f;
        eepromConfig.batteryMaxLow          = 3.10f;

        ///////////////////////////////

        eepromConfig.armCount               =  50;
        eepromConfig.disarmCount            =  0;

        ///////////////////////////////

        eepromConfig.activeTelemetry          = 0;
        eepromConfig.mavlinkEnabled           = false;

        eepromConfig.verticalVelocityHoldOnly = true;

        eepromConfig.externalHMC5883          = 0;
        eepromConfig.externalMS5611           = false;

        ///////////////////////////////

        writeEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
