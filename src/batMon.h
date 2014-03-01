/**
  \file       batMon.h
  \brief      Battery Monitoring.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for Focused Flight 32.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define BATTERY_LOW       0
#define BATTERY_VERY_LOW  1
#define BATTRY_MAX_LOW    2

///////////////////////////////////////

typedef void (*batMonCB_t)(void);

typedef struct thresholds_t
{
    float value;
    batMonCB_t func;
} thresholds_t ;

///////////////////////////////////////

extern thresholds_t thresholds[3];

extern uint8_t batMonLowWarning;
extern uint8_t batMonVeryLowWarning;

extern uint8_t batteryNumCells;

extern float   batteryVoltage;
extern float   batteryCurrent;

extern uint16_t batteryCurrentUsed;

///////////////////////////////////////////////////////////////////////////////

void batMonTick(void);

///////////////////////////////////////////////////////////////////////////////

void measureBattery(void);

///////////////////////////////////////////////////////////////////////////////

void batteryInit(void);

///////////////////////////////////////////////////////////////////////////////
