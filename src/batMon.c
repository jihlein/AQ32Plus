/**
  \file       batMon.c
  \brief      Battery Monitoring.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t batteryNumCells = 3;

float   batteryVoltage;
float   batteryCurrent;

uint16_t batteryCurrentUsed;

typedef void (*batMonCB_t)(void);

typedef struct thresholds_t
  {
    float value;
    batMonCB_t func;
  } thresholds_t ;

void batMonLow();
void batMonVeryLow();
void batMonMaxLow();

static const thresholds_t thresholds[] =
  {
    { 3.6, batMonLow  },
    { 3.5, batMonLow  },
    { 3.4, batMonVeryLow },
    { 3.3, batMonMaxLow },
  };

enum
  {
  thresholdThreshold = 20, /* 2 second at 10Hz. */
  thresholdsNUM      = sizeof(thresholds) / sizeof(thresholds_t),
  };

/* Exp Filter = LPF time const = 0.1 sampletime */
static const float alpha = 1.0/( 1.0+0.1 );
static float v_bat_ave = 0.0;
static int thresholdCount[thresholdsNUM]; /* Will be inited to zero */

///////////////////////////////////////////////////////////////////////////////

void measureBattery(void)
{
    batteryVoltage = voltageMonitor() * VOLTS_PER_BIT * (eepromConfig.voltageMonitorScale) + eepromConfig.voltageMonitorBias;
}

///////////////////////////////////////////////////////////////////////////////

void batteryInit(void)
{
    measureBattery();

    if (eepromConfig.batteryCells == 0)
        batteryNumCells = batteryVoltage / 3;
    else
        batteryNumCells = eepromConfig.batteryCells;
}

///////////////////////////////////////////////////////////////////////////////
/*
  \brief  battery Monitor Tick function.
 */
void batMonTick()
  {
  float v;
  int i;

  measureBattery();
  v = batteryVoltage / (float)batteryNumCells;
  if (0.0 == v_bat_ave)
    v_bat_ave = v;

  if (v > 1.0 ) /* There is a battery connected */
    {
    v_bat_ave = alpha * v_bat_ave + (1.0-alpha) * v;

    for ( i = 0 ; i < thresholdsNUM; ++i )
      if (v_bat_ave < thresholds[i].value )
        {
        if ( thresholdCount[i] < thresholdThreshold )
          if ( ++thresholdCount[i] == thresholdThreshold )
            thresholds[i].func() ;
        }
      else if ( thresholdCount[i] > 0 )
          --thresholdCount[i] ;
    }
  }

///////////////////////////////////////////////////////////////////////////////

void batMonLow()
  {
  /* need to do slow beeping here, push back in telem to flash controler
   * lights, etc.
   */
  evrPush(EVR_BatLow, (int)(v_bat_ave*1000.0));
  }

///////////////////////////////////////////////////////////////////////////////

void batMonVeryLow()
  {
  /* need to do fast beeping here, push back in telem to flash controler
   * lights, etc.
   * User needs to decsend now ...
   */
  evrPush(EVR_BatVeryLow, (int)(v_bat_ave*1000.0));
  }

///////////////////////////////////////////////////////////////////////////////

void batMonMaxLow()
  {
  /* User isn't listening flyer needs to auto-descend now ....
   */
  evrPush(EVR_BatMaxLow, (int)(v_bat_ave*1000.0));

  // Maybe do something more interesting like auto-descent or hover-hold.
  // armed = false;
  }

///////////////////////////////////////////////////////////////////////////////
