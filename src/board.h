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

#pragma once

///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"

#include "arm_math.h"

#include "usbd_cdc_core.h"
#include "usbd_cdc.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "mavlink.h"

/////////////////////////////////////////////////////////////////////////////

#include "pid.h"

#include "aq32Plus.h"

#include "drv_adc.h"
#include "drv_crc.h"
#include "drv_i2c.h"
#include "drv_led.h"
#include "drv_pwmEsc.h"
#include "drv_pwmServo.h"
#include "drv_rx.h"
#include "drv_sdCard.h"
#include "drv_spektrum.h"
#include "drv_spi.h"
#include "drv_system.h"
#include "drv_timingFunctions.h"
#include "drv_uart1.h"
#include "drv_uart2.h"
#include "drv_uart3.h"
#include "drv_usb.h"

#include "hmc5883.h"
#include "mpu6000.h"
#include "ms5611_I2C.h"

#include "accelCalibrationMPU.h"
#include "batMon.h"
#include "cli.h"
#include "computeAxisCommands.h"
#include "config.h"
#include "coordinateTransforms.h"
#include "escCalibration.h"
#include "evr.h"
#include "firstOrderFilter.h"
#include "flightCommand.h"
#include "geoMagElements.h"
#include "GeomagnetismHeader.h"
#include "gps.h"
#include "MargAHRS.h"
#include "magCalibration.h"
#include "mavlinkStrings.h"
#include "max7456.h"
#include "mixer.h"
#include "mpu6000Calibration.h"
#include "osdWidgets.h"
#include "rssi.h"
#include "utilities.h"
#include "vertCompFilter.h"
#include "watchdogs.h"

///////////////////////////////////////////////////////////////////////////////
