/**
  \file       crc.c
  \brief      Stdlib for STM32s CRC engine.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     The STM32Fs CRC engine uses the same polynomial as CRC32B (i.e.
              ethernet) but shifts in the oposite direction. __REVs are added
              to fix this, which shouldn't effect executino much as the CRC
              engine takes 4 clocks to execute.
*/

#include <inttypes.h>
#include "stm32f4xx.h"

#include "drv_crc.h"

/*
  \brief  Feed a uint32_t buffer to the STMs CRC engine. The buffer
          must have a size that is a multiple of 4 bytes.
  \param  start  start of buffer pointer.
  \param  end    past-end of buffer pointer.
 */
void crc32Feed(uint32_t* start, uint32_t* end)
  {
  while ( start < end )
    crc32Write(*start++);
  //assert(start==end);
  }

/*
  \brief  CRC32B a uint32_t buffer using the STMs CRC engine. The buffer
          must have a size that is a multiple of 4 bytes. The CRC engine
          is reset at the start, and the result is NOTed as expected for
          ethernet or pk CRCs.
  \param start  start of buffer pointer.
  \param end    past-end of buffer pointer.
 */
uint32_t crc32B(uint32_t* start, uint32_t* end)
  {
  crc32Reset();
  crc32Feed(start,end);
  return ~crc32Read();
  }