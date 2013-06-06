/**
  \file       crc.h
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

#ifndef LIBSTM32F_CRC_H
#define LIBSTM32F_CRC_H
#pragma once

typedef volatile uint32_t vuint32_t;

/*
  \brief    Reset the STM32s CRC Engine.
  \remark   4 NOPs are added to garrentee the reset is complete BEFORE data
            is pushed, because the reset does not seem to be treated as
            a dataregister write with the correct write stalling. In inlined
            code without the NOPs the first word was corrupted.
 */
static inline void crc32Reset()
  {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);

  CRC->CR = CRC_CR_RESET;
  __NOP(); // 4 Clocks to finish reset.
  __NOP();
  __NOP();
  __NOP();
  }

/*
  \brief    Write a 32bit word to the STMs CRC engine.
  \param x  The 32bit word to write.
 */
static inline void crc32Write(vuint32_t x)
  {
  CRC->DR = __RBIT(x);
  }

/*
  \brief   Read the STM32s CRC engine data register.
  \return  A 32 bit word read form the data register.
 */
static inline vuint32_t crc32Read ()
  {
  return __RBIT(CRC->DR);
  }

void crc32Feed(uint32_t* start, uint32_t* end);
uint32_t crc32B(uint32_t* start, uint32_t* end);

enum { crcCheckVal = 0x2144DF1C };

// LIBSTM32F_CRC_H
#endif