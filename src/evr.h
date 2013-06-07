/**
  \file       evr.h
  \brief      EVR (EVent Report).
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

typedef struct
  {
  uint32_t time;   /*!< Time of event */
  uint16_t evr;    /*! The event to report */
  uint16_t reason; /*!< optional event parameter (reason) if any */
  } evr_t;

typedef void (*evrListener_fp)(evr_t); /*!< prototype for listener functions */
typedef const char* constStrArr_t[];

typedef struct evrStringTable_t
  {
  const char*   severity;
  const char**  msgs;
  uint32_t      len;
  } evrStringTable_t;

enum { evrTypesNUM = 4 } ;

extern const evrStringTable_t evrStringTable[evrTypesNUM];

void evrPush(uint16_t evr, uint16_t reason);
int evrRegisterListener(evrListener_fp f);
void evrCheck();
void evrHistory(evrListener_fp);
uint16_t evrSeverity(uint16_t evr);
const char *evrSeverityToStr(uint16_t s);
const char *evrToSeverityStr(uint16_t evr);
const char* evrToStr(uint16_t);
/*
 Evr Lists for Information, Warnings, Errors and Fatal event reports.
 */
enum evrInfoList {
  EVR_NoEvrHere    = 0U,
  EVR_NormalReset,
  EVR_StartingMain,
  };

enum evrWarnList {
  EVR_AbnormalReset = 0x4000U,
  EVR_BatLow,
  EVR_BatVeryLow,
  EVR_ConfigBadHistory,
  EVR_SDCard_Failed,
  };

enum evrErrorList {
  EVR_OutOfListeners = 0x8000U,
  EVR_FailRegisterWatchdogFrameReset,
  EVR_FailRegisterWatchdogFrameLost,
  EVR_RxFrameLost,
  EVR_BatMaxLow,
  EVR_FlashCRCFail,
  EVR_FlashEraseFail,
  EVR_FlashProgramFail,
  };

//enum evrFatalList {
// EVR_  = 0xC000
//  };
