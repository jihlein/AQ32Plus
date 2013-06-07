/**
  \file       evr.c
  \brief      EVR (EVent Report) Module.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

enum evrConstants
  {
  evrBITS         = 16, /*!< size of evr message. */
  evrSeverityBITS = 2,  /*!< by convention top two bits are severity */
  evrBuffBITS     = 4,  /*!< size of ring buffer in bits */
  evrListenerMAX  = 8,  /*!< Max number of listeners */

// Derived constants.
  evrBuffMAX      = 1 << evrBuffBITS,
  evrBuffMASK     = evrBuffMAX - 1,
  evrMessageBITS  = evrBITS - evrSeverityBITS,
  evrMessageMAX   = 1 << evrMessageBITS,
  evrMessageMASK  = evrMessageMAX - 1,
  evrSeverityMAX  = 1 << evrSeverityBITS,
  evrSeverityMASK = evrSeverityMAX - 1,
  };

static evr_t          evrRingBuff [evrBuffMAX];
static evrListener_fp evrListeners[evrListenerMAX];

static uint32_t  evrHead = 0;
static uint32_t  evrTail = 0;
static uint32_t  evrListenerTop = 0;

///////////////////////////////////////////////////////////////////////////////
/*
  \brief         Push an Event Report. IRQ states are protected, so an EVR
                 can be pushed from anywhere, includeing IRQ Handlers.
  \param evr     Handle of event to push.
  \param reason  A paramater (reason) to be pushed with the handle.
 */
void evrPush(uint16_t evr, uint16_t reason)
  {
  evr_t e = { millis(), evr, reason };
  int i;
  do
    i = __LDREXW(&evrHead);
  while (__STREXW( (i+1) & evrBuffMASK,&evrHead));
  evrRingBuff[ i ] = e;
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief      Register a Listener function to be called in user mode
              whenever and EVR occurred.
  \param f    The call back function.
  \return     0 on success, 1 otherwise.
*/

int evrRegisterListener(evrListener_fp f)
  {
  int r = evrListenerTop < evrListenerMAX ;
  if (r)
    evrListeners[evrListenerTop++] = f ;
  else
    evrPush(EVR_OutOfListeners,0);
  return !r;
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief    Broadcast and Event Report to all listeners.
  \param e  The event to be broadcast.
 */
void evrBroadcast(evr_t e)
  {
  int i;
  for (i=0; i < evrListenerTop; ++i)
    (*(evrListeners[i]))(e);
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief   Check the EVR queue for unbroadcast EVRs, and broadcast them.
 */
void evrCheck()
  {
  while (evrHead != evrTail)
    {
    evrBroadcast( evrRingBuff[evrTail] );
    evrTail = (evrTail +1) & evrBuffMASK;
    }
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief    Rebroadcast all EVRs that have been in the queue to a
            specific listener function.
  \param f  The listener to call.
 */
void evrHistory(evrListener_fp f)
  {
  uint32_t i = evrHead;
  do
    {
    (*f)(evrRingBuff[i]);
    i = (i + 1) & evrBuffMASK;
    } while ( i != evrTail ) ;
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief     Return the severity of an EVR.
  \param evr
 */
uint16_t evrSeverity(uint16_t evr)
  {
  return  (evr >> evrMessageBITS);
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief     Return the constant string pointer associated with this Severity.
  \param s
 */
const char *evrSeverityToStr(uint16_t s)
  {
  return evrStringTable[ s & evrSeverityMASK ].severity;
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief     Return the severity of an EVR as const string pointer.
  \param evr
 */
const char *evrToSeverityStr(uint16_t evr)
  {
  return evrSeverityToStr(evrSeverity(evr));
  }

///////////////////////////////////////////////////////////////////////////////
/*
  \brief      Return the constant string pointer associated with this EVR.
  \param evr
  \return     string pointer, or NULL if evr was invalid.
 */
const char *evrToStr(uint16_t evr)
  {
  uint32_t s = evrSeverity(evr),
           i = evr & evrMessageMASK;
  if (i < evrStringTable[s].len )
    return evrStringTable[s].msgs[i];
  else
    return (const char*)0;
  }

///////////////////////////////////////////////////////////////////////////////