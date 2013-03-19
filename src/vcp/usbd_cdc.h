/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Header for usbd_cdc_vcp.c file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_H
#define __USBD_CDC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "usbd_cdc_core.h"
#include "usbd_conf.h"


#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

extern uint16_t cdc_DataTx (uint8_t* Buf, uint32_t Len);

/// cdc_RX_IsCharReady() returns (-1) if chars in Rx_Buf else (0) :
char	cdc_RX_IsCharReady(void);

/// cdc_RX_ChkChar() returns (char) if chars in Rx_Buf else (0) but no Inc BufIdx :
char	cdc_RX_ChkChar(void);

/// cdc_RX_GetChar() returns (char) if chars in Rx_Buf else (0) and Incs BufIdx :
char	cdc_RX_GetChar(void);

/// Waits until Char in RX buf then returns the char and Inc Buf Idx :
char	cdc_RX_GetWaitChar();


#endif /* __USBD_CDC_VCP_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
