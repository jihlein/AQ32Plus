/**
  ******************************************************************************
  * @file    usbd_usr.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   This file includes the user application layer
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

/* Includes ------------------------------------------------------------------*/
#include "board.h"

#include "usbd_usr.h"
#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/

/** @defgroup USBD_USR
* @brief    This file includes the user application layer
* @{
*/

/** @defgroup USBD_USR_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBD_USR_Private_Defines
* @{
*/
/**
* @}
*/


/** @defgroup USBD_USR_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup USBD_USR_Private_Variables
* @{
*/

USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,

  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};

/**
* @}
*/

/** @defgroup USBD_USR_Private_Constants
* @{
*/

/**
* @}
*/



/** @defgroup USBD_USR_Private_FunctionPrototypes
* @{
*/
/**
* @}
*/


/** @defgroup USBD_USR_Private_Functions
* @{
*/

/**
* @brief  USBD_USR_Init
*
* @param  None
* @retval None
*/
void USBD_USR_Init(void)
{

}

/**
* @brief  USBD_USR_DeviceReset
*
* @param  speed : device speed
* @retval None
*/
void USBD_USR_DeviceReset(uint8_t speed )
{

}


/**
* @brief  USBD_USR_DeviceConfigured
*         Sets usbDeviceConfigured TRUE on device configuration Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConfigured (void)
{
	usbDeviceConfigured = true;
}

/**
* @brief  USBD_USR_DeviceSuspended
*         Sets usbDeviceConfigured FALSE on device suspend Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceSuspended(void)
{
	usbDeviceConfigured = false;
}


/**
* @brief  USBD_USR_DeviceResumed
*         Sets usbDeviceConfigured TRUE on device resume Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceResumed(void)
{
	usbDeviceConfigured = true;
}

/**
* @brief  USBD_USR_DeviceConnected
*         Sets usbDeviceConnected TRUE on device connection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConnected(void)
{

}


/**
* @brief  USBD_USR_DeviceDisonnected
*         Sets usbDeviceConnected FALSE on device disconnection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceDisconnected(void)
{

}

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
