/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   USB Device configuration file
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
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */ 
#define USBD_CFG_MAX_NUM                1
#define USBD_ITF_MAX_NUM                1
#define USB_MAX_STR_DESC_SIZ            50 

/** @defgroup USB_VCP_Class_Layer_Parameter
  * @{
  */ 
#define CDC_IN_EP                       0x81  /* EP1 for data IN */
#define CDC_OUT_EP                      0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                      0x82  /* EP2 for CDC commands */

#define CDC_IN_EP1                      CDC_IN_EP  /* EP1 for data IN */
#define CDC_OUT_EP1                     CDC_OUT_EP  /* EP1 for data OUT */
#define CDC_CMD_EP1                     CDC_CMD_EP  /* EP2 for CDC commands */

#define	USBCOMPMSC_E	1	// -
#ifdef	USBCOMPMSC_E
#define CDC_IN_EP2                   0x83  /* EP1 for data IN */
#define CDC_OUT_EP2                  0x02  /* EP1 for data OUT */
#define CDC_CMD_EP2                  0x84  /* EP2 for CDC commands */

#define MSC_IN_EP2                   CDC_IN_EP2  /* EP1 for data IN */
#define MSC_OUT_EP2                  CDC_OUT_EP2  /* EP1 for data OUT */
#define MSC_CMD_EP2                  CDC_CMD_EP2  /* EP2 for CDC commands */

#define MSC_IN_EP                    MSC_IN_EP2
#define MSC_OUT_EP                   MSC_OUT_EP2
#endif

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#ifdef USE_USB_OTG_HS
 #define CDC_DATA_MAX_PACKET_SIZE       512  /* Endpoint IN & OUT Packet size */
 #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */

 #define CDC_IN_FRAME_INTERVAL          40   /* Number of micro-frames between IN transfers */
 #define APP_RX_DATA_SIZE               2048 /* Total size of IN buffer: 
                                                APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL*8 */
#else
 #define CDC_DATA_MAX_PACKET_SIZE       64   /* Endpoint IN & OUT Packet size */
 #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */

 #define CDC_IN_FRAME_INTERVAL          5    /* Number of frames between IN transfers */
 #define APP_RX_DATA_SIZE               2048 /* Total size of IN buffer: 
                                                APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL */
#endif /* USE_USB_OTG_HS */

////#define	USBCOMPMSC_E	1	// -set above-^
#ifdef	USBCOMPMSC_E
//#define MSC_IN_EP                    0x81
//#define MSC_OUT_EP                   0x01
#ifdef USE_USB_OTG_HS
#ifdef USE_ULPI_PHY
#define MSC_MAX_PACKET               512
#else
#define MSC_MAX_PACKET               64
#endif
#else  /*USE_USB_OTG_FS*/
#define MSC_MAX_PACKET                64
#endif

#define MSC_MEDIA_PACKET             4096
#endif

#define APP_FOPS                        cdc_fops
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif //__USBD_CONF__H__

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

