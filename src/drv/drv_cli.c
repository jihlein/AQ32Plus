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

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t usbDeviceConfigured = false;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

///////////////////////////////////////////////////////////////////////////////

void cliInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = USB_DISCONNECT_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(USB_DISCONNECT_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

    delay(200);

	GPIO_SetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

	USBD_Init(&USB_OTG_dev,	USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
}

///////////////////////////////////////////////////////////////////////////////

uint8_t cliAvailable(void)
{
    if (cdc_RX_IsCharReady() == -1)
    	return(true);
    else
    	return(false);
}

///////////////////////////////////////////////////////////////////////////////

char cliRead(void)
{
    if (usbDeviceConfigured == true)
        return cdc_RX_GetChar();
    else
        return(0);
}

///////////////////////////////////////////////////////////////////////////////

void cliPrint(char* str)
{
	if (usbDeviceConfigured == true)
	{
		cdc_DataTx((unsigned char*)str, strlen(str));
	}
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print Formatted - Print formatted string to USB VCP
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void cliPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	cliPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
