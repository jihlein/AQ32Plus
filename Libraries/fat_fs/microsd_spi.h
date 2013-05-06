/*
 * microsd_spi.h
 *
 *  Created on: Mar 8, 2012
 *      Author: rdouguet
 */

#ifndef MICROSD_SPI_H_
#define MICROSD_SPI_H_

#include "diskio.h"

DSTATUS disk_initialize(BYTE drv);
void disk_timerproc(void);
/*static void interface_speed( enum speed_setting speed );
static BYTE stm32_spi_rw( BYTE out );
static BYTE rcvr_spi (void);
static BYTE wait_ready (void);
static void release_spi (void);


static void power_off (void);
static BOOL rcvr_datablock(BYTE *buff, UINT btr);
static BOOL xmit_datablock ( const BYTE *buff, BYTE token );
static BYTE send_cmd ( BYTE cmd, DWORD arg );*/

#endif /* MICROSD_SPI_H_ */
