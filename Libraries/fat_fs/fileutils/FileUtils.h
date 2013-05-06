/*
 * FileUtils.h
 *
 *  Created on: Mar 21, 2012
 *      Author: rdouguet
 */

#ifndef FILEUTILS_H_
#define FILEUTILS_H_

#include "stm32f4xx.h"
//#include "utils.h"
#include "microsd_spi.h"
#include "ff.h"

void WriteFile(void);
void die(FRESULT rc);
void ReadFile(void);
void CreateFile(void);


#endif /* FILEUTILS_H_ */
