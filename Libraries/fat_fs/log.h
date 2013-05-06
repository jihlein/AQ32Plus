#ifndef LOG_H_
#define LOG_H_

//#include "main.h"
#include <stdint.h>

void log_init(void);
void log_printf(const char *text, ...);
void log_sync(void);
void write_to_file(const char *filename, uint8_t *buffer, uint32_t length);


#endif
