/*
 * log_cdc.h
 *
 *  Created on: 26 de jun de 2019
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#ifndef INC_LOG_CDC_H_
#define INC_LOG_CDC_H_


#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

void HAL_printf_valist(const char *fmt, va_list argp);
void HAL_printf(const char *fmt, ...);
void logUSB(const char *fmt, va_list argp);
void logI(const char* fmt, ...);
void logE(const char* fmt, ...);
char* concat(int count, ...);


#endif /* INC_LOG_CDC_H_ */
