/*
 * Drv_XPT2046.h
 *
 *  Created on: Feb 11, 2020
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#ifndef DRIVER_DRV_XPT2046_H_
#define DRIVER_DRV_XPT2046_H_

#include <stdint.h>
#include <stdbool.h>
#include "lv_drv_conf.h"
#include "../lvgl/src/lv_hal/lv_hal_indev.h"


void XPT2046_init(void);
uint16_t XPT2046_GetTouch(uint8_t address);
void XPT2046_GetTouch_XY(volatile uint16_t* x_kor, volatile uint16_t* y_kor, uint8_t count_read);
void xpt2046_avg(uint16_t * x, uint16_t * y);
void xpt2046_corr(uint16_t * x, uint16_t * y);
bool XPT2046_read(lv_indev_drv_t * drv, lv_indev_data_t*data);

#endif /* DRIVER_DRV_XPT2046_H_ */
