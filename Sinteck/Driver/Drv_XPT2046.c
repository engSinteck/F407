/*
 * Drv_XPT2046.c
 *
 *  Created on: Feb 11, 2020
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#include "main.h"
#include "Sinteck/Driver/Drv_XPT2046.h"

#include "lv_drv_conf.h"

extern SPI_HandleTypeDef hspi2;

#define XPT2046_CS_GPIO_Port 	TFT_CS_GPIO_Port
#define XPT2046_CS_Pin 			TFT_CS_Pin
#define XPT2046_SPI				hspi2

#define Y 						0x90
#define X 						0xD0


uint8_t SpiBuffer[10] = {0};
uint8_t SpiBufferRx[10] = {0};
int16_t avg_buf_x[XPT2046_AVG];
int16_t avg_buf_y[XPT2046_AVG];
uint8_t avg_last;

void XPT2046_init(void)
{
	SpiBuffer[0]=0x80;
	SpiBuffer[1]=0x00;
	SpiBuffer[2]=0x00;

	HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&XPT2046_SPI, SpiBuffer, SpiBufferRx, 3, 100);
	HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_SET);
}

uint16_t XPT2046_GetTouch(uint8_t address)
{
    uint16_t data = 0;

    HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_RESET);

    SpiBuffer[0] = address;
    SpiBuffer[1] = 0x00;
    SpiBuffer[2] = 0x00;

    HAL_SPI_TransmitReceive(&XPT2046_SPI, SpiBuffer, SpiBufferRx, 3, 100);

    HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_SET);

    data = SpiBufferRx[1];
    data <<= 8;

    data |= SpiBufferRx[2];
    data >>= 3;

    return data;
}

void XPT2046_GetTouch_XY(volatile uint16_t* x_kor, volatile uint16_t* y_kor, uint8_t count_read)
{
    uint8_t i = 0;
    uint16_t tmpx, tmpy, touch_x, touch_y = 0;

    touch_x = XPT2046_GetTouch(X);
    //delay_us(100);
    touch_y = XPT2046_GetTouch(Y);
    for (i = 0; i < count_read; i++) {
        tmpx = XPT2046_GetTouch(X);
        //delay_us(100);
        tmpy = XPT2046_GetTouch(Y);

        if (tmpx == 0)  tmpy = 0;
        else if (tmpy == 0)  tmpx = 0;
    else
    {
			touch_x = (touch_x + tmpx) / 2;
			touch_y = (touch_y + tmpy) / 2;
    }

	}
		*x_kor = touch_x;
		*y_kor = touch_y;

}

void xpt2046_corr(uint16_t * x, uint16_t * y)
{
#if XPT2046_XY_SWAP
   int16_t swap_tmp;
   swap_tmp = *x;
   *x = *y;
   *y = swap_tmp;
#endif

   if((*x) > XPT2046_X_MIN)(*x) -= XPT2046_X_MIN;
   else(*x) = 0;

   if((*y) > XPT2046_Y_MIN)(*y) -= XPT2046_Y_MIN;
   else(*y) = 0;

   (*x) = (uint32_t)((uint32_t)(*x) * XPT2046_HOR_RES) /
          (XPT2046_X_MAX - XPT2046_X_MIN);

   (*y) = (uint32_t)((uint32_t)(*y) * XPT2046_VER_RES) /
          (XPT2046_Y_MAX - XPT2046_Y_MIN);

#if XPT2046_X_INV
   (*x) =  XPT2046_HOR_RES - (*x);
#endif

#if XPT2046_Y_INV
   (*y) =  XPT2046_VER_RES - (*y);
#endif
}

void xpt2046_avg(uint16_t * x, uint16_t * y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for(i = XPT2046_AVG - 1; i > 0 ; i--) {
        avg_buf_x[i] = avg_buf_x[i - 1];
        avg_buf_y[i] = avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    avg_buf_x[0] = *x;
    avg_buf_y[0] = *y;
    if(avg_last < XPT2046_AVG) avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for(i = 0; i < avg_last ; i++) {
        x_sum += avg_buf_x[i];
        y_sum += avg_buf_y[i];
    }

    /*Normalize the sums*/
    (*x) = (int32_t)x_sum / avg_last;
    (*y) = (int32_t)y_sum / avg_last;
}

bool XPT2046_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
    uint16_t x = 0;
    uint16_t y = 0;
    uint8_t irq = LV_DRV_INDEV_IRQ_READ;

    if (irq == 0) {
        XPT2046_GetTouch_XY(&x, &y, 1);
        xpt2046_corr(&x, &y);

        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PR;
    }
    else
        data->state = LV_INDEV_STATE_REL;
   // printf("X=%d  Y= %d  m_sec=%d count=%d\n\r", x, y, millis(), count);
    return false;
}
