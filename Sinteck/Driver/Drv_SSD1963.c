/**
 * @file SSD1963.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "Sinteck/Driver/Drv_SSD1963.h"
#include "main.h"
#include "Sinteck/src/log_cdc.h"

#if USE_SSD1963

#include <stdbool.h>
#include LV_DRV_DISP_INCLUDE
//#include LV_DRV_DELAY_INCLUDE

/*********************
 *      DEFINES
 *********************/
#define SSD1963_CMD_MODE     0
#define SSD1963_DATA_MODE    1

/**********************
 *      TYPEDEFS
 **********************/
#define TFT_CMD              ((uint32_t)0x60000000) /* RS = 0 */
#define TFT_DATA             ((uint32_t)0x60010000) /* RS = 1 */

/**********************
 *  STATIC PROTOTYPES
 **********************/
static inline void drv_ssd1963_cmd_mode(void);
static inline void drv_ssd1963_data_mode(void);
//static inline void drv_ssd1963_cmd(uint8_t cmd);
//static inline void drv_ssd1963_data(uint8_t data);
//static void ssd1963_io_init(void);
static void drv_ssd1963_reset(void);
//static void ssd1963_set_clk(void);
//static void ssd1963_set_tft_spec(void);
//static void ssd1963_init_bl(void);

/**********************
 *  STATIC VARIABLES
 **********************/
static bool cmd_mode = true;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void drv_ssd1963_init(void)
{

    LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
    cmd_mode = true;

    drv_ssd1963_reset();
    LV_DRV_DELAY_MS(250);


    drv_ssd1963_cmd(0x00E2);    //PLL multiplier, set PLL clock to 120M
    drv_ssd1963_data(0x0023);   //N=0x36 for 6.5M, 0x23 for 10M crystal
    drv_ssd1963_data(0x0002);
    drv_ssd1963_data(0x0004);
    drv_ssd1963_cmd(0x00E0);    // PLL enable
    drv_ssd1963_data(0x0001);
    LV_DRV_DELAY_MS(1);
    drv_ssd1963_cmd(0x00E0);
    drv_ssd1963_data(0x0003);   // now, use PLL output as system clock
    LV_DRV_DELAY_MS(1);
    drv_ssd1963_cmd(0x0001);    // software reset
    LV_DRV_DELAY_MS(1);
    drv_ssd1963_cmd(0x00E6);    //PLL setting for PCLK, depends on resolution

    drv_ssd1963_data(0x0001);  //HX8257C
    drv_ssd1963_data(0x0047);  //HX8257C
    drv_ssd1963_data(0x00B1);  //HX8257C


    drv_ssd1963_cmd(0x00B0);    //LCD SPECIFICATION
    drv_ssd1963_data(0x0020);
    drv_ssd1963_data(0x0000);
    drv_ssd1963_data(((SSD1963_HOR_RES - 1) >> 8) & 0X00FF); //Set HDP
    drv_ssd1963_data((SSD1963_HOR_RES - 1) & 0X00FF);
    drv_ssd1963_data(((SSD1963_VER_RES - 1) >> 8) & 0X00FF); //Set VDP
    drv_ssd1963_data((SSD1963_VER_RES - 1) & 0X00FF);
    drv_ssd1963_data(0x0000);
    LV_DRV_DELAY_MS(1);//Delay10us(5);
    drv_ssd1963_cmd(0x00B4);            //HSYNC
    drv_ssd1963_data((SSD1963_HT >> 8) & 0X00FF); //Set HT
    drv_ssd1963_data(SSD1963_HT & 0X00FF);
    drv_ssd1963_data((SSD1963_HPS >> 8) & 0X00FF); //Set HPS
    drv_ssd1963_data(SSD1963_HPS & 0X00FF);
    drv_ssd1963_data(SSD1963_HPW);              //Set HPW
    drv_ssd1963_data((SSD1963_LPS >> 8) & 0X00FF); //SetLPS
    drv_ssd1963_data(SSD1963_LPS & 0X00FF);
    drv_ssd1963_data(0x0000);

    drv_ssd1963_cmd(0x00B6);            //VSYNC
    drv_ssd1963_data((SSD1963_VT >> 8) & 0X00FF); //Set VT
    drv_ssd1963_data(SSD1963_VT & 0X00FF);
    drv_ssd1963_data((SSD1963_VPS >> 8) & 0X00FF); //Set VPS
    drv_ssd1963_data(SSD1963_VPS & 0X00FF);
    drv_ssd1963_data(SSD1963_VPW);              //Set VPW
    drv_ssd1963_data((SSD1963_FPS >> 8) & 0X00FF); //Set FPS
    drv_ssd1963_data(SSD1963_FPS & 0X00FF);

    drv_ssd1963_cmd(0x00B8);
    drv_ssd1963_data(0x0007);    //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF
    drv_ssd1963_data(0x0001);    //GPIO0 normal

    drv_ssd1963_cmd(0x00BA);
    drv_ssd1963_data(0x000F);    //GPIO[0] out 1 --- LCD display on/off control PIN

    drv_ssd1963_cmd(0x0036);    //rotation
    drv_ssd1963_data(0x0010);   //RGB=BGR

    drv_ssd1963_cmd(0x003A);    //Set the current pixel format for RGB image data
    drv_ssd1963_data(0x0050);   //16-bit/pixel

    drv_ssd1963_cmd(0x00F0);    //Pixel Data Interface Format
    drv_ssd1963_data(0x0000);   //16-bit(565 format) data

    drv_ssd1963_cmd(0x00BC);
    drv_ssd1963_data(0x0040);   //contrast value
    drv_ssd1963_data(0x0080);   //brightness value
    drv_ssd1963_data(0x0040);   //saturation value
    drv_ssd1963_data(0x0001);   //Post Processor Enable

    LV_DRV_DELAY_MS(1);

    drv_ssd1963_cmd(0x2a);			//SET page address
    drv_ssd1963_data(0x00);			//SET start page address
    drv_ssd1963_data(0x00);
    drv_ssd1963_data((SSD1963_HOR_RES - 1) >> 8);		//SET end page address
    drv_ssd1963_data((SSD1963_HOR_RES - 1) & 0xFF);

    drv_ssd1963_cmd(0x2b);			//SET column address
    drv_ssd1963_data(0x00);				//SET start column address
    drv_ssd1963_data(0x00);
    drv_ssd1963_data((SSD1963_VER_RES - 1) >> 8);		//SET end column address
    drv_ssd1963_data((SSD1963_VER_RES - 1) & 0xFF);

    drv_ssd1963_cmd(0x2c);
    for (uint16_t x=0; x < SSD1963_HOR_RES; x++)
    {
      for (uint16_t y= 0; y < SSD1963_VER_RES; y++)
      {
    	  drv_ssd1963_data((0x000000)>>16); 		// color is red
    	  drv_ssd1963_data((0x000000)>>8);  		// color is green
    	  drv_ssd1963_data(0x000000);  		 		// color is blue
      }
    }

    drv_ssd1963_cmd(0x29); //display on

    drv_ssd1963_cmd(0xBE); //set PWM for B/L
    drv_ssd1963_data(0x06);
    drv_ssd1963_data(0xF0);
    drv_ssd1963_data(0x01);
    drv_ssd1963_data(0xf0);
    drv_ssd1963_data(0x00);
    drv_ssd1963_data(0x00);

    drv_ssd1963_cmd(0x00d0);
    drv_ssd1963_data(0x000d);

//    DisplayBacklightOn();
    drv_ssd1963_cmd(0x2c);

    LV_DRV_DELAY_MS(30);
}

void drv_ssd1963_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*Return if the area is out the screen*/
    if(area->x2 < 0) return;
    if(area->y2 < 0) return;
    if(area->x1 > SSD1963_HOR_RES - 1) return;
    if(area->y1 > SSD1963_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > SSD1963_HOR_RES - 1 ? SSD1963_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 > SSD1963_VER_RES - 1 ? SSD1963_VER_RES - 1 : area->y2;

    //Set the rectangular area
    drv_ssd1963_cmd(0x002A);
    drv_ssd1963_data(act_x1 >> 8);
    drv_ssd1963_data(0x00FF & act_x1);
    drv_ssd1963_data(act_x2 >> 8);
    drv_ssd1963_data(0x00FF & act_x2);

    drv_ssd1963_cmd(0x002B);
    drv_ssd1963_data((act_y1 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y1 + OFFSET_Y));
    drv_ssd1963_data((act_y2 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y2 + OFFSET_Y));

    drv_ssd1963_cmd(0x2c);

    drv_ssd1963_data_mode();

#if LV_COLOR_DEPTH == 16
//    int32_t full_w = area->x2 - area->x1 + 1;
    for(int32_t i = act_y1; i <= act_y2; i++) {
        //LV_DRV_DISP_PAR_WR_ARRAY((uint16_t *)color_p, act_w);
    	drv_ssd1963_data(color_p->full);
    	//color_p += full_w;
    	color_p++;
    }
    LV_DRV_DISP_PAR_CS(1);
#else
    int32_t size = (act_x2 - act_x1 + 1) * (act_y2 - act_y1 + 1);
    for(int32_t i = 0; i <= size-1; i++) {
    	drv_ssd1963_data(color_p->ch.red); 			// color red
    	drv_ssd1963_data(color_p->ch.green); 		// color green
    	drv_ssd1963_data(color_p->ch.blue); 		// color blue
    	color_p++;
    }
#endif

    lv_disp_flush_ready(disp_drv);                  /* Tell you are ready with the flushing*/
}

void ssd1963_fill(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*Return if the area is out the screen*/
    if(area->x2 < 0) return;
    if(area->y2 < 0) return;
    if(area->x1 > SSD1963_HOR_RES - 1) return;
    if(area->y1 > SSD1963_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > SSD1963_HOR_RES - 1 ? SSD1963_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 > SSD1963_VER_RES - 1 ? SSD1963_VER_RES - 1 : area->y2;

    //Set the rectangular area
    drv_ssd1963_cmd(0x002A);
    drv_ssd1963_data(act_x1 >> 8);
    drv_ssd1963_data(0x00FF & act_x1);
    drv_ssd1963_data(act_x2 >> 8);
    drv_ssd1963_data(0x00FF & act_x2);

    drv_ssd1963_cmd(0x002B);
    drv_ssd1963_data((act_y1 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y1 + OFFSET_Y));
    drv_ssd1963_data((act_y2 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y2 + OFFSET_Y));

    drv_ssd1963_cmd(0x2c);

    LV_DRV_DISP_PAR_CS(0);
    drv_ssd1963_data_mode();

    //uint16_t color16 = lv_color_to16(color);
    int32_t size = (act_x2 - act_x1 + 1) * (act_y2 - act_y1 + 1);
    int32_t i;
    for(i = 0; i < size-1; i++) {
    	drv_ssd1963_data(color_p->ch.red); 			// color red
    	drv_ssd1963_data(color_p->ch.green); 		// color green
    	drv_ssd1963_data(color_p->ch.blue); 		// color blue
    }
    LV_DRV_DISP_PAR_CS(1);
}

void drv_ssd1963_map(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{

    /*Return if the area is out the screen*/
    if(area->x2 < 0) return;
    if(area->y2 < 0) return;
    if(area->x1 > SSD1963_HOR_RES - 1) return;
    if(area->y1 > SSD1963_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > SSD1963_HOR_RES - 1 ? SSD1963_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 > SSD1963_VER_RES - 1 ? SSD1963_VER_RES - 1 : area->y2;

    //Set the rectangular area
    drv_ssd1963_cmd(0x002A);
    drv_ssd1963_data(act_x1 >> 8);
    drv_ssd1963_data(0x00FF & act_x1);
    drv_ssd1963_data(act_x2 >> 8);
    drv_ssd1963_data(0x00FF & act_x2);

    drv_ssd1963_cmd(0x002B);
    drv_ssd1963_data((act_y1 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y1 + OFFSET_Y));
    drv_ssd1963_data((act_y2 + OFFSET_Y) >> 8);
    drv_ssd1963_data(0x00FF & (act_y2 + OFFSET_Y));

    drv_ssd1963_cmd(0x2c);

    LV_DRV_DISP_PAR_CS(0);
    drv_ssd1963_data_mode();

#if LV_COLOR_DEPTH == 16
    int32_t full_w = area->x2 - area->x1 + 1;
    for(int32_t i = act_y1; i <= act_y2; i++) {
        LV_DRV_DISP_PAR_WR_ARRAY((uint16_t *)color_p, act_w);
        color_p += full_w;
    }
    LV_DRV_DISP_PAR_CS(1);
#else
    int32_t j;
    int32_t i;
    int32_t full_w = area->x2 - area->x1 + 1;
    for(i = act_y1; i <= act_y2; i++) {
        for(j = 0; j <= act_x2 - act_x1 + 1; j++) {
            LV_DRV_DISP_PAR_WR_WORD(color_p[j]);
            color_p += full_w;
        }
    }
#endif
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

//static void ssd1963_io_init(void)
//{
//    LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
//    cmd_mode = true;
//}

static void drv_ssd1963_reset(void)
{
    /*Hardware reset*/
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(0);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);

    /*Software reset*/
    drv_ssd1963_cmd(0x01);
    LV_DRV_DELAY_MS(20);

    drv_ssd1963_cmd(0x01);
    LV_DRV_DELAY_MS(20);

    drv_ssd1963_cmd(0x01);
    LV_DRV_DELAY_MS(20);

}

/**
 * Command mode
 */
static inline void drv_ssd1963_cmd_mode(void)
{
    if(cmd_mode == false) {
        LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
        cmd_mode = true;
    }
}

/**
 * Data mode
 */
static inline void drv_ssd1963_data_mode(void)
{
    if(cmd_mode != false) {
        LV_DRV_DISP_CMD_DATA(SSD1963_DATA_MODE);
        cmd_mode = false;
    }
}

/**
 * Write command
 * @param cmd the command
 */
void drv_ssd1963_cmd(uint8_t cmd)
{
	*(__IO uint8_t *)(TFT_CMD) = cmd;
}

/**
 * Write data
 * @param data the data
 */
void drv_ssd1963_data(uint8_t data)
{
	*(__IO uint8_t *)(TFT_DATA) = data;
}

void drv_ssd1963_SetBacklight(uint8_t intensity)
{
	drv_ssd1963_cmd(0xBE);			// Set PWM configuration for backlight control
	drv_ssd1963_data(0x06);			// PWMF[7:0] = 2, PWM base freq = PLL/(256*(1+5))/256 =
									// 300Hz for a PLL freq = 120MHz
	drv_ssd1963_data(intensity);	// Set duty cycle, from 0x00 (total pull-down) to 0xFF
									// (99% pull-up , 255/256)
	drv_ssd1963_data(0x01);			// PWM enabled and controlled by host (mcu)
	drv_ssd1963_data(0x00);
	drv_ssd1963_data(0x00);
	drv_ssd1963_data(0x00);
}

void my_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time, uint32_t px)
{
	logI("Debug: %d px refreshed in %d ms\n", px, time);
}

#endif
