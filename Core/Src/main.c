/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "lvgl/lvgl.h"
#include "lv_fs_if/lv_fs_if.h"
#include "Sinteck/Driver/Drv_SSD1963.h"
#include "Sinteck/Driver/Drv_XPT2046.h"
#include "Sinteck/src/log_cdc.h"
#include "Sinteck/src/w25qxx.h"
#include "Sinteck/src/keys.h"
#include "lv_drivers/indev/XPT2046.h"
#include "lv_examples/lv_apps/benchmark/benchmark.h"
#include "lv_examples/lv_apps/sysmon/sysmon.h"
#include "lv_examples/lv_apps/demo/demo.h"
#include "lv_examples/lv_apps/tpcal/tpcal.h"

#include "Sinteck/GUI/Screen_Main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BANK1_LCD_C    ((uint32_t)0x60000000)    // Disp Reg  ADDR
#define BANK1_LCD_D    ((uint32_t)0x60010000)    // Disp Data ADDR
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t timer_led = 0;
uint32_t timer_adc = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Mount_FATFS(void);
uint32_t read_btn_user(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */

FATFS *pfs;
char line[100]; /* Line buffer */
FRESULT fr;     /* FatFs return code */
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
uint32_t size;
unsigned int ByteRead;

uint8_t rData[4096];

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];		// Declare a buffer for 10 lines
static lv_color_t buf2[LV_HOR_RES_MAX * 10];	// Declare a buffer for 10 lines

uint32_t Btn_K1_0 = 0;
uint32_t Btn_K1_1 = 0;
uint32_t timer_key = 0;
uint32_t timer_sd = 0;
uint32_t duracao = 0;
//
uint16_t adcBuffer[4]; // Buffer for store the results of the ADC conversion
float tensao, corrente, potencia, setVolts, setAMP;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  logI("STM32F407 Test....\n\r");
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 4); // Start ADC in DMA
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
  // Init Flash SPI
  W25qxx_Init();
  BSP_SD_Init();
  // Teste Velocidade Leitura Flash SPI
  logI("\r\n\r\n");
  logI(" W25Q128FV SPI Test Start\n\r");

  duracao = HAL_GetTick();
  W25qxx_ReadBytes(rData, 0, 512);
  duracao = HAL_GetTick() - duracao;

  logI(" W25Q128FV SPI Test End. ( Sector = 512 ) Duracao: %ld\r\n", duracao);
  logI("\r\n\r\n");

  logI(" W25Q128FV SPI Test Start\n\r");

  duracao = HAL_GetTick();
  W25qxx_ReadBytes(rData, 0, 4096);
  duracao = HAL_GetTick() - duracao;

  logI(" W25Q128FV SPI Test End. ( Sector = 4096 ) Duracao: %ld\r\n", duracao);
  logI("\r\n\r\n");

  // Mount SD-CARD
  Mount_FATFS();

tst:
//
//  //HAL_GPIO_TogglePin(TFT_RST_GPIO_Port, TFT_RST_Pin);
//
//  //HAL_Delay(1);
//
*(__IO uint16_t *)(BANK1_LCD_C) = 0x55AA;
*(__IO uint16_t *)(BANK1_LCD_D) = 0xAA55;
//
//  //drv_ssd1963_cmd(0x55);
//  //drv_ssd1963_data(0xAA);

  goto tst;

  // Init SSD1963
  drv_ssd1963_init();
  drv_ssd1963_SetBacklight(200);

  lv_disp_buf_init(&disp_buf, buf, buf2, LV_HOR_RES_MAX * 10);    // Initialize the display buffer
  lv_init();

   // Lvgl File System
   lv_fs_if_init();

   // LVGL Setup
   lv_disp_drv_t disp_drv;               	// Descriptor of a display driver
   lv_disp_drv_init(&disp_drv);          	// Basic initialization
   // SSD1963
   disp_drv.hor_res = 800;               	// Set the horizontal resolution
   disp_drv.ver_res = 480;               	// Set the vertical resolution
   disp_drv.flush_cb = drv_ssd1963_flush;	// Set your driver function
   disp_drv.buffer = &disp_buf;          	// Assign the buffer to teh display
   lv_disp_drv_register(&disp_drv);      	// Finally register the driver

   //Initialize the touch pad
   XPT2046_init();
   lv_indev_drv_t indev_drv;
   lv_indev_drv_init(&indev_drv);  //Basic initialization
   indev_drv.type = LV_INDEV_TYPE_POINTER;
   indev_drv.read_cb = XPT2046_read;
   lv_indev_t * mouse_indev = lv_indev_drv_register(&indev_drv);

   // Mouse Icon
   LV_IMG_DECLARE(mouse_cursor_icon);                          /*Declare the image file.*/
   lv_obj_t * cursor_obj =  lv_img_create(lv_scr_act(), NULL); /*Create an image object for the cursor */
   lv_img_set_src(cursor_obj, &mouse_cursor_icon);             /*Set the image source*/
   lv_indev_set_cursor(mouse_indev, cursor_obj);               /*Connect the image  object to the driver*/

   //demo_create();
   //benchmark_create();
   //sysmon_create();
   GUI_PowerSupply();
   //tpcal_create();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Eventos de Teclado
	  KeyboardEvent();
	  // Eventos da GUI LittleVGL
	  lv_task_handler();

	  // Pisca Led 100ms
	  if(HAL_GetTick() - timer_led > 100) {
		  timer_led = HAL_GetTick();
		  HAL_GPIO_TogglePin(GPIOA, LED_Pin);
	  }
	  // Test Button K1
	  if(!read_btn_user()) {
		  CDC_Transmit_FS((uint8_t*)"E - Print\n", 14);
	  }

	  // Transmite ADC Value
	  if(HAL_GetTick() - timer_adc > 500) {
		  timer_adc = HAL_GetTick();
		  logI("ADC1: %ld, ADC2: %ld, ADC3: %ld, ADC4: %ld\n",
				  adcBuffer[0], adcBuffer[1], adcBuffer[2], adcBuffer[3]);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Mount_FATFS(void)
{
	  MX_FATFS_Init();

	  if( retSD != 0) {
		 // Erro FATFS
		 logI("STM32F407 FatFs - ERROR...\n\r");
	  }
	  else {
		  logI("STM32F407 FatFs - OK...\n\r");
	  }

	  if(f_mount(&SDFatFS, "", 0) != FR_OK) {
		  logI("STM32F407 FatFs - Mount Drive ERROR...\n\r");
	  }
	  else {
		  logI("STM32F407 FatFs - Mount Drive...\n\r");
	  }
	  // Check freeSpace space
	  if(f_getfree("", &fre_clust, &pfs) != FR_OK){
	  }

	  totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize);
	  freeSpace = (uint32_t)(fre_clust * pfs->csize);

	  logI("STM32F407 FatFs - %10lu KiB total drive space. %10lu KiB available.\n", totalSpace / 2, freeSpace / 2);

	  // Test Open config.txt
	  fr = f_open(&SDFile, "/Config.bin", FA_READ);
	  if(fr != FR_OK) {
	 	 logI("STM32F407 FatFs - Config.bin Error...Result: %d\n\r", fr);
	  }
	  else {
	 	 size = f_size(&SDFile);
	 	 logI("STM32F407 FatFs - Open File Config.bin... Result: %d Size: %d\n\r", fr, size);
	      fr = f_read(&SDFile, line , size, &ByteRead);
	      if(fr == FR_OK) {
	    	     logI("STM32F407 FatFs ReadFile...line: %s\n\r", line);
	      }
	      else {
	     	 logI("STM32F407 FatFs ReadFile...Error:  Result: %d \n\r", fr);
	      }
	      f_close(&SDFile);
	  }
	  // Test Open File + 8 Caracteres Tela_Principal.bin
	  duracao = HAL_GetTick();
	  fr = f_open(&SDFile, "/Tela_Principal.bin", FA_READ);
	  if(fr != FR_OK) {
	 	 logI("STM32F407 FatFs - Tela_Principal.bin Error...Result: %d\n\r", fr);
	  }
	  else {
	 	 size = f_size(&SDFile);
	 	 logI("STM32F407 FatFs - Open File Tela_Principal.bin... Result: %d  Size:%d\n\r", fr, size);
	 	 timer_sd = HAL_GetTick();
	 	 fr = f_read(&SDFile, line , 100, &ByteRead);
	     timer_sd = HAL_GetTick() - timer_sd;

	 	 if(fr == FR_OK) {
	     	 logI("STM32F407 FatFs ReadFile... Timer: %ld line: %s\n\r", timer_sd, line);
	      }
	      else {
	     	 logI("STM32F407 FatFs ReadFile...Error:  Result: %d \n\r", fr);
	      }
	 	  duracao = HAL_GetTick() - duracao;
	 	  logI("STM32F407 FatFs ReadFile Duracao:  Result: %ld \n\r", duracao);
	      f_close(&SDFile);
	  }
}

uint32_t read_btn_user(void)
{
	uint32_t ret = 0;

	if(HAL_GPIO_ReadPin(BT_USER_GPIO_Port, BT_USER_Pin) == 1) {
		Btn_K1_0 = 0;
		Btn_K1_1++;
		if(Btn_K1_1 > DEBOUNCE_BTN) {
			Btn_K1_1 = DEBOUNCE_BTN + 1;
			ret = 1;
		}
	}
	else {
		Btn_K1_1 = 0;
		Btn_K1_0++;
		if(Btn_K1_0 > DEBOUNCE_BTN) {
			Btn_K1_0 = DEBOUNCE_BTN + 1;
			ret = 0;
		}
	}
	return ret;
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
	  lv_tick_inc(1);
  }
  if (htim->Instance == TIM6) {
	  timer_key++;
	  if(timer_key >= 30) {
		  timer_key = 0;
		  Key_Read();
	  }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
