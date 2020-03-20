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
#include "cmsis_os.h"
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
#include "lv_examples/lv_apps/benchmark/benchmark.h"
#include "lv_examples/lv_apps/sysmon/sysmon.h"
#include "lv_examples/lv_apps/demo/demo.h"
#include "lv_examples/lv_apps/tpcal/tpcal.h"

#include "Sinteck/GUI/Screen_Main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BANK1_LCD_C    ((uint32_t)0x60000000)    // Disp Reg  ADDR
#define BANK1_LCD_D_8  ((uint32_t)0x60010000)    // Disp Data ADDR 8Bits
#define BANK1_LCD_D_16 ((uint32_t)0x60010000)    // Disp Data ADDR 16Bits
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t timer_key = 0;
uint16_t adcBuffer[4]; // Buffer for store the results of the ADC conversion

extern uint32_t duracao;
extern uint8_t rData[];

extern FATFS *pfs;
extern char line[]; /* Line buffer */
extern FRESULT fr;     /* FatFs return code */
extern DWORD fre_clust;
extern uint32_t totalSpace, freeSpace;
extern uint32_t size;
extern unsigned int ByteRead;
extern uint32_t timer_sd;

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

   logI("\r\n\r\n");
   logI(" W25Q128FV SPI Test Start\n\r");

   duracao = HAL_GetTick();
   W25qxx_ReadBytes(rData, 0, 512);
   duracao = HAL_GetTick() - duracao;

   logI(" W25Q128FV SPI Test End. ( Sector = 512 ) Duracao: %ld\r\n", duracao);
   logI("\r\n\r\n");

//   logI(" W25Q128FV SPI Test Start\n\r");

 //  duracao = HAL_GetTick();
 //  W25qxx_ReadBytes(rData, 0, 4096);
//   duracao = HAL_GetTick() - duracao;

//   logI(" W25Q128FV SPI Test End. ( Sector = 4096 ) Duracao: %ld\r\n", duracao);
   logI("\r\n\r\n");

//tst:
//
//  //HAL_GPIO_TogglePin(TFT_RST_GPIO_Port, TFT_RST_Pin);
//
//  //HAL_Delay(1);
//
//*(__IO uint8_t *)(BANK1_LCD_C) = 0x55;
//*(__IO uint8_t *)(BANK1_LCD_D_8) = 0xAA;
//
//*(__IO uint8_t *)(BANK1_LCD_C) = 0x55;
//*(__IO uint16_t *)(BANK1_LCD_D_16) = 0xAA55;
//
//
//  goto tst;

   /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
