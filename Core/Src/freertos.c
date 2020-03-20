/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
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
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];		// Declare a buffer for 10 lines
static lv_color_t buf2[LV_HOR_RES_MAX * 10];	// Declare a buffer for 10 lines
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

uint32_t Btn_K1_0 = 0;
uint32_t Btn_K1_1 = 0;
uint32_t timer_adc = 0;
uint32_t timer_sd = 0;
uint32_t timer_led = 0;
uint32_t duracao = 0;
float tensao, corrente, potencia, setVolts, setAMP;

extern uint16_t adcBuffer[4]; // Buffer for store the results of the ADC conversion

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Mount_FATFS(void);
uint32_t read_btn_user(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for TaskGUI */
osThreadId_t TaskGUIHandle;
const osThreadAttr_t TaskGUI_attributes = {
  .name = "TaskGUI",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 6000 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskGUI(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskGUI */
  TaskGUIHandle = osThreadNew(StartTaskGUI, NULL, &TaskGUI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
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

  /* Infinite loop */
  for(;;)
  {
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
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskGUI */
/**
* @brief Function implementing the TaskGUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGUI */
void StartTaskGUI(void *argument)
{
  /* USER CODE BEGIN StartTaskGUI */
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
  /* Infinite loop */
  for(;;)
  {
	  // Eventos de Teclado
	  KeyboardEvent();
	  // Eventos da GUI LittleVGL
	  lv_task_handler();

	  osDelay(10);
  }
  /* USER CODE END StartTaskGUI */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
