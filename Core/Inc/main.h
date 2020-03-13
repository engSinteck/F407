/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_UP_Pin GPIO_PIN_2
#define KEY_UP_GPIO_Port GPIOE
#define KEY_DN_Pin GPIO_PIN_3
#define KEY_DN_GPIO_Port GPIOE
#define KEY_ENTER_Pin GPIO_PIN_4
#define KEY_ENTER_GPIO_Port GPIOE
#define KEY_ESC_Pin GPIO_PIN_5
#define KEY_ESC_GPIO_Port GPIOE
#define SD_PRESENT_Pin GPIO_PIN_6
#define SD_PRESENT_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define BT_USER_Pin GPIO_PIN_0
#define BT_USER_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define TFT_DC_Pin GPIO_PIN_5
#define TFT_DC_GPIO_Port GPIOC
#define TFT_RST_Pin GPIO_PIN_0
#define TFT_RST_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define TFT_SCK_Pin GPIO_PIN_13
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_MISO_Pin GPIO_PIN_14
#define TFT_MISO_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define CS_FLASH_Pin GPIO_PIN_15
#define CS_FLASH_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_3
#define FLASH_SCK_GPIO_Port GPIOB
#define FLASH_MISO_Pin GPIO_PIN_4
#define FLASH_MISO_GPIO_Port GPIOB
#define FLASH_MOSI_Pin GPIO_PIN_5
#define FLASH_MOSI_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOE
#define TOUCH_IRQ_Pin GPIO_PIN_1
#define TOUCH_IRQ_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define	DEBOUNCE_BTN	5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
