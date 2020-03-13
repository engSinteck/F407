/**
 ******************************************************************************
  * @file    bsp_driver_sd.h for F4 (based on stm324x9i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for 
  *          the bsp_driver_sd.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_MMC_H
#define __STM32F4_MMC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "fatfs_platform.h"

/* Exported types --------------------------------------------------------*/ 
/** 
  * @brief MMC Card information structure
  */
#define BSP_MMC_CardInfo HAL_MMC_CardInfoTypeDef

/* Exported constants --------------------------------------------------------*/ 
/**
  * @brief  MMC status structure definition
  */     
#define   MMMC_OK                        ((uint8_t)0x00)
#define   MMMC_ERROR                     ((uint8_t)0x01)

/** 
  * @brief  MMC transfer state definition
  */     
#define   MMC_TRANSFER_OK                ((uint8_t)0x00)
#define   MMC_TRANSFER_BUSY              ((uint8_t)0x01)

#define MMC_PRESENT               ((uint8_t)0x01)
#define MMC_NOT_PRESENT           ((uint8_t)0x00)
#define MMC_DATATIMEOUT           ((uint32_t)100000000)

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */ 
#else
/* USER CODE BEGIN BSP_H_CODE */
/* Exported functions --------------------------------------------------------*/   
uint8_t BSP_MMC_Init(void);
uint8_t BSP_MMC_ITConfig(void);
void    BSP_MMC_DetectIT(void);
void    BSP_MMC_DetectCallback(void);
uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr);
void BSP_MMC_IRQHandler(void);
void BSP_MMC_DMA_Tx_IRQHandler(void);
void BSP_MMC_DMA_Rx_IRQHandler(void);
uint8_t BSP_MMC_GetCardState(void);
void    BSP_MMC_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);
uint8_t BSP_MMC_IsDetected(void);

/* These functions can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void    BSP_MMC_AbortCallback(void);
void    BSP_MMC_WriteCpltCallback(void);
void    BSP_MMC_ReadCpltCallback(void);
/* USER CODE END BSP_H_CODE */
#endif
   
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_MMC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
