/**
 ******************************************************************************
  * @file    bsp_driver_mmc.c for F4 (based on stm324x9i_eval_sd.c)
 * @brief   This file includes a generic uSD card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Functions generated as weak: they can be overriden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
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

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_mmc.h"

/* Extern variables ---------------------------------------------------------*/ 

extern MMC_HandleTypeDef hmmc;

/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_MMC_Init(void)
{
  uint8_t mmc_state = MMMC_OK;
  /* Check if the SD card is plugged in the slot */
  if (BSP_MMC_IsDetected() != MMC_PRESENT)
  {
    return MMMC_ERROR;
  }
  /* HAL SD initialization */
  mmc_state = HAL_MMC_Init(&hmmc);
  /* Configure SD Bus width (4 bits mode selected) */
  if (mmc_state == MMMC_OK)
  {
    /* Enable wide operation */
    if (HAL_MMC_ConfigWideBusOperation(&hmmc, SDIO_BUS_WIDE_4B) != HAL_OK)
    {
      mmc_state = MMMC_ERROR;
    }
  }

  return mmc_state;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/* USER CODE BEGIN InterruptMode */
/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0
  */
uint8_t BSP_MMC_ITConfig(void)
{  
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */
  
  return (uint8_t)0;
}

/** @brief  SD detect IT treatment
  */
void BSP_MMC_DetectIT(void)
{
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */
}
/* USER CODE END InterruptMode */

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t mmc_state = MSD_OK;

  if (HAL_MMC_ReadBlocks(&hmmc, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    mmc_state = MSD_ERROR;
  }

  return mmc_state;
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t mmc_state = MSD_OK;

  if (HAL_MMC_WriteBlocks(&hmmc, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    mmc_state = MSD_ERROR;
  }

  return mmc_state;
}

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @retval SD status
  */
uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t mmc_state = MSD_OK;
  
  /* Read block(s) in DMA transfer mode */
  if (HAL_MMC_ReadBlocks_DMA(&hmmc, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    mmc_state = MSD_ERROR;
  }
  
  return mmc_state;
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t mmc_state = MSD_OK;
  
  /* Write block(s) in DMA transfer mode */
  if (HAL_MMC_WriteBlocks_DMA(&hmmc, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
  {
    mmc_state = MSD_ERROR;
  }
  
  return mmc_state;
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given SD card. 
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t mmc_state = MSD_OK;

  if (HAL_MMC_Erase(&hmmc, StartAddr, EndAddr) != HAL_OK)
  {
    mmc_state = MSD_ERROR;
  }

  return mmc_state;
}

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_MMC_GetCardState(void)
{
  return ((HAL_MMC_GetCardState(&hmmc) == HAL_MMC_CARD_TRANSFER ) ? MMC_TRANSFER_OK : MMC_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None 
  */
void BSP_MMC_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
  /* Get SD card Information */
  HAL_MMC_GetCardInfo(&hmmc, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_MMC_AbortCallback(SD_HandleTypeDef *hmmc)
{
  BSP_MMC_AbortCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_MMC_TxCpltCallback(SD_HandleTypeDef *hmmc)
{
  BSP_MMC_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_MMC_RxCpltCallback(SD_HandleTypeDef *hmmc)
{
  BSP_MMC_ReadCpltCallback();
}

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP SD Abort callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
void BSP_MMC_AbortCallback(void)
{

}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
//void BSP_SD_WriteCpltCallback(void)
//{
//
//}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
//void BSP_SD_ReadCpltCallback(void)
//{
//
//}
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_MMC_IsDetected(void)
{
  __IO uint8_t status = MMC_PRESENT;

  if (BSP_PlatformIsDetected() == 0x0) 
  {
    status = MMC_NOT_PRESENT;
  }

  return status;
}

/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
/* USER CODE END AdditionalCode */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
