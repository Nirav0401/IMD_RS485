/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32l0xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_usart4_rx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART4)
  {
  /* USER CODE BEGIN USART4_MspInit 0 */

  /* USER CODE END USART4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART4 GPIO Configuration
    PA15     ------> USART4_DE
    PC10     ------> USART4_TX
    PC11     ------> USART4_RX
    */
    GPIO_InitStruct.Pin = RS485_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_USART4;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RS485_TX_Pin|RS485_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_USART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART4 DMA Init */
    /* USART4_RX Init */
    hdma_usart4_rx.Instance = DMA1_Channel2;
    hdma_usart4_rx.Init.Request = DMA_REQUEST_12;
    hdma_usart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart4_rx.Init.Mode = DMA_NORMAL;
    hdma_usart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart4_rx);

    /* USART4 interrupt Init */
    HAL_NVIC_SetPriority(USART4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART4_5_IRQn);
  /* USER CODE BEGIN USART4_MspInit 1 */

  /* USER CODE END USART4_MspInit 1 */
  }
  else if(huart->Instance==USART5)
  {
  /* USER CODE BEGIN USART5_MspInit 0 */

  /* USER CODE END USART5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART5_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART5 GPIO Configuration
    PB3     ------> USART5_TX
    PB4     ------> USART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_USART5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART5 interrupt Init */
    HAL_NVIC_SetPriority(USART4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART4_5_IRQn);
  /* USER CODE BEGIN USART5_MspInit 1 */

  /* USER CODE END USART5_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART4)
  {
  /* USER CODE BEGIN USART4_MspDeInit 0 */

  /* USER CODE END USART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART4_CLK_DISABLE();

    /**USART4 GPIO Configuration
    PA15     ------> USART4_DE
    PC10     ------> USART4_TX
    PC11     ------> USART4_RX
    */
    HAL_GPIO_DeInit(RS485_DE_GPIO_Port, RS485_DE_Pin);

    HAL_GPIO_DeInit(GPIOC, RS485_TX_Pin|RS485_RX_Pin);

    /* USART4 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);

    /* USART4 interrupt DeInit */
  /* USER CODE BEGIN USART4:USART4_5_IRQn disable */
    /**
    * Uncomment the line below to disable the "USART4_5_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USART4_5_IRQn); */
  /* USER CODE END USART4:USART4_5_IRQn disable */

  /* USER CODE BEGIN USART4_MspDeInit 1 */

  /* USER CODE END USART4_MspDeInit 1 */
  }
  else if(huart->Instance==USART5)
  {
  /* USER CODE BEGIN USART5_MspDeInit 0 */

  /* USER CODE END USART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART5_CLK_DISABLE();

    /**USART5 GPIO Configuration
    PB3     ------> USART5_TX
    PB4     ------> USART5_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);

    /* USART5 interrupt DeInit */
  /* USER CODE BEGIN USART5:USART4_5_IRQn disable */
    /**
    * Uncomment the line below to disable the "USART4_5_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USART4_5_IRQn); */
  /* USER CODE END USART5:USART4_5_IRQn disable */

  /* USER CODE BEGIN USART5_MspDeInit 1 */

  /* USER CODE END USART5_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
