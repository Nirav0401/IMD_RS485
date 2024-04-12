/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_crc.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_usart4_rx;

/* USER CODE BEGIN PV */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart5, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_USART5_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RxData[32];
uint8_t TxData[8];
uint16_t Data[8];
char buffer[40];
int flag = 1;

void sendData (uint8_t *data)
{
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);  // enable the transmission
	HAL_UART_Transmit(&huart4, data, 8, 1000);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port,RS485_DE_Pin , GPIO_PIN_RESET);  // stop the transmission

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET); // for led indication
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Data[0] = RxData[3]<<8 | RxData[4];
	Data[1] = RxData[5]<<8 | RxData[6];
	Data[2] = RxData[7]<<8 | RxData[8];
	Data[3] = RxData[9]<<8 | RxData[10];
	Data[4] = RxData[11]<<8 | RxData[12];
	Data[5] = RxData[13]<<8 | RxData[14];
	Data[6] = RxData[15]<<8 | RxData[16];
	Data[7] = RxData[17]<<8 | RxData[18];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
}

void Insulation_Monitoring_ON (void)
{
	TxData[0] = 0x01;  // slave address
	TxData[1] = 0x06;  // Function code for Read Holding Registers
	TxData[2] = 0x01;
	TxData[3] = 0x02;
	TxData[4] = 0x00;
	TxData[5] = 0x11;
	TxData[6] = 0xE9;
	TxData[7] = 0xFA;

	sendData(TxData);
}

void Self_Test_ON (void)
{
	TxData[0] = 0x01;
	TxData[1] = 0x06;
	TxData[2] = 0x01;
	TxData[3] = 0x03;
	TxData[4] = 0x00;
	TxData[5] = 0x13;
	TxData[6] = 0x39;
	TxData[7] = 0xFB;

	sendData(TxData);
}

void Self_Test_OFF (void)
{
	TxData[0] = 0x01;
	TxData[1] = 0x06;
	TxData[2] = 0x01;
	TxData[3] = 0x03;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x78;
	TxData[7] = 0x36;

	sendData(TxData);
}

void Insulation_Resistance (void)  // DC voltage measurement
{
	TxData[0] = 0x01;
	TxData[1] = 0x03;
	TxData[2] = 0x00;
	TxData[3] = 0x10;
	TxData[4] = 0x00;
	TxData[5] = 0x04;
	TxData[6] = 0x45;
	TxData[7] = 0xCC;

	sendData(TxData);
}

void Insulation_Monitoring_OFF (void)
{
	TxData[0] = 0x01;
	TxData[1] = 0x06;
	TxData[2] = 0x01;
	TxData[3] = 0x02;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x29;
	TxData[7] = 0xF6;

	sendData(TxData);
}

void Isolation_Monitoring_OFF_BY_EM (void)
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET)
	{
		Insulation_Monitoring_OFF ();
		HAL_Delay(5);
	}
	else
	{
		Insulation_Resistance();
	    HAL_Delay(20);
	}
}
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
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, 32);

//  TxData[0] = 0x01;  // slave address
//  TxData[1] = 0x06;  // Function code for Read Holding Registers
//  TxData[1] = 0x04;  // Function code for Read Input Registers

  /*
   * The function code 0x03 means we are reading Holding Registers
   * The Register address ranges from 40001 - 50000
   * The register address we input can range from 0-9999 (0x00-0x270F)
   * Here 0 corresponds to the Address 40001 and 9999 corresponds to 50000
   * Although we can only read 125 registers sequentially at once
   */
//  TxData[2] = 0x01;  //00
//  TxData[3] = 0x03; // voltage current and power values done
  //The Register address will be 00000000 00000100 = 4 + 40001 = 40005

//  TxData[2] = 0;
//  TxData[3] = 0x01;
  //The Register address will be 00000000 00000001 = 1 +30001 = 30002

//  TxData[4] = 0x00;
//  TxData[5] = 0x13; //meter will start fetching data from 0061H address to total 1f consecutive data from the next addresses.
  // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes

//  uint16_t crc = crc16(TxData, 6);
//  TxData[6] = crc&0xFF;   // CRC LOW
//  TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

//  TxData[6] = 0x39;
//  TxData[7] = 0xFB;  // CRC HIGH // needed to set the baudrate I lost my 1 hour.. keep focus on your work.

    Insulation_Monitoring_ON ();
    HAL_Delay(100);
//   Self_Test_ON ();
//   HAL_Delay(5000);
//   Self_Test_OFF ();
//   HAL_Delay(5000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, 32);

	  Isolation_Monitoring_OFF_BY_EM ();

	  printf("***** IMD Values *****\n");
	  printf("\n");
	  uint16_t DC_Voltage = Data[0]; // dc voltage reading
	  int Value = (int)DC_Voltage;   // Convert hexadecimal to integer
	  float floatValue = Value / 10.0;  // Divide by 10 to get float value
	  sprintf(buffer, "DC_Voltage: %.2f\r\n", floatValue);
	  HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART

	  uint16_t Tot_Insulation_resistance = Data[1]; // total insulation resistance
	  int Value1 = (int)Tot_Insulation_resistance;   // Convert hexadecimal to integer
	  sprintf(buffer, "Tot_Insulation_resistance: %d\r\n", Value1);
	  HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART

	  uint16_t PosBus_Insulation_resistance = Data[2]; // Positive Bus insulation resistance
	  int Value2 = (int)PosBus_Insulation_resistance;   // Convert hexadecimal to integer
	  sprintf(buffer, "PosBus_Insulation_resistance: %d\r\n", Value2);
	  HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART

	  uint16_t NegBus_Insulation_resistance = Data[3]; // Negative Bus insulation resistance
	  int Value3 = (int)NegBus_Insulation_resistance;   // Convert hexadecimal to integer
	  sprintf(buffer, "NegBus_Insulation_resistance: %d\r\n", Value3);
	  HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART
	  printf("\n\n\n");

	  HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
