/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define INH_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define INH_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)

#define MULTIPLEXER_A_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)
#define MULTIPLEXER_A_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)

#define MULTIPLEXER_B_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define MULTIPLEXER_B_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)

#define RE_LOW    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET)
#define RE_HIGH 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET)


#define BLE_Mode_Data				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_RESET)
#define BLE_Mode_AT					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_SET)


#define BLE_RESET_ON		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_RESET)
#define BLE_RESET_OFF		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET)


#define ESP32_ON		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_SET)
#define ESP32_OFF	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_RESET)


#define LTE_Enable  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_SET)
#define LTE_Disable HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_RESET)

#define W5500_ON	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5, GPIO_PIN_SET)
#define W5500_OFF		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t Received_rx_byte[400] = {0};
uint8_t buffer_counter = 0;
uint8_t rx_buffer[250] = {0};

uint8_t transmit_buffer[50] = {0};


uint8_t Pair_status = 0;



const char *at_cwlap = "AT+CWLAP";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); // Turn on 5V regulator
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
	
	
	ESP32_OFF;
	HAL_Delay(500);
	ESP32_ON;
	HAL_Delay(500);
	HAL_Delay(2000);
	
	HAL_Delay(2000);
	
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = 0x0D;
	transmit_buffer[3] = 0x0A;
	
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = 'E';
	transmit_buffer[3] = '0';
	transmit_buffer[4] = 0x0D;
	transmit_buffer[5] = 0x0A;
	
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();
	
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'S';
	transmit_buffer[4] = 'L';
	transmit_buffer[5] = 'E';
	transmit_buffer[6] = 'E';
	transmit_buffer[7] = 'P';
	transmit_buffer[8] = '=';
	transmit_buffer[9] = '0';
	transmit_buffer[10] = 0x0D;
	transmit_buffer[11] = 0x0A;
	
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();
	
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'W';
	transmit_buffer[5] = 'M';
	transmit_buffer[6] = 'O';
	transmit_buffer[7] = 'D';
	transmit_buffer[8] = 'E';
	transmit_buffer[9] = '=';
	transmit_buffer[10] = '1';
	transmit_buffer[11] = 0x0D;
	transmit_buffer[12] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();
	
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'W';
	transmit_buffer[5] = 'L';
	transmit_buffer[6] = 'A';
	transmit_buffer[7] = 'P';
	transmit_buffer[8] = '=';
	transmit_buffer[9] = '"';
	transmit_buffer[10] = 'M';
	transmit_buffer[11] = 'E';
	transmit_buffer[12] = 'A';
	transmit_buffer[13] = 'M';
	transmit_buffer[14] = 'A';
	transmit_buffer[15] = ' ';
	transmit_buffer[16] = 'H';
	transmit_buffer[17] = 'A';
	transmit_buffer[18] = 'R';
	transmit_buffer[19] = 'D';
	transmit_buffer[20] = 'W';
	transmit_buffer[21] = 'A';
	transmit_buffer[22] = 'R';
	transmit_buffer[23] = 'E';
	transmit_buffer[24] = '"';
	transmit_buffer[25] = 0x0D;
	transmit_buffer[26] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(5000);
	__NOP();
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'W';
	transmit_buffer[5] = 'Q';
	transmit_buffer[6] = 'A';
	transmit_buffer[7] = 'P';
	transmit_buffer[8] = 0x0D;
	transmit_buffer[9] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();

	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	

	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'W';
	transmit_buffer[5] = 'J';
	transmit_buffer[6] = 'A';
	transmit_buffer[7] = 'P';
	transmit_buffer[8] = '=';
	transmit_buffer[9] = '"';
	transmit_buffer[10] = 'M';
	transmit_buffer[11] = 'E';
	transmit_buffer[12] = 'A';
	transmit_buffer[13] = 'M';
	transmit_buffer[14] = 'A';
	transmit_buffer[15] = ' ';
	transmit_buffer[16] = 'H';
	transmit_buffer[17] = 'A';
	transmit_buffer[18] = 'R';
	transmit_buffer[19] = 'D';
	transmit_buffer[20] = 'W';
	transmit_buffer[21] = 'A';
	transmit_buffer[22] = 'R';
	transmit_buffer[23] = 'E';
	transmit_buffer[24] = '"';
	transmit_buffer[25] = ',';
	transmit_buffer[26] = '"';
	transmit_buffer[27] = 'm';
	transmit_buffer[28] = 'e';
	transmit_buffer[29] = 'a';
	transmit_buffer[30] = 'm';
	transmit_buffer[31] = 'a';
	transmit_buffer[32] = '2';
	transmit_buffer[33] = '0';
	transmit_buffer[34] = '2';
	transmit_buffer[35] = '0';
	transmit_buffer[36] = '2';
	transmit_buffer[37] = '0';
	transmit_buffer[38] = '"';
	transmit_buffer[39] = 0x0D;
	transmit_buffer[40] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(10000);
	__NOP();
	
	
	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'I';
	transmit_buffer[5] = 'F';
	transmit_buffer[6] = 'S';
	transmit_buffer[7] = 'R';
	transmit_buffer[8] = 0x0D;
	transmit_buffer[9] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();

	
	memset(Received_rx_byte,0,250);
	memset(transmit_buffer,0,50);
	buffer_counter = 0;
	__NOP();
	
	
	/*
	transmit_buffer[0] = 'A';
	transmit_buffer[1] = 'T';
	transmit_buffer[2] = '+';
	transmit_buffer[3] = 'C';
	transmit_buffer[4] = 'W';
	transmit_buffer[5] = 'Q';
	transmit_buffer[6] = 'A';
	transmit_buffer[7] = 'P';
	transmit_buffer[8] = 0x0D;
	transmit_buffer[9] = 0x0A;
	
	__NOP();
	HAL_UART_Transmit(&huart2,transmit_buffer,strlen((char*)transmit_buffer),100);
	HAL_Delay(500);
	__NOP();*/
	
	
	

	
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Nextion_Enabl_Pin|Led_LANe_Pin|Pin_Out_7_Pin|Error_4_Pin
                          |Error_3_Pin|Error_2_Pin|Error_1_Pin|RE_Pin
                          |UART_MLTPX_INH_Pin|UART_MLTPX_B_Pin|UART_MLTPX_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led_WiFi_Pin|Led_LTE_Pin|Led_BLE_Pin|GPIO_PIN_2
                          |ESP32_EN_Pin|W5500_CS_Pin|W5500_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Pin_Out_1_Pin|Pin_Out_3_Pin|Pin_Out_5_Pin|Pin_Out_9_Pin
                          |WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Pin_Out_8_Pin|Pin_Out_6_Pin|Pin_Out_4_Pin|Pin_Out_2_Pin
                          |LTE_Enable_Pin|Mode_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLE_Reset_Pin|SPI_3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Nextion_Enabl_Pin Led_LANe_Pin Pin_Out_7_Pin Error_4_Pin
                           Error_3_Pin Error_2_Pin Error_1_Pin RE_Pin
                           UART_MLTPX_INH_Pin UART_MLTPX_B_Pin UART_MLTPX_A_Pin */
  GPIO_InitStruct.Pin = Nextion_Enabl_Pin|Led_LANe_Pin|Pin_Out_7_Pin|Error_4_Pin
                          |Error_3_Pin|Error_2_Pin|Error_1_Pin|RE_Pin
                          |UART_MLTPX_INH_Pin|UART_MLTPX_B_Pin|UART_MLTPX_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_WiFi_Pin Led_LTE_Pin Led_BLE_Pin PC2
                           ESP32_EN_Pin W5500_CS_Pin W5500_RST_Pin */
  GPIO_InitStruct.Pin = Led_WiFi_Pin|Led_LTE_Pin|Led_BLE_Pin|GPIO_PIN_2
                          |ESP32_EN_Pin|W5500_CS_Pin|W5500_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_EXTI_Pin */
  GPIO_InitStruct.Pin = W5500_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(W5500_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Pin_Out_1_Pin Pin_Out_3_Pin Pin_Out_5_Pin Pin_Out_9_Pin
                           WP_Pin */
  GPIO_InitStruct.Pin = Pin_Out_1_Pin|Pin_Out_3_Pin|Pin_Out_5_Pin|Pin_Out_9_Pin
                          |WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Pin_In_2_Pin Pin_In_1_Pin */
  GPIO_InitStruct.Pin = Pin_In_2_Pin|Pin_In_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Pin_Out_8_Pin Pin_Out_6_Pin Pin_Out_4_Pin Pin_Out_2_Pin
                           LTE_Enable_Pin Mode_Switch_Pin */
  GPIO_InitStruct.Pin = Pin_Out_8_Pin|Pin_Out_6_Pin|Pin_Out_4_Pin|Pin_Out_2_Pin
                          |LTE_Enable_Pin|Mode_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_Reset_Pin SPI_3_CS_Pin */
  GPIO_InitStruct.Pin = BLE_Reset_Pin|SPI_3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pair_Status_Pin */
  GPIO_InitStruct.Pin = Pair_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pair_Status_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Received_rx_byte[buffer_counter] = rx_buffer[0];
	buffer_counter++;
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1); //restart interrupt
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
