/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Nextion_Enabl_Pin GPIO_PIN_5
#define Nextion_Enabl_GPIO_Port GPIOE
#define Led_LANe_Pin GPIO_PIN_6
#define Led_LANe_GPIO_Port GPIOE
#define Led_WiFi_Pin GPIO_PIN_13
#define Led_WiFi_GPIO_Port GPIOC
#define Led_LTE_Pin GPIO_PIN_14
#define Led_LTE_GPIO_Port GPIOC
#define Led_BLE_Pin GPIO_PIN_15
#define Led_BLE_GPIO_Port GPIOC
#define ESP32_EN_Pin GPIO_PIN_3
#define ESP32_EN_GPIO_Port GPIOC
#define W5500_EXTI_Pin GPIO_PIN_4
#define W5500_EXTI_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOC
#define W5500_RST_Pin GPIO_PIN_5
#define W5500_RST_GPIO_Port GPIOC
#define Pin_Out_1_Pin GPIO_PIN_0
#define Pin_Out_1_GPIO_Port GPIOB
#define Pin_Out_3_Pin GPIO_PIN_1
#define Pin_Out_3_GPIO_Port GPIOB
#define Pin_Out_5_Pin GPIO_PIN_2
#define Pin_Out_5_GPIO_Port GPIOB
#define Pin_Out_7_Pin GPIO_PIN_7
#define Pin_Out_7_GPIO_Port GPIOE
#define Error_4_Pin GPIO_PIN_8
#define Error_4_GPIO_Port GPIOE
#define Error_3_Pin GPIO_PIN_9
#define Error_3_GPIO_Port GPIOE
#define Error_2_Pin GPIO_PIN_10
#define Error_2_GPIO_Port GPIOE
#define Error_1_Pin GPIO_PIN_11
#define Error_1_GPIO_Port GPIOE
#define RE_Pin GPIO_PIN_12
#define RE_GPIO_Port GPIOE
#define UART_MLTPX_INH_Pin GPIO_PIN_13
#define UART_MLTPX_INH_GPIO_Port GPIOE
#define UART_MLTPX_B_Pin GPIO_PIN_14
#define UART_MLTPX_B_GPIO_Port GPIOE
#define UART_MLTPX_A_Pin GPIO_PIN_15
#define UART_MLTPX_A_GPIO_Port GPIOE
#define Pin_Out_9_Pin GPIO_PIN_12
#define Pin_Out_9_GPIO_Port GPIOB
#define Pin_In_2_Pin GPIO_PIN_8
#define Pin_In_2_GPIO_Port GPIOD
#define Pin_In_1_Pin GPIO_PIN_9
#define Pin_In_1_GPIO_Port GPIOD
#define Pin_Out_8_Pin GPIO_PIN_10
#define Pin_Out_8_GPIO_Port GPIOD
#define Pin_Out_6_Pin GPIO_PIN_11
#define Pin_Out_6_GPIO_Port GPIOD
#define Pin_Out_4_Pin GPIO_PIN_12
#define Pin_Out_4_GPIO_Port GPIOD
#define Pin_Out_2_Pin GPIO_PIN_13
#define Pin_Out_2_GPIO_Port GPIOD
#define LTE_Enable_Pin GPIO_PIN_15
#define LTE_Enable_GPIO_Port GPIOD
#define BLE_Reset_Pin GPIO_PIN_8
#define BLE_Reset_GPIO_Port GPIOA
#define SPI_3_CS_Pin GPIO_PIN_15
#define SPI_3_CS_GPIO_Port GPIOA
#define Pair_Status_Pin GPIO_PIN_3
#define Pair_Status_GPIO_Port GPIOD
#define Mode_Switch_Pin GPIO_PIN_4
#define Mode_Switch_GPIO_Port GPIOD
#define WP_Pin GPIO_PIN_8
#define WP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
