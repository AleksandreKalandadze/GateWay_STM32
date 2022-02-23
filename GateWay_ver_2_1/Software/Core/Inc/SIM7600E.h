#ifndef _SIM7600E_H
#define _SIM7600E_H


/**
  ******************************************************************************
  * @file    SIM7600E.h
  * @author  Aleksandre Kalandadze
  * @brief   Defining configuration of SIM7600E module control hardware        
  ******************************************************************************

  */
	

#include "stm32f4xx_hal.h"


extern UART_HandleTypeDef huart6;   //define used UART


#define SIM7600E_ENABLE_PORT   GPIOD
#define SIM7600E_ENABLE_PIN		GPIO_PIN_15

#define SIM7600E_On   HAL_GPIO_WritePin(SIM7600E_ENABLE_PORT,SIM7600E_ENABLE_PIN, GPIO_PIN_SET)
#define SIM7600E_Off	HAL_GPIO_WritePin(SIM7600E_ENABLE_PORT,SIM7600E_ENABLE_PIN, GPIO_PIN_RESET)






#endif
