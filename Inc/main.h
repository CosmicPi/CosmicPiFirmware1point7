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
#define hvpsu_cl1_Pin GPIO_PIN_1
#define hvpsu_cl1_GPIO_Port GPIOC
#define hvpsu_cl2_Pin GPIO_PIN_2
#define hvpsu_cl2_GPIO_Port GPIOC
#define shaped_out_1_Pin GPIO_PIN_0
#define shaped_out_1_GPIO_Port GPIOA
#define shaped_out_2_Pin GPIO_PIN_1
#define shaped_out_2_GPIO_Port GPIOA
#define gpsrx_Pin GPIO_PIN_2
#define gpsrx_GPIO_Port GPIOA
#define gpstx_Pin GPIO_PIN_3
#define gpstx_GPIO_Port GPIOA
#define biasfb1_Pin GPIO_PIN_4
#define biasfb1_GPIO_Port GPIOA
#define biasfb2_Pin GPIO_PIN_5
#define biasfb2_GPIO_Port GPIOA
#define inj_led_Pin GPIO_PIN_6
#define inj_led_GPIO_Port GPIOA
#define pwr_led_Pin GPIO_PIN_7
#define pwr_led_GPIO_Port GPIOA
#define flag_Pin GPIO_PIN_1
#define flag_GPIO_Port GPIOB
#define hvpsu_cs2_Pin GPIO_PIN_7
#define hvpsu_cs2_GPIO_Port GPIOC
#define hvpsu_cs1_Pin GPIO_PIN_8
#define hvpsu_cs1_GPIO_Port GPIOC
#define evt_led_Pin GPIO_PIN_8
#define evt_led_GPIO_Port GPIOA
#define GPSPPS_Pin GPIO_PIN_15
#define GPSPPS_GPIO_Port GPIOA
#define strigout_b_Pin GPIO_PIN_11
#define strigout_b_GPIO_Port GPIOC
#define strigout_a_Pin GPIO_PIN_12
#define strigout_a_GPIO_Port GPIOC
#define trig_out_Pin GPIO_PIN_3
#define trig_out_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
