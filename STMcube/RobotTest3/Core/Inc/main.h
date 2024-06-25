/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g0xx_hal.h"

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
#define Motor2b_PWM_Pin GPIO_PIN_11
#define Motor2b_PWM_GPIO_Port GPIOC
#define CE_BOOST_Pin GPIO_PIN_12
#define CE_BOOST_GPIO_Port GPIOC
#define OTG1_Pin GPIO_PIN_0
#define OTG1_GPIO_Port GPIOC
#define OE_MUX_Pin GPIO_PIN_1
#define OE_MUX_GPIO_Port GPIOC
#define S_MUX_Pin GPIO_PIN_2
#define S_MUX_GPIO_Port GPIOC
#define NEOPIXEL_DI_Pin GPIO_PIN_3
#define NEOPIXEL_DI_GPIO_Port GPIOC
#define ADC_PMID_MEAS_Pin GPIO_PIN_4
#define ADC_PMID_MEAS_GPIO_Port GPIOC
#define ADC_3V3_MEAS_Pin GPIO_PIN_5
#define ADC_3V3_MEAS_GPIO_Port GPIOC
#define ADC_VBUS_MEAS_Pin GPIO_PIN_0
#define ADC_VBUS_MEAS_GPIO_Port GPIOB
#define ADC_VBAT_MEAS_Pin GPIO_PIN_1
#define ADC_VBAT_MEAS_GPIO_Port GPIOB
#define STM_INT_OUT2_Pin GPIO_PIN_2
#define STM_INT_OUT2_GPIO_Port GPIOB
#define CTRL_SCL_Pin GPIO_PIN_10
#define CTRL_SCL_GPIO_Port GPIOB
#define CTRL_SDA_Pin GPIO_PIN_11
#define CTRL_SDA_GPIO_Port GPIOB
#define STM_INT_OUT1_Pin GPIO_PIN_12
#define STM_INT_OUT1_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_14
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_15
#define SERVO2_GPIO_Port GPIOB
#define Motor1a_PWM_Pin GPIO_PIN_8
#define Motor1a_PWM_GPIO_Port GPIOA
#define SYS_SCL_Pin GPIO_PIN_9
#define SYS_SCL_GPIO_Port GPIOA
#define Hall1_1_Pin GPIO_PIN_6
#define Hall1_1_GPIO_Port GPIOC
#define Hall1_1_EXTI_IRQn EXTI4_15_IRQn
#define Hall1_2_Pin GPIO_PIN_7
#define Hall1_2_GPIO_Port GPIOC
#define Hall1_2_EXTI_IRQn EXTI4_15_IRQn
#define Hall2_1_Pin GPIO_PIN_8
#define Hall2_1_GPIO_Port GPIOD
#define Hall2_1_EXTI_IRQn EXTI4_15_IRQn
#define Hall2_2_Pin GPIO_PIN_9
#define Hall2_2_GPIO_Port GPIOD
#define Hall2_2_EXTI_IRQn EXTI4_15_IRQn
#define SYS_SDA_Pin GPIO_PIN_10
#define SYS_SDA_GPIO_Port GPIOA
#define ON_PRESS_Pin GPIO_PIN_15
#define ON_PRESS_GPIO_Port GPIOA
#define ON_PRESS_EXTI_IRQn EXTI4_15_IRQn
#define INPUT_BUTTON_Pin GPIO_PIN_8
#define INPUT_BUTTON_GPIO_Port GPIOC
#define Motor1b_PWM_Pin GPIO_PIN_9
#define Motor1b_PWM_GPIO_Port GPIOC
#define INT_BOOST_Pin GPIO_PIN_4
#define INT_BOOST_GPIO_Port GPIOB
#define INT_BOOST_EXTI_IRQn EXTI4_15_IRQn
#define INT_ACCEL_Pin GPIO_PIN_5
#define INT_ACCEL_GPIO_Port GPIOB
#define INT_ACCEL_EXTI_IRQn EXTI4_15_IRQn
#define Motor2a_PWM_Pin GPIO_PIN_6
#define Motor2a_PWM_GPIO_Port GPIOB
#define EN_ESP_Pin GPIO_PIN_7
#define EN_ESP_GPIO_Port GPIOB
#define nSLEEP_MOTORS_Pin GPIO_PIN_8
#define nSLEEP_MOTORS_GPIO_Port GPIOB
#define TRIG_SONAR_Pin GPIO_PIN_9
#define TRIG_SONAR_GPIO_Port GPIOB
#define ECHO_SONAR_Pin GPIO_PIN_10
#define ECHO_SONAR_GPIO_Port GPIOC
#define ECHO_SONAR_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
