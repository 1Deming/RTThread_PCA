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
#include "stm32l1xx_hal.h"

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
#define KEY_BACK_Pin GPIO_PIN_3
#define KEY_BACK_GPIO_Port GPIOE
#define KEY_LEFT_Pin GPIO_PIN_4
#define KEY_LEFT_GPIO_Port GPIOE
#define KEY_RIGHT_Pin GPIO_PIN_5
#define KEY_RIGHT_GPIO_Port GPIOE
#define KEY_CONFIRM_Pin GPIO_PIN_6
#define KEY_CONFIRM_GPIO_Port GPIOE
#define KEY_START_Pin GPIO_PIN_13
#define KEY_START_GPIO_Port GPIOC
#define SENSOR_PWR_EN_Pin GPIO_PIN_0
#define SENSOR_PWR_EN_GPIO_Port GPIOC
#define STF_POS_DETECT_PWR_EN_Pin GPIO_PIN_1
#define STF_POS_DETECT_PWR_EN_GPIO_Port GPIOC
#define STF_POS_DETECT_A_Pin GPIO_PIN_2
#define STF_POS_DETECT_A_GPIO_Port GPIOC
#define STF_POS_DETECT_B_Pin GPIO_PIN_3
#define STF_POS_DETECT_B_GPIO_Port GPIOC
#define VREF_EN_Pin GPIO_PIN_1
#define VREF_EN_GPIO_Port GPIOA
#define M_TX2_BT_RX_Pin GPIO_PIN_2
#define M_TX2_BT_RX_GPIO_Port GPIOA
#define M_RX2_BT_TX_Pin GPIO_PIN_3
#define M_RX2_BT_TX_GPIO_Port GPIOA
#define VOICE_SPI1_SSB_Pin GPIO_PIN_4
#define VOICE_SPI1_SSB_GPIO_Port GPIOA
#define VOICE_SPI1_SCK_Pin GPIO_PIN_5
#define VOICE_SPI1_SCK_GPIO_Port GPIOA
#define VOICE_SPI1_MISO_Pin GPIO_PIN_6
#define VOICE_SPI1_MISO_GPIO_Port GPIOA
#define VOICE_SPI1_MOSI_Pin GPIO_PIN_7
#define VOICE_SPI1_MOSI_GPIO_Port GPIOA
#define VOICE_SPI1_RDY_Pin GPIO_PIN_4
#define VOICE_SPI1_RDY_GPIO_Port GPIOC
#define VOICE_PWR_EN_Pin GPIO_PIN_5
#define VOICE_PWR_EN_GPIO_Port GPIOC
#define BUBBLE_DETECT_Pin GPIO_PIN_2
#define BUBBLE_DETECT_GPIO_Port GPIOB
#define VOICE_INT8_Pin GPIO_PIN_8
#define VOICE_INT8_GPIO_Port GPIOE
#define SYS_PWR_EN_Pin GPIO_PIN_9
#define SYS_PWR_EN_GPIO_Port GPIOE
#define PCA_KEY_Pin GPIO_PIN_10
#define PCA_KEY_GPIO_Port GPIOE
#define LED_Y_Pin GPIO_PIN_11
#define LED_Y_GPIO_Port GPIOE
#define SYS_PWR_ADC_EN_Pin GPIO_PIN_12
#define SYS_PWR_ADC_EN_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOE
#define YH_ZJ_PWR_EN_RIGHT_Pin GPIO_PIN_14
#define YH_ZJ_PWR_EN_RIGHT_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOE
#define RTC_I2C_SCK_Pin GPIO_PIN_10
#define RTC_I2C_SCK_GPIO_Port GPIOB
#define RTC_I2C_SDA_Pin GPIO_PIN_11
#define RTC_I2C_SDA_GPIO_Port GPIOB
#define OLED_SPI2_CS_Pin GPIO_PIN_12
#define OLED_SPI2_CS_GPIO_Port GPIOB
#define OLED_SPI2_SCLK_Pin GPIO_PIN_13
#define OLED_SPI2_SCLK_GPIO_Port GPIOB
#define S_MCU_3_3V_ADC_Pin GPIO_PIN_14
#define S_MCU_3_3V_ADC_GPIO_Port GPIOB
#define OLED_SPI2_SDIN_Pin GPIO_PIN_15
#define OLED_SPI2_SDIN_GPIO_Port GPIOB
#define M_PWR_EN_Pin GPIO_PIN_10
#define M_PWR_EN_GPIO_Port GPIOD
#define ALL_MOTOR_PWR_EN_Pin GPIO_PIN_11
#define ALL_MOTOR_PWR_EN_GPIO_Port GPIOD
#define SLAVER_NRST_Pin GPIO_PIN_14
#define SLAVER_NRST_GPIO_Port GPIOD
#define M_VIBRATOR_EN_Pin GPIO_PIN_15
#define M_VIBRATOR_EN_GPIO_Port GPIOD
#define STF_5V_IN_EN_Pin GPIO_PIN_6
#define STF_5V_IN_EN_GPIO_Port GPIOC
#define BT_PWR_EN_Pin GPIO_PIN_7
#define BT_PWR_EN_GPIO_Port GPIOC
#define ALL_5V_PWR_EN_Pin GPIO_PIN_8
#define ALL_5V_PWR_EN_GPIO_Port GPIOC
#define YH_ZJ_PWR_EN_LEFT_Pin GPIO_PIN_9
#define YH_ZJ_PWR_EN_LEFT_GPIO_Port GPIOC
#define YH_ZJ_LEFT_Pin GPIO_PIN_8
#define YH_ZJ_LEFT_GPIO_Port GPIOA
#define YH_ZJ_RIGHT_Pin GPIO_PIN_11
#define YH_ZJ_RIGHT_GPIO_Port GPIOA
#define FLASH_PWR_EN_Pin GPIO_PIN_12
#define FLASH_PWR_EN_GPIO_Port GPIOA
#define S_WKUP_Pin GPIO_PIN_2
#define S_WKUP_GPIO_Port GPIOH
#define FLASH_NSS_Pin GPIO_PIN_15
#define FLASH_NSS_GPIO_Port GPIOA
#define FLASH_SPI3_SCK_Pin GPIO_PIN_10
#define FLASH_SPI3_SCK_GPIO_Port GPIOC
#define FLASH_SPI3_MISO_Pin GPIO_PIN_11
#define FLASH_SPI3_MISO_GPIO_Port GPIOC
#define FLASH_SPI3_MOSI_Pin GPIO_PIN_12
#define FLASH_SPI3_MOSI_GPIO_Port GPIOC
#define FLASH_HOLD_Pin GPIO_PIN_0
#define FLASH_HOLD_GPIO_Port GPIOD
#define FLASH_WP_Pin GPIO_PIN_1
#define FLASH_WP_GPIO_Port GPIOD
#define OLED_PWR_EN_Pin GPIO_PIN_2
#define OLED_PWR_EN_GPIO_Port GPIOD
#define STF_MOTOR_SLEEP_Pin GPIO_PIN_3
#define STF_MOTOR_SLEEP_GPIO_Port GPIOD
#define S_RST_STA_Pin GPIO_PIN_4
#define S_RST_STA_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_5
#define OLED_DC_GPIO_Port GPIOD
#define SZ_MOTOR_SLEEP_Pin GPIO_PIN_6
#define SZ_MOTOR_SLEEP_GPIO_Port GPIOD
#define SZ_HALL_B_Pin GPIO_PIN_7
#define SZ_HALL_B_GPIO_Port GPIOD
#define SZ_HALL_A_Pin GPIO_PIN_3
#define SZ_HALL_A_GPIO_Port GPIOB
#define SZ_MOTOR_IN1_Pin GPIO_PIN_4
#define SZ_MOTOR_IN1_GPIO_Port GPIOB
#define SZ_MOTOR_IN2_Pin GPIO_PIN_5
#define SZ_MOTOR_IN2_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_6
#define OLED_RES_GPIO_Port GPIOB
#define SZDJ_SUDU_Pin GPIO_PIN_7
#define SZDJ_SUDU_GPIO_Port GPIOB
#define STF_HALL_A_Pin GPIO_PIN_8
#define STF_HALL_A_GPIO_Port GPIOB
#define STF_HALL_B_Pin GPIO_PIN_9
#define STF_HALL_B_GPIO_Port GPIOB
#define STF_MOTOR_IN1_Pin GPIO_PIN_0
#define STF_MOTOR_IN1_GPIO_Port GPIOE
#define STF_MOTOR_IN2_Pin GPIO_PIN_1
#define STF_MOTOR_IN2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
