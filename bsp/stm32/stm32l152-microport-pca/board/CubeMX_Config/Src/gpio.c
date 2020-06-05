/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
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
  HAL_GPIO_WritePin(GPIOC, SENSOR_PWR_EN_Pin|STF_POS_DETECT_PWR_EN_Pin|VOICE_PWR_EN_Pin|STF_5V_IN_EN_Pin 
                          |BT_PWR_EN_Pin|ALL_5V_PWR_EN_Pin|YH_ZJ_PWR_EN_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VREF_EN_Pin|VOICE_SPI1_SSB_Pin|FLASH_PWR_EN_Pin|FLASH_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SYS_PWR_EN_Pin|LED_Y_Pin|SYS_PWR_ADC_EN_Pin|LED_G_Pin 
                          |YH_ZJ_PWR_EN_RIGHT_Pin|LED_R_Pin|STF_MOTOR_IN1_Pin|STF_MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RTC_I2C_SCK_Pin|RTC_I2C_SDA_Pin|OLED_SPI2_CS_Pin|SZ_MOTOR_IN1_Pin 
                          |SZ_MOTOR_IN2_Pin|OLED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M_PWR_EN_Pin|ALL_MOTOR_PWR_EN_Pin|SLAVER_NRST_Pin|M_VIBRATOR_EN_Pin 
                          |FLASH_HOLD_Pin|FLASH_WP_Pin|OLED_PWR_EN_Pin|STF_MOTOR_SLEEP_Pin 
                          |OLED_DC_Pin|SZ_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_WKUP_GPIO_Port, S_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin 
                           PEPin */
  GPIO_InitStruct.Pin = KEY_BACK_Pin|KEY_LEFT_Pin|KEY_RIGHT_Pin|KEY_CONFIRM_Pin 
                          |PCA_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = KEY_START_Pin|STF_POS_DETECT_A_Pin|STF_POS_DETECT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin 
                           PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SENSOR_PWR_EN_Pin|STF_POS_DETECT_PWR_EN_Pin|VOICE_PWR_EN_Pin|STF_5V_IN_EN_Pin 
                          |BT_PWR_EN_Pin|ALL_5V_PWR_EN_Pin|YH_ZJ_PWR_EN_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = VREF_EN_Pin|VOICE_SPI1_SSB_Pin|FLASH_PWR_EN_Pin|FLASH_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = VOICE_SPI1_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VOICE_SPI1_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BUBBLE_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUBBLE_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = VOICE_INT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VOICE_INT8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin 
                           PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = SYS_PWR_EN_Pin|LED_Y_Pin|SYS_PWR_ADC_EN_Pin|LED_G_Pin 
                          |YH_ZJ_PWR_EN_RIGHT_Pin|LED_R_Pin|STF_MOTOR_IN1_Pin|STF_MOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin PBPin */
  GPIO_InitStruct.Pin = RTC_I2C_SCK_Pin|RTC_I2C_SDA_Pin|OLED_SPI2_CS_Pin|SZ_MOTOR_IN1_Pin 
                          |SZ_MOTOR_IN2_Pin|OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin 
                           PDPin PDPin PDPin PDPin 
                           PDPin PDPin */
  GPIO_InitStruct.Pin = M_PWR_EN_Pin|ALL_MOTOR_PWR_EN_Pin|SLAVER_NRST_Pin|M_VIBRATOR_EN_Pin 
                          |FLASH_HOLD_Pin|FLASH_WP_Pin|OLED_PWR_EN_Pin|STF_MOTOR_SLEEP_Pin 
                          |OLED_DC_Pin|SZ_MOTOR_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = YH_ZJ_LEFT_Pin|YH_ZJ_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = S_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin */
  GPIO_InitStruct.Pin = S_RST_STA_Pin|SZ_HALL_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = SZ_HALL_A_Pin|SZDJ_SUDU_Pin|STF_HALL_A_Pin|STF_HALL_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
