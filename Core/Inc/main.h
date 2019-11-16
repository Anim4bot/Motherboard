/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SERVO_PWR_EARS_Pin GPIO_PIN_2
#define SERVO_PWR_EARS_GPIO_Port GPIOE
#define SERVO_PWR_NECK_Pin GPIO_PIN_3
#define SERVO_PWR_NECK_GPIO_Port GPIOE
#define SERVO_PWR_HOOD_Pin GPIO_PIN_4
#define SERVO_PWR_HOOD_GPIO_Port GPIOE
#define REG_3V3_EN_Pin GPIO_PIN_5
#define REG_3V3_EN_GPIO_Port GPIOE
#define LTC4015_ALERT_Pin GPIO_PIN_3
#define LTC4015_ALERT_GPIO_Port GPIOC
#define STM_FAN_Pin GPIO_PIN_1
#define STM_FAN_GPIO_Port GPIOA
#define STM_BUZZ_Pin GPIO_PIN_2
#define STM_BUZZ_GPIO_Port GPIOA
#define PI_SPI_CS_Pin GPIO_PIN_4
#define PI_SPI_CS_GPIO_Port GPIOA
#define PI_SPI_SCK_Pin GPIO_PIN_5
#define PI_SPI_SCK_GPIO_Port GPIOA
#define PI_SPI_MISO_Pin GPIO_PIN_6
#define PI_SPI_MISO_GPIO_Port GPIOA
#define PI_SPI_MOSI_Pin GPIO_PIN_7
#define PI_SPI_MOSI_GPIO_Port GPIOA
#define SIG_SYSTEM_Pin GPIO_PIN_5
#define SIG_SYSTEM_GPIO_Port GPIOC
#define PI_I2C_LINE_Pin GPIO_PIN_2
#define PI_I2C_LINE_GPIO_Port GPIOB
#define PI_I2C_BUSY_Pin GPIO_PIN_7
#define PI_I2C_BUSY_GPIO_Port GPIOE
#define PI_GPIO_B_Pin GPIO_PIN_8
#define PI_GPIO_B_GPIO_Port GPIOE
#define PI_GPIO_A_Pin GPIO_PIN_9
#define PI_GPIO_A_GPIO_Port GPIOE
#define PI_SIG_OFF_Pin GPIO_PIN_10
#define PI_SIG_OFF_GPIO_Port GPIOE
#define PI_SIG_ON_Pin GPIO_PIN_11
#define PI_SIG_ON_GPIO_Port GPIOE
#define PI_PWR_ON_OFF_Pin GPIO_PIN_12
#define PI_PWR_ON_OFF_GPIO_Port GPIOE
#define SW_PI_BOOT_Pin GPIO_PIN_14
#define SW_PI_BOOT_GPIO_Port GPIOE
#define SW_HOOD_Pin GPIO_PIN_15
#define SW_HOOD_GPIO_Port GPIOE
#define I2C_SCL_BOARD_Pin GPIO_PIN_10
#define I2C_SCL_BOARD_GPIO_Port GPIOB
#define I2C_SDA_BOARD_Pin GPIO_PIN_11
#define I2C_SDA_BOARD_GPIO_Port GPIOB
#define FLEX_OLED_CS_Pin GPIO_PIN_12
#define FLEX_OLED_CS_GPIO_Port GPIOB
#define FLEX_OLED_SCK_Pin GPIO_PIN_13
#define FLEX_OLED_SCK_GPIO_Port GPIOB
#define FLEX_OLED_RST_Pin GPIO_PIN_14
#define FLEX_OLED_RST_GPIO_Port GPIOB
#define FLEX_OLED_MOSI_Pin GPIO_PIN_15
#define FLEX_OLED_MOSI_GPIO_Port GPIOB
#define SERVOS_TX_STM_Pin GPIO_PIN_8
#define SERVOS_TX_STM_GPIO_Port GPIOD
#define SERVOS_RX_STM_Pin GPIO_PIN_9
#define SERVOS_RX_STM_GPIO_Port GPIOD
#define MAIN_SWITCH_Pin GPIO_PIN_10
#define MAIN_SWITCH_GPIO_Port GPIOD
#define EYES_RST_Pin GPIO_PIN_11
#define EYES_RST_GPIO_Port GPIOD
#define REG_5V_EN_Pin GPIO_PIN_12
#define REG_5V_EN_GPIO_Port GPIOD
#define HEAD_YAW_Pin GPIO_PIN_13
#define HEAD_YAW_GPIO_Port GPIOD
#define HEAD_PITCH_Pin GPIO_PIN_14
#define HEAD_PITCH_GPIO_Port GPIOD
#define PWM_HOOD_Pin GPIO_PIN_15
#define PWM_HOOD_GPIO_Port GPIOD
#define PWM_EAR_R_Pin GPIO_PIN_6
#define PWM_EAR_R_GPIO_Port GPIOC
#define PWM_EAR_L_Pin GPIO_PIN_7
#define PWM_EAR_L_GPIO_Port GPIOC
#define HODD_SW1_Pin GPIO_PIN_8
#define HODD_SW1_GPIO_Port GPIOC
#define HOOD_SW2_Pin GPIO_PIN_9
#define HOOD_SW2_GPIO_Port GPIOC
#define SW_USER_Pin GPIO_PIN_8
#define SW_USER_GPIO_Port GPIOA
#define LED_ACT_Pin GPIO_PIN_9
#define LED_ACT_GPIO_Port GPIOA
#define LED_B0_Pin GPIO_PIN_10
#define LED_B0_GPIO_Port GPIOA
#define LED_ERR_Pin GPIO_PIN_11
#define LED_ERR_GPIO_Port GPIOA
#define PSU_EN_Pin GPIO_PIN_12
#define PSU_EN_GPIO_Port GPIOA
#define BT_CS_Pin GPIO_PIN_0
#define BT_CS_GPIO_Port GPIOD
#define DOCK_SENS_Pin GPIO_PIN_1
#define DOCK_SENS_GPIO_Port GPIOD
#define BT_RESET_Pin GPIO_PIN_2
#define BT_RESET_GPIO_Port GPIOD
#define BT_TX_Pin GPIO_PIN_5
#define BT_TX_GPIO_Port GPIOD
#define BT_RX_Pin GPIO_PIN_6
#define BT_RX_GPIO_Port GPIOD
#define I2C_SCL_HEAD_Pin GPIO_PIN_6
#define I2C_SCL_HEAD_GPIO_Port GPIOB
#define I2C_SDA_HEAD_Pin GPIO_PIN_7
#define I2C_SDA_HEAD_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
