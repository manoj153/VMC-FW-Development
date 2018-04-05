/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#define AUTO_Pin GPIO_PIN_5
#define AUTO_GPIO_Port GPIOE
#define SW_R6_Pin GPIO_PIN_6
#define SW_R6_GPIO_Port GPIOE
#define SW_R5_Pin GPIO_PIN_13
#define SW_R5_GPIO_Port GPIOC
#define SW_R4_Pin GPIO_PIN_14
#define SW_R4_GPIO_Port GPIOC
#define SW_R3_Pin GPIO_PIN_15
#define SW_R3_GPIO_Port GPIOC
#define SW_R2_Pin GPIO_PIN_0
#define SW_R2_GPIO_Port GPIOC
#define SW_R1_Pin GPIO_PIN_1
#define SW_R1_GPIO_Port GPIOC
#define SW_8_Pin GPIO_PIN_2
#define SW_8_GPIO_Port GPIOC
#define SW_7_Pin GPIO_PIN_3
#define SW_7_GPIO_Port GPIOC
#define SW_6_Pin GPIO_PIN_1
#define SW_6_GPIO_Port GPIOA
#define PWM5_Pin GPIO_PIN_2
#define PWM5_GPIO_Port GPIOA
#define SW_5_Pin GPIO_PIN_3
#define SW_5_GPIO_Port GPIOA
#define SW_4_Pin GPIO_PIN_4
#define SW_4_GPIO_Port GPIOA
#define SW_3_Pin GPIO_PIN_5
#define SW_3_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_6
#define PWM3_GPIO_Port GPIOA
#define SW_2_Pin GPIO_PIN_7
#define SW_2_GPIO_Port GPIOA
#define SW_1_Pin GPIO_PIN_4
#define SW_1_GPIO_Port GPIOC
#define SW_F6_Pin GPIO_PIN_5
#define SW_F6_GPIO_Port GPIOC
#define SW_F5_Pin GPIO_PIN_0
#define SW_F5_GPIO_Port GPIOB
#define SW_F4_Pin GPIO_PIN_1
#define SW_F4_GPIO_Port GPIOB
#define SW_F3_Pin GPIO_PIN_2
#define SW_F3_GPIO_Port GPIOB
#define SW_F2_Pin GPIO_PIN_7
#define SW_F2_GPIO_Port GPIOE
#define SW_F1_Pin GPIO_PIN_8
#define SW_F1_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define OUT8_Pin GPIO_PIN_10
#define OUT8_GPIO_Port GPIOE
#define OUT7_Pin GPIO_PIN_11
#define OUT7_GPIO_Port GPIOE
#define OUT6_Pin GPIO_PIN_12
#define OUT6_GPIO_Port GPIOE
#define OUT5_Pin GPIO_PIN_13
#define OUT5_GPIO_Port GPIOE
#define OUT4_Pin GPIO_PIN_14
#define OUT4_GPIO_Port GPIOE
#define OUT3_Pin GPIO_PIN_15
#define OUT3_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_10
#define OUT2_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_11
#define OUT1_GPIO_Port GPIOB
#define SD6_Pin GPIO_PIN_12
#define SD6_GPIO_Port GPIOB
#define SD5_Pin GPIO_PIN_13
#define SD5_GPIO_Port GPIOB
#define SD4_Pin GPIO_PIN_14
#define SD4_GPIO_Port GPIOB
#define SD3_Pin GPIO_PIN_15
#define SD3_GPIO_Port GPIOB
#define SD2_Pin GPIO_PIN_8
#define SD2_GPIO_Port GPIOD
#define SD1_Pin GPIO_PIN_9
#define SD1_GPIO_Port GPIOD
#define IN_29_Pin GPIO_PIN_10
#define IN_29_GPIO_Port GPIOD
#define IN_28_Pin GPIO_PIN_11
#define IN_28_GPIO_Port GPIOD
#define PWM4_Pin GPIO_PIN_12
#define PWM4_GPIO_Port GPIOD
#define IN_27_Pin GPIO_PIN_13
#define IN_27_GPIO_Port GPIOD
#define IN_26_Pin GPIO_PIN_14
#define IN_26_GPIO_Port GPIOD
#define IN_25_Pin GPIO_PIN_15
#define IN_25_GPIO_Port GPIOD
#define IN_24_Pin GPIO_PIN_6
#define IN_24_GPIO_Port GPIOC
#define IN_23_Pin GPIO_PIN_7
#define IN_23_GPIO_Port GPIOC
#define IN_22_Pin GPIO_PIN_8
#define IN_22_GPIO_Port GPIOC
#define IN_21_Pin GPIO_PIN_9
#define IN_21_GPIO_Port GPIOC
#define IN_20_Pin GPIO_PIN_8
#define IN_20_GPIO_Port GPIOA
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define RX1_Pin GPIO_PIN_10
#define RX1_GPIO_Port GPIOA
#define IN_19_Pin GPIO_PIN_11
#define IN_19_GPIO_Port GPIOA
#define IN_18_Pin GPIO_PIN_12
#define IN_18_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_15
#define PWM2_GPIO_Port GPIOA
#define IN_16_Pin GPIO_PIN_10
#define IN_16_GPIO_Port GPIOC
#define IN_15_Pin GPIO_PIN_11
#define IN_15_GPIO_Port GPIOC
#define IN_14_Pin GPIO_PIN_12
#define IN_14_GPIO_Port GPIOC
#define IN_13_Pin GPIO_PIN_0
#define IN_13_GPIO_Port GPIOD
#define IN_12_Pin GPIO_PIN_1
#define IN_12_GPIO_Port GPIOD
#define IN_11_Pin GPIO_PIN_2
#define IN_11_GPIO_Port GPIOD
#define IN_10_Pin GPIO_PIN_3
#define IN_10_GPIO_Port GPIOD
#define IN_9_Pin GPIO_PIN_4
#define IN_9_GPIO_Port GPIOD
#define IN_8_Pin GPIO_PIN_5
#define IN_8_GPIO_Port GPIOD
#define IN_7_Pin GPIO_PIN_6
#define IN_7_GPIO_Port GPIOD
#define IN_6_Pin GPIO_PIN_7
#define IN_6_GPIO_Port GPIOD
#define IN_5_Pin GPIO_PIN_3
#define IN_5_GPIO_Port GPIOB
#define IN_4_Pin GPIO_PIN_4
#define IN_4_GPIO_Port GPIOB
#define IN_3_Pin GPIO_PIN_5
#define IN_3_GPIO_Port GPIOB
#define IN_2_Pin GPIO_PIN_6
#define IN_2_GPIO_Port GPIOB
#define IN_1_Pin GPIO_PIN_7
#define IN_1_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_8
#define PWM6_GPIO_Port GPIOB
#define IN_17_Pin GPIO_PIN_9
#define IN_17_GPIO_Port GPIOB

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
