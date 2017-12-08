/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Radar_A9_Pin GPIO_PIN_0
#define Radar_A9_GPIO_Port GPIOC
#define Radar_A10_Pin GPIO_PIN_1
#define Radar_A10_GPIO_Port GPIOC
#define Radar_Trig_Pin GPIO_PIN_2
#define Radar_Trig_GPIO_Port GPIOC
#define Bat_Level_Pin GPIO_PIN_3
#define Bat_Level_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Radar_A0_Pin GPIO_PIN_1
#define Radar_A0_GPIO_Port GPIOA
#define Radar_A1_Pin GPIO_PIN_2
#define Radar_A1_GPIO_Port GPIOA
#define Radar_A2_Pin GPIO_PIN_3
#define Radar_A2_GPIO_Port GPIOA
#define Radar_A3_Pin GPIO_PIN_4
#define Radar_A3_GPIO_Port GPIOA
#define Radar_A4_Pin GPIO_PIN_5
#define Radar_A4_GPIO_Port GPIOA
#define Radar_A5_Pin GPIO_PIN_6
#define Radar_A5_GPIO_Port GPIOA
#define Radar_A6_Pin GPIO_PIN_7
#define Radar_A6_GPIO_Port GPIOA
#define Radar_A7_Pin GPIO_PIN_0
#define Radar_A7_GPIO_Port GPIOB
#define Radar_A8_Pin GPIO_PIN_1
#define Radar_A8_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define Wireless_Power_Pin GPIO_PIN_11
#define Wireless_Power_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_Out_Pin GPIO_PIN_3
#define LED_Out_GPIO_Port GPIOB
#define Radar_Power_1a_Pin GPIO_PIN_4
#define Radar_Power_1a_GPIO_Port GPIOB
#define Radar_Power_1b_Pin GPIO_PIN_5
#define Radar_Power_1b_GPIO_Port GPIOB
#define Radar_Power_2a_Pin GPIO_PIN_6
#define Radar_Power_2a_GPIO_Port GPIOB
#define Radar_Power_2b_Pin GPIO_PIN_7
#define Radar_Power_2b_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
