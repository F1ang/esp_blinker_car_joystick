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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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

/* Private define ------------------------------------------------------------*/

#define Start_Stop_Pin             GPIO_PIN_0
#define Start_Stop_GPIO_Port       GPIOE
#define Start_Stop_EXTI_IRQn       EXTI0_IRQn
#define BTN_START_STOP_IRQHandler  EXTI0_IRQHandler
#define BTN_START_STOP_EXTI_LINE   LL_EXTI_LINE_0

#define M1_PWM_WH_Pin              GPIO_PIN_7
#define M1_PWM_WH_GPIO_Port        GPIOI
#define M1_PWM_VH_Pin              GPIO_PIN_6
#define M1_PWM_VH_GPIO_Port        GPIOI
#define M1_PWM_UH_Pin              GPIO_PIN_5
#define M1_PWM_UH_GPIO_Port        GPIOI
#define M1_PWM_WL_Pin              GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port        GPIOH
#define M1_PWM_UL_Pin              GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port        GPIOH
#define M1_PWM_VL_Pin              GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port        GPIOH

#define M1_HALL_H3_Pin             GPIO_PIN_12
#define M1_HALL_H3_GPIO_Port       GPIOH
#define M1_HALL_H2_Pin             GPIO_PIN_11
#define M1_HALL_H2_GPIO_Port       GPIOH
#define M1_HALL_H1_Pin             GPIO_PIN_10
#define M1_HALL_H1_GPIO_Port       GPIOH

#define M1_CURR_AMPL_U_Pin         GPIO_PIN_6
#define M1_CURR_AMPL_U_GPIO_Port   GPIOA
#define M1_CURR_AMPL_W_Pin         GPIO_PIN_1
#define M1_CURR_AMPL_W_GPIO_Port   GPIOB
#define M1_CURR_AMPL_V_Pin         GPIO_PIN_0
#define M1_CURR_AMPL_V_GPIO_Port   GPIOB

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
