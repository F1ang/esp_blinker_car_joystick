/**
  ******************************************************************************
  * @file    parameters_conversion_f4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the Parameter conversion on the base
  *          of stdlib F4xx for the first drive
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F4XX_H
#define __PARAMETERS_CONVERSION_F4XX_H

#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "drive_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/

#define SYSCLK_FREQ           168000000uL           // MPU系统时钟频率
#define TIM_CLOCK_DIVIDER     1                     // 定时器时钟频率 
#define ADV_TIM_CLK_MHz       168/TIM_CLOCK_DIVIDER // 高级控制定时器频率(MHz)
#define ADC_CLK_MHz           21                    // ADC时钟频率(MHz)
#define HALL_TIM_CLK          84000000uL            // 霍尔传感器定时器时钟频率(Hz)

/*************************  IRQ Handler Mapping  *********************/
#define TIMx_UP_M1_IRQHandler TIM8_UP_TIM13_IRQHandler

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/

#define TRIG_CONV_LATENCY_NS	100
#define SAMPLING_TIME_NS      (3 * 1000uL/ADC_CLK_MHz) /// 3个周期的采样时间(ns)
#define TW_BEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS + TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1u)

#endif /*__PARAMETERS_CONVERSION_F4XX_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
