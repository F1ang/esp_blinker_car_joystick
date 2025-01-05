/**
  ******************************************************************************
  * @file    stm32f4xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
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
  * @ingroup STM32F4xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "stm32f4xx_ll_exti.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
  * @{
  */
/* USER CODE BEGIN PRIVATE */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000) // HAL时基分频器,用于生成1Khz

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */
/**
  * 函数功能: ADC1/ADC2中断处理函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void ADC_IRQHandler(void)
{
  if(LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    // Clear Flags
    /* 注入转换中断 */
    ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
    TSK_HighFrequencyTask();// 执行高频任务(FOC控制)
  }
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
/**
  * 函数功能: 定时器更新事件中断
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void TIMx_UP_M1_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
  R3F4XX_TIMx_UP_IRQHandler(&PWM_Handle_M1);

}
 

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
/**
  * 函数功能: 速度传感器定时器中断
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: TIM5中断事件,更新事件需要记录下来,用于修改预分频值
  */
void SPD_TIM_M1_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);
  }
  
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M1);
  }
}

/*Start here***********************************************************/

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */

void HardFault_Handler(void)
{
  TSK_HardwareFaultTask();
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }

}
/**
  * 函数功能: 系统滴答定时器中断
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 2Khz,执行电机控制任务,同时生成1Khz的时基用于HAL库
  */
void SysTick_Handler(void)
{
  static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler(); // 1Khz(1ms)的时基
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;  
  MC_RunMotorControlTasks(); // 状态机:电机调度 中频任务
}

/*GUI, this section is present only if start/stop button is enabled*/
/**
  * @brief  This function handles Button IRQ on PIN PE0.
  * @param  None
  * @retval None
  */
/**
  * 函数功能: 按键启动关闭
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 外部中断执行电机启动和关闭
  */
void BTN_START_STOP_IRQHandler (void)
{
  LL_EXTI_ClearFlag_0_31 (BTN_START_STOP_EXTI_LINE);
  UI_HandleStartStopButton_cb ();
}


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
