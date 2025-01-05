/**
  ******************************************************************************
  * @file    motorcontrol.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem initialization functions.
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
  */ 
#include "main.h"
#include "mc_tuning.h"

#include "mc_tasks.h"
#include "ui_task.h"
#include "mc_config.h"
#include "parameters_conversion.h"


MCI_Handle_t* pMCI[NBR_OF_MOTORS];
MCT_Handle_t* pMCT[NBR_OF_MOTORS];
//uint32_t wConfig[NBR_OF_MOTORS] = {UI_CONFIG_M1};

/**
 * @brief Initializes and configures the Motor Control Subsystem
 *
 *  This function initializes and configures all the structures and components needed
 * for the Motor Control subsystem required by the Application. It expects that
 * all the peripherals needed for Motor Control purposes are already configured but
 * that their interrupts are not enabled yet. 
 *
 * CubeMX calls this function after all peripherals initializations and 
 * before the NVIC is configured
 */
/**
  * 函数功能: 电机控制初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 初始和配置了所有应用需要的结构体和组件,该函数应该在初始化所有需要的外设但没有使能中断之前调用
  *           CubeMX 在初始化所有外设之后调用这个函数
  */
void MX_MotorControl_Init(void) 
{
  /* 重新配置系统滴答定时器的时基为 500 us */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYS_TICK_FREQUENCY);
  
  /* 初始化Motor Control Subsystem */
  MCboot(pMCI,pMCT);// pMCI 用于控制接口(Interface),pMCT 则是实际控制实体
  
  /* 锁定所有使用的引脚,写保护 */
  mc_lock_pins();
  
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
