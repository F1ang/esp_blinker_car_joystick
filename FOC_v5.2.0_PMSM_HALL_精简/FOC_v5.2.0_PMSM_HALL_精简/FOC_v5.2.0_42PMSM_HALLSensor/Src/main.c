
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "motorcontrol.h"
#include "ui_task.h" 
#include "key/bsp_key.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* Private function prototypes -----------------------------------------------*/
extern void test_RampCtrl(void);// 测试用函数,非FOC库

/**
  * 函数功能: main主函数入口
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* 复位所有外设,初始化Flash 接口和系统滴答定时器. */
  HAL_Init();

  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化所有外设 */
  KEY_GPIO_Init(); // 所有按键初始化为轮询模式
  MX_GPIO_Init();  // KEY1为中断模式,用于启动/停止电机
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_MotorControl_Init();

  /* 初始化中断 */
  MX_NVIC_Init();

  
  /* Infinite loop */
  HAL_Delay(500);
  MC_StartMotor1();
  HAL_Delay(2000);
  UI_SPDRampCtrl(2000,1000);// 1000RPM, 1000ms
  HAL_Delay(2000);
//  MC_StopMotor1();
  while (1)
  {
    if(KEY5_StateRead() == KEY_DOWN)
    {
      UI_SPDRampCtrl(-1000,1000);     // 速度控制
      MC_StartMotor1();
    }
    if(KEY4_StateRead() == KEY_DOWN)
    {
      MC_StopMotor1();
    }
  }
}

/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* 配置PLL 得到CPU的系统时钟,168MHz */
  /* 8M晶振  ( (8/4) * 168 ) / 2 = 168MHz  PLLQ实际上没有使用到*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**配置AHB,APB总线时钟频率和定时器时钟频率
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  /* Enables the Clock Security System */
  HAL_RCC_EnableCSS();

  /**Configure the Systick interrupt time */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /*  选择HCLK为滴答定时器的时钟 168MHz */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 中断优先级 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 4, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(Start_Stop_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(Start_Stop_EXTI_IRQn);
}

/**
  * 函数功能: ADC外设初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 使用CH6,CH8,CH9为注入通道(A6,B0,B1相电流采样引脚),使用不连续采样模式
              CH10,CH13为规则通道(C0,C3电压,温度采样引脚),使用扫描模式
  */
static void MX_ADC1_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
     配置ADC的全局特性,包括时钟,分辨率,数据对齐方向,转换通道数量
  */
  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;     //时钟=84/4 = 21MHz 
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;           // 12位分辨率(转换时间15个时钟周期)
  hadc1.Init.ScanConvMode          = ENABLE;                       // 扫描模式   √
  hadc1.Init.ContinuousConvMode    = DISABLE;                      // 连续采样   ×
  hadc1.Init.DiscontinuousConvMode = DISABLE;                      // 不连续采样 ×
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;// 无外部触发
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;           // 软件启动
  hadc1.Init.DataAlign             = ADC_DATAALIGN_LEFT;           // 左对齐
  hadc1.Init.NbrOfConversion       = 2;                            // 转换通道 2
  hadc1.Init.DMAContinuousRequests = DISABLE;                      // DMA传输请求 ×
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;          // 单次转换完成标记
  HAL_ADC_Init(&hadc1);


  /* 配置注入通道的采样顺序和采样时间 */
  sConfigInjected.InjectedChannel           = ADC_CHANNEL_6;// CH6
  sConfigInjected.InjectedRank              = 1; // 采样顺序     
  sConfigInjected.InjectedNbrOfConversion   = 3; // 总的转换通道数量 
  sConfigInjected.InjectedSamplingTime      = ADC_SAMPLETIME_3CYCLES;// 采样时间,3个周期,单次转换时间是15个周期
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING; // 外部信号上升沿触发(指ADC的外部)
  sConfigInjected.ExternalTrigInjecConv     = ADC_EXTERNALTRIGINJECCONV_T8_CC4;     // 触发源:TIM8 CH4的比较事件
  sConfigInjected.AutoInjectedConv          = DISABLE;        // 自动注入 ×
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;     // 不连续采样 √
  sConfigInjected.InjectedOffset            = 0;              // 数据偏移值
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);


  /* Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;// CH8 
  sConfigInjected.InjectedRank    = 2;            // 采样顺序//其他配置与CH6一样
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;// CH9 
  sConfigInjected.InjectedRank = 3;               // 采样顺序//其他配置与CH6一样
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

}

/**
  * 函数功能: ADC外设初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 使用CH6,CH8,CH9为注入通道(A6,B0,B1相电流采样引脚),使用不连续采样模式         
  */
static void MX_ADC2_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

  /* 配置ADC的全局特性,包括时钟,分辨率,数据对齐方向,转换通道数量 */
  hadc2.Instance                   = ADC2;
  hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;       //时钟=84/4 = 21MHz 
  hadc2.Init.Resolution            = ADC_RESOLUTION_12B;             // 12位分辨率(转换时间15个时钟周期)
  hadc2.Init.ScanConvMode          = ENABLE;                         // 扫描模式   √
  hadc2.Init.ContinuousConvMode    = DISABLE;                        // 连续采样   ×
  hadc2.Init.DiscontinuousConvMode = DISABLE;                        // 不连续采样 ×
  hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  // 无外部触发
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;             // 软件启动
  hadc2.Init.DataAlign             = ADC_DATAALIGN_LEFT;             // 左对齐
  hadc2.Init.NbrOfConversion       = 1;                              // 转换通道 1
  hadc2.Init.DMAContinuousRequests = DISABLE;                        // DMA传输请求 ×
  hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;            // 单次转换完成标记
  HAL_ADC_Init(&hadc2);

  /* 配置注入通道的采样顺序和采样时间 */
  sConfigInjected.InjectedChannel           = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank              = 1;
  sConfigInjected.InjectedNbrOfConversion   = 3;
  sConfigInjected.InjectedSamplingTime      = ADC_SAMPLETIME_3CYCLES;              // 采样时间,3个周期,单次转换时间是15个周期
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;// 外部信号上升沿触发(指ADC的外部)
  sConfigInjected.ExternalTrigInjecConv     = ADC_EXTERNALTRIGINJECCONV_T8_CC4;    // 触发源:TIM8 CH4的比较事件
  sConfigInjected.AutoInjectedConv          = DISABLE;        // 自动注入 ×
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;     // 不连续采样 √
  sConfigInjected.InjectedOffset            = 0;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

  /* 配置注入通道CH8的采用顺序 */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = 2;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);


  /* 配置注入通道CH9的采用顺序 */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 3;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);


}

/**
  * 函数功能: TIIM5配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 将TIM5配置为霍尔传感器模式
  */static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_HallSensor_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance           = TIM5;                   //
  htim5.Init.Prescaler     = 0;                      //
  htim5.Init.CounterMode   = TIM_COUNTERMODE_UP;     // 向上计数
  htim5.Init.Period        = M1_HALL_TIM_PERIOD;     //
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);


  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);


  sConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler      = TIM_ICPSC_DIV1;
  sConfig.IC1Filter         = M1_HALL_IC_FILTER;
  sConfig.Commutation_Delay = 0;                ///无延迟
  HAL_TIMEx_HallSensor_Init(&htim5, &sConfig);
  

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;//TIM5 _OC2 触发输出
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);


}

/**
  * 函数功能: TIM8初始化配置
  * 输入参数: 无 
  * 返 回 值: 无 
  * 说    明: 无
  */static void MX_TIM8_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance               = TIM8;                           // 
  htim8.Init.Prescaler         = ((TIM_CLOCK_DIVIDER) - 1);      // 中心对齐模式1,递减计数的时候才会触发比较中断事件
  htim8.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1; //168,000,000 / 16,000 = 10500 ,16kHz
  htim8.Init.Period            = ((PWM_PERIOD_CYCLES) / 2);      // 84MHz用于生成死区
  htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV2;         // 1 两个脉冲中断一次
  htim8.Init.RepetitionCounter = (REP_COUNTER);                  // 重复计数器1
  HAL_TIM_Base_Init(&htim8);
  

  HAL_TIM_PWM_Init(&htim8);

  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_TRIGGER;  // 从模式
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;            // 触发源 ITR1 链接到TIM2的TRGO 
  HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;              // 默认值
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE; // 主从模式 ×
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

  /* PWM输出通道配置 */
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;           // PWM1模式,先高电平再低电平
  sConfigOC.Pulse        = 0;                         
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;       // 输出极性
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;      // 
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;        // 快速模式 ×
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;     // 空闲状态 RESET
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  ;HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);


  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);// CH2

  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);// CH3

  sConfigOC.OCMode = TIM_OCMODE_PWM2;// PWM2模式,先低电平再高电平
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));// 比较值,触发ADC采样的时刻点
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
  
  /* 刹车与死区配置 */
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;   // OSSR √
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;   // OSSI √
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_1;   // LOCK Level 1  对BDTR.DTG/BKE/BKP/AOE , CR2.OISx/OISxN 执行写保护
  sBreakDeadTimeConfig.DeadTime         = ((DEAD_TIME_COUNTS) / 2);// 死区时间
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE; // 刹车 ×
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;//AOE × 
  HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

  HAL_TIM_MspPostInit(&htim8);

}

/**
  * 函数功能: 引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 配置启动/停止按钮,并使能GPIO时钟
  */
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();  

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
