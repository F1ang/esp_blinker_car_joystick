/**
  ******************************************************************************
  * @file    r3_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F302x8
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
  *           + three shunt current sesnsing
  *           + space vector modulation function
  *           + ADC sampling function
  *
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

/* Includes ------------------------------------------------------------------*/
#include "r3_f4xx_pwm_curr_fdbk.h"

/** @defgroup r3_f4XX_pwm_curr_fdbk R3 F4xx PWM & Current Feedback
 *
 * @brief STM32F4, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F4 MCU
 * and using a three shunt resistors current sensing topology.
 *
 */
/* Private defines -----------------------------------------------------------*/
/* #define SAMPLING_POINT_DEBUG */
/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)
#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u)
#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))
#define ADC_SR_MASK                ((uint32_t) (0xC))
#define NB_CONVERSIONS    16u
#define PHASE_A_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIaChannel) << 15)
#define PHASE_B_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIbChannel) << 15)
#define PHASE_C_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIcChannel) << 15)
#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u

/* Private function prototypes -----------------------------------------------*/
static void R3F4XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl );
static uint16_t R3F4XX_WriteTIMRegisters( PWMC_Handle_t * pHdl );
static void R3F4XX_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static void R3F4XX_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );


/**
  * @brief  It initializes TIM, ADC, GPIO, DMA and NVIC for current reading and
  *         PWM generation in three shunt configuration using STM32F4XX
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: F4的三相采样外设初始化
  * 输入参数: pHandle
  * 返 回 值: 无
  * 说    明: 这是FOC库的兼容性部分,主要是为了兼容不容的板子,如果是使用F1,则是R3F1
  */
void R3F4XX_Init( PWMC_R3_F4_Handle_t * pHandle )
{
  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {
    /* 重新初始化定时器的周期,比较值 */
    R3F4XX_TIMxInit( pHandle->pParams_str->TIMx, &pHandle->_Super );

    /* debug的时候停止定时器 */
    if ( pHandle->pParams_str->TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );;
    }
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
    }

    /* ADC1 and ADC2 registers configuration ---------------------------------*/
    /* Enable ADC1 and ADC2 */
    LL_ADC_Enable( ADC1 ); // 使能ADC1
    LL_ADC_Enable( ADC2 ); // 使能ADC2

    /* ADC1 Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS( ADC1 );// 先清除中断标记
    LL_ADC_EnableIT_JEOS( ADC1 );

    /* To pre-compute the following variables is used the configuration already
     performed on ADC1 and ADC2. This means that ADC configurations run from here
     on out will be overwritten during the context switching.*/
    if ( pHandle->pParams_str->TIMx == TIM1 )// 设置ADC的触发源-> TIM1_CH4
    {
      /* The following two variables are pre-computed and used to disable/enable
       the ADC injected external trigger during the context switching. */
      pHandle->wADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4) */
      pHandle->wADCTriggerSet   = pHandle->wADCTriggerUnSet |
                                  0x00100000u; /* JEXTEN = 01b (Enable), JEXTSEL = 0000b (TIM1_CC4) */
    }
    else// 设置ADC的触发源-> TIM8_CH4
    {
      /* The following two variables are pre-computed and used to disable/enable
       the ADC injected external trigger during the context switching. */
      pHandle->wADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4 "dummy") */
      pHandle->wADCTriggerSet   = pHandle->wADCTriggerUnSet |
                                  0x001E0000u; /* JEXTEN = 01b (Enable), JEXTSEL = 1110b (TIM8_CC4) */
    }
  }
}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 定时器初始化
  * 输入参数: 
  * 返 回 值: 无
  * 说    明: 使能通道预装载,配置周期值和计数器初始值
  */
static void R3F4XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;

  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter( TIMx );// 停止定时器,由TIM2触发同步启动

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );// CCMR1 使能预装载
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE( TIMx );

  /* 重复计数器的配置,用来调整FOC调制率 */
  if ( pHandle->pParams_str->bFreqRatio == 2u ) // 2
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->bRepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1u );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );//重复计数器设置为3,4个PWM周期
      }
    }

    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  }
  else /* bFreqRatio equal to 1 or 3 */ 
  {
    if ( pHandle->_Super.bMotor == M1 )
    {
      LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );//计数器初始值为周期的一半
    }
  }
}

/**
  * @brief  It stores into the component's handle the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowin into the
  *         motor
  * @param  pHandle handler of the current instance of the PWM component
  */
/**
  * 函数功能: 电流读取校准
  * 输入参数: 
  * 返 回 值: 无
  * 说    明: 无
  */
void R3F4XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;

  pHandle->bIndex = 0u;

  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= TIMxCCER_MASK; // 关闭全部PWM通道

  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationAB;// 获取AB相电流

  pHandle->wADC1Channel = PHASE_A_MSK;// A相ADC通道
  pHandle->wADC2Channel = PHASE_B_MSK;// B相ADC通道

  R3F4XX_SwitchOnPWM( &pHandle->_Super );// 启动输出

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < ( NB_CONVERSIONS ) )// 等待转换完成
  {
    if ( LL_TIM_IsEnabledIT_UPDATE( TIMx ) )    // 定时器更新一次就退出,否则就是
    {}                                          // 等待bIndex = NB_CONVERSIONS 
    else                                        // 转换16个数据之后才退出循环
    {
      pHandle->bIndex = NB_CONVERSIONS;
    }
  }

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex = 0u;

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationC;// 获取C相电流

  pHandle->wADC1Channel = PHASE_C_MSK;// C相ADC通道
  pHandle->wADC2Channel = PHASE_C_MSK;// C相ADC通道

  R3F4XX_SwitchOnPWM( &pHandle->_Super );// 启动输出

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < ( NB_CONVERSIONS / 2u ) )// 等待转换完成
  { 
    if ( LL_TIM_IsEnabledIT_UPDATE( TIMx ) )         // C相只需要8次即可
    {}
    else
    {
      pHandle->bIndex = NB_CONVERSIONS;
    }
  }

  pHandle->wPhaseAOffset >>= 3;// 右移3位
  pHandle->wPhaseBOffset >>= 3;// 左对齐模式,右三位是无效位
  pHandle->wPhaseCOffset >>= 3;// 累加值
  // 在其他函数中累加
  
  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_GetPhaseCurrents;// 获取三相电流

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  TIMx->CCMR1 &= 0xF7F7u; // 禁用CCRx预装载
  TIMx->CCMR2 &= 0xF7F7u; // 然后设置比较值,为50%占空比
  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->Half_PWMPeriod );

  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;// 使能预装载 OCxPE = 1 CCRx预装载
  TIMx->CCMR2 |= 0x0808u;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= TIMxCCER_MASK_CH123;
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
/**
  * 函数功能: 获取相电流
  * 输入参数: @pStator_Currents 用于存放转子电流Ia,Ib
  * 返 回 值: 无
  * 说    明: 无
  */

void R3F4XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  int32_t wAux;
  uint8_t bSector;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  BB_REG_BIT_CLR ( &pHandle->pParams_str->TIMx->CCER, TIM_CCER_CC4E_Pos );

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  bSector = pHandle->_Super.hSector; // 获取当前扇区

  switch ( bSector )
  {
    case SECTOR_4:// 不同扇区,但是同样的绕组
    case SECTOR_5:// 采样Ia Ib
      /* Current on Phase C is not accessible     */// 读取A B相电流
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( ADC1->JDR1 );              // 获取ADC1值,获取Ia
      wAux *= 2;                                     // 左移一位,去掉符号位
      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - wAux;

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )// 限制数值范围
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( ADC2->JDR1 );               // ADC2采样 获取Ib
      wAux *= 2;                                   // 左移一位,去掉符号位
      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - wAux;

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )// 限制数值范围
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:// 采样Ib,Ic计算Ia
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( ADC1->JDR1 ); // 采样Ib
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - wAux;

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }

      /* Ia = -Ic -Ib */
      wAux = ( int32_t )( ADC2->JDR1 ); // 采样Ic , 计算Ia
      wAux *= 2;
      wAux -= ( int32_t )pHandle->wPhaseCOffset;
      wAux -= ( int32_t )pStator_Currents->qI_Component2;

      /* Saturation of Ia */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:// 采样Ia,Ic,计算Ib
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( ADC1->JDR1 );
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - wAux;

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ib = -Ic -Ia */
      wAux = ( int32_t )( ADC2->JDR1 );
      wAux *= 2;
      wAux -= ( int32_t )pHandle->wPhaseCOffset;
      wAux -= ( int32_t )pStator_Currents->qI_Component1;

      /* Saturation of Ib */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }
      break;

    default:
      break;
  }
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
/**
  * 函数功能: 读取AB相电流校准
  * 输入参数: @pStator_Currents 用于存放电流
  * 返 回 值: {0,0} 
  * 说    明: 累加ADC采样值,不做计算
  */
static void R3F4XX_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  BB_REG_BIT_CLR ( &pHandle->pParams_str->TIMx->CCER, TIM_CCER_CC4E_Pos );

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  if ( pHandle->bIndex < NB_CONVERSIONS )
  {
    pHandle-> wPhaseAOffset += ADC1->JDR1;
    pHandle-> wPhaseBOffset += ADC2->JDR1;
    pHandle->bIndex++;
  }
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC input before to enable
  *         the offset computation.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
/**
  * 函数功能: 读取AB相电流校准
  * 输入参数: @pStator_Currents 用于存放电流
  * 返 回 值: {0,0} 
  * 说    明: 两个通道同时采集C相电流,所以采样次数少一半
  */
static void R3F4XX_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  BB_REG_BIT_CLR ( &pHandle->pParams_str->TIMx->CCER, TIM_CCER_CC4E_Pos );

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  if ( pHandle->bIndex < NB_CONVERSIONS / 2u )
  {
    pHandle-> wPhaseCOffset += ADC1->JDR1;
    pHandle-> wPhaseCOffset += ADC2->JDR1;
    pHandle->bIndex++;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 导通下桥臂
  * 输入参数: @pHdl
  * 返 回 值: 无
  * 说    明: 设置比较值为0,然后开启所有通道,
  */
void R3F4XX_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, 0 );
  LL_TIM_OC_SetCompareCH2( TIMx, 0 );
  LL_TIM_OC_SetCompareCH3( TIMx, 0 );

  /* Wait until next update */
  //等待下一个更新事件,使CCRx配置生效
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  return;
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 启动所有输出
  * 输入参数: @pHdl
  * 返 回 值: 无
  * 说    明: MOE=1,启动输出
  */
void R3F4XX_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* It clears ADCs JSTRT and JEOC bits */
  ADC1->SR &= ~ADC_SR_MASK;
  ADC2->SR &= ~ADC_SR_MASK;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_EnableIT_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  return;
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 关断所有输出
  * 输入参数: @pHdl
  * 返 回 值: 无
  * 说    明: 关断所有PWM通道,然后MOE=0,再配置CCER寄存器
  */
void R3F4XX_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  TIMx->CCER &= ( uint16_t )( ~TIMxCCER_MASK_CH123 );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {
    if ( LL_TIM_IsEnabledIT_UPDATE( TIMx ) )
    {
      break;
    }
  }

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  TIMx->CCER |= TIMxCCER_MASK_CH123;

  return;
}

/**
  * @brief  It stores into 'this' object variables the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowin into the
  *         motor
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 设置定时器的比较值
  * 输入参数: @pHdl
  * 返 回 值: 如果在FOC计算期间发生更新事件则返回MC_FOC_DURATION,否则返回MC_NO_ERROR
  * 说    明: 用于在FOC计算结束之后修改定时器比较值,同时判断是否发生的更新事件
  */
static uint16_t R3F4XX_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if ( pHandle->bSoFOC != 0u )
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  return hAux;
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  this related object of class CPWMC
  * @retval none
  */
/**
  * 函数功能: 在扇区1采样
  * 输入参数: @pHdl
  * 返 回 值: 
  * 说    明: 
  */
uint16_t R3F4XX_SetADCSampPointSect1( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 1 (i.e phase A duty cycle) */
  /* 检测采样点是否在PWM的中间位置 */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4; // 固定采样AB相

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;// PWM中间点采样

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 2.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F4XX_SetADCSampPointSect2( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 2 (i.e phase B duty cycle) */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 3.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F4XX_SetADCSampPointSect3( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 3 (i.e phase B duty cycle) */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 4.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F4XX_SetADCSampPointSect4( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 4 (i.e phase C duty cycle) */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 5.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F4XX_SetADCSampPointSect5( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 5 (i.e phase C duty cycle) */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 6.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F4XX_SetADCSampPointSect6( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  uint32_t adcTrig = LL_ADC_INJ_TRIG_EXT_RISING;
  PWMC_R3_F4_Handle_t * pHandle = ( PWMC_R3_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* reset ADC trigger edge */
  pHandle->wADCTriggerSet &= ~LL_ADC_INJ_TRIG_EXT_RISINGFALLING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 6 (i.e phase A duty cycle) */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        adcTrig = LL_ADC_INJ_TRIG_EXT_FALLING;
        
        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

  
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  /* set ADC trigger edge */
  pHandle->wADCTriggerSet |= adcTrig;

  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3F4XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
/**
  * 函数功能: 定时器更新中断
  * 输入参数: @pHandle
  * 返 回 值: 无
  * 说    明: 
  */
void * R3F4XX_TIMx_UP_IRQHandler( PWMC_R3_F4_Handle_t * pHandle )
{
  uint32_t wADCInjFlags;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pHandle->bSoFOC = 1u;// 

  wADCInjFlags = ( ADC1-> SR ) & ADC_SR_MASK; 

  if ( wADCInjFlags == CONV_STARTED ) // 如果已经开始了AD转换
  {                                   // 等待完成
    do
    {
      wADCInjFlags = ( ADC1-> SR ) & ADC_SR_MASK;
    }
    while ( wADCInjFlags != CONV_FINISHED );
  }
  else if ( wADCInjFlags == FLAGS_CLEARED )// 已经转换完成
  {
    while ( ( TIMx->CNT ) < ( pHandle->pParams_str->Tw ) )//  Tw = 0
    {}
    wADCInjFlags = ( ADC1-> SR ) & ADC_SR_MASK; 

    if ( wADCInjFlags == CONV_STARTED )  
    {
      do
      {
        wADCInjFlags = ( ADC1-> SR ) & ADC_SR_MASK;
      }
      while ( wADCInjFlags != CONV_FINISHED );
    }
  }

  /* Switch Context */
  /* Disabling trigger to avoid unwanted conversion */
  ADC1->CR2 = pHandle->wADCTriggerUnSet;// 避免不必要的转换
  ADC2->CR2 = pHandle->wADCTriggerUnSet;

  /* Enabling next Trigger */
  TIMx->CCER |= 0x1000u; // CC4E = 1

  /* It re-initilize AD converter in run time when using dual MC */
  ADC1->CR2 = pHandle->wADCTriggerSet;
  ADC2->CR2 = pHandle->wADCTriggerSet;

  /* Change channels keeping equal to 1 element the sequencer length */
  ADC1->JSQR = pHandle->wADC1Channel; // 重新设置转换通道
  ADC2->JSQR = pHandle->wADC2Channel;

  return &( pHandle->_Super.bMotor );
}

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
