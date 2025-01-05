/**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component of the Motor Control SDK.
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
#include "hall_speed_pos_fdbk.h"


/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
 *
  * This component is used in applications controlling a motor equipped with Hall effect sensors.
  *
  * This component uses the output of two Hall effects sensors to provide a measure of the speed
  * and the position of the rotor of the motor.
  *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD   ((uint16_t)0x5500u)// 速度下限捕获值,用于降低预分频

#define HALL_COUNTER_RESET  ((uint16_t) 0u)       

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)// <==> (360/3 = 120)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)// <==> (360/6 = 60)

#define STATE_0 (uint8_t)0// 0~7 霍尔信号的值
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE          (int8_t)-1    //转动方向
#define POSITIVE          (int8_t)1


/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED        ((int16_t)0x7FFF) // π

#define CCER_CC1E_Set               ((uint16_t)0x0001) // CCER的CC1E 置位
#define CCER_CC1E_Reset             ((uint16_t)0xFFFE) // CCER的CC1E 复位

static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle );
static int16_t HALL_CalcAvrgElSpeedDpp( HALL_Handle_t * pHandle );

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
/**
  * 函数功能: 霍尔传感器的初始化
  * 输入参数: @pHandle
  * 返 回 值: 无
  * 说    明: 
  */
void HALL_Init( HALL_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;
  /* 最大,最小 额定速度 */
  uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz * // 0*4
                                     pHandle->_Super.bElToMecRatio;// MecSpeed * 极对数
  uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz * // (3000*1.15/6) *4
                                     pHandle->_Super.bElToMecRatio;// MecSpeed * 极对数
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;

  /* Adjustment factor: minimum measurable speed is x time less than the minimum
  reliable speed */
  hMinReliableElSpeed01Hz /= 4u;// 最小的可测量速度是最小的额定速度的1/4  0/4 

  /* Adjustment factor: maximum measurable speed is x time greather than the
  maximum reliable speed */
  hMaxReliableElSpeed01Hz *= 2u;// 最大的可测量速度是最大的额定速度的2倍  2300*2

  pHandle->OvfFreq = ( uint16_t )( pHandle->TIMClockFreq / 65536u ); // 计数器最小溢出频率 84MHz/65536

  /* SW Init */
  if ( hMinReliableElSpeed01Hz == 0u )
  {
    /* Set fixed to 150 ms */
    pHandle->HallTimeout = 150u; // 最低速度的时候的间隔时间(两次霍尔信号之间的时间)
  }
  else
  {
    /* 10000来自于 ms和0.1hz , 6来自于 传感器每60度反转一次信号
     * 1s/最低转速的一圈翻转次数,如: 20(0.1Hz) ->  10000/(20*6) = 两次信号之间的间隔(ms)
     */
    pHandle->HallTimeout = 10000u / ( 6u * hMinReliableElSpeed01Hz ); //(两次霍尔信号之间的最大时间间隔)
  }

  /* Compute the prescaler to the closet value of the TimeOut (in mS )*/
  /* 两次信号的时间间隔*最小的溢出频率,-> 最大时间间隔/最大周期 = 最小的预分频值 除以1000转换标准单位 */
  pHandle->HALLMaxRatio = ( pHandle->HallTimeout * pHandle->OvfFreq ) / 1000 ;

  /* Align MaxPeriod to a multiple of Overflow.*/
  pHandle->MaxPeriod = ( pHandle->HALLMaxRatio ) * 65536uL; // 将MaxPeriod对齐为溢出的倍数

  pHandle->SatSpeed = hMaxReliableElSpeed01Hz;  // 最大可测量速度,超速的时候强制设置为最大速度

  /** ((84M/6) /18k) *65536 = 777.7*65536
    */
  pHandle->PseudoFreqConv = ( ( pHandle->TIMClockFreq / 6u ) // 
                              / ( pHandle->_Super.hMeasurementFrequency ) ) * 65536u; 

  /* 计数频率除以霍尔信号最大频率 */
  pHandle->MinPeriod = ( ( 10u * pHandle->TIMClockFreq ) / 6u ) // 最大转速对应的定时器周期
                       / hMaxReliableElSpeed01Hz;               // 6是电角度6分频,乘10是因为0.1Hz,(10*84Mhz/6)/4600

  pHandle->PWMNbrPSamplingFreq = ( pHandle->_Super.hMeasurementFrequency / // 18k / 500 =36
                                   pHandle->SpeedSamplingFreqHz ) - 1u;    // 速度采样频率

  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;

  /* Force the TIMx prescaler with immediate access (gen update event)
  */
  /* 生成更新事件使预分频立即生效*/
  LL_TIM_SetPrescaler ( TIMx, pHandle->HALLMaxRatio );
  LL_TIM_GenerateEvent_UPDATE ( TIMx );


  /* Clear the TIMx's pending flags */
  LL_TIM_WriteReg( TIMx, SR, 0 ); // 状态寄存器

  /* Selected input capture and Update (overflow) events generate interrupt */

  /* 设置更新源,只能是溢出更新 */
  LL_TIM_SetUpdateSource ( TIMx, LL_TIM_UPDATESOURCE_COUNTER );

  LL_TIM_EnableIT_CC1 ( TIMx );       /* CC1中断*/
  LL_TIM_EnableIT_UPDATE ( TIMx );    /* 更新中断中断*/
  LL_TIM_SetCounter ( TIMx, HALL_COUNTER_RESET );/* 复位计数器 */

  LL_TIM_CC_EnableChannel  ( TIMx, LL_TIM_CHANNEL_CH1 );/* 使能CC1捕获 */
  LL_TIM_EnableCounter ( TIMx );      /* 使能计数器 */


  /* Erase speed buffer */
  /* 复位速度缓存器 */
  bSpeedBufferSize = pHandle->SpeedBufferSize;

  for ( bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++ )
  {
    pHandle->SensorSpeed[bIndex]  = 0;
  }
}

/**
* @brief  Clear software FIFO where are "pushed" latest speed information
*         This function must be called before starting the motor to initialize
*         the speed measurement process.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component*
* @retval none
*/
/**
  * 函数功能: 复位跟hall传感器相关的变量
  * 输入参数: @pHandle
  * 返 回 值: 无
  * 说    明: 无
  */
void HALL_Clear( HALL_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;

  /* Mask interrupts to insure a clean intialization */
  LL_TIM_DisableIT_CC1 ( TIMx );

  pHandle->RatioDec = false;
  pHandle->RatioInc = false;

  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;

  /* Acceleration measurement not implemented.*/
  pHandle->_Super.hMecAccel01HzP = 0;

  pHandle->FirstCapt = 0u;
  pHandle->BufferFilled = 0u;
  pHandle->OVFCounter = 0u;

  pHandle->CompSpeed = 0;
  pHandle->ElSpeedSum = 0;

  pHandle->Direction = POSITIVE;

  /* Initialize speed buffer index */
  pHandle->SpeedFIFOIdx = 0u;

  /* Clear new speed acquisitions flag */
  pHandle->NewSpeedAcquisition = 0;

  /* Re-initialize partly the timer */
  LL_TIM_SetPrescaler ( TIMx, pHandle->HALLMaxRatio );

  LL_TIM_SetCounter ( TIMx, HALL_COUNTER_RESET );

  LL_TIM_EnableCounter ( TIMx );

  LL_TIM_EnableIT_CC1 ( TIMx );

  HALL_Init_Electrical_Angle( pHandle );
}


/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t Measured electrical angle in s16degree format.
*/
/**
  * 函数功能: 计算电角度
  * 输入参数: @pHandle
  * 返 回 值: 电角度(s16degree)
  * 说    明: dpp(每周期的数字量化值(s16degree), 将每采样周期的数值累加就得到所需值)
  */
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle )
{

  if ( pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED ) //最高转速
  {
    pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
    pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
  }
  else
  {
    pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
  }

  return pHandle->_Super.hElAngle;
}


/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HALL_CalcElAngle function and SPD_GetElAngle.
  *         Then compute rotor average el speed (express in dpp considering the
  *         measurement frequency) based on the buffer filled by IRQ, then - as
  *         a consequence - compute, store and return - through parameter
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then check, store and return the reliability state of
  *         the sensor; in this function the reliability is measured with
  *         reference to specific parameters of the derived
  *         sensor (HALL) through internal variables managed by IRQ.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
/**
  * 函数功能: 计算平均机械转速
  * 输入参数: @hMecSpeed01Hz 用于存放机械转速,单位是0.1Hz
  * 返 回 值: @bool 传感器可靠性
  * 说    明: 无
  */
bool HALL_CalcAvrgMecSpeed01Hz( HALL_Handle_t * pHandle, int16_t * hMecSpeed01Hz )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;
  int16_t SpeedMeasAux;
  bool bReliability;

  /* Computing the rotor istantaneous el speed */
  SpeedMeasAux = pHandle->CurrentSpeed; // 在 HALL_IRQ_HANDLER中计算的当前速度

  if ( pHandle->SensorIsReliable )
  {
    /* No errors have been detected during rotor speed information
    extrapolation */
    if ( LL_TIM_GetPrescaler ( TIMx ) >= pHandle->HALLMaxRatio )
    {
      /* At start-up or very low freq */
      /* Based on current prescaler value only */
      pHandle->_Super.hElSpeedDpp = 0;
      *hMecSpeed01Hz = 0;
    }
    else
    {
      pHandle->_Super.hElSpeedDpp = SpeedMeasAux; // 电角度速度(dpp)
      if ( SpeedMeasAux == 0 )
      {
        /* Speed is too low */
        *hMecSpeed01Hz = 0;  // 速度过低
      }
      else
      {
        /* Check if speed is not to fast */
        if ( SpeedMeasAux != HALL_MAX_PSEUDO_SPEED )// != π(180°)
        {
            pHandle->TargetElAngle = pHandle->MeasuredElAngle;
            pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle; // 电角度增量
          pHandle->CompSpeed = ( int16_t )  // 补偿系数,用于同步 当前的电角度 和 目标电角度 
            ( ( int32_t )( pHandle->DeltaAngle ) /
              ( int32_t )( pHandle->PWMNbrPSamplingFreq ) );


          *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp( pHandle );// 计算平均电角度速度(dpp)

          /* Converto el_dpp to Mec01Hz */
          *hMecSpeed01Hz = ( int16_t )( ( *hMecSpeed01Hz * // 将dpp转换成0.1Hz
                                          ( int32_t )pHandle->_Super.hMeasurementFrequency * 10 ) /
                                        ( 65536 * ( int32_t )pHandle->_Super.bElToMecRatio ) );

        }
        else
        {
          *hMecSpeed01Hz = ( int16_t )pHandle->SatSpeed;// 超速
        }
      }
    }
    bReliability = true;
  }
  else
  {
    bReliability = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    /* If speed is not reliable the El and Mec speed is set to 0 */
    pHandle->_Super.hElSpeedDpp = 0;
    *hMecSpeed01Hz = 0;
  }

  pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

  return ( bReliability );
}


/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx capture event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
/**
  * 函数功能: 定时器捕获中断
  * 输入参数: @pHandleVoid
  * 返 回 值: 无
  * 说    明: 定时器霍尔传感器接口,中断捕获霍尔信号
  */
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;
  uint8_t bPrevHallState;
  uint32_t wCaptBuf;
  uint16_t hPrscBuf;
  uint16_t hHighSpeedCapture;

  /*---------------------- 计算电角度 ---------------------- */
  if ( pHandle->SensorIsReliable )
  {
    /* A capture event generated this interrupt */
    bPrevHallState = pHandle->HallState; // 上一次的霍尔状态

    if ( pHandle->SensorPlacement == DEGREES_120 ) // 120°的霍尔传感器
    {
      /* 读取新的状态 */
      pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2    //Hight     Low
                            | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1  // H3   H2   H1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );      //
    }
    else
    {
      /* 将60°的一个通道异或翻转,然后调整两个通道的顺序,就可以得到跟120°相同的数值 */
      pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                            | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
    }

    /* 根据读到的霍尔信号累加电角度 */
    switch ( pHandle->HallState )
    {
      case STATE_5: // 当前状态值
        if ( bPrevHallState == STATE_4 ) //上一次的状态值是4,则是正转方向
        {
          pHandle->Direction = POSITIVE;
          /* 默认 5就是电角度的0度(H1的上升沿),同步电角度作为0度的偏移值 */
          pHandle->MeasuredElAngle = pHandle->PhaseShift;// 电角度 = 同步电角度 
        }
        else if ( bPrevHallState == STATE_1 )// 否则是反转方向
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT );//同步电角度 + 60°
        }
        break;

      case STATE_1:
        if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;// 同步电角度 + 60°
        }
        else if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        break;

      case STATE_3:
        if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );// 同步电角度 + 120°
        }
        else if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                                  S16_60_PHASE_SHIFT );
        }
        break;

      case STATE_2:
        if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT  // 同步电角度 + 120° + 60°
                                                  + S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        break;

      case STATE_6:
        if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );// 同步电角度 - 120°
        }
        else if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        break;

      case STATE_4:
        if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );// 同步电角度 - 60°
        }
        else if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift );
        }
        break;

      default:
        /* Bad hall sensor configutarion so update the speed reliability */
        pHandle->SensorIsReliable = false;

        break;
    }
    
    /*---------------------- 计算速度 ---------------------- */
    /* 丢弃第一次捕获的数据 */
    if ( pHandle->FirstCapt == 0u )
    {
      pHandle->FirstCapt++;
      LL_TIM_IC_GetCaptureCH1( TIMx );
    }
    else
    { // 用于验证测量的平均速度
      /* used to validate the average speed measurement */
      if ( pHandle->BufferFilled < pHandle->SpeedBufferSize ) 
      {
        pHandle->BufferFilled++;
      }

      /* Store the latest speed acquisition */
      /* 获取定时器捕获值和预分频 */
      hHighSpeedCapture = LL_TIM_IC_GetCaptureCH1( TIMx );// 定时器捕获值
      wCaptBuf = ( uint32_t )hHighSpeedCapture; // uint16_t -> Uint32_t
      hPrscBuf =  LL_TIM_GetPrescaler ( TIMx ); // 预分频值

      /* Add the numbers of overflow to the counter */
      /* 捕获值加上溢出的次数,得到两次信号之间的计数间隔 */
      wCaptBuf += ( uint32_t )pHandle->OVFCounter * 0x10000uL;// 溢出计数

      /* 根据是否溢出来重新计算wCaptBuf */
      if ( pHandle->OVFCounter != 0u ) // 溢出
      {
        /* Adjust the capture using prescaler */ 
        uint16_t hAux;
        hAux = hPrscBuf + 1u;
        wCaptBuf *= hAux; // 预分频乘上捕获值,得到的是定时器计数频率的计数值  就是以84Mhz计数得到的计数值

        if ( pHandle->RatioInc )// 上一次已经修改了预分频但这一次还是溢出了,下一次还需要修改预分频
        {
          pHandle->RatioInc = false;  /* Previous capture caused overflow */
          /* Don't change prescaler (delay due to preload/update mechanism) */
        }
        else// 上一次没有改预分频值但是这一次溢出了,说明这一次需要修改预分频值
        {
          if ( LL_TIM_GetPrescaler ( TIMx ) < pHandle->HALLMaxRatio ) /* Avoid OVF w/ very low freq */
          {
            LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) + 1 ); /* To avoid OVF during speed decrease */
            pHandle->RatioInc = true;   /* new prsc value updated at next capture only */
          }
        }
      }
      else // 没有溢出
      {
        /* If prsc preload reduced in last capture, store current register + 1 */
        // 没有溢出但是上一次捕获的时候修改了预分频,这一次计算的时候需要调整预分频值
        if ( pHandle->RatioDec ) /* and don't decrease it again */ // 预分频改变flag 
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux;
          hAux = hPrscBuf + 2u; // 预分频+2
          wCaptBuf *= hAux;     // 捕获值×预分频

          pHandle->RatioDec = false;
        }
        /* 没有溢出并且上一次没有修改预分频,说明可以调整预分频,提高定时器计数频率 */
        else  /* If prescaler was not modified on previous capture */
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux = hPrscBuf + 1u;// 预分频+1
          wCaptBuf *= hAux;             // 捕获值×预分频

          /* 捕获值小于阈值,说明计数频率比较慢,转速较快,可以修改预分频,提高计数频率,使捕获值在一个合适的范围内 */
          if ( hHighSpeedCapture < LOW_RES_THRESHOLD ) /* If capture range correct */ 
          {
            /* 避免预分频值等于0的情况,修改预分频,并且标记这一次修改了预分频 */
            if ( LL_TIM_GetPrescaler ( TIMx ) > 0u ) /* or prescaler cannot be further reduced */
            {
              LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) - 1 ); /* Increase accuracy by decreasing prsc */
              /* Avoid decrementing again in next capt.(register preload delay) */
              pHandle->RatioDec = true;
            }
          }
        }
      }


      if ( wCaptBuf < pHandle->MinPeriod )//小于最小的周期值,就是转速过快,超速了
      {
        pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
        pHandle->NewSpeedAcquisition = 0;
      }
      else
      {
        /* 滑动均值滤波 */
        // 减去累加器中的速度值,也就是队列中的第一个速度值,然后存入最后的速度值,再累加
        pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator */
        if ( wCaptBuf >= pHandle->MaxPeriod ) // 速度过慢
        {
          pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = 0; //设为0
        }
        else
        {
          /* 捕获值转换成 */
          pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = ( int16_t ) ( pHandle->PseudoFreqConv / wCaptBuf );
          pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] *= pHandle->Direction; // 方向决定正负
          pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];// 电角度速度累加
        }
        /* Update pointers to speed buffer */
        pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];// 更新最新的速度
        pHandle->SpeedFIFOIdx++; // 更新目录索引
        if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
        {
          pHandle->SpeedFIFOIdx = 0u;
        }
        /* Indicate new speed acquisitions */ // 标记获得新的速度值
        pHandle->NewSpeedAcquisition = 1;
      }
      /* Reset the number of overflow occurred */
      pHandle->OVFCounter = 0u;
    }
  }
  return MC_NULL;
}


/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
/**
  * 函数功能: 更新中断
  * 输入参数: @pHandleVoid
  * 返 回 值: 无
  * 说    明: TIM5的更新中断处理
  */
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;

  if ( pHandle->SensorIsReliable )
  {
    uint16_t hMaxTimerOverflow;  // 允许的最大溢出次数
    /* an update event occured for this interrupt request generation */
    pHandle->OVFCounter++; // 溢出计数

    /* 预分频值越大,允许的次数越少 */
    hMaxTimerOverflow = ( uint16_t )( ( ( uint32_t )pHandle->HallTimeout * pHandle->OvfFreq )
                                      / ( ( LL_TIM_GetPrescaler ( TIMx ) + 1 ) * 1000u ) );
    if ( pHandle->OVFCounter >= hMaxTimerOverflow )// 超过了允许的最大次数就复位变零
    {
      /* Set rotor speed to zero */
      pHandle->_Super.hElSpeedDpp = 0;

      /* Reset the electrical angle according the hall sensor configuration */
      HALL_Init_Electrical_Angle( pHandle );

      /* Reset the overflow counter */
      pHandle->OVFCounter = 0u;


      /* Reset the SensorSpeed buffer*/
      uint8_t bIndex;
      for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
      {
        pHandle->SensorSpeed[bIndex]  = 0;
      }
      pHandle->BufferFilled = 0 ;
      pHandle->CurrentSpeed = 0;
      pHandle->SpeedFIFOIdx = 1;
      pHandle->ElSpeedSum = 0;

    }
  }
  return MC_NULL;
}

/**
* @brief  Compute and returns the average rotor electrical speed express in dpp
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t the average rotor electrical speed express in dpp
*/
/**
  * 函数功能: 计算平均的电角度速度
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 电角度的速度平均值(dpp)
  */
static int16_t HALL_CalcAvrgElSpeedDpp( HALL_Handle_t * pHandle )
{

  if ( pHandle->NewSpeedAcquisition == 1 )
  {

    if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
    {
      pHandle->AvrElSpeedDpp = ( int16_t )  pHandle->CurrentSpeed; //直接使用当前的速度值
    }
    else
    {
      pHandle->AvrElSpeedDpp = ( int16_t )( pHandle->ElSpeedSum / ( int32_t )(  // 电角度速度累加值除以缓存数
                                              pHandle->SpeedBufferSize ) ); /* Average value */
    }

    /* Clear new speed acquisitions flag */
    pHandle->NewSpeedAcquisition = 0;
  }

  return pHandle->AvrElSpeedDpp;
}

/**
* @brief  Read the logic level of the three Hall sensor and individuates in this
*         way the position of the rotor (+/- 30�). Electrical angle is then
*         initialized.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
/**
  * 函数功能: 初始化电角度
  * 输入参数: @pHandle
  * 返 回 值: 无
  * 说    明: 初始位置,采用折中的方式,按照30°来计算
  */
static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle )
{

  if ( pHandle->SensorPlacement == DEGREES_120 )
  {
    pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }
  else
  {
    pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }

  switch ( pHandle->HallState )
  {
    case STATE_5:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_1:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_3:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_2:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_6:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_4:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT / 2 );
      break;
    default:
      /* Bad hall sensor configutarion so update the speed reliability */
      pHandle->SensorIsReliable = false;
      break;
  }

  /* Initialize the measured angle */
  pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;

}


/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
