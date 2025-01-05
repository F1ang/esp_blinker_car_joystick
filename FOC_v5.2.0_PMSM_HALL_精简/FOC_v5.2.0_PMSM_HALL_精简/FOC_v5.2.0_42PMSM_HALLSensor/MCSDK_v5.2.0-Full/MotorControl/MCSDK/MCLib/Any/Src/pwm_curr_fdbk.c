/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
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
#include "pwm_curr_fdbk.h"


/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM.
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * comparators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
  *
  * The choice of the implementation mostly depend on these two factors and is performed by the
  * Motor Control Workbench tool.
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

/**
  * @brief Returns the phase current of the motor as read by the ADC (in s16A unit)
  *
  * The function actually returns the current values of phase A & B. Phase C current
  * can be deduced thanks to the formula:
  *
  * @f[
  * I_{C} = -I_{A} - I_{C}
  * @f]
  *
  * @param  pHandle handle on the target PWMC component
  * @param  pStator_Currents Pointer to the structure that will receive motor current
  *         of phase A and B in Curr_Components format.
*/
/**
  * 函数功能: 获取相电流
  * 输入参数: @pStator_Currents 获取相电流
  * 返 回 值: 无
  * 说    明: 调用一个指针函数,所指向不同的函数
  */
void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents )
{
  pHandle->pFctGetPhaseCurrents( pHandle, pStator_Currents );
  // R3F4XX_GetPhaseCurrents
}

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and feed them to the inverter.
  * @param  pHandle handler on the target PWMC component.
  * @param  Valfa_beta Voltage Components expressed in the @f$(\alpha, \beta) @f$ reference frame
  *
  * This function computes the the time during which the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. Then, the function
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION which entails a
  * Motor Control Fault that stops the motor.
  *
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
/**
  * 函数功能: 将输入的Vα 和Vβ转换成为3相PWM输出
  * 输入参数: @Valfa_beta  Vα 和 Vβ
  * 返 回 值: 错误状态,如果出错就是 error FOC duration
  * 说    明: 从Vα 和 Vβ计算出3相PWM占空比,就是SVPWM的实现函数
  */
uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle, Volt_Components Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
  PWMC_SetSampPointSectX_Cb_t pSetADCSamplingPoint;

  /* Uα = Vα * T * SQRT(3) * 2  */  /* hT_Sqrt3 = (T*16384*sqrt(3)*2) / 16384 */
  wUAlpha = Valfa_beta.qV_Component1 * ( int32_t )pHandle->hT_Sqrt3;
  /* Uβ = -(Vβ * T)  */
  wUBeta = -( Valfa_beta.qV_Component2 * ( int32_t )( pHandle->hPWMperiod ) ) * 2;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      pHandle->hSector = SECTOR_5;
      wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );// 2^1+2^17=2^18 = 262144
      wTimePhB = wTimePhA + wZ / 131072;// 2^2+ 2^15 = 2^17 = 131072
      wTimePhC = wTimePhA - wY / 131072;
      pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect5;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
        pHandle->hSector = SECTOR_4;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect4;
      }
      else /* wX > 0 */
      {
        pHandle->hSector = SECTOR_3;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect3;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      pHandle->hSector = SECTOR_2;
      wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect2;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        pHandle->hSector = SECTOR_6;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect6;
      }
      else /* wX > 0 */
      {
        pHandle->hSector = SECTOR_1;
        wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect1;
      }
  }

  pHandle->hCntPhA = ( uint16_t )wTimePhA;
  pHandle->hCntPhB = ( uint16_t )wTimePhB;
  pHandle->hCntPhC = ( uint16_t )wTimePhC;


  return ( pSetADCSamplingPoint( pHandle ) );// 根据扇区设置采样通道
}

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  * @param  pHandle Handle on the target instance of the PWMC component
  */
/**
  * 函数功能: 关断PWM输出
  * 输入参数: 
  * 返 回 值: 
  * 说    明: MOE =0 
  */
void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in.
  *
  * This function should be called before each motor start-up.
  *
  * @param  pHandle Handle on the target instance of the PWMC component
  * @param  action Can be #CRC_START to initialize the offset calibration or
  *         #CRC_EXEC to execute the offset calibration.
  * @retval true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
/**
  * 函数功能: 读取电流值校准
  * 输入参数: action动作
  * 返 回 值: false:没有完成读取电流校准的动作.
  * 说    明: 每次电机启动之前都要执行一次电流校准
  */
bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle, CRCAction_t action )
{
  bool retVal = false;
  if ( action == CRC_START )     // 启动校准动作
  {
    PWMC_SwitchOffPWM( pHandle );// 先掉PWM
    pHandle->hOffCalibrWaitTimeCounter = pHandle->hOffCalibrWaitTicks; // 设置等待时间
    if ( pHandle->hOffCalibrWaitTicks == 0u )
    {
      pHandle->pFctCurrReadingCalib( pHandle );
      retVal = true;
    }
  }
  else if ( action == CRC_EXEC )// 执行中
  {
    if ( pHandle->hOffCalibrWaitTimeCounter > 0u )
    {
      pHandle->hOffCalibrWaitTimeCounter--;
      if ( pHandle->hOffCalibrWaitTimeCounter == 0u ) //等待超时时间
      {
        pHandle->pFctCurrReadingCalib( pHandle );     // 读取并校准电流
        retVal = true;
      }
    }
    else
    {
      retVal = true;
    }
  }
  return retVal;
}


/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
