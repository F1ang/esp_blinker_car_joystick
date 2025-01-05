/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Speed & Position Feedback component of the Motor Control SDK.
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
#include "speed_pos_fdbk.h"



/** @defgroup SpeednPosFdbk Speed & Position Feedback
 *
  * @brief Speed & Position Feedback components of the Motor Control SDK
  *
  * These components provide the speed and the angular position of the rotor of a motor (both
  * electrical and mechanical).
  *
  * Several implementations of the Speed and Position Feedback feature are provided by the Motor
  * to account for the specificities of the motor used on the application:
  *
  * - @ref hall_speed_pos_fdbk "Hall Speed & Position Feedback" for motors with Hall effect sensors
  * - @ref Encoder  "Encoder Speed & Position Feedback" for motors with a quadrature encoder
  * - two general purpose sensorless implementations are provided:
  *   @ref SpeednPosFdbk_STO "State Observer with PLL" and
  *   @ref STO_CORDIC_SpeednPosFdbk "State Observer with CORDIC"
  * - "High Frequency Injection" for anisotropic I-PMSM motors (Not included in this release).
  *
  * @{
  */

/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360�/65536
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical angle (s16degrees)
  */
/**
  * 函数功能: 获取电角度
  * 输入参数: @pHandle
  * 返 回 值: 电角度(s16degrees)
  * 说    明: 无
  */
int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hElAngle );
}

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  * @note   both Hall sensor and Sensor-less do not implement either
  *         mechanical angle computation or acceleration computation.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
/**
  * 函数功能: 获取机械转速平均值
  * 输入参数: @pHandle
  * 返 回 值: 机械转速(0.1Hz)的平均值 
  * 说    明: 无
  */
int16_t SPD_GetAvrgMecSpeed01Hz( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hAvrMecSpeed01Hz );
}


/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
