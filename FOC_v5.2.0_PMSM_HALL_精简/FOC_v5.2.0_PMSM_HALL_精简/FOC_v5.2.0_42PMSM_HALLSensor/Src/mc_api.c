/**
  ******************************************************************************
  * @file    mc_api.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the high level interface of the Motor Control SDK.
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
  
#include "mc_api.h"

/** @defgroup MCIAPI Motor Control API
  *
  * @brief High level Programming Interface of the Motor Control SDK
  *
  *  This interface allows for performing basic operations on the motor(s) driven by an
  * Motor Control SDK based application. With it, motors can be started and stopped, speed or
  * torque ramps can be programmed and executed and information on the state of the motors can
  * be retrieved, among others.
  *
  *  This interface consists in functions that target a specific motor, indicated in their name. 
  * These functions aims at being the main interface used by an Application to control motors.
  *
  *  The current Motor Control API can cope with up to 2 motors.  
  * @{
  */

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

/**
  * @brief  Initiates the start-up procedure for Motor 1
  * 
  * If the state machine of Motor 1 is in #IDLE state, the command is immediately
  * executed. Otherwise the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *
  * One of the following commands must be executed before calling MC_StartMotor1():
  *
  * - MC_ProgramSpeedRampMotor1()
  * - MC_ProgramTorqueRampMotor1()
  * - MC_SetCurrentReferenceMotor1()
  *
  * Failing to do so results in an unpredictable behaviour.
  *
  * @note The MC_StartMotor1() command only triggers the start-up procedure: 
  * It moves Motor 1's state machine from the #IDLE to the #IDLE_START state and then 
  * returns. It is not blocking the application until the motor is indeed running. 
  * To know if it is running, the application can query Motor 1's state machine and 
  * check if it has reached the #RUN state. See MC_GetSTMStateMotor1() for more details.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */

/**
  * 函数功能: 启动电机
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 调用之前先调用 - MC_ProgramSpeedRampMotor1()
  *                          - MC_ProgramTorqueRampMotor1()
  *                          - MC_SetCurrentReferenceMotor1()
  * 中的任意一个函数设定目标值,这只是触发启动命令,不是立即执行
  */
bool MC_StartMotor1(void)
{
	return MCI_StartMotor( pMCI[M1] );
}

/**
  * @brief  Initiates the stop procedure for Motor 1.
  *
  *  If the state machine is in #RUN or #START states the command is immediately
  * executed. Otherwise, the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *  
  * @note The MCI_StopMotor1() command only triggers the stop motor procedure 
  * moving Motor 1's state machine to #ANY_STOP and then returns. It is not 
  * blocking the application until the motor is indeed stopped. To know if it has
  * stopped, the application can query Motor 1's state machine ans check if the 
  * #IDLE state has been reached back.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */

/**
  * 函数功能: 停止电机转动
  * 输入参数:
  * 返 回 值:
  * 说    明: 这只是触发停止命令,不是立即执行
  */
bool MC_StopMotor1(void)
{
	return MCI_StopMotor( pMCI[M1] );
}


/**
 * @brief 返回当前状态机的状态值
 */
State_t  MC_GetSTMStateMotor1(void)
{
	return MCI_GetSTMState( pMCI[M1] );
}

