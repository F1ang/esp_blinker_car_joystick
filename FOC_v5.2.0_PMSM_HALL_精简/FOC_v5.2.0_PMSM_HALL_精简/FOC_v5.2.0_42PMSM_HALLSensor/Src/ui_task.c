/**
  ******************************************************************************
  * @file    ui_task.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes user interface tasks definition
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
/* Pre-compiler coherency check */

#include "ui_task.h"

/* Setup the exported functions see UIExportedFunctions.h enum. */
extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

/**
  * 函数功能: 按键中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 启动电机或者停止电机
  */
void UI_HandleStartStopButton_cb (void)
{
  if (MC_GetSTMStateMotor1() == IDLE)
  {
    /* Ramp parameters should be tuned for the actual motor */
    MC_StartMotor1();
  }
  else
  {
    MC_StopMotor1();
  }
}



/**
  * 函数功能: 速度Ramp控制
  * 输入参数: @SpdRPM 电机转速,单位是(RPM), @Durationms 持续时间,单位是(ms)
  * 返 回 值: 无
  * 说    明: 控制电机速度执行Ramp指令  
  *  v↑         _______
  *   |        /
  *   |       /
  *   |      /
  *   |____ /
  *   |    ↑Ramp↑
  *   +-------------------→t
  */
void UI_SPDRampCtrl(int16_t SpdRPM ,uint16_t Durationms)
{
  int16_t SPD01Hz = SpdRPM / 6; // 将RPM转换成0.1Hz
  MCI_ExecSpeedRamp(pMCI[M1],SPD01Hz ,Durationms);     // 速度控制
}
/**
  * 函数功能: 扭矩Ramp控制
  * 输入参数: @Torque 电机扭矩, @Durationms 持续时间,单位是(ms)
  * 返 回 值: 无
  * 说    明: 控制电机扭矩进行Ramp指令
  *  i↑         _______
  *   |        /
  *   |       /
  *   |      /
  *   |____ /
  *   |    ↑Ramp↑
  *   +-------------------→t
  */
void UI_TorRampCtrl(int16_t Torque ,uint16_t Durationms)
{
  MCI_ExecTorqueRamp(pMCI[M1], Torque ,1000);  // 扭矩控制
}

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
