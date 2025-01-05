/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"


#define OFFCALIBRWAIT_MS     0 // 电机1电流测量校准之前的等待时间,使用宏来修改
#define OFFCALIBRWAIT_MS2    0 // 电机2电流测量校准之前的等待时间, 

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
  
/* 电机1的PI速度环参数 */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,       /* 默认Kp ,用于初始化变量 */
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,       /* 默认Ki ,用于初始化变量 */
  
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,  /*!< 积分饱和上限 */
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV, /*!< 积分饱和下限  */
  .hUpperOutputLimit   = (int16_t)IQMAX,                      /*!< PI输出饱和上限*/
  .hLowerOutputLimit   = -(int16_t)IQMAX,                     /*!< PI输出饱和下限*/
  .hKpDivisor          = (uint16_t)SP_KPDIV,                  /*!< Kp的分母,用除法代替浮点数 */
  .hKiDivisor          = (uint16_t)SP_KIDIV,                  /*!< Ki的分母,用除法代替浮点数 */
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,              /*!< Kp的分母(2的指数)E.g. 如果 SP_KPDIV = 512 => SP_KPDIV_LOG = 9 ,2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,              /*!< Ki的分母(2的指数)*/

};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
/* 电机1的Iq PID环参数 */

PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,   /* 默认Kp ,用于初始化变量 */
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,   /* 默认Ki ,用于初始化变量 */

  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,    /*!< 积分饱和上限 */
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,   /*!< 积分饱和下限  */
  .hUpperOutputLimit       = INT16_MAX,                    /*!< PI输出饱和上限*/
  .hLowerOutputLimit       = -INT16_MAX,                   /*!< PI输出饱和下限*/
  .hKpDivisor          = (uint16_t)TF_KPDIV,               /*!< Kp的分母,用除法代替浮点数 */
  .hKiDivisor          = (uint16_t)TF_KIDIV,               /*!< Ki的分母,用除法代替浮点数 */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,           /*!< Kp的分母(2的指数)E.g. 如果 SP_KPDIV = 512 => SP_KPDIV_LOG = 9 ,2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,           /*!< Ki的分母(2的指数)*/

};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
/* 电机1的Id PID环参数 */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,    /* 默认Kp ,用于初始化变量 */
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,    /* 默认Ki ,用于初始化变量 */

  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,   /*!< 积分饱和上限 */
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,  /*!< 积分饱和下限  */
  .hUpperOutputLimit       = INT16_MAX,                   /*!< PI输出饱和上限*/
  .hLowerOutputLimit       = -INT16_MAX,                  /*!< PI输出饱和下限*/
  .hKpDivisor          = (uint16_t)TF_KPDIV,              /*!< Kp的分母,用除法代替浮点数 */
  .hKiDivisor          = (uint16_t)TF_KIDIV,              /*!< Ki的分母,用除法代替浮点数 */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,          /*!< Kp的分母(2的指数)E.g. 如果 SP_KPDIV = 512 => SP_KPDIV_LOG = 9 ,2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,          /*!< Ki的分母(2的指数)*/

};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
/* 电机1的速度扭矩控制器 参数 */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE, 	    /*!< 用户调用STC_CalcTorqueReference更新扭矩目标值的频率,单位是hz(500Hz) */
  .MaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED/6),/*!< 最大的应用转速,单位是0.1Hz(0.1RPS). 3000RPM/6  */
  .MinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED/6),/*!< 最小的应用转速,单位是0.1Hz(0.1RPS).*/
  .MaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED/6),/*!< 最小的应用转速,单位是0.1Hz(0.1RPS).*/
  .MinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED/6),/*!< 最大的应用转速,单位是0.1Hz(0.1RPS). -3000RPM/6  */
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		          /*!< 最大的扭矩,Iq的数字量化值 */
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,            /*!< 最小的扭矩,Iq的数字量化值 */
  .ModeDefault =					DEFAULT_CONTROL_MODE,                     /*!< 默认的控制模式,速度模式.*/
  .MecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM/6),/*!< 默认的目标转速,单位是0.1Hz.*/
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,      /*!< 默认的扭矩目标值,Iq的数字量化值 */
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,          /*!< 默认的Id目标值.*/
};

PWMC_R3_F4_Handle_t PWM_Handle_M1=
{
  { /* 父类指针函数实例化,指向实例函数(用于规范接口标准) */
    .pFctGetPhaseCurrents              = &R3F4XX_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3F4XX_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3F4XX_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3F4XX_CurrentReadingCalibration,/* 电流读取校准 */
    .pFctTurnOnLowSides                = &R3F4XX_TurnOnLowSides,          
    .pFctSetADCSampPointSect1          = &R3F4XX_SetADCSampPointSect1,   /* 扇区1的ADC采样点 */  
    .pFctSetADCSampPointSect2          = &R3F4XX_SetADCSampPointSect2,   /* 扇区2的ADC采样点 */    
    .pFctSetADCSampPointSect3          = &R3F4XX_SetADCSampPointSect3,   /* 扇区3的ADC采样点 */    
    .pFctSetADCSampPointSect4          = &R3F4XX_SetADCSampPointSect4,   /* 扇区4的ADC采样点 */    
    .pFctSetADCSampPointSect5          = &R3F4XX_SetADCSampPointSect5,   /* 扇区5的ADC采样点 */    
    .pFctSetADCSampPointSect6          = &R3F4XX_SetADCSampPointSect6,   /* 扇区6的ADC采样点 */    
    /*  */
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,   /*!< (T*16384*sqrt(3)*2)/16384 系数,用于生成SVPWM */
    .hSector = 0,     /*!< 空间向量扇区 编号 */
    .hCntPhA = 0,     /*!< A相绕组PWM通道的比较值 */
    .hCntPhB = 0,     /*!< B相绕组PWM通道的比较值 */
    .hCntPhC = 0,     /*!< C相绕组PWM通道的比较值 */
    .bTurnOnLowSidesAction = false, /*!< true if TurnOnLowSides action is active,false otherwise. */
    .hOffCalibrWaitTimeCounter = 0, /*!< 在电机电流测量校准之前等待固定的时间的计数器 */
    .bMotor = 0,     /*!< 电机编号 */

    .hPWMperiod          = PWM_PERIOD_CYCLES,     /*!<  PWM 周期值: hPWMPeriod = Timer Fclk / Fpwm 168M/18k= 9333  */
    .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),/*!< 在电机电流测量校准之前等待固定的时间 */

  },
  .wPhaseAOffset = 0,   /*!< A相绕组的AD采样偏移值 */
  .wPhaseBOffset = 0,   /*!< B相绕组的AD采样偏移值 */
  .wPhaseCOffset = 0,   /*!< C相绕组的AD采样偏移值 */
  .wADC1Channel = 0,    /*!< ADC1的采样通道,用于运行时实时修改采样通道  */
  .wADC2Channel = 0,    /*!< ADC1的采样通道,用于运行时实时修改采样通道 */
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,  /* 半PWM周期值 9333/2 = 4666*/
  .bSoFOC = 0,  /*!< 正常情况下是0,在TIM_UP中断中置位,如果在执行FOC算法的之后,被置1了,就说明FOC算法占用率过高 */
  .bIndex = 0,  /*!< 索引目录 */
  .wADCTriggerSet = 0,   /*!< 存储ADC->CR2的值,用于修改触发配置 */
  .wADCTriggerUnSet = 0, /*!< 存储ADC->CR2的值,用于修改触发配置 */
  .pParams_str = &R3_F4_ParamsM1,/*!< 电机控制参数*/

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
/* 霍尔接口参数 */
HALL_Handle_t HALL_M1 =
{
  ._Super = {/* 继承SpeednPosFdbk_Handle_t,用于访问父类 */
    .bElToMecRatio                     =	POLE_PAIR_NUM, /*!< 用于转换机械转角和电角度,极对数 */
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6), /*!< 最大有效转速,单位是0.1Hz */
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),/*!< 最小有效转速,单位是0.1Hz*/
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,          /*!< 最大的允许错误次数.*/
    .hMaxReliableMecAccel01HzP         =	65535,                              /*!< 最大的加速度,单位是0.1HzP(每速度测量周期的速度变化值) */
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 /*!< 测量电机电角度的频率,同样用于将速度单位从0.1Hz转换成dpp */
  }, 
  /* SW Settings */
  .SensorPlacement     = HALL_SENSORS_PLACEMENT, /*!< 霍尔传感器摆放位置 DEGREES_120 or DEGREES_60.*/
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360), /*!< 同步电角度(s16degree) */

  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE, /*!< 计算电机速度的频率Frequency (Hz) 500 */

  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,  /*!< 速度缓存 必须小于18.*/

  /* HW Settings */
 .TIMClockFreq       = HALL_TIM_CLK,           /*!< 定时器时钟频率 (Hz) */

 .TIMx                = HALL_TIM5,             /*!< TIM5 */

 .H1Port             =  M1_HALL_H1_GPIO_Port,  /*!< HALL sensor H1 channel GPIO input port */
 .H1Pin              =  M1_HALL_H1_Pin,        /*!< HALL sensor H1 channel GPIO output pin */
 .H2Port             =  M1_HALL_H2_GPIO_Port,  /*!< HALL sensor H2 channel GPIO input port */
 .H2Pin              =  M1_HALL_H2_Pin,        /*!< HALL sensor H2 channel GPIO output pin */
 .H3Port             =  M1_HALL_H3_GPIO_Port,  /*!< HALL sensor H3 channel GPIO input port */
 .H3Pin              =  M1_HALL_H3_Pin,        /*!< HALL sensor H3 channel GPIO output pin */												 
};


/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
/* 圆限制 */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,      /*!< 最大的调制值,root(Vd^2+Vq^2) <= MAX_MODULE = 32767*97% */
  .Circle_limit_table = MMITABLE,        /*!< 数组表格 */
  .Start_index        = START_INDEX, 		 /*!< 索引起始值*/
};


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

