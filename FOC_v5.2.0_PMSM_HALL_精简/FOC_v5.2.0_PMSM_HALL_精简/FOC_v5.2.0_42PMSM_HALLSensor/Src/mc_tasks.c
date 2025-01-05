
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition
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
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "mc_config.h"

#include "mc_interface.h"
#include "mc_tuning.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS         10
#define OFFCALIBRWAIT_MS            0
#define STOPPERMANENCY_MS         400
#define CHARGE_BOOT_CAP_TICKS    (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define OFFCALIBRWAITTICKS       (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define STOPPERMANENCY_TICKS     (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;
/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
static void TSK_MediumFrequencyTaskM1(void);
static void FOC_Clear(uint8_t bMotor);
static void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrController(uint8_t bMotor);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
static void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM1(void);


/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
/**
  * 函数功能: 启动电机控制
  * 输入参数: @pMCIList 电机控制接口 @pMCTList 电机控制实体
  * 返 回 值: 无
  * 说    明: 初始化各个指针
  */
void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1; /* 圆限制 */

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3F4XX_Init(&PWM_Handle_M1);   // 初始化F4相关的TIM,ADC
  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();    // 使用TIM2来触启动其他定时器,TIM1,TIM8都可以被TIM2触发启动

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);// 状态机初始化(IDLE,无错误)
  
  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);// PID初始化,速度调制
  
  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;// 电机1的速度环参数
  pSTC[M1] = &SpeednTorqCtrlM1;      // 速度扭矩控制
  HALL_Init (&HALL_M1);              // 初始化霍尔传感器
  
  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M1], pPIDSpeed[M1], &HALL_M1._Super);// 速度扭矩控制
  
  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);// Iq
  PID_HandleInit(&PIDIdHandle_M1);// Id
  pPIDIq[M1] = &PIDIqHandle_M1;  
  pPIDId[M1] = &PIDIdHandle_M1;
  
  FOC_Clear(M1);
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);// 默认的Iq
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
  MCI_ExecSpeedRamp(oMCInterface[M1],    // 默认为速度模式 ,设置默认的转速
  STC_GetMecSpeedRef01HzDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M1; 
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  pMCTList[M1] = &MCT[M1];
 

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 
 */
/**
  * 函数功能: 电机控制任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 执行MC_Scheduler,该函数在Systick中被调用,执行频率是2kHz
  */
void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance. 
 *
 * It is to be clocked at the Systick frequency.
 */
/**
  * 函数功能: 电机控制调度
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 执行中等频率任务,调用该函数频率是2KHz
  */
void MC_Scheduler(void)
{

  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--; // 中频计数器
    }
    else
    {
      TSK_MediumFrequencyTaskM1();// 执行中等频率的任务
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;// 计数器复位
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;  // 自举电容充电时间
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;// 状态机STOP的持续时间
    }
  }
}

/**
  * @brief  It executes some of the control duties on Motor 1 accordingly with
  *         the present state of its state machine. In particular, duties
  *         requiring a specific timing (e.g. speed controller) are here
  *         executed
  * @param  None
  * @retval void
  */
/**
  * 函数功能: 执行中等频率的任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 测量电机转速(hall),状态机->启动或者停止电机
  */
void TSK_MediumFrequencyTaskM1(void)
{
  State_t StateM1;
  int16_t wAux = 0;

  (void) HALL_CalcAvrgMecSpeed01Hz(&HALL_M1,&wAux); // 计算平均机械转速
  StateM1 = STM_GetState(&STM[M1]);
  switch(StateM1)
  {
    /* 从IDLE转到START */
    case IDLE_START:
      R3F4XX_TurnOnLowSides(pwmcHandle[M1]);    // 先导通下桥臂MOS管
      TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);// 设置自举电容充电时间
      STM_NextState(&STM[M1],CHARGE_BOOT_CAP); // -> CHARGE_BOOT_CAP
      break;
    case CHARGE_BOOT_CAP:
      if (TSK_ChargeBootCapDelayHasElapsedM1())// 等待充电时间
      {
        PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_START);// 开始读取电机电流值校准
        STM_NextState(&STM[M1],OFFSET_CALIB);  // -> OFFSET_CALIB
      }
      break;
    case OFFSET_CALIB:
      if (PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_EXEC))// 读取电流值之前可能需要等待一段时间,
      {
        STM_NextState(&STM[M1],CLEAR);// -> CLEAR
      }
      break;
    case CLEAR:
      HALL_Clear(&HALL_M1);// 清空速度值
      if(STM_NextState(&STM[M1], START) == true) // -> START
      {
        FOC_Clear(M1);     // 复位FOC变量
        R3F4XX_SwitchOnPWM(pwmcHandle[M1]); //启动PWM
      }
      break;  
    case START:
      {
        STM_NextState(&STM[M1], START_RUN); /* only for sensored*/// -> RUN
      }
      break;
    case START_RUN: // 无感模式
      {
        FOC_CalcCurrRef(M1);
        STM_NextState(&STM[M1], RUN);
      }
      STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
      MCI_ExecBufferedCommands(oMCInterface[M1]); /* Exec the speed ramp after changing of the speed sensor */
    
      break;
    case RUN:
      MCI_ExecBufferedCommands(oMCInterface[M1]);// 执行指令缓存里面的指令
      FOC_CalcCurrRef(M1); // FOC 更新目标值(电流值)
   
      break;
    case ANY_STOP:
      R3F4XX_SwitchOffPWM(pwmcHandle[M1]); // 关闭PWM
      FOC_Clear(M1);         // 复位FOC变量
      TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);// 设置停止持续时间
      STM_NextState(&STM[M1], STOP);// -> STOP
      break;
    case STOP:
      if(TSK_StopPermanencyTimeHasElapsedM1())// 等待持续时间
      {
        STM_NextState(&STM[M1], STOP_IDLE);// -> STOP_IDLE
      }
      break;
    case STOP_IDLE:
      STM_NextState(&STM[M1], IDLE);// -> IDLE
      break;
    default:// IDLE
      break;
  }
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
/**
  * 函数功能: 复位FOC的变量
  * 输入参数: @bMotor 电机编号
  * 返 回 值: 无
  * 说    明: 复位变量,关闭PWM
  */
void FOC_Clear(uint8_t bMotor)
{
  
  Curr_Components Inull = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iqdref = Inull;
  FOCVars[bMotor].hTeref = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
/**
  * 函数功能: 更新目标值
  * 输入参数: @bMotor 电机编号
  * 返 回 值: 无
  * 说    明: 计算新的目标值,使用Ramp的过程需要更新目标值,如果是速度模式则会再执行一次PID算法 
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
/**
  * 函数功能: 自举电容充电时间设置
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
/**
  * 函数功能: 查询自举电容充电是否已完成
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
/**
  * 函数功能: 设置STOP持续时间
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
/**
  * 函数功能: 查询STOP持续时间
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}


/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
/**
  * 函数功能: 高频任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 执行频率是ADC的采样频率,也就是PWM频率
  */
uint8_t TSK_HighFrequencyTask(void)
{
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
 
  HALL_CalcElAngle (&HALL_M1);          // 计算电角度

  hFOCreturn = FOC_CurrController(M1); // FOC 电流控制
  if(hFOCreturn == MC_FOC_DURATION)    // 在执行FOC算法期间出现定时器更新事件
  {
    STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);// 错误报警
  }
  return bMotorNbr;
}


/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
/**
  * 函数功能: FOC电流控制器
  * 输入参数: @bMotor 电机编号
  * 返 回 值: @MC_NO_FAULTS or @MC_FOC_DURATION 
  * 说    明: 获取电角度,相电流
  */
inline uint16_t FOC_CurrController(uint8_t bMotor)
{
  Curr_Components Iab, Ialphabeta, Iqd;
  Volt_Components Valphabeta, Vqd;
  int16_t hElAngle;
  uint16_t hCodeError;

  hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[bMotor])); // 电角度
  PWMC_GetPhaseCurrents(pwmcHandle[bMotor], &Iab); // 获取相电流 根据扇区获取A,B,C相电流 Ia, Ib, Ic
  Ialphabeta = MCM_Clarke(Iab);                    // Clarke变换 Ia,Ib,Ic -> Iα,Iβ
  Iqd = MCM_Park(Ialphabeta, hElAngle);            // Park变换   Iα,Iβ - >  Iq,Id
  Vqd.qV_Component1 = PI_Controller(pPIDIq[bMotor],// PI控制器   Iq -> Vq
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);

  Vqd.qV_Component2 = PI_Controller(pPIDId[bMotor],// PI控制器   Id -> Vd
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);
  Vqd = Circle_Limitation(pCLM[bMotor], Vqd);      // 圆限制 97%
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);        // Rev_Park变换 Vq, Vd -> Vα,Vβ
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[bMotor], Valphabeta);// SVPWM实现函数
  return(hCodeError);
}


/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
/**
  * 函数功能: HardFault执行任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 由于硬件问题进入HardFault IRQ的时候调用,关断PWM
  */
void TSK_HardwareFaultTask(void)
{
  
  R3F4XX_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
}


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
