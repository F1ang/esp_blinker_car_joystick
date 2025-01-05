/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED           3000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED           0 /*!< rpm, mechanical,  
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       200 /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */


/****** Hall sensors ************/ 
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  200 /*!< Number of failed   
                                                           derived class specific speed 
                                                           measurements before main sensor  
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        6 /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */                                                                                                           

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                    18000
 
#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   200 /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
                                                                                         
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         2172       
#define PID_TORQUE_KI_DEFAULT         2024
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           2172
#define PID_FLUX_KI_DEFAULT           2024
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      1024
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ       500 /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
#define PID_SPEED_KP_DEFAULT          100
#define PID_SPEED_KI_DEFAULT          50
#define PID_SPEED_KD_DEFAULT          0
/* Speed PID parameter dividers */
#define SP_KPDIV                      16
#define SP_KIDIV                      256
#define SP_KDDIV                      16


/* Default settings */
/* 默为速度模式,在mc_task.c::MCboot()里面设置了第一个指令就是执行
 * 速度Ramp,所以即使在这里改为扭矩模式也是没用的.需要在代码中,初始化之后调用
 *  MCI_ExecTorqueRamp() 执行扭矩Ramp指令才能修改为扭矩模式
 */
#define DEFAULT_CONTROL_MODE          STC_SPEED_MODE   /*!< STC_TORQUE_MODE or 
                                                       //   STC_SPEED_MODE */  
#define DEFAULT_TARGET_SPEED_RPM       -1000            // 速度模式 默认转速
#define DEFAULT_TORQUE_COMPONENT       0               // 扭矩模式 默认参数
#define DEFAULT_FLUX_COMPONENT         0               // 扭矩模式 默认参数

                                                               
/******************************   ADDITIONAL FEATURES   **********************/
/*  Maximum Torque Per Ampere strategy parameters */
#define IQMAX                          4997    // 输出阈值

#endif /*__DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
