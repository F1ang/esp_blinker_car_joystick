
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#include "pmsm_motor_parameters.h"
#include "parameters_conversion_f4xx.h"
#include "mc_math.h"

#define MCU_SUPPLY_VOLTAGE          3.30f

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint16_t) ((uint16_t)(PWM_FREQUENCY)/REGULATION_EXECUTION_RATE)
#define REP_COUNTER 			  (uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define SYS_TICK_FREQUENCY          2000

#define MF_TASK_OCCURENCE_TICKS     (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES           (uint16_t)(ADV_TIM_CLK_MHz*\
                                    (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY)))

#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz   (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1          (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif


#define TMIN (((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS))+\
			 ((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1)
#define HTMIN (uint16_t)(TMIN >> 1)


#define TBEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))\
                                           *ADV_TIM_CLK_MHz)/1000ul))+1)
                                           

#define MAX_TNTR_NS TRISE_NS

#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))

/*************** PI divisor  ***************/
#define SP_KPDIV_LOG   LOG2(16)
#define SP_KIDIV_LOG   LOG2(256)
#define SP_KDDIV_LOG   LOG2(16)
#define TF_KPDIV_LOG   LOG2(1024)
#define TF_KIDIV_LOG   LOG2(16384)
#define TF_KDDIV_LOG   LOG2(8192)
 
 

/****** Prepares the UI configurations according the MCconfxx settings ********/

/* Sensors setting */

#define MAIN_SCFG UI_SCODE_HALL

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE | (MAIN_SCFG << MAIN_SCFG_POS)  )

 /*** Temporary bridge between workbench data model****/
#define PWM_TIM8  TIM8
#define HALL_TIM5 TIM5


/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD 65535
#define M1_HALL_IC_FILTER  15
#define SPD_TIM_M1_IRQHandler TIM5_IRQHandler

#define START_INDEX     60
#define MAX_MODULE      31783   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*97%
#define MMITABLE {\
32483,32206,31936,31672,31415,31289,31041,30799,30563,30331,\
30105,29884,29668,29456,29352,29147,28947,28750,28557,28369,\
28183,28002,27824,27736,27563,27393,27226,27062,26901,26743,\
26588,26435,26360,26211,26065,25921,25780,25641,25504,25369,\
25236,25171,25041,24913,24788,24664,24542,24422,24303,24186,\
24129,24015,23902,23791,23681,23573,23467,23362,23258,23206,\
23105,23004,22905,22808,22711,22616,22521,22429\
}

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
