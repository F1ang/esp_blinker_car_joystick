/**
  ******************************************************************************
  * @file    mc_type.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control SDK global types definitions
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
  * @ingroup MC_Type
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Type Motor Control types
  * @{
  */

/**
  * @define MISRA_C_2004_BUILD
  * @brief Used to build the library with MISRA C support
  *
  * Uncomment #define MISRA_C_2004_BUILD to build the library including
  * "stm32fxxx_MisraCompliance.h" instead of "stm32fxxx.h".
  *
  * This will build the library in 'strict ISO/ANSI C' and in
  * compliance with MISRA C 2004 rules (check project options).
  *
  * @note Do not use this flag with the current version of the SDK.
  */
/*#define MISRA_C_2004_BUILD*/

#include <mc_stm_types.h>

/** @name Macros to use bit banding capability */
/** @{ */
#define BB_REG_BIT_SET(regAddr,bit_number) *(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000 )<<5) + (bit_number <<2)) = (uint32_t)(0x1u)
#define BB_REG_BIT_CLR(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) = (uint32_t)(0x0u))
#define BB_REG_BIT_READ(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) )
/** @} */

/** @brief Not initialized pointer */
#define MC_NULL    (uint16_t)(0x0000u)

/** @name Motor identification macros */
/** @{ */
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
/** @} */

/** @name Fault source error codes */
/** @{ */
#define  MC_NO_ERROR  (uint16_t)(0x0000u)      /**< @brief No error.*/
#define  MC_NO_FAULTS  (uint16_t)(0x0000u)     /**< @brief No error.*/
#define  MC_FOC_DURATION  (uint16_t)(0x0001u)  /**< @brief Error: FOC rate to high.*/
#define  MC_SW_ERROR  (uint16_t)(0x0080u)      /**< @brief Software Error.*/
/** @} */

/** @name Dual motor Frequency comparison definition */
/** @{ */
#define SAME_FREQ   0u
#define HIGHER_FREQ 1u
#define LOWER_FREQ  2u

#define HIGHEST_FREQ 1u
/** @} */

/**
  * @brief Two components stator current type definition
  */
typedef struct
{
  int16_t qI_Component1;
  int16_t qI_Component2;
} Curr_Components;

/**
  * @brief  Two components stator voltage type definition
  */
typedef struct
{
  int16_t qV_Component1;
  int16_t qV_Component2;
} Volt_Components;

/**
  * @brief  ADConv_t type definition, it is used by PWMC_ADC_SetSamplingTime method of PWMnCurrFdbk class for user defined A/D regular conversions
  */
typedef struct
{
  uint8_t Channel;   /*!< Integer channel number, from 0 to 15 */
  uint8_t SamplTime; /*!< Sampling time selection, ADC_SampleTime_nCycles5
                          in case of STM32F10x, n= 1, 3, ...; */
} ADConv_t;

/**
  * @brief  STC_Modality_t type definition, it's used by STC_SetControlMode and STC_GetControlMode methods in
  *         SpeednTorqCtrl class to specify the control modality type
  */
typedef enum
{
  STC_TORQUE_MODE, /**< @brief Torque mode.*/
  STC_SPEED_MODE   /**< @brief Speed mode.*/
} STC_Modality_t;


/**
  * @brief  FOC variables structure
  */
typedef struct
{
  Curr_Components Iqdref;      /**< @brief Stator current on rotor reference frame qd */
  int16_t hTeref;              /**< @brief Reference torque */
} FOCVars_t, *pFOCVars_t;


/**
  * @brief  MPInfo structure (used only for serial communication)
  */
typedef struct
{
  uint8_t * data;
  uint8_t len;
} MPInfo_t, *pMPInfo_t;

#define MAIN_SCFG_POS (28) // 主要的速度传感器,bit 28


/** @name ADC channel number definitions */
/** @{ */

#define MC_ADC_CHANNEL_6     6
#define MC_ADC_CHANNEL_8     8
#define MC_ADC_CHANNEL_9     9
/** @} */



#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
