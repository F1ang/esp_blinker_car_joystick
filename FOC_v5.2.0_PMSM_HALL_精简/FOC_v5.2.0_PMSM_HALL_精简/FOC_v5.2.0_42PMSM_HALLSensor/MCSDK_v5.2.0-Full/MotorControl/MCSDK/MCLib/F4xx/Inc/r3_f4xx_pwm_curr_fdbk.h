/**
  ******************************************************************************
  * @file    r3_f4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_f4_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r3_f4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_F4XX_PWMNCURRFDBK_H
#define __R3_F4XX_PWMNCURRFDBK_H


/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"


/* Exported types ------------------------------------------------------- */
/**
  * @brief  PWMC_R3_F4_Handle_t parameters structure definition
  */
typedef struct
{
  /* Dual MC parameters --------------------------------------------------------*/
  uint16_t Tw;                    /*!< It is used for switching the context
                                       in dual MC. It contains biggest delay
                                       (expressed in counter ticks) between
                                       the counter crest and ADC latest trigger
                                       */
  uint8_t  bFreqRatio;             /*!< It is used in case of dual MC to
                                        synchronize TIM1 and TIM8. It has
                                        effect only on the second instanced
                                        object and must be equal to the
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are
                                        1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;       /*!< When bFreqRatio is greather than 1
                                        this param is used to indicate if this
                                        instance is the one with the highest
                                        frequency. Allowed value are: HIGHER_FREQ
                                        or LOWER_FREQ */
  /* Current reading A/D Conversions initialization -----------------------------*/
  uint8_t bIaChannel;                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIbChannel;                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIcChannel;                   /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/
  /* PWM generation parameters --------------------------------------------------*/
  TIM_TypeDef * TIMx;                   /*!< It contains the pointer to the timer
                                            used for PWM generation. It must
                                            equal to TIM1 if M1, to TIM8 otherwise */
  uint16_t hDeadTime;                  /*!< Dead time in number of TIM clock
                                            cycles. If CHxN are enabled, it must
                                            contain the dead time to be generated
                                            by the microcontroller, otherwise it
                                            expresses the maximum dead time
                                            generated by driving network */
  uint8_t  bRepetitionCounter;         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
  uint16_t hTafter;                    /*!< It is the sum of dead time plus max
                                            value between rise time and noise time
                                            express in number of TIM clocks.*/
  uint16_t hTbefore;                   /*!< It is the sampling time express in
                                            number of TIM clocks.*/


} R3_F4_Params_t;
/**
  * @brief  This structure is used to handle an instance of the
  *         r3_f4XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*!< base component handler   */
  uint32_t wPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;   /*!< Offset of Phase C current sensing network  */
  uint32_t wADC1Channel;    /*!< ADC1 programmed channel for motor current
                                 sampling */
  uint32_t wADC2Channel;    /*!< ADC2 programmed channel for motor current
                                 sampling */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts */
  volatile uint8_t bSoFOC;  /*!< This flag is reset at the beginning of FOC
                                 and it is set in the TIM UP IRQ. If at the end of
                                 FOC this flag is set, it means that FOC rate is too
                                 high and thus an error is generated */
  volatile uint8_t  bIndex;
  uint32_t wADCTriggerSet;  /*!< Store the value for ADC CR2 to proper configure
                                 current sampling during the context switching*/
  uint32_t wADCTriggerUnSet;/*!< Store the value for ADC CR2 to disable the
                                 current sampling during the context switching*/
  R3_F4_Params_t const * pParams_str;
} PWMC_R3_F4_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*  It initializes peripherals for current reading and PWM generation
 *  in three shunts configuration using STM32F4XX *****/
void R3F4XX_Init( PWMC_R3_F4_Handle_t * pHandle );

/**
  * It measures and stores into handler component variables the offset voltage on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor
  */
void R3F4XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/**
  * It computes and return latest converted motor phase currents motor
  */
void R3F4XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R3F4XX_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * It enables PWM generation on the proper Timer peripheral acting on MOE
  * bit
  */
void R3F4XX_SwitchOnPWM( PWMC_Handle_t * pHdl );

/* It disables PWM generation on the proper Timer peripheral acting on
 * MOE bit
 */
void R3F4XX_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 1.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect1( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 2.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect2( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 3.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect3( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 4.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect4( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 5.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect5( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 6.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F4XX_SetADCSampPointSect6( PWMC_Handle_t * pHdl );


/**
  * @brief  It contains the TIMx Update event interrupt
  */
void * R3F4XX_TIMx_UP_IRQHandler( PWMC_R3_F4_Handle_t * pHdl );


#endif /*__R3_F4XX_PWMNCURRFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
