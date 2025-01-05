/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PWM & Current Feedback component of the Motor Control SDK.
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
  * @ingroup pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWMNCURRFDBK_H
#define __PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"



/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u
#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct PWMC_Handle PWMC_Handle_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMC_Handle::pFctSwitchOffPwm
  * - PWMC_Handle::pFctSwitchOnPwm
  * - PWMC_Handle::pFctCurrReadingCalib
  * - PWMC_Handle::pFctTurnOnLowSides
  *
  *
  */
typedef void ( *PWMC_Generic_Cb_t )( PWMC_Handle_t * pHandle );

/**
  * @brief Pointer on the interrupt handling function of the PWMC component instance.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIrqHandler).
  *
  */
typedef void * ( *PWMC_IrqHandler_Cb_t )( PWMC_Handle_t * pHandle, unsigned char flag );

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase current.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetPhaseCurrents).
  *
  */
typedef void ( *PWMC_GetPhaseCurr_Cb_t )( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents );
/**
  * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
  *        point for each sectors.
  *
  * This type is needed because the actual function to use can change at run-time. See:
  * - PWMC_Handle::pFctSetADCSampPointSect1
  * - PWMC_Handle::pFctSetADCSampPointSect2
  * - PWMC_Handle::pFctSetADCSampPointSect3
  * - PWMC_Handle::pFctSetADCSampPointSect4
  * - PWMC_Handle::pFctSetADCSampPointSect5
  * - PWMC_Handle::pFctSetADCSampPointSect6
  *
  */
typedef uint16_t ( *PWMC_SetSampPointSectX_Cb_t )( PWMC_Handle_t * pHandle );

/**
  * @brief Pointer on the function provided by the PMWC component instance to check if an over current
  *        condition has occured.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIsOverCurrentOccurred).
  *
  */
typedef uint16_t ( *PWMC_OverCurr_Cb_t )( PWMC_Handle_t * pHandle );


/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_IrqHandler_Cb_t pFctIrqHandler;                   /**< pointer on the interrupt handling function. */
  PWMC_GetPhaseCurr_Cb_t
  pFctGetPhaseCurrents;           /**< pointer on the function the component instance uses to retrieve pahse currents */
  PWMC_Generic_Cb_t
  pFctSwitchOffPwm;                    /**< pointer on the function the component instance uses to switch PWM off */
  PWMC_Generic_Cb_t
  pFctSwitchOnPwm;                     /**< pointer on the function the component instance uses to switch PWM on */
  PWMC_Generic_Cb_t
  pFctCurrReadingCalib;                /**< pointer on the function the component instance uses to calibrate the current reading ADC(s) */
  PWMC_Generic_Cb_t
  pFctTurnOnLowSides;                  /**< pointer on the function the component instance uses to turn low sides on */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect1;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 1 */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect2;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 2 */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect3;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 3 */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect4;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 4 */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect5;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 5 */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSect6;  /**< pointer on the function the component instance uses to set the ADC sampling point when in sector 6 */

  /** @} */
  uint16_t  hT_Sqrt3;                                    /**< a constant utilized by PWM algorithm (@f$\sqrt{3}@f$) */
  uint16_t  hSector;                                     /**< the space vector sector number */
  uint16_t  hCntPhA;                                     /**< PWM Duty cycle for phase A */
  uint16_t  hCntPhB;                                     /**< PWM Duty cycle for phase B */
  uint16_t  hCntPhC;                                     /**< PWM Duty cycle for phase C */
  bool bTurnOnLowSidesAction;                            /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
  uint16_t  hOffCalibrWaitTimeCounter;                   /**< Counter to wait fixed time before motor
                                                              current measurement offset calibration. */
  uint8_t   bMotor;                                      /**< Motor reference number */

  /* former  PWMnCurrFdbkParams_t */
  uint16_t hPWMperiod;                                   /**< PWM period expressed in timer clock cycles unit:
                                                           *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
  uint16_t hOffCalibrWaitTicks;                          /**< Wait time duration before current reading
                                                           *  calibration expressed in number of calls
                                                           *  of PWMC_CurrentReadingCalibr() with action
                                                           *  #CRC_EXEC */
};

/**
  * @brief  Current reading calibration definition
  */
typedef enum CRCAction
{
  CRC_START, /**< Initialize the current reading calibration.*/
  CRC_EXEC   /**< Execute the current reading calibration.*/
} CRCAction_t;


/* Used to get the motor phase current in Curr_Components format as read by AD converter */
void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle,
                            Curr_Components * pStator_Currents );

/*  Converts input voltage components Valfa, beta into duty cycles and feed it to the inverter */
uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle,
                               Volt_Components Valfa_beta );

/* Switches the PWM generation off, setting the outputs to inactive */
void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* Switches the PWM generation on */
void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle );

/* Calibrates ADC current conversions by reading the offset voltage
 * present on ADC pins when no motor current is flowing.
 * This function should be called before each motor start-up */
bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle,
                                CRCAction_t action );


#endif /* __PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
