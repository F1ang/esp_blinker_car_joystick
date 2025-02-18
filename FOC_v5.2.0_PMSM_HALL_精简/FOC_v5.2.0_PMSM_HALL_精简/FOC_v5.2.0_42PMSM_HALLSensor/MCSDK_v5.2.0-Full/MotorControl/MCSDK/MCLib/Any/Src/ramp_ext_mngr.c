/**
  ******************************************************************************
  * @file    ramp_ext_mngr.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Ramp Extended Manager component of the Motor Control SDK:
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
#include "ramp_ext_mngr.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup RampExtMngr Ramp Manager
  * @brief Ramp Extended Manager component of the Motor Control SDK
  *
  * @todo Document the Ramp Extended Manager "module".
  *
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
uint32_t getScalingFactor( int32_t Target );

/**
  * @brief  It reset the state variable to zero.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval none.
  */
void REMNG_Init( RampExtMngr_Handle_t * pHandle )
{
  pHandle->Ext = 0;
  pHandle->TargetFinal = 0;
  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
  pHandle->ScalingFactor = 1u;

}

/**
  * @brief  Exec the ramp calculations and returns the current value of the
            state variable.
            It must be called at fixed interval defined in the hExecFreq.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
int32_t REMNG_Calc( RampExtMngr_Handle_t * pHandle )
{
  int32_t ret_val;
  int32_t current_ref;

  current_ref = pHandle->Ext;

  /* Update the variable and terminates the ramp if needed. */
  if ( pHandle->RampRemainingStep > 1u )
  {
    /* Increment/decrement the reference value. */
    current_ref += pHandle->IncDecAmount;

    /* Decrement the number of remaining steps */
    pHandle->RampRemainingStep --;
  }
  else if ( pHandle->RampRemainingStep == 1u )
  {
    /* Set the backup value of TargetFinal. */
    current_ref = pHandle->TargetFinal * ( int32_t )( pHandle->ScalingFactor );
    pHandle->RampRemainingStep = 0u;
  }
  else
  {
    /* Do nothing. */
  }

  pHandle->Ext = current_ref;


  ret_val = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );


  return ret_val;
}

/**
  * @brief  Setup the ramp to be executed
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @param  hTargetFinal (signed 32bit) final value of state variable at the end
  *         of the ramp.
  * @param  hDurationms (unsigned 32bit) the duration of the ramp expressed in
  *         milliseconds. It is possible to set 0 to perform an instantaneous
  *         change in the value.
  * @retval bool It returns true is command is valid, false otherwise
  */
bool REMNG_ExecRamp( RampExtMngr_Handle_t * pHandle, int32_t TargetFinal, uint32_t Durationms )
{
  uint32_t aux;
  int32_t aux1;
  int32_t current_ref;
  bool retVal = true;


  current_ref = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );

  if ( Durationms == 0u )
  {
    pHandle->ScalingFactor = getScalingFactor( TargetFinal );
    pHandle->Ext = TargetFinal * ( int32_t )( pHandle->ScalingFactor );
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
  }
  else
  {
    uint32_t wScalingFactor = getScalingFactor( TargetFinal - current_ref );
    uint32_t wScalingFactor2 = getScalingFactor( current_ref );
    uint32_t wScalingFactor3 = getScalingFactor( TargetFinal );
    uint32_t wScalingFactorMin;

    if ( wScalingFactor <  wScalingFactor2 )
    {
      if ( wScalingFactor < wScalingFactor3 )
      {
        wScalingFactorMin = wScalingFactor;
      }
      else
      {
        wScalingFactorMin = wScalingFactor3;
      }
    }
    else
    {
      if ( wScalingFactor2 < wScalingFactor3 )
      {
        wScalingFactorMin = wScalingFactor2;
      }
      else
      {
        wScalingFactorMin = wScalingFactor3;
      }
    }

    pHandle->ScalingFactor = wScalingFactorMin;
    pHandle->Ext = current_ref * ( int32_t )( pHandle->ScalingFactor );

    /* Store the TargetFinal to be applied in the last step */
    pHandle->TargetFinal = TargetFinal;

    /* Compute the (wRampRemainingStep) number of steps remaining to complete
    the ramp. */
    aux = Durationms * ( uint32_t )pHandle->FrequencyHz; /* Check for overflow and use prescaler */
    aux /= 1000u;
    pHandle->RampRemainingStep = aux;
    pHandle->RampRemainingStep++;

    /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
    the reference value at each CalcTorqueReference. */
    aux1 = ( TargetFinal - current_ref ) * ( int32_t )( pHandle->ScalingFactor );
    aux1 /= ( int32_t )( pHandle->RampRemainingStep );
    pHandle->IncDecAmount = aux1;
  }

  return retVal;
}

/**
  * @brief  Returns the current value of the state variable.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
int32_t REMNG_GetValue( RampExtMngr_Handle_t * pHandle )
{
  int32_t ret_val;
  ret_val = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );
  return ret_val;
}

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool REMNG_RampCompleted( RampExtMngr_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->RampRemainingStep == 0u )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval none
  */
void REMNG_StopRamp( RampExtMngr_Handle_t * pHandle )
{
  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
}

/**
  * @brief  Calculating the scaling factor to maximixe the resolution. It
  *         perform the 2^int(31-log2(Target)) with an iterative approach.
  *         It allows to keep Target * Scaling factor inside int32_t type.
  * @param  Target Input data.
  * @retval uint32_t It returns the optimized scaling factor.
  */
uint32_t getScalingFactor( int32_t Target )
{
  uint8_t i;
  uint32_t TargetAbs;
  int32_t aux;

  if ( Target < 0 )
  {
    aux = -Target;
    TargetAbs = ( uint32_t )( aux );
  }
  else
  {
    TargetAbs = ( uint32_t )( Target );
  }
  for ( i = 1u; i < 32u; i++ )
  {
    uint32_t limit = ( ( uint32_t )( 1 ) << ( 31u - i ) );
    if ( TargetAbs >= limit )
    {
      break;
    }
  }
  return ( ( uint32_t )( 1u ) << ( i - 1u ) );
}


/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
