/**
 ******************************************************************************
 * @file    pqd_motor_power_measurement.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          pqd_motor_power_measurement component of the Motor Control SDK.
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
 * @ingroup motorpowermeasurement
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PQD_MOTORPOWERMEASUREMENT_H
#define __PQD_MOTORPOWERMEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "motor_power_measurement.h"
#include "bus_voltage_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup motorpowermeasurement
 * @{
 */

/** @addtogroup pqd_motorpowermeasurement
 * @{
 */

typedef struct
{
    MotorPowMeas_Handle_t _super;

    int32_t wConvFact; /* It is the conversion factor used to convert the
                           variables expressed in digit into variables expressed
                           in physical measurement unit. It is used to convert the
                           power in watts. It must be equal to
                           (1000 * 3 * Vdd锟�)/(sqrt(3) * Rshunt * Aop) */

    pFOCVars_t pFOCVars;             /*!< Pointer to FOC vars used by MPM.*/
    BusVoltageSensor_Handle_t *pVBS; /*!< Bus voltage sensor object used by MPM.*/
} PQD_MotorPowMeas_Handle_t;

/**
 * @brief Implementation of derived class init method. It should be called before each motor restart.
 * @param pHandle related component instance.
 * @retval none.
 */
void PQD_Clear(PQD_MotorPowMeas_Handle_t *pHandle);

/**
 * @brief Implementation of derived class CalcElMotorPower.
 * @param pHandle related component instance.
 * @retval int16_t The measured motor power expressed in watt.
 */
void PQD_CalcElMotorPower(PQD_MotorPowMeas_Handle_t *pHandle);

/**
 * @}
 */

/**
 * @}
 */

/** @} */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
/* ICL activation: counting the step before pass in ICL_ACTIVE */
if (pHandle->hICLTicksCounter == 0u) {
    pHandle->ICLstate = ICL_ACTIVE;
} else {
    pHandle->hICLTicksCounter--;
}
}
break;

case ICL_DEACTIVATION: {
    /* ICL deactivation: counting the step before pass in ICL_INACTIVE.*/
    if (pHandle->hICLTicksCounter == 0u) {
        pHandle->ICLstate = ICL_INACTIVE;
    } else {
        pHandle->hICLTicksCounter--;
    }
} break;

case ICL_ACTIVE: {
    /* ICL is active: if bus is present deactivate the ICL */
    if (VBS_CheckVbus(pHandle->pVBS) != MC_UNDER_VOLT) {
        DOUT_SetOutputState(pHandle->pDOUT, INACTIVE);
        pHandle->ICLstate = ICL_DEACTIVATION;
        pHandle->hICLTicksCounter = pHandle->hICLTotalTicks;
    }
} break;

case ICL_INACTIVE: {
    /* ICL is inactive: if bus is not present activate the ICL */
    if (VBS_CheckVbus(pHandle->pVBS) == MC_UNDER_VOLT) {
        DOUT_SetOutputState(pHandle->pDOUT, ACTIVE);
        pHandle->ICLstate = ICL_ACTIVATION;
        pHandle->hICLTicksCounter = pHandle->hICLTotalTicks;
    }
} break;

case ICL_IDLE:
default: {
} break;
}

return pHandle->ICLstate;
}

/**
 * @brief It returns the current state of the ICL state machine.
 * @param  pHandle: handler of the current instance of the ICL component
 * @retval ICLState_t returns the current ICL state machine
 */
ICL_State_t ICL_GetState(ICL_Handle_t *pHandle)
{
    return pHandle->ICLstate;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
