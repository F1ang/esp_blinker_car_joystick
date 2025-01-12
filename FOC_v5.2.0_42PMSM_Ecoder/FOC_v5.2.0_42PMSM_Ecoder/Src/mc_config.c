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
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

  #define MAX_TWAIT 0                 /* Dummy value for single drive */
  #define FREQ_RATIO 1                /* Dummy value for single drive */
  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0     

#include "pqd_motor_power_measurement.h"

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR/*!< It is the conversion factor used to convert the
                                         variables expressed in digit into variables expressed
                                         in physical measurement unit. It is used to convert
                                         the power in watts. It must be equal to
                                         (1000 * 3 * Vdd)/(sqrt(3) * CurrentAmpGain) */
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1; 

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,       /*!< Default Kp gain, used to initialize Kp gain
                                                                   software variable*/
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,       /*!< Default Ki gain, used to initialize Ki gain
                                                                   software variable*/
  
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,  /*!< Upper limit used to saturate the integral
                                                                   term given by Ki / Ki divisor * integral of
                                                                   process variable error */
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV, /*!< Lower limit used to saturate the integral
                                                                   term given by Ki / Ki divisor * integral of
                                                                   process variable error */
  .hUpperOutputLimit       = (int16_t)IQMAX,                      /*!< Upper limit used to saturate the PI output
                                                               */
  .hLowerOutputLimit       = -(int16_t)IQMAX,                     /*!< Lower limit used to saturate the PI output
                                                               */
  .hKpDivisor          = (uint16_t)SP_KPDIV,                  /*!< Kp gain divisor, used in conjuction with
                                                                   Kp gain allows obtaining fractional values.
                                                                   If FULL_MISRA_C_COMPLIANCY is not defined
                                                                   the divisor is implemented through
                                                                   algebrical right shifts to speed up PI
                                                                   execution. Only in this case this parameter
                                                                   specifies the number of right shifts to be
                                                                   executed */
  .hKiDivisor          = (uint16_t)SP_KIDIV,                  /*!< Ki gain divisor, used in conjuction with
                                                                   Ki gain allows obtaining fractional values.
                                                                   If FULL_MISRA_C_COMPLIANCY is not defined
                                                                   the divisor is implemented through
                                                                   algebrical right shifts to speed up PI
                                                                   execution. Only in this case this parameter
                                                                   specifies the number of right shifts to be
                                                                   executed */
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,              /*!< Kp gain divisor expressed as power of 2.
                                                                   E.g. if gain divisor is 512 the value
                                                                   must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,               /*!< Ki gain divisor expressed as power of 2.
                                                                   E.g. if gain divisor is 512 the value
                                                                   must be 9 because 2^9 = 512 */
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,   /*!< Default Kp gain, used to initialize Kp gain
                                                                  software variable*/
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,   /*!< Default Ki gain, used to initialize Ki gain
                                                                software variable*/

  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,      /*!< Upper limit used to saturate the integral
                                                                term given by Ki / Ki divisor * integral of
                                                                process variable error */
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,      /*!< Lower limit used to saturate the integral
                                                                term given by Ki / Ki divisor * integral of
                                                                process variable error */
  .hUpperOutputLimit       = INT16_MAX,                          /*!< Upper limit used to saturate the PI output
                                                           */
  .hLowerOutputLimit       = -INT16_MAX,                         /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisor          = (uint16_t)TF_KPDIV,               /*!< Kp gain divisor, used in conjuction with
                                                                Kp gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV,               /*!< Ki gain divisor, used in conjuction with
                                                                Ki gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,           /*!< Kp gain divisor expressed as power of 2.
                                                                E.g. if gain divisor is 512 the value
                                                                must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,            /*!< Ki gain divisor expressed as power of 2.
                                                                E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,    /*!< Default Kp gain, used to initialize Kp gain
                                                               software variable*/
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,    /*!< Default Ki gain, used to initialize Ki gain
                                                               software variable*/

  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,     /*!< Upper limit used to saturate the integral
                                                               term given by Ki / Ki divisor * integral of
                                                               process variable error */
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,     /*!< Lower limit used to saturate the integral
                                                               term given by Ki / Ki divisor * integral of
                                                               process variable error */
  .hUpperOutputLimit       = INT16_MAX,                         /*!< Upper limit used to saturate the PI output
                                                           */
  .hLowerOutputLimit       = -INT16_MAX,                        /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisor          = (uint16_t)TF_KPDIV,              /*!< Kp gain divisor, used in conjuction with
                                                               Kp gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV,              /*!< Ki gain divisor, used in conjuction with
                                                               Ki gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,          /*!< Kp gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,           /*!< Ki gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE, 	         /*!< Frequency on which the user updates
                                                                               the torque reference calling
                                                                               STC_CalcTorqueReference method
                                                                               expressed in Hz */
  .MaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED/6),  /*!< Application maximum positive value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .MinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED/6),      /*!< Application minimum positive value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .MaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED/6),    /*!< Application maximum negative value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .MinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED/6),/*!< Application minimum negative value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		     /*!< Maximum positive value of motor
                                                                               torque. This value represents
                                                                               actually the maximum Iq current
                                                                               expressed in digit.*/
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,                 /*!< Maximum negative value of motor
                                                                               torque. This value represents
                                                                               actually the maximum Iq current
                                                                               expressed in digit.*/
  .ModeDefault =					DEFAULT_CONTROL_MODE,                     /*!< Default STC modality.*/
  .MecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM/6),    /*!< Default mechanical rotor speed
                                                                               reference expressed in tenths of
                                                                               HZ.*/
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,        /*!< Default motor torque reference.
                                                                               This value represents actually the
                                                                               Iq current reference expressed in
                                                                               digit.*/
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,      /*!< Default motor torque reference.
                                                                               This value represents actually the
                                                                               Iq current reference expressed in
                                                                               digit.*/
};

RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,         /*!< Frequency expressed in Hz at which the user
                                                                         clocks the RUC calling RUC_Exec method */
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360), /*!< Starting angle of programmed rev up.*/

  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,    /*< Indicate here in which of the rev-up phases
                                                               the final acceleration is started. The
                                                               sensor-less algorithm is re-initialized
                                                               (cleared) when this phase begins.
                                                                NOTE: The rev up phases counting starts from 0.
                                                                      First phase should be indicated with zero */
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED,                  /*!< Minimum mechanical speed (expressed in
                                                                         01Hz required to validate the start-up */
  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED/2),     /*!< Minimum mechanical speed (expressed in
                                                                         01Hz required to validate the on the fly*/
  .OTFStartupEnabled       = false,  /*!< On The Fly enable control */ 

  .OTFPhaseParams         = {(uint16_t)500,                 /*!<  On The Fly section1 duration, millisecond */
                                         0,                 /*!<  Dummy value not used by OTF feature       */
                             (int16_t)PHASE5_FINAL_CURRENT, /*!<  final revup current                       */
                             (void*)MC_NULL},
  
  .ParamsData             = {{(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_RPM/6),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_RPM/6),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_RPM/6),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_RPM/6),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_RPM/6),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
                            },
};

PWMC_R3_F4_Handle_t PWM_Handle_M1=
{
  {
    .pFctGetPhaseCurrents              = &R3F4XX_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3F4XX_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3F4XX_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3F4XX_CurrentReadingCalibration,
    .pFctTurnOnLowSides                = &R3F4XX_TurnOnLowSides,           
    .pFctSetADCSampPointSect1          = &R3F4XX_SetADCSampPointSect1,     
    .pFctSetADCSampPointSect2          = &R3F4XX_SetADCSampPointSect2,     
    .pFctSetADCSampPointSect3          = &R3F4XX_SetADCSampPointSect3,     
    .pFctSetADCSampPointSect4          = &R3F4XX_SetADCSampPointSect4,     
    .pFctSetADCSampPointSect5          = &R3F4XX_SetADCSampPointSect5,     
    .pFctSetADCSampPointSect6          = &R3F4XX_SetADCSampPointSect6,              
    .pFctIsOverCurrentOccurred         = &R3F4XX_IsOverCurrentOccurred,    
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3F4XX_RLDetectionModeEnable,    
    .pFctRLDetectionModeDisable        = &R3F4XX_RLDetectionModeDisable,   
    .pFctRLDetectionModeSetDuty        = &R3F4XX_RLDetectionModeSetDuty,   
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,   /*!< Contains a constant utilized by pwm algorithm */
    .hSector = 0,    /*!< Contains the space vector sector number */
    .hCntPhA = 0,
    .hCntPhB = 0,
    .hCntPhC = 0,
    .SWerror = 0,
    .bTurnOnLowSidesAction = false, /*!< true if TurnOnLowSides action is active,
                                     false otherwise. */
    .hOffCalibrWaitTimeCounter = 0, /*!< Counter to wait fixed time before
                                              motor current measurement offset
                                              calibration. */
    .bMotor = 0,     /*!< Motor reference number */
    .RLDetectionMode = false, /*!< true if enabled, false if disabled. */
    .hIa = 0, /* Last Ia measurement. */
    .hIb = 0, /* Last Ib measurement. */
    .hIc = 0, /* Last Ic measurement. */
    .DTTest = 0,    /* Reserved */
    .DTCompCnt = 0, /* Reserved */
  
  
    .hPWMperiod          = PWM_PERIOD_CYCLES,     /*!< It contains the PWM period expressed
                                                       in timer clock cycles unit:
                                                       hPWMPeriod = Timer Fclk / Fpwm    */
    .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),              /*!< Wait time duration before current reading
                                                                calibration expressed in number of calls
                                                                of PWMC_CurrentReadingCalibr with action
                                                                CRC_EXEC */
    .hDTCompCnt          = DTCOMPCNT,                        /*!< It contains half of Dead time expressed
                                                                in timer clock cycles unit:
                                                                hDTCompCnt = (DT(s) * Timer Fclk)/2 */
    .Ton                 = TON,                              /*!< Reserved */
    .Toff                = TOFF                              /*!< Reserved */
  },
  .wPhaseAOffset = 0,   /*!< Offset of Phase A current sensing network  */
  .wPhaseBOffset = 0,   /*!< Offset of Phase B current sensing network  */
  .wPhaseCOffset = 0,   /*!< Offset of Phase C current sensing network  */
  .wADC1Channel = 0,    /*!< ADC1 programmed channel for motor current
                         sampling */
  .wADC2Channel = 0,    /*!< ADC2 programmed channel for motor current
                         sampling */
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,  /* Half PWM Period in timer clock counts */
  .bSoFOC = 0,  /*!< This flag is reset at the beginning of FOC
                         and it is set in the TIM UP IRQ. If at the end of
                         FOC this flag is set, it means that FOC rate is too
                         high and thus an error is generated */
  .bIndex = 0,
  .wADCTriggerSet = 0,   /*!< Store the value for ADC CR2 to proper configure
                         current sampling during the context switching*/
  .wADCTriggerUnSet = 0, /*!< Store the value for ADC CR2 to disable the
                         current sampling during the context switching*/

  .pParams_str = &R3_F4_ParamsM1,

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{
  
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,                /*!< Coefficient used to transform electrical to
                                                                           mechanical quantities and viceversa. It usually
                                                                           coincides with motor pole pairs number*/
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6), /*< Maximum value of measured speed that is
                                                                                      considered to be valid. It's expressed
                                                                                      in tenth of mechanical Hertz.*/
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),/*< Minimum value of measured speed that is
                                                                           considered to be valid. It's expressed
                                                                           in tenth of mechanical Hertz.*/
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            /*< Maximum value of not valid measurements
                                                                             before an error is reported.*/
    .hMaxReliableMecAccel01HzP         =	65535,                                 /*< Maximum value of measured acceleration
                                                                              that is considered to be valid. It's
                                                                              expressed in 01HzP (tenth of Hertz per
                                                                              speed calculation period)*/
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                    /*< Frequency on which the user will request
                                                                                   a measurement of the rotor electrical angle.
                                                                                   It's also used to convert measured speed from
                                                                                   tenth of Hz to dpp and viceversa.*/
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency (Hz) at which motor speed is to
																														be computed. It must be equal to the frequency
																													    at which function SPD_CalcAvrgMecSpeed01Hz
																														is called.*/
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),
                             /*!< Number of steps to perform
                                  the transition phase from CVSS_SPD to other CSPD;
                                  if the Transition PHase should last TPH milliseconds,
                                  and the FOC Execution Frequency is set to FEF kHz,
                                  then hTransitionSteps = TPH * FEF. If set to zero,
                                  it disables soft transition. It's recommended to
                                  set TPH as multiple of medium frequency task period
                              */
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
ENCODER_Handle_t ENCODER_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,                /*!< Coefficient used to transform electrical to
                                                                           mechanical quantities and viceversa. It usually
                                                                           coincides with motor pole pairs number*/
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6), /*!< Maximum value of measured speed that is
                                                                                 considered to be valid. It's expressed
                                                                                 in tenth of mechanical Hertz.*/
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),/*!< Minimum value of measured speed that is
                                                                           considered to be valid. It's expressed
                                                                           in tenth of mechanical Hertz.*/
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            /*!< Maximum value of not valid measurements
                                                                             before an error is reported.*/
    .hMaxReliableMecAccel01HzP         =	65535,                                 /*!< Maximum value of measured acceleration
                                                                              that is considered to be valid. It's
                                                                              expressed in 01HzP (tenth of Hertz per
                                                                              speed calculation period)*/
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                    /*!< Frequency on which the user will request
                                                                                   a measurement of the rotor electrical angle.
                                                                                   It's also used to convert measured speed from
                                                                                   tenth of Hz to dpp and viceversa.*/
  },  
  /* SW Settings */
  .PulseNumber           =	M1_ENCODER_PPR*4,		  /*!< Number of pulses per revolution, provided by each
                                                       of the two encoder signals, multiplied by 4 */
  .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED, /*!< To be enabled if measured speed is opposite
                                                                    to real one (ENABLE/DISABLE)*/

  .SpeedSamplingFreq01Hz =	10*MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency (01Hz) at which motor speed is to be
                                                                computed. It must be equal to the frequency
                                                                at which function SPD_CalcAvrgMecSpeed01Hz
                                                                is called.*/

  .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH,  /*!< Size of the buffer used to calculate the average
                                                            speed. It must be <= 16.*/
  
  /* HW Settings */
  .TIMx                =	ENC_TIM2,	    /*!< Timer used for  MC.ENCODER sensor management.*/

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
EncAlign_Handle_t EncAlignCtrlM1 =
{
  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency
                                                       expressed in Hz at which the user
                                                       clocks the EAC calling EAC_Exec method */
  .hFinalTorque    =	FINAL_I_ALIGNMENT,              /*!< Motor torque reference imposed by STC at the
                                                       end of programmed alignment. This value
                                                       represents actually the Iq current expressed in
                                                       digit.*/
  .hElAngle        =	ALIGNMENT_ANGLE_S16,      /*!< Mechanical angle of programmed alignment
                                                       expressed in s16degrees.*/
  .hDurationms     =	ALIGNMENT_DURATION,              
  
  .bElToMecRatio   =	POLE_PAIR_NUM,            
};

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC1, 
    .channel = MC_ADC_CHANNEL_13,
    .samplingTime = M1_TEMP_SAMPLING_TIME,   
  },  
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(MCU_SUPPLY_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ MCU_SUPPLY_VOLTAGE),
  .hT0                     = T0_C,											 
};

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,                 /*!< It contains the information about the type
                                                             of instanced bus voltage sensor object.
                                                             It can be equal to REAL_SENSOR or
                                                             VIRTUAL_SENSOR */
    .ConversionFactor    = (uint16_t)(MCU_SUPPLY_VOLTAGE / BUS_ADC_CONV_RATIO),
                                                        /*!< It is used to convert bus voltage from
                                                             u16Volts into real Volts (V).
                                                             1 u16Volt = 65536/hConversionFactor Volts
                                                             For real sensors hConversionFactor it's
                                                             equal to the product between the expected MCU
                                                             voltage and the partioning network
                                                             attenuation. For virtual sensors it is
                                                             assumed to be equal to 500 */
  },
  
  .VbusRegConv =
  {
    .regADC = ADC1, 
    .channel = MC_ADC_CHANNEL_10,
    .samplingTime = M1_VBUS_SAMPLING_TIME,   
  },

  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,   /*!< Use this number to configure the Vbus
                                                             first order software filter bandwidth.
                                                             hLowPassFilterBW = VBS_CalcBusReading
                                                             call rate [Hz]/ FilterBandwidth[Hz] */

  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,     /*!< It represents the over voltage protection
                                                             intervention threshold. To be expressed
                                                             in digit through formula:
                                                             hOverVoltageThreshold (digit) =
                                                             Over Voltage Threshold (V) * 65536
                                                                                / hConversionFactor */

  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,   /*!< It represents the under voltage protection
                                                             intervention threshold. To be expressed
                                                             in digit through formula:
                                                             hUnderVoltageThreshold (digit) =
                                                             Under Voltage Threshold (V) * 65536
                                                                                / hConversionFactor */
};

UI_Handle_t UI_Params =
{
	      .bDriveNum = 0,
	      .pFct_DACInit = &DAC_Init,               
	      .pFct_DACExec = &DAC_Exec,
	      .pFctDACSetChannelConfig    = &DAC_SetChannelConfig,
	      .pFctDACGetChannelConfig    = &DAC_GetChannelConfig,
	      .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
	      .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,
 
};

DAC_UI_Handle_t DAC_UI_Params = 
{
  .hDAC_CH1_ENABLED = ENABLE,  
  .hDAC_CH2_ENABLED = ENABLE
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE /*!< Execution frequency expressed in Hz */
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,        	/*!< Circle limitation maximum allowed
												 module */
  .Circle_limit_table = MMITABLE,        	/*!< Circle limitation table */
  .Start_index        = START_INDEX, 		/*!< Circle limitation table indexing
												 start */
};

UFCP_Handle_t pUSART =
{
    ._Super.RxTimeout = 0, 

    .USARTx              = USART,                
       
};

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

