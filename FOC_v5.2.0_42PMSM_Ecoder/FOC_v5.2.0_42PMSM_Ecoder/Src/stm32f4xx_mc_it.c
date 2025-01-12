/**
 ******************************************************************************
 * @file    stm32f4xx_mc_it.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Main Interrupt Service Routines.
 *          This file provides exceptions handler and peripherals interrupt
 *          service routine related to Motor Control
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
 * @ingroup STM32F4xx_IRQ_Handlers
 */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f4xx_ll_exti.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
 * @{
 */
/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY / 1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */
/**
 * @brief  This function handles ADC1/ADC2 interrupt request.
 * @param  None
 * @retval None
 */
void ADC_IRQHandler(void)
{
    /* USER CODE BEGIN ADC_IRQn 0 */

    /* USER CODE END ADC_IRQn 0 */
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        // Clear Flags
        ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

        UI_DACUpdate(TSK_HighFrequencyTask()); /*GUI, this section is present only if DAC is enabled*/
    }
#ifdef ADC3
    else {
        // Clear Flags
        ADC3->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

        UI_DACUpdate(TSK_HighFrequencyTask()); /*GUI, this section is present only if DAC is enabled*/
    }
#endif
    /* USER CODE BEGIN ADC_IRQn 1 */

    /* USER CODE END ADC_IRQn 1 */
}

/**
 * @brief  This function handles first motor TIMx Update interrupt request.
 * @param  None
 * @retval None
 */
void TIMx_UP_M1_IRQHandler(void)
{
    /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

    /* USER CODE END TIMx_UP_M1_IRQn 0 */
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    R3F4XX_TIMx_UP_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

    /* USER CODE END TIMx_UP_M1_IRQn 1 */
}

/**
 * @brief  This function handles first motor BRK interrupt.
 * @param  None
 * @retval None
 */
void TIMx_BRK_M1_IRQHandler(void)
{
    /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

    /* USER CODE END TIMx_BRK_M1_IRQn 0 */
    if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx)) {
        LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
        R3F4XX_BRK_IRQHandler(&PWM_Handle_M1);
    }
    /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
    MC_Scheduler();

    /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

    /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
 * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
 * @param  None
 * @retval None
 */
void SPD_TIM_M1_IRQHandler(void)
{
    /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

    /* USER CODE END SPD_TIM_M1_IRQn 0 */

    /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
    if (LL_TIM_IsEnabledIT_UPDATE(ENCODER_M1.TIMx) && LL_TIM_IsActiveFlag_UPDATE(ENCODER_M1.TIMx)) {
        LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
        ENC_IRQHandler(&ENCODER_M1);
        /* USER CODE BEGIN M1 ENCODER_Update */

        /* USER CODE END M1 ENCODER_Update   */
    } else {
        /* No other IT to manage for encoder config */
    }
    /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

    /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
 * @brief  This function handles USART interrupt request.
 * @param  None
 * @retval None
 */
void USART_IRQHandler(void)
{
    /* USER CODE BEGIN USART_IRQn 0 */

    /* USER CODE END USART_IRQn 0 */

    if (LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)) /* Valid data received */
    {
        uint16_t retVal;
        retVal = *(uint16_t *)UFCP_RX_IRQ_Handler(&pUSART, LL_USART_ReceiveData8(pUSART.USARTx)); /* Flag 0 = RX */
        if (retVal == 1) {
            UI_SerialCommunicationTimeOutStart();
        }
        if (retVal == 2) {
            UI_SerialCommunicationTimeOutStop();
        }
        /* USER CODE BEGIN USART_RXNE */

        /* USER CODE END USART_RXNE   */
    }

    if (LL_USART_IsActiveFlag_ORE(pUSART.USARTx)) // Overrun error occurs after SR access and before DR access
    {
        /* Send Overrun message */
        UI_SerialCommunicationTimeOutStop();
        UFCP_OVR_IRQ_Handler(&pUSART);
        LL_USART_ClearFlag_ORE(pUSART.USARTx);
        /* USER CODE BEGIN USART_ORE */

        /* USER CODE END USART_ORE   */
    }

    if (LL_USART_IsActiveFlag_TXE(pUSART.USARTx)) {
        UFCP_TX_IRQ_Handler(&pUSART); /* Flag 1 = TX */
                                      /* USER CODE BEGIN USART_TXE */

        /* USER CODE END USART_TXE   */
    }

    /* USER CODE BEGIN USART_IRQn 1 */

    /* USER CODE END USART_IRQn 1 */
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    TSK_HardwareFaultTask();

    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
        {
            if (LL_USART_IsActiveFlag_ORE(pUSART.USARTx)) /* Overrun error occurs */
            {
                /* Send Overrun message */
                UFCP_OVR_IRQ_Handler(&pUSART);
                LL_USART_ClearFlag_ORE(pUSART.USARTx); /* Clear overrun flag */
                UI_SerialCommunicationTimeOutStop();
            }

            if (LL_USART_IsActiveFlag_TXE(pUSART.USARTx)) {
                UFCP_TX_IRQ_Handler(&pUSART);
            }

            if (LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)) /* Valid data have been received */
            {
                uint16_t retVal;
                retVal = *(uint16_t *)(UFCP_RX_IRQ_Handler(&pUSART, LL_USART_ReceiveData8(pUSART.USARTx)));
                if (retVal == 1) {
                    UI_SerialCommunicationTimeOutStart();
                }
                if (retVal == 2) {
                    UI_SerialCommunicationTimeOutStop();
                }
            } else {
            }
        }
    }
    /* USER CODE BEGIN HardFault_IRQn 1 */

    /* USER CODE END HardFault_IRQn 1 */
}

void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
    static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    if (SystickDividerCounter == SYSTICK_DIVIDER) {
        HAL_IncTick();
        HAL_SYSTICK_IRQHandler();
        SystickDividerCounter = 0;
    }
    SystickDividerCounter++;
#endif /* MC_HAL_IS_USED */

    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();
    /* USER CODE BEGIN SysTick_IRQn 2 */
    /* USER CODE END SysTick_IRQn 2 */
}

/*GUI, this section is present only if start/stop button is enabled*/
/**
 * @brief  This function handles Button IRQ on PIN PE0.
 * @param  None
 * @retval None
 */

void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN START_STOP_BTN */
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    UI_HandleStartStopButton_cb();
    /* USER CODE END START_STOP_BTN */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
