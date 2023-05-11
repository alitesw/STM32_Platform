<#ftl>
<#if !MC??>
<#if SWIPdatas??>
<#list SWIPdatas as SWIP>
<#if SWIP.ipName == "MotorControl">
<#if SWIP.parameters??>
<#assign MC = SWIP.parameters>
<#break>
</#if>
</#if>
</#list>
</#if>
<#if MC??>
<#else>
<#stop "No MotorControl SW IP data found">
</#if>
</#if>
/**
  ******************************************************************************
  * @file    stm32h7xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32H7 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32H7xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
<#if MC.START_STOP_BTN == true>
#include "stm32h7xx_ll_exti.h"
</#if>
<#if MC.MCP_USED>
#include "mcp_config.h"  
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32H7xx_IRQ_Handlers STM32H7xx IRQ Handlers
  * @{
  */
/* USER CODE BEGIN PRIVATE */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1'>
    <#assign TIMx     = "TIM1">
    <#assign Stream   = "4">
    <#assign DMAIrq   = "DMA2_Stream4_IRQHandler">
</#if>
<#if MC.PWM_TIMER_SELECTION == 'PWM_TIM8'>
    <#assign TIMx     = "TIM8">
    <#assign Stream   = "7">
    <#assign DMAIrq   = "DMA2_Stream7_IRQHandler">
</#if>
<#if MC.SINGLE_SHUNT == true>
void ${DMAIrq}(void);
</#if>
<#if MC.ENCODER == true || MC.AUX_ENCODER == true || MC.HALL_SENSORS == true || MC.AUX_HALL_SENSORS == true>
void SPD_TIM_M1_IRQHandler(void);
</#if>
void HardFault_Handler(void);
void SysTick_Handler(void);
<#if MC.START_STOP_BTN == true>
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)} (void);
</#if>

<#if MC.ADC_PERIPH == "ADC1" ||  MC.ADC_PERIPH == "ADC2" >
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
<#if MC.ADC_PERIPH == "ADC1" >
  if ( LL_ADC_IsActiveFlag_JEOS( ADC1 ) )
  {
    LL_ADC_ClearFlag_JEOS( ADC1 );
  }
<#elseif MC.ADC_PERIPH == "ADC2">
  if ( LL_ADC_IsActiveFlag_JEOS( ADC2 ) )
  {
    LL_ADC_ClearFlag_JEOS( ADC2 );
  }
</#if>

<#if MC.DAC_FUNCTIONALITY == true>
  // Highfrequency task Single or M1
  UI_DACUpdate(TSK_HighFrequencyTask());
<#else>
  // Highfrequency task Single or M1
  TSK_HighFrequencyTask();
</#if>
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

<#elseif MC.ADC_PERIPH == "ADC3">  
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC3_IRQHandler(void)

  if ( LL_ADC_IsActiveFlag_JEOS( ADC3 ) )
  {
    LL_ADC_ClearFlag_JEOS( ADC3 );
  }
<#if MC.DAC_FUNCTIONALITY == true>
  // Highfrequency task Single or M1
  UI_DACUpdate(TSK_HighFrequencyTask());
<#else>
  // Highfrequency task Single or M1
  TSK_HighFrequencyTask();
</#if>
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}
</#if>

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
<#if MC.THREE_SHUNT == true> 
  R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif MC.SINGLE_SHUNT == true>      
  R1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif MC.ICS_SENSORS == true> 
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
</#if>

  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */  
}
<#if MC.SINGLE_SHUNT == true>

void ${DMAIrq}(void)
{
  if (LL_DMA_IsActiveFlag_TC${Stream}(DMA2))
  {
    LL_DMA_ClearFlag_TC${Stream}(DMA2);
    /* When FOC execution rate is > 1, disable DMA to not insert an active window 
       on the next PWM cycles until next TIMx update */
    LL_TIM_DisableDMAReq_CC4(${TIMx});
    LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_${Stream});
  }
}
</#if>

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */ 
  if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
<#if MC.THREE_SHUNT == true> 
    R3_1_BRK_IRQHandler(&PWM_Handle_M1);
<#elseif MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>  
    R3_2_BRK_IRQHandler(&PWM_Handle_M1);
<#elseif  MC.SINGLE_SHUNT == true>  
    R1_BRK_IRQHandler(&PWM_Handle_M1);
<#elseif MC.ICS_SENSORS == true> 
    ICS_BRK_IRQHandler(&PWM_Handle_M1);
<#else>
</#if>
  }
  
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */ 
}

<#if MC.ENCODER==true || MC.AUX_ENCODER==true || MC.HALL_SENSORS==true || MC.AUX_HALL_SENSORS==true>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 
  
<#if MC.HALL_SENSORS==true || MC.AUX_HALL_SENSORS==true>
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */ 
  }
  else
  {
    /* Nothing to do */
  }
  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */ 
  }
  else
  {
  /* Nothing to do */
  }
<#else>
 /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M1.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M1.TIMx))
  { 
    LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
    ENC_IRQHandler(&ENCODER_M1);
    /* USER CODE BEGIN M1 ENCODER_Update */

    /* USER CODE END M1 ENCODER_Update   */ 
  }
  else
  {
  /* No other IT to manage for encoder config */
  }
</#if>
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if>

<#-- ST MCWB monitoring usage management (used when MC.MCP_OVER_UART_A == true) -->
<#if MC.MCP_OVER_UART_A>
/**
  * @brief This function handles DMA_RX_A channel DMACH_RX_A global interrupt.
  */
void ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 0 */
  
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 0 */

  
  /* Buffer is ready by the HW layer to be processed */ 
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A) ){
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT (&aspepOverUartA);
  }
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 1 */
  
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 1 */

}

/* This section is present only when serial communication is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void ${MC.MCP_IRQ_HANDLER_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
    
  if ( LL_USART_IsActiveFlag_TC (USARTA) )
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
    LL_DMA_DisableStream( DMA_TX_A, DMACH_TX_A );
    LL_USART_ClearFlag_TC (USARTA);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
  }
  if ( (LL_USART_IsActiveFlag_ORE (USARTA) || LL_USART_IsActiveFlag_FE (USARTA) || LL_USART_IsActiveFlag_NE (USARTA)) 
        && LL_USART_IsEnabledIT_ERROR (USARTA) )  
  { /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR (USARTA);
    LL_USART_EnableIT_IDLE (USARTA);        
  }
  if ( LL_USART_IsActiveFlag_IDLE (USARTA) && LL_USART_IsEnabledIT_IDLE (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_ClearFlag_FE(USARTA);
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_ClearFlag_NE(USARTA);

    LL_USART_DisableIT_IDLE (USARTA);
    /* Once the complete unexpected data are received, we enable back the error IT*/
    LL_USART_EnableIT_ERROR (USARTA);    
    /* To be sure we fetch the potential pendig data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX (USARTA);
    LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX (USARTA);
    LL_DMA_ClearFlag_TE (DMA_RX_A, DMACH_RX_A );    
    ASPEP_HWDMAReset (&aspepOverUartA);
  }  

  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
 
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
}

</#if>

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
  while (1)
  {

  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

<#if MC.RTOS == "NONE">

void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;  
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();
  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}
</#if>
<#function EXT_IRQHandler line>
    <#local EXTI_IRQ =
        [ {"name": "EXTI0_IRQHandler", "line": 0} 
        , {"name": "EXTI1_IRQHandler", "line": 1} 
        , {"name": "EXTI2_IRQHandler", "line": 2}
        , {"name": "EXTI3_IRQHandler", "line": 3} 
        , {"name": "EXTI4_IRQHandler", "line": 4} 
        , {"name": "EXTI9_5_IRQHandler", "line": 9}
        , {"name": "EXTI15_10_IRQHandler", "line": 15}
        ] >
    <#list EXTI_IRQ as handler >
        <#if line <= (handler.line ) >
           <#return  handler.name >
         </#if>
    </#list>
     <#return "EXTI15_10_IRQHandler" >
</#function>

<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
<#function _last_char text><#return text[text?length-1]></#function>

<#if MC.START_STOP_BTN == true>
/*GUI, this section is present only if start/stop button is enabled*/
/**
  * @brief  This function handles Button IRQ on PIN P${ _last_char(MC.START_STOP_GPIO_PORT)}${_last_word(MC.START_STOP_GPIO_PIN)}.
  */
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)} (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
  LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
  <#else>
  LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
  </#if>
  UI_HandleStartStopButton_cb ();
/* USER CODE END START_STOP_BTN */
}
</#if>

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
