<#ftl strip_whitespace = true>
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
<#if MC.PWM_TIMER_SELECTION == "PWM_TIM1">
  <#assign Stream   = "5"> 
</#if>
<#if MC.PWM_TIMER_SELECTION == "PWM_TIM8">
  <#assign Stream   = "7">   
</#if>
<#if MC.PWM_TIMER_SELECTION2 == "PWM_TIM1">
  <#assign Stream2   = "5"> 
</#if>
<#if MC.PWM_TIMER_SELECTION2 == "PWM_TIM8">
  <#assign Stream2   = "7">   
</#if>
/**
  ******************************************************************************
  * @file    stm32f4xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32F4 Family.
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
  * @ingroup STM32F4xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
	<#if MC.START_STOP_BTN == true>
#include "stm32f4xx_ll_exti.h"
	</#if>
#include "stm32f4xx_hal.h"
</#if>
#include "stm32f4xx.h"
<#-- Specific to 6_STEP algorithm usage -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
#include "stm32f4xx_it.h"
#include "6step_core.h"
	<#-- Only for the TERATERM usage -->
	<#if MC.SIX_STEP_COMMUNICATION_IF == "TERATERM_IF"><#-- TERATERM I/F usage -->
#include "6step_com.h"
	</#if>
</#if>
<#if MC.MCP_USED >
#include "mcp_config.h"
</#if>
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
<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)
</#if>
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */
<#-- Specific to 6_STEP algorithm usage -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern MC_Handle_t Motor_Device1;

void SysTick_Handler(void);
void TIM1_BRK_TIM9_IRQHandler(void);
void USART_IRQHandler(void);
</#if>

<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
	<#if MC.SINGLE_SHUNT == true >
void DMAx_R1_M1_IRQHandler(void);  
	</#if>
	<#if (MC.ENCODER == true) || (MC.AUX_ENCODER == true) || (MC.HALL_SENSORS == true) || (MC.AUX_HALL_SENSORS == true)>
void SPD_TIM_M1_IRQHandler(void);
	</#if>
	<#if MC.DUALDRIVE == true>
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
		<#if (MC.ENCODER2 == true) || (MC.AUX_ENCODER2 == true) || (MC.HALL_SENSORS2 == true) || (MC.AUX_HALL_SENSORS2 == true)>
void SPD_TIM_M2_IRQHandler(void);
		</#if>
		<#if MC.SINGLE_SHUNT2 == true >
void DMAx_R1_M2_IRQHandler(void);  
		</#if>
	</#if>

void HardFault_Handler(void);
void SysTick_Handler(void);
	<#if MC.START_STOP_BTN == true>
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)} (void);
	</#if>

/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */

<#assign ADCx=MC.M1_CS_ADC_U >
  
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  if(LL_ADC_IsActiveFlag_JEOS(${ADCx}))
  {
    // Clear Flags
    ${ADCx}->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }  
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
	<#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
		<#if MC.SINGLEDRIVE == true>
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
		<#else>
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
		</#if>
	<#elseif MC.THREE_SHUNT == true>
  R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
	<#elseif MC.ICS_SENSORS == true>
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
	<#elseif MC.SINGLE_SHUNT == true>      
		<#if MC.SINGLEDRIVE == true>
			<#if (MC.PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
			<#elseif (MC.PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M1);
			</#if>
		<#else>
			<#if (MC.PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
			<#elseif (MC.PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M1);
			</#if>
		</#if>
	</#if>
	<#if MC.DUALDRIVE == true>
  TSK_DualDriveFIFOUpdate( M1 );
	</#if>
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */  
}

	<#if MC.DUALDRIVE == true>
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M2_IRQn 0 */

  /* USER CODE END TIMx_UP_M2_IRQn 0 */ 
  LL_TIM_ClearFlag_UPDATE(PWM_Handle_M2.pParams_str->TIMx);
		<#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true>  
			<#if MC.SINGLEDRIVE==true>
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
			<#else>
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
			</#if>
		<#elseif MC.ICS_SENSORS2 == true>
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M2);
		<#elseif MC.SINGLE_SHUNT2 == true>
			<#if MC.SINGLEDRIVE==true>
			<#-- Any chance this can ever be valid? -->
				<#if (MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1') || (MC.PWM_TIMER_SELECTION2 == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M2);
				<#elseif (MC.PWM_TIMER_SELECTION2 == 'PWM_TIM8') || (MC.PWM_TIMER_SELECTION2 == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M2);
				</#if>
			<#else>
				<#if (MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1') || (MC.PWM_TIMER_SELECTION2 == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M2);
				<#elseif (MC.PWM_TIMER_SELECTION2 == 'PWM_TIM8') || (MC.PWM_TIMER_SELECTION2 == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M2);
				</#if>
			</#if>
		</#if>
  TSK_DualDriveFIFOUpdate( M2 );
  /* USER CODE BEGIN TIMx_UP_M2_IRQn 1 */

  /* USER CODE END TIMx_UP_M2_IRQn 1 */ 
}
	</#if>

	<#if MC.SINGLE_SHUNT == true >
/**
  * @brief  This function handles first motor DMAx TC interrupt request.
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M1_IRQHandler(void)
{
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 0 */

  /* USER CODE END DMAx_R1_M1_IRQn 0 */ 
  if (LL_DMA_IsActiveFlag_HT${Stream}(DMA2) && LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_${Stream}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);
    LL_DMA_ClearFlag_HT${Stream}(DMA2);      
  }
  
  if (LL_DMA_IsActiveFlag_TC${Stream}(DMA2))
  {
    LL_DMA_ClearFlag_TC${Stream}(DMA2);
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);      
  }

  /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

  /* USER CODE END DMAx_R1_M1_IRQn 1 */ 
}

		<#if MC.SINGLE_SHUNT2 == true >
/**
  * @brief  This function handles second motor DMAx TC interrupt request.
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M2_IRQHandler(void)
{
  /* USER CODE BEGIN DMAx_R1_M2_IRQn 0 */

  /* USER CODE END DMAx_R1_M2_IRQn 0 */

  if (LL_DMA_IsActiveFlag_HT${Stream2}(DMA2) && LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_${Stream2}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);      
    LL_DMA_ClearFlag_HT${Stream2}(DMA2);
  }
    
  if (LL_DMA_IsActiveFlag_TC${Stream2}(DMA2))
  {
    LL_DMA_ClearFlag_TC${Stream2}(DMA2);
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);      
  }
  
  /* USER CODE BEGIN DMAx_R1_M2_IRQn 1 */

  /* USER CODE END DMAx_R1_M2_IRQn 1 */

}
		</#if>
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
	</#if>
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */ 
}

	<#if (MC.ENCODER==true) || (MC.AUX_ENCODER==true) || (MC.HALL_SENSORS==true) || (MC.AUX_HALL_SENSORS==true)>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 
  
		<#if (MC.HALL_SENSORS==true) || (MC.AUX_HALL_SENSORS==true)>
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

	<#if MC.DUALDRIVE==true>
/**
  * @brief  This function handles second motor BRK interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 0 */

  /* USER CODE END TIMx_BRK_M2_IRQn 0 */
  
  if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M2.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M2.pParams_str->TIMx);
		<#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true>
    R3_2_BRK_IRQHandler(&PWM_Handle_M2);
		<#elseif  MC.SINGLE_SHUNT2 == true>  
    R1_BRK_IRQHandler(&PWM_Handle_M2);
		<#elseif  MC.ICS_SENSORS2 == true> 
    ICS_BRK_IRQHandler(&PWM_Handle_M2);
		<#else>
		</#if>
  /* USER CODE BEGIN BRK */

  /* USER CODE END BRK */

  }
   /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 1 */

  /* USER CODE END TIMx_BRK_M2_IRQn 1 */
}

		<#if (MC.ENCODER2==true) || (MC.AUX_ENCODER2==true) || (MC.HALL_SENSORS2==true) || (MC.AUX_HALL_SENSORS2==true)>
/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M2_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 0 */

  /* USER CODE END SPD_TIM_M2_IRQn 0 */ 

			<#if (MC.HALL_SENSORS2 == true) || (MC.AUX_HALL_SENSORS2 == true)>
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M2.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M2.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M2);
    /* USER CODE BEGIN M2 HALL_Update */

    /* USER CODE END M2 HALL_Update   */ 
  }
  else
  {
    /* Nothing to do */
  }
  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M2.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M2.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M2);
    /* USER CODE BEGIN M2 HALL_CC1 */

    /* USER CODE END M2 HALL_CC1 */ 
  }
  else
  {
  /* Nothing to do */
  }
			<#else>
  /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M2.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M2.TIMx))
  { 
    LL_TIM_ClearFlag_UPDATE(ENCODER_M2.TIMx);
    ENC_IRQHandler(&ENCODER_M2);
    /* USER CODE BEGIN M2 ENCODER_Update */

    /* USER CODE END M2 ENCODER_Update   */ 
  }
  else
  {
  /* No other IT to manage for encoder config */
  }
			</#if>
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 1 */

  /* USER CODE END SPD_TIM_M2_IRQn 1 */ 
}
		</#if>
	</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#-- ST MCWB monitoring usage management -->
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
  
  /* USER CODE END U${MC.MCP_IRQ_HANDLER_UART_A} 0 */
    
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
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_ClearFlag_FE(USARTA);
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_ClearFlag_NE(USARTA);
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR (USARTA);
    LL_USART_EnableIT_IDLE (USARTA);        
  }
  if ( LL_USART_IsActiveFlag_IDLE (USARTA) && LL_USART_IsEnabledIT_IDLE (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/
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

<#if MC.MCP_OVER_UART_B>
/**
  * @brief This function handles DMA_RX_B channel DMACH_RX_B global interrupt.
  */
void ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_B}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_B} 0 */
  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_B} 0 */
    
  /* Buffer is ready by the HW layer to be processed */ 
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_B, DMACH_RX_B) ){
    LL_DMA_ClearFlag_TC (DMA_RX_B, DMACH_RX_B);
    ASPEP_HWDataReceivedIT (&aspepOverUartB);
  }
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_B} 1 */
  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_B} 1 */  
}

/* This section is present only when serial communication is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USARTB_IRQHandler(void)
{
  /* USER CODE BEGIN USARTB_IRQn 0 */
  
  /* USER CODE END USARTB_IRQn 0 */
  if ( LL_USART_IsActiveFlag_TC (USARTB) )
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
    LL_DMA_DisableChannel( DMA_TX_B, DMACH_TX_B );
    LL_USART_ClearFlag_TC (USARTB);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT (&aspepOverUartB);
  }
  if ( (LL_USART_IsActiveFlag_ORE (USARTB) || LL_USART_IsActiveFlag_FE (USARTB) || LL_USART_IsActiveFlag_NE (USARTB)) 
        && LL_USART_IsEnabledIT_ERROR (USARTB) )  
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_ClearFlag_FE(USARTB);
    LL_USART_ClearFlag_ORE(USARTB);
    LL_USART_ClearFlag_NE(USARTB);
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR (USARTB);
    LL_USART_EnableIT_IDLE (USARTB);        
  }
  if ( LL_USART_IsActiveFlag_IDLE (USARTB) && LL_USART_IsEnabledIT_IDLE (USARTB) )
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_DisableIT_IDLE (USARTB);
    /* Once the complete unexpected data are received, we enable back the error IT*/
    LL_USART_EnableIT_ERROR (USARTB);    
    /* To be sure we fetch the potential pendig data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX (USARTB);
    LL_USART_ReceiveData8(USARTB);
    LL_USART_EnableDMAReq_RX (USARTB);
    LL_DMA_ClearFlag_TE (DMA_RX_B, DMACH_RX_B );    
    ASPEP_HWDMAReset (&aspepOverUartB);
  }  
 
  /* USER CODE BEGIN USARTB_IRQn 1 */
 
  /* USER CODE END USARTB_IRQn 1 */
}
</#if>

<#-- Specific to 6_STEP algorithm usage -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
	<#-- Only for the TERATERM usage -->
	<#if MC.SIX_STEP_COMMUNICATION_IF == "TERATERM_IF"><#-- TERATERM I/F usage -->
/* This section is present only when TeraTerm is used for 6-steps */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USART_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(UI_Params.pUART);
  /* USER CODE BEGIN USART2_IRQn 1 */
  MC_Com_ProcessInput();
  /* USER CODE END USART2_IRQn 1 */
}
	</#if>

/**
  * @brief This function handles System tick timer.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  HAL_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  * @param  None
  * @retval None
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */
  Motor_Device1.status = MC_OVERCURRENT;
  MC_Core_LL_Error(&Motor_Device1);
  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

	<#if MC.SIX_STEP_COMMUNICATION_IF == "PWM_IF"><#-- PWM I/F usage -->
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
		<#if MC.SIX_STEP_SENSING == "SENSORS_LESS"><#-- Sensorless usage -->
  /* PWM INTERFACE BEGIN 1 */
  MC_Core_LL_PwmInterfaceIrqHandler((uint32_t *) &htim2);
  /* PWM INTERFACE END 1 */
		</#if>
  /* USER CODE END TIM2_IRQn 0 */

		<#if MC.SIX_STEP_SENSING == "HALL_SENSORS"><#-- Hall Sensors usage -->
  HAL_TIM_IRQHandler(&htim2);
		</#if>

  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
		<#if MC.SIX_STEP_SENSING == "HALL_SENSORS"><#-- Hall Sensors usage -->
  /* PWM INTERFACE BEGIN 1 */
  MC_Core_LL_PwmInterfaceIrqHandler((uint32_t *) &htim4);
  /* PWM INTERFACE END 1 */
		</#if>
  /* USER CODE END TIM4_IRQn 0 */

		<#if MC.SIX_STEP_SENSING == "SENSORS_LESS"><#-- Sensorless usage -->
  HAL_TIM_IRQHandler(&htim4);
		</#if>

  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
	</#if>
</#if>

<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
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

<#if  MC.POSITION_CTRL_ENABLING == true >
    TC_IncTick(&PosCtrlM1);
</#if>

<#if  MC.POSITION_CTRL_ENABLING2 == true >
    TC_IncTick(&PosCtrlM2);
</#if>	

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

<#if MC.START_STOP_BTN == true || MC.ENC_USE_CH3 == true || MC.ENC_USE_CH32 == true >
<#-- GUI, this section is present only if start/stop button and/or Position Control with Z channel is enabled -->
  
<#assign EXT_IRQHandler_StartStopName = "" >
<#assign EXT_IRQHandler_ENC_Z_M1_Name = "" >
<#assign EXT_IRQHandler_ENC_Z_M2_Name = "" >
<#assign Template_StartStop ="">
<#assign Template_Encoder_Z_M1 ="">
<#assign Template_Encoder_Z_M2 ="">

	<#if MC.START_STOP_BTN == true>
  <#assign EXT_IRQHandler_StartStopName = "${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)}" >
  <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
  <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) ) 
  {                                                                                
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});  
    UI_HandleStartStopButton_cb ();                                               
  }'> 
		<#else>
  <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) )
  {
    LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    UI_HandleStartStopButton_cb ();
  }'> 
		</#if>
</#if>

<#if MC.ENC_USE_CH3 == true>
  <#assign EXT_IRQHandler_ENC_Z_M1_Name = "${EXT_IRQHandler(_last_word(MC.ENC_Z_GPIO_PIN)?number)}" >
  <#if _last_word(MC.ENC_Z_GPIO_PIN)?number < 32 >
  <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)}))  
  {                                                                          
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)});  
    TC_EncoderReset(&PosCtrlM1);                                            
  }'> 
  <#else>
  <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)})) 
  {                                                                          
    LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)});  
    TC_EncoderReset(&PosCtrlM1);                                            
  }'> 
  </#if> 	
</#if> 

<#if MC.ENC_USE_CH32 == true>
  <#assign EXT_IRQHandler_ENC_Z_M2_Name = "${EXT_IRQHandler(_last_word(MC.ENC_Z_GPIO_PIN2)?number)}" >
  <#if _last_word(MC.ENC_Z_GPIO_PIN2)?number < 32 >
  <#assign Template_Encoder_Z_M2 = '/* USER CODE BEGIN ENCODER Z INDEX M2 */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN2)}))  
  {                                                                           
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN2)});  
    TC_EncoderReset(&PosCtrlM2);                                             
  }'> 
  <#else>
  <#assign Template_Encoder_Z_M2 = '/* USER CODE BEGIN ENCODER Z INDEX M2 */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN2)}))  
  {                                                                            
    LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN2)});  
    TC_EncoderReset(&PosCtrlM2);                                              
  }'> 
  </#if> 
</#if> 
  
<#if MC.START_STOP_BTN == true>
/**
  * @brief  This function handles Button IRQ on PIN P${ _last_char(MC.START_STOP_GPIO_PORT)}${_last_word(MC.START_STOP_GPIO_PIN)}.
<#if MC.ENC_USE_CH3 == true && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}">
  *                 and M1 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT)}${_last_word(MC.ENC_Z_GPIO_PIN)}.
</#if>  
<#if MC.ENC_USE_CH32 == true && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}">
  *                 and M2 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT2)}${_last_word(MC.ENC_Z_GPIO_PIN2)}.
</#if>  
  */
void ${EXT_IRQHandler_StartStopName} (void)
{
	${Template_StartStop}

	<#if "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}" >
	${Template_Encoder_Z_M1}
	</#if>
	
	<#if "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" >
	${Template_Encoder_Z_M2}
	</#if>
}
</#if>

<#if MC.ENC_USE_CH3 == true>
	<#if "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M1_Name}" >
/**
  * @brief  This function handles M1 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT)}${_last_word(MC.ENC_Z_GPIO_PIN)}.
<#if MC.ENC_USE_CH32 == true && "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" >
  *                 and M2 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT2)}${_last_word(MC.ENC_Z_GPIO_PIN2)}.
</#if>  
  */
void ${EXT_IRQHandler_ENC_Z_M1_Name} (void)
{
	${Template_Encoder_Z_M1}
	
	<#if "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" >
	${Template_Encoder_Z_M2}
	</#if>

}	
	</#if>
</#if> 

<#if MC.ENC_USE_CH32 == true>
	<#if "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M2_Name}" && "${EXT_IRQHandler_ENC_Z_M1_Name}" != "${EXT_IRQHandler_ENC_Z_M2_Name}">
/**
  * @brief  This function handles M2 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT2)}${_last_word(MC.ENC_Z_GPIO_PIN2)}.
  */	
void ${EXT_IRQHandler_ENC_Z_M2_Name} (void)
{
	${Template_Encoder_Z_M2}
	
}	
	</#if>
</#if> 

</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
