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
/**
  ******************************************************************************
  * @file    stm32f0xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32F0 Family.
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
  * @ingroup STM32F0xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
<#if MC.MCP_USED>
#include "mcp_config.h"
</#if>
<#-- Specific to FOC algorithm usage -->
#include "mc_type.h"
#include "mc_tasks.h"

#include "parameters_conversion.h"
#include "motorcontrol.h"
	<#if MC.START_STOP_BTN == true>
#include "stm32f0xx_ll_exti.h"
	</#if>
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
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
<#if MC.DRIVE_TYPE == "FOC">
void CURRENT_REGULATION_IRQHandler(void);
void DMAx_R1_M1_IRQHandler(void);
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
void BEMF_READING_IRQHandler(void);
void PERIOD_COMM_Handler(void);
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
void TIMx_UP_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void USART_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)} (void);

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief  This function handles current regulation interrupt request.
  * @param  None
  * @retval None
  */
void CURRENT_REGULATION_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */
  
  /* Clear Flags */
  DMA1->IFCR = (LL_DMA_ISR_GIF1|LL_DMA_ISR_TCIF1|LL_DMA_ISR_HTIF1);

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */          
    TSK_HighFrequencyTask();

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */   
}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */   

  if(LL_TIM_IsActiveFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_UPDATE(PWM_Handle_M1.pParams_str->TIMx))
  {
<#if MC.DRIVE_TYPE == "FOC">
	<#if MC.SINGLE_SHUNT == true>  
    R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);    
	<#elseif MC.THREE_SHUNT == true>
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    R3_1_TIMx_UP_IRQHandler( &PWM_Handle_M1 );
	</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    (void)TSK_HighFrequencyTask();
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
    /* USER CODE BEGIN PWM_Update */

    /* USER CODE END PWM_Update */  
  }
  if(LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_BRK(PWM_Handle_M1.pParams_str->TIMx)) 
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);   
<#if MC.DRIVE_TYPE == "FOC">
   <#if MC.SINGLE_SHUNT == true>    
    R1_BRK_IRQHandler(&PWM_Handle_M1);
   <#elseif MC.THREE_SHUNT == true>
   F0XX_BRK_IRQHandler(&PWM_Handle_M1);
   </#if> 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
<#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
    SixPwm_BRK_IRQHandler(&PWM_Handle_M1);
<#else>
    ThreePwm_BRK_IRQHandler(&PWM_Handle_M1);
</#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else 
  {
   /* No other interrupts are routed to this handler */
  }
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */   
}

<#if MC.DRIVE_TYPE == "FOC">
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
  if (LL_DMA_IsActiveFlag_TC5(DMA1))
  {
    LL_DMA_ClearFlag_TC5(DMA1);
	<#if MC.SINGLE_SHUNT == true>     
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);
	<#elseif MC.THREE_SHUNT == true>
	</#if>
    /* USER CODE BEGIN DMAx_R1_M1_TC5 */

    /* USER CODE END DMAx_R1_M1_TC5 */     
  } 

  /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

  /* USER CODE END DMAx_R1_M1_IRQn 1 */ 
}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
/**
  * @brief  This function handles BEMF sensing interrupt request.
  * @param[in] None
  * @retval None
  */
void BEMF_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_DMA_IsActiveFlag_TC1(DMA1) && LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1))
  {
  /* Clear Flags */
    LL_DMA_ClearFlag_TC1( DMA1 );
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */
    BADC_IsZcDetected( &Bemf_ADC_M1, &PWM_Handle_M1._Super );
  }
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief     LFtimer interrupt handler
  * @param[in] None
  * @retval None
  */
void PERIOD_COMM_Handler(void)
{
  /* TIM Update event */

  if(LL_TIM_IsActiveFlag_UPDATE(Bemf_ADC_M1.pParams_str->LfTim) && LL_TIM_IsEnabledIT_UPDATE(Bemf_ADC_M1.pParams_str->LfTim))
  {
    LL_TIM_ClearFlag_UPDATE(Bemf_ADC_M1.pParams_str->LfTim);
    BADC_LfTim_UP_IRQHandler(&Bemf_ADC_M1);
  }
}

</#if>
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
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN HALL_Update */

    /* USER CODE END HALL_Update   */ 
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
    /* USER CODE BEGIN HALL_CC1 */

    /* USER CODE END HALL_CC1 */ 
  }
  else
  {
  /* Nothing to do */
  }
		<#else><#-- MC.HALL_SENSORS==true || MC.AUX_HALL_SENSORS==true -->
 /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M1.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M1.TIMx))
  { 
    LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
    ENC_IRQHandler(&ENCODER_M1);
    /* USER CODE BEGIN ENCODER_Update */

    /* USER CODE END ENCODER_Update   */ 
  }
  else
  {
  /* No other IT to manage for encoder config */
  }
		</#if><#-- end of else if MC.HALL_SENSORS==true || MC.AUX_HALL_SENSORS==true -->
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
	</#if><#-- MC.ENCODER==true || MC.AUX_ENCODER==true || MC.HALL_SENSORS==true || MC.AUX_HALL_SENSORS==true -->

<#if MC.MCP_OVER_UART_A >
void DMA1_Channel2_3_IRQHandler (void)
{  
  /* Buffer is ready by the HW layer to be processed */ 
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A) ){
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT (&aspepOverUartA);
  }
}

void ${MC.MCP_IRQ_HANDLER_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  if ( LL_USART_IsActiveFlag_TC (USARTA) )
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
    LL_DMA_DisableChannel( DMA_TX_A, DMACH_TX_A );
    LL_USART_ClearFlag_TC (USARTA);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
  }
  if ( LL_USART_IsActiveFlag_ORE (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_ClearFlag_ORE (USARTA);
    LL_USART_EnableIT_IDLE (USARTA);    
  }
  if ( LL_USART_IsActiveFlag_IDLE (USARTA) && LL_USART_IsEnabledIT_IDLE (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/

    //LL_USART_ClearFlag_IDLE (USARTA);
    LL_USART_DisableIT_IDLE (USARTA);
    /* To be sure we fetch the potential pendig data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX (USARTA);
    LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX (USARTA);
    ASPEP_HWDMAReset (&aspepOverUartA);

  }  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A}n 0 */

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
  else
  {
    /* Nothing to do */
  }
  SystickDividerCounter ++;  
#endif /* MC_HAL_IS_USED */

<#if MC.MCP_OVER_UART_A>
  /* Buffer is ready by the HW layer to be processed */ 
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A))
  {
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT(&aspepOverUartA);
  }
  else
  {
    /* Nothing to do */
  }
</#if>

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */

    MC_RunMotorControlTasks();

<#if  MC.POSITION_CTRL_ENABLING == true >
    TC_IncTick(&PosCtrlM1);
</#if>

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

</#if> <#--  MC.RTOS == "NONE" -->
	<#function EXT_IRQHandler line>
	<#local EXTI_IRQ =
        [ {"name": "EXTI0_1_IRQHandler", "line": 1} 
        , {"name": "EXTI2_3_IRQHandler", "line": 3} 
        , {"name": "EXTI4_15_IRQHandler", "line": 15}
        ] >
	<#list EXTI_IRQ as handler >
        <#if line <= (handler.line ) >
           <#return  handler.name >
         </#if>
	</#list>
	<#return "EXTI4_15_IRQHandler" >
	</#function>

	<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
	<#function _last_char text><#return text[text?length-1]></#function>

<#if MC.START_STOP_BTN == true || MC.ENC_USE_CH3 == true>
<#-- GUI, this section is present only if start/stop button and/or Position Control with Z channel is enabled -->
  
<#assign EXT_IRQHandler_StartStopName = "" >
<#assign EXT_IRQHandler_ENC_Z_M1_Name = "" >
<#assign Template_StartStop ="">
<#assign Template_Encoder_Z_M1 ="">

	<#if MC.START_STOP_BTN == true>
  <#assign EXT_IRQHandler_StartStopName = "${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)}" >
  <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
    <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) ) 
  {                                                                                
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});  
    UI_HandleStartStopButton_cb ();                                               
  }'> 
  <#else><#-- _last_word(MC.START_STOP_GPIO_PIN)?number < 32 -->
    <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) )
  {
    LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    UI_HandleStartStopButton_cb ();
  }'> 
  </#if><#-- else _last_word(MC.START_STOP_GPIO_PIN)?number < 32 -->
</#if><#-- MC.START_STOP_BTN == true -->

<#if MC.ENC_USE_CH3 == true>
  <#assign EXT_IRQHandler_ENC_Z_M1_Name = "${EXT_IRQHandler(_last_word(MC.ENC_Z_GPIO_PIN)?number)}" >
  <#if _last_word(MC.ENC_Z_GPIO_PIN)?number < 32 >
  <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)}))  
  {                                                                          
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)});  
    TC_EncoderReset(&PosCtrlM1);                                            
  }'> 
  <#else><#-- _last_word(MC.ENC_Z_GPIO_PIN)?number < 32 -->
  <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)})) 
  {                                                                          
    LL_EXTI_ClearFlag_32_63 (LL_EXTI_LINE_${_last_word(MC.ENC_Z_GPIO_PIN)});  
    TC_EncoderReset(&PosCtrlM1);                                            
  }'> 
  </#if><#-- else _last_word(MC.ENC_Z_GPIO_PIN)?number < 32 -->
</#if><#-- MC.ENC_USE_CH3 == true -->
  
  <#if MC.START_STOP_BTN == true>
/**
  * @brief  This function handles Button IRQ on PIN P${ _last_char(MC.START_STOP_GPIO_PORT)}${_last_word(MC.START_STOP_GPIO_PIN)}.
    <#if MC.ENC_USE_CH3 == true && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}">
  *                 and M1 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT)}${_last_word(MC.ENC_Z_GPIO_PIN)}.
    </#if>  
  */
void ${EXT_IRQHandler_StartStopName} (void)
{
	${Template_StartStop}

	  <#if "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}" >
	${Template_Encoder_Z_M1}
  </#if>
	
}
  </#if><#-- MC.START_STOP_BTN == true -->

  <#if MC.ENC_USE_CH3 == true>
	  <#if "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M1_Name}" >
/**
  * @brief  This function handles M1 Encoder Index IRQ on PIN P${ _last_char(MC.ENC_Z_GPIO_PORT)}${_last_word(MC.ENC_Z_GPIO_PIN)}.
  */
void ${EXT_IRQHandler_ENC_Z_M1_Name} (void)
{
	${Template_Encoder_Z_M1}
	
}	
	  </#if><#-- "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M1_Name}" -->
  </#if><#-- MC.ENC_USE_CH3 == true -->
</#if><#-- MC.START_STOP_BTN == true || MC.ENC_USE_CH3 == true -->

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/