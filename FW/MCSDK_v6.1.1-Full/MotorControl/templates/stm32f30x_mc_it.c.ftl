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
  * @file    stm32f30x_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32F3 Family.
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
  * @ingroup STM32F30x_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
#include "mc_type.h"
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
<#if MC.RTOS == "FREERTOS">
#include "cmsis_os.h"
</#if>
<#if MC.START_STOP_BTN == true>
#include "stm32f3xx_ll_exti.h"
</#if>
#include "stm32f3xx_it.h"

<#if MC.MCP_USED >
#include "mcp_config.h"
</#if>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F30x_IRQ_Handlers STM32F30x IRQ Handlers
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
<#-- Specific to 6_STEP algorithm usage -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
void ADC_ConvCplt_Handler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if  MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS">
void HALL_COMM_Handler(void);
void TIMx_COM_M1_Handler(void);
</#if><#-- MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS" -->
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
void PERIOD_COMM_Handler(void);
</#if><#-- MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC" -->
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
<#-- Specific to FOC algorithm usage -->

<#if MC.MCP_OVER_STLNK >
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */

  STLNK_HWDataTransmittedIT (&STLNK );
  
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}
</#if>
<#if MC.DRIVE_TYPE == "FOC">
/* Public prototypes of IRQ handlers called from assembly code ---------------*/
<#if MC.PWM_TIMER_SELECTION == "PWM_TIM1">
  <#assign DMA_nb   = "1">
  <#assign DMA_channel   = "4"> 
</#if>
<#if MC.PWM_TIMER_SELECTION == "PWM_TIM8">
  <#assign DMA_nb   = "2">
  <#assign DMA_channel   = "2">   
</#if>
<#if MC.PWM_TIMER_SELECTION2 == "PWM_TIM1">
  <#assign DMA_nb2   = "1">
  <#assign DMA_channel2   = "4"> 
</#if>
<#if MC.PWM_TIMER_SELECTION2 == "PWM_TIM8">
  <#assign DMA_nb2   = "2">
  <#assign DMA_channel2   = "2">   
</#if>

	<#if MC.ADC_PERIPH == "ADC1" || MC.ADC_PERIPH == "ADC2" || MC.ADC_PERIPH2 == "ADC1" || MC.ADC_PERIPH2 == "ADC2">
void ADC1_2_IRQHandler(void);
	</#if>
	<#if MC.ADC_PERIPH == "ADC3" || MC.ADC_PERIPH2 == "ADC3" >
void ADC3_IRQHandler(void);
	</#if>
	<#if MC.ADC_PERIPH == "ADC4" || MC.ADC_PERIPH2 == "ADC4" >
void ADC4_IRQHandler(void);
	</#if>
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
	<#if MC.ENCODER == true || MC.AUX_ENCODER == true || MC.HALL_SENSORS == true || MC.AUX_HALL_SENSORS == true>
void SPD_TIM_M1_IRQHandler(void);
	</#if>
	<#if MC.DUALDRIVE == true>
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
		<#if MC.ENCODER2 == true || MC.AUX_ENCODER2 == true || MC.HALL_SENSORS2 == true || MC.AUX_HALL_SENSORS2 == true>
void SPD_TIM_M2_IRQHandler(void);
		</#if>
	</#if>
</#if><#-- Specific to FOC algorithm usage -->
<#if MC.PFC_ENABLED == true && MC.PWM_TIMER_SELECTION == 'PWM_TIM8' && MC.SINGLEDRIVE == true>
void TIM1_UP_TIM16_IRQHandler(void);
</#if>
void HardFault_Handler(void);
void SysTick_Handler(void);
<#if MC.START_STOP_BTN == true || MC.ENC_USE_CH3 == true>
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)} (void);
</#if>
<#if MC.DRIVE_TYPE == "FOC">
	<#if MC.ADC_PERIPH == "ADC1" || MC.ADC_PERIPH == "ADC2" || MC.ADC_PERIPH2 == "ADC1" || MC.ADC_PERIPH2 == "ADC2">
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

		<#if MC.DUALDRIVE == true >
			<#if (MC.ADC_PERIPH == "ADC1" && MC.ADC_PERIPH2 == "ADC2") || 
     (MC.ADC_PERIPH == "ADC2" && MC.ADC_PERIPH2 == "ADC1") >
  // Shared IRQ management - begin
  if ( LL_ADC_IsActiveFlag_JEOS( ${MC.ADC_PERIPH}) )
  {
			</#if>
		</#if> 
		<#if MC.ADC_PERIPH == "ADC1" || MC.ADC_PERIPH == "ADC2" >
  // Clear Flags M1
  LL_ADC_ClearFlag_JEOS( ${MC.ADC_PERIPH} ); 
		</#if>

		<#if MC.DUALDRIVE == true>
			<#if (MC.ADC_PERIPH == "ADC1" && MC.ADC_PERIPH2 == "ADC2") || 
     (MC.ADC_PERIPH == "ADC2" && MC.ADC_PERIPH2 == "ADC1") >
  }
  else if ( LL_ADC_IsActiveFlag_JEOS( ${MC.ADC_PERIPH2} ) )
  {
			</#if> 
<#-- In case of same ADC for both motors, we must not clear the interrupt twice -->
			<#if MC.ADC_PERIPH2 == "ADC1" || MC.ADC_PERIPH2 == "ADC2" >
				<#if MC.ADC_PERIPH != MC.ADC_PERIPH2 >
  // Clear Flags M2
  LL_ADC_ClearFlag_JEOS( ${MC.ADC_PERIPH2} );
				</#if>
			</#if>
			<#if ( MC.ADC_PERIPH == "ADC1" && MC.ADC_PERIPH2 == "ADC2" ) || 
     (MC.ADC_PERIPH == "ADC2" && MC.ADC_PERIPH2 == "ADC1") >
  }
			</#if>
		</#if>
  // Highfrequency task 
  TSK_HighFrequencyTask();

 /* USER CODE BEGIN HighFreq */

 /* USER CODE END HighFreq  */  
 
 /* USER CODE BEGIN ADC1_2_IRQn 1 */

 /* USER CODE END ADC1_2_IRQn 1 */
}
	</#if>

	<#if MC.ADC_PERIPH == "ADC3" || MC.ADC_PERIPH2 == "ADC3" >
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC3 interrupt request.
  * @param  None
  * @retval None
  */
void ADC3_IRQHandler(void)
{
 /* USER CODE BEGIN ADC3_IRQn 0 */

 /* USER CODE END  ADC3_IRQn 0 */   

  // Clear Flags
  LL_ADC_ClearFlag_JEOS( ADC3 );

  /* Highfrequency task ADC3 */
  TSK_HighFrequencyTask();

 /* USER CODE BEGIN ADC3_IRQn 1 */

 /* USER CODE END  ADC3_IRQn 1 */  
}
	</#if>

	<#if MC.ADC_PERIPH == "ADC4" || MC.ADC_PERIPH2 == "ADC4" >
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC4 interrupt request.
  * @param  None
  * @retval None
  */
void ADC4_IRQHandler(void)
{
 /* USER CODE BEGIN ADC4_IRQn 0 */

 /* USER CODE END  ADC4_IRQn 0 */  

  // Clear Flags
  LL_ADC_ClearFlag_JEOS( ADC4 );
  
  // Highfrequency task ADC4
  TSK_HighFrequencyTask();

 /* USER CODE BEGIN ADC4_IRQn 1 */

 /* USER CODE END  ADC4_IRQn 1 */ 
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

 /* USER CODE END  TIMx_UP_M1_IRQn 0 */ 
 
	<#if MC.PFC_ENABLED == true>
  if(LL_TIM_IsActiveFlag_BRK(TIM16))
  {
    LL_TIM_ClearFlag_BRK(TIM16);
    PFC_OCP_Processing(&PFC);
  } 

  if (LL_TIM_IsActiveFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION)}))
  {
</#if>
<#if MC.SINGLE_SHUNT == true>
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION)});
    R1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif MC.ICS_SENSORS == true>     
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION)});
    ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif MC.THREE_SHUNT == true>
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION)});
    R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);    
<#else>
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION)});
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
</#if>
 <#if MC.DUALDRIVE == true>    
    TSK_DualDriveFIFOUpdate(M1);
</#if>

<#if MC.PFC_ENABLED == true>
  }
</#if>
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

 /* USER CODE END  TIMx_UP_M1_IRQn 1 */ 
}

	<#if MC.PFC_ENABLED == true && MC.PWM_TIMER_SELECTION == 'PWM_TIM8' && MC.SINGLEDRIVE == true>
/**
  * @brief  This function handles PFC function if motor 1 use TIM8.
  * @param  None
  * @retval None 
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
 /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

 /* USER CODE END  TIM1_UP_TIM16_IRQn 0 */ 
 
  if(LL_TIM_IsActiveFlag_BRK(TIM16))
  {
    LL_TIM_ClearFlag_BRK(TIM16);
    PFC_OCP_Processing(&PFC);
    /* USER CODE BEGIN PFCM1 OCP */

    /* USER CODE END  PFCM1 OCP */
  } 
 /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

 /* USER CODE END  TIM1_UP_TIM16_IRQn 1 */  
}
	</#if>

	<#if MC.PFC_ENABLED == true>
/**
  * @brief  This function schedules PFC algortihm.
  * @param  None
  * @retval None 
  */
void TIM4_IRQHandler(void)
{
  PFC_TIM_CC_IRQHandler(&PFC );
}
	</#if>

void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */ 
  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.PWM_TIMER_SELECTION)});
<#if  MC.SINGLE_SHUNT == true>  
    R1_BRK_IRQHandler(&PWM_Handle_M1);
<#elseif  MC.ICS_SENSORS == true> 
    ICS_BRK_IRQHandler(&PWM_Handle_M1);
<#elseif  MC.THREE_SHUNT == true> 
    R3_1_BRK_IRQHandler(&PWM_Handle_M1);    
<#else>
    R3_2_BRK_IRQHandler(&PWM_Handle_M1);
</#if>
  }
  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.PWM_TIMER_SELECTION)});  
<#if MC.SINGLE_SHUNT == true>  
    R1_BRK2_IRQHandler(&PWM_Handle_M1);
<#elseif MC.ICS_SENSORS == true> 
    ICS_BRK2_IRQHandler(&PWM_Handle_M1);
<#elseif MC.THREE_SHUNT == true> 
    R3_1_BRK2_IRQHandler(&PWM_Handle_M1);    
<#else>
    R3_2_BRK2_IRQHandler(&PWM_Handle_M1); 
</#if>
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */ 
}

<#if  MC.SINGLE_SHUNT == true >  
void R1_DMAx_M1_IRQHandler (void)
{
  /* USER CODE BEGIN DMAx_M1_IRQn 0 */

  /* USER CODE END DMAx_M1_IRQn 0 */  
  if (LL_DMA_IsActiveFlag_HT${DMA_channel}(DMA${DMA_nb}) && LL_DMA_IsEnabledIT_HT(DMA${DMA_nb}, LL_DMA_CHANNEL_${DMA_channel}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);      
    LL_DMA_ClearFlag_HT${DMA_channel}(DMA${DMA_nb});    
  }
   
  if (LL_DMA_IsActiveFlag_TC${DMA_channel}(DMA${DMA_nb}))
  {
    LL_DMA_ClearFlag_TC${DMA_channel}(DMA${DMA_nb});
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);      
  }
 
  /* USER CODE BEGIN DMAx_M1_IRQn 1 */

  /* USER CODE END DMAx_M1_IRQn 1 */ 

}
</#if>
<#if MC.DUALDRIVE == true>
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  * @retval None 
  */
void TIMx_UP_M2_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M2_IRQn 0 */

 /* USER CODE END  TIMx_UP_M2_IRQn 0 */ 
		<#if MC.PFC_ENABLED == true && MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1'>
  if(LL_TIM_IsActiveFlag_BRK(TIM16))
  {
    LL_TIM_ClearFlag_BRK(TIM16);
    PFC_OCP_Processing(&PFC);
    /* USER CODE BEGIN PFCM2 OCP */

    /* USER CODE END  PFCM2 OCP */ 
  } 
  
  if (LL_TIM_IsActiveFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION2)}))
  {
</#if>
 <#if MC.SINGLE_SHUNT2 == true>  
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION2)});
    R1_TIMx_UP_IRQHandler(&PWM_Handle_M2);

 <#elseif MC.ICS_SENSORS2 == true>     
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION2)}); 
    ICS_TIMx_UP_IRQHandler(&PWM_Handle_M2);
 <#else>
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.PWM_TIMER_SELECTION2)}); 
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
 </#if>
    TSK_DualDriveFIFOUpdate(M2);

		<#if MC.PFC_ENABLED == true && MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1'>
  }
		</#if>
 /* USER CODE BEGIN TIMx_UP_M2_IRQn 1 */

 /* USER CODE END  TIMx_UP_M2_IRQn 1 */ 
}

void TIMx_BRK_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 0 */

  /* USER CODE END TIMx_BRK_M2_IRQn 0 */
  
  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.PWM_TIMER_SELECTION2)}))
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.PWM_TIMER_SELECTION2)});
<#if  MC.SINGLE_SHUNT2 == true>  
    R1_BRK_IRQHandler(&PWM_Handle_M2);
<#elseif  MC.ICS_SENSORS2 == true> 
    ICS_BRK_IRQHandler(&PWM_Handle_M2);
<#else>
    R3_2_BRK_IRQHandler(&PWM_Handle_M2);
</#if>    
  /* USER CODE BEGIN BRK */

  /* USER CODE END BRK */

  }
  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.PWM_TIMER_SELECTION2)}))
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.PWM_TIMER_SELECTION2)});
    
<#if  MC.SINGLE_SHUNT2 == true>  
    R1_BRK2_IRQHandler(&PWM_Handle_M2);
<#elseif  MC.ICS_SENSORS2 == true> 
    ICS_BRK2_IRQHandler(&PWM_Handle_M2);
<#else>
    R3_2_BRK2_IRQHandler(&PWM_Handle_M2);
</#if>    
  /* USER CODE BEGIN BRK2 */

  /* USER CODE END BRK2 */
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 1 */

  /* USER CODE END TIMx_BRK_M2_IRQn 1 */
}
</#if>

<#if  MC.SINGLE_SHUNT2 == true >  
void R1_DMAx_M2_IRQHandler (void)
{
  /* USER CODE BEGIN DMAx_M2_IRQn 0 */

  /* USER CODE END DMAx_M2_IRQn 0 */  
  if (LL_DMA_IsActiveFlag_HT${DMA_channel2}(DMA${DMA_nb2}) && LL_DMA_IsEnabledIT_HT(DMA${DMA_nb2}, LL_DMA_CHANNEL_${DMA_channel2}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M2);
    LL_DMA_ClearFlag_HT${DMA_channel2}(DMA${DMA_nb2});      
  }
  
  if (LL_DMA_IsActiveFlag_TC${DMA_channel2}(DMA${DMA_nb2}))
  {
    LL_DMA_ClearFlag_TC${DMA_channel2}(DMA${DMA_nb2});
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M2);      
  }


  
  /* USER CODE BEGIN DMAx_M2_IRQn 1 */

  /* USER CODE END DMAx_M2_IRQn 1 */ 

}
</#if>

<#if MC.ENCODER == true || MC.AUX_ENCODER == true || MC.HALL_SENSORS == true || MC.AUX_HALL_SENSORS == true>

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

	<#if MC.DUALDRIVE == true>
		<#if MC.ENCODER2 == true || MC.AUX_ENCODER2 == true || MC.HALL_SENSORS2 == true || MC.AUX_HALL_SENSORS2 == true>

/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M2_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 0 */

  /* USER CODE END SPD_TIM_M2_IRQn 0 */ 

			<#if MC.HALL_SENSORS2 == true || MC.AUX_HALL_SENSORS2 == true>
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M2.TIMx) != 0)
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
<#if MC.MCP_OVER_UART_A == true>

/*Start here***********************************************************/
/*GUI, this section is present only if MCP over USARTA is enabled*/
/**
  * @brief This function handles the recepetion of a Data packet.
  * It is always the channel of the DMA configured to handle UARTA_RX data
  */
void ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 0 */
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_DMA_RX_UART_A} 0 */
  
  /* Buffer is ready by the HW layer to be processed */ 
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A) ){
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT (&aspepOverUartA);
  }
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
    LL_DMA_DisableChannel( DMA_TX_A, DMACH_TX_A );
    LL_USART_ClearFlag_TC (USARTA);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
  }
  if ( (LL_USART_IsActiveFlag_ORE (USARTA) || LL_USART_IsActiveFlag_FE (USARTA) || LL_USART_IsActiveFlag_NE (USARTA)) 
        && LL_USART_IsEnabledIT_ERROR (USARTA) )  
  { /* Stopping the debugger will generate an OverRun error*/
    WRITE_REG(USARTA->ICR, USART_ICR_FECF|USART_ICR_ORECF|USART_ICR_NCF);
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
    ASPEP_HWDMAReset (&aspepOverUartA);
  }  
 
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
 
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
}

</#if>
<#-- Specific to 6_STEP algorithm usage -->

<#if MC.DRIVE_TYPE == "SIX_STEP">
/**
  * @brief     ADC conversion complete interrupt handle 
  * @param[in] None  
  * @retval None
  */
void ADC_ConvCplt_Handler(void)
{
  /* ========== Check End of Conversion flag for regular group ========== */
  if (LL_ADC_IsActiveFlag_EOC((ADC_TypeDef *) ADC1) && LL_ADC_IsEnabledIT_EOC((ADC_TypeDef *) ADC1))
  {
    MC_Core_ProcessAdcMeasurement((uint32_t*) ADC1, (uint16_t) LL_TIM_GetCounter((TIM_TypeDef *) ((&Motor_Device1)->plf_timer)), (uint16_t) LL_ADC_REG_ReadConversionData12((ADC_TypeDef *) ADC1));
    LL_ADC_ClearFlag_EOC((ADC_TypeDef *) ADC1);
  }
  else     LL_ADC_ClearFlag_EOC((ADC_TypeDef *) ADC1);
  if (LL_ADC_IsActiveFlag_OVR((ADC_TypeDef *) ADC1) && LL_ADC_IsEnabledIT_OVR((ADC_TypeDef *) ADC1))
  {
    LL_ADC_ClearFlag_OVR((ADC_TypeDef *) ADC1);
  }
}


/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* TIM Break input event */
    if(LL_TIM_IsActiveFlag_BRK((TIM_TypeDef *) Motor_Device1.phf_timer) )
  {
    LL_TIM_ClearFlag_BRK((TIM_TypeDef *) Motor_Device1.phf_timer);      
    Motor_Device1.status = MC_OVERCURRENT;
    MC_Core_Error(&Motor_Device1);
  }
    if(LL_TIM_IsActiveFlag_BRK2((TIM_TypeDef *) Motor_Device1.phf_timer) )
  {
    LL_TIM_ClearFlag_BRK2((TIM_TypeDef *) Motor_Device1.phf_timer);      
    Motor_Device1.status = MC_OVERCURRENT;
    MC_Core_Error(&Motor_Device1);
  }
}

<#if  MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS">
/**
  * @brief     HFtimer COM interrupt handler
  * @param[in] None
  * @retval None
  */

void COMM_EVENT_M1_Handler(void)
{
  /* TIM commutation event */
    if(LL_TIM_IsActiveFlag_COM((TIM_TypeDef *) Motor_Device1.phf_timer) && LL_TIM_IsEnabledIT_COM((TIM_TypeDef *) Motor_Device1.phf_timer))
  {
    LL_TIM_ClearFlag_COM((TIM_TypeDef *) Motor_Device1.phf_timer);
  }  
}

/**
  * @brief     Hall timer interrupt handler
  * @param[in] None
  * @retval None
  */
void HALL_TIM_M1_IRQHandler(void)
{
  /* Capture compare 1 event */
    if(LL_TIM_IsActiveFlag_CC1((TIM_TypeDef *) Motor_Device1.plf_timer) && LL_TIM_IsEnabledIT_CC1((TIM_TypeDef *) Motor_Device1.plf_timer))
  {
    if (LL_TIM_OC_GetMode((TIM_TypeDef *) Motor_Device1.plf_timer,LL_TIM_CHANNEL_CH1) != LL_TIM_OCMODE_ACTIVE)
    {
      LL_TIM_ClearFlag_CC1((TIM_TypeDef *) Motor_Device1.plf_timer);
      if (Motor_Device1.status != MC_STOP)
      {
        MC_Core_NextStepScheduling(&Motor_Device1);
<#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
        MC_Core_HallNextStep(&Motor_Device1, 0);
</#if><#-- 3PWM/6PWM selection -->
      }
    }
  }  
  /* Capture compare 2 event */
  if(LL_TIM_IsActiveFlag_CC2((TIM_TypeDef *) Motor_Device1.plf_timer) && LL_TIM_IsEnabledIT_CC2((TIM_TypeDef *) Motor_Device1.plf_timer))

  {
    if (LL_TIM_OC_GetMode((TIM_TypeDef *) Motor_Device1.plf_timer,LL_TIM_CHANNEL_CH2) == LL_TIM_OCMODE_PWM2)
    {
      LL_TIM_ClearFlag_CC2((TIM_TypeDef *) Motor_Device1.plf_timer);
<#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
      MC_Core_HallNextStep(&Motor_Device1, 0);
</#if><#-- 3PWM/6PWM selection -->
      MC_Core_HallPrepareNextStep(&Motor_Device1);
    }
  }   
}
</#if><#-- MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS" -->
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
/**
  * @brief     LFtimer interrupt handler
  * @param[in] None
  * @retval None
  */

void PERIOD_COMM_Handler(void)
{
  /* TIM Update event */
    if(LL_TIM_IsActiveFlag_UPDATE((TIM_TypeDef *) Motor_Device1.plf_timer) && LL_TIM_IsEnabledIT_UPDATE((TIM_TypeDef *) Motor_Device1.plf_timer))
  {
    LL_TIM_ClearFlag_UPDATE((TIM_TypeDef *) Motor_Device1.plf_timer);      
    MC_Core_NextStep(&Motor_Device1, 0);
  }
}
</#if><#-- MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC" -->
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

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
    <#if MC.DRIVE_TYPE == "SIX_STEP">
    Motor_Device1.uw_tick_cnt++;
	</#if>
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
        , {"name": "EXTI2_TSC_IRQHandler", "line": 2}
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


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
