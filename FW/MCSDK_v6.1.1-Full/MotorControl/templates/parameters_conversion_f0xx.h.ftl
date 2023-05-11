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
<#if configs[0].peripheralParams.get("RCC")??>
<#assign RCC = configs[0].peripheralParams.get("RCC")>
</#if>
<#if !RCC??>
<#stop "No RCC found">
</#if>
</#if>

<#function _remove_last_char text><#return text[0..text?length-2]></#function>
<#function _last_char text><#return text[text?length-1]></#function>
<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
<#function TimerHandler timer><#return "${timer}_IRQHandler"></#function>

<#-- Condition for STM32F072xxx MCU -->
<#assign CondMcu_STM32F072xxx = (McuName?? && McuName?matches("STM32F072.*"))>
<#-- Condition for STM32F050xxx MCU -->
<#assign CondMcu_STM32F050xxx = (McuName?? && McuName?matches("STM32F050.*"))>
<#-- Condition for STM32F051xxx MCU -->
<#assign CondMcu_STM32F051xxx = (McuName?? && McuName?matches("STM32F051.*"))>
<#-- Condition for STM32F030xxx MCU -->
<#assign CondMcu_STM32F030xxx = (McuName?? && McuName?matches("STM32F030.*"))>
<#-- Condition for STM32F031xxx MCU -->
<#assign CondMcu_STM32F031xxx = (McuName?? && McuName?matches("STM32F031.*"))>

/**
  ******************************************************************************
  * @file    parameters_conversion_f0xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F0 Family.
  *
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F0XX_H
#define __PARAMETERS_CONVERSION_F0XX_H

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
<#assign SYSCLKFreq = RCC.get("SYSCLKFreq_VALUE")?number>
#define SYSCLK_FREQ      ${SYSCLKFreq}uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz    ${(SYSCLKFreq/(1000000))?floor}
#define ADC_CLK_MHz    14uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK       ${SYSCLKFreq}uL
#define ADC1_2  ADC1
#define REF_TIM_CLK       ${SYSCLKFreq}uL
#define REF_TIM_CLK_MHz   ${(SYSCLKFreq/(1000000))?floor}
				  														  
/*************************  IRQ Handler Mapping  *********************/	
<#if MC.DRIVE_TYPE == "FOC">													  
#define CURRENT_REGULATION_IRQHandler          DMA1_Channel1_IRQHandler
#define DMAx_R1_M1_IRQHandler                   DMA1_Channel4_5_IRQHandler
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP" && MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
#define BEMF_READING_IRQHandler          DMA1_Channel1_IRQHandler
<#if MC.LF_TIMER_SELECTION == 'LF_TIM2'>
#define PERIOD_COMM_Handler              TIM2_IRQHandler
<#elseif MC.LF_TIMER_SELECTION == 'LF_TIM4'>
#define PERIOD_COMM_Handler              TIM4_IRQHandler
<#elseif MC.LF_TIMER_SELECTION == 'LF_TIM3'>
#define PERIOD_COMM_Handler              TIM3_IRQHandler
<#elseif MC.LF_TIMER_SELECTION == 'LF_TIM16'>
#define PERIOD_COMM_Handler              TIM16_IRQHandler
</#if>
</#if>
#define TIMx_UP_BRK_M1_IRQHandler               TIM1_BRK_UP_TRG_COM_IRQHandler

<#if MC.DRIVE_TYPE == "FOC">	
/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
<#if CondMcu_STM32F051xxx || CondMcu_STM32F072xxx >
#define R1_PWM_AUX_TIM                  TIM15
<#elseif CondMcu_STM32F050xxx || CondMcu_STM32F030xxx || CondMcu_STM32F031xxx > <#-- Also needed for STSPIN -->
#define R1_PWM_AUX_TIM                  TIM3
<#else>
#error "Invalid configuration: MCU definition in parameters_conversion.h"
</#if>
</#if>

/* Referred to Maximum value indicated in the Datasheet Table 50 if ADC clock = HSI14 converted in number of cycle*/ 
/* 4 Cycles at 14Mhz = 285 ns - Table mentions 259 ns  */
#define ADC_TRIG_CONV_LATENCY_CYCLES 4
#define ADC_SAR_CYCLES 12.5

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /*__PARAMETERS_CONVERSION_F0XX_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
