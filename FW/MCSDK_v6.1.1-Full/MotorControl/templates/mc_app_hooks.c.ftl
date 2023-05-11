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
<#assign CondIncludeMCConfig = (MC.M1_POTENTIOMETER_ENABLE == true) || (MC.ESC_ENABLE == true) >
<#assign CondBootHookUsed = (MC.M1_POTENTIOMETER_ENABLE == true) || (MC.ESC_ENABLE == true) >
<#assign CondPostMediumFrequencyHookM1Used = (MC.M1_POTENTIOMETER_ENABLE == true) || (MC.ESC_ENABLE == true) >
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"
<#if CondIncludeMCConfig>
#include "mc_config.h"
</#if>
<#if MC.M1_POTENTIOMETER_ENABLE == true >
#include "speed_potentiometer.h"
</#if>
<#if MC.ESC_ENABLE >
#include "esc.h"
</#if>

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks
 *
 *
 * @{
 */

/**
 * @brief Hook function called right after the Medium Frequency Task
 * 
 * 
 * 
 */
__weak void MC_APP_BootHook(void)
{
<#if CondBootHookUsed>
  <#if MC.M1_POTENTIOMETER_ENABLE == true >
  SPDPOT_Init( & SpeedPotentiometer_M1 );
  </#if>  
  <#if MC.ESC_ENABLE == true>
  esc_boot(&ESC_M1);
  </#if>
<#else>
  /* 
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

</#if>
/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task
 * 
 * 
 * 
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void) 
{
<#if CondPostMediumFrequencyHookM1Used >
  <#if MC.M1_POTENTIOMETER_ENABLE == true>
  SPDPOT_Run( & SpeedPotentiometer_M1 );
  </#if>  
  <#if MC.ESC_ENABLE == true>
  esc_pwm_control(&ESC_M1); 
  </#if>
<#else>
  /* 
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1.
   */

</#if>
/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
