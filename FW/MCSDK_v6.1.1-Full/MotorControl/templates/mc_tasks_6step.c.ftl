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
<#-- Condition for STM32F302x8x MCU -->
<#assign CondMcu_STM32F302x8x = (McuName?? && McuName?matches("STM32F302.8.*"))>
<#-- Condition for STM32F072xxx MCU -->
<#assign CondMcu_STM32F072xxx = (McuName?? && McuName?matches("STM32F072.*"))>
<#-- Condition for STM32F446xCx or STM32F446xEx -->
<#assign CondMcu_STM32F446xCEx = (McuName?? && McuName?matches("STM32F446.(C|E).*"))>
<#-- Condition for STM32F0 Family -->
<#assign CondFamily_STM32F0 = (FamilyName?? && FamilyName=="STM32F0")>
<#-- Condition for STM32G0 Family -->
<#assign CondFamily_STM32G0 = (FamilyName?? && FamilyName=="STM32G0") >
<#-- Condition for STM32F3 Family -->
<#assign CondFamily_STM32F3 = (FamilyName?? && FamilyName == "STM32F3")>
<#-- Condition for STM32F4 Family -->
<#assign CondFamily_STM32F4 = (FamilyName?? && FamilyName == "STM32F4") >
<#-- Condition for STM32G4 Family -->
<#assign CondFamily_STM32G4 = (FamilyName?? && FamilyName == "STM32G4") >
<#-- Condition for STM32L4 Family -->
<#assign CondFamily_STM32L4 = (FamilyName?? && FamilyName == "STM32L4") >
<#-- Condition for STM32F7 Family -->
<#assign CondFamily_STM32F7 = (FamilyName?? && FamilyName == "STM32F7") >
<#-- Condition for STM32H7 Family -->
<#assign CondFamily_STM32H7 = (FamilyName?? && FamilyName == "STM32H7") >
<#-- Define some helper symbols -->

<#-- Charge Boot Cap enable condition -->
<#assign CHARGE_BOOT_CAP_ENABLING = ! MC.OTF_STARTUP>
<#assign CHARGE_BOOT_CAP_ENABLING2 = ! MC.OTF_STARTUP2>
<#if MC.OVERMODULATION == true> <#assign OVM ="_OVM"> <#else> <#assign OVM =""> </#if>
<#if MC.OVERMODULATION2 == true> <#assign OVM2 ="_OVM"> <#else>	<#assign OVM2 =""> </#if>

/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1 
#include "mc_tasks.h"
#include "parameters_conversion.h"
<#if MC.MCP_USED == true>
#include "mcp_config.h"
</#if>
<#if MC.DAC_FUNCTIONALITY>
#include "dac_ui.h"
</#if>

<#if MC.TESTENV == true && MC.PFC_ENABLED == false >
#include "mc_testenv.h"
</#if>
#include "mc_config.h"
#include "mc_interface.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */

/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

  #define M1_CHARGE_BOOT_CAP_MS  ((uint16_t)10)
  #define M2_CHARGE_BOOT_CAP_MS ((uint16_t)10)
  #define STOPPERMANENCY_MS   ((uint16_t)400)
  #define STOPPERMANENCY_MS2  ((uint16_t)400)
  #define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * M1_CHARGE_BOOT_CAP_MS) / ((uint16_t)1000))
  #define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * M2_CHARGE_BOOT_CAP_MS)/ ((uint16_t)1000))
  #define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
  #define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
extern MC_Handle_t Motor_Device1;
uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
<#if  MC.DUALDRIVE == true>
static volatile uint16_t hMFTaskCounterM2 = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);
</#if>

static volatile uint8_t bMCBootCompleted = ((uint8_t)0);

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
MC_FuncStatus_t MC_Core_MediumFrequencyTask(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
<#if MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM">
void TSK_SafetyTask_PWMOFF(uint8_t motor);
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
void TSK_SafetyTask_RBRK(uint8_t motor);
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES">
void TSK_SafetyTask_LSON(uint8_t motor);
</#if>
<#if  MC.DUALDRIVE == true>
void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
bool TSK_StopPermanencyTimeHasElapsedM2(void);

#define FOC_ARRAY_LENGTH 2
static uint8_t FOC_array[FOC_ARRAY_LENGTH]={0, 0};
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived
</#if>
<#if MC.PFC_ENABLED == true>
void PFC_Scheduler(void);
</#if>

<#if  MC.EXAMPLE_SPEEDMONITOR == true>
/****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/
   void ARR_TIM5_update(SpeednPosFdbk_Handle_t pSPD);
/**************************************************************************/
</#if>
/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */

  __weak void MCboot( MC_Handle_t* pMCIList[] )
  {
    /* USER CODE BEGIN MCboot 0 */
  
    /* USER CODE END MCboot 0 */
    
    <#-- Moved to mc_config
    MC_Core_AssignTimers(&Motor_Device1, (uint32_t *)&htim1, (uint32_t *)&htim2, (uint32_t *)&htim3, NULL);
    MC_Core_ConfigureUserAdc(&Motor_Device1, pTrigTim, TrigTimChannel, ADC_USER_NUMBER_OF_CHANNELS);
    MC_Core_ConfigureUserAdcChannel(&Motor_Device1, (uint32_t *)&hadc, STEVAL_SPIN3202_ADC_SPEED_CH, STEVAL_SPIN3202_ADC_SPEED_ST, MC_USER_MEAS_1);
    MC_Core_ConfigureUserAdcChannel(&Motor_Device1, (uint32_t *)&hadc, STEVAL_SPIN3202_ADC_CURRENT_CH, STEVAL_SPIN3202_ADC_CURRENT_ST, MC_USER_MEAS_2);
    MC_Core_ConfigureUserAdcChannel(&Motor_Device1, (uint32_t *)&hadc, STEVAL_SPIN3202_ADC_VBUS_CH, STEVAL_SPIN3202_ADC_VBUS_ST, MC_USER_MEAS_3);
    MC_Core_ConfigureUserAdcChannel(&Motor_Device1, (uint32_t *)&hadc, STEVAL_SPIN3202_ADC_TEMP_CH, STEVAL_SPIN3202_ADC_TEMP_ST, MC_USER_MEAS_4);
    MC_Core_ConfigureUserAdcChannel(&Motor_Device1, (uint32_t *)&hadc, STEVAL_SPIN3202_ADC_VREF_CH, STEVAL_SPIN3202_ADC_VREF_ST, MC_USER_MEAS_5);
    MC_Core_ConfigureUserButton(&Motor_Device1, (uint16_t) M1_STARTSTOP_BUTTON_Pin, (uint16_t) 500);
    -->
    bMCBootCompleted = 0U;
    if (MC_Core_Init(&Motor_Device1) != MC_FUNC_OK)
    {
      Error_Handler();
    }
    
    /* USER CODE BEGIN MCboot 1 */
  
    /* USER CODE END MCboot 1 */
  
  <#if MC.MCP_OVER_UART_A >
    ASPEP_start(&aspepOverUartA);
  </#if>
  <#if MC.MCP_OVER_STLNK >  
    STLNK_init(&STLNK);
  </#if>
  
    /* USER CODE BEGIN MCboot 2 */
  
    /* USER CODE END MCboot 2 */
  
    bMCBootCompleted = 1U;
  }


/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();
<#if MC.RTOS == "NONE">    

    /* Safety task is run after Medium Frequency task so that  
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
    
</#if>
<#if MC.PFC_ENABLED == true>
    /* ** Power Factor Correction Task ** */ 
    PFC_Scheduler();
</#if>
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance. 
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (((uint8_t)1) == bMCBootCompleted)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {

      MC_Core_MediumFrequencyTask();

      
<#if MC.MCP_OVER_UART_A >
      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer, 
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer, 
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC)) 
        {
          /* no buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer, 
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* no buffer available to build the answer ... should not occur */
        }
      }
</#if>
<#if MC.MCP_OVER_UART_B >
      MCP_Over_UartB.rxBuffer = MCP_Over_UartB.pTransportLayer->fRXPacketProcess(MCP_Over_UartB.pTransportLayer,
                                                                                 &MCP_Over_UartB.rxLength);
      if (MCP_Over_UartB.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_UartB.pTransportLayer->fGetBuffer(MCP_Over_UartB.pTransportLayer,
                                                       (void **) &MCP_Over_UartB.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_UartB);
          MCP_Over_UartB.pTransportLayer->fSendPacket(MCP_Over_UartB.pTransportLayer, MCP_Over_UartB.txBuffer,
                                                      MCP_Over_UartB.txLength, MCTL_SYNC);
        }
        else 
        {
          /* no buffer available to build the answer ... should not occur */
        }
      }
</#if>      
<#if MC.MCP_OVER_STLNK >
      MCP_Over_STLNK.rxBuffer = MCP_Over_STLNK.pTransportLayer->fRXPacketProcess( MCP_Over_STLNK.pTransportLayer,
                                                                                  &MCP_Over_STLNK.rxLength);
      if (0U == MCP_Over_STLNK.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_STLNK.pTransportLayer->fGetBuffer(MCP_Over_STLNK.pTransportLayer,
                                      (void **) &MCP_Over_STLNK.txBuffer, MCTL_SYNC)) //cstat !MISRAC2012-Rule-11.3
        {
          /* no buffer available to build the answer ... should not occur */
        }
        else 
        {
          MCP_ReceivedPacket(&MCP_Over_STLNK);
          MCP_Over_STLNK.pTransportLayer->fSendPacket (MCP_Over_STLNK.pTransportLayer, MCP_Over_STLNK.txBuffer,
                                                       MCP_Over_STLNK.txLength, MCTL_SYNC);
        }
      }
</#if>
      
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
<#if  MC.EXAMPLE_SPEEDMONITOR == true>
      /****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/
      ARR_TIM5_update(${SPD_M1});

      /**************************************************************************/
</#if>
      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
<#if  MC.DUALDRIVE == true>
    if(hMFTaskCounterM2 > ((uint16_t )0))
    {
      hMFTaskCounterM2--;
    }
    else
    {
      TSK_MediumFrequencyTaskM2();
      /* USER CODE BEGIN MC_Scheduler MediumFrequencyTask M2 */

      /* USER CODE END MC_Scheduler MediumFrequencyTask M2 */
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
</#if>
    if(hBootCapDelayCounterM1 > 0U)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
<#if  MC.DUALDRIVE == true>
    if(hBootCapDelayCounterM2 > 0U)
    {
      hBootCapDelayCounterM2--;
    }
    if(hStopPermanencyCounterM2 > 0U)
    {
      hStopPermanencyCounterM2--;
    }
</#if>
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak MC_FuncStatus_t MC_Core_MediumFrequencyTask()
{
  switch (Motor_Device1.status)
  {
    case MC_RUN:
    {
       Motor_Device1.speed_target_value = Motor_Device1.speed_target_command;
       MC_Core_DisableIrq();
       Motor_Device1.pulse_value = MC_Core_SpeedControl(&Motor_Device1);
<#if MC.DRIVE_MODE == "CM">
       MC_Core_SetDutyCycleRefPwm(Motor_Device1.pref_timer, Motor_Device1.pulse_value);
</#if>
<#if MC.DRIVE_MODE == "VM">
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
      if ((0 == Motor_Device1.bemf.pwm_on_sensing_enabled) 
      && (Motor_Device1.pulse_value > Motor_Device1.bemf.pwm_on_sensing_en_thres))
      {
        (Motor_Device1.bemf.pwm_on_sensing_enabled)++;
        Motor_Device1.bemf.trig_time = Motor_Device1.bemf.pwm_on_sensing_trig_time;
        Motor_Device1.bemf.adc_threshold_up = Motor_Device1.bemf.const_adc_threshold_up_on; 
        Motor_Device1.bemf.adc_threshold_down = Motor_Device1.bemf.const_adc_threshold_down_on;
        MC_Core_ResetBemfGpio(&Motor_Device1);
      }
      else if ((Motor_Device1.bemf.pwm_on_sensing_enabled == 1)
      && (Motor_Device1.pulse_value < Motor_Device1.bemf.pwm_on_sensing_dis_thres))
      {
        Motor_Device1.bemf.pwm_on_sensing_enabled = 0;
        Motor_Device1.bemf.trig_time = Motor_Device1.bemf.pwm_off_sensing_trig_time;
        Motor_Device1.bemf.adc_threshold_up = Motor_Device1.bemf.const_adc_threshold_up;
        Motor_Device1.bemf.adc_threshold_down = Motor_Device1.bemf.const_adc_threshold_down;
        MC_Core_SetBemfGpio(&Motor_Device1);
      }
</#if>
      Motor_Device1.hf_timer_pulse_value_max = Motor_Device1.pulse_value;	  
<#if MC.FAST_DEMAG == true && MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" && MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
		if (Motor_Device1.bemf.demagn_counter < Motor_Device1.bemf.demagn_value)
		{
			switch (Motor_Device1.step_position)
			{
			case 1:
			{
				MC_Core_SetDutyCycleHfPwmV(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max);
			}
			break;
			case 3:
			{
				MC_Core_SetDutyCycleHfPwmW(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max);
			}
			break;
			case 5:
			{
				MC_Core_SetDutyCycleHfPwmU(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max);
			}
			break;
			default:
			{
                MC_Core_SetDutyCycleHfPwmForStepN(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max, 
                                        Motor_Device1.step_position);
			}
			break;
			}
		}
<#else>
		MC_Core_SetDutyCycleHfPwmForStepN(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_position);
</#if>
</#if>
      MC_Core_EnableIrq();
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
      if (0 == Motor_Device1.step_prepared)
      {
        MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.bemf.ptrig_timer, Motor_Device1.bemf.trig_timer_channel,
                                          Motor_Device1.bemf.trig_time);
      } 
      else
      {  
			if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer, Motor_Device1.adc_user.trig_timer_channel, Motor_Device1.adc_user.trig_time);
      }
<#else>	  
		if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer, Motor_Device1.adc_user.trig_timer_channel, Motor_Device1.adc_user.trig_time);
</#if>
  }
	break;
    case MC_ALIGNMENT:
    {
<#if MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS">
        MC_Core_AlignmentToCurrentStep(&Motor_Device1);
</#if>
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
    MC_Core_Alignment(&Motor_Device1);
</#if>  
    }
    break;
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
    case MC_STARTUP:
    {
       MC_Core_RampCalc(&Motor_Device1);
    }
    break;
</#if>  
	default:
	break;
  }
  return MC_FUNC_OK; 
}

/**
  * @brief  Process the ADC measurement
  * @param[in] pAdc pointer to the ADC
  * @param[in] LfTimerCounterSnapshot value of the LF timer counter at the beginning of the ADC callback
  * @param[in] AdcMeasurement the ADC conversion result, aka the ADC measurement
  * @retval  Function Status
  */
MC_FuncStatus_t  MC_Core_ProcessAdcMeasurement(uint32_t* pAdc, uint16_t LfTimerCounterSnapshot, uint16_t AdcMeasurement)
{
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
  if (0 == Motor_Device1.step_prepared)
  {
    Motor_Device1.step_change = 0;
    if (Motor_Device1.bemf.demagn_counter > Motor_Device1.bemf.demagn_value)
    {
      if (0 == Motor_Device1.direction)
      {
        if ((0 == (Motor_Device1.step_position&0x1)) && (AdcMeasurement > Motor_Device1.bemf.adc_threshold_up))
        {
          {
            MC_Core_LfTimerPeriodCompute(&Motor_Device1, LfTimerCounterSnapshot, 1);
            MC_Core_PrepareNextStep(&Motor_Device1);
        <#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
            MC_Core_SetHFPWM(&Motor_Device1, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_pos_next);
        </#if><#-- 3PWM/6PWM selection -->
            MC_Core_SelectAdcChannelDuringCallback(Motor_Device1.bemf.padc, 
                                       Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]);
            if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer,
	                                                                                          Motor_Device1.adc_user.trig_timer_channel,
												  Motor_Device1.adc_user.trig_time);
          }
        }
        if (((Motor_Device1.step_position&0x1) != 0) && (AdcMeasurement < Motor_Device1.bemf.adc_threshold_down))
        {
          {
            MC_Core_LfTimerPeriodCompute(&Motor_Device1, LfTimerCounterSnapshot, 0);
            MC_Core_PrepareNextStep(&Motor_Device1);
       <#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
            MC_Core_SetHFPWM(&Motor_Device1, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_pos_next);
       </#if><#-- 3PWM/6PWM selection -->
            MC_Core_SelectAdcChannelDuringCallback(Motor_Device1.bemf.padc, 
			                                       Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index], 
			                                       Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index], 
			                                       Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]); 
            if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer,
	                                                                                              Motor_Device1.adc_user.trig_timer_channel,
												                                                  Motor_Device1.adc_user.trig_time);
          }
        }
      }
      else
      {
        if (((Motor_Device1.step_position&0x1) != 0) && (AdcMeasurement > Motor_Device1.bemf.adc_threshold_up))
        {
          {
            MC_Core_LfTimerPeriodCompute(&Motor_Device1, LfTimerCounterSnapshot, 1);
           MC_Core_PrepareNextStep(&Motor_Device1);
        <#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
            MC_Core_SetHFPWM(&Motor_Device1, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_pos_next);
        </#if><#-- 3PWM/6PWM selection -->
            MC_Core_SelectAdcChannelDuringCallback(Motor_Device1.bemf.padc,
                                       Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]);
            if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer,
	                                                                                          Motor_Device1.adc_user.trig_timer_channel,
												  Motor_Device1.adc_user.trig_time);
          }
        }
        if ((0 == (Motor_Device1.step_position&0x1)) && (AdcMeasurement < Motor_Device1.bemf.adc_threshold_down))
        {
          {
            MC_Core_LfTimerPeriodCompute(&Motor_Device1, LfTimerCounterSnapshot, 0);
            MC_Core_PrepareNextStep(&Motor_Device1);
        <#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
            MC_Core_SetHFPWM(&Motor_Device1, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_pos_next);
        </#if><#-- 3PWM/6PWM selection -->
            MC_Core_SelectAdcChannelDuringCallback(Motor_Device1.bemf.padc,
                                       Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index],
                                       Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]);
            if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer,
	                                                                                          Motor_Device1.adc_user.trig_timer_channel,
												  Motor_Device1.adc_user.trig_time);
          }
        }
      }
    }
    else
    {
      Motor_Device1.bemf.demagn_counter++;
<#if MC.FAST_DEMAG == true && MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" && MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
      if (Motor_Device1.bemf.demagn_counter > Motor_Device1.bemf.demagn_value)
      {
        switch (Motor_Device1.step_position)
        {
          case 1:
          {
            MC_Core_SetDutyCycleHfPwmV(Motor_Device1.phf_timer, 0);
            MC_Core_SetDutyCycleHfPwmU(Motor_Device1.phf_timer, Motor_Device1.pulse_value);
            break;
          }
          
          case 3:
          {
            MC_Core_SetDutyCycleHfPwmW(Motor_Device1.phf_timer, 0);
            MC_Core_SetDutyCycleHfPwmV(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max);
            break;
          }

          case 5:
          {
            MC_Core_SetDutyCycleHfPwmW(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max);
            MC_Core_SetDutyCycleHfPwmU(Motor_Device1.phf_timer, 0);
            break;
          }
          
          default:
            break;
        }
        MC_Core_ResetPolarityHfPwm(Motor_Device1.phf_timer, Motor_Device1.channel_polarity, Motor_Device1.Nchannel_polarity);
        MC_Core_GenerateComEvent(Motor_Device1.phf_timer);
      }
</#if>       
    }
  }
  else if (Motor_Device1.adc_user.number_of_channels != 0)
  {  
	if (pAdc == Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index])
	{
		/* Process user measurement */
		if (Motor_Device1.step_change!=0)
		{

		Motor_Device1.step_change = 0;
		}

		else
		{

		Motor_Device1.adc_user.measurement[Motor_Device1.adc_user.channel_index] = AdcMeasurement;
		}
	}
  }
</#if>
<#if MC.SPEED_SENSOR_SELECTION == "HALL_SENSORS">
  uint8_t tmp = Motor_Device1.adc_user.channel_index;
  if (Motor_Device1.adc_user.number_of_channels != 0)
  { 

	if (pAdc == Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index])
	{
		/* Process user measurement */
		Motor_Device1.adc_user.measurement[Motor_Device1.adc_user.channel_index] = AdcMeasurement;
	}

	(Motor_Device1.adc_user.channel_index)++;
	if (Motor_Device1.adc_user.channel_index == Motor_Device1.adc_user.number_of_channels)
	{
		Motor_Device1.adc_user.channel_index = 0;
	}
	MC_Core_SelectAdcChannel(Motor_Device1.adc_user.padc[tmp], Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index], Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index], Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]);
  }

</#if> 
  return MC_FUNC_OK; 
}


/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
    <#if (MC.MOTOR_PROFILER == true)>
    SCC_CheckOC_RL(pSCC);
    </#if>
    <#if ( MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM")>
    TSK_SafetyTask_PWMOFF(M1);
    <#elseif ( MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M1);
    <#elseif ( MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M1);
    </#if>
<#if ( MC.DUALDRIVE == true)>
    /* Second drive */
    <#if ( MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM")>
    TSK_SafetyTask_PWMOFF(M2);
    <#elseif ( MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M2);
    <#elseif ( MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M2);
    </#if>
</#if>

  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

<#if MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_ON_R_BRAKE
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 0 */

  /* USER CODE END TSK_SafetyTask_RBRK 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t BusVoltageFaultsFlag;
<#if  MC.DUALDRIVE == true>  
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
<#else>
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
</#if> 

  /* Brake resistor management */
<#if  MC.BUS_VOLTAGE_READING == true>
  if(M1 == bMotor)
  {
  <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M1);
  <#else>
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1);
  </#if>
  }
<#else> 
 <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
</#if>
<#if  MC.BUS_VOLTAGE_READING2 == true>
  if(M2 == bMotor)
  {
  <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    BusVoltageFaultsFlag = errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M2);
  <#else>
    BusVoltageFaultsFlag = errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2);
  </#if>
  }
<#else>
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
</#if>
  if (MC_OVER_VOLT == BusVoltageFaultsFlag)
  {
    DOUT_SetOutputState(pR_Brake[bMotor], ACTIVE);
  }
  else
  {
    DOUT_SetOutputState(pR_Brake[bMotor], INACTIVE);
  }
  CodeReturn |= NTC_CalcAvTemp(pTemperatureSensor[bMotor]);   /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= (BusVoltageFaultsFlag & MC_UNDER_VOLT);       /* MC_UNDER_VOLT generates fault if FW protection is activated,
                                                                 MC_OVER_VOLT doesn't generate fault */
  (void)STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  switch (STM_GetState(&STM[bMotor]))
  {
    case FAULT_NOW:
    {
<#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(pSCC);
      OTT_Stop(pOTT);
</#if>
<#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >  
      /* reset Encoder state */
      if (pEAC[bMotor] != MC_NULL)
      {       
        EAC_SetRestartState( pEAC[bMotor], false );
      }
</#if>
      PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
      FOC_Clear(bMotor);
      PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3
      /* USER CODE BEGIN TSK_SafetyTask_RBRK 1 */

      /* USER CODE END TSK_SafetyTask_RBRK 1 */
      break;
    }
    
    case FAULT_OVER:
    {
<#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(pSCC);
      OTT_Stop(pOTT);
</#if>
      PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
      /* USER CODE BEGIN TSK_SafetyTask_RBRK 2 */

      /* USER CODE END TSK_SafetyTask_RBRK 2 */
      break;
    }
    
    default:
      break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 3 */

  /* USER CODE END TSK_SafetyTask_RBRK 3 */  
}
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_LSON(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_LSON 0 */

  /* USER CODE END TSK_SafetyTask_LSON 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
<#if  MC.DUALDRIVE == true>  
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
<#else>
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
</#if> 
  bool TurnOnLowSideAction;
  
  TurnOnLowSideAction = PWMC_GetTurnOnLowSidesAction(pwmcHandle[bMotor]);
  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* for fault. It return MC_BREAK_IN or MC_NO_FAULTS
                                                                                (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  /* USER CODE BEGIN TSK_SafetyTask_LSON 1 */

  /* USER CODE END TSK_SafetyTask_LSON 1 */
<#if  MC.BUS_VOLTAGE_READING == true>
  if(M1 == bMotor)
  {
  <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M1);
  <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1);
  </#if>
  }
<#else>
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
</#if>
<#if  MC.BUS_VOLTAGE_READING2 == true>
  if(M2 == bMotor)
  {
  <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M2);
  <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2);
  </#if> 
  }
<#else>
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... --> 
</#if>
  (void)STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  if (((MC_OVER_VOLT == CodeReturn & MC_OVER_VOLT)) && (false == TurnOnLowSideAction))
  {
<#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true > 
    /* reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {       
      EAC_SetRestartState(pEAC[bMotor], false);
    }
</#if>
    /* Start turn on low side action */
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]); /* Required before //PWMC_TurnOnLowSides */
<#if ( MC.HW_OV_CURRENT_PROT_BYPASS == true)>
    DOUT_SetOutputState(pOCPDisabling[bMotor], ACTIVE); /* Disable the OCP */
</#if>
    /* USER CODE BEGIN TSK_SafetyTask_LSON 2 */

    /* USER CODE END TSK_SafetyTask_LSON 2 */
    PWMC_TurnOnLowSides(pwmcHandle[bMotor],0UL); /* Turn on Low side switches */
  }
  else
  {
    switch (STM_GetState(&STM[bMotor])) /* Is state equal to FAULT_NOW or FAULT_OVER */
    {
    
      case IDLE:
      {
        /* After a OV occurs the turn on low side action become active. It is released just after a fault acknowledge -> state == IDLE */
        if (true == TurnOnLowSideAction)
        {
          /* End of TURN_ON_LOW_SIDES action */
<#if ( MC.HW_OV_CURRENT_PROT_BYPASS == true)>
          DOUT_SetOutputState(pOCPDisabling[bMotor], INACTIVE); /* Re-enable the OCP */
</#if>
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);  /* Switch off the PWM */
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 3 */

        /* USER CODE END TSK_SafetyTask_LSON 3 */
        break;
      }
      
      case FAULT_NOW:
      {
        if (TurnOnLowSideAction == false)
        {
<#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >   
          /* reset Encoder state */
          if (pEAC[bMotor] != MC_NULL)
          {       
            EAC_SetRestartState(pEAC[bMotor], false);
          }
</#if>
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
          FOC_Clear(bMotor);
          /* Misra violation Rule 11.3 A cast shall not be performed between a pointer to object 
           * type and a pointer to a different object type. */
          PQD_Clear(pMPM[bMotor]);
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 4 */

        /* USER CODE END TSK_SafetyTask_LSON 4 */
        break;
      }
      
      case FAULT_OVER:
      {
        if (TurnOnLowSideAction == false)
        {
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
        }
	    /* USER CODE BEGIN TSK_SafetyTask_LSON 5 */

        /* USER CODE END TSK_SafetyTask_LSON 5 */
        break;
      }
      
      default:
        break;
    }
  }
  /* USER CODE BEGIN TSK_SafetyTask_LSON 6 */

  /* USER CODE END TSK_SafetyTask_LSON 6 */
}
</#if>

<#if  MC.DUALDRIVE == true>
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Reserves FOC execution on ADC ISR half a PWM period in advance
  *
  *  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  * it allows to reserve half PWM period in advance the FOC execution on
  * ADC ISR
  * @param  pDrive Pointer on the FOC Array
  */
__weak void TSK_DualDriveFIFOUpdate(uint8_t Motor)
{
  FOC_array[FOC_array_tail] = Motor;
  FOC_array_tail++;
  if (FOC_ARRAY_LENGTH == FOC_array_tail)
  {
    FOC_array_tail = 0;
  }
}
</#if>

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */

    /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
<#if MC.PFC_ENABLED == true>

/**
  * @brief  Executes the PFC Task.
  */
void PFC_Scheduler(void)
{
	PFC_Task(&PFC);
}
</#if>
<#if MC.RTOS == "FREERTOS">

/* startMediumFrequencyTask function */
void startMediumFrequencyTask(void const * argument)
{
<#if MC.CUBE_MX_VER == "xxx" >
  /* init code for MotorControl */
  MX_MotorControl_Init();
<#else>
<#assign cubeVersion = MC.CUBE_MX_VER?replace(".","") >
<#if cubeVersion?number < 540 >
  /* init code for MotorControl */
  MX_MotorControl_Init();
</#if>  
</#if> 
  /* USER CODE BEGIN MF task 1 */
  /* Infinite loop */
  for(;;)
  {
    /* delay of 500us */
    vTaskDelay(1);
    MC_RunMotorControlTasks();
  }
  /* USER CODE END MF task 1 */
}

/* startSafetyTask function */
void StartSafetyTask(void const * argument)
{
  /* USER CODE BEGIN SF task 1 */
  /* Infinite loop */
  for(;;)
  {
    /* delay of 500us */
    vTaskDelay(1);
    TSK_SafetyTask();
  }
  /* USER CODE END SF task 1 */ 
}

</#if>   

<#if MC.START_STOP_BTN == true>
__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  MC_Status_t status;
  status = MC_Core_GetStatus(&Motor_Device1);
  if (MC_RUN == status)
  {
    MC_Core_Stop(&Motor_Device1);
    MC_Core_Reset(&Motor_Device1);
  }
  else if (MC_STOP == status)
  {
    MC_Core_Start(&Motor_Device1);
  }

/* USER CODE END START_STOP_BTN */
}
</#if>

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
__weak void mc_lock_pins (void)
{
<#list configs as dt>
<#list dt.peripheralGPIOParams.values() as io>
<#list io.values() as ipIo>
<#list ipIo.entrySet() as e>
<#if (e.getKey().equals("GPIO_Label")) && (e.getValue()?matches("^M[0-9]+_.*$"))>
LL_GPIO_LockPin(${e.getValue()}_GPIO_Port, ${e.getValue()}_Pin);
</#if>
</#list>
</#list>
</#list>
</#list>
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
