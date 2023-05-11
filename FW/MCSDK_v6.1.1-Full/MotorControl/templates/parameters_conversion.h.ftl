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

<#function Fx_Freq_Scaling pwm_freq>
<#list [ 1 , 2 , 4 , 8 , 16 ] as scaling>
       <#if (pwm_freq/scaling) < 65536 >
            <#return scaling >
        </#if>
    </#list>
    <#return 1 >
</#function>

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
<#assign CondFamily_STM32F4 = (FamilyName?? && FamilyName == "STM32F4")>
<#-- Condition for STM32G4 Family -->
<#assign CondFamily_STM32G4 = (FamilyName?? && FamilyName == "STM32G4") >
<#-- Condition for STM32L4 Family -->
<#assign CondFamily_STM32L4 = (FamilyName?? && FamilyName == "STM32L4") >
<#-- Condition for STM32F7 Family -->
<#assign CondFamily_STM32F7 = (FamilyName?? && FamilyName == "STM32F7") >
<#-- Condition for STM32H7 Family -->
<#assign CondFamily_STM32H7 = (FamilyName?? && FamilyName == "STM32H7") >

/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef PARAMETERS_CONVERSION_H
#define PARAMETERS_CONVERSION_H

#include "mc_math.h"
<#if CondFamily_STM32F0>
#include "parameters_conversion_f0xx.h"
<#elseif CondFamily_STM32F3>
#include "parameters_conversion_f30x.h"
<#elseif CondFamily_STM32F4>
#include "parameters_conversion_f4xx.h"
<#elseif CondFamily_STM32L4>
#include "parameters_conversion_l4xx.h"
<#elseif CondFamily_STM32F7>
#include "parameters_conversion_f7xx.h"
<#elseif CondFamily_STM32H7>
#include "parameters_conversion_h7xx.h"
<#elseif CondFamily_STM32G4>
#include "parameters_conversion_g4xx.h"
<#elseif CondFamily_STM32G0>
#include "parameters_conversion_g0xx.h"
</#if>
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"

#define ADC_REFERENCE_VOLTAGE  ${MC.ADC_REFERENCE_VOLTAGE}

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint32_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */ 
#define TF_REGULATION_RATE_SCALED (uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR (65536/PWM_FREQ_SCALING) 

<#if MC.DRIVE_TYPE == "FOC">
/* Current conversion from Ampere unit to 16Bit Digit */
#define CURRENT_CONV_FACTOR		(uint16_t)((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV		(1.0 / ((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE))

#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
#define REP_COUNTER 			(uint16_t) (REGULATION_EXECUTION_RATE -1u)
</#if>
#define SYS_TICK_FREQUENCY          ( uint16_t )2000U
#define UI_TASK_FREQUENCY_HZ        10U


<#if MC.POSITION_CTRL_ENABLING == true >
#define MEDIUM_FREQUENCY_TASK_RATE        (uint16_t)POSITION_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY/POSITION_LOOP_FREQUENCY_HZ)-1u
<#else>
#define MEDIUM_FREQUENCY_TASK_RATE        (uint16_t)SPEED_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u
</#if>

#define UI_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS          (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

<#if MC.M1_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M1_ICL_RELAY_SWITCHING_DELAY_TICKS    (uint16_t)(((M1_ICL_RELAY_SWITCHING_DELAY_MS*MEDIUM_FREQUENCY_TASK_RATE)/1000) - 1u)
#define M1_ICL_CAPS_CHARGING_DELAY_TICKS      (uint16_t)(((M1_ICL_CAPS_CHARGING_DELAY_MS*MEDIUM_FREQUENCY_TASK_RATE)/1000) - 1u)
#define M1_ICL_VOLTAGE_THRESHOLD_U16VOLT      (uint16_t)((M1_ICL_VOLTAGE_THRESHOLD*65536U)/(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
</#if>

<#if MC.DUALDRIVE == true>
#define TF_REGULATION_RATE2               (uint32_t) ((uint32_t)(PWM_FREQUENCY2)/REGULATION_EXECUTION_RATE2)

/* TF_REGULATION_RATE_SCALED2 is TF_REGULATION_RATE2 divided by PWM_FREQ_SCALING2 to allow more dynamic */ 
#define TF_REGULATION_RATE_SCALED2        (uint16_t) ((uint32_t)(PWM_FREQUENCY2)/(REGULATION_EXECUTION_RATE2*PWM_FREQ_SCALING2))

/* DPP_CONV_FACTOR2 is introduce to compute the right DPP with TF_REGULATOR_SCALED2  */
#define DPP_CONV_FACTOR2                  (65536/PWM_FREQ_SCALING2) 

#define REP_COUNTER2                      (uint16_t) ((REGULATION_EXECUTION_RATE2 *2u)-1u)

<#if MC.POSITION_CTRL_ENABLING2 == true >
#define MEDIUM_FREQUENCY_TASK_RATE2       (uint16_t)POSITION_LOOP_FREQUENCY_HZ2
<#else>
#define MEDIUM_FREQUENCY_TASK_RATE2       (uint16_t)SPEED_LOOP_FREQUENCY_HZ2
</#if>

#define MF_TASK_OCCURENCE_TICKS2          (SYS_TICK_FREQUENCY/MEDIUM_FREQUENCY_TASK_RATE2)-1u

<#if MC.M2_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M2_ICL_RELAY_SWITCHING_DELAY_TICKS    (uint16_t)(((M2_ICL_RELAY_SWITCHING_DELAY_MS*MEDIUM_FREQUENCY_TASK_RATE)/1000) - 1u)
#define M2_ICL_CAPS_CHARGING_DELAY_TICKS      (uint16_t)(((M2_ICL_CAPS_CHARGING_DELAY_MS*MEDIUM_FREQUENCY_TASK_RATE)/1000) - 1u)
#define M2_ICL_VOLTAGE_THRESHOLD_U16VOLT      (uint16_t)(uint16_t)((M2_ICL_VOLTAGE_THRESHOLD*65536U)/(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
</#if>


</#if>

<#if MC.STATE_OBSERVER_PLL ||  MC.AUX_STATE_OBSERVER_PLL || MC.STATE_OBSERVER_CORDIC || MC.AUX_STATE_OBSERVER_CORDIC >
/************************* COMMON OBSERVER PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE  (uint16_t)((MAX_APPLICATION_SPEED_RPM * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))
  <#if MC.BUS_VOLTAGE_READING>
/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((ADC_REFERENCE_VOLTAGE/SQRT_3)/VBUS_PARTITIONING_FACTOR) 
  <#else>
#define MAX_VOLTAGE (int16_t)(500/2) /* Virtual sensor conversion factor */
  </#if>

  <#if MC.ICS_SENSORS> 
#define MAX_CURRENT (ADC_REFERENCE_VOLTAGE/(2*AMPLIFICATION_GAIN))
  <#else>
#define MAX_CURRENT (ADC_REFERENCE_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN))
  </#if>
#define OBS_MINIMUM_SPEED_UNIT    (uint16_t) ((OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/U_RPM)  
</#if>

#define MAX_APPLICATION_SPEED_UNIT ((MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/U_RPM)
#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/U_RPM)

<#if MC.STATE_OBSERVER_PLL ||  MC.AUX_STATE_OBSERVER_PLL >
/************************* PLL PARAMETERS **************************/
#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)      
#define HFI_MINIMUM_SPEED    (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)
</#if>
<#if MC.STATE_OBSERVER_CORDIC || MC.AUX_STATE_OBSERVER_CORDIC >  
/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1)*RS)/(LS*TF_REGULATION_RATE))
#define CORD_C2 (int32_t) CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT\
                                                           *TF_REGULATION_RATE))
#define CORD_C4 (int32_t) CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*\
                                                          TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR    (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)      
</#if>

<#if MC.STATE_OBSERVER_PLL2 ||  MC.AUX_STATE_OBSERVER_PLL2 || MC.STATE_OBSERVER_CORDIC2 || MC.AUX_STATE_OBSERVER_CORDIC2 >
/************************* COMMON OBSERVER PARAMETERS Motor 2 **************************/
#define MAX_BEMF_VOLTAGE2  (uint16_t)((MAX_APPLICATION_SPEED_RPM2 * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT2*SQRT_2)/(1000u*SQRT_3))

  <#if MC.BUS_VOLTAGE_READING2>
/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE2 (int16_t)((ADC_REFERENCE_VOLTAGE/SQRT_3)/VBUS_PARTITIONING_FACTOR2) 
  <#else>
#define MAX_VOLTAGE2 (int16_t)(500/2) /* Virtual sensor conversion factor */
  </#if>

  <#if MC.ICS_SENSORS2> 
#define MAX_CURRENT2 (ADC_REFERENCE_VOLTAGE/(2*AMPLIFICATION_GAIN2))
  <#else>
#define MAX_CURRENT2 (ADC_REFERENCE_VOLTAGE/(2*RSHUNT2*AMPLIFICATION_GAIN2))
  </#if>
#define OBS_MINIMUM_SPEED_UNIT2        (uint16_t) ((OBS_MINIMUM_SPEED_RPM2*SPEED_UNIT)/_RPM)  
</#if>

#define MAX_APPLICATION_SPEED_UNIT2 ((MAX_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)
#define MIN_APPLICATION_SPEED_UNIT2 ((MIN_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)

<#if MC.STATE_OBSERVER_PLL2 ||  MC.AUX_STATE_OBSERVER_PLL2 >
/************************* PLL PARAMETERS **************************/
#define C12 (int32_t)((((int16_t)F12)*RS2)/(LS2*TF_REGULATION_RATE2))
#define C22 (int32_t) GAIN12
#define C32 (int32_t)((((int16_t)F12)*MAX_BEMF_VOLTAGE2)/(LS2*MAX_CURRENT2*TF_REGULATION_RATE2))
#define C42 (int32_t) GAIN22
#define C52 (int32_t)((((int16_t)F12)*MAX_VOLTAGE2)/(LS2*MAX_CURRENT2*TF_REGULATION_RATE2))

#define PERCENTAGE_FACTOR2    (uint16_t)(VARIANCE_THRESHOLD2*128u)      

#define HFI_MINIMUM_SPEED2        (uint16_t) (HFI_MINIMUM_SPEED_RPM2/6u)
</#if>

<#if MC.STATE_OBSERVER_CORDIC2 || MC.AUX_STATE_OBSERVER_CORDIC2 >  
#define CORD_C12 (int32_t)((((int16_t)CORD_F12)*RS2)/(LS2*TF_REGULATION_RATE2))
#define CORD_C22 (int32_t) CORD_GAIN12
#define CORD_C32 (int32_t)((((int16_t)CORD_F12)*MAX_BEMF_VOLTAGE2)/(LS2*MAX_CURRENT2\
                                                           *TF_REGULATION_RATE2))
#define CORD_C42 (int32_t) CORD_GAIN22
#define CORD_C52 (int32_t)((((int16_t)CORD_F12)*MAX_VOLTAGE2)/(LS2*MAX_CURRENT2*\
                                                          TF_REGULATION_RATE2))
#define CORD_PERCENTAGE_FACTOR2    (uint16_t)(CORD_VARIANCE_THRESHOLD2*128u)      
</#if>

<#if MC.DRIVE_TYPE == "SIX_STEP">
  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)  

/************************* BEMF OBSERVER PARAMETERS **************************/
#define OBS_MINIMUM_SPEED_UNIT          (uint16_t) ((OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/U_RPM)

#define BEMF_THRESHOLD_DOWN             ((uint16_t) (4096*BEMF_THRESHOLD_DOWN_V/(BEMF_ON_SENSING_DIVIDER*ADC_REFERENCE_VOLTAGE)))<< 4    /*!< BEMF ADC digit threshold for zero crossing detection when BEMF is expected to decrease */
#define BEMF_THRESHOLD_UP               ((uint16_t) (4096*BEMF_THRESHOLD_UP_V/(BEMF_ON_SENSING_DIVIDER*ADC_REFERENCE_VOLTAGE)))<< 4  /*!< BEMF ADC digit threshold for zero crossing detection when BEMF is expected to increase */
#define BEMF_ADC_TRIG_TIME              (uint16_t) (PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_DPP / 1024)

  <#if  MC.DRIVE_MODE == "VM">    
#define BEMF_THRESHOLD_DOWN_ON          ((uint16_t) (4096*BEMF_THRESHOLD_DOWN_ON_V/(BEMF_ON_SENSING_DIVIDER*ADC_REFERENCE_VOLTAGE)))<< 4   /*!< BEMF ADC digit threshold during PWM ON time for zero crossing detection when BEMF is expected to decrease */
#define BEMF_THRESHOLD_UP_ON            ((uint16_t) (4096*BEMF_THRESHOLD_UP_ON_V/(BEMF_ON_SENSING_DIVIDER*ADC_REFERENCE_VOLTAGE)))<< 4   /*!< BEMF ADC digit threshold during PWM ON time for zero crossing detection when BEMF is expected to increase */
#define BEMF_ADC_TRIG_TIME_ON           (uint16_t) (PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_ON_DPP / 1024)

#define BEMF_PWM_ON_ENABLE_THRES        (uint16_t) (PWM_PERIOD_CYCLES * BEMF_PWM_ON_ENABLE_THRES_DPP / 1024)
#define BEMF_PWM_ON_DISABLE_THRES       (uint16_t) (PWM_PERIOD_CYCLES * (BEMF_PWM_ON_ENABLE_THRES_DPP - BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP) / 1024)

  </#if>  
    <#if  MC.DRIVE_MODE == "VM"> <#-- VOLTAGE MODE -->
#define PHASE1_VOLTAGE_DPP              ((PWM_PERIOD_CYCLES * PHASE1_VOLTAGE_RMS * PHASE1_VOLTAGE_RMS) / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE2_VOLTAGE_DPP              ((PWM_PERIOD_CYCLES * PHASE2_VOLTAGE_RMS * PHASE2_VOLTAGE_RMS) / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE3_VOLTAGE_DPP              ((PWM_PERIOD_CYCLES * PHASE3_VOLTAGE_RMS * PHASE3_VOLTAGE_RMS) / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE4_VOLTAGE_DPP              ((PWM_PERIOD_CYCLES * PHASE4_VOLTAGE_RMS * PHASE4_VOLTAGE_RMS) / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE5_VOLTAGE_DPP              ((PWM_PERIOD_CYCLES * PHASE5_VOLTAGE_RMS * PHASE5_VOLTAGE_RMS) / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
    <#else> <#-- CURRENT MODE -->
      <#if  MC.CURRENT_LIMITER_OFFSET>
#define PHASE1_FINAL_CURRENT_DPP        ((uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE1_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE2_FINAL_CURRENT_DPP        ((uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE2_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE3_FINAL_CURRENT_DPP        ((uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE3_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE4_FINAL_CURRENT_DPP        ((uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE4_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE5_FINAL_CURRENT_DPP        ((uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE5_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
      <#else>
#define PHASE1_FINAL_CURRENT_DPP        ((uint16_t) ((1024*PHASE1_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE2_FINAL_CURRENT_DPP        ((uint16_t) ((1024*PHASE2_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE3_FINAL_CURRENT_DPP        ((uint16_t) ((1024*PHASE3_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE4_FINAL_CURRENT_DPP        ((uint16_t) ((1024*PHASE4_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
#define PHASE5_FINAL_CURRENT_DPP        ((uint16_t) ((1024*PHASE5_FINAL_CURRENT*RSHUNT*AMPLIFICATION_GAIN)/(1000*ADC_REFERENCE_VOLTAGE)))
	  </#if>
#define PHASE1_FINAL_CURRENT_REF        (PWM_PERIOD_CYCLES_REF * PHASE1_FINAL_CURRENT_DPP / 1024)
#define PHASE2_FINAL_CURRENT_REF        (PWM_PERIOD_CYCLES_REF * PHASE2_FINAL_CURRENT_DPP / 1024)
#define PHASE3_FINAL_CURRENT_REF        (PWM_PERIOD_CYCLES_REF * PHASE3_FINAL_CURRENT_DPP / 1024)
#define PHASE4_FINAL_CURRENT_REF        (PWM_PERIOD_CYCLES_REF * PHASE4_FINAL_CURRENT_DPP / 1024)
#define PHASE5_FINAL_CURRENT_REF        (PWM_PERIOD_CYCLES_REF * PHASE5_FINAL_CURRENT_DPP / 1024)
	</#if>
	
#define DEMAG_MINIMUM_SPEED             SPEED_THRESHOLD_DEMAG * SPEED_UNIT / U_RPM
#define DEMAG_REVUP_CONV_FACTOR         (uint16_t) ((DEMAG_REVUP_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT) / (600 * POLE_PAIR_NUM ))
#define DEMAG_RUN_CONV_FACTOR           (uint16_t) ((DEMAG_RUN_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT) / (600 * POLE_PAIR_NUM ))
  </#if>
</#if>
/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
<#if   MC.ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V)*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
<#else>
#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(M1_OVP_THRESHOLD_HIGH*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d   (uint16_t)(M1_OVP_THRESHOLD_LOW*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))                                  
</#if>                                 
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)
<#if MC.DUALDRIVE == true>
/**************************   VOLTAGE CONVERSIONS  Motor 2 *************************/
<#if   MC.ON_OVER_VOLTAGE2 != "TURN_ON_R_BRAKE">  
#define OVERVOLTAGE_THRESHOLD_d2   (uint16_t)(OV_VOLTAGE_THRESHOLD_V2*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
#define OVERVOLTAGE_THRESHOLD_LOW_d2   (uint16_t)(OV_VOLTAGE_THRESHOLD_V2)*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
<#else>
#define OVERVOLTAGE_THRESHOLD_d2   (uint16_t)(M2_OVP_THRESHOLD_HIGH*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
#define OVERVOLTAGE_THRESHOLD_LOW_d2   (uint16_t)(M2_OVP_THRESHOLD_LOW*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR2))
</#if>                                 
#define UNDERVOLTAGE_THRESHOLD_d2  (uint16_t)((UD_VOLTAGE_THRESHOLD_V2*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR2)))
#define INT_SUPPLY_VOLTAGE2          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD2        (OV_TEMPERATURE_THRESHOLD_C2- T0_C2)
#define DELTA_V_THRESHOLD2           (dV_dT2 * DELTA_TEMP_THRESHOLD2)
#define OV_TEMPERATURE_THRESHOLD_d2  ((V0_V2 + DELTA_V_THRESHOLD2)*INT_SUPPLY_VOLTAGE2)

#define DELTA_TEMP_HYSTERESIS2        (OV_TEMPERATURE_HYSTERESIS_C2)
#define DELTA_V_HYSTERESIS2           (dV_dT2 * DELTA_TEMP_HYSTERESIS2)
#define OV_TEMPERATURE_HYSTERESIS_d2  (DELTA_V_HYSTERESIS2*INT_SUPPLY_VOLTAGE2)
</#if>


<#if ( MC.ENCODER2 == true ||  MC.AUX_ENCODER2 == true)>
/*************** Encoder Alignemnt ************************/    
#define ALIGNMENT_ANGLE_S162      (int16_t)  (ALIGNMENT_ANGLE_DEG2*65536u/360u)
</#if>
<#if ( MC.ENCODER == true ||  MC.AUX_ENCODER == true)>
#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)
</#if>

<#if MC.MOTOR_PROFILER == true>
#undef MAX_APPLICATION_SPEED_RPM
#define MAX_APPLICATION_SPEED_RPM 50000
#undef F1
#define F1 0
#undef CORD_F1
#define CORD_F1 0

#define SPEED_REGULATOR_BANDWIDTH 0 // Dummy value

#define LDLQ_RATIO              1.000 /*!< Ld vs Lq ratio.*/

<#-- X-NUCLEO-IHM07M1 -->
<#if MC.POWERBOARD_NAME == "X-NUCLEO-IHM07M1"> 
#define CALIBRATION_FACTOR            1.50
#define BUS_VOLTAGE_CONVERSION_FACTOR 63.2
#define CURRENT_REGULATOR_BANDWIDTH     6000
#define MP_KP 10.00f
#define MP_KI 0.1f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- X-NUCLEO-IHM16M1 -->
<#if MC.POWERBOARD_NAME == "X-NUCLEO-IHM16M1" || MC.POWERBOARD_NAME == "53"> 
#define CALIBRATION_FACTOR            1.50
#define BUS_VOLTAGE_CONVERSION_FACTOR 63.2
#define CURRENT_REGULATOR_BANDWIDTH     6000
#define MP_KP 10.00f
#define MP_KI 0.1f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- X-NUCLEO-IHM08M1 -->
<#if MC.POWERBOARD_NAME == "X-NUCLEO-IHM08M1"> 
#define CALIBRATION_FACTOR            0.02
#define BUS_VOLTAGE_CONVERSION_FACTOR 63.2
#define CURRENT_REGULATOR_BANDWIDTH      6000
#define MP_KP 1.00f
#define MP_KI 0.01f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- B-G431B-ESC1 -->
<#if MC.POWERBOARD_NAME == "B-G431B-ESC1"> 
#define CALIBRATION_FACTOR           0.181
#define BUS_VOLTAGE_CONVERSION_FACTOR 34.29
#define CURRENT_REGULATOR_BANDWIDTH   6000
#define MP_KP 1.00f
#define MP_KI 0.01f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- STEVAL-IHM023V3 LV -->
<#if MC.POWERBOARD_NAME == "STEVAL-IHM023V3"> 
#define CALIBRATION_FACTOR            	 0.67            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 447.903
#define CURRENT_REGULATOR_BANDWIDTH      6000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- STEVAL-IHM028V2 3Sh -->
<#if MC.POWERBOARD_NAME == "STEVAL-IHM028V2"> 
#define CALIBRATION_FACTOR            	 0.3            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- STEVAL-IPM05F 3 Sh -->
<#if MC.POWERBOARD_NAME == "STEVAL-IPM05F"> 
#define CALIBRATION_FACTOR            	 0.5            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      3000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- STEVAL-IPM10B 3 Sh -->
<#if MC.POWERBOARD_NAME == "STEVAL-IPM10B"> 
#define CALIBRATION_FACTOR            	 0.5            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- STEVAL-IPM15B 3 Sh -->
<#if MC.POWERBOARD_NAME == "STEVAL-IPM15B"> 
#define CALIBRATION_FACTOR            	 0.5           
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 1.66f
#define MP_KI 0.01f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- STEVAL-IHM025V1 3Sh -->
<#if MC.POWERBOARD_NAME == "STEVAL-IHM025V1"> 
#define CALIBRATION_FACTOR            	 0.90            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.10 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- STEVAL-IHM023V3 3Sh HV -->
<#if MC.POWERBOARD_NAME == "STEVAL-IHM023V3"> 
#define CALIBRATION_FACTOR            	 0.67            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 447.903
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.95 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.5
</#if>
<#-- STEVAL-IPMN3GQ 3Sh  -->
<#if MC.POWERBOARD_NAME == "STEVAL-IPMN3GQ" > 
#define CALIBRATION_FACTOR            	 1.03            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 416.9
#define CURRENT_REGULATOR_BANDWIDTH      3000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      2.95 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>
<#-- STEVAL-IPMNM1N 3Sh  -->
<#if MC.POWERBOARD_NAME == "STEVAL-IPMNM1N" > 
#define CALIBRATION_FACTOR            	 1.03           
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 416.9
#define CURRENT_REGULATOR_BANDWIDTH      3000
#define MP_KP 5.00f
#define MP_KI 0.05f
#define DC_CURRENT_RS_MEAS      1.00 /*!< Maxium level of DC current used */
#define I_THRESHOLD                   0.05
</#if>


</#if> <#-- MC.MOTOR_PROFILER -->

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))& ( uint16_t )0xFFFE)
<#if MC.DRIVE_MODE == "CM">
#define PWM_PERIOD_CYCLES_REF (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY_REF)))& ( uint16_t )0xFFFE)
</#if>
<#if MC.DUALDRIVE == true>
#define PWM_PERIOD_CYCLES2 (uint16_t)((ADV_TIM_CLK_MHz2*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY2)))& (uint16_t )0xFFFE)
</#if>

<#if MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#define DEADTIME_NS  SW_DEADTIME_NS
<#else>
#define DEADTIME_NS  HW_DEAD_TIME_NS
</#if>
<#if MC.DUALDRIVE == true>
<#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "LS_PWM_TIMER">
#define DEADTIME_NS2  SW_DEADTIME_NS2
<#else>
#define DEADTIME_NS2  HW_DEAD_TIME_NS2
</#if>
</#if>


#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
<#if MC.DRIVE_TYPE == "FOC">
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/500uL)
</#if>

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif
<#if MC.DRIVE_TYPE == "FOC">
<#if MC.DUALDRIVE == true>
#define DEAD_TIME_ADV_TIM_CLK_MHz2 (ADV_TIM_CLK_MHz2 * TIM_CLOCK_DIVIDER2)
#define DEAD_TIME_COUNTS2_1  (DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/1000uL)

#if (DEAD_TIME_COUNTS2_1 <= 255)
#define DEAD_TIME_COUNTS2 (uint16_t) DEAD_TIME_COUNTS2_1
#elif (DEAD_TIME_COUNTS2_1 <= 508)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS2_1 <= 1008)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS2_1 <= 2015)
#define DEAD_TIME_COUNTS2 (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS2 510
#endif
</#if>

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)
<#if MC.DUALDRIVE == true>
#define DTCOMPCNT2 (uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2) / 2000)
#define TON_NS2  500
#define TOFF_NS2 500
#define TON2  (uint16_t)((TON_NS2 * ADV_TIM_CLK_MHz2)  / 2000)
#define TOFF2 (uint16_t)((TOFF_NS2 * ADV_TIM_CLK_MHz2) / 2000)
</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
#define SAMPLING_TIME ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz) /* In ADV_TIMER CLK cycles*/
<#if MC.DRIVE_TYPE == "FOC" >
#define TRISE ((TRISE_NS * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD ((uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000))
#define TNOISE ((uint16_t)((TNOISE_NS*ADV_TIM_CLK_MHz)/1000))
<#if MC.SINGLE_SHUNT>
#define TMAX_TNTR ((uint16_t)((MAX_TNTR_NS * ADV_TIM_CLK_MHz)/1000uL))
#define TAFTER ((uint16_t)(TDEAD + TMAX_TNTR))
#define TBEFORE ((uint16_t)(((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz)  + 1U)
#define TMIN ((uint16_t)( TAFTER + TBEFORE ))
#define HTMIN ((uint16_t)(TMIN >> 1))
#define CHTMIN ((uint16_t)(TMIN/(REGULATION_EXECUTION_RATE*2)))
#if (TRISE > SAMPLING_TIME)
#define MAX_TRTS (2 * TRISE)
#else
#define MAX_TRTS (2 * SAMPLING_TIME)
#endif
<#else>
#define HTMIN 1 /* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define TW_BEFORE ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
#define TW_BEFORE_R3_1 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES*2 + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000UL))
#define MAX_TWAIT ((uint16_t)((TW_AFTER - SAMPLING_TIME)/2))
</#if>

<#if MC.DUALDRIVE == true>
/**********************/
/* MOTOR 2 ADC Timing */
/**********************/
#define SAMPLING_TIME2 ((ADC_SAMPLING_CYCLES2 * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2) /* In ADV_TIMER2 CLK cycles*/ 
#define TRISE2 (((TRISE_NS2) * ADV_TIM_CLK_MHz2)/1000uL)
#define TDEAD2 ((uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2)/1000uL))
#define TNOISE2 ((uint16_t)((TNOISE_NS2*ADV_TIM_CLK_MHz2)/1000ul))
<#if MC.SINGLE_SHUNT2 >
#define TMAX_TNTR2 ((uint16_t)((MAX_TNTR_NS2 * ADV_TIM_CLK_MHz)/1000uL))
#define TAFTER2 ((uint16_t)( TDEAD2 + TMAX_TNTR2 ))
#define TBEFORE2 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2 ) * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2  + 1u)
#define TMIN2  (TAFTER2 + TBEFORE2)
#define HTMIN2 (uint16_t)(TMIN2 >> 1)
#define CHTMIN2 (uint16_t)(TMIN2/(REGULATION_EXECUTION_RATE2*2))
#if (TRISE2 > SAMPLING_TIME2)
#define MAX_TRTS2 (2 * TRISE2)
#else
#define MAX_TRTS2 (2 * SAMPLING_TIME2)
#endif
<#else>
#define HTMIN2 0 /* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define TW_BEFORE2 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2) * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2  + 1u)
#define TW_BEFORE_R3_1_2 ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2*2 + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2  + 1u)
#define TW_AFTER2 ((uint16_t)(((DEADTIME_NS2+MAX_TNTR_NS2)*ADV_TIM_CLK_MHz2)/1000UL))
#define MAX_TWAIT2 ((uint16_t)((TW_AFTER2 - SAMPLING_TIME2)/2))
</#if>
</#if> <#-- MC.DUALDRIVE -->
</#if>
/* USER CODE BEGIN temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u

/* USER CODE END temperature */
<#if MC.DRIVE_TYPE = "FOC">
<#if  MC.FLUX_WEAKENING_ENABLING || MC.FEED_FORWARD_CURRENT_REG_ENABLING >
/* Flux Weakening - Feed forward */
#define M1_VQD_SW_FILTER_BW_FACTOR       128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M1_VQD_SW_FILTER_BW_FACTOR)
</#if>

<#if MC.DUALDRIVE == true>
/* USER CODE BEGIN temperature M2*/

#define M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M2_TEMP_SW_FILTER_BW_FACTOR     250u

/* USER CODE END temperature M2*/
<#if  MC.FLUX_WEAKENING_ENABLING2 || MC.FEED_FORWARD_CURRENT_REG_ENABLING2 >
/* Flux Weakening - Feed forward Motor 2*/
#define M2_VQD_SW_FILTER_BW_FACTOR      128u
#define M2_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M2_VQD_SW_FILTER_BW_FACTOR)
</#if>
</#if>

<#if MC.ICS_SENSORS>
#define PQD_CONVERSION_FACTOR (float)((( 1.732 * ADC_REFERENCE_VOLTAGE ) /\
             ( AMPLIFICATION_GAIN ))/65536.0f)
<#else>
#define PQD_CONVERSION_FACTOR (float)((( 1.732 * ADC_REFERENCE_VOLTAGE ) /\
             ( RSHUNT * AMPLIFICATION_GAIN ))/65536.0f)
</#if>
<#if MC.DUALDRIVE == true>
<#if MC.ICS_SENSORS2>
#define PQD_CONVERSION_FACTOR2 (float)((( 1.732 * ADC_REFERENCE_VOLTAGE ) /\
             ( AMPLIFICATION_GAIN2 ))/65536.0f)
<#else>
#define PQD_CONVERSION_FACTOR2 (float)((( 1.732 * ADC_REFERENCE_VOLTAGE ) /\
             ( RSHUNT2 * AMPLIFICATION_GAIN2 ))/65536.0f)
</#if>
</#if>
<#if MC.DUALDRIVE == true>
<#-- If this check really needed? -->
<#if MC.FREQ_RELATION ==  'HIGHEST_FREQ'> 
<#-- First instance has higher frequency --> 
#define PWM_FREQUENCY_CHECK_RATIO   (PWM_FREQUENCY*10000u/PWM_FREQUENCY2)
<#else> 
<#-- Second instance has higher frequency -->
#define PWM_FREQUENCY_CHECK_RATIO   (PWM_FREQUENCY2*10000u/PWM_FREQUENCY)
</#if>

#define MAGN_FREQ_RATIO   (${MC.FREQ_RATIO2}*10000u)

#if (PWM_FREQUENCY_CHECK_RATIO != MAGN_FREQ_RATIO)
#error "The two motor PWM frequencies should be integer multiple"  
#endif

</#if>
</#if>   



/****** Prepares the UI configurations according the MCconfxx settings ********/

<#if MC.DAC_FUNCTIONALITY == true>
#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC
<#else>
#define DAC_ENABLE
#define DAC_OP_ENABLE
</#if>

<#if MC.DRIVE_TYPE = "FOC">
/* Motor 1 settings */
<#if MC.FLUX_WEAKENING_ENABLING>
#define FW_ENABLE | UI_CFGOPT_FW
<#else>
#define FW_ENABLE
</#if>

<#if MC.DIFFERENTIAL_TERM_ENABLED>
#define DIFFTERM_ENABLE | UI_CFGOPT_SPEED_KD | UI_CFGOPT_Iq_KD | UI_CFGOPT_Id_KD
<#else>
#define DIFFTERM_ENABLE
</#if>
<#if MC.DUALDRIVE == true>
/* Motor 2 settings */
<#if MC.FLUX_WEAKENING_ENABLING2>
#define FW_ENABLE2 | UI_CFGOPT_FW
<#else>
#define FW_ENABLE2
</#if>

<#if MC.DIFFERENTIAL_TERM_ENABLED2>
#define DIFFTERM_ENABLE2 | UI_CFGOPT_SPEED_KD | UI_CFGOPT_Iq_KD | UI_CFGOPT_Id_KD
<#else>
#define DIFFTERM_ENABLE2
</#if>
</#if>

/* Sensors setting */
<#-- All this could be simpler... -->
<#if MC.STATE_OBSERVER_PLL>
#define MAIN_SCFG UI_SCODE_STO_PLL
</#if>

<#if MC.STATE_OBSERVER_CORDIC>
#define MAIN_SCFG UI_SCODE_STO_CR
</#if>

<#if MC.AUX_STATE_OBSERVER_PLL>
#define AUX_SCFG UI_SCODE_STO_PLL
</#if>

<#if MC.AUX_STATE_OBSERVER_CORDIC>
#define AUX_SCFG UI_SCODE_STO_CR
</#if>

<#if MC.ENCODER>
#define MAIN_SCFG UI_SCODE_ENC
</#if>

<#if MC.AUX_ENCODER>
#define AUX_SCFG UI_SCODE_ENC
</#if>

<#if MC.HALL_SENSORS>
#define MAIN_SCFG UI_SCODE_HALL
</#if>

<#if MC.AUX_HALL_SENSORS>
#define AUX_SCFG UI_SCODE_HALL
</#if>

<#if MC.AUX_STATE_OBSERVER_CORDIC == false && MC.AUX_STATE_OBSERVER_PLL==false && MC.AUX_ENCODER == false && MC.AUX_HALL_SENSORS==false>
#define AUX_SCFG 0x0
</#if>
<#if MC.DUALDRIVE == true>
<#if MC.STATE_OBSERVER_PLL2>
#define MAIN_SCFG2 UI_SCODE_STO_PLL
</#if>

<#if MC.STATE_OBSERVER_CORDIC2>
#define MAIN_SCFG2 UI_SCODE_STO_CR
</#if>

<#if MC.AUX_STATE_OBSERVER_PLL2>
#define AUX_SCFG2 UI_SCODE_STO_PLL
</#if>

<#if MC.AUX_STATE_OBSERVER_CORDIC2>
#define AUX_SCFG2 UI_SCODE_STO_CR
</#if>

<#if MC.ENCODER2>
#define MAIN_SCFG2 UI_SCODE_ENC
</#if>

<#if MC.AUX_ENCODER2>
#define AUX_SCFG2 UI_SCODE_ENC
</#if>

<#if MC.HALL_SENSORS2>
#define MAIN_SCFG2 UI_SCODE_HALL
</#if>

<#if MC.AUX_HALL_SENSORS2>
#define AUX_SCFG2 UI_SCODE_HALL
</#if>

<#if MC.AUX_STATE_OBSERVER_CORDIC2 == false && MC.AUX_STATE_OBSERVER_PLL2==false && MC.AUX_ENCODER2 == false && MC.AUX_HALL_SENSORS2==false>
#define AUX_SCFG2 0x0
</#if>
</#if>

<#-- Seems Useless -->
<#if MC.PLLTUNING?? && MC.PLLTUNING==true>
#define PLLTUNING_ENABLE | UI_CFGOPT_PLLTUNING
<#else>
#define PLLTUNING_ENABLE
</#if>
<#if MC.DUALDRIVE == true>
<#if MC.PLLTUNING?? && MC.PLLTUNING==true>
#define PLLTUNING_ENABLE2 | UI_CFGOPT_PLLTUNING
<#else>
#define PLLTUNING_ENABLE2
</#if>
</#if>

<#if MC.PFC_ENABLED>
#define UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PFC
<#else>
#define UI_CFGOPT_PFC_ENABLE
</#if>

</#if>
/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

<#if MC.SINGLEDRIVE>
#define UI_CONFIG_M2
<#else>
/* Specific options of UI, Motor 2 */
#define UI_CONFIG_M2 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE2 \
  | (MAIN_SCFG2 << MAIN_SCFG_POS) | (AUX_SCFG2 << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE2 )

</#if>

<#-- Can't this be removed? Seems only used in START_STOP_POLARITY which comes from WB... -->
#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

<#-- Only left because of the FreeRTOS project that we will need to adapt sooner or later... 
#define USE_EVAL (defined(USE_STM32446E_EVAL) || defined(USE_STM324xG_EVAL) || defined(USE_STM32F4XX_DUAL))
 -->

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

<#-- This table is an implementation of the Bits 7:4 IC1F register definition -->
<#function Fx_ic_filter icx>
    <#local coefficients =
        [ {"divider":  1, "N": 1} <#--   1 -->
        , {"divider":  1, "N": 2} <#--   2 -->
        , {"divider":  1, "N": 4} <#--   4 -->
        , {"divider":  1, "N": 8} <#--   8 -->

        , {"divider":  2, "N": 6} <#--  12 -->
        , {"divider":  2, "N": 8} <#--  16 -->

        , {"divider":  4, "N": 6} <#--  24 -->
        , {"divider":  4, "N": 8} <#--  32 -->

        , {"divider":  8, "N": 6} <#--  48 -->
        , {"divider":  8, "N": 8} <#--  64 -->

        , {"divider": 16, "N": 5} <#--  80 -->
        , {"divider": 16, "N": 6} <#--  96 -->
        , {"divider": 16, "N": 8} <#-- 128 -->

        , {"divider": 32, "N": 5} <#-- 160 -->
        , {"divider": 32, "N": 6} <#-- 192 -->
        , {"divider": 32, "N": 8} <#-- 256 -->
        ] >

    <#list coefficients as coeff >
        <#if icx <= (coeff.divider * coeff.N) >
            <#return  coeff_index >
        </#if>
    </#list>
    <#return 15 >
</#function>

<#macro define_IC_FILTER  motor sensor icx_filter>
#define M${ motor }_${ sensor }_IC_FILTER  ${ Fx_ic_filter(icx_filter)  }
</#macro>

<#function TimerHandler timer>
  <#return "${timer}_IRQHandler">
</#function>

<#if MC.HALL_SENSORS ==true || MC.AUX_HALL_SENSORS == true>
/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD 65535
<@define_IC_FILTER motor=1 sensor='HALL' icx_filter=MC.HALL_ICx_FILTER?number />
#define SPD_TIM_M1_IRQHandler ${TimerHandler(_last_word(MC.HALL_TIMER_SELECTION))}
</#if>

<#if MC.DRIVE_TYPE == "FOC">
<#if MC.HALL_SENSORS2==true || MC.AUX_HALL_SENSORS2 == true>
/**********  AUXILIARY HALL TIMER MOTOR 2 *************/
#define M2_HALL_TIM_PERIOD 65535
<@define_IC_FILTER motor=2 sensor='HALL' icx_filter=MC.HALL_ICx_FILTER2?number />
#define SPD_TIM_M2_IRQHandler ${TimerHandler(_last_word(MC.HALL_TIMER_SELECTION2))}
</#if>

<#if MC.ENCODER == true || MC.AUX_ENCODER == true >
/**********  AUXILIARY ENCODER TIMER MOTOR 1 *************/
#define M1_PULSE_NBR ( (4 * (M1_ENCODER_PPR)) - 1 )
<@define_IC_FILTER motor=1 sensor='ENC' icx_filter=MC.ENC_ICx_FILTER?number />
#define SPD_TIM_M1_IRQHandler ${TimerHandler(_last_word(MC.ENC_TIMER_SELECTION))}
</#if>
<#if MC.ENCODER2 == true || MC.AUX_ENCODER2 == true>
/**********  AUXILIARY ENCODER TIMER MOTOR 2 *************/
#define M2_PULSE_NBR ( (4 * (M2_ENCODER_PPR)) - 1 )
<@define_IC_FILTER motor=2 sensor='ENC' icx_filter=MC.ENC_ICx_FILTER2?number />
#define SPD_TIM_M2_IRQHandler ${TimerHandler(_last_word(MC.ENC_TIMER_SELECTION2))}
</#if>
<#if MC.PFC_ENABLED == true>
#define PFC_ETRFILTER_IC  ${ Fx_ic_filter(MC.ETRFILTER?number) }
#define PFC_SYNCFILTER_IC ${ Fx_ic_filter(MC.SYNCFILTER?number) }
</#if>
</#if>

<#function MMIfunction Modulation>

<#local MMI =
        [ {"MAX_MODULATION":'MAX_MODULATION_81_PER_CENT', 
           "MAX_MODULE": 26541 
          } 
        , {"MAX_MODULATION":'MAX_MODULATION_83_PER_CENT', 
           "MAX_MODULE": 27196 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_85_PER_CENT', 
           "MAX_MODULE": 27851 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_87_PER_CENT', 
           "MAX_MODULE": 28507 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_89_PER_CENT', 
           "MAX_MODULE": 29162 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_91_PER_CENT', 
           "MAX_MODULE": 29817 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_92_PER_CENT',
           "MAX_MODULE": 30145 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_93_PER_CENT', 
           "MAX_MODULE": 30473 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_94_PER_CENT', 
           "MAX_MODULE": 30800 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_95_PER_CENT', 
           "MAX_MODULE": 31128 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_96_PER_CENT', 
           "MAX_MODULE": 31456 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_97_PER_CENT', 
           "MAX_MODULE": 31783 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_98_PER_CENT', 
           "MAX_MODULE": 32111 
           }
        , {"MAX_MODULATION":'MAX_MODULATION_99_PER_CENT', 
           "MAX_MODULE": 32439 
           }  
        , {"MAX_MODULATION":'MAX_MODULATION_100_PER_CENT', 
           "MAX_MODULE": 32767 
           }          
        ] > 
          
    <#list MMI as MMIitem >
      <#if Modulation == MMIitem.MAX_MODULATION >
         <#return  MMIitem >
      </#if>
    </#list>
    <#local Defaultitem = {"MAX_MODULATION":'ERROR_MAX_MODULATION_NOT_FOUND'
                      ,"MAX_MODULE":'ERROR_MAX_MODULE_NOT_FOUND'} >
    <#return Defaultitem>
</#function>

<#if MC.DRIVE_TYPE == "FOC">
#define LPF_FILT_CONST ((int16_t)(32767 * 0.5))
<#if MC.OVERMODULATION == false>
<#assign MMIvar = MMIfunction(MC.MAX_MODULATION_INDEX) >
/* MMI Table Motor 1 ${MC.MAX_MODULATION_INDEX} */
#define MAX_MODULE ${MMIvar.MAX_MODULE}
<#else>
/* MMI Table Motor 1 100% */
#define MAX_MODULE 32767
</#if>
</#if>

<#if MC.DUALDRIVE == true>
<#if MC.OVERMODULATION2 == false>
<#assign MMIvar2 = MMIfunction(_remove_last_char(MC.MAX_MODULATION_INDEX2)) >
/* MMI Table Motor 2 ${_remove_last_char(MC.MAX_MODULATION_INDEX2)} */
#define MAX_MODULE2      ${MMIvar2.MAX_MODULE}
<#else>
/* MMI Table Motor 2 100% */
#define MAX_MODULE2 32767
}
</#if>
</#if> 
 
<#if CondFamily_STM32F4 || CondFamily_STM32F7 >
  <#assign LL_ADC_CYCLE_SUFFIX = 'CYCLES'>
#define  SAMPLING_CYCLE_CORRECTION 0 /* ${McuName} ADC sampling time is an integer number */
<#else>
  <#assign LL_ADC_CYCLE_SUFFIX = 'CYCLES_5'>
  <#-- Addition of the Half cycle of ADC sampling time-->
#define SAMPLING_CYCLE_CORRECTION 0.5 /* Add half cycle required by ${McuName} ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5 LL_ADC_SAMPLINGTIME_1CYCLE_5
</#if>

#define LL_ADC_SAMPLING_CYCLE(CYCLE) LL_ADC_SAMPLINGTIME_ ## CYCLE ## ${LL_ADC_CYCLE_SUFFIX} //cstat !MISRAC2012-Rule-20.10 !DEFINE-hash-multiple
  
#endif /*PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
