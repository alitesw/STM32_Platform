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
<#-- Condition for STM32G0 Family -->
<#assign CondFamily_STM32G0 = (FamilyName?? && FamilyName=="STM32G0") >
/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
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
  
#ifndef MC_CONFIG_H
#define MC_CONFIG_H

<#-- Specific to FOC algorithm usage -->
#include "pid_regulator.h"
<#if MC.DRIVE_TYPE == "FOC">
#include "speed_torq_ctrl.h"
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
#include "speed_ctrl.h"
</#if>
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
<#if MC.DRIVE_TYPE == "FOC">
#include "revup_ctrl.h"
#include "pwm_curr_fdbk.h"
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
#include "revup_ctrl_sixstep.h"
#include "pwm_common_sixstep.h"
<#if  MC.DRIVE_MODE == "CM">
#include "current_ref_ctrl.h"
</#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
#include "mc_interface.h"
#include "mc_configuration_registers.h"
  <#if  MC.BUS_VOLTAGE_READING == true || MC.BUS_VOLTAGE_READING2 == true>
#include "r_divider_bus_voltage_sensor.h"
  </#if>
  <#if  MC.BUS_VOLTAGE_READING == false || MC.BUS_VOLTAGE_READING2 == false>
#include "virtual_bus_voltage_sensor.h"
  </#if>
<#if MC.DRIVE_TYPE == "FOC">
  <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
#include "feed_forward_ctrl.h"
  </#if> 
  <#if MC.FLUX_WEAKENING_ENABLING == true || MC.FLUX_WEAKENING_ENABLING2 == true>
#include "flux_weakening_ctrl.h"
  </#if> 
  <#if MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>
#include "trajectory_ctrl.h"
  </#if>
#include "pqd_motor_power_measurement.h"
  <#if MC.USE_STGAP1S >
#include "gap_gate_driver_ctrl.h"
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || (MC.HW_OV_CURRENT_PROT_BYPASS == true && MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES") || 
     (MC.DUALDRIVE == true && ((MC.HW_OV_CURRENT_PROT_BYPASS2 == true && MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES") || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE")) >
#include "digital_output.h"
</#if><#-- Resistive break or OC protection bypass on OV --> 
<#if MC.STSPIN32G4 == true >
#include "stspin32g4.h"
</#if><#-- MC.STSPIN32G4 == true -->
  <#if MC.MOTOR_PROFILER == true>
#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"
  <#if  MC.AUX_HALL_SENSORS>
#include "mp_hall_tuning.h"
  </#if>
  </#if>

<#-- Specific to FOC algorithm usage -->
<#if MC.DRIVE_TYPE == "FOC">
  <#-- Specific to F3 family usage -->
  <#if CondFamily_STM32F3 && MC.THREE_SHUNT == true> 
#include "r3_1_f30x_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F3 && (MC.SINGLE_SHUNT == true || MC.SINGLE_SHUNT2 == true)>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F3 && (MC.THREE_SHUNT_SHARED_RESOURCES == true || MC.THREE_SHUNT_SHARED_RESOURCES2 == true ||
                            MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true)>  
#include "r3_2_f30x_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F3 && (MC.ICS_SENSORS == true || MC.ICS_SENSORS2 == true)>
#include "ics_f30x_pwm_curr_fdbk.h"
  </#if>
  <#-- Specific to F4 family usage -->
  <#if CondFamily_STM32F4 > 
    <#if MC.SINGLE_SHUNT == true || MC.SINGLE_SHUNT2 == true  >
#include "r1_ps_pwm_curr_fdbk.h"          
    </#if>
    <#if MC.THREE_SHUNT == true >
#include "r3_1_f4xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true>
#include "r3_2_f4xx_pwm_curr_fdbk.h"    
    </#if>
    <#if MC.ICS_SENSORS || MC.ICS_SENSORS2 == true > 
#include "ics_f4xx_pwm_curr_fdbk.h"    
    </#if>  
  </#if>
  <#-- Specific to G0 family usage -->
  <#if CondFamily_STM32G0 > 
    <#if MC.SINGLE_SHUNT>
#include "r1_ps_pwm_curr_fdbk.h"          
    </#if>
    <#if MC.THREE_SHUNT == true >
#include "r3_g0xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS == true > 
#include "ics_g0xx_pwm_curr_fdbk.h"    
    </#if>  
  </#if>
  <#-- Specific to L4 family usage -->
  <#if CondFamily_STM32L4 && MC.THREE_SHUNT == true> 
#include "r3_1_l4xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32L4 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true> 
#include "r3_2_l4xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32L4 && MC.SINGLE_SHUNT == true> 
#include "r1_ps_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32L4 && MC.ICS_SENSORS == true> 
#include "ics_l4xx_pwm_curr_fdbk.h"
  </#if>
  <#-- Specific to F7 family usage -->
  <#if CondFamily_STM32F7 && MC.THREE_SHUNT == true> 
#include "r3_1_f7xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F7 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true> 
#include "r3_2_f7xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F7 && MC.SINGLE_SHUNT == true> 
#include "r1_ps_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32F7 && MC.ICS_SENSORS == true> 
#include "ics_f7xx_pwm_curr_fdbk.h"
  </#if>
  <#-- Specific to H7 family usage -->
  <#if CondFamily_STM32H7 && MC.THREE_SHUNT == true> 
#include "r3_1_h7xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32H7 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true> 
#include "r3_2_h7xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32H7 && MC.SINGLE_SHUNT == true> 
#include "r1_h7xx_pwm_curr_fdbk.h"
  </#if>
  <#if CondFamily_STM32H7 && MC.ICS_SENSORS == true> 
#include "ics_h7xx_pwm_curr_fdbk.h"
  </#if>
  <#-- Specific to F0 family usage -->
  <#if CondFamily_STM32F0 && MC.SINGLE_SHUNT == true> 
#include "r1_ps_pwm_curr_fdbk.h"      
  </#if>
  <#if CondFamily_STM32F0 && MC.THREE_SHUNT == true>
#include "r3_f0xx_pwm_curr_fdbk.h"
  </#if>
  <#-- Specific to G4 family usage -->
  <#if CondFamily_STM32G4 >
    <#if MC.SINGLE_SHUNT || MC.SINGLE_SHUNT2 >
#include "r1_ps_pwm_curr_fdbk.h"          
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2  >
#include "r3_2_g4xx_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.ICS_SENSORS || MC.ICS_SENSORS2  >
#include "ics_g4xx_pwm_curr_fdbk.h"
    </#if>
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if MC.DRIVE_TYPE == "SIX_STEP">
  <#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#include "pwmc_6pwm.h"
  <#else>
#include "pwmc_3pwm.h"
  </#if>

  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
	<#if CondFamily_STM32F0>
#include "f0xx_bemf_ADC_fdbk.h"
    </#if>
	<#if CondFamily_STM32G0>
#include "g0xx_bemf_ADC_fdbk.h"
    </#if>
	<#if CondFamily_STM32G4>
#include "g4xx_bemf_ADC_fdbk.h"
    </#if>
  </#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

<#if MC.DRIVE_TYPE == "FOC">
  <#-- MTPA feature usage -->
  <#if MC.MTPA_ENABLING == true || MC.MTPA_ENABLING2 == true>
#include "max_torque_per_ampere.h"
  </#if>
  <#-- ICL feature usage -->
  <#if  MC.M1_ICL_ENABLED == true || MC.M2_ICL_ENABLED == true>
#include "inrush_current_limiter.h"
  </#if>
  <#-- Open Loop feature usage -->
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
#include "open_loop.h"
  </#if> 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if MC.DRIVE_TYPE == "FOC">
  <#-- Position sensors feature usage -->
  <#if MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true>
#include "encoder_speed_pos_fdbk.h"
#include "enc_align_ctrl.h"
  </#if>
</#if>

  <#if MC.HALL_SENSORS ==true || MC.AUX_HALL_SENSORS == true || MC.HALL_SENSORS2==true || MC.AUX_HALL_SENSORS2 == true>
#include "hall_speed_pos_fdbk.h"
  </#if>
<#if MC.M1_POTENTIOMETER_ENABLE == true>
#include "speed_potentiometer.h"
</#if>
<#if   MC.ESC_ENABLE == true>
#include "esc.h"
</#if>

<#if MC.DRIVE_TYPE == "FOC">
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"

  <#if  MC.STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_PLL == true || 
      MC.STATE_OBSERVER_CORDIC == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true>
#include "sto_speed_pos_fdbk.h"
  </#if>
  <#if MC.AUX_STATE_OBSERVER_PLL == true || MC.AUX_STATE_OBSERVER_PLL2== true ||
    MC.STATE_OBSERVER_PLL== true || MC.STATE_OBSERVER_PLL2== true >
#include "sto_pll_speed_pos_fdbk.h"
  </#if>
  <#if MC.AUX_STATE_OBSERVER_CORDIC == true || MC.AUX_STATE_OBSERVER_CORDIC2== true ||
    MC.STATE_OBSERVER_CORDIC== true || MC.STATE_OBSERVER_CORDIC2== true >
#include "sto_cordic_speed_pos_fdbk.h"
  </#if>
  <#-- PFC feature usage -->
  <#if MC.PFC_ENABLED == true>
#include "pfc.h"
  </#if>
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 
</#if>

<#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
extern RevUpCtrl_Handle_t RevUpControlM1;
</#if> 

extern PID_Handle_t PIDSpeedHandle_M1;
<#if MC.DRIVE_TYPE == "FOC">
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
extern NTC_Handle_t TempSensor_M1;
<#if MC.DRIVE_TYPE == "FOC">
  <#-- Flux Weakening feature usage -->
  <#if MC.FLUX_WEAKENING_ENABLING == true>
extern PID_Handle_t PIDFluxWeakeningHandle_M1;
extern FW_Handle_t FW_M1;
  </#if> 
  <#if  MC.POSITION_CTRL_ENABLING == true >
extern PID_Handle_t PID_PosParamsM1;
extern PosCtrl_Handle_t PosCtrlM1;
  </#if>
  
  <#if (CondFamily_STM32H7 == false) && MC.SINGLE_SHUNT == true>
extern PWMC_R1_Handle_t PWM_Handle_M1;
  </#if>
    
  <#if (CondFamily_STM32G4 == false || CondFamily_STM32H7 == false ) && MC.THREE_SHUNT == true>
extern PWMC_R3_1_Handle_t PWM_Handle_M1;
  </#if>

  <#if (CondFamily_STM32F0 == false || CondFamily_STM32G0 == false || CondFamily_STM32H7 == false) && MC.ICS_SENSORS == true>
extern PWMC_ICS_Handle_t PWM_Handle_M1;
  </#if>
  
 	<#if ((CondFamily_STM32F4 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true) || (CondFamily_STM32F3 &&  (MC.THREE_SHUNT_SHARED_RESOURCES || MC.THREE_SHUNT_INDEPENDENT_RESOURCES)) || (CondFamily_STM32L4 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true) ||(CondFamily_STM32F7 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true) || (CondFamily_STM32H7 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES) ||(CondFamily_STM32G4 && (MC.THREE_SHUNT_INDEPENDENT_RESOURCES || MC.THREE_SHUNT_SHARED_RESOURCES)))> 
extern PWMC_R3_2_Handle_t PWM_Handle_M1;
  </#if>
  
  
  <#-- Specific to F4 family usage -->
  <#if CondFamily_STM32F4 >
    <#if MC.SINGLE_SHUNT2 == true> 
extern PWMC_R1_Handle_t PWM_Handle_M2;
    <#elseif MC.ICS_SENSORS2 == true> 
extern PWMC_ICS_Handle_t PWM_Handle_M2;
    </#if>
  </#if>
   <#-- Specific to G4 family usage -->
  <#if CondFamily_STM32G4 >
    <#if MC.SINGLE_SHUNT2>
extern PWMC_R1_Handle_t PWM_Handle_M2;
    </#if> 
    <#if MC.ICS_SENSORS2>
extern PWMC_ICS_Handle_t PWM_Handle_M2;
    </#if>  
  </#if>
 </#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
  <#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
extern PWMC_SixPwm_Handle_t PWM_Handle_M1;
  <#else>
extern PWMC_ThreePwm_Handle_t PWM_Handle_M1;
  </#if>
  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
extern Bemf_ADC_Handle_t Bemf_ADC_M1;
  </#if>
  <#if  MC.DRIVE_MODE == "CM">
extern CurrentRef_Handle_t CurrentRef_M1;
  </#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
<#if MC.DRIVE_TYPE == "FOC">
  <#-- Specific to Dual Drive feature usage -->
  <#if MC.DUALDRIVE == true>
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2;
extern PID_Handle_t PIDSpeedHandle_M2;
extern PID_Handle_t PIDIqHandle_M2;
extern PID_Handle_t PIDIdHandle_M2;
extern NTC_Handle_t TempSensor_M2;
    <#if MC.FLUX_WEAKENING_ENABLING2 == true>
extern PID_Handle_t PIDFluxWeakeningHandle_M2;
extern FW_Handle_t FW_M2;
    </#if>
    <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
extern FF_Handle_t FF_M2;
    </#if> 
    <#if  MC.POSITION_CTRL_ENABLING2 == true >
extern PID_Handle_t PID_PosParamsM2;
extern PosCtrl_Handle_t PosCtrlM2;
    </#if>
    <#if CondFamily_STM32F3 && MC.SINGLE_SHUNT2 == true>
extern PWMC_R1_Handle_t PWM_Handle_M2;
    </#if>
    <#if (MC.THREE_SHUNT_SHARED_RESOURCES2 || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 )>
extern PWMC_R3_2_Handle_t PWM_Handle_M2;
    </#if>
    <#if CondFamily_STM32F3 && MC.ICS_SENSORS2 == true> 
extern PWMC_ICS_Handle_t PWM_Handle_M2;
    </#if>
  </#if> 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
<#if MC.DRIVE_TYPE == "FOC">
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1; 
  <#if MC.USE_STGAP1S >
extern GAP_Handle_t STGAP_M1;
  </#if>
  <#if MC.DUALDRIVE == true>
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2; 
  </#if> 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.ENCODER == true || MC.VIEW_ENCODER_FEEDBACK == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC"> 
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
</#if> 
<#if MC.DRIVE_TYPE == "FOC">
  <#if MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true || MC.ENCODER2 == true || MC.VIEW_ENCODER_FEEDBACK2 == true> 
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2;
  </#if> 
  <#if  MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true>
extern STO_Handle_t STO_M1;
  </#if> 
  <#if  MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true>
extern RevUpCtrl_Handle_t RevUpControlM2;
extern STO_Handle_t STO_M2;
  </#if> 
  <#if MC.STATE_OBSERVER_PLL == true || MC.AUX_STATE_OBSERVER_PLL == true>
extern STO_PLL_Handle_t STO_PLL_M1;
  </#if>  
  <#if MC.STATE_OBSERVER_PLL2 == true || MC.AUX_STATE_OBSERVER_PLL2 == true  >
  extern STO_PLL_Handle_t STO_PLL_M2;
  </#if>
  <#if MC.STATE_OBSERVER_CORDIC2 == true || MC.AUX_STATE_OBSERVER_CORDIC2 == true>
extern STO_CR_Handle_t STO_CR_M2;
  </#if>  
  <#if MC.STATE_OBSERVER_CORDIC == true || MC.AUX_STATE_OBSERVER_CORDIC == true>
extern STO_CR_Handle_t STO_CR_M1;
  </#if>  
  <#if MC.ENCODER == true|| MC.AUX_ENCODER == true >
extern ENCODER_Handle_t ENCODER_M1;
extern EncAlign_Handle_t EncAlignCtrlM1;
  </#if>  
  <#if MC.ENCODER2 == true|| MC.AUX_ENCODER2 == true >
extern ENCODER_Handle_t ENCODER_M2;
extern EncAlign_Handle_t EncAlignCtrlM2;
  </#if>  
  <#if MC.HALL_SENSORS2 == true || MC.AUX_HALL_SENSORS2 == true >
extern HALL_Handle_t HALL_M2;
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.HALL_SENSORS == true || MC.AUX_HALL_SENSORS == true >
extern HALL_Handle_t HALL_M1;
  </#if>  
<#if MC.DRIVE_TYPE == "FOC">
  <#if  MC.M1_ICL_ENABLED == true>
extern ICL_Handle_t ICL_M1;
  </#if>
  <#if MC.M2_ICL_ENABLED == true && MC.DUALDRIVE == true>
extern ICL_Handle_t ICL_M2;
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if  MC.BUS_VOLTAGE_READING == true>
extern RDivider_Handle_t BusVoltageSensor_M1;
<#else>
extern VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M1;
</#if>
<#if  MC.DUALDRIVE == true>
  <#if  MC.BUS_VOLTAGE_READING2 == true>
extern RDivider_Handle_t BusVoltageSensor_M2;
  <#else>
extern VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M2;
  </#if>
</#if>
<#if MC.DRIVE_TYPE == "FOC">
extern CircleLimitation_Handle_t CircleLimitationM1;
  <#if MC.DUALDRIVE == true>
extern CircleLimitation_Handle_t CircleLimitationM2;
  </#if>
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
  <#if MC.DUALDRIVE == true >
extern RampExtMngr_Handle_t RampExtMngrHFParamsM2;
  </#if>
  <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING == true>
extern FF_Handle_t FF_M1;
  </#if> 
  <#if MC.MTPA_ENABLING == true>
extern MTPA_Handle_t MTPARegM1;
  </#if>
  <#if MC.DUALDRIVE == true && MC.MTPA_ENABLING2 == true>
extern MTPA_Handle_t MTPARegM2;
  </#if>
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
extern OpenLoop_Handle_t OpenLoop_ParamsM1;
  </#if> 
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
extern OpenLoop_Handle_t OpenLoop_ParamsM2;
  </#if>
  <#if  MC.DUALDRIVE == true>
    <#if  MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
extern DOUT_handle_t R_BrakeParamsM2;
    </#if>
    <#if MC.HW_OV_CURRENT_PROT_BYPASS2 == true && MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES">
extern DOUT_handle_t DOUT_OCPDisablingParamsM2;
    </#if>
    <#if MC.M2_ICL_ENABLED == true>
extern DOUT_handle_t ICLDOUTParamsM2;
    </#if>
  </#if>
  <#if  MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
extern DOUT_handle_t R_BrakeParamsM1;
  </#if>
  <#if MC.HW_OV_CURRENT_PROT_BYPASS == true && MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
extern DOUT_handle_t DOUT_OCPDisablingParamsM1;
  </#if>
  <#if MC.M1_ICL_ENABLED == true>
extern DOUT_handle_t ICLDOUTParamsM1;
  </#if>
</#if><#-- DRIVE_TYPE == "FOC" -->


<#if MC.DRIVE_TYPE == "FOC">
  <#-- PFC feature usage -->
  <#if MC.PFC_ENABLED == true>
extern PFC_Handle_t PFC;
  </#if>
  
  <#-- Motor Profiler feature usage -->
  <#if MC.MOTOR_PROFILER == true>
extern RampExtMngr_Handle_t RampExtMngrParamsSCC;
extern RampExtMngr_Handle_t RampExtMngrParamsOTT;
extern SCC_Handle_t SCC;
extern OTT_Handle_t OTT;
  <#if  MC.AUX_HALL_SENSORS>
extern HT_Handle_t HT;  
  </#if>
</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.M1_POTENTIOMETER_ENABLE == true>
extern SpeedPotentiometer_Handle_t SpeedPotentiometer_M1;
</#if>
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
<#if MC.DRIVE_TYPE == "FOC">
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
extern NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
<#if MC.DRIVE_TYPE == "FOC">
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]; 
  <#if MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>
extern PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS];
  </#if>
  <#if MC.FLUX_WEAKENING_ENABLING==true || MC.FLUX_WEAKENING_ENABLING2==true>
extern FW_Handle_t *pFW[NBR_OF_MOTORS];
  </#if>
  <#if  MC.FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
extern FF_Handle_t *pFF[NBR_OF_MOTORS];
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
  <#if MC.STSPIN32G4 == true >
extern STSPIN32G4_HandleTypeDef HdlSTSPING4;
  </#if><#-- MC.STSPIN32G4 == true -->
<#if MC.ESC_ENABLE == true>
extern ESC_Handle_t ESC_M1;
</#if>
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  
 
  

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
