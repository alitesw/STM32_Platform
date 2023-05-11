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
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
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
  
#include "mc_type.h"
#include "mc_configuration_registers.h"
#include "register_interface.h"
#include "parameters_conversion.h"

#define FIRMWARE_NAME_STR "ST MC SDK\tVer.6.1.1"
#define MAX_READABLE_CURRENT ${MC.ADC_REFERENCE_VOLTAGE}/(2*${MC.RSHUNT}*${MC.AMPLIFICATION_GAIN})

const char_t CTL_BOARD[] = "${MC.CTLBOARD_NAME}";
static const char_t M1_PWR_BOARD[] = "${MC.POWERBOARD_NAME}";
const char_t FIRMWARE_NAME [] = FIRMWARE_NAME_STR;

const GlobalConfig_reg_t globalConfig_reg = 
{
  .SDKVersion =  SDK_VERSION,
  .MotorNumber = <#if MC.SINGLEDRIVE> 1 <#else> 2 </#if>,
  .MCP_Flag = MCP_OVER_STLINK+MCP_OVER_UARTA+MCP_OVER_UARTB,
  .MCPA_UARTA_LOG = ${MC.MCP_DATALOG_OVER_UART_A_SIGNALS},
  .MCPA_UARTB_LOG = ${MC.MCP_DATALOG_OVER_UART_B_SIGNALS},
  .MCPA_STLNK_LOG = ${MC.MCP_DATALOG_OVER_STLNK_SIGNALS},
};

static const ApplicationConfig_reg_t M1_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = ${MC.MAX_APPLICATION_SPEED},
  .maxReadableCurrent = MAX_READABLE_CURRENT,
  .nominalCurrent = ${MC.NOMINAL_CURRENT},
  .nominalVoltage = ${MC.NOMINAL_BUS_VOLTAGE_V},
  .driveType = DRIVE_TYPE_M1,
};

const MotorConfig_reg_t M1_MotorConfig_reg =
{
  .polePairs = ${MC.POLE_PAIR_NUM},
  .ratedFlux = ${MC.MOTOR_VOLTAGE_CONSTANT},
  .rs = ${MC.RS},
  .ls = ${MC.LS}*${MC.LDLQ_RATIO},
  .ld = ${MC.LS},   <#-- This is not a Typo, WB stores ld value into LS field-->
  .maxCurrent = ${MC.NOMINAL_CURRENT},
  .name = "${MC.MOTOR_NAME}"
};

<#if MC.DRIVE_TYPE == "FOC" || MC.DRIVE_TYPE == "ACIM" >
static const FOCFwConfig_reg_t M1_FOCConfig_reg =
{
  .primarySensor = (uint8_t) PRIM_SENSOR_M1,
  .auxiliarySensor = (uint8_t) AUX_SENSOR_M1,
  .topology = (uint8_t) TOPOLOGY_M1,
  .FOCRate = (uint8_t) FOC_RATE_M1,
  .PWMFrequency = (uint32_t) PWM_FREQ_M1,
  .MediumFrequency = (uint16_t) MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t) configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .configurationFlag2 = (uint16_t) configurationFlag2_M1, //cstat !MISRAC2012-Rule-10.1_R6
};
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP" >
static const SixStepFwConfig_reg_t M1_SixStepConfig_reg =
{
  .primarySensor = (uint8_t) PRIM_SENSOR_M1,
  .topology = (uint8_t) TOPOLOGY_M1,
  .PWMFrequency = (uint32_t) PWM_FREQ_M1,
  .MediumFrequency = (uint16_t) MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t) configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
};
</#if>

<#if   MC.DUALDRIVE == true>
const char_t M2_PWR_BOARD[] = ${MC.POWERBOARD_NAME2}

const MotorConfig_reg_t M2_MotorConfig_reg =
{
  .polePairs = ${MC.POLE_PAIR_NUM2},
  .ratedFlux = ${MC.MOTOR_VOLTAGE_CONSTANT2},
  .rs = ${MC.RS2},
  .ls = ${MC.LS2}*${MC.LDLQ_RATIO2},
  .ld = ${MC.LS2},   <#-- This is not a Typo, if LDLQ_RATIO != 1, WB stores ld value into LS field-->
  .maxCurrent = ${MC.NOMINAL_CURRENT2},
  .name = "${MC.MOTOR_NAME2}"
};

const ApplicationConfig_reg_t M2_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = ${MC.MAX_APPLICATION_SPEED2},
  .maxReadableCurrent = MAX_READ_CURRENT2,
  .nominalCurrent = ${MC.NOMINAL_CURRENT2},
  .nominalVoltage = ${MC.NOMINAL_BUS_VOLTAGE_V2},
  .driveType = DRIVE_TYPE_M2,
};

const FOCFwConfig_reg_t M2_FOCConfig_reg =
{
  .primarySensor = (uint8_t) PRIM_SENSOR_M2,
  .auxiliarySensor = (uint8_t) AUX_SENSOR_M2,
  .topology = (uint8_t) TOPOLOGY_M2,
  .FOCRate = (uint8_t) FOC_RATE_M2,
  .PWMFrequency = (uint32_t) PWM_FREQ_M2,
  .configurationFlag1 = (uint16_t) configurationFlag1_M2,
};

const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS]={ &M1_FOCConfig_reg, &M2_FOCConfig_reg};
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS]={ &M1_MotorConfig_reg, &M2_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS]={ &M1_ApplicationConfig_reg, &M2_ApplicationConfig_reg};
const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD ,M2_PWR_BOARD };
<#else>
const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD};
  <#if MC.DRIVE_TYPE == "FOC" ||  MC.DRIVE_TYPE == "ACIM">
const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS]={ &M1_FOCConfig_reg };
  </#if>
  <#if MC.DRIVE_TYPE == "SIX_STEP" >
const SixStepFwConfig_reg_t* SixStepConfig_reg[NBR_OF_MOTORS]={ &M1_SixStepConfig_reg };
  </#if>
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS]={ &M1_MotorConfig_reg };
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS]={ &M1_ApplicationConfig_reg};
</#if>


/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
