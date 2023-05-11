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
  * @file    dac_ui.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          dac_ui component of the Motor Control SDK.
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
#ifndef DACUI_H
#define DACUI_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

 /** @addtogroup MCSDK
   * @{
   */

 /** @addtogroup MCUI
   * @{
   */

 /** @addtogroup DAC_UserInterface
   * @{
   */

#define DAC_CH_NBR 2
#define DAC_CH_USER 2

typedef enum
{
  DAC_CH1,
  DAC_CH2,
  DAC_CH3,
} DAC_Channel_t;

typedef struct
{

  uint16_t *ptrDataCh[DAC_CH_NBR]; /* Pointer of the data dumped into DAC */
  uint16_t dataCh[DAC_CH_NBR]; /* ID of the data dumped into DAC*/
<#if !MC.SINGLEDRIVE >
  uint8_t motorCh[DAC_CH_NBR]; /* In case of multi drive, moteurID per data is required*/
</#if>  
} DAC_Handle_t;

extern DAC_Handle_t DAC_Handle;

void DAC_Init(DAC_Handle_t *pHandle);

void DAC_SetChannelConfig(DAC_Handle_t *pHandle, DAC_Channel_t bChannel, uint16_t regID);

uint16_t DAC_GetChannelConfig(DAC_Handle_t *pHandle, DAC_Channel_t bChannel);

<#if MC.SINGLEDRIVE >
void DAC_Exec(DAC_Handle_t *pHandle);
<#else>
void DAC_Exec(DAC_Handle_t *pHandle, uint8_t motorID);
</#if>


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* DACUI_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
