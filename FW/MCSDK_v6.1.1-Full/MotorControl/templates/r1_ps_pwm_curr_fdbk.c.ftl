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
<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
<#if CondFamily_STM32G0 || CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32F0 || CondFamily_STM32G4>
<#assign currentFactor = "">
<#elseif CondFamily_STM32F4 || CondFamily_STM32F7>
<#assign currentFactor = "*2">
</#if>
<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32G4>
<#assign HAS_ADC_INJ = true>
<#else>
<#assign HAS_ADC_INJ = false>
</#if>
<#if CondFamily_STM32F4 || CondFamily_STM32F7>
<#assign DMA_TYPE = "Stream">
<#assign DMA_MODE = "NORMAL">
<#else>
<#assign DMA_TYPE = "Channel">
<#assign DMA_MODE = "CIRCULAR">
</#if>
<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4 || CondFamily_STM32F7>
<#assign HAS_BRK2 = true>
<#else>
<#assign HAS_BRK2 = false>
</#if>

/**
  ******************************************************************************
  * @file    r1_ps_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the CCC component of the Motor Control SDK:
  *           + initializes MCU peripheral for 1 shunt topology
  *           + performs PWM duty cycle computation and generation
  *           + performs current sensing
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
  * @ingroup r1_ps_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r1_ps_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r1_ps_pwm_curr_fdbk R1 PWM & Current Feedback
 *
 * @brief 1-Shunt with phase shift PWM & Current Feedback implementation for F30X, F7XX, G0XX, G4XX and L4XX MCUs.
 *
 * This component is used in applications based on an STM32 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @{
 */

/* Constant values -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123 (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|\
                             TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)
                                 
#define DMA_DUTY_CFG (LL_DMA_PERIPH_NOINCREMENT|LL_DMA_MEMORY_INCREMENT|LL_DMA_PDATAALIGN_HALFWORD|\
                      LL_DMA_DIRECTION_MEMORY_TO_PERIPH|LL_DMA_MDATAALIGN_HALFWORD|LL_DMA_PRIORITY_VERYHIGH|\
                      LL_DMA_MODE_${DMA_MODE})           
#define DMA_TRANSFER_LENGTH  8u                                      

#define IA_OK 0x01U
#define IB_OK 0x02U
#define IC_OK 0x04U

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void R1_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
<#if CondFamily_STM32F3 || CondFamily_STM32G4>
<#if MC.INTERNAL_OVERCURRENTPROTECTION == true || MC.INTERNAL_OVERCURRENTPROTECTION2 == true || 
     MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true> 
static void R1_SetAOReferenceVoltage( uint32_t DAC_Channel, uint16_t hDACVref );
</#if>
</#if>
static void R1_1ShuntMotorVarsInit( PWMC_Handle_t * pHdl );
static void R1_TIMxInit( TIM_TypeDef * TIMx, PWMC_R1_Handle_t * pHdl );
static uint16_t R1_SetADCSampPointPolarization( PWMC_Handle_t * pHdl );

/**
  * @brief  Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in single shunt topology using STM32 MCU and ADC.
  * 
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
void R1_Init(PWMC_R1_Handle_t *pHandle)
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  if ( NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif

<#if MC.USE_INTERNAL_OPAMP == true || MC.USE_INTERNAL_OPAMP2 == true>
    OPAMP_TypeDef * OPAMPx = pHandle->pParams_str->OPAMP_Selection;
</#if>  
<#if MC.INTERNAL_OVERCURRENTPROTECTION == true || MC.INTERNAL_OVERCURRENTPROTECTION2 == true>
    COMP_TypeDef * COMP_OCPx = pHandle->pParams_str->CompOCPSelection;
</#if>
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>    
    COMP_TypeDef * COMP_OVPx = pHandle->pParams_str->CompOVPSelection;
</#if>  
    TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
    ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
    DMA_TypeDef * DMAx = pHandle->pParams_str->DMAx;  
<#if MC.USE_INTERNAL_OPAMP == true || MC.USE_INTERNAL_OPAMP2 == true ||
     MC.INTERNAL_OVERCURRENTPROTECTION == true || MC.INTERNAL_OVERCURRENTPROTECTION2 == true ||
     MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true> 
  volatile uint16_t waittime; 
</#if> 

    R1_1ShuntMotorVarsInit( &pHandle->_Super );

  /********************************************************************************************/
  /*                                      TIM Initialization                                  */
  /********************************************************************************************/
    R1_TIMxInit(TIMx, pHandle);
  
  /********************************************************************************************/
  /*                                      DMA Initialization                                  */
  /********************************************************************************************/
    LL_DMA_ConfigTransfer(DMAx, pHandle->pParams_str->DMAChannelX, DMA_DUTY_CFG); 

    LL_TIM_ConfigDMABurst(TIMx, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);

    /* cfg dma source and dest address */
    //cstat !MISRAC2012-Rule-11.4
    LL_DMA_SetMemoryAddress(DMAx, pHandle->pParams_str->DMAChannelX, (uint32_t)&pHandle->DmaBuffCCR[0]);
    //cstat !MISRAC2012-Rule-11.4
    LL_DMA_SetPeriphAddress(DMAx, pHandle->pParams_str->DMAChannelX, (uint32_t)&TIMx->DMAR); 

    /* cfg dma transfer size */
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH);
 
<#if HAS_ADC_INJ == false>
    /* DMA dedicated to ADC conversion*/
    LL_DMA_SetMemoryAddress(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)pHandle->CurConv);
    //cstat !MISRAC2012-Rule-11.4
    LL_DMA_SetPeriphAddress(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)&ADCx->DR);
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, 2u);
</#if>
<#if MC.USE_INTERNAL_OPAMP == true || MC.USE_INTERNAL_OPAMP2 == true>
  /********************************************************************************************/
  /*                                      OPAMP Initialization                                */
  /********************************************************************************************/
    if (OPAMPx)
    {
      /* enable OPAMP */
      LL_OPAMP_Enable(OPAMPx);
     LL_OPAMP_Lock(OPAMPx);
    }
  
</#if>
<#if MC.INTERNAL_OVERCURRENTPROTECTION == true || MC.INTERNAL_OVERCURRENTPROTECTION2 == true>
  /********************************************************************************************/
  /*                                      COMP Initialization                                 */
  /********************************************************************************************/
    /* Over current protection */
    if (COMP_OCPx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->CompOCPInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_INPUT_MINUS_DAC1_CH1 == LL_COMP_GetInputMinus(COMP_OCPx))
        {
          R1_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_INPUT_MINUS_DAC1_CH2 == LL_COMP_GetInputMinus(COMP_OCPx))
        {
          R1_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
        }
#endif
        else
        {
          /* Nothing to do */
        }
      }

      /* Wait to stabilize DAC voltage */
    waittime = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US + LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US) * (SystemCoreClock / 1000000U));
    while(waittime != 0UL)
    {
      waittime--;
    }

     
    /* enable comparator */
#if defined(COMP_CSR_COMPxHYST)
    LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
    LL_COMP_Enable(COMP_OCPx);
    LL_COMP_Lock(COMP_OCPx);
    
    /* Wait to stabilize COMP voltage */
    waittime = ((LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US + LL_COMP_DELAY_STARTUP_US) * (SystemCoreClock / 1000000UL)); 
    while(waittime != 0UL)
    {
      waittime--;
    }    
  }
</#if>
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>   

    /* Over voltage protection */
    if (COMP_OVPx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->CompOCPInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_INPUT_MINUS_DAC1_CH1 == LL_COMP_GetInputMinus(COMP_OVPx))
        {
          R1_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->DAC_OVP_Threshold));
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_INPUT_MINUS_DAC1_CH2 == LL_COMP_GetInputMinus(COMP_OVPx))
        {
          R1_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->DAC_OVP_Threshold));
        }
#endif
        else
        {
          /* Nothing to do */
        }
      }
    
      /* Wait to stabilize DAC voltage */
      waittime = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US+ LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US) * (SystemCoreClock / 1000000UL));
      while(waittime != 0UL)
      {
        waittime--;
      }
      

    
      /* enable comparator */
#if defined(COMP_CSR_COMPxHYST)
      LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
      LL_COMP_Enable(COMP_OVPx);
      LL_COMP_Lock(COMP_OVPx);
   
      /* Wait to stabilize COMP voltage */
      waittime = ((LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US + LL_COMP_DELAY_STARTUP_US) * (SystemCoreClock / 1000000UL)); 
      while(waittime != 0UL)
      {
        waittime--;
      }
    
    }
    
</#if>
  /********************************************************************************************/
  /*                                      TIM Initialization                                  */
  /********************************************************************************************/
  R1_TIMxInit( TIMx, pHandle );

  /********************************************************************************************/
  /*                                      ADC Initialization                                  */
  /********************************************************************************************/
<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32F7>

<#if CondFamily_STM32F7> 
    LL_ADC_DisableIT_EOCS(ADCx);
    LL_ADC_ClearFlag_EOCS(ADCx);
    LL_ADC_DisableIT_JEOS(ADCx);
    LL_ADC_ClearFlag_JEOS(ADCx);
<#else>
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
<#if HAS_ADC_INJ == false>
    LL_ADC_DisableIT_EOC(ADCx);
    LL_ADC_ClearFlag_EOC(ADCx);
    LL_ADC_DisableIT_EOS(ADCx);
    LL_ADC_ClearFlag_EOS(ADCx);
    
<#else>
    LL_ADC_DisableIT_EOC(ADCx);
    LL_ADC_ClearFlag_EOC(ADCx);
    LL_ADC_DisableIT_JEOC(ADCx);
    LL_ADC_ClearFlag_JEOC(ADCx);
  
</#if>
</#if>

<#if CondFamily_STM32G4>
    /* - Exit from deep-power-down mode */     
    LL_ADC_DisableDeepPowerDown(ADCx);
</#if> 
  
<#if CondFamily_STM32F7>
    LL_ADC_Enable(ADCx);
<#else>  

    LL_ADC_EnableInternalRegulator(ADCx);
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) 
                                       * (SystemCoreClock / (100000UL * 2UL)));      
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
    
<#if HAS_ADC_INJ == false>
    LL_ADC_StartCalibration(ADCx);
<#else>
    LL_ADC_StartCalibration(ADCx, LL_ADC_SINGLE_ENDED);
</#if>
    while (LL_ADC_IsCalibrationOnGoing(ADCx) != 0U)
    {
      /* Nothing to do */
    }
    /* ADC Enable (must be done after calibration) */
    /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0 
    * following a calibration phase, could have no effect on ADC 
    * within certain AHB/ADC clock ratio.
    */
    while (0U == LL_ADC_IsActiveFlag_ADRDY(ADCx))  
    { 
      LL_ADC_Enable(ADCx);
    }
</#if>  
<#if HAS_ADC_INJ == false>
    LL_ADC_REG_SetSequencerConfigurable (ADCx, LL_ADC_REG_SEQ_FIXED);
    LL_ADC_REG_SetTriggerSource (ADCx, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_Enable(ADCx);
    
<#else>

    LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);
<#if CondFamily_STM32F7>
   
    /* store ADCx trigger source */
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
    pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_GetTriggerSource(ADCx);
#endif
#if (defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx))
    /* workaround software to fix hardware bug on STM32F4xx and STM32F5xx
     * set trigger detection to rising and falling edges to be able to
     * trigger ADC on TRGO2
     * refer to errata sheet ES0290 */
    LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISINGFALLING);
#else
    LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
#endif
   
    LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);
   
    LL_ADC_INJ_SetSequencerLength(ADCx, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
    LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_2, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
                                 
    if (TIM1 == pHandle->pParams_str->TIMx)
    {
      LL_ADC_INJ_SetTriggerSource(ADCx,LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
    }
#if defined(TIM8)
    else
    {
      LL_ADC_INJ_SetTriggerSource(ADCx,LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2);
    }
#endif 
                              
<#else>
<#-- TODO! need to be checked when dual drive will be supported -->
<#if CondFamily_STM32F3 || CondFamily_STM32L4>
    /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
    LL_ADC_INJ_StopConversion(ADCx);
    
</#if>
    LL_ADC_INJ_SetQueueMode(ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
    if (TIM1 == pHandle->pParams_str->TIMx)
    {
      LL_ADC_INJ_ConfigQueueContext(ADCx,
                                    LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
                                    LL_ADC_INJ_TRIG_EXT_RISING,
                                    LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
    }
#if defined(TIM8)
    else
    {
      LL_ADC_INJ_ConfigQueueContext(ADCx,
                                    LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2,
                                    LL_ADC_INJ_TRIG_EXT_RISING,
                                    LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
    }
#endif 
<#if CondFamily_STM32G4>
    /* dummy conversion (ES0431 doc chap. 2.5.4) */
    LL_ADC_REG_StartConversion(ADCx);
</#if>
</#if> 
</#if>  
</#if>
    /* Clear the flags */
<#if HAS_ADC_INJ == false>
    LL_TIM_EnableCounter(TIM1);
   
    pHandle->ADCRegularLocked=false; /* We allow ADC usage for regular conversion on Systick*/ 
</#if>
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;
   
    pHandle->_Super.DTTest = 0U;
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief Initializes @p TIMx peripheral with @p pHandle handler for PWM generation.
  * 
  */
void R1_TIMxInit(TIM_TypeDef *TIMx, PWMC_R1_Handle_t *pHandle)
{
<#if CondFamily_STM32G0>
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DBGMCU);
</#if>
<#if MC.DUALDRIVE == true> 
  if (pHandle->pParams_str->TIMx == TIM1)
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.PWM_TIMER_SELECTION)}_STOP);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.PWM_TIMER_SELECTION2)}_STOP);
  }
<#else>
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.PWM_TIMER_SELECTION)}_STOP);
</#if>
  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);
  
  LL_TIM_SetCounterMode(TIMx, TIM_CR1_CMS_1);
  
  /* Disable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
  
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);  
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH6); 
  
    
  /* Always enable BKIN for safety feature */
  LL_TIM_ClearFlag_BRK(TIMx);
<#if HAS_BRK2 == true>

  if ((pHandle->pParams_str->BKIN2Mode) != NONE)
  {
    LL_TIM_ClearFlag_BRK2(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  } 
<#else>
  /* BKIN, if enabled */
  if ((pHandle->pParams_str->EmergencyStop) != DISABLE)
  {
    LL_TIM_ClearFlag_BRK(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  }  
</#if>  
<#if MC.DUALDRIVE == true> 
</#if> 
  /* Enable drive of TIMx CHy and CHyN by TIMx CHyRef */
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
}

/**
  * @brief  First initialization of the @p pHdl handler.
  * 
  */
void R1_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  /* Init motor vars */
  pHandle->iflag=0;
  
  pHandle->Half_PWMPeriod = (pHandle->_Super.PWMperiod / 2U);

  pHandle->CntSmp1 = ((pHandle->Half_PWMPeriod) >> 1)
                   - (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((pHandle->Half_PWMPeriod) >> 1)
                   + (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
      
  pHandle->_Super.CntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhC = pHandle->Half_PWMPeriod >> 1;

  /* initialize buffer with the default duty cycle value */
  pHandle->DmaBuffCCR[0]       = pHandle->_Super.CntPhA;       // CCR1 value overwritten during first half PWM period
  pHandle->DmaBuffCCR_latch[0] = pHandle->_Super.CntPhA;       // CCR1 value overwritten during first half PWM period 
  pHandle->DmaBuffCCR[1]       = pHandle->_Super.CntPhB;       // CCR2 value overwritten during first half PWM period
  pHandle->DmaBuffCCR_latch[1] = pHandle->_Super.CntPhB;       // CCR2 value overwritten during first half PWM period
  pHandle->DmaBuffCCR[2]       = pHandle->_Super.CntPhC;       // CCR3 value overwritten during first half PWM period
  pHandle->DmaBuffCCR_latch[2] = pHandle->_Super.CntPhC;       // CCR3 value overwritten during first half PWM period  
  pHandle->DmaBuffCCR[3]       = pHandle->Half_PWMPeriod-1U;    // CCR4 value overwritten during first half PWM period
  
  pHandle->DmaBuffCCR[4]        = pHandle->_Super.CntPhA;       // CCR1 value overwritten during second half PWM period
  pHandle->DmaBuffCCR_latch[4]  = pHandle->_Super.CntPhA;       // CCR1 value overwritten during second half PWM period  
  pHandle->DmaBuffCCR[5]        = pHandle->_Super.CntPhB;       // CCR2 value overwritten during second half PWM period
  pHandle->DmaBuffCCR_latch[5]  = pHandle->_Super.CntPhB;       // CCR2 value overwritten during second half PWM period  
  pHandle->DmaBuffCCR[6]        = pHandle->_Super.CntPhC;       // CCR3 value overwritten during second half PWM period
  pHandle->DmaBuffCCR_latch[6]  = pHandle->_Super.CntPhC;       // CCR3 value overwritten during second half PWM period  
  pHandle->DmaBuffCCR[7]        = 0;                            // CCR4 value overwritten during second half PWM period
  
  pHandle->BrakeActionLock = false;
}

/**
  * @brief  Stores in @p pHdl handler the calibrated @p offsets.
  * 
  * In single shunt configuration, only phase A is used.
  */
__weak void R1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseOffset = offsets->phaseAOffset;

  pHdl->offsetCalibStatus = true;
}

/**
  * @brief Reads the calibrated @p offsets stored in @p pHdl.
  * 
  * In single shunt configuration, only phase A is read.
  */
__weak void R1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseOffset;
}

/**
  * @brief  Stores into the @p pHdl the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * 
  */
__weak void R1_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
 
  if (false == pHandle->_Super.offsetCalibStatus)
  {
  pHandle->PhaseOffset = 0u;
  pHandle->Index = 0u;
    
    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);
    
    /* Offset calibration  */
    /* Change function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R1_HFCurrentsCalibration;
    pHandle->_Super.pFctSetADCSampPointSectX = &R1_SetADCSampPointPolarization;
    
  R1_SwitchOnPWM( &pHandle->_Super );
    
    /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd(TIMx, &pHandle->_Super.SWerror, pHandle->pParams_str->RepetitionCounter, &pHandle->Index );
    
  R1_SwitchOffPWM( &pHandle->_Super );
    
  pHandle->PhaseOffset >>= 4u;
    pHandle->_Super.offsetCalibStatus = 0;
    
    /* Change back function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R1_GetPhaseCurrents;
    pHandle->_Super.pFctSetADCSampPointSectX = &R1_CalcDutyCycles;
  }  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  R1_1ShuntMotorVarsInit(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents ab_t format.
  *
  */
__weak void R1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *pStator_Currents)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
<#if HAS_ADC_INJ == true>   
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
</#if>  

  int32_t wAux1;
  int32_t wAux2;
  int32_t tempCurent;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC;
  
  /* clear TIMx UPDATE flag, used for FOC duration check */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Disabling the External triggering for ADCx */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO_RESET);

  /* First sampling point */  
<#if HAS_ADC_INJ == false> 
  wAux1 = (int32_t)pHandle->CurConv[0] ${currentFactor};
<#else>
  wAux1 = (int32_t)(ADCx->JDR1 ${currentFactor});
</#if> 
  wAux1 -= (int32_t)(pHandle->PhaseOffset);
  
  /* Check saturation */
  if (wAux1 > (-INT16_MAX))
  {
    if (wAux1 < INT16_MAX)
    {
      /* Nothing to do */
    }
    else
    {
      wAux1 = INT16_MAX;
    }
  }
  else
  {
    wAux1 = (-INT16_MAX);
  } 
   /* Second sampling point */
<#if HAS_ADC_INJ == false>    
  wAux2 = (int32_t)pHandle->CurConv[1] ${currentFactor};
<#else>  
  wAux2 = (int32_t)(ADCx->JDR2 ${currentFactor});
</#if> 
  wAux2 -= (int32_t)(pHandle->PhaseOffset);

  /* Check saturation */
  if (wAux2 > (-INT16_MAX))
  {
    if (wAux2 < INT16_MAX)
    {
      /* Nothing to do */
    }
    else
    {
      wAux2 = INT16_MAX;
    }
  }
  else
  {
    wAux2 = (-INT16_MAX);
  }


  switch (pHandle->_Super.Sector)
  {
    case SECTOR_1:
    {
      if ((IA_OK | IC_OK) == (pHandle->iflag & (IA_OK | IC_OK))) /* iA and -iC are available to be sampled */
      {
        hCurrA = (int16_t)wAux2;
        wAux1 = -wAux1;
        /* hCurrC = (int16_t)wAux1 */
        tempCurent = (-wAux2 - wAux1);  /* (-hCurrA - hCurrC) */
        hCurrB = (int16_t)((tempCurent <= (-INT16_MAX)) ? (-INT16_MAX) :
                 ((tempCurent >= INT16_MAX) ? INT16_MAX : tempCurent));
      }
      else
      {
        if ((pHandle->iflag & (IA_OK | IC_OK)) != 0U) /* iA or -iC is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position     Aligning_angle=30 degree */
          {
            if (IA_OK == (pHandle->iflag & (IA_OK | IC_OK))) /* iA is available to be sampled and not iC */
            {
              hCurrA = (int16_t)wAux2;
              hCurrB = 0;
              /* hCurrC = -hCurrA */
            }
            else   /* 0x04 -ic is available */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t)wAux1;
              hCurrA = -hCurrC;
              hCurrB = 0;
            }
          }
          else  /* not START Position */
          {
            if (IA_OK == (pHandle->iflag & (IA_OK | IC_OK))) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t)wAux2;
              hCurrB = pHandle->_Super.IbEst;
            }
            else   /* 0x04 -ic is available */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t)wAux1;
              hCurrB = pHandle->_Super.IbEst; 
              hCurrA = -hCurrB-hCurrC;
            }
          }
        }
        else
        {
          hCurrA = pHandle->_Super.IaEst; 
          hCurrC = pHandle->_Super.IcEst;
          hCurrB = -hCurrA-hCurrC;
        }
      }
      break;
    }
    
    case SECTOR_2:
    {
      if ((IB_OK | IC_OK) == (pHandle->iflag & (IB_OK | IC_OK))) /* iB,-iC are available to be sampled */
      {
        hCurrB = (int16_t)wAux2;
        wAux1 = -wAux1;
        /* hCurrC = (int16_t)wAux1 */
        tempCurent = (-wAux2 - wAux1);  /* (-hCurrB - hCurrC) */
        hCurrA = (int16_t)((tempCurent <= (-INT16_MAX)) ? (-INT16_MAX) :
                 ((tempCurent >= INT16_MAX) ? INT16_MAX : tempCurent));
      }
      else
      {
        if((pHandle->iflag & (IB_OK | IC_OK)) != 0U) /* iB, or -iC is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position     Aligning_angle=90 degree */
          {
            if (IB_OK == (pHandle->iflag & (IB_OK | IC_OK))) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t)wAux2;
              hCurrA = 0;
            }
            else   /* 0x04 -ic */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t)wAux1;
              hCurrA = 0;
              hCurrB = -hCurrC;
            }
          }
          else  /* not START Position */
          {
            if (IB_OK == (pHandle->iflag & (IB_OK | IC_OK))) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t)wAux2;
              hCurrA = pHandle->_Super.IaEst;
            }
            else   /* 0x04 -ic */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t)wAux1;
              hCurrA = pHandle->_Super.IaEst;
              hCurrB = -hCurrA-hCurrC;
            }
          }
        }
        else
        {
          hCurrB = pHandle->_Super.IbEst;
          hCurrC = pHandle->_Super.IcEst;
          hCurrA = -hCurrB-hCurrC;
        }
      }
      break;
    }
    
    case SECTOR_3:
    {
      if ((IA_OK | IB_OK) == (pHandle->iflag & (IA_OK | IB_OK))) /* iB,-iA are available to be sampled */
      {
        hCurrB = (int16_t)wAux2;
        wAux1 = -wAux1;
        hCurrA = (int16_t)wAux1;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IB_OK)) != 0U) /* iB, or -iA is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position    Aligning_angle=150 degree */
          {
            if (IB_OK == (pHandle->iflag & (IA_OK | IB_OK))) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t)wAux2;
              hCurrA = -hCurrB;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t)wAux1;
              hCurrB = -hCurrA;
            }
          }
          else  /* not START Position */
          {
            if (IB_OK == (pHandle->iflag & (IA_OK | IB_OK))) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t)wAux2;
              hCurrA = pHandle->_Super.IaEst;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t)wAux1;
              hCurrB = pHandle->_Super.IbEst;
            }
          }
        }
        else
        {
          hCurrB = pHandle->_Super.IbEst; 
          hCurrA = pHandle->_Super.IaEst;
        }
      }
      break;
    }
    
    case SECTOR_4:
    {
      if ((IA_OK | IC_OK) == (pHandle->iflag & (IA_OK | IC_OK))) /* iC,-iA are available to be sampled */
      {
        /* hCurrC = (int16_t)wAux2 */
        wAux1 = -wAux1;
        hCurrA = (int16_t)wAux1;
        tempCurent = (-wAux1 - wAux2);  /* (-hCurrA - hCurrC) */
        hCurrB = (int16_t)((tempCurent <= (-INT16_MAX)) ? (-INT16_MAX) :
                 ((tempCurent >= INT16_MAX) ? INT16_MAX : tempCurent));
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IC_OK)) != 0U) /* iC, or -iA is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position     Aligning_angle=210 degree */
          {
            if (IC_OK == (pHandle->iflag & (IA_OK | IC_OK))) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t)wAux2;
              hCurrA = -hCurrC;
              hCurrB = 0;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t)wAux1;
              hCurrB = 0;
            }
          }
          else  /* not START Position */
          {
            if (IC_OK == (pHandle->iflag & (IA_OK | IC_OK))) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t)wAux2;
              hCurrB = pHandle->_Super.IbEst;
              hCurrA = -hCurrB-hCurrC;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t)wAux1;
              hCurrB = pHandle->_Super.IbEst;
            }
          }
        }
        else
        {
          hCurrC = pHandle->_Super.IcEst;
          hCurrA = pHandle->_Super.IaEst;
          hCurrB = -hCurrA-hCurrC;
        }
      }
      break;
    }
    
    case SECTOR_5:
    {
      if ((IB_OK | IC_OK) == (pHandle->iflag & (IB_OK | IC_OK))) /* iC,-iB are available to be sampled */
      {
        /* hCurrC = (int16_t)wAux2 */
        wAux1 = -wAux1;
        hCurrB = (int16_t)wAux1;
        tempCurent = (-wAux1 - wAux2);  /* (-hCurrB - hCurrC) */
        hCurrA = (int16_t)((tempCurent <= (-INT16_MAX)) ? (-INT16_MAX) :
                 ((tempCurent >= INT16_MAX) ? INT16_MAX : tempCurent));
      }
      else
      {
        if ((pHandle->iflag & (IB_OK | IC_OK)) != 0U) /* iC, or -iB is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position     Aligning_angle=270 degree */
          {
            if (IC_OK == (pHandle->iflag & (IB_OK | IC_OK))) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t) wAux2;
              hCurrA = 0;
              hCurrB = -hCurrC;
            }
            else  /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t)wAux1;
              hCurrA = 0;
            }
          }
          else  /* not START Position */
          {
            if (IC_OK == (pHandle->iflag & (IB_OK | IC_OK))) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t)wAux2;
              hCurrA = pHandle->_Super.IaEst;
              hCurrB = -hCurrA-hCurrC;
            }
            else /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t)wAux1;
              hCurrA = pHandle->_Super.IaEst;
            }
          }
        }
        else
        {
          hCurrC = pHandle->_Super.IcEst; 
          hCurrB = pHandle->_Super.IbEst;
          hCurrA = -hCurrB-hCurrC;
        }
      }
      break;
    }
    
    case SECTOR_6:
    {
      if ((IA_OK | IB_OK) == (pHandle->iflag & (IA_OK | IB_OK))) /* iA,-iB are available to be sampled */
      {
        hCurrA = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrB = (int16_t) wAux1;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IB_OK)) != 0U) /* iA, or -iB is available to be sampled */
        {
          if (1U == pHandle->_Super.AlignFlag) /* START Position     Aligning_angle=330 degree */
          {
            if (IA_OK == (pHandle->iflag & (IA_OK | IB_OK))) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t)wAux2;
              hCurrB = -hCurrA;
            }
            else  /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t)wAux1;
              hCurrA = -hCurrB;
            }
          }
          else  /* not START Position */
          {
            if (IA_OK == (pHandle->iflag & (IA_OK | IB_OK))) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t)wAux2;
              hCurrB = pHandle->_Super.IbEst;
            }
            else  //0x02 -ib
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t)wAux1;
              hCurrA = pHandle->_Super.IaEst;
            }
          }
        }
        else
        {
          hCurrA = pHandle->_Super.IaEst; 
          hCurrB = pHandle->_Super.IbEst;
        }
      }
      break;
    }   
      
    default:
      break;
  }
  
  pHandle->CurrAOld = hCurrA;
  pHandle->CurrBOld = hCurrB;

  if (NULL == pStator_Currents)
  {
    /* Nothing to do */
  }
  else
  {
    pStator_Currents->a = hCurrA;
    pStator_Currents->b = hCurrB;
  }
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during polarization.
  * 
  * It sums up injected conversion data into PhaseOffset to compute the
  * offset introduced in the current feedback network. It is required 
  * to properly configure ADC inputs before enabling the offset computation.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
static void R1_HFCurrentsCalibration(PWMC_Handle_t *pHdl, ab_t *pStator_Currents)
{
  /* Derived class members container */
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;  
<#if HAS_ADC_INJ == true>  
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
</#if>

  /* clear flag used for FOC duration check */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Disabling the External triggering for ADCx */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO_RESET);

  if (pHandle->Index < NB_CONVERSIONS)
  {
<#if HAS_ADC_INJ == false>
    pHandle->PhaseOffset += pHandle->CurConv[1] ${currentFactor};
<#else>    
    pHandle->PhaseOffset += ADCx->JDR2 ${currentFactor};
</#if>    
    pHandle->Index++;
  }

  if (NULL == pStator_Currents)
  {
    /* Nothing to do */
  }
  else
  {
    /* during offset calibration no current is flowing in the phases */
    pStator_Currents->a = 0;
    pStator_Currents->b = 0;
  }
}

/**
  * @brief  Configures the ADC for the current sampling during calibration.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_2_WriteTIMRegisters.
  */
static uint16_t R1_SetADCSampPointPolarization(PWMC_Handle_t *pHdl)
{
  /* Derived class members container */
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  uint16_t hAux;
  pHandle->CntSmp1 = ((pHandle->Half_PWMPeriod) >> 1U)
                    - (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((pHandle->Half_PWMPeriod) >> 1U)
                    + (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  <#if CondFamily_STM32G0	>

  /* reset ADC channel to be sample */
  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->IChannel));
  /* ADC trigger */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
</#if>  
  
  LL_TIM_OC_SetCompareCH5( TIMx, ( uint32_t )pHandle->CntSmp1 );
  LL_TIM_OC_SetCompareCH6( TIMx, ( uint32_t )pHandle->CntSmp2 );


  /*check software error*/
  if (0U == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    hAux = MC_NO_ERROR;
  }
  else
  {
    hAux = MC_DURATION;
  }
  if (1U == pHandle->_Super.SWerror)
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0U;
  }
  else
  {
    /* Nothing to do */
  }  
  return (hAux);
}

/**
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called on each motor start-up when using high voltage drivers.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,ticks);
  LL_TIM_OC_SetCompareCH2(TIMx,ticks);
  LL_TIM_OC_SetCompareCH3(TIMx,ticks);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Wait until next update */
  while ((uint32_t)RESET == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if (ES_GPIO == (pHandle->pParams_str->LowSideOutputs))
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
  pHandle->CntSmp1 = ((pHandle->Half_PWMPeriod) >> 1)
                    - (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((pHandle->Half_PWMPeriod) >> 1)
                    + (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
<#if CondFamily_STM32G4 == true>   

  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
</#if>    
  pHandle->_Super.TurnOnLowSidesAction = false;

/* Set all duty to 50% */
  uint16_t tempValue = pHandle->Half_PWMPeriod >> 1U;
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)tempValue);
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)tempValue);
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)tempValue); 
  LL_TIM_OC_SetCompareCH5(TIMx, (uint32_t)pHandle->CntSmp1);
  LL_TIM_OC_SetCompareCH6(TIMx, (uint32_t)pHandle->CntSmp2);
     
  /* TIM output trigger 2 for ADC */  
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if (ES_GPIO == (pHandle->pParams_str->LowSideOutputs))
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
   
  /* wait for second half PWM cycle to enable dma request */
  if (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
  {
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }
  else
  {
    while (LL_TIM_COUNTERDIRECTION_DOWN == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is downcounting */
    }
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }

  
  /* at this point we are down counting */
  
  /* clear peripheral flags */
  LL_DMA_ClearFlag_TC(DMAx, pHandle->pParams_str->DMAChannelX);
<#if CondFamily_STM32F7 == false>   
  LL_DMA_ClearFlag_GI1(DMAx);    
</#if>   
  /* clear peripheral flags */
  LL_TIM_ClearFlag_UPDATE(TIMx);
<#if HAS_ADC_INJ == true>
  LL_ADC_ClearFlag_JEOS(ADCx);
</#if>  


  pHandle->TCCnt = 0;
  pHandle->TCDoneFlag = false;

  /* start DMA that modifies CC register of CH1,2 and 3 in order to create the
     phase shifting */ 
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH);
  LL_DMA_Enable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMAChannelX);  

  /* enable DMA trigger: TIMx CC channel 4 */
  LL_TIM_OC_SetCompareCH4(TIMx, 0UL);
  LL_TIM_EnableDMAReq_CC4(TIMx);
 
<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4>       
  /*start injected conversion */
  LL_ADC_INJ_StartConversion(ADCx);
<#elseif CondFamily_STM32F7>
#if (defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx))
  /* workaround software to fix hardware bug on STM32F4xx and STM32F5xx
   * set trigger detection to rising and falling edges to be able to
   * trigger ADC on TRGO2
   * refer to errata sheet ES0290 */
  LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISINGFALLING);
#else
  LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
#endif
</#if>

<#if HAS_ADC_INJ == false>
  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, 2U);
  LL_DMA_Enable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);  
  LL_ADC_REG_SetSequencerChannels (ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL (pHandle->pParams_str->IChannel));
  LL_ADC_REG_SetTriggerSource (ADCx, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_REG_SetDMATransfer(ADCx, LL_ADC_REG_DMA_TRANSFER_LIMITED);  
  LL_ADC_REG_StartConversion (ADCx); 
  pHandle->ADCRegularLocked=true;
</#if>
  /* enable peripheral interrupt */
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_DMA_EnableIT_HT(DMAx, pHandle->pParams_str->DMAChannelX);  
<#if HAS_ADC_INJ == true >    
  LL_ADC_EnableIT_JEOS(ADCx);  
<#else >   
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX); 
</#if>

}


/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;   
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
  pHandle->CntSmp1 = ((pHandle->Half_PWMPeriod) >> 1)
                    - (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((pHandle->Half_PWMPeriod) >> 1)
                    + (pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);

  /* This allow to control cycles during next sequence */
  LL_TIM_DisableIT_UPDATE(TIMx); 
  
  /* synchronize the disable of the DMA and all settings with the douwn counting */
  if (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
  {
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }
  else
  {
    while (LL_TIM_COUNTERDIRECTION_DOWN == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is downcounting */
    }
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  } 

  LL_DMA_DisableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);  
  LL_DMA_DisableIT_HT(DMAx, pHandle->pParams_str->DMAChannelX);      
   
  <#if CondFamily_STM32G0>
  /* Switch off the DMA from this point High frequency task is shut down*/
  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  LL_DMA_DisableIT_TC(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  </#if>  
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableDMAReq_CC4(TIMx);
  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMAChannelX);
    
  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if (true == pHandle->BrakeActionLock)
  {
    /* Nothing to do */
  }
  else
  {
    if (ES_GPIO == (pHandle->pParams_str->LowSideOutputs))
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  

<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4> 
  /* Stop ADC injected conversion */
  LL_ADC_INJ_StopConversion(ADCx);
<#elseif CondFamily_STM32F7> 
  /* Stop ADC injected conversion */
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
<#else>
  /*Clear potential ADC Ongoing conversion*/
  if (LL_ADC_REG_IsConversionOngoing(ADCx))
  {
    LL_ADC_REG_StopConversion(ADCx);
    while (LL_ADC_REG_IsConversionOngoing(ADCx))
    {
      /* Nothing to do */
    }
  }

  LL_ADC_REG_SetTriggerSource(ADCx, LL_ADC_REG_TRIG_SOFTWARE);    
</#if>  
  
  R1_1ShuntMotorVarsInit(&pHandle->_Super);
<#if CondFamily_STM32G4 || !HAS_ADC_INJ >     

 /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false;
   
</#if>    
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Contains the TIMx Break2 event interrupt.
  * 
  * @param pHandle: Handler of the current instance of the PWM component.
  */
__weak void *R1_BRK2_IRQHandler(PWMC_R1_Handle_t *pHandle )
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
<#if CondFamily_STM32G0 == false>
    if (false == pHandle->BrakeActionLock)
    {
      if (ES_GPIO == (pHandle->pParams_str->LowSideOutputs))
      {
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
    pHandle->OverCurrentFlag = true;
<#else>  
    pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    pHandle->OverVoltageFlag = true;
    pHandle->BrakeActionLock = true;
</#if> 
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  }
  return ((NULL == pHandle) ? NULL : &(pHandle->_Super.Motor));
#else
  return (&(pHandle->_Super.Motor));
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Contains the TIMx Break1 event interrupt.
  * 
  * @param pHandle: Handler of the current instance of the PWM component.
  */
__weak void *R1_BRK_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
<#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4>
    pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    pHandle->OverVoltageFlag = true;
    pHandle->BrakeActionLock = true;
<#elseif CondFamily_STM32F4 || CondFamily_STM32G0>
    if (false == pHandle->BrakeActionLock)
    {
      if (ES_GPIO == (pHandle->pParams_str->LowSideOutputs))
      {
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
  pHandle->OverCurrentFlag = true;
</#if>
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  }
  return ((NULL == pHandle) ? NULL : &(pHandle->_Super.Motor));
#else
  return (&(pHandle->_Super.Motor));
#endif
}

/**
  * @brief  Checks if an overcurrent occurred since last call.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns #MC_BREAK_IN if an overcurrent has been
  *                  detected since last method call, #MC_NO_FAULTS otherwise.
  */
__weak uint16_t R1_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  uint16_t retVal = MC_NO_FAULTS;

  if (true == pHandle->OverVoltageFlag)
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }

  if (true == pHandle->OverCurrentFlag)
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }

  return (retVal);
}

<#if CondFamily_STM32F3 || CondFamily_STM32G4>
<#if MC.INTERNAL_OVERCURRENTPROTECTION == true || MC.INTERNAL_OVERCURRENTPROTECTION2 == true || 
     MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true> 

/**
  * @brief  Configures the analog output used for protection thresholds.
  *
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected.
  *            @arg DAC_Channel_2: DAC Channel2 selected.
  * @param  DACx: DAC to be configured.
  * @param  hDACVref: Value of DAC reference expressed as 16bit unsigned integer. \n
  *         Ex. 0 = 0V ; 65536 = VDD_DAC.
  */
void R1_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{

  LL_DAC_ConvertData12LeftAligned(DAC1, DAC_Channel, hDACVref);
  LL_DAC_TrigSWConversion(DAC1, DAC_Channel);
  /* Enable DAC Channel */
  LL_DAC_Enable(DAC1, DAC_Channel);

}

</#if>
</#if>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t R1_CalcDutyCycles(PWMC_Handle_t *pHdl)
{
  static const uint8_t ALFLAG[3] = {IA_OK,IB_OK,IC_OK};
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;

  
  int16_t submax_mid;
  int16_t submax_mid_deltmin;
  int16_t submid_min;
  int16_t submid_min_deltmin;
  int16_t aCCRval[3];   /* CCR setting values */
  int16_t SamplePoint1;
  int16_t SamplePoint2;
<#if MC.DISCONTINUOUS_PWM == true || MC.DISCONTINUOUS_PWM2 == true>    
  int16_t stored_val;
</#if>  
  uint16_t hAux;
  uint8_t maxVal;
  uint8_t midVal;
  uint8_t minVal;
  uint8_t max_bad_flag;
  uint8_t min_bad_flag;

  aCCRval[0] = pHandle->_Super.CntPhA;
  aCCRval[1] = pHandle->_Super.CntPhB;
  aCCRval[2] = pHandle->_Super.CntPhC;
  
<#if MC.DISCONTINUOUS_PWM == true || MC.DISCONTINUOUS_PWM2 == true>     
  if (pHandle->_Super.DPWM_Mode == true)
  {
    /* Check if the stator flux localized in boundary regions */
    if ((aCCRval[pHandle->_Super.midDuty] - aCCRval[pHandle->_Super.lowDuty]) <= pHandle->pParams_str->TMin)
    {
      stored_val = (int16_t)pHandle->_Super.HighDutyStored;
      /* Bundary 2 and 3 */
      pHandle->_Super.CntPhA += (uint16_t)stored_val;
      pHandle->_Super.CntPhB += (uint16_t)stored_val;
      pHandle->_Super.CntPhC += (uint16_t)stored_val;
      
      aCCRval[0] = pHandle->_Super.CntPhA;
      aCCRval[1] = pHandle->_Super.CntPhB;
      aCCRval[2] = pHandle->_Super.CntPhC;
      
    }
    else 
    {
      /* Other cases nothing to do */
    }
  }
  else
  {
    /* DPWM is not used nothing to do */
  }
</#if>  


  
<#if  MC.OVERMODULATION == true >	
  //Sector dividing and setting the value index for max, mid and min
  if(aCCRval[0] >= aCCRval[1])  //A>=B
  {
    if(aCCRval[1] >= aCCRval[2]) //B>=C
    {
      //bSector = 1;
      maxVal = 0;
      midVal = 1;
      minVal = 2;      
    }
    else  //B<C
    {
      if(aCCRval[2] >= aCCRval[0])  //C>=A
      {
        //bSector = 5;
        maxVal = 2;
        midVal = 0;
        minVal = 1;  
      }
      else //C<A
      {
        //bSector = 6;
        maxVal = 0;
        midVal = 2;
        minVal = 1;          
      }
    }
    
  }
  else  //A<B
  {
    if(aCCRval[1] >= aCCRval[2]) //B>=C
    {
       if(aCCRval[2] >= aCCRval[0])  //C>=A
      {
        //bSector = 3;
        maxVal = 1;
        midVal = 2;
        minVal = 0;  
      }
      else //C<A
      {
       //bSector = 2;
        maxVal = 1;
        midVal = 0;
        minVal = 2;          
      }    
    }
    else  //B<C
    {
      //bSector = 4;
      maxVal = 2;
      midVal = 1;
      minVal = 0;      
    }
    
  }
<#else>
  maxVal =  (uint16_t)pHandle->_Super.highDuty;  
  midVal =  (uint16_t)pHandle->_Super.midDuty; 
  minVal =  (uint16_t)pHandle->_Super.lowDuty;
</#if>
  pHandle->iflag=0x00; 
  
  /* Phase-shift and set iflag */
  submax_mid = aCCRval[maxVal] - aCCRval[midVal];
  submax_mid_deltmin = submax_mid - (int16_t)pHandle->pParams_str->TMin; 
  submid_min = aCCRval[midVal] - aCCRval[minVal];
  submid_min_deltmin = submid_min - (int16_t)pHandle->pParams_str->TMin; 
  pHandle->aShiftval[0]=0;
  pHandle->aShiftval[1]=0;
  pHandle->aShiftval[2]=0; 
  max_bad_flag = 0;
  min_bad_flag = 0;
  
  if (submax_mid_deltmin > 0)     
  {
    pHandle->iflag |= ALFLAG[maxVal];
  }
  else                           
  {
    if ((1 - submax_mid_deltmin + aCCRval[maxVal] + (int16_t) pHandle->pParams_str->hTADConv)
       > (int16_t)(pHandle->Half_PWMPeriod))
    {
      pHandle->iflag &= ~ALFLAG[maxVal];
      max_bad_flag = 1U;
    }
    else
    {
      pHandle->iflag |= ALFLAG[maxVal];
      pHandle->aShiftval[maxVal] = 1U - (uint16_t)submax_mid_deltmin;
    }
  }
  
  if (submid_min_deltmin > 0)
  {
    pHandle->iflag |= ALFLAG[minVal];
  }
  else
  {
    if ((submid_min_deltmin - 1 + aCCRval[minVal]) < 0)
    {
      pHandle->iflag &= ~ALFLAG[minVal];
      min_bad_flag = 1;
    }
    else
    {
      pHandle->iflag |= ALFLAG[minVal];
      pHandle->aShiftval[minVal] = (uint16_t)submid_min_deltmin - 1U;
    }
  }
  
  if ((0U == max_bad_flag) && (0U == min_bad_flag))
  {
    SamplePoint1 = (int16_t)aCCRval[midVal] - (int16_t)pHandle->pParams_str->TSample;
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->TMin - (int16_t)pHandle->pParams_str->TSample;
  }
  else if ((1U == max_bad_flag) && (1U == min_bad_flag))
  {
    SamplePoint1 = (int16_t)pHandle->Half_PWMPeriod / 2;
    SamplePoint2 = SamplePoint1 + (int16_t)pHandle->pParams_str->TSample + (int16_t)pHandle->pParams_str->hTADConv;
  }
  else if (1U == max_bad_flag)
  {
    SamplePoint1 = aCCRval[midVal] - (int16_t)pHandle->pParams_str->TSample- (int16_t)pHandle->pParams_str->hTADConv;
    SamplePoint2 = aCCRval[midVal];
  }
  else
  {
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->hTADConv + (int16_t)pHandle->pParams_str->TMin 
                 - (int16_t)pHandle->pParams_str->TSample;
    SamplePoint1 = aCCRval[midVal];
  }
  
  if ((SamplePoint2-SamplePoint1) < (int16_t)pHandle->pParams_str->hTADConv)
  {
    SamplePoint1 = aCCRval[midVal]
                 - (((int16_t)(pHandle->pParams_str->hTADConv) - (int16_t)(pHandle->pParams_str->TMin)) / 2);   
    SamplePoint2 = aCCRval[midVal] + (int16_t)(pHandle->pParams_str->TMin) - (int16_t)(pHandle->pParams_str->TSample) 
                 + (((int16_t)(pHandle->pParams_str->hTADConv) - (int16_t)(pHandle->pParams_str->TMin)) / 2) + 1;
  }

  /* saturate sampling point */
  if ((SamplePoint2 >= (int16_t)(pHandle->Half_PWMPeriod)) || (SamplePoint2 <= 0))
  {
    SamplePoint2 = (int16_t)pHandle->Half_PWMPeriod - 1;
  }    
  if ((SamplePoint1 >= (int16_t)pHandle->Half_PWMPeriod) || (SamplePoint1 <= 0))
  {
    SamplePoint1 = (int16_t)pHandle->Half_PWMPeriod - 1;
  }  

  /* critical section start */
  LL_DMA_DisableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);

  pHandle->DmaBuffCCR_latch[0] = pHandle->_Super.CntPhA + pHandle->aShiftval[0];
  pHandle->DmaBuffCCR_latch[1] = pHandle->_Super.CntPhB + pHandle->aShiftval[1];
  pHandle->DmaBuffCCR_latch[2] = pHandle->_Super.CntPhC + pHandle->aShiftval[2];
    /*  second half PWM period CCR value transfered by DMA */
  pHandle->DmaBuffCCR_latch[4]= pHandle->_Super.CntPhA - pHandle->aShiftval[0];
  pHandle->DmaBuffCCR_latch[5]= pHandle->_Super.CntPhB - pHandle->aShiftval[1];
  pHandle->DmaBuffCCR_latch[6]= pHandle->_Super.CntPhC - pHandle->aShiftval[2];

  if (true == pHandle->TCDoneFlag)
  {
    /*  first half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[0] = pHandle->DmaBuffCCR_latch[0];
    pHandle->DmaBuffCCR[1] = pHandle->DmaBuffCCR_latch[1];
    pHandle->DmaBuffCCR[2] = pHandle->DmaBuffCCR_latch[2];
    /*  second half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[4]= pHandle->DmaBuffCCR_latch[4];
    pHandle->DmaBuffCCR[5]= pHandle->DmaBuffCCR_latch[5];
    pHandle->DmaBuffCCR[6]= pHandle->DmaBuffCCR_latch[6];
<#if CondFamily_STM32F7  >

    /* Disable and re-enable the stream in order to load the correct value (DMA configured in
      direct mode) */ 
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMAChannelX);
    while (true == LL_DMA_IsEnabledStream(DMAx, pHandle->pParams_str->DMAChannelX))
    {
      /* Nothing to do */
    }
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMAChannelX);
</#if>    
  }
  else
  {
    /* do nothing, it will be applied during DMA transfer complete IRQ */
  }
  /* critical section end */
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
<#if CondFamily_STM32G0	>

  /* reset ADC channel to be sample */
  LL_ADC_REG_SetSequencerChannels ( ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->IChannel ));
  /* ADC trigger */
  LL_TIM_SetTriggerOutput2( TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING );
</#if>  

  LL_TIM_OC_SetCompareCH5(TIMx, ( uint32_t )SamplePoint1);
  LL_TIM_OC_SetCompareCH6(TIMx, ( uint32_t )SamplePoint2);
  
  /*check software error*/
  if (0U == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    hAux = MC_NO_ERROR;
  }
  else
  {
    hAux = MC_DURATION;
  }
  if (1U == pHandle->_Super.SWerror)
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0U;
  }  
  
  return (hAux);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void *R1_TIMx_UP_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  return ((NULL == pHandle) ? NULL : (&( pHandle->_Super.Motor)));
#else
  return (&( pHandle->_Super.Motor));
#endif
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Contains the motor DMAx TC interrupt request. 
  *
  * Required only for R1 with rep rate > 1.
  *
  * @param pHdl: Handler of the current instance of the PWM component.
  */
__weak void *R1_DMAx_TC_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
<#if CondFamily_STM32G0  == false>    
    TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
</#if>

    LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMAChannelX);
    pHandle->TCCnt++;
    if (pHandle->TCCnt == pHandle->pParams_str->RepetitionCounter)
    {
      pHandle->DmaBuffCCR[0] = pHandle->DmaBuffCCR_latch[0];
      pHandle->DmaBuffCCR[1] = pHandle->DmaBuffCCR_latch[1];
      pHandle->DmaBuffCCR[2] = pHandle->DmaBuffCCR_latch[2];
      
      /* second half PWM period CCR value transfered by DMA*/
      pHandle->DmaBuffCCR[4]= pHandle->DmaBuffCCR_latch[4];
      pHandle->DmaBuffCCR[5]= pHandle->DmaBuffCCR_latch[5];
      pHandle->DmaBuffCCR[6]= pHandle->DmaBuffCCR_latch[6];

<#if CondFamily_STM32F7  >
      LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH);
      LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMAChannelX);
</#if>
      
      pHandle->TCCnt = 0;
      pHandle->TCDoneFlag =true;
    
<#if CondFamily_STM32G0 == false  >     
      /* ADC trigger */
      LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
</#if>      
    }
    else
    {
<#if CondFamily_STM32F7>
      LL_DMA_DisableStream( DMAx, pHandle->pParams_str->DMAChannelX);
      LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMAChannelX);      
      LL_DMA_SetDataLength( DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH);
      LL_DMA_EnableStream( DMAx, pHandle->pParams_str->DMAChannelX);  
</#if>  
    }
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  }
  
  return ((NULL == pHandle) ? NULL : &(pHandle->_Super.Motor));
#else
  return (&(pHandle->_Super.Motor));
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Contains the motor DMAx HT interrupt request.
  *
  * Required only for R1 with rep rate > 1.
  *
  * @param pHdl: Handler of the current instance of the PWM component.
  */
__weak void *R1_DMAx_HT_IRQHandler(PWMC_R1_Handle_t *pHandle )
{
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif  
<#if CondFamily_STM32G0  >    
    TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
    /* ADC trigger */
    LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
</#if>

    if  (pHandle->TCDoneFlag ==true)
    {  
<#if CondFamily_STM32G0  >
      LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
      LL_ADC_REG_StartConversion (ADC1); 
   
</#if>  
      pHandle->TCDoneFlag = false;
    }
#ifdef NULL_PTR_R1_PS_PWR_CUR_FDB
  }
  return ((NULL == pHandle) ? NULL : &(pHandle->_Super.Motor));
#else
  return (&(pHandle->_Super.Motor));
#endif
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
