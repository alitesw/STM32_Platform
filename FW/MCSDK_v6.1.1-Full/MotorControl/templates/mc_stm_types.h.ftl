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
<#-- Condition for STM32F302x8x MCU -->
<#assign CondMcu_STM32F302x8x = (McuName?? && McuName?matches("STM32F302.8.*"))>
<#-- Condition for STM32F072xxx MCU -->
<#assign CondMcu_STM32F072xxx = (McuName?? && McuName?matches("STM32F072.*"))>
<#-- Condition for STM32F0 Family -->
<#assign CondFamily_STM32F0 = (FamilyName?? && FamilyName=="STM32F0")>
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
<#-- Condition for STM32F7 Family -->
<#assign CondFamily_STM32H7 = (FamilyName?? && FamilyName == "STM32H7") >
<#-- Condition for STM32G0 Family -->
<#assign CondFamily_STM32G0 = (FamilyName?? && FamilyName == "STM32G0") >

<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
<#assign DMAStream = CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7 >
/**
  ******************************************************************************
  * @file    mc_stm_types.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Includes HAL/LL headers relevant to the current configuration.
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
#ifndef MC_STM_TYPES_H
#define MC_STM_TYPES_H

#ifdef FULL_MISRA_C_COMPLIANCY
#define FULL_MISRA_C_COMPLIANCY_ENC_SPD_POS
#define FULL_MISRA_C_COMPLIANCY_FWD_FDB
#define FULL_MISRA_C_COMPLIANCY_FLUX_WEAK
#define FULL_MISRA_C_COMPLIANCY_MAX_TOR
#define FULL_MISRA_C_COMPLIANCY_STO_CORDIC
#define FULL_MISRA_C_COMPLIANCY_NTC_TEMP
#define FULL_MISRA_C_COMPLIANCY_PID_REGULATOR
#define FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
#define FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
#define FULL_MISRA_C_COMPLIANCY_STO_PLL
#define FULL_MISRA_C_COMPLIANCY_VIRT_SPD_SENS
#define FULL_MISRA_C_COMPLIANCY_MC_MATH
#define FULL_MISRA_C_COMPLIANCY_PFC
#define FULL_MISRA_C_COMPLIANCY_PWM_CURR
#endif

#ifdef NULL_PTR_CHECK
#define NULL_PTR_CHECK_BUS_VOLT
#define NULL_PTR_CHECK_CRC_LIM
#define NULL_PTR_CHECK_DIG_OUT
#define NULL_PTR_CHECK_ENC_ALI_CTRL
#define NULL_PTR_CHECK_ENC_SPD_POS_FDB
#define NULL_PTR_CHECK_FEED_FWD_CTRL
#define NULL_PTR_CHECK_FLUX_WEAK
#define NULL_PTR_CHECK_HALL_SPD_POS_FDB
#define NULL_PTR_CHECK_MAX_TRQ_PER_AMP
#define NULL_PTR_CHECK_MCP
#define NULL_PTR_CHECK_MCPA
#define NULL_PTR_CHECK_MOT_POW_MES
#define NULL_PTR_CHECK_NTC_TEMP_SENS
#define NULL_PTR_CHECK_OPEN_LOOP
#define NULL_PTR_CHECK_PID_REG
#define NULL_PTR_MOT_POW_MEAS
#define NULL_PTR_POW_COM
#define NULL_PTR_PWM_CUR_FDB_OVM
#define NULL_PTR_RDIV_BUS_VLT_SNS
#define NULL_PTR_REV_UP_CTL
#define NULL_PTR_SPD_POS_FBK
#define NULL_PTR_SPD_TRQ_CTL
#define NULL_PTR_STA_MCH
#define NULL_PTR_STO_COR_SPD_POS_FDB
#define NULL_PTR_STO_PLL_SPD_POS_FDB
#define NULL_PTR_VIR_SPD_SEN
#define NULL_PTR_R3_G4X_PWM_CUR_FDB
#define NULL_PTR_ASP
#define NULL_PTR_DAC_UI
#define NULL_PTR_MC_INT_ACM
#define NULL_PTR_MC_INT
#define NULL_PTR_PWR_CUR_FDB
#define NULL_PTR_R1_PS_PWR_CUR_FDB
#define NULL_PTR_REG_INT
#define NULL_PTR_REG_CON_MNG
#define NULL_PTR_STL_MNG
#define NULL_PTR_USA_ASP_DRV
#define NULL_PTR_POT
#define NULL_PTR_SPD_POT
#endif


#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#ifdef MISRA_C_2004_BUILD
#error "The code is not ready for that..."
#endif

<#if CondFamily_STM32F4>
  #include "stm32f4xx_ll_system.h"
  #include "stm32f4xx_ll_adc.h"
  #include "stm32f4xx_ll_tim.h"
  #include "stm32f4xx_ll_gpio.h"
  #include "stm32f4xx_ll_usart.h"
  #include "stm32f4xx_ll_dac.h"
  #include "stm32f4xx_ll_dma.h"
  #include "stm32f4xx_ll_bus.h"
<#elseif CondFamily_STM32F7>
  #include "stm32f7xx_ll_system.h"
  #include "stm32f7xx_ll_adc.h"
  #include "stm32f7xx_ll_tim.h"
  #include "stm32f7xx_ll_gpio.h"
  #include "stm32f7xx_ll_usart.h"
  #include "stm32f7xx_ll_dac.h"
  #include "stm32f7xx_ll_dma.h"
  #include "stm32f7xx_ll_bus.h"
<#elseif CondFamily_STM32H7>
  #include "stm32h7xx_ll_bus.h"
  #include "stm32h7xx_ll_rcc.h"
  #include "stm32h7xx_ll_system.h"
  #include "stm32h7xx_ll_adc.h"
  #include "stm32h7xx_ll_tim.h"
  #include "stm32h7xx_ll_gpio.h"
  #include "stm32h7xx_ll_usart.h"
  #include "stm32h7xx_ll_dac.h"
  #include "stm32h7xx_ll_dma.h"
  #include "stm32h7xx_ll_comp.h"
  #include "stm32h7xx_ll_opamp.h"
<#elseif CondFamily_STM32L4>
  #include "stm32l4xx_ll_system.h"
  #include "stm32l4xx_ll_adc.h"
  #include "stm32l4xx_ll_tim.h"
  #include "stm32l4xx_ll_gpio.h"
  #include "stm32l4xx_ll_usart.h"
  #include "stm32l4xx_ll_dac.h"
  #include "stm32l4xx_ll_dma.h"
  #include "stm32l4xx_ll_bus.h"
  #include "stm32l4xx_ll_comp.h"
  #include "stm32l4xx_ll_opamp.h"
<#elseif CondFamily_STM32F0>
  #include "stm32f0xx_ll_bus.h"
  #include "stm32f0xx_ll_rcc.h"
  #include "stm32f0xx_ll_system.h"
  #include "stm32f0xx_ll_adc.h"
  #include "stm32f0xx_ll_tim.h"
  #include "stm32f0xx_ll_gpio.h"
  #include "stm32f0xx_ll_usart.h"
  #include "stm32f0xx_ll_dac.h"
  #include "stm32f0xx_ll_dma.h"
  #include "stm32f0xx_ll_comp.h"
<#elseif CondFamily_STM32F3>
  #include "stm32f3xx_ll_bus.h"
  #include "stm32f3xx_ll_rcc.h"
  #include "stm32f3xx_ll_system.h"
  #include "stm32f3xx_ll_adc.h"
  #include "stm32f3xx_ll_tim.h"
  #include "stm32f3xx_ll_gpio.h"
  #include "stm32f3xx_ll_usart.h"
  #include "stm32f3xx_ll_dac.h"
  #include "stm32f3xx_ll_dma.h"
  #include "stm32f3xx_ll_comp.h"
  #include "stm32f3xx_ll_opamp.h"
  #include "stm32f3xx_ll_spi.h"
<#elseif CondFamily_STM32G4>
  #include "stm32g4xx_ll_bus.h"
  #include "stm32g4xx_ll_rcc.h"
  #include "stm32g4xx_ll_system.h"
  #include "stm32g4xx_ll_adc.h"
  #include "stm32g4xx_ll_tim.h"
  #include "stm32g4xx_ll_gpio.h"
  #include "stm32g4xx_ll_usart.h"
  #include "stm32g4xx_ll_dac.h"
  #include "stm32g4xx_ll_dma.h"
  #include "stm32g4xx_ll_comp.h"
  #include "stm32g4xx_ll_opamp.h"
  #include "stm32g4xx_ll_cordic.h"
<#elseif CondFamily_STM32G0>
  #include "stm32g0xx_ll_bus.h"
  #include "stm32g0xx_ll_rcc.h"
  #include "stm32g0xx_ll_system.h"
  #include "stm32g0xx_ll_adc.h"
  #include "stm32g0xx_ll_tim.h"
  #include "stm32g0xx_ll_gpio.h"
  #include "stm32g0xx_ll_usart.h"
  #include "stm32g0xx_ll_dac.h"
  #include "stm32g0xx_ll_dma.h"
  #include "stm32g0xx_ll_comp.h"
<#else>
  #error "No MCU selected"
</#if>

<#if DMAStream >
/**
  * @brief  Driver macro reserved for internal use: set a pointer to
  *         a register from a register basis from which an offset
  *         is applied.
  * @param  __REG__ Register basis from which the offset is applied.
  * @param  __REG_OFFFSET__ Offset to be applied (unit number of registers).
  * @retval Pointer to register address
  */
#define __DMA_PTR_REG_OFFSET(__REG__, __REG_OFFFSET__)                         \
 ((__IO uint32_t *)((uint32_t) ((uint32_t)(&(__REG__)) + ((__REG_OFFFSET__) << 2U))))
   
#define __LL_DMA_IT_TC_BIT(__STREAM_NB__) \
  (((__STREAM_NB__&0x3) == 0U) ? 5 : \
   ((__STREAM_NB__&0x3) == 1U) ? 11 :\
   ((__STREAM_NB__&0x3) == 2U) ? 21 : 27) 

#define __LL_DMA_IT_HT_BIT(__STREAM_NB__) \
  (((__STREAM_NB__&0x3) == 0U) ? 4 : \
   ((__STREAM_NB__&0x3) == 1U) ? 10 :\
   ((__STREAM_NB__&0x3) == 2U) ? 20 : 26)

#define __LL_DMA_IT_TE_BIT(__STREAM_NB__) \
  (((__STREAM_NB__&0x3) == 0U) ? 3 : \
   ((__STREAM_NB__&0x3) == 1U) ? 9 :\
   ((__STREAM_NB__&0x3) == 2U) ? 19 : 25)     
   
__STATIC_INLINE void LL_DMA_ClearFlag_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  if (NULL == DMAx)
  {
    /* Nothing to do */
  }
  else
  {
    /* Clear TC bits with bits position depending on parameter "Stream".      */  
    register __IO uint32_t *preg = __DMA_PTR_REG_OFFSET(DMAx->LIFCR,(Stream<=3)?0:1);
  
    WRITE_REG (*preg, 1 << __LL_DMA_IT_TC_BIT(Stream) );
  }
}

__STATIC_INLINE void LL_DMA_ClearFlag_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  if (NULL == DMAx)
  {
    /* Nothing to do */
  }
  else
  {
    /* Clear TE bits with bits position depending on parameter "Stream".      */
    register __IO uint32_t *preg = __DMA_PTR_REG_OFFSET(DMAx->LIFCR,(Stream<=3)?0:1);

    WRITE_REG (*preg, 1 << __LL_DMA_IT_TE_BIT(Stream) );
  }
}

__STATIC_INLINE uint32_t LL_DMA_IsActiveFlag_TC(DMA_TypeDef *DMAx, uint32_t Stream )
{
  uint32_t retVal;
  if (NULL == DMAx)
  {
    retVal = 0;
  }
  else
  {
    register __IO uint32_t *preg = __DMA_PTR_REG_OFFSET(DMAx->LISR,(Stream<=3)?0:1);
    retVal = ((READ_BIT(*preg, 1 << __LL_DMA_IT_TC_BIT(Stream) ) == (1 << __LL_DMA_IT_TC_BIT(Stream))) ? 1UL : 0UL);
  }
  return (retVal);
}

__STATIC_INLINE void LL_DMA_ClearFlag_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  if (NULL == DMAx)
  {
    /* Nothing to do */
  }
  else
  {
    /* Clear TC bits with bits position depending on parameter "Stream".      */  
    register __IO uint32_t *preg = __DMA_PTR_REG_OFFSET(DMAx->LIFCR,(Stream<=3)?0:1);
  
    WRITE_REG (*preg, 1 << __LL_DMA_IT_HT_BIT(Stream) );
  }
}

__STATIC_INLINE uint32_t LL_DMA_IsActiveFlag_HT(DMA_TypeDef *DMAx, uint32_t Stream )
{
  uint32_t retVal;
  if (NULL == DMAx)
  {
     retVal = 0;
  }
  else
  {
    register __IO uint32_t *preg = __DMA_PTR_REG_OFFSET(DMAx->LISR,(Stream<=3)?0:1);
    retVal = ((READ_BIT(*preg, 1 << __LL_DMA_IT_HT_BIT(Stream) ) == (1 << __LL_DMA_IT_HT_BIT(Stream))) ? 1UL : 0UL);
  }
  return (retVal);
}

<#else>
__STATIC_INLINE void LL_DMA_ClearFlag_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  if (NULL == DMAx)
  {
    /* Nothing to do */
  }
  else
  {
    /* Clear TC bits with bits position depending on parameter "Channel".      */
    WRITE_REG (DMAx->IFCR, DMA_IFCR_CTCIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2));
  }
}

__STATIC_INLINE uint32_t LL_DMA_IsActiveFlag_TC(DMA_TypeDef *DMAx, uint32_t Channel )
{
  return ((NULL == DMAx) ? 0U : ((READ_BIT(DMAx->ISR, (DMA_ISR_TCIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2) )) == (DMA_ISR_TCIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2))) ? 1UL : 0UL));
}
__STATIC_INLINE void LL_DMA_ClearFlag_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  if (NULL == DMAx)
  {
    /* Nothing to do */
  }
  else
  {
    /* Clear HT bits with bits position depending on parameter "Channel".      */
    WRITE_REG (DMAx->IFCR, DMA_IFCR_CHTIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2));
  }
}

__STATIC_INLINE uint32_t LL_DMA_IsActiveFlag_HT(DMA_TypeDef *DMAx, uint32_t Channel )
{
 return ((NULL == DMAx) ? 0U : ((READ_BIT(DMAx->ISR, (DMA_ISR_HTIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2) )) == (DMA_ISR_HTIF1 << ((Channel-LL_DMA_CHANNEL_1)<<2))) ? 1UL : 0UL));
}
</#if>


<#if CondFamily_STM32F0 || CondFamily_STM32G0>
  #define CIRCLE_LIMITATION_SQRT_M0
</#if>

<#if CondFamily_STM32G4>
#define PIN_CONNECT (uint32_t)(0)
#define DIRECT_CONNECT (uint32_t)(OPAMP_CSR_OPAMPINTEN)
#define OPAMP_UNCHANGED (uint32_t)(0xFFFFFFFFUL)
#define VPOPAMP3 18
</#if>

<#if CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32L4>
//#define ADC_INJ_TRIG_TIMER LL_ADC_INJ_TRIG_EXT_${_last_word(MC.PWM_TIMER_SELECTION)}_TRGO
</#if>



/**
 * @name Predefined Speed Units
 *
 * Each of the following symbols defines a rotation speed unit that can be used by the 
 * functions of the API for their speed parameter. Each Unit is defined by expressing 
 * the value of 1 Hz in this unit.
 *
 * These symbols can be used to set the #SPEED_UNIT macro which defines the rotation speed
 * unit used by the functions of the API.
 *
 * @anchor SpeedUnit
 */
/** @{ */
/** Revolutions Per Minute: 1 Hz is 60 RPM */
#define U_RPM 60
/** Tenth of Hertz: 1 Hz is 10 01Hz */
#define U_01HZ 10
/* Hundreth of Hertz: 1 Hz is 100 001Hz */
/* #define _001HZ 100 */
/** @} */ 

/* USER CODE BEGIN DEFINITIONS */
/* Definitions placed here will not be erased by code generation */
/**
 * @brief Rotation speed unit used at the interface with the application 
 *
 * This symbols defines the value of 1 Hertz in the unit used by the functions of the API for 
 * their speed parameters. 
 *
 * For instance, if the chosen unit is the RPM, SPEED_UNIT is defined to 60, since 1 Hz is 60 RPM.
 * The default unit is #U_01HZ, set on the initial generation of the project by the Workbench. 
 * As this symbol is defined in a User Section, custom values set by users persist across project 
 * regeneration.
 *
 * PID parameters computed by the Motor Control Workbench for speed regulation are 
 * suited for a speed in 01Hz. The motor control subsystem internally scales them to adapt to the 
 * actual speed unit.
 *
 * This symbol should not be set to a literal numeric value. Rather, it should be set to one
 * of the symbols predefined for that purpose such as #U_RPM, #U_01HZ,... See @ref SpeedUnit for 
 * more details. 
 * 
 * Refer to the documentation of the @ref MCIAPI for the functions that use this unit.
 *
 * @{
 */
#define SPEED_UNIT U_01HZ

/* USER CODE END DEFINITIONS */

#define RPM_2_SPEED_UNIT(rpm)   ((int16_t)(((rpm)*SPEED_UNIT)/U_RPM)) /*!< Convenient macro to convert user friendly RPM into SpeedUnit used by MC API */
#define SPEED_UNIT_2_RPM(speed)   ((int16_t)(((speed)*U_RPM)/SPEED_UNIT)) /*!< Convenient macro to convert SpeedUnit used by MC API into user friendly RPM */
/**
* @}
*/


#endif /* MC_STM_TYPES_H */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
