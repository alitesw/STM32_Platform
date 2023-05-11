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
<#assign AUX_SPEED_FDBK_M1 = (MC.AUX_HALL_SENSORS  || MC.AUX_ENCODER || MC.AUX_STATE_OBSERVER_PLL || MC.AUX_STATE_OBSERVER_CORDIC)>
<#assign AUX_SPEED_FDBK_M2 = (MC.AUX_HALL_SENSORS2 || MC.AUX_ENCODER2 || MC.AUX_STATE_OBSERVER_PLL2 || MC.AUX_STATE_OBSERVER_CORDIC2)>
<#assign IS_STO_CORDIC = MC.STATE_OBSERVER_CORDIC ||  MC.AUX_STATE_OBSERVER_CORDIC>
<#assign IS_STO_CORDIC2 = MC.STATE_OBSERVER_CORDIC2 ||  MC.AUX_STATE_OBSERVER_CORDIC2>
<#assign IS_STO_PLL = MC.STATE_OBSERVER_PLL ||  MC.AUX_STATE_OBSERVER_PLL>
<#assign IS_STO_PLL2 = MC.STATE_OBSERVER_PLL2 ||  MC.AUX_STATE_OBSERVER_PLL2>
<#assign IS_SENSORLESS = MC.STATE_OBSERVER_CORDIC ||  MC.STATE_OBSERVER_PLL>
<#assign IS_SENSORLESS2 = MC.STATE_OBSERVER_CORDIC2 ||  MC.STATE_OBSERVER_PLL2>
<#assign IS_ENCODER = MC.ENCODER ||  MC.AUX_ENCODER>
<#assign IS_ENCODER2 = MC.ENCODER2 ||  MC.AUX_ENCODER2>
<#assign IS_HALL_SENSORS = MC.HALL_SENSORS ||  MC.AUX_HALL_SENSORS>
<#assign IS_HALL_SENSORS2 = MC.HALL_SENSORS2 ||  MC.AUX_HALL_SENSORS2>

<#assign CondFamily_STM32F0 = (FamilyName?? && FamilyName=="STM32F0")>
<#-- Condition for STM32G0 Family -->
<#assign CondFamily_STM32G0 = (FamilyName?? && FamilyName=="STM32G0")>

<#assign UNALIGNMENT_SUPPORTED = !(CondFamily_STM32F0 || CondFamily_STM32G0)>
<#assign DWT_CYCCNT_SUPPORTED = !(CondFamily_STM32F0 || CondFamily_STM32G0)>
<#assign MEMORY_FOOTPRINT_REG = !(MC.LOW_MEMORY_FOOTPRINT_REG)>
<#assign MEMORY_FOOTPRINT_REG2 = !(MC.LOW_MEMORY_FOOTPRINT_REG2)>

<#function GetSpeedSensor motor aux=false> 
      <#local SENSORS =
        [ {"name": "PLL", "variable": "&STO_PLL"} 
        , {"name": "CORDIC", "variable": "&STO_CR"} 
        , {"name": "HALL", "variable": "&HALL"}
        , {"name": "ENCODER", "variable": "&ENCODER"}
        ] >
      <#if motor == "M1"> 
        <#if !aux > <#assign entry = MC.SPEED_SENSOR_SELECTION> <#else> <#assign entry = MC.AUXILIARY_SPEED_MEASUREMENT_SELECTION> </#if>
      <#else>
        <#if !aux > <#assign entry = MC.SPEED_SENSOR_SELECTION2> <#else> <#assign entry = MC.AUXILIARY_SPEED_MEASUREMENT_SELECTION2> </#if>
      </#if>
      <#list SENSORS as sensor>
        <#if entry?contains(sensor.name)>
         <#return  sensor.variable+"_"+motor>
        </#if>
      </#list>
     <#return "MC_NULL">
</#function>
/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
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
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
<#if MC.MCP_USED>
#include "mcp.h"
#include "mcp_config.h"
  <#if MC.MCP_DATALOG_USED>
#include "mcpa.h"
  </#if>
</#if>
<#if MC.DAC_FUNCTIONALITY>
#include "dac_ui.h"
</#if>
<#if MC.MOTOR_PROFILER == true>
#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"
</#if>
#include "mc_configuration_registers.h"

<#if MC.DUALDRIVE == true>
  <#if IS_SENSORLESS || IS_SENSORLESS2> 
 static RevUpCtrl_Handle_t * RevUpControl [NBR_OF_MOTORS] = {<#if IS_SENSORLESS >&RevUpControlM1<#else>MC_NULL</#if>, <#if IS_SENSORLESS2>&RevUpControlM2<#else>MC_NULL</#if>};  
  </#if>
  <#if  IS_STO_CORDIC || IS_STO_CORDIC2> 
static STO_CR_Handle_t * stoCRSensor [NBR_OF_MOTORS] = {<#if IS_STO_CORDIC >&STO_CR_M1<#else>MC_NULL</#if>, <#if IS_STO_CORDIC2>&STO_CR_M2<#else>MC_NULL</#if>};  
  </#if>
  <#if IS_STO_PLL || IS_STO_PLL2>
static STO_PLL_Handle_t * stoPLLSensor [NBR_OF_MOTORS] = {<#if IS_STO_PLL >&STO_PLL_M1<#else>MC_NULL</#if>, <#if IS_STO_PLL2>&STO_PLL_M2<#else>MC_NULL</#if>}; 
  </#if> 
  <#if MEMORY_FOOTPRINT_REG2>
static PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS] = { &PIDSpeedHandle_M1, &PIDSpeedHandle_M2 };
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
static OpenLoop_Handle_t *pOL[NBR_OF_MOTORS] = { &OpenLoop_ParamsM1, &OpenLoop_ParamsM2};
  </#if>
  <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
PID_Handle_t *pPIDPosCtrl[NBR_OF_MOTORS] = {<#if MC.POSITION_CTRL_ENABLING >&PID_PosParamsM1<#else>MC_NULL</#if>, <#if MC.POSITION_CTRL_ENABLING2>&PID_PosParamsM2<#else>MC_NULL</#if>};
  </#if>
  <#if MC.FLUX_WEAKENING_ENABLING==true || MC.FLUX_WEAKENING_ENABLING2==true> 
static PID_Handle_t *pPIDFW[NBR_OF_MOTORS] = { <#if MC.FLUX_WEAKENING_ENABLING>&PIDFluxWeakeningHandle_M1<#else>MC_NULL</#if>,<#if MC.FLUX_WEAKENING_ENABLING2>&PIDFluxWeakeningHandle_M2<#else>MC_NULL </#if>};
  </#if>
  <#if IS_ENCODER || IS_ENCODER2>
static ENCODER_Handle_t *pEncoder[NBR_OF_MOTORS] = {<#if IS_ENCODER>&ENCODER_M1<#else>MC_NULL</#if>,<#if IS_ENCODER2>&ENCODER_M2<#else>MC_NULL</#if>};
  </#if>
  <#if IS_HALL_SENSORS || IS_HALL_SENSORS2>
static HALL_Handle_t *pHallSensor[NBR_OF_MOTORS] = {<#if IS_HALL_SENSORS>&HALL_M1<#else>MC_NULL</#if>,<#if IS_HALL_SENSORS2>&HALL_M2<#else>MC_NULL</#if>};
  </#if>
<#else>
  <#if IS_SENSORLESS || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
static RevUpCtrl_Handle_t *RevUpControl[NBR_OF_MOTORS] = { &RevUpControlM1 };
  </#if>
  <#if  IS_STO_CORDIC>
static STO_CR_Handle_t * stoCRSensor [NBR_OF_MOTORS] = { &STO_CR_M1 };
  </#if>
  <#if  IS_STO_PLL>
static STO_PLL_Handle_t * stoPLLSensor [NBR_OF_MOTORS] = { &STO_PLL_M1 };
  </#if>
  <#if MEMORY_FOOTPRINT_REG2>
static PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS] = { &PIDSpeedHandle_M1 };
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
  <#if MC.POSITION_CTRL_ENABLING>
PID_Handle_t *pPIDPosCtrl[NBR_OF_MOTORS] = { &PID_PosParamsM1 };
  </#if>
  <#if MC.FLUX_WEAKENING_ENABLING==true>
static PID_Handle_t *pPIDFW[NBR_OF_MOTORS] = { &PIDFluxWeakeningHandle_M1};
  </#if>
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
static OpenLoop_Handle_t *pOL[NBR_OF_MOTORS] = { &OpenLoop_ParamsM1};
  </#if>
  <#if IS_ENCODER>
static ENCODER_Handle_t *pEncoder[NBR_OF_MOTORS] = {&ENCODER_M1};
  </#if>
  <#if IS_HALL_SENSORS>
static HALL_Handle_t *pHallSensor[NBR_OF_MOTORS] = {&HALL_M1};
  </#if>
  <#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
static Bemf_ADC_Handle_t *pBemfADC[NBR_OF_MOTORS] = {&Bemf_ADC_M1};
  </#if>
  <#if MC.DRIVE_TYPE == "SIX_STEP">
    <#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
static PWMC_SixPwm_Handle_t *pwmcHandle[NBR_OF_MOTORS] = {&PWM_Handle_M1};
    <#else>
static PWMC_ThreePwm_Handle_t *pwmcHandle[NBR_OF_MOTORS] = {&PWM_Handle_M1};
    </#if>
  </#if>	
</#if> <#-- Single Drive -->

static uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t dataAvailable);
static uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t maxSize);
<#if MEMORY_FOOTPRINT_REG>
static uint8_t RI_MovString(const char_t * srcString, char_t * destString, uint16_t *size, int16_t maxSize);
</#if><#-- MEMORY_FOOTPRINT_REG -->
__weak uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT
  if (MC_NULL == pHandle)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    int16_t rxLength = pHandle->rxLength;
    uint16_t size = 0U;
    uint8_t number_of_item =0;
    pHandle->txLength = 0;
    uint8_t accessResult;
    while (rxLength > 0)
    {
       number_of_item++;
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
      rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data
      accessResult = RI_SetReg (*dataElementID,rxData,&size,rxLength);
      
      /* Prepare next data*/
      rxLength = (int16_t) (rxLength - size);
      rxData = rxData+size;
      /* If there is only one CMD in the buffer, we do not store the result */
        if ((1U == number_of_item) && (0 == rxLength))
      {
        retVal = accessResult;
      }
      else
      {/* Store the result for each access to be able to report failling access */
        if (txSyncFreeSpace !=0 ) 
        {
          *txData = accessResult;
          txData = txData+1;
          pHandle->txLength++;
          txSyncFreeSpace--; /* decrement one by one no wraparound possible */
          retVal = (accessResult != MCP_CMD_OK) ? MCP_CMD_NOK : retVal;
          if ((accessResult == MCP_ERROR_BAD_DATA_TYPE) || (accessResult == MCP_ERROR_BAD_RAW_FORMAT))
          { /* From this point we are not able to continue to decode CMD buffer*/
            /* We stop the parsing */
            rxLength = 0;
          }
        }
        else
        {
          /* Stop parsing the cmd buffer as no space to answer */
          /* If we reach this state, chances are high the command was badly formated or received */
          rxLength = 0; 
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
      }
    }
    /* If all accesses are fine, just one global MCP_CMD_OK is required*/  
      if (MCP_CMD_OK == retVal)
    {
      pHandle->txLength = 0;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

__weak uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_NOK;
#ifdef NULL_PTR_REG_INT  
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    uint16_t size = 0;
    uint16_t rxLength = pHandle->rxLength;
    int16_t freeSpaceS16 = (int16_t) txSyncFreeSpace;

    pHandle->txLength = 0;
    
    while (rxLength > 0U)
    {
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength-MCP_ID_SIZE; 
      rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next MCP_ID 
      retVal = RI_GetReg (*dataElementID,txData, &size, freeSpaceS16); 
      if (retVal == MCP_CMD_OK )
      {
        txData = txData+size;
        pHandle->txLength += size;
        freeSpaceS16 = freeSpaceS16-size;
      }
      else 
      { 
        rxLength = 0;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif 
  return (retVal);
}

uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT  
  if ((MC_NULL == data) || (MC_NULL == size))
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {  
#endif
    uint16_t regID = dataID & REG_MASK;
    uint8_t motorID;
    uint8_t typeID;

    typeID = (uint8_t)dataID & TYPE_MASK;
<#if MC.DUALDRIVE == true>
    motorID = (uint8_t)((dataID & MOTOR_MASK)-1U);
    motorID = (motorID <= (MAX_OF_MOTORS - 1U)) ? motorID : (MAX_OF_MOTORS - 1U);
<#else>
    motorID = 0U;
</#if>
    MCI_Handle_t *pMCIN = &Mci[motorID];
  
    switch (typeID)
    { //cstat !MISRAC2012-Rule-16.1
      case TYPE_DATA_8BIT:
      {
        switch (regID)
        {
          case MC_REG_STATUS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_CONTROL_MODE:
          {
            uint8_t regdata8 = *data;
            if ((uint8_t)MCM_TORQUE_MODE == regdata8)
            {
<#if MC.DRIVE_TYPE == "FOC">
              MCI_ExecTorqueRamp(pMCIN, MCI_GetTeref(pMCIN), 0);
</#if>
            }
            else
            {
              /* Nothing to do */
            }

            if ((uint8_t)MCM_SPEED_MODE == regdata8)
            {
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == false && MC.M2_DBG_OPEN_LOOP_ENABLE == false> 
              MCI_ExecSpeedRamp(pMCIN, MCI_GetMecSpeedRefUnit(pMCIN), 0);
<#else>
              MCI_SetSpeedMode(pMCIN);
</#if>
            }
            else
            {
              /* Nothing to do */
            }

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            if ((uint8_t)regdata8 == MCM_OPEN_LOOP_CURRENT_MODE)
            {
              MCI_SetOpenLoopCurrent(pMCIN);
            }
            else
            {
              /* Nothing to do */
            }
            
            if ((uint8_t)regdata8 == MCM_OPEN_LOOP_VOLTAGE_MODE)
            {
              MCI_SetOpenLoopVoltage(pMCIN);
            }
            else
            {
              /* Nothing to do */
            }
</#if> 
            break;
          }

<#if IS_SENSORLESS || IS_SENSORLESS2>
          case MC_REG_RUC_STAGE_NBR:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
</#if> 
<#if MC.PFC_ENABLED>
          case MC_REG_PFC_STATUS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_PFC_ENABLED:
            break;

</#if>
<#if MC.MOTOR_PROFILER>
          case MC_REG_SC_PP:
          { 
            uint8_t regdataU8 = *(uint8_t *)data;
  <#if IS_STO_PLL>  
            SPD_SetElToMecRatio(&STO_PLL_M1._Super, regdataU8);
  </#if>
  <#if IS_STO_CORDIC>
            SPD_SetElToMecRatio(&STO_CR_M1._Super, regdataU8);
  </#if>
            SPD_SetElToMecRatio(&VirtualSpeedSensorM1._Super, regdataU8);
  <#if MC.AUX_HALL_SENSORS>
            SPD_SetElToMecRatio(&HALL_M1._Super, regdataU8);
  </#if>
            SCC_SetPolesPairs(&SCC, regdataU8);
            OTT_SetPolesPairs(&OTT, regdataU8);
            break;
          }
          case MC_REG_SC_CHECK:
          case MC_REG_SC_STATE:
          case MC_REG_SC_STEPS:
          case MC_REG_SC_FOC_REP_RATE:
          case MC_REG_SC_COMPLETED:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  <#if MC.AUX_HALL_SENSORS>
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_HT_MECH_WANTED_DIRECTION:
          {
            uint8_t regdataU8 = *(uint8_t *)data;
            HT_SetMechanicalWantedDirection(&HT,regdataU8);
            break;    
          }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if>
</#if>
<#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_POSITION_CTRL_STATE:
          case MC_REG_POSITION_ALIGN_STATE:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
          case MC_REG_FAST_DEMAG:
          {
            uint8_t regdata8 = *data;
            PWMC_SetFastDemagState(&pwmcHandle[motorID]->_Super, regdata8 );
            break;
          }

          case MC_REG_QUASI_SYNCH:
          {
            uint8_t regdata8 = *data;
            PWMC_SetQuasiSynchState(&pwmcHandle[motorID]->_Super, regdata8 );
            break;
          }

</#if>
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 1;
        break;
      }

      case TYPE_DATA_16BIT:
      {
<#if ( MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2)>
        uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
</#if><#-- MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2 -->
        switch (regID) 
        {
<#if MEMORY_FOOTPRINT_REG2>
          case MC_REG_SPEED_KP:
          {
            PID_SetKP(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_SPEED_KI:
          {
            PID_SetKI(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_SPEED_KD:
          {
            PID_SetKD(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
<#if MC.DRIVE_TYPE == "FOC">
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_Q_KP:
          {
            PID_SetKP(pPIDIq[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_Q_KI:
          {
            PID_SetKI(pPIDIq[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_Q_KD:
          {
            PID_SetKD(pPIDIq[motorID], (int16_t)regdata16);
            break;           
          }

          case MC_REG_I_D_KP:
          {
            PID_SetKP(pPIDId[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_D_KI:
          {
            PID_SetKI(pPIDId[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_D_KD:
          {
            PID_SetKD(pPIDId[motorID], (int16_t)regdata16);
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
  <#if MEMORY_FOOTPRINT_REG2>
    <#if MC.FLUX_WEAKENING_ENABLING==true || MC.FLUX_WEAKENING_ENABLING2==true>

          case MC_REG_FLUXWK_KP:
          {
            PID_SetKP(pPIDFW[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_FLUXWK_KI:
          {          
            PID_SetKI(pPIDFW[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_FLUXWK_BUS:
          {
            FW_SetVref(pFW[motorID], regdata16);
            break;
          }

    </#if> <#-- FLUX_WEAKENING_ENABLING -->
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
</#if><#-- MC.DRIVE_TYPE -->

          case MC_REG_BUS_VOLTAGE:
          case MC_REG_HEATS_TEMP:
          case MC_REG_MOTOR_POWER:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
<#if MEMORY_FOOTPRINT_REG2>
  <#if MC.DAC_FUNCTIONALITY>
          case MC_REG_DAC_OUT1:
          {
            DAC_SetChannelConfig(&DAC_Handle , DAC_CH1, regdata16);
            break;
          }
          
          case MC_REG_DAC_OUT2:
          {
            DAC_SetChannelConfig(&DAC_Handle , DAC_CH2, regdata16);
            break;
          }
  </#if>
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
<#if MC.DRIVE_TYPE == "FOC">
  <#if MEMORY_FOOTPRINT_REG2>
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_A:
          case MC_REG_I_B:
          case MC_REG_I_ALPHA_MEAS:
          case MC_REG_I_BETA_MEAS:
          case MC_REG_I_Q_MEAS:
          case MC_REG_I_D_MEAS:
    </#if><#-- MEMORY_FOOTPRINT_REG -->

          case MC_REG_FLUXWK_BUS_MEAS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_Q_REF:
          {
            qd_t currComp;
            currComp = MCI_GetIqdref(pMCIN);
            currComp.q = (int16_t)regdata16;
            MCI_SetCurrentReferences(pMCIN,currComp);
            break;
          }

          case MC_REG_I_D_REF:
          {
            qd_t currComp;
            currComp = MCI_GetIqdref(pMCIN);
            currComp.d = (int16_t)regdata16;
            MCI_SetCurrentReferences(pMCIN,currComp);
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
  <#if MEMORY_FOOTPRINT_REG2>  
          case MC_REG_V_Q:
          case MC_REG_V_D:
          case MC_REG_V_ALPHA:
          case MC_REG_V_BETA:
  <#if IS_ENCODER || IS_ENCODER2>
          case MC_REG_ENCODER_EL_ANGLE:
          case MC_REG_ENCODER_SPEED:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  </#if><#-- IS_ENCODER || IS_ENCODER2  -->
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
</#if><#-- DRIVE_TYPE == "FOC" -->
<#if MEMORY_FOOTPRINT_REG2>
  <#if IS_HALL_SENSORS || IS_HALL_SENSORS2>
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_HALL_EL_ANGLE:
          case MC_REG_HALL_SPEED:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if><#-- IS_HALL_SENSORS || IS_HALL_SENSORS2 -->
  <#if IS_STO_PLL || IS_STO_PLL2>

          case MC_REG_STOPLL_C1:
          {
            int16_t hC1;
            int16_t hC2;
            STO_PLL_GetObserverGains(stoPLLSensor[motorID], &hC1, &hC2);
            STO_PLL_SetObserverGains(stoPLLSensor[motorID], (int16_t)regdata16, hC2);
            break;
          }

          case MC_REG_STOPLL_C2:
          {
            int16_t hC1;
            int16_t hC2;
            STO_PLL_GetObserverGains(stoPLLSensor[motorID], &hC1, &hC2);
            STO_PLL_SetObserverGains(stoPLLSensor[motorID], hC1, (int16_t)regdata16);
            break;
          }

          case MC_REG_STOPLL_KI:
          {
            PID_SetKI (&stoPLLSensor[motorID]->PIRegulator, (int16_t)regdata16);
            break;
          }

          case MC_REG_STOPLL_KP:
          {
            PID_SetKP (&stoPLLSensor[motorID]->PIRegulator, (int16_t)regdata16);
            break;
          }
          case MC_REG_STOPLL_EL_ANGLE:
          case MC_REG_STOPLL_ROT_SPEED:
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_STOPLL_I_ALPHA:
          case MC_REG_STOPLL_I_BETA:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_STOPLL_BEMF_ALPHA:
          case MC_REG_STOPLL_BEMF_BETA:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  </#if>
  <#if IS_STO_CORDIC || IS_STO_CORDIC2>
          case MC_REG_STOCORDIC_EL_ANGLE:
          case MC_REG_STOCORDIC_ROT_SPEED:
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_STOCORDIC_I_ALPHA:
          case MC_REG_STOCORDIC_I_BETA:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_STOCORDIC_BEMF_ALPHA:
          case MC_REG_STOCORDIC_BEMF_BETA:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
    
          case MC_REG_STOCORDIC_C1:
          {
            int16_t hC1,hC2;
            STO_CR_GetObserverGains(stoCRSensor[motorID], &hC1,&hC2);
            STO_CR_SetObserverGains(stoCRSensor[motorID], (int16_t)regdata16, hC2);
             break;
          }
          
          case MC_REG_STOCORDIC_C2:
          {
            int16_t hC1,hC2;
            STO_CR_GetObserverGains(stoCRSensor[motorID], &hC1, &hC2);
            STO_CR_SetObserverGains(stoCRSensor[motorID], hC1, (int16_t)regdata16);
            break;
          }


  </#if>
          case MC_REG_DAC_USER1:
          case MC_REG_DAC_USER2:
            break;


  <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING || MC.FEED_FORWARD_CURRENT_REG_ENABLING2>     
          case MC_REG_FF_VQ:
          case MC_REG_FF_VD:
          case MC_REG_FF_VQ_PIOUT:
          case MC_REG_FF_VD_PIOUT:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

  </#if>
  <#if MC.PFC_ENABLED>
          case MC_REG_PFC_DCBUS_REF:
          case MC_REG_PFC_DCBUS_MEAS:
          case MC_REG_PFC_ACBUS_FREQ:
          case MC_REG_PFC_ACBUS_RMS:
    <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_PFC_I_KP:
          case MC_REG_PFC_I_KI:
          case MC_REG_PFC_I_KD:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_PFC_V_KP:
          case MC_REG_PFC_V_KI:
          case MC_REG_PFC_V_KD:
          case MC_REG_PFC_STARTUP_DURATION:
            break;
  </#if>     
  <#if MC.MOTOR_PROFILER>
          case MC_REG_SC_PWM_FREQUENCY:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
  </#if>
  <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
          case MC_REG_POSITION_KP:
          {
            PID_SetKP(pPIDPosCtrl[motorID], regdata16);  
            break;
          }

          case MC_REG_POSITION_KI:
          {
            PID_SetKI(pPIDPosCtrl[motorID], regdata16);  
            break;
          }

          case MC_REG_POSITION_KD:
          {
            PID_SetKD(pPIDPosCtrl[motorID], regdata16);  
            break;
          }
  </#if>
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_SPEED_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          } 

          case MC_REG_SPEED_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          }

          case MC_REG_SPEED_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
<#if MEMORY_FOOTPRINT_REG>
  <#if MC.DRIVE_TYPE == "FOC">

          case MC_REG_I_D_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }

          case MC_REG_I_D_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }
          
          case MC_REG_I_D_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }
          
          case MC_REG_I_Q_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }
          
          case MC_REG_I_Q_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }
          
          case MC_REG_I_Q_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }
  </#if><#-- DRIVE_TYPE == "FOC" -->
  <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
          case MC_REG_POSITION_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDPosCtrl[motorID], regdata16);
            break;
          }

          case MC_REG_POSITION_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDPosCtrl[motorID], regdata16);
            break;
          }

          case MC_REG_POSITION_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDPosCtrl[motorID], regdata16);
            break;
          }
  </#if>
  <#if MC.PFC_ENABLED>
          case MC_REG_PFC_I_KP_DIV:
          case MC_REG_PFC_I_KI_DIV:
          case MC_REG_PFC_I_KD_DIV:
          case MC_REG_PFC_V_KP_DIV:
          case MC_REG_PFC_V_KI_DIV:
          case MC_REG_PFC_V_KD_DIV:
  </#if>
  <#if IS_STO_PLL || IS_STO_PLL2>
          case MC_REG_STOPLL_KI_DIV:
          {
            PID_SetKIDivisorPOW2 (&stoPLLSensor[motorID]->PIRegulator,regdata16);
            break;
          }

          case MC_REG_STOPLL_KP_DIV:
          {
            PID_SetKPDivisorPOW2 (&stoPLLSensor[motorID]->PIRegulator,regdata16);
            break;
          }
  </#if>

  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
          case MC_REG_FOC_VDREF:
          {
            OL_UpdateVoltage( pOL[motorID], ((regdata16 * 32767) / 100));
            break;
          }
 
  </#if>
  <#if MC.FLUX_WEAKENING_ENABLING==true || MC.FLUX_WEAKENING_ENABLING2==true>
          case MC_REG_FLUXWK_KP_DIV: 
          {
            PID_SetKPDivisorPOW2 (pPIDFW[motorID],regdata16);
            break;
          }

          case MC_REG_FLUXWK_KI_DIV:
          {
            PID_SetKIDivisorPOW2 (pPIDFW[motorID],regdata16);
            break;
          }
  </#if>     
  <#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
          case MC_REG_PULSE_VALUE:
          case MC_REG_BEMF_ZCR:
          case MC_REG_BEMF_U:
          case MC_REG_BEMF_V:
          case MC_REG_BEMF_W:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

  </#if> 
</#if><#-- MEMORY_FOOTPRINT_REG -->
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 2;
        break;
      }

      case TYPE_DATA_32BIT:
      {
<#if UNALIGNMENT_SUPPORTED>
        uint32_t regdata32 = *(uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
<#else>
        uint32_t regdata32 = (((uint32_t)(*(uint16_t *)&data[2])) << 16) | *(uint16_t *)data;
</#if>

        switch (regID)
        {

          case MC_REG_FAULTS_FLAGS:
          case MC_REG_SPEED_MEAS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_SPEED_REF:
          {
            MCI_ExecSpeedRamp(pMCIN,((((int16_t)regdata32) * ((int16_t)SPEED_UNIT)) / (int16_t)U_RPM), 0);
            break;
          }
<#if MEMORY_FOOTPRINT_REG2>
<#if IS_STO_PLL || IS_STO_PLL2>
          case MC_REG_STOPLL_EST_BEMF:
          case MC_REG_STOPLL_OBS_BEMF:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
</#if><#-- IS_STO_PLL || IS_STO_PLL2 -->
<#if IS_STO_CORDIC || IS_STO_CORDIC2>
          case MC_REG_STOCORDIC_EST_BEMF:
          case MC_REG_STOCORDIC_OBS_BEMF:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
</#if><#-- IS_STO_CORDIC || IS_STO_CORDIC2 -->
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING || MC.FEED_FORWARD_CURRENT_REG_ENABLING2>
          case MC_REG_FF_1Q:
          {
            pFF[motorID]->wConstant_1Q = (int32_t)regdata32;
            break;
          }

          case MC_REG_FF_1D:
          {
            pFF[motorID]->wConstant_1D = (int32_t)regdata32;
            break;
          }

          case MC_REG_FF_2:
          {
            pFF[motorID]->wConstant_2 = (int32_t)regdata32;
            break;
          }
</#if> 
<#if MC.PFC_ENABLED>
          case MC_REG_PFC_FAULTS:
            break;
</#if>

<#if MC.MOTOR_PROFILER>
          case MC_REG_SC_RS:
          case MC_REG_SC_LS:
          case MC_REG_SC_KE:
          case MC_REG_SC_VBUS:
          case MC_REG_SC_MEAS_NOMINALSPEED:
            retVal = MCP_ERROR_RO_REG;
            break;
          case MC_REG_SC_CURRENT:
          { /* Profiler is supported only by series supporting unaligned access*/
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            SCC_SetNominalCurrent(&SCC, fregdata);
            break;
          }
          case MC_REG_SC_SPDBANDWIDTH:
          { 
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            OTT_SetSpeedRegulatorBandwidth(&OTT, fregdata);
            break;
          }
          case MC_REG_SC_LDLQRATIO:
          { 
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            SCC_SetLdLqRatio(&SCC, fregdata);
            break;
          }          
          case MC_REG_SC_NOMINAL_SPEED:
            SCC_SetNominalSpeed (&SCC, (int32_t) regdata32);
            break;
          case MC_REG_SC_CURRBANDWIDTH:
          { 
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            SCC_SetCurrentBandwidth(&SCC, fregdata);
            break;
          }
          case MC_REG_SC_J:
          case MC_REG_SC_F:
          case MC_REG_SC_MAX_CURRENT:
          case MC_REG_SC_STARTUP_SPEED:
          case MC_REG_SC_STARTUP_ACC:
            retVal = MCP_ERROR_RO_REG;
            break;
</#if>
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 4;
        break;
      }
<#if MEMORY_FOOTPRINT_REG>
      case TYPE_DATA_STRING:
      {
        const char_t *charData = (const char_t *)data;
        char_t *dummy = (char_t *)data ;
        retVal = MCP_ERROR_RO_REG;
        /* Used to compute String length stored in RXBUFF even if Reg does not exist*/
        /* It allows to jump to the next command in the buffer */
        (void)RI_MovString (charData, dummy, size, dataAvailable);
        break;
      }
</#if><#-- MEMORY_FOOTPRINT_REG -->

      case TYPE_DATA_RAW:
      {
        uint16_t rawSize = *(uint16_t *) data; //cstat !MISRAC2012-Rule-11.3
        /* The size consumed by the structure is the structure size + 2 bytes used to store the size*/ 
        *size = rawSize + 2U; 
        uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
        rawData++;
        rawData++;

        if (*size > dataAvailable )
        { /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer construction*/
          *size = 0; 
          retVal = MCP_ERROR_BAD_RAW_FORMAT; /* this error stop the parsing of the CMD buffer */
        }
        else
        {
          switch (regID)
          {
<#if MEMORY_FOOTPRINT_REG>
            case MC_REG_GLOBAL_CONFIG:
            case MC_REG_MOTOR_CONFIG: 
            case MC_REG_APPLICATION_CONFIG:
            case MC_REG_FOCFW_CONFIG:
            {
              retVal = MCP_ERROR_RO_REG;
              break;
            }
</#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_SPEED_RAMP:
            {
              int32_t rpm;
              uint16_t duration;

              <#if UNALIGNMENT_SUPPORTED >
              rpm = *(int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
              <#else>
              /* 32 bits access are splited into 2x16 bits access */
              rpm = (((int32_t)(*(int16_t *)&rawData[2])) << 16) | *(uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
              </#if>
              duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
              MCI_ExecSpeedRamp(pMCIN, (int16_t)((rpm * SPEED_UNIT) / U_RPM), duration);
              break;
            }
<#if MEMORY_FOOTPRINT_REG>
  <#if MC.DRIVE_TYPE == "FOC">
            case MC_REG_TORQUE_RAMP:
            {
              uint32_t torque;
              uint16_t duration;
              
              <#if UNALIGNMENT_SUPPORTED >
              torque = *(uint32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
              <#else>
              /* 32 bits access are splited into 2x16 bits access */
              //cstat !MISRAC2012-Rule-11.3
              torque = ((uint32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData;
              </#if>
              duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
              MCI_ExecTorqueRamp(pMCIN, (int16_t)torque, duration);
              break;
            }

    <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
            case MC_REG_POSITION_RAMP:
            {
              FloatToU32 Position;
              FloatToU32 Duration;

              Position.U32_Val = ((int32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData; /* 32 bits access are split into 2x16 bits access */
              Duration.U32_Val = ((int32_t)(*(int16_t *)&rawData[6]))<<16 | *(uint16_t *)&rawData[4]; /* 32 bits access are split into 2x16 bits access */
              MCI_ExecPositionCommand(pMCIN, Position.Float_Val, Duration.Float_Val);
              break;  
            }

    </#if>  
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if IS_SENSORLESS || IS_SENSORLESS2 || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
            case MC_REG_REVUP_DATA:
            {
              int32_t rpm;
              RevUpCtrl_PhaseParams_t revUpPhase;
              uint8_t i;
              uint8_t nbrOfPhase = (((uint8_t)rawSize) / 8U);
              
              if (((0U != ((rawSize) % 8U))) || ((nbrOfPhase > RUC_MAX_PHASE_NUMBER) != 0))
              {
                retVal = MCP_ERROR_BAD_RAW_FORMAT;
              }
              else
              {
                for (i = 0; i <nbrOfPhase; i++)
                {
    <#if UNALIGNMENT_SUPPORTED>
                rpm = *(int32_t *) &rawData[i * 8U]; //cstat !MISRAC2012-Rule-11.3
    <#else>
                /* &rawData is guarantee to be 16 bits aligned, so 32 bits access are to be splitted */
                //cstat !MISRAC2012-Rule-11.3
                rpm = ((int32_t)(*(int16_t *)&rawData[2+i*8]))<<16 | *(uint16_t *)&rawData[i*8];
    </#if>
                revUpPhase.hFinalMecSpeedUnit = (((int16_t)rpm) * ((int16_t)SPEED_UNIT)) / ((int16_t)U_RPM);
    <#if MC.DRIVE_TYPE == "FOC">
                revUpPhase.hFinalTorque = *((int16_t *) &rawData[4U + (i * 8U)]); //cstat !MISRAC2012-Rule-11.3
    </#if><#-- MC.DRIVE_TYPE == "FOC" -->
    <#if MC.DRIVE_TYPE == "SIX_STEP">
                revUpPhase.hFinalPulse = *((int16_t *) &rawData[4U + (i * 8U)]); //cstat !MISRAC2012-Rule-11.3
    </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
                revUpPhase.hDurationms  = *((uint16_t *) &rawData[6U +(i * 8U)]); //cstat !MISRAC2012-Rule-11.3
                (void)RUC_SetPhase( RevUpControl[motorID], i, &revUpPhase);
                }
              }
              break;
            }

  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_A>
            case MC_REG_ASYNC_UARTA:
            {
              retVal =  MCPA_cfgLog (&MCPA_UART_A, rawData);
              break;
            }

  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
            case MC_REG_ASYNC_UARTB:
            {          
              retVal =  MCPA_cfgLog (&MCPA_UART_B, rawData);
              break;
            }

  </#if>
  <#if MC.MCP_DATALOG_OVER_STLNK>
            case MC_REG_ASYNC_STLNK:
            {          
              retVal =  MCPA_cfgLog (&MCPA_STLNK, rawData);
              break;
            }

  </#if>
  <#if MC.DRIVE_TYPE == "FOC">
            case MC_REG_CURRENT_REF:
            {
              qd_t currComp;
              currComp.q = *((int16_t *) rawData); //cstat !MISRAC2012-Rule-11.3
              currComp.d = *((int16_t *) &rawData[2]); //cstat !MISRAC2012-Rule-11.3
              MCI_SetCurrentReferences(pMCIN, currComp);
              break;
            }

  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
</#if><#-- MEMORY_FOOTPRINT_REG -->
          }
        }
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        *size =0; /* From this point we are not able anymore to decode the RX buffer*/
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t freeSpace)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT
  if ((MC_NULL == data) || (MC_NULL == size))
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif 
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
<#if   MC.DUALDRIVE == true>
    uint8_t motorID = (((uint8_t)dataID) -1U) & MOTOR_MASK;
    motorID = (motorID <= (MAX_OF_MOTORS - 1U)) ? motorID : (MAX_OF_MOTORS - 1U);
    BusVoltageSensor_Handle_t* BusVoltageSensor[NBR_OF_MOTORS]={ &BusVoltageSensor_M1._Super, &BusVoltageSensor_M2._Super};
<#else>
    BusVoltageSensor_Handle_t* BusVoltageSensor[NBR_OF_MOTORS]={ &BusVoltageSensor_M1._Super};
    uint8_t motorID = 0U;
</#if>

    MCI_Handle_t *pMCIN = &Mci[motorID];
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0U)
        {
          switch (regID)
          {
            case MC_REG_STATUS:
            {
              *data = (uint8_t)MCI_GetSTMState(pMCIN);
              break;
            }
            
            case MC_REG_CONTROL_MODE:
            {
              *data = (uint8_t)MCI_GetControlMode(pMCIN);
              break;
            }
<#if IS_SENSORLESS || IS_SENSORLESS2>
            
            case MC_REG_RUC_STAGE_NBR:
            {
              *data = (RevUpControl[motorID] != MC_NULL) ? (uint8_t)RUC_GetNumberOfPhases(RevUpControl[motorID]) : 0U;
              break;
            }
</#if>

<#if MC.PFC_ENABLED>
            case MC_REG_PFC_STATUS:
            case MC_REG_PFC_ENABLED:
              break;
</#if>
<#if MC.MOTOR_PROFILER>
            case MC_REG_SC_CHECK:
              *data = (uint8_t) 1u;
              break;
            case MC_REG_SC_STATE:
            {
              uint8_t state ;
              state = SCC_GetState(&SCC);
              state += OTT_GetState (&OTT);
              *data = state;
              break;
            }
            case MC_REG_SC_STEPS:
            {
              uint8_t steps ;
              steps = SCC_GetSteps(&SCC);
              steps += OTT_GetSteps (&OTT);
              *data = steps-1u;
              break;
            }
            case MC_REG_SC_PP:
              *data = SPD_GetElToMecRatio(&STO_PLL_M1._Super);
              break;
            case MC_REG_SC_FOC_REP_RATE:
              *data = SCC_GetFOCRepRate(&SCC);
              break;
            case MC_REG_SC_COMPLETED:
              *data = OTT_IsMotorAlreadyProfiled(&OTT);
              break;
  <#if MC.AUX_HALL_SENSORS>
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_HT_STATE:
              *data = (uint8_t) HT.sm_state;
              break;
            case MC_REG_HT_PROGRESS:
              *data = HT.bProgressPercentage;
              break;
            case MC_REG_HT_PLACEMENT:
              *data = HT.bPlacement;
              break;
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if>
</#if> <#-- MOTOR_PROFILER -->
<#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2 > 
  <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_POSITION_CTRL_STATE:
            {
              *data = (uint8_t) TC_GetControlPositionStatus(pPosCtrl[motorID]);
              break;
            }
            case MC_REG_POSITION_ALIGN_STATE:
            {
              *data = (uint8_t) TC_GetAlignmentStatus(pPosCtrl[motorID]);
              break;
            }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
            case MC_REG_FAST_DEMAG:
            {
              *data = PWMC_GetFastDemagState(&pwmcHandle[motorID]->_Super);
              break;
            }              
            case MC_REG_QUASI_SYNCH:  
            {
              *data = PWMC_GetQuasiSynchState(&pwmcHandle[motorID]->_Super);
              break;
            }
</#if>
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }
      case TYPE_DATA_16BIT:
      {
        uint16_t *regdataU16 = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 2U)
        {
          switch (regID) 
          {
 <#if MEMORY_FOOTPRINT_REG2>
            case MC_REG_SPEED_KP:
            {
              *regdata16 = PID_GetKP(pPIDSpeed[motorID]);
              break;
            }
            
            case MC_REG_SPEED_KI:
            {
              *regdata16 = PID_GetKI(pPIDSpeed[motorID]);
              break;
            }
            
            case MC_REG_SPEED_KD:
            {
              *regdata16 = PID_GetKD(pPIDSpeed[motorID]);
              break;
            }

  <#if MC.DRIVE_TYPE == "FOC">
    <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_I_Q_KP:
            {
              *regdata16 = PID_GetKP(pPIDIq[motorID]);
              break;
            }

        case MC_REG_I_Q_KI:
            {
              *regdata16 = PID_GetKI(pPIDIq[motorID]);
              break;
            }
            
        case MC_REG_I_Q_KD:
            {
              *regdata16 = PID_GetKD(pPIDIq[motorID]);
              break;
            }

        case MC_REG_I_D_KP:
            {
              *regdata16 = PID_GetKP(pPIDId[motorID]);
              break;
            }
            
        case MC_REG_I_D_KI:
            {
              *regdata16 = PID_GetKI(pPIDId[motorID]);
              break;
            }
            
        case MC_REG_I_D_KD:
            {
              *regdata16 = PID_GetKD(pPIDId[motorID]);
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.FLUX_WEAKENING_ENABLING || MC.FLUX_WEAKENING_ENABLING2 >
            case MC_REG_FLUXWK_KP:
            {
              *regdata16 = PID_GetKP(pPIDFW[motorID]);
              break;
            }
            
            case MC_REG_FLUXWK_KI:
            {
              *regdata16 = PID_GetKI(pPIDFW[motorID]);
              break;
            }
            
            case MC_REG_FLUXWK_BUS:
            {
              *regdataU16 = FW_GetVref(pFW[motorID]);
              break;
            }
            
            case MC_REG_FLUXWK_BUS_MEAS:
            {
              *regdata16 = (int16_t)FW_GetAvVPercentage(pFW[motorID]);
              break;
            }

  </#if>
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
            case MC_REG_BUS_VOLTAGE:
            {
              *regdataU16 = VBS_GetAvBusVoltage_V(BusVoltageSensor[motorID]);
              break;
            }
            
            case MC_REG_HEATS_TEMP:
            {
              *regdata16 = NTC_GetAvTemp_C(pTemperatureSensor[motorID]);
              break;
            }
<#if MEMORY_FOOTPRINT_REG2>
  <#if MC.DAC_FUNCTIONALITY>
            case MC_REG_DAC_OUT1:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH1);
              break;
            }
            
            case MC_REG_DAC_OUT2:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH2);
              break;
            }
  </#if>

  <#if MC.DRIVE_TYPE == "FOC">
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_I_A:
            {
              *regdata16 = MCI_GetIab(pMCIN).a;
              break;
            }

            case MC_REG_I_B:
            {
              *regdata16 = MCI_GetIab(pMCIN).b;
              break;
            }

            case MC_REG_I_ALPHA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_I_BETA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_I_Q_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).q;
              break;
            }

            case MC_REG_I_D_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).d;
              break;
            }

            case MC_REG_I_Q_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).q;
              break;
            }

            case MC_REG_I_D_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).d;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_V_Q:
            {
              *regdata16 = MCI_GetVqd(pMCIN).q;
              break;
            }

            case MC_REG_V_D:
            {
              *regdata16 = MCI_GetVqd(pMCIN).d;
              break;
            }

            case MC_REG_V_ALPHA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_V_BETA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).beta;
              break;
            }
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if IS_ENCODER || IS_ENCODER2>

            case MC_REG_ENCODER_EL_ANGLE:
            {
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]); //cstat !MISRAC2012-Rule-11.3
              break;
            }

            case MC_REG_ENCODER_SPEED:
            {
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]); //cstat !MISRAC2012-Rule-11.3
              break;
            }

  </#if>
  <#if IS_HALL_SENSORS || IS_HALL_SENSORS2>
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_HALL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) pHallSensor[motorID]);
              break;
            }

            case MC_REG_HALL_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) pHallSensor[motorID]);
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if>
  <#if IS_STO_PLL || IS_STO_PLL2>
            case MC_REG_STOPLL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle((SpeednPosFdbk_Handle_t *)stoPLLSensor[motorID]); 
              break;
            }

            case MC_REG_STOPLL_ROT_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed((SpeednPosFdbk_Handle_t *)stoPLLSensor[motorID]); 
              break;
            }
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_STOPLL_I_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(stoPLLSensor[motorID]).alpha;
              break;
            }

            case MC_REG_STOPLL_I_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(stoPLLSensor[motorID]).beta;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_STOPLL_BEMF_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(stoPLLSensor[motorID]).alpha;
              break;
            }

            case MC_REG_STOPLL_BEMF_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(stoPLLSensor[motorID]).beta;
              break;
            }

            case MC_REG_STOPLL_C1:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(stoPLLSensor[motorID], &hC1, &hC2);
              *regdata16 = hC1;
              break;
            }

            case MC_REG_STOPLL_C2:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(stoPLLSensor[motorID], &hC1, &hC2);
              *regdata16 = hC2;
              break;
            }

            case MC_REG_STOPLL_KI:
            {
              *regdata16 = PID_GetKI (&stoPLLSensor[motorID]->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP:
            {
              *regdata16 = PID_GetKP (&stoPLLSensor[motorID]->PIRegulator);
              break;
            }

  </#if>
  <#if IS_STO_CORDIC || IS_STO_CORDIC2>
            case MC_REG_STOCORDIC_EL_ANGLE:
            {
              if (stoCRSensor[motorID] != MC_NULL)
              {
                //cstat !MISRAC2012-Rule-11.3
                *regdata16 = SPD_GetElAngle((SpeednPosFdbk_Handle_t *)stoCRSensor[motorID]); 
              }
              else 
              {
                retVal = MCP_ERROR_UNKNOWN_REG;
              }
              break;
            }
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_STOCORDIC_ROT_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed((SpeednPosFdbk_Handle_t*) stoCRSensor[motorID]);
              break;
            }

            case MC_REG_STOCORDIC_I_ALPHA:
            {
              *regdata16 = STO_CR_GetEstimatedCurrent(stoCRSensor[motorID]).alpha;
              break;
            }

            case MC_REG_STOCORDIC_I_BETA:
            {
              *regdata16 = STO_CR_GetEstimatedCurrent(stoCRSensor[motorID]).beta;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_STOCORDIC_BEMF_ALPHA:
            {
              *regdata16 = STO_CR_GetEstimatedBemf(stoCRSensor[motorID]).alpha; 
              break;
            }

            case MC_REG_STOCORDIC_BEMF_BETA:
            {
              *regdata16 = STO_CR_GetEstimatedBemf(stoCRSensor[motorID]).beta;
              break;
            }

            case MC_REG_STOCORDIC_C1:
            {
              int16_t hC1;
              int16_t hC2;
              STO_CR_GetObserverGains(stoCRSensor[motorID], &hC1, &hC2);
              *regdata16 = hC1;
              break;
            }

            case MC_REG_STOCORDIC_C2:
            {
              int16_t hC1;
              int16_t hC2;
              STO_CR_GetObserverGains(stoCRSensor[motorID], &hC1, &hC2);
              *regdata16 = hC2;
              break;
            }

  </#if>         
            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;

  <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING || MC.FEED_FORWARD_CURRENT_REG_ENABLING2>
            case MC_REG_FF_VQ:
            {
              *regdata16 = FF_GetVqdff(pFF[motorID]).q;
              break;
            }

            case MC_REG_FF_VD:
            {
              *regdata16 = FF_GetVqdff(pFF[motorID]).d;
              break;
            }

            case MC_REG_FF_VQ_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(pFF[motorID]).q;
              break;
            }

            case MC_REG_FF_VD_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(pFF[motorID]).d;
              break;
            }

  </#if>
  <#if MC.PFC_ENABLED>
            case MC_REG_PFC_DCBUS_REF:
            case MC_REG_PFC_DCBUS_MEAS:
            case MC_REG_PFC_ACBUS_FREQ:
            case MC_REG_PFC_ACBUS_RMS:
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_PFC_I_KP:
            case MC_REG_PFC_I_KI:
            case MC_REG_PFC_I_KD:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_PFC_V_KP:
            case MC_REG_PFC_V_KI:
            case MC_REG_PFC_V_KD:
            case MC_REG_PFC_STARTUP_DURATION:
              break;
  </#if>

  <#if MC.MOTOR_PROFILER>
            case MC_REG_SC_PWM_FREQUENCY:
              *regdataU16 = SCC_GetPWMFrequencyHz(&SCC);
              break;
  </#if>
  <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
            case MC_REG_POSITION_KP:
            {
              *regdata16 = PID_GetKP( pPIDPosCtrl[motorID]);
              break;
            }

            case MC_REG_POSITION_KI:
            {
              *regdata16 = PID_GetKI( pPIDPosCtrl[motorID]);
              break;
            }

            case MC_REG_POSITION_KD:
            {
              *regdata16 = PID_GetKD( pPIDPosCtrl[motorID]);
              break;
            }
  </#if>
  <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_SPEED_KP_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKPDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KI_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKIDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }
    <#if MC.DRIVE_TYPE == "FOC">
            case MC_REG_I_D_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_D_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_D_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_Q_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDIq[motorID]);
              break;
            }

            case MC_REG_I_Q_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDIq[motorID]);
              break;
            }

            case MC_REG_I_Q_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDIq[motorID]);
              break;
            }
    </#if><#-- MC.DRIVE_TYPE -->

    <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
            case MC_REG_POSITION_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDPosCtrl[motorID]);
              break;
            }

            case MC_REG_POSITION_KI_DIV: 
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDPosCtrl[motorID]);
              break;
            }

            case MC_REG_POSITION_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDPosCtrl[motorID]);
              break;
            }
    </#if> 

    <#if MC.PFC_ENABLED>
            case MC_REG_PFC_I_KP_DIV:
            case MC_REG_PFC_I_KI_DIV:
            case MC_REG_PFC_I_KD_DIV:
            case MC_REG_PFC_V_KP_DIV:
            case MC_REG_PFC_V_KI_DIV:
            case MC_REG_PFC_V_KD_DIV:
              break;
    </#if>    
    <#if IS_STO_PLL || IS_STO_PLL2>
            case MC_REG_STOPLL_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&stoPLLSensor[motorID]->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&stoPLLSensor[motorID]->PIRegulator);
              break;
            }
    </#if>
  </#if><#-- MEMORY_FOOTPRINT_REG -->
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            case MC_REG_FOC_VDREF:
            {
	          *regdata16 = ((OL_GetVoltage(pOL[motorID])*100)/32767);
              break;
            }
  </#if>
  <#if MEMORY_FOOTPRINT_REG>
    <#if MC.FLUX_WEAKENING_ENABLING || MC.FLUX_WEAKENING_ENABLING2>
            case MC_REG_FLUXWK_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDFW[motorID]);
              break;
            }

            case MC_REG_FLUXWK_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDFW[motorID]);
              break;
            }
    </#if>
    <#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
            case MC_REG_PULSE_VALUE:
            {
              *regdataU16 = MCI_GetDutyCycleRef(pMCIN);
              break;
            }  

            case MC_REG_BEMF_ZCR:
            {
              *regdataU16 = (uint16_t) BADC_GetBemfZcrFlag(pBemfADC[motorID]);
              break;
            }

            case MC_REG_BEMF_U:
            {
              *regdataU16 = BADC_GetLastBemfValue(pBemfADC[motorID], 0);
              break;
            }

            case MC_REG_BEMF_V:
            {
              *regdataU16 = BADC_GetLastBemfValue(pBemfADC[motorID], 1);
              break;
            }  

            case MC_REG_BEMF_W:           
            {
              *regdataU16 = BADC_GetLastBemfValue(pBemfADC[motorID], 2);
              break;
            }
    </#if>
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE; 
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
        int32_t *regdata32 = (int32_t *)data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 4U)
        {
          switch (regID)
          {

            case MC_REG_FAULTS_FLAGS:
            {
              *regdataU32 = MCI_GetFaultState(pMCIN);
              break;
            }

            case MC_REG_SPEED_MEAS:
            {
              *regdata32 = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

            case MC_REG_SPEED_REF:
            { 
              *regdata32 = (((int32_t)MCI_GetMecSpeedRefUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }
<#if MEMORY_FOOTPRINT_REG2> 
<#if IS_STO_PLL || IS_STO_PLL2>
            case MC_REG_STOPLL_EST_BEMF:
            {
              *regdata32 = STO_PLL_GetEstimatedBemfLevel(stoPLLSensor[motorID]);
              break;
            }

            case MC_REG_STOPLL_OBS_BEMF:
            {
              *regdata32 = STO_PLL_GetObservedBemfLevel(stoPLLSensor[motorID]);
              break;
            }

</#if>
<#if IS_STO_CORDIC || IS_STO_CORDIC2>
            case MC_REG_STOCORDIC_EST_BEMF:
            {
              *regdata32 = STO_CR_GetEstimatedBemfLevel(stoCRSensor[motorID]);
              break;
            }

            case MC_REG_STOCORDIC_OBS_BEMF:
            {
              *regdata32 = STO_CR_GetObservedBemfLevel(stoCRSensor[motorID]);
              break;
            }
</#if>
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING || MC.FEED_FORWARD_CURRENT_REG_ENABLING2>
            case MC_REG_FF_1Q:
            {
              *regdata32 = pFF[motorID]->wConstant_1Q;
              break;
            }

            case MC_REG_FF_1D:
            {
              *regdata32 = pFF[motorID]->wConstant_1D;
              break;
            }

            case MC_REG_FF_2:
            {
              *regdata32 = pFF[motorID]->wConstant_2;
              break;
            }
</#if>
<#if MC.PFC_ENABLED>
            case MC_REG_PFC_FAULTS:
              break;
</#if>
<#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
            case MC_REG_CURRENT_POSITION:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MCI_GetCurrentPosition(pMCIN);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

</#if>
<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
            case MC_REG_PERF_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_PERF_MIN_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMinCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_PERF_MAX_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMaxCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
  </#if>
</#if>
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
<#if MC.DRIVE_TYPE == "FOC"><#-- TODO: These registers should also be supported identically in 6S configurations in public releases... and some be made optional -->
            case MC_REG_MOTOR_POWER:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = PQD_GetAvrgElMotorPowerW(pMPM[M1]);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MEMORY_FOOTPRINT_REG2> 
<#if MC.MOTOR_PROFILER>
            case MC_REG_SC_RS:
              *regdataU32 = SCC_GetRs(&SCC);
              break;
            case MC_REG_SC_LS:
              *regdataU32 = SCC_GetLs(&SCC);
              break;
            case MC_REG_SC_KE:
              *regdataU32 = SCC_GetKe(&SCC);
              break;
            case MC_REG_SC_VBUS:
              *regdataU32 = SCC_GetVbus(&SCC);
              break;
            case MC_REG_SC_MEAS_NOMINALSPEED:
              *regdataU32 = OTT_GetNominalSpeed(&OTT);
              break;
            case MC_REG_SC_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetNominalCurrent(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_SPDBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetSpeedRegulatorBandwidth(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_LDLQRATIO:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetLdLqRatio(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_NOMINAL_SPEED:
              *regdata32 = SCC_GetNominalSpeed(&SCC);
              break;
            case MC_REG_SC_CURRBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetCurrentBandwidth(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_J:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetJ(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_F:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetF(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_MAX_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetStartupCurrentAmp(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            case MC_REG_SC_STARTUP_SPEED:
              *regdata32 = SCC_GetEstMaxOLSpeed(&SCC);
              break;
            case MC_REG_SC_STARTUP_ACC:
              *regdata32 = SCC_GetEstMaxAcceleration(&SCC);
              break;
</#if>
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE; 
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
<#if MEMORY_FOOTPRINT_REG>
        char_t *charData = (char_t *)data;
</#if><#-- MEMORY_FOOTPRINT_REG -->
        switch (regID)
        {
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_FW_NAME:
            retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
            break;

          case MC_REG_CTRL_STAGE_NAME:
          {
            retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
            break;
          }

          case MC_REG_PWR_STAGE_NAME:
          {
            retVal = RI_MovString (PWR_BOARD_NAME[motorID] ,charData, size, freeSpace);
            break;
          }

          case MC_REG_MOTOR_NAME:
          {
            retVal = RI_MovString (MotorConfig_reg[motorID]->name ,charData, size, freeSpace);
            break;
          }
</#if><#-- MEMORY_FOOTPRINT_REG -->
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */
            break;
          }
        }
        break;
      }

      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_GLOBAL_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(GlobalConfig_reg_t); 
            if (((*rawSize) + 2U) > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            {
              (void)memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t));
            }
            break;
          }

          case MC_REG_MOTOR_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(MotorConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              MotorConfig_reg_t const *pMotorConfig_reg = MotorConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pMotorConfig_reg, sizeof(MotorConfig_reg_t));
            }
            break;
          }
          case MC_REG_APPLICATION_CONFIG:
          {
            *rawSize = sizeof(ApplicationConfig_reg_t);
            if ((*rawSize) +2  > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              memcpy(rawData, ApplicationConfig_reg[motorID], sizeof(ApplicationConfig_reg_t));
            }
            break;
          }
          case MC_REG_FOCFW_CONFIG:
          {
  <#if MC.DRIVE_TYPE == "FOC"><#-- TODO: Create an equivalent register for 6S -->
            *rawSize = (uint16_t)sizeof(FOCFwConfig_reg_t); 
            if (((*rawSize) + 2U) > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              FOCFwConfig_reg_t const *pFOCConfig_reg = FOCConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pFOCConfig_reg, sizeof(FOCFwConfig_reg_t));
            }
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.DRIVE_TYPE == "SIX_STEP"><#-- TODO: Create an equivalent register for 6S -->
            *rawSize = (uint16_t)sizeof(SixStepFwConfig_reg_t); 
            if (((*rawSize) + 2U) > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              SixStepFwConfig_reg_t const *pSixStepConfig_reg = SixStepConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pSixStepConfig_reg, sizeof(SixStepFwConfig_reg_t));
            }
  </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
            break;
          }
</#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_SPEED_RAMP:
          {
            <#if UNALIGNMENT_SUPPORTED >
            int32_t *rpm = (int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3 
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            *rpm = (((int32_t)MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
            <#else>
            uint16_t *rpm16p = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            int32_t rpm32 = ((int32_t)(MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
            *rpm16p = (uint16_t) rpm32;
            *(rpm16p+1) = (uint16_t)(rpm32>>16);
            </#if>
            *duration = MCI_GetLastRampFinalDuration(pMCIN);
            *rawSize = 6; 
            break;
          }
<#if MEMORY_FOOTPRINT_REG>
  <#if MC.DRIVE_TYPE == "FOC"><#-- TODO: Create an equivalent register for 6S -->
          case MC_REG_TORQUE_RAMP:
          {
            int16_t *torque = (int16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3
            
            *rawSize = 4;
            *torque = MCI_GetLastRampFinalTorque(pMCIN);
            *duration = MCI_GetLastRampFinalDuration(pMCIN) ;
            break;
          }
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if IS_SENSORLESS || IS_SENSORLESS2 || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">

          case MC_REG_REVUP_DATA:
          {
    <#if UNALIGNMENT_SUPPORTED>
            int32_t *rpm;
    <#else>
            int32_t rpm32;
            int16_t *rpm16p;
    </#if>
    <#if MC.DRIVE_TYPE == "FOC">
            uint16_t *finalTorque;
    </#if><#-- MC.DRIVE_TYPE == "FOC" -->
    <#if MC.DRIVE_TYPE == "SIX_STEP">
            uint16_t *finalPulse;
    </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->		
            uint16_t *durationms;
            RevUpCtrl_PhaseParams_t revUpPhase;
            uint8_t i;

            *rawSize = (uint16_t)RUC_MAX_PHASE_NUMBER*8U; 
            if (((*rawSize) + 2U) > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              for (i = 0; i <RUC_MAX_PHASE_NUMBER; i++)
              {
                (void)RUC_GetPhase( RevUpControl[motorID] ,i, &revUpPhase);
    <#if UNALIGNMENT_SUPPORTED>
                rpm = (int32_t *) &data[2U + (i*8U)];  //cstat !MISRAC2012-Rule-11.3
                *rpm = (((int32_t)revUpPhase.hFinalMecSpeedUnit) * U_RPM) / SPEED_UNIT; //cstat !MISRAC2012-Rule-11.3
    <#else>
                rpm32 = (((int32_t)revUpPhase.hFinalMecSpeedUnit) * U_RPM) / SPEED_UNIT; //cstat !MISRAC2012-Rule-11.3
                rpm16p = (int16_t *)&data[2U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *rpm16p = (uint16_t)rpm32; /* 16 LSB access */ //cstat !MISRAC2012-Rule-11.3
                *(rpm16p+1) = ((uint16_t)(rpm32 >> 16)); /* 16 MSB access */ //cstat !MISRAC2012-Rule-11.3
    </#if>
    <#if MC.DRIVE_TYPE == "FOC">
                finalTorque = (uint16_t *)&data[6U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *finalTorque = (uint16_t)revUpPhase.hFinalTorque; //cstat !MISRAC2012-Rule-11.3
    </#if><#-- MC.DRIVE_TYPE == "FOC" -->
    <#if MC.DRIVE_TYPE == "SIX_STEP">
                finalPulse = (uint16_t *)&data[6U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *finalPulse = (uint16_t)revUpPhase.hFinalPulse; //cstat !MISRAC2012-Rule-11.3
    </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->		  
                durationms  = (uint16_t *)&data[8U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *durationms  = revUpPhase.hDurationms;
              }
            }
            break;
          }
  </#if><#-- IS_SENSORLESS || IS_SENSORLESS2 -->

  <#if MC.DRIVE_TYPE == "FOC">
          case MC_REG_CURRENT_REF:
          {
            uint16_t *iqref = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *idref = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *iqref = (uint16_t)MCI_GetIqdref(pMCIN).q;
            *idref = (uint16_t)MCI_GetIqdref(pMCIN).d;
            break;
    }
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.POSITION_CTRL_ENABLING || MC.POSITION_CTRL_ENABLING2>
          case MC_REG_POSITION_RAMP:
          {
            float Position;
            float Duration;

            *rawSize = 8;
            Position = TC_GetMoveDuration(pPosCtrl[motorID]);   /* Does this duration make sense ? */
            Duration = TC_GetTargetPosition(pPosCtrl[motorID]);
            memcpy(rawData, &Position, 4);
            memcpy(&rawData[4], &Duration, 4);
            break;
          }
  </#if>
  <#if MC.MOTOR_PROFILER && MC.DRIVE_TYPE == "FOC" && MC.AUX_HALL_SENSORS>
          case MC_REG_HT_HEW_PINS:
            {
              *rawSize = 4;
              *rawData = HT.bNewH1;
              *(rawData+1) = HT.bNewH2;
              *(rawData+2) = HT.bNewH3;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }
          case MC_REG_HT_CONNECTED_PINS:
            {
              *rawSize = 4;
              *rawData = HT.H1Connected;
              *(rawData+1) = HT.H2Connected;
              *(rawData+2) = HT.H3Connected;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }
          case MC_REG_HT_PHASE_SHIFT:
            {
              int16_t *rawData16 = (int16_t *) rawData; 
              *rawSize = 28; /*14 16 bits values  */              
              *rawData16 = HT.hPhaseShiftCircularMean;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_5;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMeanNeg;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_5;
              break;
            }
  </#if> <#-- MC.MOTOR_PROFILER && MC.DRIVE_TYPE == "FOC" && MC.AUX_HALL_SENSORS --> 
</#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size*/
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}
<#if MEMORY_FOOTPRINT_REG>
uint8_t RI_MovString(const char_t *srcString, char_t *destString, uint16_t *size, int16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  
  const char_t *tempsrcString = srcString;
  char_t *tempdestString = destString;
  *size= 1U ; /* /0 is the min String size */

  while ((*tempsrcString != (char_t)0) && (*size < maxSize))
  {
    *tempdestString = *tempsrcString;
    tempdestString++;
    tempsrcString++;
    *size = *size + 1U;
  }
  
  if (*tempsrcString != (char_t)0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *tempdestString = (int8_t)0;
  }
  
  return (retVal);
}
</#if><#-- MEMORY_FOOTPRINT_REG -->
uint8_t RI_GetIDSize(uint16_t dataID)
{
  uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
  uint8_t result;
  switch (typeID)
  {
    case TYPE_DATA_8BIT:
    {
      result = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      result = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      result = 4;
      break;
    }

    default:
    {
      result=0;
      break;
    }
  }
  
  return (result);
}

__weak uint8_t RI_GetPtrReg(uint16_t dataID, void **dataPtr)
{
  uint8_t retVal = MCP_CMD_OK;
  static uint16_t nullData16=0;

#ifdef NULL_PTR_REG_INT  
  if (MC_NULL == dataPtr)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {    
#endif

<#if MC.DUALDRIVE == true>
    uint8_t vmotorID = (((uint8_t)dataID) -1U) & MOTOR_MASK;
    motorID = (vmotorID <= (MAX_OF_MOTORS - 1U)) ? vmotorID : (MAX_OF_MOTORS - 1U);
<#else>
    uint8_t vmotorID = 0U; 
</#if>  

    MCI_Handle_t *pMCIN = &Mci[vmotorID];
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;

    switch (typeID)
    {
      case TYPE_DATA_16BIT:
      {
        switch (regID)
        {
<#if MC.DRIVE_TYPE == "FOC">
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_A:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.a);
             break;
          }

          case MC_REG_I_B:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.b);
            break;
          }

          case MC_REG_I_ALPHA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.alpha);
            break;
          }

          case MC_REG_I_BETA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.beta);
            break;
          }

          case MC_REG_I_Q_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.q);
            break;
          }

          case MC_REG_I_D_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.d);
            break;
          }

          case MC_REG_I_Q_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.q);
            break;
          }

          case MC_REG_I_D_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.d);
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_V_Q:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.q);
            break;
          }

          case MC_REG_V_D:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.d);
            break;
          }

          case MC_REG_V_ALPHA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.alpha);
            break;
          }

          case MC_REG_V_BETA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.beta);
            break;
          }
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MEMORY_FOOTPRINT_REG>
  <#if MC.DRIVE_TYPE == "SIX_STEP">

          case MC_REG_PULSE_VALUE:
          {
            *dataPtr = &(pMCIN->pSixStepVars->DutyCycleRef);
            break;
          }
    <#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">		  
          case MC_REG_BEMF_ZCR:
          {
            *dataPtr = &(pBemfADC[vmotorID]->ZcDetected);
            break;
          }

          case MC_REG_BEMF_U:
          {
            *dataPtr = &(pBemfADC[vmotorID]->BemfValues.LastValues[0]);
            break;
          }

          case MC_REG_BEMF_V:
          {		  
            *dataPtr = &(pBemfADC[vmotorID]->BemfValues.LastValues[1]);
            break;
          }

          case MC_REG_BEMF_W:
          {
            *dataPtr = &(pBemfADC[vmotorID]->BemfValues.LastValues[2]);
            break;
          }
    </#if>
  </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->  

  <#if IS_HALL_SENSORS || IS_HALL_SENSORS2>
          case MC_REG_HALL_SPEED:
          {
            *dataPtr = &(pHallSensor[vmotorID]->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_HALL_EL_ANGLE:
          {
            *dataPtr = &(pHallSensor[vmotorID]->_Super.hElAngle);
            break;
          }
  </#if>
</#if><#-- MEMORY_FOOTPRINT_REG -->

<#if IS_ENCODER || IS_ENCODER2>
          case MC_REG_ENCODER_SPEED:
          {
            *dataPtr = &(pEncoder[vmotorID]->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_ENCODER_EL_ANGLE:
          {
            *dataPtr = &(pEncoder[vmotorID]->_Super.hElAngle);
            break;
          }
</#if>

<#if IS_STO_PLL || IS_STO_PLL2>
#ifdef NOT_IMPLEMENTED  /* Not yet Implemented */
         stoPLLSensor[vmotorID];
#endif
          case MC_REG_STOPLL_ROT_SPEED:
          {
            *dataPtr = &(stoPLLSensor[vmotorID]->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOPLL_EL_ANGLE:
          {
            *dataPtr = &(stoPLLSensor[vmotorID]->_Super.hElAngle);
            break;
          }
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOPLL_I_ALPHA:
          case MC_REG_STOPLL_I_BETA:
            break;
#endif
          case MC_REG_STOPLL_BEMF_ALPHA:
          {
            *dataPtr = &(stoPLLSensor[vmotorID]->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOPLL_BEMF_BETA:
          {
            *dataPtr = &(stoPLLSensor[vmotorID]->hBemf_beta_est);
            break;
          }
</#if>
<#if IS_STO_CORDIC || IS_STO_CORDIC2>
          case MC_REG_STOCORDIC_ROT_SPEED:
          {
            *dataPtr = &(stoCRSensor[vmotorID]->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOCORDIC_EL_ANGLE:
          {
            *dataPtr = &(stoCRSensor[vmotorID]->_Super.hElAngle);
            break;
          }
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOCORDIC_I_ALPHA:
          case MC_REG_STOCORDIC_I_BETA:
            break;
#endif
          case MC_REG_STOCORDIC_BEMF_ALPHA:
          {
            *dataPtr = &(stoCRSensor[vmotorID]->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOCORDIC_BEMF_BETA:
          {
            *dataPtr = &(stoCRSensor[vmotorID]->hBemf_beta_est);
            break;
          }
</#if>
          default:
          {
            *dataPtr = &nullData16;
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        break;
      }
      
      default:
      {
        *dataPtr = &nullData16;
        retVal = MCP_ERROR_UNKNOWN_REG;
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }  
#endif
  return (retVal);
}


/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
