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

<#assign CondFamily_STM32F0 = (FamilyName?? && FamilyName=="STM32F0")>
<#-- Condition for STM32G0 Family -->
<#assign CondFamily_STM32G0 = (FamilyName?? && FamilyName=="STM32G0") >
<#assign DWT_CYCCNT_SUPPORTED = !(CondFamily_STM32F0 || CondFamily_STM32G0) >

/**
  ******************************************************************************
  * @file    mc_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC Interface component of the Motor Control SDK.
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
  * @ingroup MCInterface
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_INTERFACE_H
#define MC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
<#if MC.DRIVE_TYPE == "FOC">
#include "pwm_curr_fdbk.h"
#include "speed_torq_ctrl.h"
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
#include "mc_perf.h"
</#if>
</#if>
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
#include "pwm_common_sixstep.h"
#include "speed_ctrl.h"
</#if>
<#if MC.DRIVE_TYPE == "FOC">
<#if  MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>
#include "trajectory_ctrl.h"
</#if>
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
#include "virtual_speed_sensor.h"
</#if>
</#if>
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MCI_BUFFER_EMPTY,                  /*!< If no buffered command has been
                                            called.*/
  MCI_COMMAND_NOT_ALREADY_EXECUTED,  /*!< If the buffered command condition
                                            hasn't already occurred.*/
  MCI_COMMAND_EXECUTED_SUCCESFULLY,  /*!< If the buffered command has been
                                            executed successfully.*/
  MCI_COMMAND_EXECUTED_UNSUCCESFULLY /*!< If the buffered command has been
                                            executed unsuccessfully.*/
} MCI_CommandState_t ;

typedef enum
{
  MCI_NOCOMMANDSYET,        /*!< No command has been set by the user.*/
  MCI_CMD_EXECSPEEDRAMP,        /*!< ExecSpeedRamp command coming from the user.*/
  MCI_CMD_EXECTORQUERAMP,       /*!< ExecTorqueRamp command coming from the user.*/
  MCI_CMD_SETCURRENTREFERENCES, /*!< SetCurrentReferences command coming from the
                                 user.*/
  MCI_CMD_SETOPENLOOPCURRENT, /*!< set open loop current .*/
  MCI_CMD_SETOPENLOOPVOLTAGE, /*!< set open loop voltage .*/
} MCI_UserCommands_t;

/**
  * @brief  State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  ICLWAIT = 12,         /*!< Persistent state, the system is waiting for ICL
                           deactivation. Is not possible to run the motor if
                           ICL is active. Until the ICL is active the state is
                           forced to ICLWAIT, when ICL become inactive the state
                           is moved to IDLE */
  IDLE = 0,             /*!< Persistent state, following state can be IDLE_START
                           if a start motor command has been given or
                           IDLE_ALIGNMENT if a start alignment command has been
                           given */
  ALIGNMENT = 2,        /*!< Persistent state in which the encoder are properly
                           aligned to set mechanical angle, following state can
                           only be ANY_STOP */
  CHARGE_BOOT_CAP = 16, /*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           OFFSET_CALIB. It can also be ANY_STOP if a stop motor
                           command has been given. */
  OFFSET_CALIB = 17,    /*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  START = 4,            /*!< Persistent state where the motor start-up is intended
                           to be executed. The following state is normally
                           SWITCH_OVER or RUN as soon as first validated speed is
                           detected. Another possible following state is
                           ANY_STOP if a stop motor command has been executed */
  SWITCH_OVER = 19,     /**< TBD */
  RUN = 6,              /*!< Persistent state with running motor. The following
                           state is normally ANY_STOP when a stop motor command
                           has been executed */
  STOP = 8,             /*!< Persistent state. Following state is normally
                           STOP_IDLE as soon as conditions for moving state
                           machine are detected */
  FAULT_NOW = 10,       /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by
                           STM_FaultProcessing method. This method also manage
                           the passage to the only allowed following state that
                           is FAULT_OVER */
  FAULT_OVER = 11,       /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. Following
                          state is normally STOP_IDLE, state machine is moved as
                          soon as the user has acknowledged the fault condition.
                      */
  WAIT_STOP_MOTOR = 20

} MCI_State_t;


typedef enum
{
  MCI_NO_COMMAND = 0,    /**< No Command --- Set when going to IDLE */
  MCI_START,            /**< Start controling the Motor */
  MCI_ACK_FAULTS,       /**< Acknowledge Motor Control subsystem faults */
  MCI_MEASURE_OFFSETS,  /**< Start the ADCs Offset measurements procedure */
  /* Shouldn't we remove this command ? */ 
  MCI_ALIGN_ENCODER,    /**< Start the Encoder alignment procedure */
  MCI_STOP              /**< Stop the Motor and the control */
} MCI_DirectCommands_t;

typedef struct
{
  SpeednTorqCtrl_Handle_t * pSTC; /*!< Speed and torque controller object used by MCI.*/
<#if MC.DRIVE_TYPE == "FOC">
  pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MCI.*/
</#if><#--<#if MC.DRIVE_TYPE == "FOC">-->
<#if MC.DRIVE_TYPE == "SIX_STEP">
  pSixStepVars_t pSixStepVars;    /*!< Pointer to FOC vars used by MCI.*/
</#if><#--<#if MC.DRIVE_TYPE == "SIEX_STEP">-->
  PWMC_Handle_t *pPWM;    /*!< Pointer to PWM handle structure.*/
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  VirtualSpeedSensor_Handle_t * pVSS;    
</#if>  
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Handle_t  * pPerfMeasure;
</#if>
</#if>
<#if  MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>  
  PosCtrl_Handle_t * pPosCtrl; /*!< Position Control used by MCI.*/
</#if>
  MCI_UserCommands_t lastCommand; /*!< Last command coming from the user.*/
  int16_t hFinalSpeed;        /*!< Final speed of last ExecSpeedRamp command.*/
  int16_t hFinalTorque;       /*!< Final torque of last ExecTorqueRamp
                                   command.*/
<#if MC.DRIVE_TYPE == "FOC">
  qd_t Iqdref;     /*!< Current component of last
                                   SetCurrentReferences command.*/
</#if><#--<#if MC.DRIVE_TYPE == "FOC">-->
  uint16_t hDurationms;       /*!< Duration in ms of last ExecSpeedRamp or
                                   ExecTorqueRamp command.*/
 MCI_DirectCommands_t DirectCommand;
 MCI_State_t State;  
 uint16_t CurrentFaults;
 uint16_t PastFaults;  
 MCI_CommandState_t CommandState; /*!< The status of the buffered command.*/
 MC_ControlMode_t LastModalitySetByUser; /*!< The last MC_ControlMode_t set by the
                                             user. */
} MCI_Handle_t;

/* Exported functions ------------------------------------------------------- */
<#if  MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>  
void MCI_Init( MCI_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTC, pFOCVars_t pFOCVars, PosCtrl_Handle_t * pPosCtrl, PWMC_Handle_t *pPWMHandle );
void MCI_ExecPositionCommand( MCI_Handle_t * pHandle, float FinalPosition, float Duration );
PosCtrlStatus_t MCI_GetCtrlPositionState( MCI_Handle_t * pHandle );
AlignStatus_t  MCI_GetAlignmentStatus( MCI_Handle_t * pHandle );
float MCI_GetCurrentPosition( MCI_Handle_t * pHandle );
float MCI_GetTargetPosition( MCI_Handle_t * pHandle );
float MCI_GetMoveDuration( MCI_Handle_t * pHandle );
<#else>
<#if MC.DRIVE_TYPE == "FOC">
void MCI_Init( MCI_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTC, pFOCVars_t pFOCVars, PWMC_Handle_t *pPWMHandle );
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
void MCI_Init( MCI_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTC, pSixStepVars_t pSixStepVars, PWMC_Handle_t *pPWMHandle );
</#if>
</#if>
void MCI_ExecBufferedCommands( MCI_Handle_t * pHandle );
void MCI_ExecSpeedRamp( MCI_Handle_t * pHandle,  int16_t hFinalSpeed, uint16_t hDurationms );
void MCI_ExecSpeedRamp_F( MCI_Handle_t * pHandle, const float FinalSpeed, uint16_t hDurationms );

<#if MC.DRIVE_TYPE == "FOC">
void MCI_ExecTorqueRamp( MCI_Handle_t * pHandle,  int16_t hFinalTorque, uint16_t hDurationms );
void MCI_ExecTorqueRamp_F( MCI_Handle_t * pHandle, const float FinalTorque, uint16_t hDurationms );

void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, qd_t Iqdref );
void MCI_SetCurrentReferences_F( MCI_Handle_t * pHandle, qd_f_t Iqdref );

void MCI_SetIdref( MCI_Handle_t * pHandle, int16_t hNewIdref );
void MCI_SetIdref_F( MCI_Handle_t * pHandle, float NewIdRef );
</#if>
bool MCI_StartMotor( MCI_Handle_t * pHandle );
<#if MC.DRIVE_TYPE == "FOC">
bool MCI_StartOffsetMeasurments(MCI_Handle_t *pHandle);
bool MCI_GetCalibratedOffsetsMotor(MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets);
bool MCI_SetCalibratedOffsetsMotor( MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets);
</#if>
bool MCI_StopMotor( MCI_Handle_t * pHandle );
bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle );
void MCI_FaultProcessing(  MCI_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                                  hResetErrors );
uint32_t MCI_GetFaultState( MCI_Handle_t * pHandle );                                  
MCI_CommandState_t  MCI_IsCommandAcknowledged( MCI_Handle_t * pHandle );
MCI_State_t MCI_GetSTMState( MCI_Handle_t * pHandle );
uint16_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle );
uint16_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle );
float MCI_GetMecSpeedRef_F( MCI_Handle_t * pHandle );
float MCI_GetAvrgMecSpeed_F( MCI_Handle_t * pHandle );
MC_ControlMode_t MCI_GetControlMode( MCI_Handle_t * pHandle );
int16_t MCI_GetImposedMotorDirection( MCI_Handle_t * pHandle );
int16_t MCI_GetLastRampFinalSpeed( MCI_Handle_t * pHandle );
<#if MC.DRIVE_TYPE == "FOC">
int16_t MCI_GetLastRampFinalTorque( MCI_Handle_t * pHandle );
</#if>
uint16_t MCI_GetLastRampFinalDuration( MCI_Handle_t * pHandle );
bool MCI_RampCompleted( MCI_Handle_t * pHandle );
float MCI_GetLastRampFinalSpeed_F( MCI_Handle_t * pHandle );
bool MCI_RampCompleted( MCI_Handle_t * pHandle );
bool MCI_StopSpeedRamp( MCI_Handle_t * pHandle );
void MCI_StopRamp( MCI_Handle_t * pHandle );
bool MCI_GetSpdSensorReliability( MCI_Handle_t * pHandle );
int16_t MCI_GetAvrgMecSpeedUnit( MCI_Handle_t * pHandle );
int16_t MCI_GetMecSpeedRefUnit( MCI_Handle_t * pHandle );
<#if MC.DRIVE_TYPE == "FOC">
ab_t MCI_GetIab( MCI_Handle_t * pHandle );
ab_f_t MCI_GetIab_F( MCI_Handle_t * pHandle );
alphabeta_t MCI_GetIalphabeta( MCI_Handle_t * pHandle );
qd_t MCI_GetIqd( MCI_Handle_t * pHandle );
qd_f_t MCI_GetIqd_F( MCI_Handle_t * pHandle );
qd_t MCI_GetIqdHF( MCI_Handle_t * pHandle );
qd_t MCI_GetIqdref( MCI_Handle_t * pHandle );
qd_f_t MCI_GetIqdref_F( MCI_Handle_t * pHandle );
qd_t MCI_GetVqd( MCI_Handle_t * pHandle );
alphabeta_t MCI_GetValphabeta( MCI_Handle_t * pHandle );
int16_t MCI_GetElAngledpp( MCI_Handle_t * pHandle );
int16_t MCI_GetTeref( MCI_Handle_t * pHandle );
float MCI_GetTeref_F( MCI_Handle_t * pHandle );
int16_t MCI_GetPhaseCurrentAmplitude( MCI_Handle_t * pHandle );
int16_t MCI_GetPhaseVoltageAmplitude( MCI_Handle_t * pHandle );
void MCI_Clear_Iqdref( MCI_Handle_t * pHandle );
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
void MCI_SetSpeedMode( MCI_Handle_t * pHandle );
void MCI_SetOpenLoopCurrent( MCI_Handle_t * pHandle );
void MCI_SetOpenLoopVoltage( MCI_Handle_t * pHandle );
</#if>
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
uint16_t MCI_GetDutyCycleRef( MCI_Handle_t * pHandle );
</#if><#--<#if MC.DRIVE_TYPE == "SIX_STEP">-->
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_INTERFACE_H */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

