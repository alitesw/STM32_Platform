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

<#assign AUX_SPEED_FDBK_M1 = (MC.AUX_HALL_SENSORS == true || MC.AUX_ENCODER || MC.AUX_STATE_OBSERVER_PLL || MC.AUX_STATE_OBSERVER_CORDIC)>
<#assign AUX_SPEED_FDBK_M2 = (MC.AUX_HALL_SENSORS2 == true || MC.AUX_ENCODER2 || MC.AUX_STATE_OBSERVER_PLL2 || MC.AUX_STATE_OBSERVER_CORDIC2)>

<#if  AUX_SPEED_FDBK_M1 == true>
  <#if   MC.AUX_STATE_OBSERVER_PLL == true>
    <#assign SPD_AUX_M1 = "&STO_PLL_M1._Super">
  <#elseif  MC.AUX_STATE_OBSERVER_CORDIC == true>
    <#assign SPD_AUX_M1 = "&STO_CR_M1._Super">
  <#elseif  MC.AUX_HALL_SENSORS == true>
    <#assign SPD_AUX_M1 = "&HALL_M1._Super">
  <#elseif  MC.AUX_ENCODER == true>
    <#assign SPD_AUX_M1 = "&ENCODER_M1._Super">
  </#if>
</#if>
<#if  AUX_SPEED_FDBK_M2 == true>
  <#if   MC.AUX_STATE_OBSERVER_PLL2 == true>
    <#assign SPD_AUX_M2 = "&STO_PLL_M2._Super">
  <#elseif  MC.AUX_STATE_OBSERVER_CORDIC2 == true>
    <#assign SPD_AUX_M2 = "&STO_CR_M2._Super">
  <#elseif  MC.AUX_HALL_SENSORS2 == true>
    <#assign SPD_AUX_M2 = "&HALL_M2._Super">
  <#elseif  MC.AUX_ENCODER2 == true>
    <#assign SPD_AUX_M2 = "&ENCODER_M2._Super">
  </#if>
</#if>
/**
  ******************************************************************************
  * @file    mc_api.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the high level interface of the Motor Control SDK.
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
  
#include "mc_interface.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mcp.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCIAPI Motor Control API
  *
  * @brief High level Programming Interface of the Motor Control SDK
  *
  *  This interface allows for performing basic operations on the motor(s) driven by an
  * Motor Control SDK based application. With it, motors can be started and stopped, speed or
  * torque ramps can be programmed and executed and information on the state of the motors can
  * be retrieved, among others.
  *
  *  This interface consists in functions that target a specific motor, indicated in their name. 
  * These functions aims at being the main interface used by an Application to control motors.
  *
  *  The current Motor Control API can cope with up to 2 motors.  
  * @{
  */

/**
  * @brief  Initiates the start-up procedure for Motor 1
  * 
  * If the state machine of Motor 1 is in #IDLE state, the command is immediately
  * executed. Otherwise the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *
  * One of the following commands must be executed before calling MC_StartMotor1():
  *
  * - MC_ProgramSpeedRampMotor1()
  * - MC_ProgramTorqueRampMotor1()
  * - MC_SetCurrentReferenceMotor1()
  *
  * Failing to do so results in an unpredictable behaviour.
  *
  * @note The MC_StartMotor1() command only triggers the start-up procedure: 
  * It moves Motor 1's state machine from the #IDLE to the #IDLE_START state and then 
  * returns. It is not blocking the application until the motor is indeed running. 
  * To know if it is running, the application can query Motor 1's state machine and 
  * check if it has reached the #RUN state. See MC_GetSTMStateMotor1() for more details.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StartMotor1(void)
{
	return MCI_StartMotor( pMCI[M1] );
}

/**
  * @brief  Initiates the stop procedure for Motor 1.
  *
  *  If the state machine is in #RUN or #START states the command is immediately
  * executed. Otherwise, the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *  
  * @note The MCI_StopMotor1() command only triggers the stop motor procedure 
  * moving Motor 1's state machine to #ANY_STOP and then returns. It is not 
  * blocking the application until the motor is indeed stopped. To know if it has
  * stopped, the application can query Motor 1's state machine ans check if the 
  * #IDLE state has been reached back.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StopMotor1(void)
{
	return MCI_StopMotor( pMCI[M1] );
}

/**
  * @brief Programs a speed ramp for Motor 1 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p hFinalSpeed 
  * target speed in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramSpeedRampMotor1() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter 
  * differs from that of the current speed, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *                     Expressed in the unit defined by #SPEED_UNIT.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp( pMCI[M1], hFinalSpeed, hDurationms );
}

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief Programs a speed ramp for Motor 1 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p FinalSpeed 
  * target speed in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramSpeedRampMotor1() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter 
  * differs from that of the current speed, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  FinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *         Expressed in rpm.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor1_F( float FinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp_F( pMCI[M1], FinalSpeed, hDurationms );
}

/**
  * @brief Programs a torque ramp for Motor 1 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p hFinalTorque 
  * target torque reference in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramTorqueRampMotor1() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalTorque parameter 
  * differs from that of the current torque, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in digit.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp( pMCI[M1], hFinalTorque, hDurationms );
}

/**
  * @brief Programs a torque ramp for Motor 1 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p FinalTorque 
  * target torque reference in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramTorqueRampMotor1() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the FinalTorque parameter
  * differs from that of the current torque, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  FinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in Ampere.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor1_F( float FinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp_F( pMCI[M1], FinalTorque, hDurationms );
}
</#if>

<#if MC.POSITION_CTRL_ENABLING == true>
/**
  * @brief Programs a position command for Motor 1 for later or immediate execution.
  *
  *  A position command allows to program a specific movement from current position to
  *  the @p fTargetPosition target mechanical angle reference in the given @p fDuration time. 
  *
  *  Invoking the MC_ProgramPositionCommandMotor1() function programs a target position
  * with the provided parameters. The programmed position is executed immediately if
  * there is no movement ongoing.
  *
  *  The Application can check the status of the control with the MC_GetControlPositionStatusMotor1()
  * to know whether the motor is ready to execute a new command or not.
  *
  * @note when the duration is set to 0, the input target position is immediately executed.
  * By calling repeatedly the command with an update of input target angle, the control will be in follow mode.
  * 
  * 
  * @param  fTargetPosition Target mechanical angle reference at the end of the movement.
  *         This value represents the final position expressed in radian.
  * @param  fDuration Duration of the movement expressed in seconds.
  *         It is possible to set 0 to perform an instantaneous angular change (follow mode).
  */
__weak void MC_ProgramPositionCommandMotor1( float fTargetPosition, float fDuration )
{
	MCI_ExecPositionCommand( pMCI[M1], fTargetPosition, fDuration );
}
</#if>

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components. 
  *
  *  Invoking the MC_SetCurrentReferenceMotor1() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * command is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is 
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed 
  *         in the qd_t format.
  */
__weak void MC_SetCurrentReferenceMotor1( qd_t Iqdref )
{
	MCI_SetCurrentReferences( pMCI[M1], Iqdref );
}

/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components. 
  *
  *  Invoking the MC_SetCurrentReferenceMotor1_F() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * command is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is 
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed 
  *         in the qd_f_t format.
  */
__weak void MC_SetCurrentReferenceMotor1_F( qd_f_t IqdRef )
{
	MCI_SetCurrentReferences_F( pMCI[M1], IqdRef );
}
</#if>

<#if MC.DRIVE_TYPE == "SIX_STEP">
/**
 * @brief returns the electrical torque reference for Motor 1
 */
__weak int16_t MCI_GetDutyCycleRefMotor1(void)
{
	return MCI_GetDutyCycleRef( pMCI[M1] );
}
</#if>

/**
  * @brief  Returns the status of the last buffered command for Motor 1.
  * The status can be one of the following values:
  * - #MCI_BUFFER_EMPTY: no buffered command is currently programmed.
  * - #MCI_COMMAND_NOT_ALREADY_EXECUTED: A command has been buffered but the conditions for its
  *   execution have not occurred yet. The command is still in the buffer, pending execution.
  * - #MCI_COMMAND_EXECUTED_SUCCESFULLY: the last buffered command has been executed successfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  * - #MCI_COMMAND_EXECUTED_UNSUCCESFULLY: the buffered command has been executed unsuccessfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  */
__weak MCI_CommandState_t  MC_GetCommandStateMotor1( void)
{
	return MCI_IsCommandAcknowledged( pMCI[M1] );
}

/** 
 * @brief Stops the execution of the on-going speed ramp for Motor 1, if any.
 *
 *  If a speed ramp is currently being executed, it is immediately stopped, the rotation
 * speed of Motor 1 is maintained to its current value and true is returned. If no speed 
 * ramp is on-going, nothing is done and false is returned.
 *
 * @deprecated This function is deprecated and should not be used anymore. It will be 
 *             removed in a future version of the MCSDK. Use MC_StopRampMotor1() instead.
 */
__weak bool MC_StopSpeedRampMotor1(void)
{
	return MCI_StopSpeedRamp( pMCI[M1] );
}

/** 
 * @brief Stops the execution of the on-going ramp for Motor 1, if any.
 *
 *  If a ramp is currently being executed, it is immediately stopped, the torque or the speed
 *  of Motor 1 is maintained to its current value.
 */
__weak void MC_StopRampMotor1(void)
{
  MCI_StopRamp( pMCI[M1] );
}

/**
 * @brief Returns true if the last ramp submited for Motor 1 has completed, false otherwise 
 */
__weak bool MC_HasRampCompletedMotor1(void)
{
	return MCI_RampCompleted( pMCI[M1] );
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 1, expressed in the unit defined by #SPEED_UNIT 
 */
__weak int16_t MC_GetMecSpeedReferenceMotor1(void)
{
	return MCI_GetMecSpeedRefUnit( pMCI[M1] );
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 1, expressed in rpm.
 */
//__weak float MC_GetMecSpeedReferenceMotor1_F(void)
//{
//	return MCI_GetMecSpeedRef_F( pMCI[M1] );
//}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 1, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecSpeedAverageMotor1(void)
{
	return MCI_GetAvrgMecSpeedUnit( pMCI[M1] );
}

<#if  AUX_SPEED_FDBK_M1 == true || AUX_SPEED_FDBK_M2 == true>
#define S16ToRAD 10430.37835f            /* 2^16/2Pi */
</#if>

<#if  AUX_SPEED_FDBK_M1 == true>
/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecAuxiliarySpeedAverageMotor1(void)
{
	return SPD_GetAvrgMecSpeedUnit(${SPD_AUX_M1});
}

/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in RPM
 */
__weak float MC_GetMecAuxiliarySpeedAverageMotor1_F(void)
{
  return( (float) ( SPD_GetAvrgMecSpeedUnit(${SPD_AUX_M1}) * U_RPM) / SPEED_UNIT);
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, in DDP format
 */
__weak int16_t MC_GetAuxiliaryElAngledppMotor1(void)
{
	return (SPD_GetElAngle(${SPD_AUX_M1}));
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, expressed in radians 
 */
__weak float MC_GetAuxiliaryElAngleMotor1_F(void)
{
  return ((float)( (SPD_GetElAngle(${SPD_AUX_M1})) / S16ToRAD) );
}
</#if>

<#if  AUX_SPEED_FDBK_M2 == true>
/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 2, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecAuxiliarySpeedAverageMotor2(void)
{
	return SPD_GetAvrgMecSpeedUnit(${SPD_AUX_M2});
}

/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 2, expressed in RPM
 */
__weak float MC_GetMecAuxiliarySpeedAverageMotor2_F(void)
{
	return( (float) ( SPD_GetAvrgMecSpeedUnit(${SPD_AUX_M2}) * U_RPM) / SPEED_UNIT);
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 2, in DDP format
 */
__weak int16_t MC_GetAuxiliaryElAngledppMotor2(void)
{
	return (SPD_GetElAngle(${SPD_AUX_M2}));
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 2, expressed in radians
 */
__weak float MC_GetAuxiliaryElAngleMotor2_F(void)
{
	return ((float)( (SPD_GetElAngle(${SPD_AUX_M2})) / S16ToRAD) );
}
</#if>

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 1, expressed in rpm.
 */
//__weak float MC_GetAverageMecSpeedMotor1_F(void)
//{
//	return MCI_GetAvrgMecSpeed_F( pMCI[M1] );
//}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 1 if this ramp was a speed ramp, 0 otherwise.
 */
__weak int16_t MC_GetLastRampFinalSpeedMotor1(void)
{
	return MCI_GetLastRampFinalSpeed( pMCI[M1] );
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 1 if this ramp was a speed ramp, 0 otherwise.
 */
__weak float MC_GetLastRampFinalSpeedMotor1_F(void)
{
	return MCI_GetLastRampFinalSpeed_F( pMCI[M1] );
}

/**
 * @brief Returns the Control Mode used for Motor 1 (either Speed or Torque)
 */
__weak MC_ControlMode_t MC_GetControlModeMotor1(void)
{
	return MCI_GetControlMode( pMCI[M1] );
}

/**
 * @brief Returns the rotation direction imposed by the last command on Motor 1 
 *
 * The last command is either MC_ProgramSpeedRampMotor1(), MC_ProgramTorqueRampMotor1() or
 * MC_SetCurrentReferenceMotor1(). 
 *
 * The function returns -1 if the sign of the final speed, the final torque or the Iq current 
 * reference component of the last command is negative. Otherwise, 1 is returned. 
 *   
 * @note if no such command has ever been submitted, 1 is returned as well.  
 */
__weak int16_t MC_GetImposedDirectionMotor1(void)
{
	return MCI_GetImposedMotorDirection( pMCI[M1] );
}

/** 
 * @brief Returns true if the speed sensor used for Motor 1 is reliable, false otherwise 
 */
__weak bool MC_GetSpeedSensorReliabilityMotor1(void)
{
	return MCI_GetSpdSensorReliability( pMCI[M1] );
}

<#if MC.DRIVE_TYPE == "FOC">
/** 
 * @brief returns the amplitude of the phase current injected in Motor 1
 *
 * The returned amplitude (0-to-peak) is expressed in s16A unit. To convert it to amperes, use the following formula:
 *
 * @f[
 * I_{Amps} = \frac{ I_{s16A} \times V_{dd}}{ 65536 \times R_{shunt} \times A_{op} }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseCurrentAmplitudeMotor1(void)
{
	return MCI_GetPhaseCurrentAmplitude( pMCI[M1] );
}

/**
 * @brief returns the amplitude of the phase voltage applied to Motor 1 
 *
 * The returned amplitude (0-to-peak) is expressed in s16V unit. To convert it to volts, use the following formula:
 *
 * @f[
 * U_{Volts} = \frac{ U_{s16V} \times V_{bus}}{ \sqrt{3} \times 32768  }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseVoltageAmplitudeMotor1(void)
{
	return MCI_GetPhaseVoltageAmplitude( pMCI[M1] );
}

/**
 * @brief returns Ia and Ib current values for Motor 1 in ab_t format
 */
__weak ab_t MC_GetIabMotor1(void)
{
	return MCI_GetIab( pMCI[M1] );
}

/**
 * @brief returns Ia and Ib current values for Motor 1 in ab_f_t format
 */
__weak ab_f_t MC_GetIabMotor1_F(void)
{
	return MCI_GetIab_F( pMCI[M1] );
}

/**
 * @brief returns Ialpha and Ibeta current values for Motor 1 in alphabeta_t format
 */
__weak alphabeta_t MC_GetIalphabetaMotor1(void)
{
	return MCI_GetIalphabeta( pMCI[M1] );
}

/**
 * @brief returns Iq and Id current values for Motor 1 in qd_t format 
 */
__weak qd_t MC_GetIqdMotor1(void)
{
	return MCI_GetIqd( pMCI[M1] );
}

/**
 * @brief returns Iq and Id current values for Motor 1 in float type
 */
__weak qd_f_t MC_GetIqdMotor1_F(void)
{
	return MCI_GetIqd_F( pMCI[M1] );
}

/**
 * @brief returns Iq and Id reference current values for Motor 1 in qd_t format
 */
__weak qd_t MC_GetIqdrefMotor1(void)
{
	return MCI_GetIqdref( pMCI[M1] );
}

/**
 * @brief returns Iq and Id reference current values for Motor 1 in float type
 */
__weak qd_f_t MC_GetIqdrefMotor1_F(void)
{
	return MCI_GetIqdref_F( pMCI[M1] );
}

/**
 * @brief returns Vq and Vd voltage values for Motor 1 in qd_t format
 */
__weak qd_t MC_GetVqdMotor1(void)
{
	return MCI_GetVqd( pMCI[M1] );
}

/**
 * @brief returns Valpha and Vbeta voltage values for Motor 1 in alphabeta_t format 
 */
__weak alphabeta_t MC_GetValphabetaMotor1(void)
{
	return MCI_GetValphabeta( pMCI[M1] );
}

/**
 * @brief returns the electrical angle of the rotor of Motor 1, in DDP format 
 */
__weak int16_t MC_GetElAngledppMotor1(void)
{
	return MCI_GetElAngledpp( pMCI[M1] );
}

/**
 * @brief returns the electrical torque reference for Motor 1 
 */
__weak int16_t MC_GetTerefMotor1(void)
{
	return MCI_GetTeref( pMCI[M1] );
}

/**
 * @brief returns the electrical torque reference for Motor 1 
 */
__weak float MC_GetTerefMotor1_F(void)
{
	return MCI_GetTeref_F( pMCI[M1] );
}

/**
 * @brief re-initializes Iq and Id references to their default values for Motor 1
 *
 * The default values for the Iq and Id references are coming from the Speed
 * or the Torque controller depending on the control mode. 
 *
 * @see   SpeednTorqCtrl for more details.
 */
__weak void MC_Clear_IqdrefMotor1(void)
{
	MCI_Clear_Iqdref( pMCI[M1] );
}
</#if>

/**
 * @brief Acknowledge a Motor Control fault that occured on Motor 1
 *
 *  This function informs Motor 1's state machine that the Application has taken
 * the error condition that occured into account. If no error condition exists when
 * the function is called, nothing is done and false is returned. Otherwise, true is
 * returned. 
 */
__weak bool MC_AcknowledgeFaultMotor1( void )
{
	return MCI_FaultAcknowledged( pMCI[M1] );
}

/**
 * @brief Returns a bitfiled showing "new" faults that occured on Motor 1
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
__weak uint16_t MC_GetOccurredFaultsMotor1(void)
{
	return MCI_GetOccurredFaults( pMCI[M1] );
}

/**
 * @brief returns a bitfield showing all current faults on Motor 1 
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that are currently active.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
__weak uint16_t MC_GetCurrentFaultsMotor1(void)
{
	return MCI_GetCurrentFaults( pMCI[M1] );
}

/**
 * @brief returns the current state of Motor 1 state machine
 */
__weak MCI_State_t  MC_GetSTMStateMotor1(void)
{
	return MCI_GetSTMState( pMCI[M1] );
}

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief Sets the polarization offset values to use for Motor 1
  *
  * The Motor Control algorithm relies on a number of current and voltage measures. The hardware
  * parts that make these measurements need to be characterized at least once in the course of
  * product life, prior to its first activation. This characterization consists in measuring the
  * voltage presented to the ADC channels when either no current flows into the phases of the motor
  * or no voltage is applied to them. This characterization is named polarization offsets measurement
  * and its results are the polarization offsets.
  *
  * The Motor Control Firmware can performs this polarization offsets measurement procedure which
  * results in a number of offset values that the application can store in a non volatile memory and
  * then set into the Motor Control subsystem at power-on or after a reset.
  *
  * The application uses this function to set the polarization offset values that the Motor Control
  * subsystem is to use in the current session. This function can only be used when the state machine
  * of the motor is in the #IDLE state in which case it returns #MC_SUCCESS. Otherwise, it does nothing
  * and returns the #MC_WRONG_STATE_ERROR error code.
  *
  *  The Motor Control subsystem needs to know the polarization offsets before the motor can be controlled.
  * The MC_SetPolarizationOffsetsMotor1() function provides a way to set these offsets. Alternatively, the
  * application can either:
  *
  *  * Execute the polarization offsets measurement procedure with a call to
  *    MC_StartPolarizationOffsetsMeasurementMotor1() after a reset or a power on;
  *  * Start the motor control with the MC_StartWithPolarizationMotor1() that will execute the procedure
  *    before actually starting the motor, on the first time it is called after a reset or a power on.
  *
  * When this function completes successfully, the state of the polarization offsets measurement procedure
  * is set to #COMPLETED. See MC_GetPolarizationState().
  *
  * @param PolarizationOffsets an pointer on a structure containing the offset values
  */
bool MC_SetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets )
{
	return( MCI_SetCalibratedOffsetsMotor( pMCI[M1], PolarizationOffsets ) );
}

/**
  * @brief Returns the polarization offset values measured or set for Motor 1
  *
  *  See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  *  If the Motor Control Firmware knows the polarization offset values, they are copied into the
  * @p PolarizationOffsets structure and #MC_SUCCESS is returned. Otherwise, nothing is done and
  * #MC_NO_POLARIZATION_OFFSETS_ERROR is returned.
  *
  * @param PolarizationOffsets an pointer on the structure into which the polarization offsets will be
  *        copied
  * @return #MC_SUCCESS if calibration data were present and could be copied into @p PolarizationOffsets,
  *         #MC_NO_POLARIZATION_OFFSETS_ERROR otherwise.
  */
bool MC_GetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets )
{
   return ( MCI_GetCalibratedOffsetsMotor( pMCI[M1], PolarizationOffsets) );
}

/**
  * @brief Starts the polarization offsets measurement procedure.
  *
  * See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  * If the Motor Control Firmware is in the #IDLE state, the procedure is started, the state machine
  * of the motor switches to #OFFSET_CALIB and #MC_SUCCESS is returned. Otherwise, nothing is done
  * and the #MC_WRONG_STATE_ERROR error code is returned.
  *
  * The polarization offsets measurement procedure is only triggered by this function and it is has not
  * completed when this function returns. The application can use the MC_GetPolarizationState()
  * function to query the state of the procedure.
  *
  * @see MC_GetPolarizationState()
  */
bool MC_StartPolarizationOffsetsMeasurementMotor1( void )
{
	return( MCI_StartOffsetMeasurments( pMCI[M1] ) );
}

/**
 * @brief This method is used to get the average measured motor power
 *        expressed in watt for Motor 1. 

 * @retval float The average measured motor power expressed in watt.
 */
__weak float MC_GetAveragePowerMotor1_F(void)
{
	return (PQD_GetAvrgElMotorPowerW(pMPM[M1]));
}
</#if>

<#if MC.POSITION_CTRL_ENABLING == true>
/**
 * @brief returns the current control position state of Motor 1.
 *   */
__weak PosCtrlStatus_t  MC_GetControlPositionStatusMotor1( void )
{
	return MCI_GetCtrlPositionState( pMCI[M1] );
}

/**
 * @brief returns the alignment state of Motor 1.
 *   */
__weak AlignStatus_t  MC_GetAlignmentStatusMotor1( void )
{
	return MCI_GetAlignmentStatus( pMCI[M1] );
}

/**
 * @brief returns the current position of Motor 1.
 *   */
__weak float MC_GetCurrentPosition1( void )
{
	return MCI_GetCurrentPosition( pMCI[M1] );
}

/**
 * @brief returns the target position of Motor 1.
 *   */
__weak float MC_GetTargetPosition1( void )
{
	return MCI_GetTargetPosition( pMCI[M1] );
}

/**
 * @brief returns the total movement duration to reach the target position of Motor 1.
 *   */
__weak float MC_GetMoveDuration1( void )
{
	return MCI_GetMoveDuration( pMCI[M1] );
}
</#if>

<#if MC.MOTOR_PROFILER >
/**
 * @brief call the correct profiler command.
 * Interface is generic MCPV2 interface :
 * - rxLength : length of data send in the payload command
 * - rxBuffer : pointer to the actual payload command
 * - txSyncFreeSpace : Total amount of space available for the answer
 * - txLength : Amount of space actually used for the answer - Set in the command itself
 * - txBuffer : Pointer to the buffer containing the answer
 * @retval returns MCP_CMD_OK or MCP_CMD_NOK if profiler is started correctly or not.
 *  */
__weak uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  return SCC_CMD(&SCC, rxLength, rxBuffer, txSyncFreeSpace, txLength, txBuffer);
}
<#else>
/**
 * @brief Not implemented MC_Profiler function.
 *  */
__weak uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  return MCP_CMD_UNKNOWN;
}
</#if>

<#if MC.DUALDRIVE == true>
/**
  * @brief  Initiates the start-up procedure for Motor 2
  * 
  * If the state machine of Motor 2 is in #IDLE state, the command is immediately
  * executed. Otherwise the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *
  * One of the following commands must be executed before calling MC_StartMotor2():
  *
  * - MC_ProgramSpeedRampMotor2()
  * - MC_ProgramTorqueRampMotor2()
  * - MC_SetCurrentReferenceMotor2()
  *
  * Failing to do so results in an unpredictable behaviour.
  *
  * @note The MC_StartMotor2() command only triggers the start-up procedure: 
  * It moves Motor 1's state machine from the #IDLE to the #IDLE_START state and then 
  * returns. It is not blocking the application until the motor is indeed running. 
  * To know if it is running, the application can query Motor 1's state machine and 
  * check if it has reached the #RUN state. See MC_GetSTMStateMotor2() for more details.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StartMotor2(void)
{
	return MCI_StartMotor( pMCI[M2] );
}

/**
  * @brief  Initiates the stop procedure for Motor 2.
  *
  *  If the state machine is in #RUN or #START states the command is immediately
  * executed. Otherwise, the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *  
  * @note The MCI_StopMotor2() command only triggers the stop motor procedure 
  * moving Motor 2's state machine to #ANY_STOP and then returns. It is not 
  * blocking the application until the motor is indeed stopped. To know if it has
  * stopped, the application can query Motor 1's state machine ans check if the 
  * #IDLE state has been reached back.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StopMotor2(void)
{
	return MCI_StopMotor( pMCI[M2] );
}

/**
  * @brief Programs a speed ramp for Motor 2 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p hFinalSpeed 
  * target speed in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramSpeedRampMotor2() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 2's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter 
  * differs from that of the current speed, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *                     Expressed in the unit defined by #SPEED_UNIT.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor2( int16_t hFinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp( pMCI[M2], hFinalSpeed, hDurationms );
}

/**
  * @brief Programs a speed ramp for Motor 2 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p FinalSpeed 
  * target speed in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramSpeedRampMotor2() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter 
  * differs from that of the current speed, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  FinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *         Expressed in rpm.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor2_F( float FinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp_F( pMCI[M2], FinalSpeed, hDurationms );
}

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief Programs a torque ramp for Motor 2 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p hFinalTorque 
  * target torque reference in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramTorqueRampMotor2() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 2's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalTorque parameter 
  * differs from that of the current torque, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in digit.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor2( int16_t hFinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp( pMCI[M2], hFinalTorque, hDurationms );
}

/**
  * @brief Programs a torque ramp for Motor 2 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p FinalTorque 
  * target torque reference in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramTorqueRampMotor2() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the FinalTorque parameter
  * differs from that of the current torque, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  FinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in Ampere.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor2_F( float FinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp_F( pMCI[M2], FinalTorque, hDurationms );
}
</#if>

<#if MC.POSITION_CTRL_ENABLING2 == true>
/**
  * @brief Programs a position command for Motor 2 for later or immediate execution.
  *
  *  A position command allows to program a specific movement from current position to
  *  the @p fTargetPosition target mechanical angle reference in the given @p fDuration time. 
  *
  *  Invoking the MC_ProgramPositionCommandMotor2() function programs a target position
  * with the provided parameters. The programmed position is executed immediately if
  * there is no movement ongoing.
  *
  *  The Application can check the status of the control with the MC_GetControlPositionStatusMotor2()
  * to know whether the motor is ready to execute a new command or not.
  *
  * @note when the duration is set to 0, the input target position is immediately executed.
  * By calling repeatedly the command with an update of input target angle, the control will be in follow mode.
  * 
  * 
  * @param  fTargetPosition Target mechanical angle reference at the end of the movement.
  *         This value represents the final position expressed in radiant.
  * @param  fDuration Duration of the movement expressed in seconds.
  *         It is possible to set 0 to perform an instantaneous angular change (follow mode).
  */
__weak void MC_ProgramPositionCommandMotor2( float fTargetPosition, float fDuration )
{
	MCI_ExecPositionCommand( pMCI[M2], fTargetPosition, fDuration );
}
</#if>

/**
  * @brief Programs the current reference to Motor 2 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components. 
  *
  *  Invoking the MC_SetCurrentReferenceMotor2() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 2's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * command is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is 
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed 
  *         in the qd_t format.
  */
__weak void MC_SetCurrentReferenceMotor2( qd_t Iqdref )
{
	MCI_SetCurrentReferences( pMCI[M2], Iqdref );
}

/**
  * @brief Programs the current reference to Motor 2 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components. 
  *
  *  Invoking the MC_SetCurrentReferenceMotor2_F() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * command is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor2()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is 
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed 
  *         in the qd_f_t format.
  */
__weak void MC_SetCurrentReferenceMotor2_F( qd_f_t IqdRef )
{
	MCI_SetCurrentReferences_F( pMCI[M2], IqdRef );
}

/**
  * @brief  Returns the status of the last buffered command for Motor 2.
  * The status can be one of the following values:
  * - #MCI_BUFFER_EMPTY: no buffered command is currently programmed.
  * - #MCI_COMMAND_NOT_ALREADY_EXECUTED: A command has been buffered but the conditions for its
  *   execution have not occurred yet. The command is still in the buffer, pending execution.
  * - #MCI_COMMAND_EXECUTED_SUCCESFULLY: the last buffered command has been executed successfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  * - #MCI_COMMAND_EXECUTED_UNSUCCESFULLY: the buffered command has been executed unsuccessfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  */
__weak MCI_CommandState_t  MC_GetCommandStateMotor2( void)
{
	return MCI_IsCommandAcknowledged( pMCI[M2] );
}

/** 
 * @brief Stops the execution of the on-going speed ramp for Motor 2, if any.
 *
 *  If a speed ramp is currently being executed, it is immediately stopped, the rotation
 * speed of Motor 2 is maintained to its current value and true is returned. If no speed 
 * ramp is on-going, nothing is done and false is returned.
 *
 * @deprecated This function is deprecated and should not be used anymore. It will be 
 *             removed in a future version of the MCSDK. Use MC_StopRampMotor2() instead.
 */
__weak bool MC_StopSpeedRampMotor2(void)
{
	return MCI_StopSpeedRamp( pMCI[M2] );
}

/** 
 * @brief Stops the execution of the on-going ramp for Motor 2, if any.
 *
 *  If a ramp is currently being executed, it is immediately stopped, the torque or the speed
 *  of Motor 2 is maintained to its current value.
 */
__weak void MC_StopRampMotor2(void)
{
   MCI_StopRamp( pMCI[M2] );
}

/**
 * @brief Returns true if the last ramp submited for Motor 2 has completed, false otherwise 
 */
__weak bool MC_HasRampCompletedMotor2(void)
{
	return MCI_RampCompleted( pMCI[M2] );
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 2, expressed in the unit defined by #SPEED_UNIT 
 */
__weak int16_t MC_GetMecSpeedReferenceMotor2(void)
{
	return MCI_GetMecSpeedRefUnit( pMCI[M2] );
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 2, expressed in rpm.
 */
__weak float MC_GetMecSpeedReferenceMotor2_F(void)
{
	return MCI_GetMecSpeedRef_F( pMCI[M2] );
}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 2, expressed in the unit defined by #SPEED_UNIT 
 */
__weak int16_t MC_GetMecSpeedAverageMotor2(void)
{
	return MCI_GetAvrgMecSpeedUnit( pMCI[M2] );
}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 2, expressed in rpm.
 */
__weak float MC_GetAverageMecSpeedMotor2_F(void)
{
	return MCI_GetAvrgMecSpeed_F( pMCI[M2] );
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 2 if this ramp was a speed ramp, 0 otherwise.
 */
__weak int16_t MC_GetLastRampFinalSpeedMotor2(void)
{
	return MCI_GetLastRampFinalSpeed( pMCI[M2] );
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 2 if this ramp was a speed ramp, 0 otherwise.
 */
__weak float MC_GetLastRampFinalSpeedMotor2_F(void)
{
	return MCI_GetLastRampFinalSpeed_F( pMCI[M2] );
}

/**
 * @brief Returns the Control Mode used for Motor 2 (either Speed or Torque)
 */
__weak MC_ControlMode_t MC_GetControlModeMotor2(void)
{
	return MCI_GetControlMode( pMCI[M2] );
}

/**
 * @brief Returns the rotation direction imposed by the last command on Motor 2 
 *
 * The last command is either MC_ProgramSpeedRampMotor1(), MC_ProgramTorqueRampMotor2() or
 * MC_SetCurrentReferenceMotor2(). 
 *
 * The function returns -1 if the sign of the final speed, the final torque or the Iq current 
 * reference component of the last command is negative. Otherwise, 1 is returned. 
 *   
 * @note if no such command has ever been submitted, 1 is returned as well.  
 */
__weak int16_t MC_GetImposedDirectionMotor2(void)
{
	return MCI_GetImposedMotorDirection( pMCI[M2] );
}

/** 
 * @brief Returns true if the speed sensor used for Motor 2 is reliable, false otherwise 
 */
__weak bool MC_GetSpeedSensorReliabilityMotor2(void)
{
	return MCI_GetSpdSensorReliability( pMCI[M2] );
}

<#if MC.DRIVE_TYPE == "FOC">
/** 
 * @brief returns the amplitude of the phase current injected in Motor 2
 *
 * The returned amplitude (0-to-peak) is expressed in s16A unit. To convert it to amperes, use the following formula:
 *
 * @f[
 * I_{Amps} = \frac{ I_{s16A} \times V_{dd}}{ 65536 \times R_{shunt} \times A_{op} }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseCurrentAmplitudeMotor2(void)
{
	return MCI_GetPhaseCurrentAmplitude( pMCI[M2] );
}

/**
 * @brief returns the amplitude of the phase voltage applied to Motor 2
 *
 * The returned amplitude (0-to-peak) is expressed in s16V unit. To convert it to volts, use the following formula:
 *
 * @f[
 * U_{Volts} = \frac{ U_{s16V} \times V_{bus}}{ \sqrt{3} \times 32768  }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseVoltageAmplitudeMotor2(void)
{
	return MCI_GetPhaseVoltageAmplitude( pMCI[M2] );
}

/**
 * @brief returns Ia and Ib current values for Motor 2 in ab_t format
 */
__weak ab_t MC_GetIabMotor2(void)
{
	return MCI_GetIab( pMCI[M2] );
}

/**
 * @brief returns Ialpha and Ibeta current values for Motor 2 in alphabeta_t format
 */
__weak alphabeta_t MC_GetIalphabetaMotor2(void)
{
	return MCI_GetIalphabeta( pMCI[M2] );
}

/**
 * @brief returns Iq and Id current values for Motor 2 in qd_t format 
 */
__weak qd_t MC_GetIqdMotor2(void)
{
	return MCI_GetIqd( pMCI[M2] );
}

/**
 * @brief returns Iq and Id reference current values for Motor 2 in qd_t format
 */
__weak qd_t MC_GetIqdrefMotor2(void)
{
	return MCI_GetIqdref( pMCI[M2] );
}

/**
 * @brief returns Iq and Id reference current values for Motor 2 in float type
 */
__weak qd_f_t MC_GetIqdrefMotor2_F(void)
{
	return MCI_GetIqdref_F( pMCI[M2] );
}

/**
 * @brief returns Vq and Vd voltage values for Motor 2 in qd_t format
 */
__weak qd_t MC_GetVqdMotor2(void)
{
	return MCI_GetVqd( pMCI[M2] );
}

/**
 * @brief returns Valpha and Vbeta voltage values for Motor 2 in alphabeta_t format 
 */
__weak alphabeta_t MC_GetValphabetaMotor2(void)
{
	return MCI_GetValphabeta( pMCI[M2] );
}

/**
 * @brief returns the electrical angle of the rotor of Motor 2, in DDP format 
 */
__weak int16_t MC_GetElAngledppMotor2(void)
{
	return MCI_GetElAngledpp( pMCI[M2] );
}

/**
 * @brief returns the electrical torque reference for Motor 2 
 */
__weak int16_t MC_GetTerefMotor2(void)
{
	return MCI_GetTeref( pMCI[M2] );
}

/**
 * @brief re-initializes Iq and Id references to their default values for Motor 2
 *
 * The default values for the Iq and Id references are comming from the Speed 
 * or the Torque controller depending on the control mode. 
 *
 * @see   SpeednTorqCtrl for more details.
 */
__weak void MC_Clear_IqdrefMotor2(void)
{
	MCI_Clear_Iqdref( pMCI[M2] );
}
</#if>

/**
 * @brief Acknowledge a Motor Control fault that occured on Motor 2
 *
 *  This function informs Motor 2's state machine that the Application has taken
 * the error condition that occured into account. If no error condition exists when
 * the function is called, nothing is done and false is returned. Otherwise, true is
 * returned. 
 */
__weak bool MC_AcknowledgeFaultMotor2( void )
{
	return MCI_FaultAcknowledged( pMCI[M2] );
}

/**
 * @brief Returns a bitfiled showing "new" faults that occured on Motor 2
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
__weak uint16_t MC_GetOccurredFaultsMotor2(void)
{
	return MCI_GetOccurredFaults( pMCI[M2] );
}

/**
 * @brief returns a bitfield showing all current faults on Motor 2 
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that are currently active.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
__weak uint16_t MC_GetCurrentFaultsMotor2(void)
{
	return MCI_GetCurrentFaults( pMCI[M2] );
}

/**
 * @brief returns the current state of Motor 2 state machine
 */
__weak MCI_State_t  MC_GetSTMStateMotor2(void)
{
	return MCI_GetSTMState( pMCI[M2] );
}

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief Sets the polarization offset values to use for Motor 2
  *
  * The Motor Control algorithm relies on a number of current and voltage measures. The hardware
  * parts that make these measurements need to be characterized at least once in the course of
  * product life, prior to its first activation. This characterization consists in measuring the
  * voltage presented to the ADC channels when either no current flows into the phases of the motor
  * or no voltage is applied to them. This characterization is named polarization offsets measurement
  * and its results are the polarization offsets.
  *
  * The Motor Control Firmware can performs this polarization offsets measurement procedure which
  * results in a number of offset values that the application can store in a non volatile memory and
  * then set into the Motor Control subsystem at power-on or after a reset.
  *
  * The application uses this function to set the polarization offset values that the Motor Control
  * subsystem is to use in the current session. This function can only be used when the state machine
  * of the motor is in the #IDLE state in which case it returns #MC_SUCCESS. Otherwise, it does nothing
  * and returns the #MC_WRONG_STATE_ERROR error code.
  *
  *  The Motor Control subsystem needs to know the polarization offsets before the motor can be controlled.
  * The MC_SetPolarizationOffsetsMotor2() function provides a way to set these offsets. Alternatively, the
  * application can either:
  *
  *  * Execute the polarization offsets measurement procedure with a call to
  *    MC_StartPolarizationOffsetsMeasurementMotor2() after a reset or a power on;
  *  * Start the motor control with the MC_StartWithPolarizationMotor2() that will execute the procedure
  *    before actually starting the motor, on the first time it is called after a reset or a power on.
  *
  * When this function completes successfully, the state of the polarization offsets measurement procedure
  * is set to #COMPLETED. See MC_GetPolarizationState().
  *
  * @param PolarizationOffsets an pointer on a structure containing the offset values
  */
bool MC_SetPolarizationOffsetsMotor2( PolarizationOffsets_t * PolarizationOffsets )
{
	return( MCI_SetCalibratedOffsetsMotor( pMCI[M2], PolarizationOffsets ) );
}

/**
  * @brief Returns the polarization offset values measured or set for Motor 2
  *
  *  See MC_SetPolarizationOffsetsMotor2() for more details.
  *
  *  If the Motor Control Firmware knows the polarization offset values, they are copied into the
  * @p PolarizationOffsets structure and #MC_SUCCESS is returned. Otherwise, nothing is done and
  * #MC_NO_POLARIZATION_OFFSETS_ERROR is returned.
  *
  * @param PolarizationOffsets an pointer on the structure into which the polarization offsets will be
  *        copied
  * @return #MC_SUCCESS if calibration data were present and could be copied into @p PolarizationOffsets,
  *         #MC_NO_POLARIZATION_OFFSETS_ERROR otherwise.
  */
bool MC_GetPolarizationOffsetsMotor2( PolarizationOffsets_t * PolarizationOffsets )
{
   return ( MCI_GetCalibratedOffsetsMotor( pMCI[M2], PolarizationOffsets) );
}

/**
  * @brief Starts the polarization offsets measurement procedure for Motor 2.
  *
  * See MC_SetPolarizationOffsetsMotor2() for more details.
  *
  * If the Motor Control Firmware is in the #IDLE state, the procedure is started, the state machine
  * of the motor switches to #OFFSET_CALIB and #MC_SUCCESS is returned. Otherwise, nothing is done
  * and the #MC_WRONG_STATE_ERROR error code is returned.
  *
  * The polarization offsets measurement procedure is only triggered by this function and it is has not
  * completed when this function returns. The application can use the MC_GetPolarizationState()
  * function to query the state of the procedure.
  *
  * @see MC_GetPolarizationState()
  */
bool MC_StartPolarizationOffsetsMeasurementMotor2( void )
{
	return( MCI_StartOffsetMeasurments( pMCI[M2] ) );
}

/**
 * @brief This method is used to get the average measured motor power
 *        expressed in watt for Motor 2.
 * @retval float The average measured motor power expressed in watt.
 */
__weak float MC_GetAveragePowerMotor2_F(void)
{
	return (PQD_GetAvrgElMotorPowerW(pMPM[M2]));
}
</#if>

<#if MC.POSITION_CTRL_ENABLING2 == true>
/**
 * @brief returns the current control position state of Motor 2.
 *   */
__weak PosCtrlStatus_t  MC_GetControlPositionStatusMotor2( void )
{
	return MCI_GetCtrlPositionState( pMCI[M2] );
}

/**
 * @brief returns the alignment state of Motor 2.
 *   */
__weak AlignStatus_t  MC_GetAlignmentStatusMotor2( void )
{
	return MCI_GetAlignmentStatus( pMCI[M2] );
}

/**
 * @brief returns the current position of Motor 2.
 *   */
__weak float MC_GetCurrentPosition2( void )
{
	return MCI_GetCurrentPosition( pMCI[M2] );
}

/**
 * @brief returns the target position of Motor 2.
 *   */
__weak float MC_GetTargetPosition2( void )
{
	return MCI_GetTargetPosition( pMCI[M2] );
}

/**
 * @brief returns the total movement duration to reach the target position of Motor 2.
 *   */
__weak float MC_GetMoveDuration2( void )
{
	return MCI_GetMoveDuration( pMCI[M2] );
}
</#if>

/**
  * @}
  */

/**
  * @}
  */
</#if> <#-- MC.DUALDRIVE == true> -->
/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

