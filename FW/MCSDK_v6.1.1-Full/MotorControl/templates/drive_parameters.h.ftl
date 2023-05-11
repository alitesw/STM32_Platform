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

<#function Fx_Freq_Scaling pwm_freq>
<#list [ 1 , 2 , 4 , 8 , 16 ] as scaling>
       <#if (pwm_freq/scaling) < 65536 >
            <#return scaling >
        </#if>
    </#list>
    <#return 1 >
</#function>
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

<#if MC.DUALDRIVE == true>
/**************************
 *** Motor 1 Parameters ***
 **************************/
<#else>
/************************
 *** Motor Parameters ***
 ************************/
</#if>

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       ${MC.MAX_APPLICATION_SPEED} /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       ${MC.MIN_APPLICATION_SPEED} /*!< rpm, mechanical,  
                                                           absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS ${MC.M1_SS_MEAS_ERRORS_BEFORE_FAULTS} /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */
<#if MC.ENCODER || MC.AUX_ENCODER >
/*** Encoder **********************/                                                                                                           

#define ENC_AVERAGING_FIFO_DEPTH        ${MC.ENC_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */
</#if>
<#if MC.HALL_SENSORS || MC.AUX_HALL_SENSORS>
/****** Hall sensors ************/ 

#define HALL_AVERAGING_FIFO_DEPTH        ${MC.HALL_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */  
#define HALL_MTPA <#if MC.HALL_MTPA > true <#else> false </#if>                                                           
</#if>
<#if MC.STATE_OBSERVER_PLL ||  MC.AUX_STATE_OBSERVER_PLL >
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD             ${MC.VARIANCE_THRESHOLD} /*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */
/* State observer scaling factors F1 */                    
#define F1                               ${MC.F1}
#define F2                               ${MC.F2}
#define F1_LOG                           LOG2((${MC.F1}))
#define F2_LOG                           LOG2((${MC.F2}))

/* State observer constants */
#define GAIN1                            ${MC.GAIN1}
#define GAIN2                            ${MC.GAIN2}
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                      ${MC.PLL_KP_GAIN}
#define PLL_KI_GAIN                      ${MC.PLL_KI_GAIN}
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2((PLL_KPDIV))
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP               ${MC.STO_FIFO_DEPTH_DPP}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2((${MC.STO_FIFO_DEPTH_DPP}))
                                                            
#define STO_FIFO_DEPTH_UNIT              ${MC.STO_FIFO_DEPTH_01HZ}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed 
                                                            in the unit defined by #SPEED_UNIT */
#define BEMF_CONSISTENCY_TOL             ${MC.BEMF_CONSISTENCY_TOL}   /* Parameter for B-emf 
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN            ${MC.BEMF_CONSISTENCY_GAIN}   /* Parameter for B-emf 
                                                           amplitude-speed consistency */
                                                                                
</#if>
<#if MC.STATE_OBSERVER_CORDIC || MC.AUX_STATE_OBSERVER_CORDIC >  
/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD          ${MC.CORD_VARIANCE_THRESHOLD} /*!<Maxiumum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */                                                                                                                
#define CORD_F1                          ${MC.CORD_F1}
#define CORD_F2                          ${MC.CORD_F2}
#define CORD_F1_LOG                      LOG2((${MC.CORD_F1}))
#define CORD_F2_LOG                      LOG2((${MC.CORD_F2}))

/* State observer constants */
#define CORD_GAIN1                       ${MC.CORD_GAIN1}
#define CORD_GAIN2                       ${MC.CORD_GAIN2}

#define CORD_FIFO_DEPTH_DPP              ${MC.CORD_FIFO_DEPTH_DPP}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define CORD_FIFO_DEPTH_DPP_LOG          LOG2((${MC.CORD_FIFO_DEPTH_DPP}))
                                                            
#define CORD_FIFO_DEPTH_UNIT            ${MC.CORD_FIFO_DEPTH_01HZ}  /*!< Depth of the FIFO used  
                                                           to average mechanical speed  
                                                           in dpp format */        
#define CORD_MAX_ACCEL_DPPP              ${MC.CORD_MAX_ACCEL_DPPP}  /*!< Maximum instantaneous 
                                                              electrical acceleration (dpp 
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL        ${MC.CORD_BEMF_CONSISTENCY_TOL}  /* Parameter for B-emf 
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN       ${MC.CORD_BEMF_CONSISTENCY_GAIN}  /* Parameter for B-emf 
                                                          amplitude-speed consistency */
</#if>
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
/****** Bemf Observer ****/
#define BEMF_AVERAGING_FIFO_DEPTH        ${MC.BEMF_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */  
#define VARIANCE_THRESHOLD             ${MC.VARIANCE_THRESHOLD} /*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */														
#define BEMF_THRESHOLD_DOWN_V    		${MC.BEMF_THRESHOLD_DOWN_V}	/*!< BEMF voltage threshold for zero crossing detection when BEMF is expected to decrease */
#define BEMF_THRESHOLD_UP_V      		${MC.BEMF_THRESHOLD_UP_V}	/*!< BEMF voltage threshold for zero crossing detection when BEMF is expected to increase */
#define BEMF_ADC_TRIG_TIME_DPP          ((uint16_t) ${MC.BEMF_ADC_TRIG_TIME_DPP})   /*!< 1/1024 of PWM period elapsed */

  <#if  MC.DRIVE_MODE == "VM">                                                             
#define BEMF_THRESHOLD_DOWN_ON_V 		${MC.BEMF_THRESHOLD_DOWN_ON_V}	/*!< BEMF voltage threshold during PWM ON time for zero crossing detection when BEMF is expected to decrease */
#define BEMF_THRESHOLD_UP_ON_V   		${MC.BEMF_THRESHOLD_UP_ON_V}	/*!< BEMF voltage threshold during PWM ON time for zero crossing detection when BEMF is expected to increase */
#define BEMF_ADC_TRIG_TIME_ON_DPP    	((uint16_t) ${MC.BEMF_ADC_TRIG_TIME_ON_DPP})    /*!< 1/1024 of PWM period elapsed */
                                                         
#define BEMF_PWM_ON_ENABLE_THRES_DPP     	((uint16_t) ${MC.BEMF_PWM_ON_ENABLE_THRES_DPP})    /*!< 1/1024 of PWM period elapsed */
#define BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP   ((uint16_t) ${MC.BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP})   /*!< 1/1024 of PWM period elapsed */
  </#if>
#define ZCD_TO_COMM              ((uint16_t) ${MC.ZCD_TO_COMM})    /*!< Zero Crossing detection to commutation delay in 15/128 degrees */

#define MIN_DEMAG_TIME           ((uint16_t) ${MC.MIN_DEMAG_TIME})      /*!< Demagnetization delay in number of HF timer periods elapsed before a first BEMF ADC measurement is processed in an attempt to detect the BEMF zero crossing */
#define SPEED_THRESHOLD_DEMAG    ((uint32_t) ${MC.SPEED_THRESHOLD_DEMAG})  /*!< Speed threshold above which the RUN_DEMAGN_DELAY_MIN is applied */
#define DEMAG_RUN_STEP_RATIO     ((uint16_t) ${MC.RUN_DEMAG_TIME})    /*!< Percentage of step time allowed for demagnetization */
#define DEMAG_REVUP_STEP_RATIO   ((uint16_t) ${MC.STARTUP_DEMAG_TIME})    /*!< Percentage of step time allowed for demagnetization */
</#if>

/* USER CODE BEGIN angle reconstruction M1 */
<#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true>
#define PARK_ANGLE_COMPENSATION_FACTOR 0
</#if>
<#if MC.DRIVE_TYPE == "FOC">
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */


#define PWM_FREQUENCY   ${MC.PWM_FREQUENCY}
#define PWM_FREQ_SCALING ${Fx_Freq_Scaling((MC.PWM_FREQUENCY)?number)}

<#if MC.DRIVE_TYPE == "SIX_STEP" && MC.DRIVE_MODE == "CM">
#define PWM_FREQUENCY_REF   ${MC.REF_TIMER_FREQUENCY}
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

#define LOW_SIDE_SIGNALS_ENABLING        ${MC.LOW_SIDE_SIGNALS_ENABLING}
<#if MC.LOW_SIDE_SIGNALS_ENABLING == 'LS_PWM_TIMER'>
#define SW_DEADTIME_NS                   ${MC.SW_DEADTIME_NS} /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
</#if>

<#if MC.DRIVE_TYPE == "SIX_STEP">
/* High frequency task regulation loop */
#define REGULATION_EXECUTION_RATE     ${MC.REGULATION_EXECUTION_RATE}    /*!< Execution rate in 
                                                           number of PWM cycles */ 
</#if>														   
<#if MC.DRIVE_TYPE == "FOC">
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     ${MC.REGULATION_EXECUTION_RATE}    /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         ${MC.PID_TORQUE_KP_DEFAULT}       
#define PID_TORQUE_KI_DEFAULT         ${MC.PID_TORQUE_KI_DEFAULT}
#define PID_TORQUE_KD_DEFAULT         ${MC.PID_TORQUE_KD_DEFAULT}
#define PID_FLUX_KP_DEFAULT           ${MC.PID_FLUX_KP_DEFAULT}
#define PID_FLUX_KI_DEFAULT           ${MC.PID_FLUX_KI_DEFAULT}
#define PID_FLUX_KD_DEFAULT           ${MC.PID_FLUX_KD_DEFAULT}

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      ${MC.TF_KPDIV}
#define TF_KIDIV                      ${MC.TF_KIDIV}
#define TF_KDDIV                      ${MC.TF_KDDIV}
#define TF_KPDIV_LOG                  LOG2((${MC.TF_KPDIV}))
#define TF_KIDIV_LOG                  LOG2((${MC.TF_KIDIV}))
#define TF_KDDIV_LOG                  LOG2((${MC.TF_KDDIV}))
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if MC.POSITION_CTRL_ENABLING == true >
#define POSITION_LOOP_FREQUENCY_HZ    ( uint16_t )${MC.POSITION_LOOP_FREQUENCY_HZ} /*!<Execution rate of position control regulation loop (Hz) */
<#else>
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ       ( uint16_t )${MC.SPEED_LOOP_FREQUENCY_HZ} /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
</#if>

<#if ((MC.DRIVE_TYPE == "SIX_STEP") && (MC.CURRENT_LIMITER_OFFSET))>                                        
#define PID_SPEED_KP_DEFAULT          -${MC.PID_SPEED_KP_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          -${MC.PID_SPEED_KI_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          ${MC.PID_SPEED_KD_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
<#else>
#define PID_SPEED_KP_DEFAULT          ${MC.PID_SPEED_KP_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          ${MC.PID_SPEED_KI_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          ${MC.PID_SPEED_KD_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
</#if>
/* Speed PID parameter dividers */
#define SP_KPDIV                      ${MC.SP_KPDIV}
#define SP_KIDIV                      ${MC.SP_KIDIV}
#define SP_KDDIV                      ${MC.SP_KDDIV}
#define SP_KPDIV_LOG                  LOG2((${MC.SP_KPDIV}))
#define SP_KIDIV_LOG                  LOG2((${MC.SP_KIDIV}))
#define SP_KDDIV_LOG                  LOG2((${MC.SP_KDDIV}))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
<#if MC.DRIVE_TYPE == "FOC">
#define IQMAX                          ${MC.IQMAX}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
  <#if MC.DRIVE_MODE == "VM">
#define PERIODMAX                      (uint16_t) (PWM_PERIOD_CYCLES * ${MC.PERIODMAX} / 100)
  <#else>
#define PERIODMAX_REF                  (uint16_t) (PWM_PERIOD_CYCLES_REF * ${MC.PERIODMAX} / 100)
  </#if>
#define LF_TIMER_ARR                   ${MC.LF_TIMER_ARR}
#define LF_TIMER_PSC                   ${MC.LF_TIMER_PSC}
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

/* Default settings */
<#if MC.DRIVE_TYPE == "FOC">
<#if MC.DEFAULT_CONTROL_MODE == 'STC_SPEED_MODE'>
#define DEFAULT_CONTROL_MODE           MCM_SPEED_MODE
<#else>
#define DEFAULT_CONTROL_MODE           MCM_TORQUE_MODE
</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
#define DEFAULT_CONTROL_MODE           MCM_SPEED_MODE
#define DEFAULT_DRIVE_MODE             ${MC.DRIVE_MODE} /*!< VOLTAGE_MODE (VM) or 
                                                        CURRENT_MODE (CM)*/
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
#define DEFAULT_TARGET_SPEED_RPM       ${MC.DEFAULT_TARGET_SPEED_RPM}
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
<#if MC.DRIVE_TYPE == "FOC">
#define DEFAULT_TORQUE_COMPONENT       ${MC.DEFAULT_TORQUE_COMPONENT}
#define DEFAULT_FLUX_COMPONENT         ${MC.DEFAULT_FLUX_COMPONENT}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if  MC.POSITION_CTRL_ENABLING == true >
#define PID_POSITION_KP_GAIN			${MC.POSITION_CTRL_KP_GAIN}
#define PID_POSITION_KI_GAIN			${MC.POSITION_CTRL_KI_GAIN}
#define PID_POSITION_KD_GAIN			${MC.POSITION_CTRL_KD_GAIN}
#define PID_POSITION_KPDIV				${MC.POSITION_CTRL_KPDIV}     
#define PID_POSITION_KIDIV				${MC.POSITION_CTRL_KIDIV}
#define PID_POSITION_KDDIV				${MC.POSITION_CTRL_KDDIV}
#define PID_POSITION_KPDIV_LOG			LOG2((${MC.POSITION_CTRL_KPDIV}))    
#define PID_POSITION_KIDIV_LOG			LOG2((${MC.POSITION_CTRL_KIDIV})) 
#define PID_POSITION_KDDIV_LOG			LOG2((${MC.POSITION_CTRL_KDDIV})) 
#define PID_POSITION_ANGLE_STEP			${MC.POSITION_CTRL_ANGLE_STEP}
#define PID_POSITION_MOV_DURATION		${MC.POSITION_CTRL_MOV_DURATION}
</#if>


/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
<#if   MC.ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OV_VOLTAGE_THRESHOLD_V          ${MC.OV_VOLTAGE_THRESHOLD_V} /*!< Over-voltage 
                                                         threshold */
<#else>
#define M1_OVP_THRESHOLD_HIGH           ${MC.M1_OVP_THRESHOLD_HIGH} /*!< Over-voltage 
                                                         hysteresis high threshold */
#define M1_OVP_THRESHOLD_LOW            ${MC.M1_OVP_THRESHOLD_LOW} /*!< Over-voltage 
                                                         hysteresis low threshold */
</#if>
#define UD_VOLTAGE_THRESHOLD_V          ${MC.UD_VOLTAGE_THRESHOLD_V} /*!< Under-voltage 
                                                          threshold */
#ifdef NOT_IMPLEMENTED

#define ON_OVER_VOLTAGE                 ${MC.ON_OVER_VOLTAGE} /*!< TURN_OFF_PWM, 
                                                         TURN_ON_R_BRAKE or 
                                                         TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */

#define OV_TEMPERATURE_THRESHOLD_C      ${MC.OV_TEMPERATURE_THRESHOLD_C} /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     ${MC.OV_TEMPERATURE_HYSTERESIS_C} /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
                                                          
                                                        
#define OVP_INVERTINGINPUT_MODE         ${MC.OVP_INVERTINGINPUT_MODE}
#define OVP_INVERTINGINPUT_MODE2        ${MC.OVP_INVERTINGINPUT_MODE2}
#define OVP_SELECTION                   ${MC.OVP_SELECTION}
#define OVP_SELECTION2                  ${MC.OVP_SELECTION2}

/******************************   START-UP PARAMETERS   **********************/
<#if ( MC.ENCODER == true ||  MC.AUX_ENCODER == true)>
/* Encoder alignment */
#define ALIGNMENT_DURATION              ${MC.ALIGNMENT_DURATION} /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG             ${MC.ALIGNMENT_ANGLE_DEG} /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT               ${MC.FINAL_I_ALIGNMENT} /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input
</#if>

<#if ( MC.STATE_OBSERVER_PLL || MC.STATE_OBSERVER_CORDIC || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC" )>
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE >
/* USER CODE BEGIN OPENLOOP M1 */

#define OPEN_LOOP_VOLTAGE_d           0      /*!< Three Phase voltage amplitude
                                                      in int16_t format */
#define OPEN_LOOP_SPEED_RPM           100       /*!< Final forced speed in rpm */
#define OPEN_LOOP_SPEED_RAMP_DURATION_MS  1000  /*!< 0-to-Final speed ramp duration  */      
#define OPEN_LOOP_VF                  false     /*!< true to enable V/F mode */
#define OPEN_LOOP_K                   44        /*! Slope of V/F curve expressed in int16_t Voltage for 
                                                     each 0.1Hz of mecchanical frequency increment. */
#define OPEN_LOOP_OFF                 4400      /*! Offset of V/F curve expressed in int16_t Voltage 
                                                     applied when frequency is zero. */
/* USER CODE END OPENLOOP M1 */
  </#if> <#-- MC.M1_DBG_OPEN_LOOP_ENABLE == false inside MC.STATE_OBSERVER_PLL || MC.STATE_OBSERVER_CORDIC -->

/* Phase 1 */
#define PHASE1_DURATION                ${MC.PHASE1_DURATION} /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT         (${MC.PHASE1_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) 
   <#if MC.DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM">
#define PHASE1_VOLTAGE_RMS             ${MC.PHASE1_VOLTAGE_RMS}
   <#else>
#define PHASE1_FINAL_CURRENT           ${MC.PHASE1_FINAL_CURRENT}
   </#if>
/* Phase 2 */
#define PHASE2_DURATION                ${MC.PHASE2_DURATION} /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT         (${MC.PHASE2_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
   <#if MC.DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM">
#define PHASE2_VOLTAGE_RMS             ${MC.PHASE2_VOLTAGE_RMS}
   <#else>
#define PHASE2_FINAL_CURRENT           ${MC.PHASE2_FINAL_CURRENT}
   </#if>
/* Phase 3 */
#define PHASE3_DURATION                ${MC.PHASE3_DURATION} /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT         (${MC.PHASE3_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
   <#if MC.DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM">
#define PHASE3_VOLTAGE_RMS             ${MC.PHASE3_VOLTAGE_RMS}
   <#else>
#define PHASE3_FINAL_CURRENT           ${MC.PHASE3_FINAL_CURRENT}
   </#if>
/* Phase 4 */
#define PHASE4_DURATION                ${MC.PHASE4_DURATION} /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT         (${MC.PHASE4_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
   <#if MC.DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM">
#define PHASE4_VOLTAGE_RMS             ${MC.PHASE4_VOLTAGE_RMS}
   <#else>
#define PHASE4_FINAL_CURRENT           ${MC.PHASE4_FINAL_CURRENT}
   </#if>
/* Phase 5 */
#define PHASE5_DURATION                ${MC.PHASE5_DURATION} /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT         (${MC.PHASE5_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
   <#if MC.DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM">
#define PHASE5_VOLTAGE_RMS             ${MC.PHASE5_VOLTAGE_RMS}
   <#else>
#define PHASE5_FINAL_CURRENT           ${MC.PHASE5_FINAL_CURRENT}
   </#if>


#define ENABLE_SL_ALGO_FROM_PHASE      ${MC.ENABLE_SL_ALGO_FROM_PHASE}
/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG             ${MC.STARTING_ANGLE_DEG}  /*!< degrees [0...359] */                                                             
</#if>
<#if MC.STATE_OBSERVER_PLL || MC.STATE_OBSERVER_CORDIC || MC.AUX_STATE_OBSERVER_PLL || MC.AUX_STATE_OBSERVER_CORDIC || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          ${MC.OBS_MINIMUM_SPEED_RPM}

#define NB_CONSECUTIVE_TESTS           ${MC.NB_CONSECUTIVE_TESTS} /* corresponding to 
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         ${MC.SPEED_BAND_UPPER_LIMIT} /*!< It expresses how much 
                                                            estimated speed can exceed 
                                                            forced stator electrical 
                                                            without being considered wrong. 
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         ${MC.SPEED_BAND_LOWER_LIMIT}  /*!< It expresses how much 
                                                             estimated speed can be below 
                                                             forced stator electrical 
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */ 
</#if>                                                             

#define TRANSITION_DURATION            ${MC.TRANSITION_DURATION}  /* Switch over duration, ms */ 

<#if   MC.BUS_VOLTAGE_READING >
/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(${MC.VBUS_ADC_SAMPLING_TIME})
</#if>
<#if MC.TEMPERATURE_READING >
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(${MC.TEMP_ADC_SAMPLING_TIME})
</#if>
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (${MC.CURR_SAMPLING_TIME} + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

<#if MC.FLUX_WEAKENING_ENABLING>
#define FW_VOLTAGE_REF                ${MC.FW_VOLTAGE_REF} /*!<Vs reference, tenth 
                                                        of a percent */
#define FW_KP_GAIN                    ${MC.FW_KP_GAIN} /*!< Default Kp gain */
#define FW_KI_GAIN                    ${MC.FW_KI_GAIN} /*!< Default Ki gain */
#define FW_KPDIV                      ${MC.FW_KPDIV}      
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define FW_KIDIV                      ${MC.FW_KIDIV}
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG                  LOG2((${MC.FW_KPDIV}))
#define FW_KIDIV_LOG                  LOG2((${MC.FW_KIDIV}))
</#if>
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING>                                                
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING==true>ENABLE<#else>DISABLE</#if>
#define CONSTANT1_Q                    ${MC.CONSTANT1_Q}
#define CONSTANT1_D                    ${MC.CONSTANT1_D}
#define CONSTANT2_QD                   ${MC.CONSTANT2_QD}
</#if>
<#if MC.MTPA_ENABLING>
/*  Maximum Torque Per Ampere strategy parameters */

#define MTPA_ENABLING                  
#define SEGDIV                         ${MC.SEGDIV}
#define ANGC                           ${MC.ANGC}
#define OFST                           ${MC.OFST}
</#if>

<#if MC.M1_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M1_ICL_RELAY_SWITCHING_DELAY_MS ${MC.M1_ICL_RELAY_SWITCHING_DELAY_MS}  /* milliseconds */
#define M1_ICL_CAPS_CHARGING_DELAY_MS   ${MC.M1_ICL_CAPS_CHARGING_DELAY_MS}  /* milliseconds */  
#define M1_ICL_VOLTAGE_THRESHOLD        ${MC.M1_ICL_VOLTAGE_THRESHOLD}  /* volts */                
</#if>

<#if MC.M1_POTENTIOMETER_ENABLE == true>
/* **** Potentiometer parameters **** */

/** @brief Sampling time set to the ADC channel used by the potentiometer component */
#define POTENTIOMETER_ADC_SAMPLING_TIME_M1  LL_ADC_SAMPLING_CYCLE(${MC.POTENTIOMETER_ADC_SAMPLING_TIME})

<#--
This value is stated in "u16digit" to be compatible with the values acquired 
by the ADC of the poentiometer. The values read from the ADC range from 0 to 
65520
*/
-->
/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its maximum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is #MAX_APPLICATION_SPEED_UNIT. 
 *
 * @sa POTENTIOMETER_MIN_SPEED_M1
 */
#define POTENTIOMETER_MAX_SPEED_M1 MAX_APPLICATION_SPEED_UNIT

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its minimum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is 10 % of #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MAX_SPEED_M1
 */
#define POTENTIOMETER_MIN_SPEED_M1 ((MAX_APPLICATION_SPEED_UNIT)/10)

<#--
/**
 * @brief Conversion factor from #SPEED_UNIT to Potentiometer ADC scale for Motor 1
 *
 * This Factor is used to convert values read from an ADC by the potentiometer 
 * component into values expressed in #SPEED_UNIT according to the following
 * formula: `V_SPEED_UNIT = V_ADC / POTENTIOMETER_SPEED_CONV_FACTOR_M1`
 */
#define POTENTIOMETER_SPEED_CONV_FACTOR_M1  ((65520.0)/((POTENTIOMETER_MAX_SPEED_M1)-(POTENTIOMETER_MIN_SPEED_M1)))
-->
/**
 * @brief Potentiometer change threshold to trigger speed reference update for Motor 1
 *
 * When the potentiometer value differs from the current speed reference by more than this 
 * threshold, the speed reference set to the motor is adjusted to match the potentiometer value. 
 *
 * The threshold is expressed in u16digits. Its default value is set to 13% of the potentiometer 
 * aquisition range
 *
 */
 #define POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M1 (655)

/**
 * @brief Acceleration used to compute ramp duration when setting speed reference to Motor 1
 *
 * This acceleration is expressed in #SPEED_UNIT/s. Its default value is 100 Hz/s (provided 
 * that #SPEED_UNIT is #U_01HZ).
 *
 */
 #define POTENTIOMETER_RAMP_SLOPE_M1 1000

/**
 * @brief Bandwith of the low pass filter applied on the potentiometer values
 *
 * @see SpeedPotentiometer_Handle_t::LPFilterBandwidthPOW2
 */
#define POTENTIOMETER_LPF_BANDWIDTH_POW2_M1 4
</#if>

/*** On the fly start-up ***/
<#-- ToDo: On the Fly start-up -->

<#if MC.DUALDRIVE == true>
/**************************
 *** Motor 2 Parameters ***
 **************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM2           ${MC.MAX_APPLICATION_SPEED2} /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM2           ${MC.MIN_APPLICATION_SPEED2} /*!< rpm, mechanical,  
                                                           absolute value */
#define M2_SS_MEAS_ERRORS_BEFORE_FAULTS       ${MC.M2_SS_MEAS_ERRORS_BEFORE_FAULTS} /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */
<#if MC.ENCODER2 || MC.AUX_ENCODER2 >
/*** Encoder **********************/                                                                                                           

#define ENC_AVERAGING_FIFO_DEPTH2        ${MC.ENC_AVERAGING_FIFO_DEPTH2} /*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */
</#if>
<#if MC.HALL_SENSORS2 || MC.AUX_HALL_SENSORS2 >
/****** Hall sensors ************/ 
#define HALL_AVERAGING_FIFO_DEPTH2        ${MC.HALL_AVERAGING_FIFO_DEPTH2} /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */ 
#define HALL_MTPA2 <#if MC.HALL_MTPA2 > true <#else> false </#if>                                                           
</#if>
<#if MC.STATE_OBSERVER_PLL2 ||  MC.AUX_STATE_OBSERVER_PLL2 >
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD2               ${MC.VARIANCE_THRESHOLD2} /*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */
/* State observer scaling factors F1 */                    
#define F12                               ${MC.F12}
#define F22                               ${MC.F22}
#define F1_LOG2                           LOG2((${MC.F12})
#define F2_LOG2                           LOG2((${MC.F22})

/* State observer constants */
#define GAIN12                            ${MC.GAIN12}
#define GAIN22                            ${MC.GAIN22}
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN2                      ${MC.PLL_KP_GAIN2}
#define PLL_KI_GAIN2                      ${MC.PLL_KI_GAIN2}
#define PLL_KPDIV2                        16384
#define PLL_KPDIV_LOG2                    LOG2(PLL_KPDIV2)
#define PLL_KIDIV2                        65535
#define PLL_KIDIV_LOG2                    LOG2(PLL_KIDIV2)


#define STO_FIFO_DEPTH_DPP2               ${MC.STO_FIFO_DEPTH_DPP2}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG2           LOG2((${MC.STO_FIFO_DEPTH_DPP2}))

#define STO_FIFO_DEPTH_UNIT2              ${MC.STO_FIFO_DEPTH_01HZ2}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define BEMF_CONSISTENCY_TOL2             ${MC.BEMF_CONSISTENCY_TOL2}   /* Parameter for B-emf 
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN2            ${MC.BEMF_CONSISTENCY_GAIN2}   /* Parameter for B-emf 
                                                           amplitude-speed consistency */
</#if>

<#if MC.STATE_OBSERVER_CORDIC2 || MC.AUX_STATE_OBSERVER_CORDIC2 >                                                                                
/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD2          ${MC.CORD_VARIANCE_THRESHOLD2} /*!<Maxiumum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */                                                                                                                
#define CORD_F12                          ${MC.CORD_F12}
#define CORD_F22                          ${MC.CORD_F22}
#define CORD_F1_LOG2                      LOG2((${MC.CORD_F12}))
#define CORD_F2_LOG2                      LOG2((${MC.CORD_F22}))

/* State observer constants */
#define CORD_GAIN12                       ${MC.CORD_GAIN12}
#define CORD_GAIN22                       ${MC.CORD_GAIN22}

#define CORD_FIFO_DEPTH_DPP2              ${MC.CORD_FIFO_DEPTH_DPP2}  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define CORD_FIFO_DEPTH_DPP_LOG2          LOG2((${MC.CORD_FIFO_DEPTH_DPP2}))
                                                            
#define CORD_FIFO_DEPTH_UNIT2             ${MC.CORD_FIFO_DEPTH_01HZ2}  /*!< Depth of the FIFO used  
                                                           to average mechanical speed  
                                                           in dpp format */        
#define CORD_MAX_ACCEL_DPPP2              ${MC.CORD_MAX_ACCEL_DPPP2}  /*!< Maximum instantaneous 
                                                              electrical acceleration (dpp 
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL2        ${MC.CORD_BEMF_CONSISTENCY_TOL2}  /* Parameter for B-emf 
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN2       ${MC.CORD_BEMF_CONSISTENCY_GAIN2}  /* Parameter for B-emf 
                                                          amplitude-speed consistency */
</#if>

/* USER CODE BEGIN angle reconstruction M2 */
<#if MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true>
#define PARK_ANGLE_COMPENSATION_FACTOR2 0
</#if>
#define REV_PARK_ANGLE_COMPENSATION_FACTOR2 0
/* USER CODE END angle reconstruction M2 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* Dual drive specific parameters */
#define FREQ_RATIO                      ${MC.FREQ_RATIO2}  /* Higher PWM frequency/lower PWM frequency */  
#define FREQ_RELATION                   ${MC.FREQ_RELATION}  /* It refers to motor 1 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */
#define FREQ_RELATION2                  ${MC.FREQ_RELATION2}   /* It refers to motor 2 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */

/* PWM generation and current reading */
#define PWM_FREQUENCY2                    ${MC.PWM_FREQUENCY2}
#define PWM_FREQ_SCALING2 ${Fx_Freq_Scaling((MC.PWM_FREQUENCY2)?number)}
 
#define LOW_SIDE_SIGNALS_ENABLING2        ${MC.LOW_SIDE_SIGNALS_ENABLING2}
<#if MC.LOW_SIDE_SIGNALS_ENABLING2 == 'LS_PWM_TIMER'>
#define SW_DEADTIME_NS2                   ${MC.SW_DEADTIME_NS2} /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
</#if>
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE2     ${MC.REGULATION_EXECUTION_RATE2}    /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT2         ${MC.PID_TORQUE_KP_DEFAULT2}       
#define PID_TORQUE_KI_DEFAULT2         ${MC.PID_TORQUE_KI_DEFAULT2}
#define PID_TORQUE_KD_DEFAULT2         ${MC.PID_TORQUE_KD_DEFAULT2}
#define PID_FLUX_KP_DEFAULT2           ${MC.PID_FLUX_KP_DEFAULT2}
#define PID_FLUX_KI_DEFAULT2           ${MC.PID_FLUX_KI_DEFAULT2}
#define PID_FLUX_KD_DEFAULT2           ${MC.PID_FLUX_KD_DEFAULT2}

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV2                      ${MC.TF_KPDIV2}
#define TF_KIDIV2                      ${MC.TF_KIDIV2}
#define TF_KDDIV2                      ${MC.TF_KDDIV2}
#define TF_KPDIV_LOG2                  LOG2((${MC.TF_KPDIV2}))
#define TF_KIDIV_LOG2                  LOG2((${MC.TF_KIDIV2}))
#define TF_KDDIV_LOG2                  LOG2((${MC.TF_KDDIV2}))

#define TFDIFFERENTIAL_TERM_ENABLING2  DISABLE

<#if MC.POSITION_CTRL_ENABLING2 == true >
#define POSITION_LOOP_FREQUENCY_HZ2    ${MC.POSITION_LOOP_FREQUENCY_HZ2} /*!<Execution rate of position control regulation loop (Hz) */
<#else>
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ2       ${MC.SPEED_LOOP_FREQUENCY_HZ2} /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
</#if>
#define PID_SPEED_KP_DEFAULT2          ${MC.PID_SPEED_KP_DEFAULT2}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT2          ${MC.PID_SPEED_KI_DEFAULT2}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT2          ${MC.PID_SPEED_KD_DEFAULT2}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV2                      ${MC.SP_KPDIV2}
#define SP_KIDIV2                      ${MC.SP_KIDIV2}
#define SP_KDDIV2                      ${MC.SP_KDDIV2}
#define SP_KPDIV_LOG2                  LOG2((${MC.SP_KPDIV2}))
#define SP_KIDIV_LOG2                  LOG2((${MC.SP_KIDIV2}))
#define SP_KDDIV_LOG2                  LOG2((${MC.SP_KDDIV2}))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV2 */
#define PID_SPEED_INTEGRAL_INIT_DIV2 1 
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV2 */

#define SPD_DIFFERENTIAL_TERM_ENABLING2 DISABLE
#define IQMAX2                          ${MC.IQMAX2}

/* Default settings */
<#if MC.DEFAULT_CONTROL_MODE2 == 'STC_SPEED_MODE'>
#define DEFAULT_CONTROL_MODE2           MCM_SPEED_MODE
<#else>
#define DEFAULT_CONTROL_MODE2           MCM_TORQUE_MODE
</#if>
#define DEFAULT_TARGET_SPEED_RPM2       ${MC.DEFAULT_TARGET_SPEED_RPM2}
#define DEFAULT_TARGET_SPEED_UNIT2      (DEFAULT_TARGET_SPEED_RPM2*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT2       ${MC.DEFAULT_TORQUE_COMPONENT2}
#define DEFAULT_FLUX_COMPONENT2         ${MC.DEFAULT_FLUX_COMPONENT2}

<#if  MC.POSITION_CTRL_ENABLING2 == true >
#define PID_POSITION_KP_GAIN2			${MC.POSITION_CTRL_KP_GAIN2}
#define PID_POSITION_KI_GAIN2			${MC.POSITION_CTRL_KI_GAIN2}
#define PID_POSITION_KD_GAIN2			${MC.POSITION_CTRL_KD_GAIN2}
#define PID_POSITION_KPDIV2				${MC.POSITION_CTRL_KPDIV2}     
#define PID_POSITION_KIDIV2				${MC.POSITION_CTRL_KIDIV2}
#define PID_POSITION_KDDIV2				${MC.POSITION_CTRL_KDDIV2}
#define PID_POSITION_KPDIV_LOG2			LOG2((${MC.POSITION_CTRL_KPDIV2}))    
#define PID_POSITION_KIDIV_LOG2			LOG2((${MC.POSITION_CTRL_KIDIV2})) 
#define PID_POSITION_KDDIV_LOG2			LOG2((${MC.POSITION_CTRL_KDDIV2})) 
#define PID_POSITION_ANGLE_STEP2			${MC.POSITION_CTRL_ANGLE_STEP2}
#define PID_POSITION_MOV_DURATION2		${MC.POSITION_CTRL_MOV_DURATION2}
</#if>

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/

<#if   MC.ON_OVER_VOLTAGE2 != "TURN_ON_R_BRAKE">  
#define OV_VOLTAGE_THRESHOLD_V2          ${MC.OV_VOLTAGE_THRESHOLD_V2} /*!< Over-voltage 
                                                         threshold */
<#else>
#define M2_OVP_THRESHOLD_HIGH           ${MC.M2_OVP_THRESHOLD_HIGH} /*!< Over-voltage 
                                                         hysteresis high threshold */
#define M2_OVP_THRESHOLD_LOW            ${MC.M2_OVP_THRESHOLD_LOW} /*!< Over-voltage 
                                                         hysteresis low threshold */
</#if>
#define UD_VOLTAGE_THRESHOLD_V2          ${MC.UD_VOLTAGE_THRESHOLD_V2} /*!< Under-voltage 
                                                          threshold */
#ifdef NOT_IMPLEMENTED

#define ON_OVER_VOLTAGE2                 ${MC.ON_OVER_VOLTAGE2} /*!< TURN_OFF_PWM, 
                                                         TURN_ON_R_BRAKE or 
                                                         TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
                                                         
#define OV_TEMPERATURE_THRESHOLD_C2      ${MC.OV_TEMPERATURE_THRESHOLD_C2} /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C2     ${MC.OV_TEMPERATURE_HYSTERESIS_C2} /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS2       DISABLE /*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define ALIGNMENT_DURATION2              ${MC.ALIGNMENT_DURATION2} /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG2             ${MC.ALIGNMENT_ANGLE_DEG2} /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT2               ${MC.FINAL_I_ALIGNMENT2} /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input


<#if ( MC.STATE_OBSERVER_PLL2 || MC.STATE_OBSERVER_CORDIC2 )>
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
/* USER CODE BEGIN OPENLOOP M2 */

#define OPEN_LOOP_VOLTAGE_d2           0      /*!< Three Phase voltage amplitude
                                                      in int16_t format */
#define OPEN_LOOP_SPEED_RPM2           100       /*!< Final forced speed in rpm */
#define OPEN_LOOP_SPEED_RAMP_DURATION_MS2  1000  /*!< 0-to-Final speed ramp duration  */      
#define OPEN_LOOP_VF2                  false     /*!< true to enable V/F mode */
#define OPEN_LOOP_K2                   44        /*! Slope of V/F curve expressed in int16_t Voltage for 
                                                     each 0.1Hz of mecchanical frequency increment. */
#define OPEN_LOOP_OFF2                 4400      /*! Offset of V/F curve expressed in int16_t Voltage 
                                                     applied when frequency is zero. */
/* USER CODE END OPENLOOP M2 */

  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == false inside MC.STATE_OBSERVER_PLL2 || MC.STATE_OBSERVER_CORDIC2 -->

/* Phase 1 */
#define PHASE1_DURATION2                ${MC.PHASE1_DURATION2} /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT2        (${MC.PHASE1_FINAL_SPEED_RPM2}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE1_FINAL_CURRENT2           ${MC.PHASE1_FINAL_CURRENT2}
/* Phase 2 */
#define PHASE2_DURATION2                ${MC.PHASE2_DURATION2} /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT2         (${MC.PHASE2_FINAL_SPEED_RPM2}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE2_FINAL_CURRENT2           ${MC.PHASE2_FINAL_CURRENT2}
/* Phase 3 */
#define PHASE3_DURATION2                ${MC.PHASE3_DURATION2} /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT2         (${MC.PHASE3_FINAL_SPEED_RPM2}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE3_FINAL_CURRENT2           ${MC.PHASE3_FINAL_CURRENT2}
/* Phase 4 */
#define PHASE4_DURATION2                ${MC.PHASE4_DURATION2} /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT2         (${MC.PHASE4_FINAL_SPEED_RPM2}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE4_FINAL_CURRENT2           ${MC.PHASE4_FINAL_CURRENT2}
/* Phase 5 */
#define PHASE5_DURATION2                ${MC.PHASE5_DURATION2} /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT2         (${MC.PHASE5_FINAL_SPEED_RPM2}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE5_FINAL_CURRENT2           ${MC.PHASE5_FINAL_CURRENT2}

#define ENABLE_SL_ALGO_FROM_PHASE2      ${MC.ENABLE_SL_ALGO_FROM_PHASE2}

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG2             ${MC.STARTING_ANGLE_DEG2}  /*!< degrees [0...359] */
</#if>
<#if MC.STATE_OBSERVER_PLL2 || MC.STATE_OBSERVER_CORDIC2 || MC.AUX_STATE_OBSERVER_PLL2 || MC.AUX_STATE_OBSERVER_CORDIC2>
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM2          ${MC.OBS_MINIMUM_SPEED_RPM2}
#define NB_CONSECUTIVE_TESTS2           ${MC.NB_CONSECUTIVE_TESTS2} /* corresponding to 
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT2         ${MC.SPEED_BAND_UPPER_LIMIT2} /*!< It expresses how much 
                                                            estimated speed can exceed 
                                                            forced stator electrical 
                                                            without being considered wrong. 
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT2         ${MC.SPEED_BAND_LOWER_LIMIT2}  /*!< It expresses how much 
                                                             estimated speed can be below 
                                                             forced stator electrical 
                                                             without being considered wrong. 
                                                             In 1/16 of forced speed */  
</#if>


#define TRANSITION_DURATION2            ${MC.TRANSITION_DURATION2}  /* Switch over duration, ms */

<#if   MC.BUS_VOLTAGE_READING2 >
/******************************   BUS VOLTAGE  Motor 2  **********************/
#define  M2_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(${MC.VBUS_ADC_SAMPLING_TIME2})
</#if>
<#if  MC.TEMPERATURE_READING2 >
/******************************   Temperature sensing Motor 2  **********************/
#define  M2_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(${MC.TEMP_ADC_SAMPLING_TIME2})
</#if>

/******************************   Current sensing Motor 2   **********************/
#define ADC_SAMPLING_CYCLES2 (${MC.CURR_SAMPLING_TIME2} + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/



<#if MC.FLUX_WEAKENING_ENABLING2>
#define FW_VOLTAGE_REF2                ${MC.FW_VOLTAGE_REF2} /*!<Vs reference, tenth 
                                                        of a percent */
#define FW_KP_GAIN2                    ${MC.FW_KP_GAIN2} /*!< Default Kp gain */
#define FW_KI_GAIN2                    ${MC.FW_KI_GAIN2} /*!< Default Ki gain */
#define FW_KPDIV2                      ${MC.FW_KPDIV2}      
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define FW_KIDIV2                      ${MC.FW_KIDIV2}
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG2                  LOG2((${MC.FW_KPDIV2}))
#define FW_KIDIV_LOG2                  LOG2((${MC.FW_KIDIV2}))
</#if>
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING2>
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING2 <#if MC.FEED_FORWARD_CURRENT_REG_ENABLING2==true>ENABLE<#else>DISABLE</#if>
#define CONSTANT1_Q2                    ${MC.CONSTANT1_Q2}
#define CONSTANT1_D2                    ${MC.CONSTANT1_D2}
#define CONSTANT2_QD2                   ${MC.CONSTANT2_QD2}
</#if>

<#if MC.MTPA_ENABLING2>
/*  Maximum Torque Per Ampere strategy parameters */
#define MTPA_ENABLING2
#define SEGDIV2                         ${MC.SEGDIV2}
#define ANGC2                           ${MC.ANGC2}
#define OFST2                           ${MC.OFST2}                  
</#if>

<#if MC.M2_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M2_ICL_RELAY_SWITCHING_DELAY_MS ${MC.M2_ICL_RELAY_SWITCHING_DELAY_MS}  /* milliseconds */
#define M2_ICL_CAPS_CHARGING_DELAY_MS   ${MC.M2_ICL_CAPS_CHARGING_DELAY_MS}  /* milliseconds */  
#define M2_ICL_VOLTAGE_THRESHOLD        ${MC.M2_ICL_VOLTAGE_THRESHOLD}  /* volts */               
</#if>
/*** On the fly start-up ***/
<#-- ToDo: On the Fly start-up -->

</#if>

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
