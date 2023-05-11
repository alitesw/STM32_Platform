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
<#assign AUX_SPEED_FDBK_M1 = (MC.AUX_HALL_SENSORS == true || MC.AUX_ENCODER || MC.AUX_STATE_OBSERVER_PLL || MC.AUX_STATE_OBSERVER_CORDIC)>
<#assign AUX_SPEED_FDBK_M2 = (MC.AUX_HALL_SENSORS2 == true || MC.AUX_ENCODER2 || MC.AUX_STATE_OBSERVER_PLL2 || MC.AUX_STATE_OBSERVER_CORDIC2)>
<#assign G4_Cut2_2_patch = CondFamily_STM32G4 > 
<#assign NoInjectedChannel = (CondFamily_STM32F0 || CondFamily_STM32G0 || G4_Cut2_2_patch ) >	
<#assign DWT_CYCCNT_SUPPORTED = !(CondFamily_STM32F0 || CondFamily_STM32G0) >

  <#if  MC.STATE_OBSERVER_PLL == true>
    <#assign SPD_M1   = "&STO_PLL_M1">
    <#assign SPD_init_M1 = "STO_PLL_Init" >
    <#assign SPD_calcAvrgMecSpeedUnit_M1 = "STO_PLL_CalcAvrgMecSpeedUnit" >
	<#assign SPD_calcElAngle_M1 = "( void )STO_PLL_CalcElAngle" >    
	<#assign SPD_calcAvergElSpeedDpp_M1 = "STO_PLL_CalcAvrgElSpeedDpp"> 
	<#assign SPD_clear_M1 = "STO_PLL_Clear">  
  <#elseif  MC.STATE_OBSERVER_CORDIC == true>
    <#assign SPD_M1 = "&STO_CR_M1" >
    <#assign SPD_init_M1 = "STO_CR_Init" >
    <#assign SPD_calcElAngle_M1 = "STO_CR_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M1 = "STO_CR_CalcAvrgMecSpeedUnit" >
    <#assign SPD_calcAvergElSpeedDpp_M1 = "STO_CR_CalcAvrgElSpeedDpp"> 
	<#assign SPD_clear_M1 = "STO_CR_Clear">  
  <#elseif  MC.HALL_SENSORS == true>
    <#assign SPD_M1 = "&HALL_M1" >
    <#assign SPD_init_M1 = "HALL_Init" >
    <#assign SPD_calcElAngle_M1 = "HALL_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M1 = "HALL_CalcAvrgMecSpeedUnit" >
	<#assign SPD_clear_M1 = "HALL_Clear">  
  <#elseif  MC.ENCODER == true>
    <#assign SPD_M1 = "&ENCODER_M1" >
	<#assign SPD_init_M1 = "ENC_Init" >
    <#assign SPD_calcElAngle_M1 = "ENC_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M1 = "ENC_CalcAvrgMecSpeedUnit" >
	<#assign SPD_clear_M1 = "ENC_Clear">  
  <#elseif  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC"	>
    <#assign SPD_M1 = "&Bemf_ADC_M1" >
	<#assign SPD_init_M1 = "BADC_Init" >
    <#assign SPD_calcElAngle_M1 = "BADC_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M1 = "BADC_CalcAvrgMecSpeedUnit" >
	<#assign SPD_clear_M1 = "BADC_Clear">
  </#if>
  <#if  AUX_SPEED_FDBK_M1 == true>
    <#if   MC.AUX_STATE_OBSERVER_PLL == true>
      <#assign SPD_AUX_M1 = "&STO_PLL_M1">
	  <#assign SPD_aux_init_M1 = "STO_PLL_Init">
	  <#assign SPD_aux_calcAvrgMecSpeedUnit_M1 ="STO_PLL_CalcAvrgMecSpeedUnit">
	  <#assign SPD_aux_calcAvrgElSpeedDpp_M1 = "STO_PLL_CalcAvrgElSpeedDpp">
	  <#assign SPD_aux_calcElAngle_M1 = "( void )STO_PLL_CalcElAngle">
	  <#assign SPD_aux_clear_M1 = "STO_PLL_Clear">
    <#elseif  MC.AUX_STATE_OBSERVER_CORDIC == true>
      <#assign SPD_AUX_M1 = "&STO_CR_M1">
	  <#assign SPD_aux_init_M1 = "STO_CR_Init">
	  <#assign SPD_aux_calcAvrgMecSpeedUnit_M1 = "STO_CR_CalcAvrgMecSpeedUnit">
	  <#assign SPD_aux_calcAvrgElSpeedDpp_M1 = "STO_CR_CalcAvrgElSpeedDpp">
	  <#assign SPD_aux_calcElAngle_M1 = "STO_CR_CalcElAngle">
	  <#assign SPD_aux_clear_M1 = "STO_CR_Clear">
	<#elseif  MC.AUX_HALL_SENSORS == true>
      <#assign SPD_AUX_M1 = "&HALL_M1">
	  <#assign SPD_aux_init_M1 = "HALL_Init" >
	  <#assign SPD_aux_calcAvrgMecSpeedUnit_M1 = "HALL_CalcAvrgMecSpeedUnit">
	  <#assign SPD_aux_clear_M1 = "HALL_Clear">
    <#elseif  MC.AUX_ENCODER == true>
      <#assign SPD_AUX_M1 = "&ENCODER_M1">
	  <#assign SPD_aux_init_M1 = "ENC_Init" >
	  <#assign SPD_aux_calcAvrgMecSpeedUnit_M1 = "ENC_CalcAvrgMecSpeedUnit">
	  <#assign SPD_aux_clear_M1 = "ENC_Clear">
    </#if>
  </#if>
    <#if  MC.STATE_OBSERVER_PLL2 == true>
    <#assign SPD_M2   = "&STO_PLL_M2">
    <#assign SPD_init_M2 = "STO_PLL_Init" >
    <#assign SPD_calcAvrgMecSpeedUnit_M2 = "STO_PLL_CalcAvrgMecSpeedUnit" >
	<#assign SPD_calcElAngle_M2 = "( void )STO_PLL_CalcElAngle" >   /* if not sensorless then 2nd parameter is MC_NULL*/    
	  <#assign SPD_calcAvergElSpeedDpp_M2 = "STO_PLL_CalcAvrgElSpeedDpp"> 
	  <#assign SPD_clear_M2 = "STO_PLL_Clear">  
  <#elseif  MC.STATE_OBSERVER_CORDIC2 == true>
    <#assign SPD_M2 = "&STO_CR_M2" >
    <#assign SPD_init_M2 = "STO_CR_Init" >
    <#assign SPD_calcElAngle_M2 = "STO_CR_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M2 = "STO_CR_CalcAvrgMecSpeedUnit" >
    <#assign SPD_calcAvergElSpeedDpp_M2 = "STO_CR_CalcAvrgElSpeedDpp"> 
	  <#assign SPD_clear_M2 = "STO_CR_Clear">  
  <#elseif  MC.HALL_SENSORS2 == true>
    <#assign SPD_M2 = "&HALL_M2" >
    <#assign SPD_init_M2 = "HALL_Init" >
    <#assign SPD_calcElAngle_M2 = "HALL_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M2 = "HALL_CalcAvrgMecSpeedUnit" >
	  <#assign SPD_clear_M2 = "HALL_Clear">  
  <#elseif  MC.ENCODER2 == true>
    <#assign SPD_M2 = "&ENCODER_M2" >
	  <#assign SPD_init_M2 = "ENC_Init" >
    <#assign SPD_calcElAngle_M2 = "ENC_CalcElAngle" >
    <#assign SPD_calcAvrgMecSpeedUnit_M2 = "ENC_CalcAvrgMecSpeedUnit" >
	  <#assign SPD_clear_M2 = "ENC_Clear">  
  </#if>
  <#if  AUX_SPEED_FDBK_M2 == true>
    <#if   MC.AUX_STATE_OBSERVER_PLL2 == true>
      <#assign SPD_AUX_M2 = "&STO_PLL_M2">
	    <#assign SPD_aux_init_M2 = "STO_PLL_Init" >
	    <#assign SPD_aux_calcAvrgMecSpeedUnit_M2 ="STO_PLL_CalcAvrgMecSpeedUnit">
	    <#assign SPD_aux_calcAvrgElSpeedDpp_M2 = "STO_PLL_CalcAvrgElSpeedDpp">
	  <#assign SPD_aux_calcElAngle_M2 = "( void )STO_PLL_CalcElAngle">
	    <#assign SPD_aux_clear_M2 = "STO_PLL_Clear">
    <#elseif  MC.AUX_STATE_OBSERVER_CORDIC2 == true>
      <#assign SPD_AUX_M2 = "&STO_CR_M2">
	    <#assign SPD_aux_init_M2 = "STO_CR_Init" >
	    <#assign SPD_aux_calcAvrgMecSpeedUnit_M2 = "STO_CR_CalcAvrgMecSpeedUnit">
	    <#assign SPD_aux_calcAvrgElSpeedDpp_M2 = "STO_CR_CalcAvrgElSpeedDpp">
	    <#assign SPD_aux_calcElAngle_M2 = "STO_CR_CalcElAngle">
	    <#assign SPD_aux_clear_M2 = "STO_CR_Clear">
	<#elseif  MC.AUX_HALL_SENSORS2 == true>
      <#assign SPD_AUX_M2 = "&HALL_M2">
	    <#assign SPD_aux_init_M2 = "HALL_Init" >
	    <#assign SPD_aux_calcElAngle_M2 = "HALL_CalcElAngle">
	    <#assign SPD_aux_calcAvrgMecSpeedUnit_M2 = "HALL_CalcAvrgMecSpeedUnit">
	    <#assign SPD_aux_clear_M2 = "HALL_Clear">
    <#elseif  MC.AUX_ENCODER2 == true>
      <#assign SPD_AUX_M2 = "&ENCODER_M2">
	    <#assign SPD_aux_init_M2 = "ENC_Init" >
	    <#assign SPD_aux_calcElAngle_M2 = "ENC_CalcElAngle">
	    <#assign SPD_aux_calcAvrgMecSpeedUnit_M2 = "ENC_CalcAvrgMecSpeedUnit">
	    <#assign SPD_aux_clear_M2 = "ENC_Clear">
    </#if>
  </#if>
<#if MC.DRIVE_TYPE == "FOC">
  <#if CondFamily_STM32F3 &&  MC.THREE_SHUNT == true>
	<#assign PWM_Init = "R3_1_Init">
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_1_GetPhaseCurrents">
<#elseif CondFamily_STM32F3 &&  MC.SINGLE_SHUNT == true>
	<#assign PWM_Init = "R1_Init"> 
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R1_GetPhaseCurrents">
<#elseif CondFamily_STM32F3 &&  (MC.THREE_SHUNT_SHARED_RESOURCES ||  MC.THREE_SHUNT_INDEPENDENT_RESOURCES )>
	<#assign PWM_Init = "R3_2_Init">
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents">
<#elseif CondFamily_STM32F4 &&  MC.THREE_SHUNT == true>
  <#assign PWM_Handle_M1 = "PWMC_R3_1_Handle_M1">
	<#assign PWM_Init = "R3_1_Init">  
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_1_GetPhaseCurrents">
<#elseif CondFamily_STM32F4 &&  MC.SINGLE_SHUNT == true> 
	<#assign PWM_Init = "R1_Init"> 
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R1_GetPhaseCurrents">
<#elseif CondFamily_STM32F4 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
	<#assign PWM_Init = "R3_2_Init">
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents">
<#elseif CondFamily_STM32F0 &&  MC.SINGLE_SHUNT == true> 
	<#assign PWM_Init = "R1_Init">  
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="">
<#elseif CondFamily_STM32F0 &&  MC.THREE_SHUNT == true> 
	<#assign PWM_Init = "R3_1_Init">  
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="">
<#elseif CondFamily_STM32G0 &&  MC.SINGLE_SHUNT == true> 
	<#assign PWM_Init = "R1_Init">  
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="">
<#elseif CondFamily_STM32G0 &&  MC.THREE_SHUNT == true> 
	<#assign PWM_Init = "R3_1_Init">  
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="">
<#elseif CondFamily_STM32F3 && MC.ICS_SENSORS == true> 
	<#assign PWM_Init = "ICS_Init"> 
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">
<#elseif CondFamily_STM32F4 && MC.ICS_SENSORS == true> 
	<#assign PWM_Init = "ICS_Init">
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">
<#elseif CondFamily_STM32G4 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>   
 	<#assign PWM_Init = "R3_2_Init">  
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents"> 
	<#assign PWM_GetCalibStatus ="R3_2_GetCalibrationStatus"> 	
<#elseif CondFamily_STM32G4 && MC.SINGLE_SHUNT == true>   
 	<#assign PWM_Init = "R1_Init">  
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R1_GetPhaseCurrents">   
<#elseif CondFamily_STM32G4 && MC.ICS_SENSORS == true>   
 	<#assign PWM_Init = "ICS_Init">  
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">  				
</#if>
<#if CondFamily_STM32L4 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
	<#assign PWM_Init = "R3_2_Init">
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents">
<#elseif CondFamily_STM32L4 &&  MC.SINGLE_SHUNT == true>
	<#assign PWM_Init = "R1_Init"> 
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R1L4XX_GetPhaseCurrents">
<#elseif CondFamily_STM32L4 &&  MC.THREE_SHUNT == true>
	<#assign PWM_Init = "R3_1_Init"> 
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R3_1_GetPhaseCurrents">	
<#elseif CondFamily_STM32L4 &&  MC.ICS_SENSORS == true>
	<#assign PWM_Init = "ICS_Init"> 
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">	
</#if>
<#if CondFamily_STM32F7 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
	<#assign PWM_Init = "R3_2_Init">
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents">
<#elseif CondFamily_STM32F7 &&  MC.SINGLE_SHUNT == true>
	<#assign PWM_Init = "R1_Init"> 
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R1F7XX_GetPhaseCurrents">
<#elseif CondFamily_STM32F7 &&  MC.THREE_SHUNT == true>
	<#assign PWM_Init = "R3_1_Init"> 
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R3_1_GetPhaseCurrents">	
<#elseif CondFamily_STM32F7 &&  MC.ICS_SENSORS == true>
	<#assign PWM_Init = "ICS_Init"> 
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">	
</#if>
<#if CondFamily_STM32H7 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true>
	<#assign PWM_Init = "R3_2_Init">
	<#assign PWM_TurnOnLowSides = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents ="R3_2_GetPhaseCurrents">
<#elseif CondFamily_STM32H7 &&  MC.SINGLE_SHUNT == true>
	<#assign PWM_Init = "R1_Init"> 
	<#assign PWM_TurnOnLowSides = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R1_GetPhaseCurrents">
<#elseif CondFamily_STM32H7 &&  MC.THREE_SHUNT == true>
	<#assign PWM_Init = "R3_1_Init"> 
	<#assign PWM_TurnOnLowSides = "R3_1_TurnOnLowSides">
	<#assign PWM_SwitchOn = "R3_1_SwitchOnPWM">
	<#assign PWM_SwitchOff = "R3_1_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="R3_1_GetPhaseCurrents">	
<#elseif CondFamily_STM32H7 &&  MC.ICS_SENSORS == true>
	<#assign PWM_Init = "ICS_Init"> 
	<#assign PWM_TurnOnLowSides = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ICS_SwitchOffPWM">	
	<#assign PWM_GetPhaseCurrents ="ICS_GetPhaseCurrents">	
</#if>
<#if CondFamily_STM32F3 &&  MC.SINGLE_SHUNT2 == true>
	<#assign PWM_Init_M2 = "R1_Init">
	<#assign PWM_TurnOnLowSides_M2 = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">  
<#elseif CondFamily_STM32F3 && (MC.THREE_SHUNT_SHARED_RESOURCES2 ||  MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 )>
	<#assign PWM_Init_M2 = "R3_2_Init">
	<#assign PWM_TurnOnLowSides_M2 = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">  
<#elseif CondFamily_STM32F4 &&  MC.SINGLE_SHUNT2 == true>
	<#assign PWM_Init_M2 = "R1_Init">
	<#assign PWM_TurnOnLowSides_M2 = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">    
<#elseif CondFamily_STM32F4 &&  MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true> 
	<#assign PWM_Init_M2 = "R3_2_Init">
	<#assign PWM_TurnOnLowSides_M2 = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">
<#elseif CondFamily_STM32F3 && MC.ICS_SENSORS2 == true> 
	<#assign PWM_Init_M2 = "ICS_Init">
	<#assign PWM_TurnOnLowSides_M2 = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">    
<#elseif CondFamily_STM32F4 && MC.ICS_SENSORS2 == true> 
 	<#assign PWM_Init_M2 = "ICS_Init">
	<#assign PWM_TurnOnLowSides_M2 = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="">  
<#elseif CondFamily_STM32G4 && MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true>   
 	<#assign PWM_Init_M2 = "R3_2_Init">  
	<#assign PWM_TurnOnLowSides_M2 = "R3_2_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R3_2_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R3_2_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="R3_2_GetPhaseCurrents"> 
<#elseif CondFamily_STM32G4 && MC.SINGLE_SHUNT2 == true>   
 	<#assign PWM_Init_M2 = "R1_Init">  
	<#assign PWM_TurnOnLowSides_M2 = "R1_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "R1_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "R1_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="R1_GetPhaseCurrents">  	
<#elseif CondFamily_STM32G4 && MC.ICS_SENSORS2 == true>   
 	<#assign PWM_Init_M2 = "ICS_Init">  
	<#assign PWM_TurnOnLowSides_M2 = "ICS_TurnOnLowSides">
	<#assign PWM_SwitchOn_M2 = "ICS_SwitchOnPWM">
	<#assign PWM_SwitchOff_M2 = "ICS_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents_M2 ="ICS_GetPhaseCurrents">  		
</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
<#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
	<#assign PWM_Init = "SixPwm_Init">
	<#assign PWM_TurnOnLowSides = "SixPwm_TurnOnLowSides">
	<#assign PWM_SwitchOn = "SixPwm_SwitchOnPWM">
	<#assign PWM_SwitchOff = "SixPwm_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents =""> 
<#else>
	<#assign PWM_Init = "ThreePwm_Init">
	<#assign PWM_TurnOnLowSides = "ThreePwm_TurnOnLowSides">
	<#assign PWM_SwitchOn = "ThreePwm_SwitchOnPWM">
	<#assign PWM_SwitchOff = "ThreePwm_SwitchOffPWM">
	<#assign PWM_GetPhaseCurrents =""> 
</#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
<#-- Charge Boot Cap enable condition -->
<#assign CHARGE_BOOT_CAP_ENABLING = ! MC.OTF_STARTUP>
<#assign CHARGE_BOOT_CAP_ENABLING2 = ! MC.OTF_STARTUP2>
<#if MC.OVERMODULATION == true> <#assign OVM ="_OVM"> <#else> <#assign OVM =""> </#if>
<#if MC.OVERMODULATION2 == true> <#assign OVM2 ="_OVM"> <#else>	<#assign OVM2 =""> </#if>

/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1 
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
<#if MC.RTOS == "FREERTOS">
#include "cmsis_os.h"
</#if>
#include "mc_interface.h"
#include "digital_output.h"
<#if MC.DRIVE_TYPE == "FOC">
#include "pwm_common.h"
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
#include "pwm_common_sixstep.h"
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->


#include "mc_tasks.h"
#include "parameters_conversion.h"
<#if MC.MCP_USED == true>
#include "mcp_config.h"
</#if>
<#if MC.DAC_FUNCTIONALITY>
#include "dac_ui.h"
</#if>
#include "mc_app_hooks.h"

<#if MC.TESTENV == true && MC.DRIVE_TYPE == "FOC" && MC.PFC_ENABLED == false >
#include "mc_testenv.h"

</#if>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */

/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

<#if MC.DRIVE_TYPE == "FOC">
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
#define STOPPERMANENCY_MS   ((uint16_t)400)
#define STOPPERMANENCY_MS2  ((uint16_t)400)
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * ${MC.M1_PWM_CHARGE_BOOT_CAP_MS}) / ((uint16_t)1000))
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * ${MC.M2_PWM_CHARGE_BOOT_CAP_MS})/ ((uint16_t)1000))
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES  (uint32_t)(${MC.M1_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}*(PWM_PERIOD_CYCLES/2))
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(${MC.M2_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}*(PWM_PERIOD_CYCLES2/2))
<#if MC.DRIVE_TYPE == "FOC">
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))
<#if MC.DRIVE_TYPE == "SIX_STEP">
#define S16_90_PHASE_SHIFT  (int16_t)(65536/4)
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

/* USER CODE END Private define */
<#if MC.OV_TEMPERATURE_PROT_ENABLING == true &&  MC.UV_VOLTAGE_PROT_ENABLING == true && MC.OV_VOLTAGE_PROT_ENABLING == true>
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
<#else>
<#if  MC.UV_VOLTAGE_PROT_ENABLING == false>
<#assign UV_ERR = "MC_UNDER_VOLT">
<#else>
<#assign UV_ERR = "0">
</#if>
<#if MC.OV_VOLTAGE_PROT_ENABLING == false>
<#assign OV_ERR = "MC_OVER_VOLT">
<#else>
<#assign OV_ERR = "0">
</#if>
<#if MC.OV_TEMPERATURE_PROT_ENABLING == false>
<#assign OT_ERR = "MC_OVER_TEMP">
<#else>
<#assign OT_ERR = "0">
</#if>
#define VBUS_TEMP_ERR_MASK ~(${OV_ERR} | ${UV_ERR} | ${OT_ERR})
</#if>
<#if   MC.DUALDRIVE == true>
<#if MC.OV_TEMPERATURE_PROT_ENABLING2 == true &&  MC.UV_VOLTAGE_PROT_ENABLING2 == true && MC.OV_VOLTAGE_PROT_ENABLING2 == true>
#define VBUS_TEMP_ERR_MASK2 (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
<#else>
<#if  MC.UV_VOLTAGE_PROT_ENABLING2 == false>
<#assign UV_ERR2 = "MC_UNDER_VOLT">
<#else>
<#assign UV_ERR2 = "0">
</#if>
<#if MC.OV_VOLTAGE_PROT_ENABLING2 == false>
<#assign OV_ERR2 = "MC_OVER_VOLT">
<#else>
<#assign OV_ERR2 = "0">
</#if>
<#if MC.OV_TEMPERATURE_PROT_ENABLING == false>
<#assign OT_ERR2 = "MC_OVER_TEMP">
<#else>
<#assign OT_ERR2 = "0">
</#if>
#define VBUS_TEMP_ERR_MASK2 ~(${OV_ERR2} | ${UV_ERR2} | ${OT_ERR2})
</#if>
</#if>
/* Private variables----------------------------------------------------------*/

<#if MC.DRIVE_TYPE == "FOC">  
static FOCVars_t FOCVars[NBR_OF_MOTORS];
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
static SixStepVars_t SixStepVars[NBR_OF_MOTORS];
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
<#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >
static EncAlign_Handle_t *pEAC[NBR_OF_MOTORS];
</#if>

<#if  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE == true>
<#if  MC.SINGLEDRIVE == true>
static uint16_t nominalBusd[1] = {0u};
static uint16_t ovthd[1] = {OVERVOLTAGE_THRESHOLD_d};
<#else> 
static uint16_t nominalBusd[2] = {0u,0u};
static uint16_t ovthd[2] = {OVERVOLTAGE_THRESHOLD_d,OVERVOLTAGE_THRESHOLD_d2};
</#if>
</#if>
static PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
<#if   (MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE") || (MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE")>  
static DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
</#if>
<#if  (MC.HW_OV_CURRENT_PROT_BYPASS == true) &&  (MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES") || ( MC.HW_OV_CURRENT_PROT_BYPASS == true)>
static DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
</#if>
<#if MC.DRIVE_TYPE == "FOC"> 
//cstat !MISRAC2012-Rule-8.9_a
static RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/
</#if>
<#if  MC.MTPA_ENABLING == true || MC.MTPA_ENABLING2 == true>
static MTPA_Handle_t *pMaxTorquePerAmpere[2] = {MC_NULL,MC_NULL}; 
</#if>
<#if ( MC.DUALDRIVE == true &&  MC.M2_DBG_OPEN_LOOP_ENABLE == true)>
OpenLoop_Handle_t *pOpenLoop[2] = {MC_NULL,MC_NULL};  /* only if M1 or M2 has OPEN LOOP */
<#elseif ( MC.SINGLEDRIVE == true &&  MC.M1_DBG_OPEN_LOOP_ENABLE == true)>
OpenLoop_Handle_t *pOpenLoop[1] = {MC_NULL};          /* only if M1 has OPEN LOOP */
</#if>

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
<#if  MC.DUALDRIVE == true>
static volatile uint16_t hMFTaskCounterM2 = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);
</#if>

static volatile uint8_t bMCBootCompleted = ((uint8_t)0);

<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
/* Performs the CPU load measure of FOC main tasks. */
MC_Perf_Handle_t PerfTraces;
</#if>
</#if>

<#if (MC.M1_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM1 = true; 
</#if>
<#if (MC.M2_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM2 = true; 
</#if>

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
<#if MC.DRIVE_TYPE == "FOC">
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
void TSK_MF_StopProcessing(  MCI_Handle_t * pHandle, uint8_t motor);
MCI_Handle_t * GetMCI(uint8_t bMotor);
<#if MC.DRIVE_TYPE == "FOC">
<#if MC.MOTOR_PROFILER != true>
static uint16_t FOC_CurrControllerM1(void);
<#if  MC.DUALDRIVE == true>
static uint16_t FOC_CurrControllerM2(void);
</#if>
<#else>
bool SCC_DetectBemf( SCC_Handle_t * pHandle );
</#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
void SixStep_Clear(uint8_t bMotor);
void SixStep_InitAdditionalMethods(uint8_t bMotor);
void SixStep_CalcSpeedRef(uint8_t bMotor);
static uint16_t SixStep_StatorController(void);
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
<#if MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM">
void TSK_SafetyTask_PWMOFF(uint8_t motor);
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
void TSK_SafetyTask_RBRK(uint8_t motor);
</#if>
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES">
void TSK_SafetyTask_LSON(uint8_t motor);
</#if>
<#if  MC.DUALDRIVE == true>
void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
bool TSK_StopPermanencyTimeHasElapsedM2(void);

#define FOC_ARRAY_LENGTH 2
static uint8_t FOC_array[FOC_ARRAY_LENGTH]={0, 0};
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived
</#if>
<#if MC.PFC_ENABLED == true>
void PFC_Scheduler(void);
</#if>

<#if  MC.EXAMPLE_SPEEDMONITOR == true>
/****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/
   void ARR_TIM5_update(SpeednPosFdbk_Handle_t pSPD);
/**************************************************************************/
</#if>
/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
  
  if (MC_NULL == pMCIList)
  {
    /* Nothing to do */
  }
  else
  {
    <#if MC.TESTENV == true && MC.DRIVE_TYPE == "FOC" && MC.PFC_ENABLED == false >
    mc_testenv_init();
    </#if>
   
  <#if MC.USE_STGAP1S>
    /**************************************/
    /*    STGAP1AS initialization         */
    /**************************************/
    if (false == GAP_Configuration(&STGAP_M1))
    {
      MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);
    }
  </#if>  
    bMCBootCompleted = (uint8_t )0;
  <#if MC.DRIVE_TYPE == "FOC">
  </#if>  
  
  <#if  MC.MTPA_ENABLING == true>
    pMaxTorquePerAmpere[M1] = &MTPARegM1;
  </#if>
  <#if  MC.HW_OV_CURRENT_PROT_BYPASS == true &&  MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
    pOCPDisabling[M1] = &DOUT_OCPDisablingParamsM1;
    DOUT_SetOutputState(pOCPDisabling[M1],INACTIVE);
  </#if>
  <#if  MC.DUALDRIVE == true>
    <#if  MC.MTPA_ENABLING2 == true>
    pMaxTorquePerAmpere[M2] = &MTPARegM2;
    </#if>
  
  
  <#if  MC.HW_OV_CURRENT_PROT_BYPASS2 == true &&  MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES"> 
    pOCPDisabling[M2] = &DOUT_OCPDisablingParamsM2;
    DOUT_SetOutputState(pOCPDisabling[M2],INACTIVE);
  </#if> 
  </#if> 
    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    ${PWM_Init}(&PWM_Handle_M1);
  <#if  MC.DUALDRIVE == true> 
    pwmcHandle[M2] = &PWM_Handle_M2._Super;
    ${PWM_Init_M2}(&PWM_Handle_M2); 
  </#if>
  <#if MC.MCP_OVER_UART_A >
    ASPEP_start(&aspepOverUartA);
  </#if>
  <#if MC.MCP_OVER_UART_B >
    ASPEP_start(&aspepOverUartB);
  </#if>
  <#if MC.MCP_OVER_STLNK >  
    STLNK_init(&STLNK);
  </#if>
<#if MC.DRIVE_TYPE == "SIX_STEP" && MC.DRIVE_MODE == "CM">
    CRM_Init(&CurrentRef_M1);
</#if>

    /* USER CODE BEGIN MCboot 1 */
  
    /* USER CODE END MCboot 1 */
<#if MC.DRIVE_TYPE == "FOC">
  <#if !CondFamily_STM32F0 && !CondFamily_STM32G0 >
    /**************************************/
    /*    Start timers synchronously      */
    /**************************************/
    startTimers();    
  </#if>
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
  
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);
    
    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    ${SPD_init_M1} (${SPD_M1});

<#if MC.DRIVE_TYPE == "FOC">
  <#if  MC.ENCODER == true>
    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,${SPD_M1});
    pEAC[M1] = &EncAlignCtrlM1;
    
  </#if>  
  
  <#if  MC.POSITION_CTRL_ENABLING == true>
    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/
  PID_HandleInit(&PID_PosParamsM1);
  
  TC_Init(&PosCtrlM1, &PID_PosParamsM1, &SpeednTorqCtrlM1, &ENCODER_M1);
  </#if>  
  
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, ${SPD_M1}._Super);

<#if MC.DRIVE_TYPE == "FOC">
   <#if  AUX_SPEED_FDBK_M1  == true  >
    /******************************************************/
    /*   Auxiliary speed sensor component initialization  */
    /******************************************************/
    ${SPD_aux_init_M1} (${SPD_AUX_M1});
    
   <#if  MC.AUX_ENCODER == true>
    /***********************************************************/
    /*   Auxiliary encoder alignment component initialization  */
    /***********************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,${SPD_AUX_M1});
    pEAC[M1] = &EncAlignCtrlM1;
   </#if>
   </#if>
 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if   MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true ||  MC.ENCODER == true ||  MC.AUX_ENCODER == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC"> 
    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/ 
    VSS_Init(&VirtualSpeedSensorM1);
    
  </#if>
  <#if   MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC"> 
    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    <#if MC.DRIVE_TYPE == "FOC">
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);  

  </#if>
  <#if MC.DRIVE_TYPE == "SIX_STEP">
    RUC_Init(&RevUpControlM1,pSTC[M1],&VirtualSpeedSensorM1); 
  </#if>
  </#if>

  <#if MC.DRIVE_TYPE == "FOC">
    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->  

  <#if   MC.BUS_VOLTAGE_READING == true>
    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    RVBS_Init(&BusVoltageSensor_M1);
    
  <#else>
    /**********************************************************/
    /*   Virtual bus voltage sensor component initialization  */
    /**********************************************************/
    VVBS_Init(&BusVoltageSensor_M1);
  </#if>
 
<#if MC.DRIVE_TYPE == "FOC">
    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];
    
  <#if   MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">  
    pR_Brake[M1] = &R_BrakeParamsM1;
    DOUT_SetOutputState(pR_Brake[M1],INACTIVE);

  </#if> 
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    NTC_Init(&TempSensor_M1);    
      
<#if MC.DRIVE_TYPE == "FOC">
  <#if  MC.FLUX_WEAKENING_ENABLING == true>
    /*******************************************************/
    /*   Flux weakening component initialization           */
    /*******************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M1);
    FW_Init(pFW[M1],&PIDSpeedHandle_M1,&PIDFluxWeakeningHandle_M1);   
               

  </#if>
  <#if  MC.FEED_FORWARD_CURRENT_REG_ENABLING == true>
    /*******************************************************/
    /*   Feed forward component initialization             */
    /*******************************************************/
    FF_Init(pFF[M1],&(BusVoltageSensor_M1._Super),pPIDId[M1],pPIDIq[M1]);  
    
  </#if>
  <#if  MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    OL_Init(&OpenLoop_ParamsM1, &VirtualSpeedSensorM1);     /* only if M1 has open loop */
    pOpenLoop[M1] = &OpenLoop_ParamsM1;
  </#if>
  
    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);
  
  <#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>
    SCC.pPWMC = pwmcHandle[M1];
    SCC.pVBS = &BusVoltageSensor_M1;
    SCC.pFOCVars = &FOCVars[M1];
    SCC.pMCI = &Mci[M1];
    SCC.pVSS = &VirtualSpeedSensorM1;
    SCC.pCLM = &CircleLimitationM1;
    SCC.pPIDIq = pPIDIq[M1];
    SCC.pPIDId = pPIDId[M1];
    SCC.pRevupCtrl = &RevUpControlM1;
    SCC.pSTO = &STO_PLL_M1;
    SCC.pSTC = &SpeednTorqCtrlM1;
    SCC.pOTT = &OTT;
    <#if MC.AUX_HALL_SENSORS>
    SCC.pHT = &HT;
    <#else>
    SCC.pHT = MC_NULL;
    </#if>
    SCC_Init(&SCC);
  
    OTT.pSpeedSensor = &STO_PLL_M1._Super;
    OTT.pFOCVars = &FOCVars[M1];
    OTT.pPIDSpeed = &PIDSpeedHandle_M1;
    OTT.pSTC = &SpeednTorqCtrlM1;
    OTT_Init(&OTT);
</#if>

    
    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
  <#if MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>
  <#if MC.POSITION_CTRL_ENABLING == true>  
    MCI_Init(&Mci[M1], pSTC[M1], &FOCVars[M1], pPosCtrl[M1], pwmcHandle[M1]);
  <#else>
    MCI_Init(&Mci[M1], pSTC[M1], &FOCVars[M1], MC_NULL,pwmcHandle[M1]);
  </#if>
  <#else>
    MCI_Init(&Mci[M1], pSTC[M1], &FOCVars[M1],pwmcHandle[M1] );
  </#if>
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>  
    Mci[M1].pVSS =  &VirtualSpeedSensorM1;
    MCI_SetSpeedMode(&Mci[M1]);
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
    SixStep_Clear(M1);
    SixStepVars[M1].bDriveInput = EXTERNAL;
    MCI_Init(&Mci[M1], pSTC[M1], &SixStepVars[M1], pwmcHandle[M1] );
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
<#if MC.DEFAULT_CONTROL_MODE == "STC_TORQUE_MODE">
    MCI_ExecTorqueRamp(&Mci[M1], STC_GetDefaultIqdref(pSTC[M1]).q, 0);
<#else>   
    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /*First command to STC*/
</#if><#-- MC.DEFAULT_CONTROL_MODE == "STC_TORQUE_MODE"-->    
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
    Mci[M1].pPerfMeasure = &PerfTraces;
    MC_Perf_Measure_Init(&PerfTraces);
</#if>
</#if>
    pMCIList[M1] = &Mci[M1];
  

<#if  MC.MOTOR_PROFILER && MC.AUX_HALL_SENSORS>
    HT.pOTT = &OTT;
    HT.pMCI = &Mci[M1];
    HT.pHALL_M1 = &HALL_M1;
    HT.pSTO_PLL_M1 = &STO_PLL_M1;
    HT_Init(&HT, false);
 </#if>
   
  <#if  MC.DUALDRIVE == true>
  
    /******************************************************/
    /*   Motor 2 features initialization                  */
    /******************************************************/
    
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/  
    PID_HandleInit(&PIDSpeedHandle_M2);
    
    /***********************************************************/
    /*   Main speed  sensor initialization: speed regulation   */
    /***********************************************************/ 
    ${SPD_init_M2} (${SPD_M2});
  <#if  MC.ENCODER2 == true>
    
    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/  
    EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,${SPD_M2});
    pEAC[M2] = &EncAlignCtrlM2;  
  </#if>  
  
  <#if  MC.POSITION_CTRL_ENABLING2 == true>
    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/

  PID_HandleInit(&PID_PosParamsM2);
  
  TC_Init(&PosCtrlM2, &PID_PosParamsM2, &SpeednTorqCtrlM2, ${SPD_M2});  
  </#if> 
  
    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M2], &PIDSpeedHandle_M2, ${SPD_M2}._Super);
   <#if AUX_SPEED_FDBK_M2 >
   
    /***********************************************************/
    /*   Auxiliary speed sensor component initialization       */
    /***********************************************************/ 
    ${SPD_aux_init_M2} (${SPD_AUX_M2});
   <#if  MC.AUX_ENCODER2 == true>
  
    /***********************************************************/
    /*   Auxiliary encoder alignment component initialization  */
    /***********************************************************/ 
    EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,${SPD_AUX_M2});
    pEAC[M2] = &EncAlignCtrlM2;  
   </#if>
   </#if>  
  <#if   MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true ||  MC.ENCODER2 == true ||  MC.AUX_ENCODER2 == true > 
    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/ 
    VSS_Init(&VirtualSpeedSensorM2);                              
  </#if>
  <#if   MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true>   
    
    /****************************************************/
    /*   Rev-up component initialization                */
    /****************************************************/ 
    RUC_Init(&RevUpControlM2, pSTC[M2], &VirtualSpeedSensorM2, &STO_M2, pwmcHandle[M2]);        /* only if sensorless*/
  </#if>
  
    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M2);
    PID_HandleInit(&PIDIdHandle_M2);
  
  <#if   MC.BUS_VOLTAGE_READING2 == true>
    
    /**********************************************************/
    /*   Bus voltage sensor component initialization          */
    /**********************************************************/
    RVBS_Init(&BusVoltageSensor_M2);
  <#else>
  
    /**********************************************************/
    /*   Virtual bus voltage sensor component initialization  */
    /**********************************************************/
    VVBS_Init(&BusVoltageSensor_M2);
  </#if>
  
    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/    
    pMPM[M2]->pVBS = &(BusVoltageSensor_M2._Super);
    pMPM[M2]->pFOCVars = &FOCVars[M2];
  <#if   MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
    
    pR_Brake[M2] = &R_BrakeParamsM2;
    DOUT_SetOutputState(pR_Brake[M2],INACTIVE);
  </#if>
  
    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    NTC_Init(&TempSensor_M2);  
  
  <#if  MC.FLUX_WEAKENING_ENABLING2 == true>
  
    /*************************************************/
    /*   Flux weakening component initialization     */
    /*************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M2);
    FW_Init(pFW[M2], &PIDSpeedHandle_M2, &PIDFluxWeakeningHandle_M2);    /* only if M2 has FW */
  </#if>
  <#if  MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
  
    /*************************************************/
    /*   Feed forward component initialization       */
    /*************************************************/
    FF_Init(pFF[M2], &(BusVoltageSensor_M2._Super), pPIDId[M2], pPIDIq[M2]);    /* only if M2 has FF */
  </#if>
  <#if  MC.M2_DBG_OPEN_LOOP_ENABLE == true>
    
    OL_Init(&OpenLoop_ParamsM2, &VirtualSpeedSensorM2._Super);     /* only if M2 has open loop */
    pOpenLoop[M2] = &OpenLoop_ParamsM2;
  </#if>
  
    pREMNG[M2] = &RampExtMngrHFParamsM2;
    REMNG_Init(pREMNG[M2]);
    FOC_Clear(M2);
    FOCVars[M2].bDriveInput = EXTERNAL;
    FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);
    FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).d;
  <#if MC.POSITION_CTRL_ENABLING == true || MC.POSITION_CTRL_ENABLING2 == true>
  <#if  MC.POSITION_CTRL_ENABLING2 == true>  
    MCI_Init(&Mci[M2], pSTC[M2], &FOCVars[M2], pPosCtrl[M2]);
  <#else>
    MCI_Init(&Mci[M2], pSTC[M2], &FOCVars[M2], MC_NULL);
  </#if>
  <#else>
    MCI_Init(&Mci[M2], pSTC[M2], &FOCVars[M2]);
  </#if>  
<#if MC.DEFAULT_CONTROL_MODE2 == 'STC_SPEED_MODE'>
    MCI_ExecSpeedRamp(&Mci[M2],
    STC_GetMecSpeedRefUnitDefault(pSTC[M2]),0); /*First command to STC*/
<#else>   
    MCI_ExecTorqueRamp(&Mci[M2], STC_GetDefaultIqdref(pSTC[M2]).q, 0);
</#if>    
    pMCIList[M2] = &Mci[M2];
  </#if> <#--  end of dualdrive -->
  
  <#if  MC.M1_ICL_ENABLED == true>
    ICL_Init(&ICL_M1, &(BusVoltageSensor_M1._Super), &ICLDOUTParamsM1);
    Mci[M1].State = ICLWAIT;
  </#if>
  <#if  MC.M2_ICL_ENABLED == true>
    ICL_Init(&ICL_M2, &(BusVoltageSensor_M2._Super), &ICLDOUTParamsM2);
    Mci[M2].State = ICLWAIT;
  </#if>
  
  <#if MC.PFC_ENABLED == true>
    /* Initializing the PFC component */
    PFC_Init(&PFC);
  </#if> <#-- end of if MC.PFC_ENABLED == true -->
  
  <#if MC.DAC_FUNCTIONALITY>
    DAC_Init(&DAC_Handle);
  </#if>
  <#if MC.STSPIN32G4 == true>

    /*************************************************/
    /*   STSPIN32G4 driver component initialization  */
    /*************************************************/
    STSPIN32G4_init( &HdlSTSPING4 );
    STSPIN32G4_reset( &HdlSTSPING4 );
    STSPIN32G4_setVCC( &HdlSTSPING4, (STSPIN32G4_confVCC){ .voltage = _12V, 
                                                           .useNFAULT = true,
                                                           .useREADY = false } );
    STSPIN32G4_setVDSP( &HdlSTSPING4, (STSPIN32G4_confVDSP){ .deglitchTime = _4us, 
                                                             .useNFAULT = true } );
    STSPIN32G4_clearFaults( &HdlSTSPING4 );
  </#if><#-- MC.STSPIN32G4 == true -->

    /* Applicative hook in MCBoot() */
    MC_APP_BootHook();

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
  
    bMCBootCompleted = 1U;
  }
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();
<#if MC.RTOS == "NONE">    

    /* Safety task is run after Medium Frequency task so that  
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
    
</#if>
<#if MC.PFC_ENABLED == true>
    /* ** Power Factor Correction Task ** */ 
    PFC_Scheduler();
</#if>
  }
}

/**
 * @brief Performs stop process and update the state machine.This function 
 *        shall be called only during medium frequency task
 */
void TSK_MF_StopProcessing(  MCI_Handle_t * pHandle, uint8_t motor)
{
  ${PWM_SwitchOff}(pwmcHandle[motor]);
 
<#if MC.MOTOR_PROFILER == true && MC.DUALDRIVE == false>
  SCC_Stop(&SCC);
  OTT_Stop(&OTT); 
</#if>
<#if MC.DRIVE_TYPE == "FOC">
  FOC_Clear(motor);
  PQD_Clear(pMPM[motor]);
</#if>
<#if MC.DISCONTINUOUS_PWM == true  || MC.DISCONTINUOUS_PWM2 == true> 		  
  /* Disable DPWM mode */
  PWMC_DPWM_ModeDisable( pwmcHandle[motor] );
</#if>	
<#if MC.DRIVE_TYPE == "SIX_STEP">
  SixStep_Clear(motor);
</#if>
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;  
  return;
}


/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance. 
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (((uint8_t)1) == bMCBootCompleted)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();

      /* Applicative hook at end of Medium Frequency for Motor 1 */
      MC_APP_PostMediumFrequencyHook_M1();

<#if MC.MCP_OVER_UART_A >
      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer, 
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer, 
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC)) 
        {
          /* no buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer, 
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* no buffer available to build the answer ... should not occur */
        }
      }
</#if>
<#if MC.MCP_OVER_UART_B >
      MCP_Over_UartB.rxBuffer = MCP_Over_UartB.pTransportLayer->fRXPacketProcess(MCP_Over_UartB.pTransportLayer,
                                                                                 &MCP_Over_UartB.rxLength);
      if (MCP_Over_UartB.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_UartB.pTransportLayer->fGetBuffer(MCP_Over_UartB.pTransportLayer,
                                                       (void **) &MCP_Over_UartB.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_UartB);
          MCP_Over_UartB.pTransportLayer->fSendPacket(MCP_Over_UartB.pTransportLayer, MCP_Over_UartB.txBuffer,
                                                      MCP_Over_UartB.txLength, MCTL_SYNC);
        }
        else 
        {
          /* no buffer available to build the answer ... should not occur */
        }
      }
</#if>      
<#if MC.MCP_OVER_STLNK >
      MCP_Over_STLNK.rxBuffer = MCP_Over_STLNK.pTransportLayer->fRXPacketProcess( MCP_Over_STLNK.pTransportLayer,
                                                                                  &MCP_Over_STLNK.rxLength);
      if (0U == MCP_Over_STLNK.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_STLNK.pTransportLayer->fGetBuffer(MCP_Over_STLNK.pTransportLayer,
                                      (void **) &MCP_Over_STLNK.txBuffer, MCTL_SYNC)) //cstat !MISRAC2012-Rule-11.3
        {
          /* no buffer available to build the answer ... should not occur */
        }
        else 
        {
          MCP_ReceivedPacket(&MCP_Over_STLNK);
          MCP_Over_STLNK.pTransportLayer->fSendPacket (MCP_Over_STLNK.pTransportLayer, MCP_Over_STLNK.txBuffer,
                                                       MCP_Over_STLNK.txLength, MCTL_SYNC);
        }
      }
</#if>
      
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
<#if  MC.EXAMPLE_SPEEDMONITOR == true>
      /****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/
      ARR_TIM5_update(${SPD_M1});

      /**************************************************************************/
</#if>
      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
<#if  MC.DUALDRIVE == true>
    if(hMFTaskCounterM2 > ((uint16_t )0))
    {
      hMFTaskCounterM2--;
    }
    else
    {
      TSK_MediumFrequencyTaskM2();
      /* USER CODE BEGIN MC_Scheduler MediumFrequencyTask M2 */

      /* USER CODE END MC_Scheduler MediumFrequencyTask M2 */
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
</#if>
    if(hBootCapDelayCounterM1 > 0U)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
<#if  MC.DUALDRIVE == true>
    if(hBootCapDelayCounterM2 > 0U)
    {
      hBootCapDelayCounterM2--;
    }
    if(hStopPermanencyCounterM2 > 0U)
    {
      hStopPermanencyCounterM2--;
    }
</#if>
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_BG_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
</#if>
</#if>
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[M1] );  
</#if>
<#if  MC.M1_ICL_ENABLED == true>
  ICL_State_t ICLstate = ICL_Exec(&ICL_M1);
</#if>
<#if  AUX_SPEED_FDBK_M1 == true >
  (void)${SPD_aux_calcAvrgMecSpeedUnit_M1}(${SPD_AUX_M1}, &wAux);
</#if>
<#if  MC.SPEED_FEEDBACK_CHECK == true || MC.HALL_SENSORS == true >
  bool IsSpeedReliable = ${SPD_calcAvrgMecSpeedUnit_M1}(${SPD_M1}, &wAux);
<#else>
  (void)${SPD_calcAvrgMecSpeedUnit_M1}(${SPD_M1}, &wAux);
</#if>   
<#if MC.DRIVE_TYPE == "FOC">
  PQD_CalcElMotorPower(pMPM[M1]);
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if (MC.M1_ICL_ENABLED == true)>
  if ( !ICLFaultTreatedM1 && (ICLstate == ICL_ACTIVE)){
    ICLFaultTreatedM1 = true; 
  }
</#if>

<#if (MC.M1_ICL_ENABLED == true)>
  if ((MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS) && ICLFaultTreatedM1)
<#else>
  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
</#if>
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {  
      switch (Mci[M1].State)
      {
<#if (MC.M1_ICL_ENABLED == true)>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M1].State = IDLE; 
          }
          break;
        }
        
</#if>
        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
<#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC"> 
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE) 
</#if>            
            {     
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
<#if MC.DRIVE_TYPE == "SIX_STEP">
              RUC_UpdatePulse(&RevUpControlM1, &BusVoltageSensor_M1._Super);
</#if>
            }	    
</#if>  <#-- MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true   -->
<#if MC.DRIVE_TYPE == "FOC">
           if (pwmcHandle[M1]->offsetCalibStatus == false)
           {
             PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
             Mci[M1].State = OFFSET_CALIB;
           }
           else
           {
             /* calibration already done. Enables only TIM channels */
             pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;    
             PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
</#if>			 
<#if ( CHARGE_BOOT_CAP_ENABLING == true)>             
             ${PWM_TurnOnLowSides}(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
             TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
             Mci[M1].State = CHARGE_BOOT_CAP; 
<#else>     
<#-- test sensorless -->     
  <#if MC.OTF_STARTUP == true >
                  FOCVars[M1].bDriveInput = EXTERNAL;
                  STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
                  ${SPD_clear_M1}(${SPD_M1});
                  FOC_Clear( M1 );
                  ${PWM_SwitchOn}(pwmcHandle[M1]);          
    </#if>   <#--  MC.OTF_STARTUP == true    --> 
                  Mci[M1].State = START;    
</#if>   <#-- ( CHARGE_BOOT_CAP_ENABLING == true)    -->   
<#if MC.DRIVE_TYPE == "FOC">                                         
           }
</#if>           
<#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>
            OTT_Clear(&OTT);
</#if>  <#-- (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)    -->         
          }
          else
          {
            /* nothing to be done, FW stays in IDLE state */
          }
          break;
        }
		
<#if MC.DRIVE_TYPE == "FOC">
        case OFFSET_CALIB:
          {
            if (MCI_STOP == Mci[M1].DirectCommand)
            {
              TSK_MF_StopProcessing(&Mci[M1], M1);
            }
            else
            {      
              if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
              {
                if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
                {
                  FOC_Clear(M1);
                  PQD_Clear(pMPM[M1]);
                  Mci[M1].DirectCommand = MCI_NO_COMMAND;
                  Mci[M1].State = IDLE;
                }
                else
                {
<#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>    
                  Mci[M1].State = WAIT_STOP_MOTOR;
<#else>
  <#if ( CHARGE_BOOT_CAP_ENABLING == true)>
                  ${PWM_TurnOnLowSides}(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                  TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
                  Mci[M1].State = CHARGE_BOOT_CAP;
  <#else>
    <#if MC.OTF_STARTUP == true >
                  FOCVars[M1].bDriveInput = EXTERNAL;
                  STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
                  ${SPD_clear_M1}(${SPD_M1});
                  FOC_Clear( M1 );
<#if MC.DISCONTINUOUS_PWM == true > 		  
                  /* Enable DPWM mode before Start */
                  PWMC_DPWM_ModeEnable( pwmcHandle[M1] );
</#if>                   
                  ${PWM_SwitchOn}(pwmcHandle[M1]);          
    </#if>   <#--  MC.OTF_STARTUP == true    --> 
                  Mci[M1].State = START;    
  </#if>   <#-- ( CHARGE_BOOT_CAP_ENABLING == true)    -->                
</#if>   <#-- (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)    -->        
                }
              }
              else
              {
                /* nothing to be done, FW waits for offset calibration to finish */
              }            
            }  
            break;
          }  
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if ( CHARGE_BOOT_CAP_ENABLING == true)>
  
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              ${PWM_SwitchOff}(pwmcHandle[M1]);
<#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.ENCODER == true ||  MC.AUX_ENCODER == true>          
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
</#if>
<#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
              SixStepVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
</#if>  
              ${SPD_clear_M1}(${SPD_M1});
<#if  AUX_SPEED_FDBK_M1 == true >
              ${SPD_aux_clear_M1}(${SPD_AUX_M1});
</#if>           
<#if MC.OVERMODULATION == true>      
              PWMC_Clear(pwmcHandle[M1]);
</#if>
<#if MC.DISCONTINUOUS_PWM == true > 		  
              /* Enable DPWM mode before Start */
              PWMC_DPWM_ModeEnable( pwmcHandle[M1] );
</#if> 
<#if MC.DRIVE_TYPE == "FOC">  
              FOC_Clear( M1 );
</#if>
<#if MC.DRIVE_TYPE == "SIX_STEP">
              SixStep_Clear( M1 );
</#if>
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
              BADC_SetDirection(&Bemf_ADC_M1, MCI_GetImposedMotorDirection( &Mci[M1]));
              BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1._Super, pSTC[M1] );
</#if>		  
<#if (MC.MOTOR_PROFILER == true)>
        SCC_Start(&SCC);
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked. */
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
<#else>              
 
<#if  MC.ENCODER == true >
              if (EAC_IsAligned(&EncAlignCtrlM1) == false )
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef( M1 );
                STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands( &Mci[M1] ); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;                
              }
<#elseif  MC.AUX_ENCODER == true>               
              if (EAC_IsAligned(&EncAlignCtrlM1) == false )
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {

                VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
                FOC_Clear(M1);
                Mci[M1].State = START;
              }
<#elseif MC.HALL_SENSORS == true>      
<#if MC.DRIVE_TYPE == "FOC">
              FOC_InitAdditionalMethods(M1);
              FOC_CalcCurrRef(M1);
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
              SixStep_InitAdditionalMethods(M1);
              SixStep_CalcSpeedRef( M1 );
  <#if MC.CURRENT_LIMITER_OFFSET == true>			  
              #if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
              #else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
              #endif
  </#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
              STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands( &Mci[M1] ); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M1].State = RUN;
<#else>    <#-- sensorless mode only -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
              if ( mode == MCM_OPEN_LOOP_VOLTAGE_MODE || mode == MCM_OPEN_LOOP_CURRENT_MODE)
              {
                Mci[M1].State = RUN;
              }
              else
</#if>              
              {
                
                Mci[M1].State = START;
              } 
</#if>   <#--  MC.ENCODER == true   --> 
</#if>   <#-- (MC.MOTOR_PROFILER == true)   -->        
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
</#if> <#-- ( CHARGE_BOOT_CAP_ENABLING == true)    -->    

<#if  MC.ENCODER == true ||  MC.AUX_ENCODER == true > <#-- only for encoder -->
        case ALIGNMENT:
        {  
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {  
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM1);
            bool EACDone = EAC_Exec(&EncAlignCtrlM1);
            if ((isAligned == false)  && (EACDone == false))
            {
                qd_t IqdRef;
                IqdRef.q = 0;
                IqdRef.d = STC_CalcTorqueReference(pSTC[M1]);
                FOCVars[M1].Iqdref = IqdRef;
            }
            else
            {
              ${PWM_SwitchOff}( pwmcHandle[M1] );
<#if MC.AUX_ENCODER == false >              
              STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);                
</#if>   <#--   MC.AUX_ENCODER == false     -->             
              FOC_Clear(M1);
<#if MC.ENCODER == true || MC.AUX_ENCODER == true >                 
              TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
              Mci[M1].State = WAIT_STOP_MOTOR;
<#else>
              Mci[M1].State = START;
</#if>   <#-- MC.ENCODER == true     -->                
              /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */
            
              /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */
            }
          }
          break;  
        }
  
</#if> <#--  MC.ENCODER == true ||  MC.AUX_ENCODER == true     -->    
<#if ( MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC")>
        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {      
          <#-- only for sensor-less control -->
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
  <#if MC.DRIVE_TYPE == "FOC">
            qd_t IqdRef;
  </#if>		
            bool ObserverConverged = false;
        
            /* Execute the Rev Up procedure */
  <#if ( MC.OTF_STARTUP == true )>
            if (! RUC_OTF_Exec(&RevUpControlM1))
  <#else>  
            if(! RUC_Exec(&RevUpControlM1))
  </#if>  <#-- MC.OTF_STARTUP == true    -->
            {
            /* The time allowed for the startup sequence has expired */
  <#if ( MC.MOTOR_PROFILER == true)>
              /* However, no error is generated when OPEN LOOP is enabled 
               * since then the system does not try to close the loop... */
  <#else>
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
  </#if>  <#-- ((MC.M1_DBG_OPEN_LOOP_ENABLE == true) || (MC.MOTOR_PROFILER == true))    -->
           }
           else
           {
             /* Execute the torque open loop current start-up ramp:
              * Compute the Iq reference current as configured in the Rev Up sequence */
  <#if MC.DRIVE_TYPE == "FOC">
             IqdRef.q = STC_CalcTorqueReference( pSTC[M1] );
             IqdRef.d = FOCVars[M1].UserIdref;
             /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
             FOCVars[M1].Iqdref = IqdRef;
  </#if>
  <#if MC.DRIVE_TYPE == "SIX_STEP">
            (void) BADC_CalcRevUpDemagTime (&Bemf_ADC_M1, pSTC[M1] );
             PWMC_ForceFastDemagTime (pwmcHandle[M1], Bemf_ADC_M1.DemagCounterThreshold);		 
             SixStepVars[M1].DutyCycleRef = STC_CalcSpeedReference( pSTC[M1] );
  </#if>
           }
          
           (void) VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
         
  <#if ( MC.OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false)>
           /* check that startup stage where the observer has to be used has been reached */
           if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
  </#if>  <#-- ( MC.OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false)    -->
            {
  <#if  MC.STATE_OBSERVER_PLL == true>
             ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M1, &hForcedMecSpeedUnit);
             STO_SetDirection(&STO_PLL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
  <#elseif (( MC.STATE_OBSERVER_CORDIC == true))>
             ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M1, hForcedMecSpeedUnit);
  <#elseif MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
             BADC_SpeedMeasureOn(&Bemf_ADC_M1);
             ObserverConverged = BADC_IsObserverConverged( &Bemf_ADC_M1);
  </#if> <#-- MC.STATE_OBSERVER_PLL == true     -->
              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
            }
           
            if (ObserverConverged)
            {
  <#if MC.DRIVE_TYPE == "FOC">
              qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(${SPD_M1}._Super));
           
              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC. */
              REMNG_Init(pREMNG[M1]);
              (void)REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);
  </#if>
              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {      
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;
          
  <#if MC.MOTOR_PROFILER == false> <#-- No need to call RUC_Exec() when in MP in this state -->
    <#if (( MC.OTF_STARTUP == true))>
            if (! RUC_OTF_Exec(&RevUpControlM1))
    <#else>  
            if(! RUC_Exec(&RevUpControlM1))
    </#if> <#--  MC.OTF_STARTUP     -->
            {
          <#if ((MC.MOTOR_PROFILER == true))>
              /* No error is generated when OPEN LOOP is enabled. */
    <#else>
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);  
          </#if>  <#--    (MC.MOTOR_PROFILER == true)    -->
            } 
            else
  </#if> <#--  MC.MOTOR_PROFILER == false     -->
            { 
              /* Compute the virtual speed and positions of the rotor. 
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM1);
              LoopClosed = LoopClosed || tempBool; 
              
              /* If any of the above conditions is true, the loop is considered closed. 
                 The state machine transitions to the START_RUN state. */
              if (true ==  LoopClosed) 
              {
                #if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )  
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
                #else
  <#if MC.DRIVE_TYPE == "FOC">
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1)) 
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
  </#if>
  <#if MC.DRIVE_TYPE == "SIX_STEP">
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1)) 
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
  </#if>
				#endif
  <#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>
                OTT_SR(&OTT);
  </#if>  <#--  (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)     -->
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */
            
                /* USER CODE END MediumFrequencyTask M1 1 */ 
                STC_SetSpeedSensor(pSTC[M1], ${SPD_M1}._Super); /*Observer has converged*/     
  <#if MC.DRIVE_TYPE == "FOC">
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef( M1 );
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.DRIVE_TYPE == "SIX_STEP">
                SixStep_InitAdditionalMethods(M1);
                SixStep_CalcSpeedRef( M1 );
    <#if MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">				
                BADC_SetLoopClosed(&Bemf_ADC_M1);
    </#if>
  </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */            
                Mci[M1].State = RUN;
              }  
            }
          }
          break;
        }
    
</#if> <#--  MC.ENCODER == true ||  MC.AUX_ENCODER == true     -->
        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {      
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */
            
            /* USER CODE END MediumFrequencyTask M1 2 */
       
  <#if  MC.POSITION_CTRL_ENABLING == true >
            TC_PositionRegulation(pPosCtrl[M1]);
  </#if> <#--  MC.POSITION_CTRL_ENABLING == true     -->
            MCI_ExecBufferedCommands(&Mci[M1]);
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
            {                     
    </#if>  <#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true     --> 
  <#if MC.DRIVE_TYPE == "FOC">
              FOC_CalcCurrRef(M1);
  </#if><#-- MC.DRIVE_TYPE == "FOC" -->
  <#if MC.DRIVE_TYPE == "SIX_STEP">
            SixStep_CalcSpeedRef( M1 );
  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
            BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1._Super, pSTC[M1] );
            (void) BADC_CalcRunDemagTime (&Bemf_ADC_M1, pSTC[M1] );
            PWMC_ForceFastDemagTime (pwmcHandle[M1], Bemf_ADC_M1.DemagCounterThreshold);		
  </#if>	
  </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
         
<#if  MC.SPEED_FEEDBACK_CHECK == true || MC.HALL_SENSORS == true >
              if(!IsSpeedReliable)
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
</#if>	<#--  MC.SPEED_FEEDBACK_CHECK == true || MC.HALL_SENSORS == true     -->
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              int16_t hForcedMecSpeedUnit;
              /* Open Loop */
              VSS_CalcAvrgMecSpeedUnit( &VirtualSpeedSensorM1, &hForcedMecSpeedUnit );
     	      OL_Calc( pOpenLoop[M1] );
            }   
  </#if> 
<#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>
            OTT_MF(&OTT);
</#if> <#--  (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)    -->
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
  
<#if ( MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC")>
            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);  	/*  sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
  <#if MC.DRIVE_TYPE == "SIX_STEP">
            BADC_Clear(&Bemf_ADC_M1);
  </#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->			
</#if>   <#--   ( MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true)   -->
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */
    
            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;             
          }
          else
          {
            /* nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;             
          }
          else
          {
            /* nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
        }
        break;
        
        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
        }      
        break;
        
<#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || MC.ENCODER == true || MC.AUX_ENCODER == true)>    
        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {        
<#if (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)>         
            if (0 == SCC_DetectBemf(&SCC))
            {
              /* In a sensorless configuration. Initiate the Revup procedure */
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
               ${SPD_clear_M1}( ${SPD_M1} );   
              FOC_Clear( M1 );
              SCC_Start( &SCC );
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked. */
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
            }
            else
            {
              /* nothing to do */
            }
<#elseif MC.ENCODER == true>
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {          
              ENC_Clear(&ENCODER_M1);
              ${PWM_SwitchOn}( pwmcHandle[M1] );
<#if  MC.POSITION_CTRL_ENABLING == true >
              TC_EncAlignmentCommand(pPosCtrl[M1]);
</#if> <#--  MC.POSITION_CTRL_ENABLING == true     -->                
              FOC_InitAdditionalMethods(M1);
              STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands( &Mci[M1] ); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M1);               
              Mci[M1].State = RUN;
            } 
            else
            {
              /* nothing to do */
            }
<#elseif MC.AUX_ENCODER == true>
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {                        
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));              
              ${SPD_clear_M1}( ${SPD_M1} );   
              ENC_Clear(&ENCODER_M1);
              VSS_Clear(&VirtualSpeedSensorM1);
              ${PWM_SwitchOn}( pwmcHandle[M1] );
              Mci[M1].State = START;
            } 
            else
            {
              /* nothing to do */
            }
</#if>  <#--   (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true)  -->
          }
        }
        break;
</#if>  <#--   (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || MC.ENCODER == true)  -->
        default:
          break;
       }
    }  
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
<#if  MC.MOTOR_PROFILER>
   <#if MC.AUX_HALL_SENSORS>
  HT_MF (&HT);
   </#if>
  SCC_MF(&SCC);
</#if> <#-- (MC.MOTOR_PROFILER == true)  -->
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_BG_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
</#if>
</#if>
}

<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true> 
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[bMotor] );  
</#if>
    
  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};
  
  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>   
  if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
</#if>   
  {
    FOCVars[bMotor].Iqdref = NULL_qd;
  }
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

<#if ( MC.FLUX_WEAKENING_ENABLING == true) || (MC.FLUX_WEAKENING_ENABLING2 == true)>
  if (NULL == pFW[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FW_Clear(pFW[bMotor]);
  }
</#if>
<#if ( MC.FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true)>
  if (NULL == pFF[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FF_Clear(pFF[bMotor]);
  }
</#if>

<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Clear(&PerfTraces);  
</#if>
</#if>
  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
<#if ( MC.FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true)>
      if (NULL == pFF[bMotor])
      {
        /* Nothing to do */
      }
      else
      {
        FF_InitFOCAdditionalMethods(pFF[bMotor]);
      }
</#if>
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
<#if (( MC.MTPA_ENABLING == false) &&  (MC.FLUX_WEAKENING_ENABLING == true)) || 
     (( MC.MTPA_ENABLING2 == false) &&  (MC.FLUX_WEAKENING_ENABLING2 == true)) >
  qd_t IqdTmp;
</#if>
    
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>   
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[bMotor] );
  if (INTERNAL == FOCVars[bMotor].bDriveInput &&  (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE) )
<#else>
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
</#if>  
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;
<#if ( MC.MTPA_ENABLING == true) ||  (MC.MTPA_ENABLING2 == true)>
    if (0 == pMaxTorquePerAmpere[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      MTPA_CalcCurrRefFromIq(pMaxTorquePerAmpere[bMotor], &FOCVars[bMotor].Iqdref);
    }
</#if>

<#if ( MC.FLUX_WEAKENING_ENABLING == true) || ( MC.FLUX_WEAKENING_ENABLING2 == true)>
    if (NULL == pFW[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
<#if MC.DUALDRIVE == false >
<#if ( MC.MTPA_ENABLING == true)>
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], FOCVars[bMotor].Iqdref);
<#else>
      IqdTmp.q = FOCVars[bMotor].Iqdref.q;
      IqdTmp.d = FOCVars[bMotor].UserIdref; 
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
</#if>
<#else>
<#if ( MC.MTPA_ENABLING == true) &&  (MC.MTPA_ENABLING2 == true)>
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], FOCVars[bMotor].Iqdref);
<#elseif ( MC.MTPA_ENABLING == false) && (MC.MTPA_ENABLING2 == false)>
      IqdTmp.q = FOCVars[bMotor].Iqdref.q;
      IqdTmp.d = FOCVars[bMotor].UserIdref; 
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], IqdTmp); 
<#else>
      if (0 == pMaxTorquePerAmpere[bMotor])
      {
        IqdTmp.q = FOCVars[bMotor].Iqdref.q;
        IqdTmp.d = FOCVars[bMotor].UserIdref; 
        FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      }
      else
      {
        FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor], FOCVars[bMotor].Iqdref);
      }     
</#if>     
</#if>       
    }
</#if>
<#if ( MC.FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true)>
    if (NULL == pFF[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      FF_VqdffComputation(pFF[bMotor], FOCVars[bMotor].Iqdref, pSTC[bMotor]);
    }
</#if>
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->

<#if MC.DRIVE_TYPE == "SIX_STEP">
/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void SixStep_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN SixStep_Clear 0 */

  /* USER CODE END SixStep_Clear 0 */

  STC_Clear(pSTC[bMotor]);
  SixStepVars[bMotor].DutyCycleRef = STC_GetDutyCycleRef(pSTC[bMotor]);
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
  BADC_SpeedMeasureOff(&Bemf_ADC_M1);
  BADC_Stop( &Bemf_ADC_M1 );
  BADC_Clear( &Bemf_ADC_M1 );
</#if>
<#if  MC.CURRENT_LIMITER_OFFSET == true >
  #if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )
    PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
  #else
    PID_SetIntegralTerm(&PIDSpeedHandle_M1,
         (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
            / PID_SPEED_INTEGRAL_INIT_DIV));
  #endif
</#if>			  
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN SixStep_Clear 1 */

  /* USER CODE END SixStep_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void SixStep_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void SixStep_CalcSpeedRef(uint8_t bMotor)
{

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(SixStepVars[bMotor].bDriveInput == INTERNAL)
  {
    SixStepVars[bMotor].DutyCycleRef = STC_CalcSpeedReference(pSTC[bMotor]);
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

<#if  MC.DUALDRIVE == true>
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 2 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM2(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M2 0 */

  /* USER CODE END MediumFrequencyTask M2 0 */

  int16_t wAux = 0;

<#if  MC.M2_ICL_ENABLED == true>
  ICL_State_t ICLstate = ICL_Exec(&ICL_M2);
</#if>
<#if  AUX_SPEED_FDBK_M2 == true >
  (void)${SPD_aux_calcAvrgMecSpeedUnit_M2}(${SPD_AUX_M2}, &wAux);
</#if>
<#if  MC.SPEED_FEEDBACK_CHECK2 == true || MC.HALL_SENSORS2 == true >
  bool IsSpeedReliable = ${SPD_calcAvrgMecSpeedUnit_M2}(${SPD_M2}, &wAux);
<#else>
  (void)${SPD_calcAvrgMecSpeedUnit_M2}(${SPD_M2}, &wAux);
</#if>   
  PQD_CalcElMotorPower(pMPM[M2]);

<#if (MC.M2_ICL_ENABLED == true)>
  if ( !ICLFaultTreatedM2 && (ICLstate == ICL_ACTIVE)){
    ICLFaultTreatedM2 = true; 
  }
</#if>

<#if (MC.M2_ICL_ENABLED == true)>
  if ((MCI_GetCurrentFaults(&Mci[M2]) == MC_NO_FAULTS) && ICLFaultTreatedM2)
<#else>
  if (MCI_GetCurrentFaults(&Mci[M2]) == MC_NO_FAULTS)
</#if>
  {
    if (MCI_GetOccurredFaults(&Mci[M2]) == MC_NO_FAULTS)
    {  
      switch (Mci[M2].State)
      {
<#if (MC.M2_ICL_ENABLED == true)>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M2].State = IDLE; 
          }
          break;
        }
        
</#if>
        case IDLE:
        {
          if ((MCI_START == Mci[M2].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand))
          {          
<#if MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true>          
            RUC_Clear(&RevUpControlM2, MCI_GetImposedMotorDirection(&Mci[M2]));	    
</#if>  <#-- MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true   -->
           if (pwmcHandle[M2]->offsetCalibStatus == false)
           {
             PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_START);
             Mci[M2].State = OFFSET_CALIB;
           }
           else
           {
             /* calibration already done. Enables only TIM channels */
             pwmcHandle[M2]->OffCalibrWaitTimeCounter = 1u;    
             PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC);
<#if ( CHARGE_BOOT_CAP_ENABLING2 == true)>             
             ${PWM_TurnOnLowSides_M2}(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
             TSK_SetChargeBootCapDelayM2(CHARGE_BOOT_CAP_TICKS2);
             Mci[M2].State = CHARGE_BOOT_CAP; 
<#else>     
<#-- test sensorless -->     
  <#if MC.OTF_STARTUP2 == true >
            FOCVars[M2].bDriveInput = EXTERNAL;
            STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);
            ${SPD_clear_M2}(${SPD_M2});
            FOC_Clear(M2);
            ${PWM_SwitchOn}(pwmcHandle[M2]);          
    </#if>   <#--  MC.OTF_STARTUP2 == true    --> 
            Mci[M2].State = START;    
</#if>   <#-- ( CHARGE_BOOT_CAP_ENABLING2 == true)    -->                                            
           }                  
          }
          else
          {
            /* nothing to be done, FW stays in IDLE state */
          }
          break;
        }
    
        case OFFSET_CALIB:
          {
            if (MCI_STOP == Mci[M2].DirectCommand)
            {
              TSK_MF_StopProcessing(&Mci[M2], M2);
            }
            else
            {      
              if (PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC))
              {
                if (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand)
                {
                  FOC_Clear(M2);
                  PQD_Clear(pMPM[M2]);
                  Mci[M2].DirectCommand = MCI_NO_COMMAND;
                  Mci[M2].State = IDLE;
                }
                else
                {              
<#if ( CHARGE_BOOT_CAP_ENABLING2 == true)>
                  ${PWM_TurnOnLowSides_M2}( pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
                  TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
                  Mci[M2].State = CHARGE_BOOT_CAP;
<#else>
    <#if MC.OTF_STARTUP2 == true >
                  FOCVars[M2].bDriveInput = EXTERNAL;
                  STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
                  ${SPD_clear_M2}( ${SPD_M2} );
                  FOC_Clear( M2 );
<#if MC.DISCONTINUOUS_PWM2 == true > 		  
                  /* Enable DPWM mode before Start */
                  PWMC_DPWM_ModeEnable( pwmcHandle[M2] );
</#if> 
                  ${PWM_SwitchOn_M2}( pwmcHandle[M2] );          
    </#if>   <#--  MC.OTF_STARTUP2 == true    --> 
                  Mci[M2].State = START; 
</#if>    <#-- ( CHARGE_BOOT_CAP_ENABLING2 == true) -->       
                } 
              }
              else
              {
                /* nothing to be done, FW waits for offset calibration to finish */
              }            
            }  
            break;
          } 
<#if ( CHARGE_BOOT_CAP_ENABLING2 == true)>
  
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM2())
            {
              ${PWM_SwitchOff_M2}( pwmcHandle[M2] );
<#if MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true || MC.ENCODER2 == true ||  MC.AUX_ENCODER2 == true>          
             FOCVars[M2].bDriveInput = EXTERNAL;
             STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);
</#if>             
              ${SPD_clear_M2}( ${SPD_M2} );
<#if  AUX_SPEED_FDBK_M2 == true >
              ${SPD_aux_clear_M2}( ${SPD_AUX_M2} );
</#if>           
<#if MC.OVERMODULATION2 == true>      
              PWMC_Clear(pwmcHandle[M2]);
</#if>  
<#if MC.DISCONTINUOUS_PWM2 == true > 		  
              /* Enable DPWM mode before Start */
              PWMC_DPWM_ModeEnable( pwmcHandle[M2] );
</#if> 
              FOC_Clear( M2 );           
<#if  MC.ENCODER2 == true >
              if (EAC_IsAligned(&EncAlignCtrlM2) == false )
              {
                EAC_StartAlignment(&EncAlignCtrlM2);
              Mci[M2].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M2], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M2], &ENCODER_M2._Super);
                FOC_InitAdditionalMethods(M2);
                FOC_CalcCurrRef(M2);
                STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M2] ); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M2].State = RUN;                
              }
<#elseif  MC.AUX_ENCODER2 == true>               
              if (EAC_IsAligned( &EncAlignCtrlM2) == false )
              {
                EAC_StartAlignment(&EncAlignCtrlM2);
                Mci[M2].State = ALIGNMENT;
              }
              else
              {

                VSS_Clear(&VirtualSpeedSensorM2); /* Reset measured speed in IDLE */
                FOC_Clear(M2);
                Mci[M2].State = START;
              }
<#elseif MC.HALL_SENSORS2 == true>      
              FOC_InitAdditionalMethods(M2);
              FOC_CalcCurrRef(M2);
              STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M2] ); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands( &Mci[M2] ); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M2].State = RUN;
<#else>     
              Mci[M2].State = START;
</#if> <#-- MC.ENCODER2 == true -->  
         
              PWMC_SwitchOnPWM(pwmcHandle[M2]);
            }
            else
            {
              /* nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
</#if> <#-- ( CHARGE_BOOT_CAP_ENABLING2 == true) -->       
<#if  MC.ENCODER2 == true ||  MC.AUX_ENCODER2 == true > <#-- only for encoder -->

        case ALIGNMENT:
        {  
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {  
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM2);
            bool EACDone = EAC_Exec(&EncAlignCtrlM2);
            if ((isAligned == false)  && (EACDone == false))
            {
                qd_t IqdRef;
                IqdRef.q = 0;
                IqdRef.d = STC_CalcTorqueReference(pSTC[M2]);
                FOCVars[M2].Iqdref = IqdRef;
            }
            else
            {
              ${PWM_SwitchOff_M2}( pwmcHandle[M2] );
<#if MC.AUX_ENCODER2 == false >              
              STC_SetControlMode(pSTC[M2], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M2], &ENCODER_M2._Super);   
</#if> <#-- MC.ENCODER2 == true -->             
<#if MC.ENCODER2 == true || MC.AUX_ENCODER2 == true>    
            
              TSK_SetStopPermanencyTimeM2(STOPPERMANENCY_TICKS);
              Mci[M2].State = WAIT_STOP_MOTOR;
</#if>  <#-- MC.ENCODER2 == true -->              
              /* USER CODE BEGIN MediumFrequencyTask M2 EndOfEncAlignment */
            
              /* USER CODE END MediumFrequencyTask M2 EndOfEncAlignment */
            }
          }
          break;  
        }
  
</#if> <#-- MC.ENCODER2 == true ||  MC.AUX_ENCODER2 == true  -->
<#if ( MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true)>
        case START:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {      
          <#-- only for sensor-less control -->
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged = false;
        
            /* Execute the Rev Up procedure */
      <#if ( MC.OTF_STARTUP2 == true )>
            if (! RUC_OTF_Exec(&RevUpControlM2))
      <#else>  
            if(! RUC_Exec(&RevUpControlM2))
      </#if>      
            {
            /* The time allowed for the startup sequence has expired */
        <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
              /* However, no error is generated when OPEN LOOP is enabled 
               * since then the system does not try to close the loop... */
        <#else>
              MCI_FaultProcessing( &Mci[M2], MC_START_UP, 0 );
        </#if>
           }
           else
           {
             /* Execute the torque open loop current start-up ramp:
              * Compute the Iq reference current as configured in the Rev Up sequence */
             IqdRef.q = STC_CalcTorqueReference( pSTC[M2] );
             IqdRef.d = FOCVars[M2].UserIdref;
             /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
             FOCVars[M2].Iqdref = IqdRef;
           }
          
           (void) VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
         
      <#if ( MC.M2_DBG_OPEN_LOOP_ENABLE == true)>
            /* Open Loop */
            {
              int16_t hOLFinalMecSpeedUnit = MCI_GetLastRampFinalSpeed(&Mci[M2]);
            
              if (hOLFinalMecSpeedUnit != VSS_GetLastRampFinalSpeed(&VirtualSpeedSensorM2))
              {
                VSS_SetMecAcceleration(&VirtualSpeedSensorM2, hOLFinalMecSpeedUnit, OPEN_LOOP_SPEED_RAMP_DURATION_MS2);
              }
              
              OL_Calc(pOpenLoop[M2]);
            }
          
      </#if>
      <#if MC.OTF_STARTUP2 == false>
           /* check that startup stage where the observer has to be used has been reached */
           if (true == RUC_FirstAccelerationStageReached(&RevUpControlM2))
      </#if>
            {
    <#if (( MC.STATE_OBSERVER_PLL2 == true))>
             ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M2, &hForcedMecSpeedUnit);
             STO_SetDirection(&STO_PLL_M2, (int8_t)MCI_GetImposedMotorDirection(&Mci[M2]));
	<#elseif (( MC.STATE_OBSERVER_CORDIC == true))>
             ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M2, hForcedMecSpeedUnit);
	</#if>
              (void)VSS_SetStartTransition(&VirtualSpeedSensorM2, ObserverConverged);
            }
           
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M2].Ialphabeta, SPD_GetElAngle(${SPD_M2}._Super));
              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC. */
              REMNG_Init(pREMNG[M2]);
              (void)REMNG_ExecRamp(pREMNG[M2], FOCVars[M2].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M2], StatorCurrent.q, TRANSITION_DURATION);
              Mci[M2].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {      
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;         

        <#if (( MC.OTF_STARTUP2 == true))>
            if (!RUC_OTF_Exec(&RevUpControlM2))
        <#else>  
            if(!RUC_Exec(&RevUpControlM2))
        </#if>      
            {
          <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
              /* No error is generated when OPEN LOOP is enabled. */
          <#else>
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M2], MC_START_UP, 0);  
          </#if>
            } 
            else
            { 
              /* Compute the virtual speed and positions of the rotor. 
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM2);
              LoopClosed = LoopClosed || tempBool; 
              
              /* If any of the above conditions is true, the loop is considered closed. 
                 The state machine transitions to the START_RUN state. */
              if (true ==  LoopClosed) 
              {
                #if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )  
                PID_SetIntegralTerm(&PIDSpeedHandle_M2, 0);
                #else
                PID_SetIntegralTerm(&PIDSpeedHandle_M2,
                                    (((int32_t)FOCVars[M2].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M2)) 
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
                #endif
                /* USER CODE BEGIN MediumFrequencyTask M2 1 */
            
                /* USER CODE END MediumFrequencyTask M2 1 */ 
                STC_SetSpeedSensor(pSTC[M2], ${SPD_M2}._Super); /*Observer has converged*/     
                FOC_InitAdditionalMethods(M2);
                FOC_CalcCurrRef(M2);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */            
                Mci[M2].State = RUN;
              }  
            }
          }
          break;
        }
    
</#if>  <#--  ( MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true) -->
        case RUN:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {      
            /* USER CODE BEGIN MediumFrequencyTask M2 2 */
            
            /* USER CODE END MediumFrequencyTask M2 2 */
       
  <#if  MC.POSITION_CTRL_ENABLING2 == true >
            TC_PositionRegulation(pPosCtrl[M2]);
  </#if>
            MCI_ExecBufferedCommands(&Mci[M2]);
            FOC_CalcCurrRef(M2);
         
<#if  MC.SPEED_FEEDBACK_CHECK2 == true || MC.HALL_SENSORS2 == true >
            if(!IsSpeedReliable)
            {
              MCI_FaultProcessing( &Mci[M2], MC_SPEED_FDBK, 0 );
            }
</#if>	
          }
          break;
        }
  
        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM2())
          {
  
<#if ( MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true)>
            STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);  	/*  sensor-less */
            VSS_Clear(&VirtualSpeedSensorM2); /* Reset measured speed in IDLE */
    
</#if>
            /* USER CODE BEGIN MediumFrequencyTask M2 5 */
    
            /* USER CODE END MediumFrequencyTask M2 5 */
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;             
          }
          else
          {
            /* nothing to do, FW waits for to stop */
          }
          break;
        }
  
        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M2].DirectCommand)
          {
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;             
          }
          else
          {
            /* nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
        }
        break;
        
        case FAULT_NOW:
        {
          Mci[M2].State = FAULT_OVER;
        }      
        
<#if (MC.ENCODER2 == true || MC.AUX_ENCODER2 == true)>    
        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M2], M2);
          }
          else
          {      
<#if MC.ENCODER2 == true>              
            if (TSK_StopPermanencyTimeHasElapsedM2())
            {           
              ENC_Clear(&ENCODER_M2);
              ${PWM_SwitchOn}( pwmcHandle[M2] );
<#if  MC.POSITION_CTRL_ENABLING2 == true >
              TC_EncAlignmentCommand(pPosCtrl[M2]);
</#if> <#--  MC.POSITION_CTRL_ENABLING2 == true     -->                
              FOC_InitAdditionalMethods(M2);
              STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M2] ); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands( &Mci[M2] ); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M2);               
              Mci[M2].State = RUN;
            } 
            else
            {
              /* nothing to do */
            }
          }
        }
        break;
<#elseif (MC.AUX_ENCODER2 == true)>  
            if (TSK_StopPermanencyTimeHasElapsedM2())
            {           
              FOC_Clear(M2);              
              RUC_Clear(&RevUpControlM2, MCI_GetImposedMotorDirection(&Mci[M2]));              
              ${SPD_clear_M2}( ${SPD_M2} );   
              ENC_Clear(&ENCODER_M2);
              VSS_Clear(&VirtualSpeedSensorM2);
              ${PWM_SwitchOn}( pwmcHandle[M2] );
              Mci[M2].State = START;
            } 
            else
            {
              /* nothing to do */
            }
          }
        }
        break;
</#if>        
</#if>  <#--   (MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || MC.ENCODER == true)  -->
        default:
          break;
       }
    }  
    else
    {
      Mci[M2].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M2].State = FAULT_NOW;
  }

  /* USER CODE BEGIN MediumFrequencyTask M2 6 */

  /* USER CODE END MediumFrequencyTask M2 6 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
   hBootCapDelayCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 2 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM2 == ((uint16_t )0))
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
  hStopPermanencyCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 2 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
  bool retVal = false;
  if (0U == hStopPermanencyCounterM2)
  {
    retVal = true;
  }
  return (retVal);
}
</#if> <#-- if  MC.DUALDRIVE == true -->

<#if MC.MOTOR_PROFILER == false>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{
<#if MC.DRIVE_TYPE == "FOC">
<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_HighFrequencyTask);
</#if>
</#if>
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  
  uint16_t hFOCreturn;
  uint8_t bMotorNbr = 0;

<#if MC.TESTENV == true && MC.PFC_ENABLED == false >
  /* Performance Measurement: start measure */
  start_perf_measure();
</#if>

<#if  MC.SINGLEDRIVE == true>
  <#if  MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true>
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main*/
  </#if>
  <#if  MC.AUX_STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true >
  Observer_Inputs_t STO_aux_Inputs; /*  only if sensorless aux*/
  STO_aux_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
  </#if>

  <#if (( MC.ENCODER == true) ||( MC.AUX_ENCODER == true))>
  (void)ENC_CalcAngle(&ENCODER_M1);   /* if not sensorless then 2nd parameter is MC_NULL*/
  </#if>
  <#if  ( MC.HALL_SENSORS == true) ||  MC.AUX_HALL_SENSORS == true>
  (void)HALL_CalcElAngle(&HALL_M1); 
  </#if>

  <#if ( MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true)>
  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
    }
  }
  </#if>
  <#if (( MC.OTF_STARTUP == true))>
  if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM1))
  {
    hFOCreturn = FOC_CurrControllerM1();
  }
  else
  {
    hFOCreturn = MC_NO_ERROR;
  }
  <#else>
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  </#if>
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
  <#if ( MC.STATE_OBSERVER_PLL == true)>
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1); 
  </#if> 	   
  <#if  MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true >
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /*  only for sensorless*/
    (void)${SPD_calcElAngle_M1}(${SPD_M1}, &STO_Inputs);
    ${SPD_calcAvergElSpeedDpp_M1}(${SPD_M1}); /*  Only in case of Sensor-less */
	  <#if ( MC.STATE_OBSERVER_PLL == true)>
	 if (false == IsAccelerationStageReached)
    {
      STO_ResetPLL(&STO_PLL_M1);
    }  
	  </#if> 	
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
    /*  only for sensor-less or open loop */
    if(((uint16_t)START == Mci[M1].State) || ((uint16_t)SWITCH_OVER == Mci[M1].State) || ((uint16_t)RUN == Mci[M1].State)) 
    <#else>
    /*  only for sensor-less */
    if(((uint16_t)START == Mci[M1].State) || ((uint16_t)SWITCH_OVER == Mci[M1].State))          
    </#if>
    {
      <#if  MC.SINGLE_SHUNT == true>
      if ((uint16_t)START == Mci[M1].State )
      {  
        if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 1);
        }
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }
      </#if>    
      int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super);      
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);  
    }
  <#else>
    <#if (MC.ENCODER == true || MC.HALL_SENSORS == true) && MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    if((uint16_t)RUN == Mci[M1].State)    
    { 
      int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super);     
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    </#if> 
  </#if>    
  <#if  MC.AUX_STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true>
    STO_aux_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/    
    STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /*  only for sensorless*/ 
    (void)${SPD_aux_calcElAngle_M1} (${SPD_AUX_M1}, &STO_aux_Inputs);
	${SPD_aux_calcAvrgElSpeedDpp_M1} (${SPD_AUX_M1});
  </#if>
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */  
  }
  <#if MC.DAC_FUNCTIONALITY>
  DAC_Exec(&DAC_Handle);
  </#if>
<#else> <#-- MC.DUALDRIVE -->
  <#if MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true || MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true >
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main */
  </#if>
  <#if  MC.AUX_STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_PLL2 == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true ||  MC.AUX_STATE_OBSERVER_CORDIC2 == true >
  Observer_Inputs_t STO_aux_Inputs; 
  </#if>

  bMotorNbr = FOC_array[FOC_array_head];
  if (M1 == bMotorNbr)
  {
  <#if  MC.AUX_STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true >
    STO_aux_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
  </#if>

  <#if ( MC.AUX_ENCODER == true) || ( MC.ENCODER == true) >
    (void)ENC_CalcAngle(&ENCODER_M1);
  </#if>
  <#if ( MC.AUX_HALL_SENSORS == true)||( MC.HALL_SENSORS == true)>    
    (void)HALL_CalcElAngle(&HALL_M1);  
  </#if>
  <#if ( MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true)>
    STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;        /* only if motor0 is sensorless*/ 
    if (SWITCH_OVER == Mci[M1].State)
    {
      if (!REMNG_RampCompleted(pREMNG[M1]))
      {
        FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
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
  </#if>
  <#if (true == MC.OTF_STARTUP)>
    if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM1))
    {
      hFOCreturn = FOC_CurrControllerM1();
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    }
  <#else>
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_1 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_1 */
    hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_2 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_2 */
  </#if>
  }
  else /* bMotorNbr != M1 */
  {
  <#if  MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true>
    STO_Inputs.Valfa_beta = FOCVars[M2].Valphabeta;      /* only if motor2 is sensorless*/
  </#if>
  <#if ((  MC.AUX_STATE_OBSERVER_PLL2 == true)||(  MC.AUX_STATE_OBSERVER_CORDIC2 == true))>
    STO_aux_Inputs.Valfa_beta = FOCVars[M2].Valphabeta;   /* only if motor2 is sensorless*/
  </#if>
  <#if ( MC.AUX_ENCODER2 == true) || ( MC.ENCODER2 == true) >
    (void)ENC_CalcAngle(&ENCODER_M2);
  </#if>
  <#if ( MC.AUX_HALL_SENSORS2 == true)||( MC.HALL_SENSORS2 == true)>    
    (void)HALL_CalcElAngle(&HALL_M2);  
  </#if>    
  <#if ( MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true)>
    if (SWITCH_OVER == Mci[M2]State)
    {
      if (!REMNG_RampCompleted(pREMNG[M2]))
      {
        FOCVars[M2].Iqdref.q = ( int16_t )REMNG_Calc(pREMNG[M2]);
      }
    }
  </#if>
  <#if (( MC.OTF_STARTUP2 == true))>
    if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM2))
    {
      hFOCreturn = FOC_CurrControllerM2();
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    }
  <#else>
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_3 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_3 */
    hFOCreturn = FOC_CurrControllerM2();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_4 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_4 */
  </#if>
  }
  if(MC_DURATION == hFOCreturn)
  {
    MCI_FaultProcessing(&Mci[bMotorNbr], MC_DURATION, 0);
  }
  else
  {
    if (M1 == bMotorNbr)
    {
	<#if ( MC.STATE_OBSERVER_PLL == true)>	
	  bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
	</#if>
  <#if  MC.STATE_OBSERVER_PLL == true || MC.STATE_OBSERVER_CORDIC == true >
      STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /*  only for sensorless*/
      (void)${SPD_calcElAngle_M1}(${SPD_M1}, &STO_Inputs);
      ${SPD_calcAvergElSpeedDpp_M1}(${SPD_M1}); /*  Only in case of Sensor-less */
	  <#if ( MC.STATE_OBSERVER_PLL == true)>
	  if (false == IsAccelerationStageReached)
      {
        STO_ResetPLL(&STO_PLL_M1);
      }  
	  </#if> 	
	  
    /*  only for sensor-less*/
    if(((uint16_t)START == Mci[M1].State) || ((uint16_t)SWITCH_OVER == Mci[M1].State)) 
    {
      <#if  MC.SINGLE_SHUNT == true>
      if ((uint16_t)START == Mci[M1].State)
      {  
        if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 1);
        }
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }
      </#if>
      int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super); 
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
  </#if>    
  <#if  MC.AUX_STATE_OBSERVER_PLL == true ||  MC.AUX_STATE_OBSERVER_CORDIC == true>
    STO_aux_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/    
    STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /*  only for sensorless*/ 
    (void)${SPD_aux_calcElAngle_M1}(${SPD_AUX_M1}, &STO_aux_Inputs);
	${SPD_aux_calcAvrgElSpeedDpp_M1}(${SPD_AUX_M1});
  </#if>
    }
    else // bMotorNbr != M1
    {
  <#if (  MC.STATE_OBSERVER_PLL2 == true)>
      bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM2);
  </#if>
	<#if  MC.STATE_OBSERVER_PLL2 == true || MC.STATE_OBSERVER_CORDIC2 == true >
      STO_Inputs.Ialfa_beta = FOCVars[M2].Ialphabeta; /*  only if sensorless*/
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super)); /*  only for sensorless*/
      ${SPD_calcElAngle_M2}(${SPD_M2}, &STO_Inputs);
      ${SPD_calcAvergElSpeedDpp_M2}(${SPD_M2}); /*  Only in case of Sensor-less */
	  <#if ( MC.STATE_OBSERVER_PLL2 == true)>
	  if (false == IsAccelerationStageReached)
      {
        STO_ResetPLL(&STO_PLL_M2);
      }  
	  </#if> 	

      /*  only for sensor-less*/
      if(((uint16_t )START == Mci[M2].State) || ((uint16_t)SWITCH_OVER == Mci[M2].State)) 
      {
        <#if  MC.SINGLE_SHUNT2 == true>      
        if (Mci[M2].State == START)
        {  
          if (0U == RUC_IsAlignStageNow(&RevUpControlM2))
          {
            PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 0);
          }
          else
          {
            PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 1);
          }
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 0);
        }       
        </#if>
        int16_t hObsAngle = SPD_GetElAngle(${SPD_M2}._Super);
        (void)VSS_CalcElAngle(&VirtualSpeedSensorM2, &hObsAngle);
      }
    </#if>
  
    <#if  MC.AUX_STATE_OBSERVER_PLL2 == true ||  MC.AUX_STATE_OBSERVER_CORDIC2 == true>
      STO_aux_Inputs.Ialfa_beta = FOCVars[M2].Ialphabeta; /*  only if sensorless*/    
      STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super)); /*  only for sensorless*/ 
      ${SPD_aux_calcElAngle_M2}(${SPD_AUX_M2}, &STO_aux_Inputs);
	  ${SPD_aux_calcAvrgElSpeedDpp_M2}(${SPD_AUX_M2});
    </#if>
    }    
  }
  FOC_array_head++;
  if (FOC_array_head == FOC_ARRAY_LENGTH)
  {
    FOC_array_head = 0;
  }
  	<#if MC.DAC_FUNCTIONALITY == true>
  DAC_Exec(&DAC_Handle, bMotorNbr);
		</#if>
  </#if>
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  <#if MC.TESTENV == true && MC.PFC_ENABLED == false >
  /* Performance Measurement: end measure */
  stop_perf_measure();
  </#if>
  <#if MC.MCP_DATALOG_USED >
  GLOBAL_TIMESTAMP++;
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_A>
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
  if (0U == MCPA_UART_B.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_B);
  }
  </#if>  
  <#if MC.MCP_DATALOG_OVER_STLNK>
  if (0U == MCPA_STLNK.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_STLNK);
  }
  </#if>

<#if DWT_CYCCNT_SUPPORTED>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_HighFrequencyTask);
</#if>
</#if>
  return (bMotorNbr);
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
 /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  uint8_t bMotorNbr = 0;
  uint16_t hSixStepReturn;
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">

  if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State )) /*  only for sensor-less*/
  {
    if (START == Mci[M1].State)
    {
      if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, RUC_GetDirection(&RevUpControlM1));
      }
    }
    else
    {
      PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
    }
    int16_t hObsAngle = SPD_GetElAngle(&VirtualSpeedSensorM1._Super);
    (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
  }
  (void)BADC_CalcElAngle (&Bemf_ADC_M1);
<#elseif MC.HALL_SENSORS == true>
  (void)HALL_CalcElAngle (&HALL_M1);
</#if>
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
    hSixStepReturn = SixStep_StatorController();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(MC_DURATION == hSixStepReturn)
  {
    MCI_FaultProcessing(&Mci[bMotorNbr], MC_DURATION, 0);
  }
  else
  {
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
    BADC_CalcAvrgElSpeedDpp (&Bemf_ADC_M1); /*  Only in case of Sensor-less */
</#if>
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */
  <#if MC.MCP_DATALOG_USED >
  GLOBAL_TIMESTAMP++;
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_A>
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
  if (0U == MCPA_UART_B.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_B);
  }
  </#if>  
  <#if MC.MCP_DATALOG_OVER_STLNK>
  if (0U == MCPA_STLNK.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_STLNK);
  }
  </#if>
  return (bMotorNbr);
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
}
</#if>

<#if (MC.MOTOR_PROFILER == true)>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Motor control profiler HF task
  * @param  None
  * @retval uint8_t It return always 0.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{
  ab_t Iab;
  
  <#if MC.AUX_HALL_SENSORS>
  HALL_CalcElAngle (&HALL_M1);
  </#if>
  
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
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
<#if G4_Cut2_2_patch == true>  
  RCM_ReadOngoingConv(); 
  RCM_ExecNextConv();
</#if>
  /* The generic function needs to be called here as the undelying   
   * implementation changes in time depending on the Profiler's state 
   * machine. Calling the generic function ensures that the correct
   * implementation is invoked. */
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  FOCVars[M1].Iab = Iab;
  SCC_SetPhaseVoltage(&SCC);
  <#if MC.AUX_HALL_SENSORS>
  HT_GetPhaseShift( &HT );
  </#if>
  
  return (0); /* Single motor only */
}
</#if>

<#if MC.MOTOR_PROFILER == false>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
<#if MC.DRIVE_TYPE == "FOC">
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[M1] );
</#if>  
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
<#if  MC.STATE_OBSERVER_PLL == true ||  MC.STATE_OBSERVER_CORDIC == true>  
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
</#if>  
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  <#if NoInjectedChannel>  
    <#if G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
    </#if>
  RCM_ExecNextConv();
  </#if>
<#if (MC.AMPLIFICATION_GAIN?number <0)>
  /* As the Gain is negative, we invert the current read*/
  Iab.a = -Iab.a;
  Iab.b = -Iab.b;
</#if>
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING == true>
  Vqd = FF_VqdConditioning(pFF[M1],Vqd);
</#if>
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE)
  {
    Vqd = OL_VqdConditioning(pOpenLoop[M1]);
  }   
</#if>
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
<#if NoInjectedChannel &&  !G4_Cut2_2_patch>  
  RCM_ReadOngoingConv();
</#if>  
  hCodeError = PWMC_SetPhaseVoltage${OVM}(pwmcHandle[M1], Valphabeta);
<#if (MC.SINGLE_SHUNT == true ||   MC.OVERMODULATION == true) && (MC.ICS_SENSORS == false)>       
  PWMC_CalcPhaseCurrentsEst(pwmcHandle[M1],Iqd, hElAngle);
</#if>  

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

<#if MC.FLUX_WEAKENING_ENABLING == true>
  FW_DataProcess(pFW[M1], Vqd);
</#if>
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING == true>
  FF_DataProcess(pFF[M1]);
</#if>
  return(hCodeError);
}
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
inline uint16_t SixStep_StatorController(void)
{
  uint16_t hCodeError = MC_NO_ERROR;
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
  int16_t hElAngle, hSpeed, hDirection;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
<#if  MC.DRIVE_MODE == "VM">
  if(false == BADC_IsObserverConverged(&Bemf_ADC_M1))
  {
    hElAngle = SPD_GetElAngle(speedHandle);
  }
  else
  {
    hElAngle = SPD_GetElAngle(&Bemf_ADC_M1._Super);
  }
<#else>
  hElAngle = SPD_GetElAngle(speedHandle);
</#if>
  hSpeed = SPD_GetAvrgMecSpeedUnit(speedHandle);
  hDirection = RUC_GetDirection(&RevUpControlM1);
  <#if CondFamily_STM32F0 || CondFamily_STM32G0>
  if ((RUN == Mci[M1].State) && (true == Bemf_ADC_M1.ZcDetected))
  <#else>
  if ((RUN == Mci[M1].State))
  </#if>  
  {
    RCM_ExecNextConv();
  }
<#if  MC.DRIVE_MODE == "VM">
  PWMC_SetPhaseVoltage( pwmcHandle[M1], SixStepVars[M1].DutyCycleRef );
<#else>
  PWMC_SetPhaseVoltage( pwmcHandle[M1], PWM_Handle_M1._Super.StartCntPh);
  CRM_SetReference( &CurrentRef_M1, SixStepVars[M1].DutyCycleRef );
</#if>
  if (hDirection > 0)
  {
    SixStepVars[M1].qElAngle = hElAngle + S16_90_PHASE_SHIFT;
  }
  else
  {
    SixStepVars[M1].qElAngle = hElAngle - S16_90_PHASE_SHIFT;
  }
  PWM_Handle_M1._Super.hElAngle = SixStepVars[M1].qElAngle;
<#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
  SixPwm_LoadNextStep( &PWM_Handle_M1, hDirection );
  if (true == SixPwm_ApplyNextStep(&PWM_Handle_M1))
<#else>
  ThreePwm_LoadNextStep( &PWM_Handle_M1, hDirection );
  if (true == ThreePwm_ApplyNextStep(&PWM_Handle_M1))
</#if>
  {
    if(START == Mci[M1].State)  /*  only for sensor-less*/
    { 
      BADC_SetLfTimerRevUp(&Bemf_ADC_M1, hSpeed);
    }
  }
	if ((RUN == Mci[M1].State) && (true == Bemf_ADC_M1.ZcDetected))
	{
	  RCM_ReadOngoingConv();
	}
    BADC_Start( &Bemf_ADC_M1, PWM_Handle_M1._Super.Step );
<#elseif MC.HALL_SENSORS == true>
  int16_t hElAngle, hSpeed;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  hSpeed = STC_GetMecSpeedRefUnit(pSTC[M1]);
  if (hSpeed > 0)
  {
    SixStepVars[M1].qElAngle = hElAngle + S16_90_PHASE_SHIFT;
  }
  else
  {
    SixStepVars[M1].qElAngle = hElAngle - S16_90_PHASE_SHIFT;    
  }
  PWM_Handle_M1._Super.hElAngle = SixStepVars[M1].qElAngle;
<#if  MC.DRIVE_MODE == "VM">
  PWMC_SetPhaseVoltage( pwmcHandle[M1], SixStepVars[M1].DutyCycleRef );
<#else>
  PWMC_SetPhaseVoltage( pwmcHandle[M1], PWM_Handle_M1._Super.StartCntPh);
  CRM_SetReference( &CurrentRef_M1, SixStepVars[M1].DutyCycleRef );
</#if>
<#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
  SixPwm_LoadNextStep(&PWM_Handle_M1, hSpeed);
  SixPwm_ApplyNextStep(&PWM_Handle_M1);  
<#else>
  ThreePwm_LoadNextStep(&PWM_Handle_M1, hSpeed);
  ThreePwm_ApplyNextStep(&PWM_Handle_M1);  
</#if>
</#if>
  return(hCodeError);
}
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->

<#if ( MC.DUALDRIVE == true)>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation of motor 2. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM2(void)
{
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  qd_t Iqd, Vqd;

  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M2]);
  hElAngle = SPD_GetElAngle(speedHandle);
<#if  MC.STATE_OBSERVER_PLL2 == true ||  MC.STATE_OBSERVER_CORDIC2 == true>  
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * PARK_ANGLE_COMPENSATION_FACTOR2;
</#if> 
  PWMC_GetPhaseCurrents(pwmcHandle[M2], &Iab);
<#if NoInjectedChannel>  
  <#if G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
  </#if>
  RCM_ExecNextConv();
</#if>
<#if (MC.AMPLIFICATION_GAIN2?number <0)>
  /* As the Gain is negative, we invert the current read*/
  Iab.a = -Iab.a;
  Iab.b = -Iab.b;
</#if>
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M2], (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M2], (int32_t)(FOCVars[M2].Iqdref.d) - Iqd.d);
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
  Vqd = FF_VqdConditioning(pFF[M2],Vqd);
</#if>
  
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  Vqd = OL_VqdConditioning(pOpenLoop[M2]);
</#if>
  Vqd = Circle_Limitation(&CircleLimitationM2, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR2;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage${OVM2}(pwmcHandle[M2], Valphabeta);
<#if (MC.SINGLE_SHUNT2 == true ||   MC.OVERMODULATION2 == true) && (MC.ICS_SENSORS2 == false)>     
  PWMC_CalcPhaseCurrentsEst(pwmcHandle[M2],Iqd, hElAngle);
</#if>  
  <#if NoInjectedChannel && !G4_Cut2_2_patch> 
  RCM_ReadOngoingConv();
  </#if>
  FOCVars[M2].Vqd = Vqd;
  FOCVars[M2].Iab = Iab;
  FOCVars[M2].Ialphabeta = Ialphabeta;
  FOCVars[M2].Iqd = Iqd;
  FOCVars[M2].Valphabeta = Valphabeta;
  FOCVars[M2].hElAngle = hElAngle;
<#if MC.FLUX_WEAKENING_ENABLING2 == true>
  FW_DataProcess(pFW[M2], Vqd);
</#if>
<#if MC.FEED_FORWARD_CURRENT_REG_ENABLING2 == true>
  FF_DataProcess(pFF[M2]);
</#if>
  return(hCodeError);
}
</#if>
</#if>

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
    <#if (MC.MOTOR_PROFILER == true)>
    SCC_CheckOC_RL(&SCC);
    </#if>
    <#if ( MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM")>
    TSK_SafetyTask_PWMOFF(M1);
    <#elseif ( MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M1);
    <#elseif ( MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M1);
    </#if>
<#if ( MC.DUALDRIVE == true)>
    /* Second drive */
    <#if ( MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM")>
    TSK_SafetyTask_PWMOFF(M2);
    <#elseif ( MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M2);
    <#elseif ( MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M2);
    </#if>
</#if>
    /* User conversion execution */
    RCM_ExecUserConv();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

<#if MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
    <#if  MC.DUALDRIVE == true>  
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
    <#else>
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
    </#if>  

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
    <#if  MC.BUS_VOLTAGE_READING == true>
  if(M1 == bMotor)
  {
      <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M1);
      <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1);
      </#if>  
  }
    <#else>
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
    </#if>
    <#if  MC.BUS_VOLTAGE_READING2 == true>
  if(M2 == bMotor)
  {
      <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M2);
      <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2);
      </#if>
  }
    <#else>
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
    </#if>
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* process faults */

  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false; 
  }
  </#if>

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM2){
    ICLFaultTreatedM2 = false; 
  }
  </#if>  

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
    <#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(&SCC);
      OTT_Stop(&OTT);
    </#if>
    <#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >
    /* reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {       
      EAC_SetRestartState(pEAC[bMotor], false);
    }
    </#if>
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    <#if MC.MCP_DATALOG_OVER_UART_A>
    if (MCPA_UART_A.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    </#if>
    <#if MC.MCP_DATALOG_OVER_UART_B>
    if (MCPA_UART_B.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_UART_B);
    }
    </#if>  
    <#if MC.MCP_DATALOG_OVER_STLNK>
    if (MCPA_STLNK.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_STLNK);
    }
    </#if>  
<#if MC.DRIVE_TYPE == "FOC">	
    FOC_Clear(bMotor);
    PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3
</#if><#-- MC.DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">	
    SixStep_Clear(bMotor);
<#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
    BADC_Stop( &Bemf_ADC_M1 );
</#if>
</#if><#-- MC.DRIVE_TYPE == "SIX_STEP" -->
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* no errors */
  }
    <#if  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE == true>
  {
    /* Smooth braking action on overvoltage */
    if(M1 == bMotor)
    {
      busd = (uint16_t)VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super));
    }
    else if(M2 == bMotor)
    {
      busd = (uint16_t)VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super));
    }

    if ((Mci[bMotor].State == IDLE)||
       ((Mci[bMotor].State== RUN)&&(FOCVars[bMotor].Iqdref.q>0)))
    {
      nominalBusd[bMotor] = busd;
    }
    else
    {
      if((Mci[bMotor].State == RUN) && (FOCVars[bMotor].Iqdref.q<0))
      {
        if (busd > ((ovthd[bMotor] + nominalBusd[bMotor]) >> 1))
        {
          FOCVars[bMotor].Iqdref.q = 0;
          FOCVars[bMotor].Iqdref.d = 0;
        }
      }
    }
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF SMOOTH_BREAKING */

    /* USER CODE END TSK_SafetyTask_PWMOFF SMOOTH_BREAKING */
  }
    </#if> <#-- MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE -->
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}
</#if> <#-- MC.ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.ON_OVER_VOLTAGE2 == "TURN_OFF_PWM" -->
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_ON_R_BRAKE
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 0 */

  /* USER CODE END TSK_SafetyTask_RBRK 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t BusVoltageFaultsFlag = MC_OVER_VOLT;
  <#if  MC.DUALDRIVE == true>  
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
  <#else>
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  </#if> 

  /* Brake resistor management */
  <#if  MC.BUS_VOLTAGE_READING == true>
  if(M1 == bMotor)
  {
    <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M1);
    <#else>
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1);
    </#if>
  }
  <#else> 
 <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if>
  <#if  MC.BUS_VOLTAGE_READING2 == true>
  if(M2 == bMotor)
  {
    <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    BusVoltageFaultsFlag = errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M2);
    <#else>
    BusVoltageFaultsFlag = errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2);
    </#if>
  }
  <#else>
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if> <#-- MC.BUS_VOLTAGE_READING2 == true -->
  if (MC_OVER_VOLT == BusVoltageFaultsFlag)
  {
    DOUT_SetOutputState(pR_Brake[bMotor], ACTIVE);
  }
  else
  {
    DOUT_SetOutputState(pR_Brake[bMotor], INACTIVE);
  }
  CodeReturn |= NTC_CalcAvTemp(pTemperatureSensor[bMotor]);   /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= (BusVoltageFaultsFlag & MC_UNDER_VOLT);       /* MC_UNDER_VOLT generates fault if FW protection is activated,
                                                                 MC_OVER_VOLT doesn't generate fault */
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */

  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (BusVoltageFaultsFlag & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false; 
  }
  </#if>

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (BusVoltageFaultsFlag & MC_UNDER_VOLT)) && ICLFaultTreatedM2){
    ICLFaultTreatedM2 = false; 
  }
  </#if>
  
  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
  <#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(&SCC);
      OTT_Stop(&OTT);
  </#if>
  <#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >  
      /* reset Encoder state */
      if (pEAC[bMotor] != MC_NULL)
      {       
        EAC_SetRestartState( pEAC[bMotor], false );
      }
  </#if>
      PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_DATALOG_OVER_UART_A>
    if (MCPA_UART_A.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_UART_A);
    }
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
    if (MCPA_UART_B.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_UART_B);
    }
  </#if>  
  <#if MC.MCP_DATALOG_OVER_STLNK>
    if (MCPA_STLNK.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_STLNK);
    }
  </#if>      
      FOC_Clear(bMotor);
      PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3
      /* USER CODE BEGIN TSK_SafetyTask_RBRK 1 */

      /* USER CODE END TSK_SafetyTask_RBRK 1 */
  }                 
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 2 */

  /* USER CODE END TSK_SafetyTask_RBRK 2 */  
}
</#if> <#-- MC.ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_R_BRAKE" -->
<#if MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES">
/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_LSON(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_LSON 0 */

  /* USER CODE END TSK_SafetyTask_LSON 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  <#if  MC.DUALDRIVE == true>  
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
  <#else>
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  </#if> 
  bool TurnOnLowSideAction;
  
  TurnOnLowSideAction = PWMC_GetTurnOnLowSidesAction(pwmcHandle[bMotor]);
  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* for fault. It return MC_BREAK_IN or MC_NO_FAULTS
                                                                                (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  /* USER CODE BEGIN TSK_SafetyTask_LSON 1 */

  /* USER CODE END TSK_SafetyTask_LSON 1 */
  <#if  MC.BUS_VOLTAGE_READING == true>
  if(M1 == bMotor)
  {
    <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M1);
    <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1);
    </#if>
  }
  <#else>
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if>
  <#if  MC.BUS_VOLTAGE_READING2 == true>
  if(M2 == bMotor)
  {
    <#if CondMcu_STM32F302x8x || CondFamily_STM32F3>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbusFilt(&BusVoltageSensor_M2);
    <#else>
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2);
    </#if> 
  }
  <#else>
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... --> 
  </#if>
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false; 
  }
  </#if>

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM2){
    ICLFaultTreatedM2 = false; 
  }
  </#if>  
  
  if ((MC_OVER_VOLT == (CodeReturn & MC_OVER_VOLT)) && (false == TurnOnLowSideAction))
  {
  <#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true > 
    /* reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {       
      EAC_SetRestartState(pEAC[bMotor], false);
    }
  </#if>
    /* Start turn on low side action */
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]); /* Required before //PWMC_TurnOnLowSides */
  <#if ( MC.HW_OV_CURRENT_PROT_BYPASS == true)>
    DOUT_SetOutputState(pOCPDisabling[bMotor], ACTIVE); /* Disable the OCP */
  </#if>
    /* USER CODE BEGIN TSK_SafetyTask_LSON 2 */

    /* USER CODE END TSK_SafetyTask_LSON 2 */
    PWMC_TurnOnLowSides(pwmcHandle[bMotor], 0UL); /* Turn on Low side switches */
  }
  else
  {
    switch (Mci[bMotor].State) /* Is state equal to FAULT_NOW or FAULT_OVER */
    {
    
      case IDLE:
      {
        /* After a OV occurs the turn on low side action become active. It is released just after a fault acknowledge -> state == IDLE */
        if (true == TurnOnLowSideAction)
        {
          /* End of TURN_ON_LOW_SIDES action */
  <#if ( MC.HW_OV_CURRENT_PROT_BYPASS == true)>
          DOUT_SetOutputState(pOCPDisabling[bMotor], INACTIVE); /* Re-enable the OCP */
  </#if>
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);  /* Switch off the PWM */
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 3 */

        /* USER CODE END TSK_SafetyTask_LSON 3 */
        break;
      }
      
      case FAULT_NOW:
      {
        if (TurnOnLowSideAction == false)
        {
  <#if  MC.ENCODER == true || MC.ENCODER2 == true || MC.AUX_ENCODER == true || MC.AUX_ENCODER2 == true >   
          /* reset Encoder state */
          if (pEAC[bMotor] != MC_NULL)
          {       
            EAC_SetRestartState(pEAC[bMotor], false);
          }
  </#if>
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_DATALOG_OVER_UART_A>
          if (MCPA_UART_A.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_A);
          }
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
          if (MCPA_UART_B.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_B);
          }
  </#if>  
  <#if MC.MCP_DATALOG_OVER_STLNK>
          if (MCPA_STLNK.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_STLNK);
          }
  </#if>              
          FOC_Clear(bMotor);
          /* Misra violation Rule 11.3 A cast shall not be performed between a pointer to object 
           * type and a pointer to a different object type. */
          PQD_Clear(pMPM[bMotor]);
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 4 */

        /* USER CODE END TSK_SafetyTask_LSON 4 */
        break;
      }
      
      case FAULT_OVER:
      {
        if (TurnOnLowSideAction == false)
        {
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_DATALOG_OVER_UART_A>
          if (MCPA_UART_A.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_A);
          }
  </#if>
  <#if MC.MCP_DATALOG_OVER_UART_B>
          if (MCPA_UART_B.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_B);
          }
  </#if>  
  <#if MC.MCP_DATALOG_OVER_STLNK>
          if (MCPA_STLNK.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_STLNK);
          }
  </#if>                        
        }
	    /* USER CODE BEGIN TSK_SafetyTask_LSON 5 */

        /* USER CODE END TSK_SafetyTask_LSON 5 */
        break;
      }
      
      default:
        break;
    }
  }
  /* USER CODE BEGIN TSK_SafetyTask_LSON 6 */

  /* USER CODE END TSK_SafetyTask_LSON 6 */
}
</#if> <#-- MC.ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.ON_OVER_VOLTAGE2 == "TURN_ON_LOW_SIDES" -->

<#if  MC.DUALDRIVE == true>
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Reserves FOC execution on ADC ISR half a PWM period in advance
  *
  *  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  * it allows to reserve half PWM period in advance the FOC execution on
  * ADC ISR
  * @param  pDrive Pointer on the FOC Array
  */
__weak void TSK_DualDriveFIFOUpdate(uint8_t Motor)
{
  FOC_array[FOC_array_tail] = Motor;
  FOC_array_tail++;
  if (FOC_ARRAY_LENGTH == FOC_array_tail)
  {
    FOC_array_tail = 0;
  }
}
</#if>

<#if MC.DRIVE_TYPE == "FOC">  
/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if (bMotor < (uint8_t)NBR_OF_MOTORS)
  {
    retVal = &Mci[bMotor];
  }
  return (retVal);
}
</#if>

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
<#if MC.DRIVE_TYPE == "FOC">    
<#if (MC.MOTOR_PROFILER == true)>
  SCC_Stop(&SCC);
  OTT_Stop(&OTT);
</#if>
</#if>
  ${PWM_SwitchOff}(pwmcHandle[M1]);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);
<#if  MC.DUALDRIVE == true>
  ${PWM_SwitchOff_M2}(pwmcHandle[M2]);
  MCI_FaultProcessing(&Mci[M2], MC_SW_ERROR, 0);
</#if>

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
<#if MC.PFC_ENABLED == true>

/**
  * @brief  Executes the PFC Task.
  */
void PFC_Scheduler(void)
{
	PFC_Task(&PFC);
}
</#if>
<#if MC.RTOS == "FREERTOS">

/* startMediumFrequencyTask function */
void startMediumFrequencyTask(void const * argument)
{
<#if MC.CUBE_MX_VER == "xxx" >
  /* init code for MotorControl */
  MX_MotorControl_Init();
<#else>
<#assign cubeVersion = MC.CUBE_MX_VER?replace(".","") >
<#if cubeVersion?number < 540 >
  /* init code for MotorControl */
  MX_MotorControl_Init();
</#if>  
</#if> 
  /* USER CODE BEGIN MF task 1 */
  /* Infinite loop */
  for(;;)
  {
    /* delay of 500us */
    vTaskDelay(1);
    MC_RunMotorControlTasks();
  }
  /* USER CODE END MF task 1 */
}

/* startSafetyTask function */
void StartSafetyTask(void const * argument)
{
  /* USER CODE BEGIN SF task 1 */
  /* Infinite loop */
  for(;;)
  {
    /* delay of 500us */
    vTaskDelay(1);
    TSK_SafetyTask();
  }
  /* USER CODE END SF task 1 */ 
}

</#if>   

<#if MC.START_STOP_BTN == true>
__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  if (IDLE == MC_GetSTMStateMotor1())
  {
    /* Ramp parameters should be tuned for the actual motor */
    (void)MC_StartMotor1();
  }
  else
  {
    (void)MC_StopMotor1();
  }
/* USER CODE END START_STOP_BTN */
}
</#if>

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
__weak void mc_lock_pins (void)
{
<#list configs as dt>
<#list dt.peripheralGPIOParams.values() as io>
<#list io.values() as ipIo>
<#list ipIo.entrySet() as e>
<#if (e.getKey().equals("GPIO_Label")) && (e.getValue()?matches("^M[0-9]+_.*$"))>
LL_GPIO_LockPin(${e.getValue()}_GPIO_Port, ${e.getValue()}_Pin);
</#if>
</#list>
</#list>
</#list>
</#list>
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
