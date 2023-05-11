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


<#-- >>> NEW DATA MODEL --> 
  
<#assign OPAMPInputMapp_Shared_U =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ] >
 <#assign ADCConfig_2OPAMPs_Shared_U =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W} 
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W} 
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V} 
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V} 
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W} 
   ] > 
   
<#assign OPAMPInputMapp_Shared_V =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_NonInvertingInput_"+MC.M1_CS_OPAMP_VPSEL_W}
   ] >

 <#assign ADCConfig_2OPAMPs_Shared_V =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ] > 

<#assign OPAMPInputMapp_Shared_W =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ] >

 <#assign ADCConfig_2OPAMPs_Shared_W =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U}
   ] > 

<#assign OPAMPInputMapp_3_OPAMPS =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_OPAMP_U+"_"+MC.M1_CS_OPAMP_VPSEL_U , "PHASE_2" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_OPAMP_V+"_"+MC.M1_CS_OPAMP_VPSEL_V , "PHASE_2" : MC.M1_CS_OPAMP_W+"_"+MC.M1_CS_OPAMP_VPSEL_W}
   ] >

 <#assign ADCConfig_3ADCS =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ] > 

<#assign ADCConfig_Shared_U =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_SHARED , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_SHARED , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ] >   

<#assign ADCConfig_Shared_V =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_SHARED}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_SHARED}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_W+"_"+MC.M1_CS_CHANNEL_W}
   ] >   
   

<#assign ADCConfig_Shared_W =
  [ {"Sector": 1 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_W}
   ,{"Sector": 2 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_SHARED}
   ,{"Sector": 3 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_SHARED}
   ,{"Sector": 4 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 5 , "PHASE_1" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_U , "PHASE_2" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V}
   ,{"Sector": 6 , "PHASE_1" : MC.M1_CS_ADC_V+"_"+MC.M1_CS_CHANNEL_V , "PHASE_2" : MC.M1_CS_ADC_U+"_"+MC.M1_CS_CHANNEL_W}
   ] >   
  
 <#function getOPAMPMap Motor> 
   <#if Motor == 1>
     <#switch MC.M1_CS_OPAMP_PHASE_SHARED>
       <#case "U">
         <#local OPAMPMapp = OPAMPInputMapp_Shared_U>
         <#break>
       <#case "V">
         <#local OPAMPMapp = OPAMPInputMapp_Shared_V>
         <#break>
       <#case "W">  
         <#local OPAMPMapp = OPAMPInputMapp_Shared_W>
         <#break>
       <#default> <#-- No phase shared at Opmap Level -->
         <#local OPAMPMapp=OPAMPInputMapp_3_OPAMPS>
         <#break>
     </#switch>   
   <#elseif Motor==2>
   <#-- To be implemented -->
   </#if>
   <#return OPAMPMapp>
</#function>   
  
<#assign OPAMPip = {"U":MC.M1_CS_OPAMP_U ,"V":MC.M1_CS_OPAMP_V ,"W":MC.M1_CS_OPAMP_W }>

<#function getOPAMP config Sector Phase Motor=1>
 <#local OPAMPMap=getOPAMPMap(Motor)>
 <#list OPAMPMap as OPAMPItem>
   <#if OPAMPItem.Sector == Sector >
     <#if MC.M1_CS_OPAMP_DYNAMIC_OUTPUT_SWITCH == false>
       <#if config=="IP">
         <#return _first_word(OPAMPItem[Phase])>
       <#else>  
         <#return OPAMPItem[Phase]>
       </#if>
     <#else> <#-- Dynamic opamp to configure -->
       <#if _first_word(OPAMPItem[Phase]) == OPAMPip[MC.M1_CS_ADC_PHASE_SHARED]>
         <#if config=="IP">
           <#return OPAMPip[MC.M1_CS_ADC_PHASE_SHARED]>
         <#else>
           <#if MC.M1_CS_OPAMP_DIRECT_LINK_TO_ADC == getADC("IP",Sector,Phase)>
             <#return "DIRECT_CONNECT">
           <#else>
             <#return "PIN_CONNECT">
           </#if>
         </#if>
       <#else> <#-- Not an OPAMP we configure return "NULL" -->
         <#if config=="IP">
           <#return _first_word(OPAMPItem[Phase])>
         <#else>
           <#return "OPAMP_UNCHANGED">
         </#if>
       </#if>
     </#if>
   </#if>
 </#list> 
</#function>

<#function getADCMap Motor> 
   <#if Motor == 1>
     <#switch MC.M1_CS_ADC_PHASE_SHARED>
       <#case "U">
         <#local ADCMap = ADCConfig_Shared_U>
         <#break>
       <#case "V">
         <#local ADCMap = ADCConfig_Shared_V>
         <#break>
       <#case "W">  
         <#local ADCMap = ADCConfig_Shared_W>
         <#break>
       <#default> <#-- No pahse shared at ADC Level -->
         <#switch MC.M1_CS_OPAMP_PHASE_SHARED>
           <#case "U">
             <#local ADCMap=ADCConfig_2OPAMPs_Shared_U>
             <#break>
           <#case "V">
             <#local ADCMap=ADCConfig_2OPAMPs_Shared_V>
             <#break>
           <#case "W">
             <#local ADCMap=ADCConfig_2OPAMPs_Shared_W>
             <#break>
           <#default>
             <#local ADCMap=ADCConfig_3ADCS> <#-- No phase shared at ADC level not OPAMP level -->          
            <#break>
         </#switch>
       <#break>
     </#switch>   
   <#elseif Motor==2>
   <#-- To be implemented -->
   </#if>
   <#return ADCMap>
</#function> 

<#function getADC config sector phase Motor=1>
 <#local ADCMap = getADCMap (Motor)>
 <#list ADCMap as ADCItem>
   <#if ADCItem.Sector==sector>
     <#if config="IP">
       <#return _first_word(ADCItem[phase])>
     <#else>
       <#return _last_word(ADCItem[phase])>
     </#if>  
   </#if>
 </#list>
</#function>

<#assign PWM_Timer_M1 = _last_word(MC.PWM_TIMER_SELECTION) >
<#assign PWM_Timer_M2 = _last_word(MC.PWM_TIMER_SELECTION2) >

<#if MC.M1_CS_ADC_NUM == '2'>
<#assign ADC1Set_M1>
 <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>    
</#assign>              
<#assign ADC2Set_M1>
  <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>
</#assign> 
</#if>
<#-- <<< NEW DATA MODEL --> 


<#-- Condition for STM32F302x8x MCU -->
<#assign CondMcu_STM32F302x8x = (McuName?? && McuName?matches("STM32F302.8.*"))>
<#-- Condition for STM32F072xxx MCU -->
<#assign CondMcu_STM32F072xxx = (McuName?? && McuName?matches("STM32F072.*"))>
<#-- Condition for STM32F446xCx or STM32F446xEx -->
<#assign CondMcu_STM32F446xCEx = (McuName?? && McuName?matches("STM32F446.(C|E).*"))>
<#-- Condition for STM32F0 Family -->
<#assign CondFamily_STM32F0 = (FamilyName?? && FamilyName=="STM32F0")>
<#-- Condition for STM32F3 Family -->
<#assign CondFamily_STM32F3 = (FamilyName?? && FamilyName == "STM32F3") >
<#-- Condition for STM32F4 Family -->
<#assign CondFamily_STM32F4 = (FamilyName?? && FamilyName == "STM32F4") >
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
<#function _first_word text sep="_"><#return text?split(sep)?first></#function>

<#function _filter_opamp opamp >
  <#if opamp == "OPAMP1" >
   <#return "OPAMP" >
  <#else>
   <#return opamp >
  </#if>
</#function>

<#macro setScandir Ph1 Ph2>
<#if Ph1?number < Ph2?number>
   LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
<#else>
   LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
</#if>
<#return>
</#macro>
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

<#if MC.DRIVE_TYPE == "FOC">
  <#if CondFamily_STM32F4 > 
    <#if MC.SINGLE_SHUNT2 == true || MC.SINGLE_SHUNT == true >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
    <#if MC.THREE_SHUNT == true >
#include "r3_1_f4xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true >
#include "r3_2_f4xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS == true || MC.ICS_SENSORS2 == true >
#include "ics_f4xx_pwm_curr_fdbk.h"
    </#if>
  </#if>
  <#if CondFamily_STM32F0 >
    <#if MC.THREE_SHUNT == true >
#include "r3_f0xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.SINGLE_SHUNT == true >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
  </#if>
  <#if CondFamily_STM32F3 > <#-- CondFamily_STM32F3 --->
    <#if MC.SINGLE_SHUNT || MC.SINGLE_SHUNT2 >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS ||  MC.ICS_SENSORS2>
#include "ics_f30x_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 ||
          MC.THREE_SHUNT_SHARED_RESOURCES  || MC.THREE_SHUNT_SHARED_RESOURCES2>
#include "r3_2_f30x_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT  >
#include "r3_1_f30x_pwm_curr_fdbk.h"
    </#if>
  </#if> <#-- CondFamily_STM32F3 --->
  <#if CondFamily_STM32G4 > <#-- CondFamily_STM32G4 --->
    <#if MC.SINGLE_SHUNT || MC.SINGLE_SHUNT2 >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 >
#include "r3_2_g4xx_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS ||  MC.ICS_SENSORS2>
#include "ics_g4xx_pwm_curr_fdbk.h"
    </#if>  
  </#if> <#-- CondFamily_STM32G4 --->
  <#if CondFamily_STM32G0 > <#-- CondFamily_STM32G0 --->
    <#if MC.SINGLE_SHUNT >
#include "r1_ps_pwm_curr_fdbk.h"   
    </#if>
    <#if  MC.THREE_SHUNT  >
#include "r3_g0xx_pwm_curr_fdbk.h"
    </#if>
  </#if> <#-- CondFamily_STM32G4 --->
  <#if CondFamily_STM32L4 > <#-- CondFamily_STM32L4 --->
    <#if MC.SINGLE_SHUNT >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS >
#include "ics_l4xx_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES  >
#include "r3_2_l4xx_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT  >
#include "r3_1_l4xx_pwm_curr_fdbk.h"
    </#if>
  </#if> <#-- CondFamily_STM32L4 --->
  <#if CondFamily_STM32F7 > <#-- CondFamily_STM32F7 --->
    <#if MC.SINGLE_SHUNT >
#include "r1_ps_pwm_curr_fdbk.h"
    </#if>
    <#if MC.ICS_SENSORS >
#include "ics_f7xx_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES  >
#include "r3_2_f7xx_pwm_curr_fdbk.h"
    </#if>
    <#if  MC.THREE_SHUNT  >
#include "r3_1_f7xx_pwm_curr_fdbk.h"
    </#if>
  </#if> <#-- CondFamily_STM32F7 --->
  <#if CondFamily_STM32H7 > <#-- CondFamily_STM32H7 --->
    <#if MC.SINGLE_SHUNT || MC.SINGLE_SHUNT2 >
#error " H7 Single shunt not supported yet "
    </#if>
    <#if MC.ICS_SENSORS ||  MC.ICS_SENSORS2>
#error " H7 Single shunt not supported yet "
    </#if>
    <#if  MC.THREE_SHUNT_INDEPENDENT_RESOURCES || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 ||
          MC.THREE_SHUNT_SHARED_RESOURCES  || MC.THREE_SHUNT_SHARED_RESOURCES2>
#include "r3_2_h7xx_pwm_curr_fdbk.h"
    </#if>

    <#if  MC.THREE_SHUNT  >
#include "r3_1_f30x_pwm_curr_fdbk.h"
    </#if>
  </#if> <#-- CondFamily_STM32F3 --->
 
  <#if MC.PFC_ENABLED == true>
#include "pfc.h"
  </#if>
  <#if MC.MOTOR_PROFILER == true>
#include "mp_self_com_ctrl.h"
#include "mp_one_touch_tuning.h"
  </#if>
<#if  MC.ESC_ENABLE>
#include "esc.h"
</#if>
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */  

  <#if MC.SINGLEDRIVE == true>
#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
  </#if> 


  <#if CondFamily_STM32F0 || CondFamily_STM32G0 > 
    <#if MC.THREE_SHUNT>
extern  PWMC_R3_1_Handle_t PWM_Handle_M1;
    </#if> <#-- MC.THREE_SHUNT -->
  </#if> <#-- CondFamily_STM32F0 || CondFamily_STM32G0 -->

  <#if CondFamily_STM32F4 > 
    <#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true> <#-- inside CondFamily_STM32F4 -->
/**
  * @brief  Current sensor parameters Motor 2 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM2 =
{

/* Dual MC parameters --------------------------------------------------------*/
  .Tw               = MAX_TWAIT2,
  .bFreqRatio       = FREQ_RATIO,
  .bIsHigherFreqTim = FREQ_RELATION2,

  .ADCx_1                  = ${MC.ADC_1_PERIPH2},
  .ADCx_2                  = ${MC.ADC_2_PERIPH2},

/* PWM generation parameters --------------------------------------------------*/
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION2)},
  .hDeadTime          = DEAD_TIME_COUNTS2,
  .RepetitionCounter  = REP_COUNTER2,
  .hTafter            = TW_AFTER2,
  .hTbefore           = TW_BEFORE2,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE2 == false>
  .pwm_en_u_port     = M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin      = M2_PWM_EN_U_Pin,
  .pwm_en_v_port     = M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin      = M2_PWM_EN_V_Pin,
  .pwm_en_w_port     = M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin      = M2_PWM_EN_W_Pin,    
        <#else>
  .pwm_en_u_port     = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin      = M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port     = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin      = M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port     = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin      = M2_PWM_EN_UVW_Pin, 
        </#if>
      </#if>

   //cstat -MISRAC2012-Rule-12.1 -MISRAC2012-Rule-10.1_R6 
  .ADCConfig1 = {   ( uint32_t )( MC_${MC.PHASE_V_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_U_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_U_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_U_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_U_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_V_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                  },

  .ADCConfig2 = {   ( uint32_t )( MC_${MC.PHASE_W_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_W_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_W_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_V_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_V_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( MC_${MC.PHASE_W_CURR_CHANNEL2}<<ADC_JSQR_JSQ4_Pos )
                  },
  //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6 
  .ADCDataReg1 = {  &${MC.ADC_1_PERIPH2}->JDR1 // Phase B, Phase C
                   ,&${MC.ADC_1_PERIPH2}->JDR1 // Phase A, Phase C
                   ,&${MC.ADC_1_PERIPH2}->JDR1 // Phase A, Phase C
                   ,&${MC.ADC_1_PERIPH2}->JDR1 // Phase A, Phase B
                   ,&${MC.ADC_1_PERIPH2}->JDR1 // Phase A, Phase B
                   ,&${MC.ADC_1_PERIPH2}->JDR1 // Phase B, Phase C
                  },

  .ADCDataReg2 = {  &${MC.ADC_2_PERIPH2}->JDR1  // Phase B, Phase C
                   ,&${MC.ADC_2_PERIPH2}->JDR1  // Phase A, Phase C
                   ,&${MC.ADC_2_PERIPH2}->JDR1  // Phase A, Phase C
                   ,&${MC.ADC_2_PERIPH2}->JDR1  // Phase A, Phase B
                   ,&${MC.ADC_2_PERIPH2}->JDR1  // Phase A, Phase B
                   ,&${MC.ADC_2_PERIPH2}->JDR1  // Phase B, Phase C
                   },

   
/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop = (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING2==true>ENABLE<#else>DISABLE</#if>,
};
    </#if> <#-- MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 == true -->
 
    <#if MC.THREE_SHUNT == true > <#-- inside CondFamily_STM32F4 -->
  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - STM32F401x8
  */
R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx = ${MC.M1_CS_ADC_U},     

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER, 
  .hTafter            = TW_AFTER, 
  .hTbefore           = TW_BEFORE_R3_1,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},
  .Tsampling                  = (uint16_t)SAMPLING_TIME,
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" > 
  .pwm_en_u_port = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin  = M1_PWM_EN_U_Pin,
  .pwm_en_v_port = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin  = M1_PWM_EN_V_Pin,
  .pwm_en_w_port = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin  = M1_PWM_EN_W_Pin,
      <#else>
  .pwm_en_u_port = MC_NULL,
  .pwm_en_u_pin  = (uint16_t) 0, 
  .pwm_en_v_port = MC_NULL,
  .pwm_en_v_pin  = (uint16_t) 0,
  .pwm_en_w_port = MC_NULL,
  .pwm_en_w_pin  = (uint16_t) 0,
      </#if>

  .ADCConfig = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
              },
  
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .EmergencyStop = DISABLE,
};
    </#if> <#-- MC.THREE_SHUNT == true -->

    <#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true > <#-- inside CondFamily_STM32F4 -->

/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
  .Tw                       = MAX_TWAIT,
  .bFreqRatio               = FREQ_RATIO,
  .bIsHigherFreqTim         = FREQ_RELATION,

/* Current reading A/D Conversions initialization ----------------------------*/
 .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
 .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
   
/* PWM generation parameters --------------------------------------------------*/
  .TIMx                       =  ${_last_word(MC.PWM_TIMER_SELECTION)},
  .RepetitionCounter          =  REP_COUNTER,
  .hTafter                    =  TW_AFTER,
  .hTbefore                   =  TW_BEFORE,
  .Tsampling                  = (uint16_t)SAMPLING_TIME,
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs             =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =  M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_UVW_Pin,
        </#if>                    
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING == ES_GPIO -->

  .ADCConfig1 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos )
              },
  .ADCConfig2 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
                   ,( uint32_t )( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos )
              },                   
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =  (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING>ENABLE<#else>DISABLE</#if>,
};
    </#if> <#-- MC.THREE_SHUNT_INDEPENDENT_RESOURCES == true -->

    <#if  MC.ICS_SENSORS == true>
/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS
  */
const ICS_Params_t ICS_ParamsM1 = {

/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr =      1,
  .Tw =            MAX_TWAIT,
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel       =  MC_${MC.PHASE_U_CURR_CHANNEL},
  .IbChannel       =  MC_${MC.PHASE_V_CURR_CHANNEL},
  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs               =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =  M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_W_Pin, 
        <#else>
  .pwm_en_u_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_UVW_Pin, 
        </#if>  
      <#else>
  .pwm_en_u_port          =  MC_NULL,
  .pwm_en_u_pin           = (uint16_t) 0,
  .pwm_en_v_port          =  MC_NULL,
  .pwm_en_v_pin           = (uint16_t) 0,
  .pwm_en_w_port          =  MC_NULL,
  .pwm_en_w_pin           = (uint16_t) 0,
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING == ES_GPIO -->

/* Emergengy signal initialization ----------------------------------------*/
  .EmergencyStop        =  (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING>ENABLE<#else>DISABLE</#if>,
  
};
    </#if> <#-- MC.ICS_SENSORS == true) -->

    <#if  MC.ICS_SENSORS2 == true>
/**
  * @brief  Current sensor parameters Dual Drive Motor 2 - ICS
  */
const ICS_Params_t ICS_ParamsM2 = {
/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr     = 2,
  .Tw               = MAX_TWAIT2,
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2,
 

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel       = MC_${MC.PHASE_U_CURR_CHANNEL2},
  .IbChannel       = MC_${MC.PHASE_V_CURR_CHANNEL2},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION2)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE2 == false>
  .pwm_en_u_port          =  M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M2_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M2_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M2_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port          =  M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port          =  M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M2_PWM_EN_UVW_Pin,
        </#if> 
      <#else>
  .pwm_en_u_port          =  MC_NULL,
  .pwm_en_u_pin           = (uint16_t) 0,
  .pwm_en_v_port          =  MC_NULL,
  .pwm_en_v_pin           = (uint16_t) 0,
  .pwm_en_w_port          =  MC_NULL,
  .pwm_en_w_pin           = (uint16_t) 0,
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING2 == ES_GPIO -->

/* Emergency signal initialization ----------------------------------------*/
  .EmergencyStop =   (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING2==true>ENABLE<#else>DISABLE</#if>, 

};
    </#if> <#-- MC.ICS_SENSORS2 == true) -->

  </#if> <#-- <#if CondFamily_STM32F4 -->

  <#if CondFamily_STM32F0 >
    <#if MC.THREE_SHUNT == true>
<#assign phaseA = (MC.PHASE_U_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >
<#assign phaseB = (MC.PHASE_V_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >
<#assign phaseC = (MC.PHASE_W_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >  
/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
const R3_1_Params_t R3_1_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .b_ISamplingTime =  LL_ADC_SAMPLINGTIME_${MC.CURR_SAMPLING_TIME}<#if MC.CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter = TW_AFTER,
  .hTbefore = TW_BEFORE_R3_1, 
  .TIMx = ${_last_word(MC.PWM_TIMER_SELECTION)},
  .Tsampling                  = (uint16_t)SAMPLING_TIME,
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs= (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =  M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_UVW_Pin,
        </#if>                 
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING == ES_GPIO -->   
  .ADCConfig = {
                ( uint32_t )( 1<< ${getADC("CFG",1,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",1,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",2,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",2,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",3,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",3,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",4,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",4,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",5,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",5,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",6,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",6,"PHASE_2")} ),
  },
  .ADCScandir = {
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_V/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_V/>
  },
  .ADCDataReg1 = {          
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
  },
     
  .ADCDataReg2 = {
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],  
  },
};
    </#if> <#-- MC.THREE_SHUNT == true  -->
  </#if> <#-- CondFamily_STM32F0 -->
  <#if CondFamily_STM32G0 >
    <#if MC.THREE_SHUNT == true>
<#assign phaseA = (MC.PHASE_U_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >
<#assign phaseB = (MC.PHASE_V_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >
<#assign phaseC = (MC.PHASE_W_CURR_CHANNEL?replace("ADC_CHANNEL_", ""))?number >  
/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32G0X
  */
const R3_1_Params_t R3_1_Params =
{
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter = TW_AFTER,
  .hTbefore = TW_BEFORE_R3_1, 
  .TIMx = ${_last_word(MC.PWM_TIMER_SELECTION)},
  .Tsampling                  = (uint16_t)SAMPLING_TIME,         
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,   
   
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs= (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =  M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_UVW_Pin, 
  .pwm_en_v_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_UVW_Pin,
        </#if>                 
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING == ES_GPIO -->
  .ADCConfig = {
                ( uint32_t )( 1<< ${getADC("CFG",1,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",1,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",2,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",2,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",3,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",3,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",4,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",4,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",5,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",5,"PHASE_2")} ),
                ( uint32_t )( 1<< ${getADC("CFG",6,"PHASE_1")} ) | ( uint32_t )( 1<< ${getADC("CFG",6,"PHASE_2")} ),
  },
  .ADCScandir = {
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_V/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_V/>
  },
  .ADCDataReg1 = {          
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
  },
     
  .ADCDataReg2 = {
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],  
  },
};
    </#if> <#-- MC.THREE_SHUNT == true  -->
  </#if> <#-- CondFamily_STM32G0 -->
  <#if CondFamily_STM32L4 > <#-- CondFamily_STM32L4 --->
    <#if MC.ICS_SENSORS > <#-- Inside CondFamily_STM32L4 --->
ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ADC1,
  .ADCx_2 = ADC2,
  .IaChannel = MC_${MC.PHASE_U_CURR_CHANNEL},
  .IbChannel = MC_${MC.PHASE_V_CURR_CHANNEL},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >  
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
  .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
  .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port, 
  .pwm_en_w_pin       = M1_PWM_EN_W_Pin,   
        <#else>
  .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port      = MC_NULL, 
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
      </#if> 

/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = ${MC.BKIN2_MODE},

};
<#elseif  MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32L4 --->
  <#if MC.M1_CS_ADC_NUM=='1'> <#-- Inside CondFamily_STM32L4 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC 
  */
R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx = ${MC.M1_CS_ADC_U},     
  .ADCConfig = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),                   
               },
  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter            = TW_AFTER,
  .hTbefore           = TW_BEFORE_R3_1,  
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin  = M1_PWM_EN_U_Pin,
  .pwm_en_v_port = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin  = M1_PWM_EN_V_Pin,
  .pwm_en_w_port = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin  = M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin  = M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin  = M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin  = M1_PWM_EN_UVW_Pin, 
        </#if>
      <#else>
  .pwm_en_u_port = MC_NULL,
  .pwm_en_u_pin  = (uint16_t) 0,
  .pwm_en_v_port = MC_NULL,
  .pwm_en_v_pin  = (uint16_t) 0,
  .pwm_en_w_port = MC_NULL,
  .pwm_en_w_pin  = (uint16_t) 0,
      </#if>
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode = ${MC.BKIN2_MODE},
};
  <#elseif MC.M1_CS_ADC_NUM=='2'> <#-- Inside CondFamily_STM32L4 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - L4XX - Independent Resources
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .bFreqRatio       = FREQ_RATIO,
  .bIsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
 .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
 .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
              
  .ADCConfig1 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
              },
  .ADCConfig2 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  ( uint32_t )( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | 0<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
              },                             
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },
                  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter            = TW_AFTER,
  .hTbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
 .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false>
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if> 
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>
 
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode     = ${MC.BKIN2_MODE}, 
};
</#if>
</#if>
  </#if> <#-- CondFamily_STM32L4 --->
  <#if CondFamily_STM32F7 > <#-- CondFamily_STM32F7 --->
  
                                                                                
                                     
                                     
                                                     
  
                                                    
  <#if  MC.ICS_SENSORS> <#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS
  */
ICS_Params_t ICS_ParamsM1 = {

/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr =      1,
  .Tw =            MAX_TWAIT,
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel       =  MC_${MC.PHASE_U_CURR_CHANNEL},
  .IbChannel       =  MC_${MC.PHASE_V_CURR_CHANNEL},
  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs               =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =  M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_U_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_V_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_W_Pin,
        <#else>   
  .pwm_en_u_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin           =  M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port          =  M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin           =  M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port          =  MC_NULL,
  .pwm_en_u_pin           = (uint16_t) 0,
  .pwm_en_v_port          =  MC_NULL,
  .pwm_en_v_pin           = (uint16_t) 0,
  .pwm_en_w_port          =  MC_NULL,
  .pwm_en_w_pin           = (uint16_t) 0,
      </#if> <#-- MC.LOW_SIDE_SIGNALS_ENABLING == ES_GPIO -->

/* Emergengy signal initialization ----------------------------------------*/
  .EmergencyStop                = (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING>ENABLE<#else>DISABLE</#if>,

}; 
  <#elseif  MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32F7 --->
    <#if MC.M1_CS_ADC_NUM=='1'> <#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC
  */
R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx = ${MC.M1_CS_ADC_U}, 
  
  .ADCConfig = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
                ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ3_Pos ) | ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
              },
                                                                                      
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,                       
  .Tafter            = TW_AFTER,                          
  .Tbefore           = TW_BEFORE_R3_1,    
  .Tsampling         = (uint16_t)SAMPLING_TIME,         
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,                   
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},               
                                     
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
<#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
<#if MC.SHARED_SIGNAL_ENABLE == false>    
  .pwm_en_u_port = M1_PWM_EN_U_GPIO_Port,                                 
  .pwm_en_u_pin  = M1_PWM_EN_U_Pin,                    
  .pwm_en_v_port = M1_PWM_EN_V_GPIO_Port,                   
  .pwm_en_v_pin  = M1_PWM_EN_V_Pin,                    
  .pwm_en_w_port = M1_PWM_EN_W_GPIO_Port,                   
  .pwm_en_w_pin  = M1_PWM_EN_W_Pin,   
<#else>
  .pwm_en_u_port = M1_PWM_EN_UVW_GPIO_Port,                                 
  .pwm_en_u_pin  = M1_PWM_EN_UVW_Pin,                    
  .pwm_en_v_port = M1_PWM_EN_UVW_GPIO_Port,                   
  .pwm_en_v_pin  = M1_PWM_EN_UVW_Pin,                    
  .pwm_en_w_port = M1_PWM_EN_UVW_GPIO_Port,                   
  .pwm_en_w_pin  = M1_PWM_EN_UVW_Pin, 
    </#if>
<#else> 
  .pwm_en_u_port = MC_NULL,                                 
  .pwm_en_u_pin  = (uint16_t) 0,                    
  .pwm_en_v_port = MC_NULL,                   
  .pwm_en_v_pin  = (uint16_t) 0,                    
  .pwm_en_w_port = MC_NULL,                   
  .pwm_en_w_pin  = (uint16_t) 0,  
</#if>

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .EmergencyStop = ${MC.BKIN2_MODE},                                              
                                     
};  
    <#elseif MC.M1_CS_ADC_NUM=='2'> <#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .Tw                       =	MAX_TWAIT,
  .bFreqRatio               =	FREQ_RATIO,          
  .bIsHigherFreqTim         =	FREQ_RELATION,       
                                                     
/* Current reading A/D Conversions initialization ----------------------------*/
 .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
 .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
   
  .ADCConfig1 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
              },
  .ADCConfig2 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
                  ( uint32_t )( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ4_Pos ) | 0<<ADC_JSQR_JL_Pos,
              },                   
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },
                  
/* PWM generation parameters --------------------------------------------------*/
  .TIMx                       =	${_last_word(MC.PWM_TIMER_SELECTION)},
  .RepetitionCounter         =	REP_COUNTER,        
  .hTafter                    =	TW_AFTER,           
  .hTbefore                   =	TW_BEFORE,      
  .Tsampling         = (uint16_t)SAMPLING_TIME,         
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,  
                                                    
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs             =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
<#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> 
<#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port          =	M1_PWM_EN_U_GPIO_Port,              
  .pwm_en_u_pin           =	M1_PWM_EN_U_Pin,                    
  .pwm_en_v_port          =	M1_PWM_EN_V_GPIO_Port,              
  .pwm_en_v_pin           =	M1_PWM_EN_V_Pin,                    
  .pwm_en_w_port          =	M1_PWM_EN_W_GPIO_Port,              
  .pwm_en_w_pin           =	M1_PWM_EN_W_Pin,  
<#else>
  .pwm_en_u_port          =	M1_PWM_EN_UVW_GPIO_Port,              
  .pwm_en_u_pin           =	M1_PWM_EN_UVW_Pin,                    
  .pwm_en_v_port          =	M1_PWM_EN_UVW_GPIO_Port,              
  .pwm_en_v_pin           =	M1_PWM_EN_UVW_Pin,                    
  .pwm_en_w_port          =	M1_PWM_EN_UVW_GPIO_Port,              
  .pwm_en_w_pin           =	M1_PWM_EN_UVW_Pin,  
</#if>                    
</#if>

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =	(FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING>ENABLE<#else>DISABLE</#if>,    
};
    </#if> 
  </#if>

  </#if> <#-- CondFamily_STM32F7 --->

  <#if CondFamily_STM32H7 > <#-- CondFamily_STM32H7 --->
    <#if MC.SINGLE_SHUNT > 
#error " H7 Single shunt not supported yet "
    <#elseif MC.ICS_SENSORS > <#-- Inside CondFamily_STM32H7 --->
#error " H7 ICS not supported yet "

  <#elseif MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32H7 --->
    <#if MC.M1_CS_ADC_NUM=='2'> 
      <#if MC.USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - H7 
  */
const R3_2_OPAMPParams_t R3_2_OPAMPParamsM1 =
{
  .OPAMPx_1 = ${MC.OPAMP1_SELECTION},
  .OPAMPx_2 = ${MC.OPAMP2_SELECTION},
  .OPAMPConfig1 = { ${OPAMPPhase_Input (1,"INPUT_1")} 
                   ,${OPAMPPhase_Input (2,"INPUT_1")} 
                   ,${OPAMPPhase_Input (3,"INPUT_1")} 
                   ,${OPAMPPhase_Input (4,"INPUT_1")} 
                   ,${OPAMPPhase_Input (5,"INPUT_1")} 
                   ,${OPAMPPhase_Input (6,"INPUT_1")}  
                 }, 
  .OPAMPConfig2 = { ${OPAMPPhase_Input (1,"INPUT_2")}   
                   ,${OPAMPPhase_Input (2,"INPUT_2")} 
                   ,${OPAMPPhase_Input (3,"INPUT_2")} 
                   ,${OPAMPPhase_Input (4,"INPUT_2")}
                   ,${OPAMPPhase_Input (5,"INPUT_2")}
                   ,${OPAMPPhase_Input (6,"INPUT_2")}
                  },                    
};
      </#if>

  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - H7 - Shared Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
  .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
  
  .ADCConfig1 = { ( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
              },
  .ADCConfig2 = { ( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
              },                   
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },
                  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false > 
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_2_OPAMPParamsM1,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else>  
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF},
};
    <#elseif MC.M1_CS_ADC_NUM=='1'> <#-- Inside CondFamily_STM32H7 --->
    #error " H7 Single ADC not supported yet"
    </#if>
    </#if>
  </#if> <#-- CondFamily_STM32H7 --->
  
  <#if CondFamily_STM32F3 > <#-- CondFamily_STM32F3 --->
    <#if MC.ICS_SENSORS > <#-- Inside CondFamily_STM32F3 --->
const ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.ADC_1_PERIPH},
  .ADCx_2 = ${MC.ADC_2_PERIPH},
  .IaChannel = MC_${MC.PHASE_U_CURR_CHANNEL},
  .IbChannel = MC_${MC.PHASE_V_CURR_CHANNEL},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >  
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
  .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
  .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port      = MC_NULL,
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
      </#if> 

/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = ${MC.BKIN2_MODE},

};

  <#elseif MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32F3 --->
    <#if MC.M1_CS_ADC_NUM=='2'> 
      <#if MC.USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - F3xx 
  */
const R3_2_OPAMPParams_t R3_2_OPAMPParamsM1 =
{
  .OPAMPx_1 = ${MC.OPAMP1_SELECTION},
  .OPAMPx_2 = ${MC.OPAMP2_SELECTION},
  .OPAMPConfig1 = { ${getOPAMP ("CFG",1,"PHASE_1")} 
                   ,${getOPAMP ("CFG",2,"PHASE_1")} 
                   ,${getOPAMP ("CFG",3,"PHASE_1")} 
                   ,${getOPAMP ("CFG",4,"PHASE_1")} 
                   ,${getOPAMP ("CFG",5,"PHASE_1")} 
                   ,${getOPAMP ("CFG",6,"PHASE_1")}  
                 }, 
  .OPAMPConfig2 = { ${getOPAMP ("CFG",1,"PHASE_2")}   
                   ,${getOPAMP ("CFG",2,"PHASE_2")} 
                   ,${getOPAMP ("CFG",3,"PHASE_2")} 
                   ,${getOPAMP ("CFG",4,"PHASE_2")}
                   ,${getOPAMP ("CFG",5,"PHASE_2")}
                   ,${getOPAMP ("CFG",6,"PHASE_2")}
                  },                    
};
      </#if>
  
  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - F30x - Shared Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
 .ADCx_1 = ${ADC1Set_M1},                 
 .ADCx_2 = ${ADC2Set_M1},
  
  .ADCConfig1 = { ( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
              },
  .ADCConfig2 = { ( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
             , ( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
              },                   
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false > 
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_2_OPAMPParamsM1,
      <#else>  
  .OPAMPParams     = MC_NULL,
      </#if>  
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else>
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF},
};
  <#elseif MC.M1_CS_ADC_NUM=='1'> <#-- Inside CondFamily_STM32F3 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC (STM32F302x8)
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx           = ${MC.M1_CS_ADC_U},
  .ADCConfig = { ( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )
             , ( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",2,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )
             , ( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",3,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )
             , ( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )
             , ( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )
             , ( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ2_Pos ) | 1<<ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT )                       
               },

 
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE_R3_1,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false >
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE},
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},  
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else>  
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF},
};
    </#if>
    </#if>
    <#if MC.ICS_SENSORS2 == true>  <#-- Inside CondFamily_STM32F3 --->
ICS_Params_t ICS_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.ADC_1_PERIPH2},
  .ADCx_2 = ${MC.ADC_2_PERIPH2},
  .IaChannel       =  MC_${MC.PHASE_U_CURR_CHANNEL2},
  .IbChannel       =  MC_${MC.PHASE_V_CURR_CHANNEL2},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER2,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION2)},
  
/* PWM Driving signals initialization ----------------------------------------*/

  .LowSideOutputs     = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >   
        <#if MC.SHARED_SIGNAL_ENABLE2 == false>
  .pwm_en_u_port      = M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_U_Pin,
  .pwm_en_v_port      = M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_V_Pin,
  .pwm_en_w_port      = M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_W_Pin, 
        <#else>
  .pwm_en_u_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port      = MC_NULL,
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
      </#if> 
/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = ${MC.BKIN2_MODE2},
};
  <#elseif MC.THREE_SHUNT_SHARED_RESOURCES2 || MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 > <#-- Inside CondFamily_STM32F3 --->
     <#if MC.USE_INTERNAL_OPAMP2>
/**
  * @brief  Internal OPAMP parameters Motor 2 - three shunt - F3xx 
  */
R3_2_OPAMPParams_t R3_2_OPAMPParamsM2 =
{
  .OPAMPx_1 = ${MC.OPAMP1_SELECTION2},
  .OPAMPx_2 = ${MC.OPAMP2_SELECTION2},
  .OPAMPConfig1 = { ${OPAMPPhase_Input (1,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (2,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (3,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (4,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (5,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (6,"INPUT_1",2)}  
                 }, 
  .OPAMPConfig2 = { ${OPAMPPhase_Input (1,"INPUT_2",2)}   
                   ,${OPAMPPhase_Input (2,"INPUT_2",2)} 
                   ,${OPAMPPhase_Input (3,"INPUT_2",2)} 
                   ,${OPAMPPhase_Input (4,"INPUT_2",2)}
                   ,${OPAMPPhase_Input (5,"INPUT_2",2)}
                   ,${OPAMPPhase_Input (6,"INPUT_2",2)}
                  },                    
};
    </#if>
  
  /**
  * @brief  Current sensor parameters Motor 2 - three shunt - F30x 
  */
const R3_2_Params_t R3_2_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ${MC.ADC_1_PERIPH2},
  .ADCx_2           = ${MC.ADC_2_PERIPH2},

  .ADCConfig1 = {${ADCConfig(1,"Sampling_1",2)}
                ,${ADCConfig(2,"Sampling_1",2)}
                ,${ADCConfig(3,"Sampling_1",2)}
                ,${ADCConfig(4,"Sampling_1",2)}
                ,${ADCConfig(5,"Sampling_1",2)}
                ,${ADCConfig(6,"Sampling_1",2)}
                },
  .ADCConfig2 = {${ADCConfig(1,"Sampling_2",2)}
                ,${ADCConfig(2,"Sampling_2",2)}
                ,${ADCConfig(3,"Sampling_2",2)}
                ,${ADCConfig(4,"Sampling_2",2)}
                ,${ADCConfig(5,"Sampling_2",2)}
                ,${ADCConfig(6,"Sampling_2",2)} 
                },
  .ADCDataReg1 = { ${ADCDataRead(1,"Sampling_1",2)}
                 , ${ADCDataRead(2,"Sampling_1",2)}
                 , ${ADCDataRead(3,"Sampling_1",2)}
                 , ${ADCDataRead(4,"Sampling_1",2)}
                 , ${ADCDataRead(5,"Sampling_1",2)}
                 , ${ADCDataRead(6,"Sampling_1",2)}
                 },
  .ADCDataReg2 =  { ${ADCDataRead(1,"Sampling_2",2)}
                  , ${ADCDataRead(2,"Sampling_2",2)}
                  , ${ADCDataRead(3,"Sampling_2",2)}
                  , ${ADCDataRead(4,"Sampling_2",2)}
                  , ${ADCDataRead(5,"Sampling_2",2)}
                  , ${ADCDataRead(6,"Sampling_2",2)}
                  },
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .Tafter            = TW_AFTER2,
  .Tbefore           = TW_BEFORE2,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION2)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >  
        <#if MC.SHARED_SIGNAL_ENABLE2 == false >
 .pwm_en_u_port      = M2_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M2_PWM_EN_U_Pin,
 .pwm_en_v_port      = M2_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M2_PWM_EN_V_Pin,
 .pwm_en_w_port      = M2_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M2_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M2_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M2_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M2_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE2},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP2>
  .OPAMPParams     = &R3_2_OPAMPParamsM2,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>
/* Internal COMP settings ----------------------------------------------------*/
<#if MC.M2_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
<#else>  
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
</#if>

<#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection      = OVP_SELECTION2,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE2,
<#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
</#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF2},
};

    <#elseif MC.THREE_SHUNT2>  <#-- Inside CondFamily_STM32F3 --->
  <#-- provision for future 
extern R3_1_F30XParams_t R3_1_F30XParamsM2;
  -->
    </#if>
  </#if> <#-- CondFamily_STM32F3 --->

  <#if CondFamily_STM32G4 >
    <#if MC.THREE_SHUNT_INDEPENDENT_RESOURCES2 ||MC.THREE_SHUNT_SHARED_RESOURCES2  > 
      <#if MC.USE_INTERNAL_OPAMP2>
R3_3_OPAMPParams_t R3_3_OPAMPParamsM2 =  
{ 
  .OPAMPx_1 = ${MC.OPAMP1_SELECTION2},
  .OPAMPx_2 = ${MC.OPAMP2_SELECTION2},
  .OPAMPx_3 = MC_NULL,
  .OPAMPSelect_1 = { ${MC.OPAMP1_SELECTION2}
                    ,${MC.OPAMP1_SELECTION2} 
                    ,${MC.OPAMP1_SELECTION2} 
                    ,${MC.OPAMP1_SELECTION2} 
                    ,${MC.OPAMP1_SELECTION2} 
                    ,${MC.OPAMP1_SELECTION2} 
               },               
  .OPAMPSelect_2 = { ${MC.OPAMP2_SELECTION2} 
                    ,${MC.OPAMP2_SELECTION2}  
                    ,${MC.OPAMP2_SELECTION2}  
                    ,${MC.OPAMP2_SELECTION2}  
                    ,${MC.OPAMP2_SELECTION2}  
                    ,${MC.OPAMP2_SELECTION2}  
               },                        
  .OPAMPConfig1 = { ${OPAMPPhase_Input (1,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (2,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (3,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (4,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (5,"INPUT_1",2)} 
                   ,${OPAMPPhase_Input (6,"INPUT_1",2)}  
                 }, 
  .OPAMPConfig2 = { ${OPAMPPhase_Input (1,"INPUT_2",2)}   
                   ,${OPAMPPhase_Input (2,"INPUT_2",2)} 
                   ,${OPAMPPhase_Input (3,"INPUT_2",2)} 
                   ,${OPAMPPhase_Input (4,"INPUT_2",2)}
                   ,${OPAMPPhase_Input (5,"INPUT_2",2)}
                   ,${OPAMPPhase_Input (6,"INPUT_2",2)}
                  },
};
      </#if>
/**
  * @brief  Current sensor parameters Motor 2 - three shunt - G4 
  */
R3_2_Params_t R3_2_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx1=<#if MC.M2_CS_ADC_PHASE_SHARED!="R">${MC.M2_CS_ADC_R}<#else>${MC.M2_CS_ADC_S}</#if>,
  .ADCx2=<#if MC.M2_CS_ADC_PHASE_SHARED!="T">${MC.M2_CS_ADC_T}<#else>${MC.M2_CS_ADC_S}</#if>,  
  
  /* Motor Control Kit config */ 
   .ADCConfig1 = { ( uint32_t )( ${ADCConfig(1,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(2,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(3,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(4,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(5,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(6,"Sampling_1",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },
   .ADCConfig2 = { ( uint32_t )( ${ADCConfig(1,"Sampling_2",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(2,"Sampling_2",2)}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(3,"Sampling_2",2)}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(4,"Sampling_2",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(5,"Sampling_2",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${ADCConfig(6,"Sampling_2",2)}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },              
  .ADCDataReg1 = { ${ADCDataRead(1,"Sampling_1",2)}
                 , ${ADCDataRead(2,"Sampling_1",2)}
                 , ${ADCDataRead(3,"Sampling_1",2)}
                 , ${ADCDataRead(4,"Sampling_1",2)}
                 , ${ADCDataRead(5,"Sampling_1",2)}
                 , ${ADCDataRead(6,"Sampling_1",2)}
                 },
  .ADCDataReg2 = { ${ADCDataRead(1,"Sampling_2",2)}
                 , ${ADCDataRead(2,"Sampling_2",2)}
                 , ${ADCDataRead(3,"Sampling_2",2)}
                 , ${ADCDataRead(4,"Sampling_2",2)}
                 , ${ADCDataRead(5,"Sampling_2",2)}
                 , ${ADCDataRead(6,"Sampling_2",2)}
                 },    
    /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .Tafter            = TW_AFTER2,
  .Tbefore           = TW_BEFORE2,
  .Tsampling         = (uint16_t)SAMPLING_TIME2,
  .Tcase2            = (uint16_t)SAMPLING_TIME2 + (uint16_t)TDEAD2 + (uint16_t)TRISE2,
  .Tcase3            = ((uint16_t)TDEAD2 + (uint16_t)TNOISE2 + (uint16_t)SAMPLING_TIME2)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION2)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE2 == false > 
 .pwm_en_u_port      = M2_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M2_PWM_EN_U_Pin,
 .pwm_en_v_port      = M2_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M2_PWM_EN_V_Pin,
 .pwm_en_w_port      = M2_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M2_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M2_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M2_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M2_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M2_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE2},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP2>
  .OPAMPParams     = &R3_3_OPAMPParamsM2,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>  
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M2_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
        <#if MC.M2_OCP_COMP_SRC == "DAC">  
  .DAC_OCP_ASelection    = ${MC.M2_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M2_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M2_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_W, "OUT")},
        <#else>
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
        </#if>
      <#else>
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection      = OVP_SELECTION2,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE2,  
        <#if MC.DAC_OVP_M2 != "NOT_USED">
  .DAC_OVP_Selection = ${_first_word(MC.DAC_OVP_M2)},
  .DAC_Channel_OVP = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M2, "CH")},
        <#else>
  .DAC_OVP_Selection = MC_NULL,
  .DAC_Channel_OVP = (uint32_t) 0,
        </#if>  
      <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection = MC_NULL, 
  .DAC_Channel_OVP = (uint32_t) 0,  
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF2},
}; 
    </#if>
    <#if MC.SINGLE_SHUNT2 > 
/**
  * @brief  Current sensor parameters Motor 1 - single shunt - G40x
  */
const R1_Params_t R1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2, 
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION2)}, 

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ${MC.ADC_PERIPH2},
  .IChannel        = MC_${MC.PHASE_CURRENTS_CHANNEL2},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .Tafter            = TAFTER2,
  .Tbefore           = TBEFORE2,
  .TMin              = TMIN2,
  .HTMin             = HTMIN2,
  .CHTMin            = CHTMIN2,
  .TSample           = SAMPLING_TIME2,


/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE2 == false>
  .pwm_en_u_port  = M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin   = M2_PWM_EN_U_Pin,
  .pwm_en_v_port  = M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin   = M2_PWM_EN_V_Pin,
  .pwm_en_w_port  = M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin   = M2_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port  = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin   = M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port  = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin   = M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port  = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin   = M2_PWM_EN_UVW_Pin,  
        </#if>
      <#else>
  .pwm_en_u_port  = MC_NULL,
  .pwm_en_u_pin   = (uint16_t) 0,
  .pwm_en_v_port  = MC_NULL,
  .pwm_en_v_pin   = (uint16_t) 0,
  .pwm_en_w_port  = MC_NULL,
  .pwm_en_w_pin   = (uint16_t) 0,
      </#if>

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode      = ${MC.BKIN2_MODE2},
  
/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP2>
  .OPAMP_Selection = ${_filter_opamp (MC.OPAMP_SELECTION2)},
      <#else>
  .OPAMP_Selection = MC_NULL,
      </#if>
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.INTERNAL_OVERCURRENTPROTECTION2>
  .CompOCPSelection = ${MC.OCP_SELECTION2},
  .CompOCPInvInput_MODE = ${MC.OCP_INVERTINGINPUT_MODE2},
        <#if MC.DAC_OCP_M2 != "NOT_USED">
  .DAC_OCP_Selection    = ${_first_word(MC.DAC_OCP_M2)},
  .DAC_Channel_OCP      = LL_DAC_CHANNEL_${_last_word(MC.DAC_OCP_M2, "CH")},
        <#else>
  .DAC_OCP_Selection    = MC_NULL,
  .DAC_Channel_OCP      = (uint32_t) 0,
        </#if>
      <#else>
  .CompOCPSelection = MC_NULL,
  .CompOCPInvInput_MODE = NONE,
  .DAC_OCP_Selection    = MC_NULL,
  .DAC_Channel_OCP      = (uint32_t) 0,
      </#if>
      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection = OVP_SELECTION2,
  .CompOVPInvInput_MODE = OVP_INVERTINGINPUT_MODE2,  
        <#if MC.DAC_OVP_M2 != "NOT_USED">  
  .DAC_OVP_Selection    =  ${_first_word(MC.DAC_OVP_M2)},
  .DAC_Channel_OVP      = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M2, "CH")},   
        <#else>
  .DAC_OVP_Selection    = MC_NULL,
  .DAC_Channel_OVP      = (uint32_t) 0, 
        </#if>
      <#else>
  .CompOVPSelection = MC_NULL,
  .CompOVPInvInput_MODE =  NONE,
  .DAC_OVP_Selection    = MC_NULL,  
  .DAC_Channel_OVP      = (uint32_t) 0,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF2},
};
    </#if> <#-- SINGLE_SHUNT 2 -->

    <#if MC.ICS_SENSORS2>
const ICS_Params_t ICS_ParamsM2 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.ADC_1_PERIPH2},
  .ADCx_2 = ${MC.ADC_2_PERIPH2},
      <#if MC.PWM_TIMER_SELECTION2 == "PWM_TIM1">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,  
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,     
      <#elseif MC.PWM_TIMER_SELECTION2 == "PWM_TIM8">
        <#if MC.PHASE_CURRENTS_ADC2 == "ADC1_2">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,  
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
        <#elseif  MC.PHASE_CURRENTS_ADC2 == "ADC3">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,  
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL2} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
        </#if>
      </#if> 

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER2,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION2)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE2 == false> 
  .pwm_en_u_port      = M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_U_Pin,
  .pwm_en_v_port      = M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_V_Pin,
  .pwm_en_w_port      = M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_W_Pin,   
        <#else>
  .pwm_en_u_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port      = MC_NULL,
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
      </#if> 

/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = ${MC.BKIN2_MODE2},

};
    </#if>

    <#if MC.ICS_SENSORS>
  const ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio =        FREQ_RATIO,
  .IsHigherFreqTim =    FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.ADC_1_PERIPH},
  .ADCx_2 = ${MC.ADC_2_PERIPH},
      <#if MC.PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
      <#elseif MC.PWM_TIMER_SELECTION == "PWM_TIM8">
        <#if MC.PHASE_CURRENTS_ADC == "ADC1_2">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
        <#elseif  MC.PHASE_CURRENTS_ADC == "ADC3">
  .ADCConfig1 = ( uint32_t )(MC_${MC.PHASE_U_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
  .ADCConfig2 = ( uint32_t )(MC_${MC.PHASE_V_CURR_CHANNEL} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
        </#if>
      </#if>

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter =  REP_COUNTER,
  .TIMx               =  ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs =  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
  .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
  .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
  .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
  .pwm_en_u_port      = MC_NULL,
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
      </#if> 

/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = ${MC.BKIN2_MODE},

};
    </#if>
  
  <#if MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32G4 --->
    <#if MC.M1_CS_ADC_NUM=='2'>
      <#if MC.USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx - Shared Resources
  * 
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{
   .OPAMPx_1 = ${MC.OPAMP1_SELECTION},
   .OPAMPx_2 = ${MC.OPAMP2_SELECTION},
   .OPAMPx_3 = <#if MC.M1_CS_OPAMP_NUM == "3" >OPAMP3<#else> MC_NULL</#if>,
    // OPAMPMatrix is used to specify if the ADC source comes from internal channel of which OPAMP. 
     
  .OPAMPSelect_1 = { ${getOPAMP ("IP",1,"PHASE_1")} 
                   ,${getOPAMP ("IP",2,"PHASE_1")} 
                   ,${getOPAMP ("IP",3,"PHASE_1")} 
                   ,${getOPAMP ("IP",4,"PHASE_1")} 
                   ,${getOPAMP ("IP",5,"PHASE_1")} 
                   ,${getOPAMP ("IP",6,"PHASE_1")}  
                 }, 
  .OPAMPSelect_2 = { ${getOPAMP ("IP",1,"PHASE_2")}   
                   ,${getOPAMP ("IP",2,"PHASE_2")} 
                   ,${getOPAMP ("IP",3,"PHASE_2")} 
                   ,${getOPAMP ("IP",4,"PHASE_2")}
                   ,${getOPAMP ("IP",5,"PHASE_2")}
                   ,${getOPAMP ("IP",6,"PHASE_2")}
                  }, 


  .OPAMPConfig1 = { ${getOPAMP ("CFG",1,"PHASE_1")} 
                   ,${getOPAMP ("CFG",2,"PHASE_1")} 
                   ,${getOPAMP ("CFG",3,"PHASE_1")} 
                   ,${getOPAMP ("CFG",4,"PHASE_1")} 
                   ,${getOPAMP ("CFG",5,"PHASE_1")} 
                   ,${getOPAMP ("CFG",6,"PHASE_1")}  
                 }, 
  .OPAMPConfig2 = { ${getOPAMP ("CFG",1,"PHASE_2")}   
                   ,${getOPAMP ("CFG",2,"PHASE_2")} 
                   ,${getOPAMP ("CFG",3,"PHASE_2")} 
                   ,${getOPAMP ("CFG",4,"PHASE_2")}
                   ,${getOPAMP ("CFG",5,"PHASE_2")}
                   ,${getOPAMP ("CFG",6,"PHASE_2")}
                  },   
};
      </#if> 
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
  .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
  /* Motor Control Kit config */ 
   .ADCConfig1 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },
   .ADCConfig2 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",2,"PHASE_2")}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",3,"PHASE_2")}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },              
 
   .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                  },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
      <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
        <#if MC.SHARED_SIGNAL_ENABLE == false > 
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port, 
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
        <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
        </#if>
      <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
      </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>  
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
        <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
        <#else>
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,  
        </#if> 
      <#else>
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
        <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
        <#else>
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
        </#if>
        <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF},

};
  </#if> <#-- <#if MC.M1_CS_ADC_NUM=='2' --> <#-- Inside CondFamily_STM32G4 -->   
 </#if> <#-- MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT' --> <#-- Inside CondFamily_STM32G4 --> 
  </#if><#-- CondFamily_STM32G4 -->

  <#if CondFamily_STM32G0 || CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32F0 || CondFamily_STM32G4>
<#assign currentFactor = "">
  <#elseif CondFamily_STM32F4 || CondFamily_STM32F7>
<#assign currentFactor = "*2">
  </#if>
  <#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32F4 ||  CondFamily_STM32F7 || CondFamily_STM32G4>
<#assign HAS_ADC_INJ = true>
  <#else>
<#assign HAS_ADC_INJ = false>
  </#if>
  <#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G0 || CondFamily_STM32F7 || CondFamily_STM32G4>
<#assign HAS_TIM_6_CH = true>
  <#else>
<#assign HAS_TIM_6_CH = false>
  </#if>
  <#if CondFamily_STM32F4 || CondFamily_STM32F7>
<#assign DMA_TYPE = "Stream">
  <#else>
<#assign DMA_TYPE = "Channel">
  </#if>
  <#if CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4 || CondFamily_STM32F7>
<#assign HAS_BRK2 = true>
  <#else>
<#assign HAS_BRK2 = false>
  </#if>

  <#if CondFamily_STM32F0 == true> 
<#assign R1_DMA_M1 = "DMA1">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_5">
<#assign R1_DMA_CH_AUX_M1 = "LL_DMA_CHANNEL_4">  
<#assign R1_DMA_CH_ADC_M1 = "LL_DMA_CHANNEL_1">
  </#if>
  <#if CondFamily_STM32G0 == true> 
<#assign R1_DMA_M1 = "DMA1">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_4">
<#assign R1_DMA_CH_ADC_M1 = "LL_DMA_CHANNEL_1">
  </#if>  
  <#if CondFamily_STM32F3 == true> 
    <#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION == 'TIM1'>
<#assign R1_DMA_M1 = "DMA1">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_4"> 
    <#elseif MC.PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION == 'TIM8'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_2">  
    </#if>
  </#if>  
  <#if CondFamily_STM32G4 == true> 
    <#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION == 'TIM1'>
<#assign R1_DMA_M1 = "DMA1">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_1"> 
    <#elseif MC.PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION == 'TIM8'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_1">  
    </#if>
  </#if> 
  <#if CondFamily_STM32L4 == true> 
<#assign R1_DMA_STATUS_REG_M1 = "ISR">  
<#assign R1_DMA_CLEAR_REG_M1 = "IFCR"> 
    <#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION == 'TIM1'>
<#assign R1_DMA_M1 = "DMA1">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_4"> 
    <#elseif MC.PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION == 'TIM8'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_CHANNEL_2">  
    </#if>
  </#if> 
  <#if CondFamily_STM32F4 == true> 
    <#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION == 'TIM1'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_STREAM_5">
<#assign R1_DMA_CH_AUX_M1 = "LL_DMA_STREAM_4">   
    <#elseif MC.PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION == 'TIM8'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_STREAM_1">
<#assign R1_DMA_CH_AUX_M1 = "LL_DMA_STREAM_7">    
    </#if>  
  </#if>    
  <#if CondFamily_STM32F7 == true> 
    <#if MC.PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION == 'TIM1'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_STREAM_4">    
    <#elseif MC.PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION == 'TIM8'>
<#assign R1_DMA_M1 = "DMA2">
<#assign R1_DMA_CH_M1 = "LL_DMA_STREAM_7">    
    </#if>  
  </#if>
  <#if MC.DUALDRIVE == true> 
    <#if CondFamily_STM32F3 == true> 
      <#if MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION2 == 'TIM1'>
<#assign R1_DMA_M2 = "DMA1">
<#assign R1_DMA_CH_M2 = "LL_DMA_CHANNEL_5"> 
      <#elseif MC.PWM_TIMER_SELECTION2 == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION2 == 'TIM8'>
<#assign R1_DMA_M2 = "DMA2">
<#assign R1_DMA_CH_M2 = "LL_DMA_CHANNEL_1"> 
      </#if>
    </#if>  
    <#if CondFamily_STM32G4 == true> 
      <#if MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION2 == 'TIM1'>
<#assign R1_DMA_M2 = "DMA1">
<#assign R1_DMA_CH_M2 = "LL_DMA_CHANNEL_1">  
      <#elseif MC.PWM_TIMER_SELECTION2 == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION2 == 'TIM8'>
<#assign R1_DMA_M2 = "DMA2">
<#assign R1_DMA_CH_M2 = "LL_DMA_CHANNEL_1"> 
      </#if>
    </#if> 
    <#if CondFamily_STM32F4 == true> 
      <#if MC.PWM_TIMER_SELECTION2 == 'PWM_TIM1' || MC.PWM_TIMER_SELECTION2 == 'TIM1'>
<#assign R1_DMA_M2 = "DMA2">
<#assign R1_DMA_CH_M2 = "LL_DMA_STREAM_5">
<#assign R1_DMA_CH_AUX_M2 = "LL_DMA_STREAM_4">   
      <#elseif MC.PWM_TIMER_SELECTION2 == 'PWM_TIM8' || MC.PWM_TIMER_SELECTION2 == 'TIM8'>
<#assign R1_DMA_M2 = "DMA2">
<#assign R1_DMA_CH_M2 = "LL_DMA_STREAM_1">
<#assign R1_DMA_CH_AUX_M2 = "LL_DMA_STREAM_7">    
      </#if>  
    </#if>
  </#if>
<#-- one shunt pahse shift********************************* --> 
  <#if MC.SINGLE_SHUNT == true >
/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const R1_Params_t R1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION, 

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ${MC.ADC_PERIPH},
  .IChannel       = MC_${MC.PHASE_CURRENTS_CHANNEL},
    <#if CondFamily_STM32F0 || CondFamily_STM32G0  >
  .ISamplingTime = LL_ADC_SAMPLINGTIME_${MC.CURR_SAMPLING_TIME}<#if MC.CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,
    </#if>
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = ${MC.REGULATION_EXECUTION_RATE},
  .TMin               = TMIN,
  .TSample            = (uint16_t)(SAMPLING_TIME + TBEFORE),
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},
  .DMAx               = ${R1_DMA_M1},
  .DMAChannelX        = ${R1_DMA_CH_M1},
    <#if HAS_TIM_6_CH == false>   
  .DMASamplingPtChannelX = ${R1_DMA_CH_AUX_M1},
    </#if>
    <#if HAS_ADC_INJ == false>
  .DMA_ADC_DR_ChannelX = ${R1_DMA_CH_ADC_M1},
    </#if> 
  .hTADConv          = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz/ADC_CLK_MHz)),
<#-- ADC_TRIG_CONV_LATENCY_CYCLES = 40 for G4 TODO -->
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
    <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
      <#if MC.SHARED_SIGNAL_ENABLE == false>
  .pwm_en_u_port  = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin   = M1_PWM_EN_U_Pin,
  .pwm_en_v_port  = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin   = M1_PWM_EN_V_Pin,
  .pwm_en_w_port  = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin   = M1_PWM_EN_W_Pin,
      <#else>
  .pwm_en_u_port  = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin   = M1_PWM_EN_UVW_Pin,
  .pwm_en_v_port  = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin   = M1_PWM_EN_UVW_Pin,
  .pwm_en_w_port  = M1_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin   = M1_PWM_EN_UVW_Pin,
      </#if>
    <#else>
  .pwm_en_u_port  = MC_NULL,
  .pwm_en_u_pin   = (uint16_t) 0,
  .pwm_en_v_port  = MC_NULL,
  .pwm_en_v_pin   = (uint16_t) 0,
  .pwm_en_w_port  = MC_NULL,
  .pwm_en_w_pin   = (uint16_t) 0,
    </#if>
    <#if HAS_BRK2 == true>
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode      = ${MC.BKIN2_MODE},
    <#else>
 .EmergencyStop = (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING==true>ENABLE<#else>DISABLE</#if>,
    </#if> 
/* Internal OPAMP common settings --------------------------------------------*/
    <#if MC.USE_INTERNAL_OPAMP>
  .OPAMP_Selection = ${_filter_opamp (MC.OPAMP_SELECTION)},
    </#if>
/* Internal COMP settings ----------------------------------------------------*/
    <#if MC.INTERNAL_OVERCURRENTPROTECTION>
  .CompOCPSelection = ${MC.OCP_SELECTION},            
  .CompOCPInvInput_MODE = ${MC.OCP_INVERTINGINPUT_MODE},
    </#if>
    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection = OVP_SELECTION, 
  .CompOVPInvInput_MODE = OVP_INVERTINGINPUT_MODE,
    </#if>

/* DAC settings --------------------------------------------------------------*/
    <#if MC.INTERNAL_OVERCURRENTPROTECTION>
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},  
    </#if>
    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .DAC_OVP_Threshold =  ${MC.OVPREF},
    </#if>

};  
  </#if>
  <#if MC.SINGLE_SHUNT2 == true >
 /*
  * @brief  Current sensor parameters Motor 2 - single shunt - F30x
  */
const R1_Params_t R1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2,  

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ${MC.ADC_PERIPH2},
  .IChannel       = MC_${MC.PHASE_CURRENTS_CHANNEL2},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = ${MC.REGULATION_EXECUTION_RATE2},
  .TMin              = TMIN2,
  .TSample           = (uint16_t)SAMPLING_TIME2,
  .TIMx              = ${_last_word(MC.PWM_TIMER_SELECTION2)},
  .DMAx              = ${R1_DMA_M2},
  .DMAChannelX       = ${R1_DMA_CH_M2},
    <#if HAS_TIM_6_CH == false>
  .DMASamplingPtChannelX = ${R1_DMA_CH_AUX_M2},
    </#if>
  .hTADConv          = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz2/ADC_CLK_MHz2)),

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,
    <#if MC.LOW_SIDE_SIGNALS_ENABLING2 == "ES_GPIO" >
      <#if MC.SHARED_SIGNAL_ENABLE2 == false>
  .pwm_en_u_port      = M2_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_U_Pin,
  .pwm_en_v_port      = M2_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_V_Pin,
  .pwm_en_w_port      = M2_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_W_Pin, 
      <#else>
  .pwm_en_u_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_u_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_v_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_v_pin       = M2_PWM_EN_UVW_Pin,
  .pwm_en_w_port      = M2_PWM_EN_UVW_GPIO_Port,
  .pwm_en_w_pin       = M2_PWM_EN_UVW_Pin, 
      </#if>
    <#else>
  .pwm_en_u_port      = MC_NULL,
  .pwm_en_u_pin       = (uint16_t) 0,
  .pwm_en_v_port      = MC_NULL,
  .pwm_en_v_pin       = (uint16_t) 0,
  .pwm_en_w_port      = MC_NULL,
  .pwm_en_w_pin       = (uint16_t) 0, 
    </#if>
    <#if HAS_BRK2 == true >
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode      = ${MC.BKIN2_MODE2},
    <#else>
 .EmergencyStop = (FunctionalState) <#if MC.SW_OV_CURRENT_PROT_ENABLING2==true>ENABLE<#else>DISABLE</#if>,
    </#if> 
/* Internal OPAMP common settings --------------------------------------------*/
    <#if MC.USE_INTERNAL_OPAMP2>
  .OPAMP_Selection = ${_filter_opamp (MC.OPAMP_SELECTION2)},
    </#if>
/* Internal COMP settings ----------------------------------------------------*/
    <#if MC.INTERNAL_OVERCURRENTPROTECTION2>
  .CompOCPSelection = ${MC.OCP_SELECTION2},
  .CompOCPInvInput_MODE = ${MC.OCP_INVERTINGINPUT_MODE2},
    </#if>
    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection = OVP_SELECTION2,
  .CompOVPInvInput_MODE = OVP_INVERTINGINPUT_MODE2,
    </#if>

/* DAC settings --------------------------------------------------------------*/
    <#if MC.INTERNAL_OVERCURRENTPROTECTION2>
  .DAC_OCP_Threshold =  ${MC.M2_DAC_CURRENT_THRESHOLD},
    </#if>
    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .DAC_OVP_Threshold =  ${MC.OVPREF2},  
    </#if>

};
  </#if>
<#-- one shunt pahse shift********************************* -->

  <#if MC.PFC_ENABLED == true>
const PFC_Parameters_t PFC_Params = 
{

    <#if CondFamily_STM32F3>
  .ADCx                     = ADC4,
  .TIMx                     = TIM16,
  .TIMx_2                   = TIM4,
    </#if>
  .wCPUFreq                 = SYSCLK_FREQ,
  .wPWMFreq                 = PFC_PWMFREQ,
  .hPWMARR                  = (SYSCLK_FREQ/PFC_PWMFREQ),
  .bFaultPolarity           = PFC_FAULTPOLARITY,
  .hFaultPort               = PFC_FAULTPORT,
  .hFaultPin                = PFC_FAULTPIN,
  .bCurrentLoopRate         = (uint8_t) (PFC_PWMFREQ/PFC_CURRCTRLFREQUENCY),
  .bVoltageLoopRate         = (uint8_t) (SYS_TICK_FREQUENCY/PFC_VOLTCTRLFREQUENCY),
  .hNominalCurrent          = (uint16_t) PFC_NOMINALCURRENTS16A,
  .hMainsFreqLowTh          = PFC_MAINSFREQLOWTHR,
  .hMainsFreqHiTh           = PFC_MAINSFREQHITHR,
  .OutputPowerActivation    = PFC_OUTPUTPOWERACTIVATION,
  .OutputPowerDeActivation  = PFC_OUTPUTPOWERDEACTIVATION,
  .SWOverVoltageTh          = PFC_SWOVERVOLTAGETH,
  .hPropDelayOnTimCk        = (uint16_t) (PFC_PROPDELAYON/PFC_TIMCK_NS),
  .hPropDelayOffTimCk       = (uint16_t) (PFC_PROPDELAYOFF/PFC_TIMCK_NS),
  .hADCSamplingTimeTimCk    = (uint16_t) (SYSCLK_FREQ/(ADC_CLK_MHz*1000000.0)*PFC_ISAMPLINGTIMEREAL),
  .hADCConversionTimeTimCk  = (uint16_t) (SYSCLK_FREQ/(ADC_CLK_MHz*1000000.0)*12),
  .hADCLatencyTimeTimCk     = (uint16_t) (SYSCLK_FREQ/(ADC_CLK_MHz*1000000.0)*3),
  .hMainsConversionFactor   = (uint16_t) (65535.0/(3.3 * PFC_VMAINS_PARTITIONING_FACTOR)),
  .hCurrentConversionFactor = (uint16_t) ((PFC_SHUNTRESISTOR*PFC_AMPLGAIN*65536.0)/3.3),
    <#if CondFamily_STM32F3>
  .hDAC_OCP_Threshold       = (uint16_t) PFC_OCP_REF,
    </#if>
};
  </#if>

  <#if MC.MOTOR_PROFILER == true>

/*** Motor Profiler ***/

SCC_Params_t SCC_Params = 
{
  {
    .FrequencyHz = TF_REGULATION_RATE,
  },
  .fRshunt = RSHUNT,   
  .fAmplificationGain = AMPLIFICATION_GAIN, 
  .fVbusConvFactor = BUS_VOLTAGE_CONVERSION_FACTOR,
  .fVbusPartitioningFactor = VBUS_PARTITIONING_FACTOR, 
  
  .fRVNK = (float)(CALIBRATION_FACTOR),

  .fRSMeasCurrLevelMax = (float)(DC_CURRENT_RS_MEAS),
  
  .hDutyRampDuration  = (uint16_t) 8000,
  
  .hAlignmentDuration = (uint16_t)(1000),
  .hRSDetectionDuration = (uint16_t)(500),
  .fLdLqRatio = (float)(LDLQ_RATIO),
  .fCurrentBW = (float)(CURRENT_REGULATOR_BANDWIDTH),
  .bPBCharacterization = false,
                      
  .wNominalSpeed = MOTOR_MAX_SPEED_RPM, 
  .hPWMFreqHz = (uint16_t)(PWM_FREQUENCY), 
  .bFOCRepRate = (uint8_t)(REGULATION_EXECUTION_RATE), 
  .fMCUPowerSupply = (float) ADC_REFERENCE_VOLTAGE,  
  .IThreshold = I_THRESHOLD

};

OTT_Params_t OTT_Params =
{
  {
    .FrequencyHz = MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency expressed in Hz at which the user
                                   clocks the OTT calling OTT_MF method */
  },
  .fBWdef = (float)(SPEED_REGULATOR_BANDWIDTH),/*!< Default bandwidth of speed regulator.*/
  .fMeasWin = 1.0f,                       /*!< Duration of measurement window for speed and
                                   current Iq, expressed in seconds.*/
  .bPolesPairs = POLE_PAIR_NUM,              /*!< Number of motor poles pairs.*/
  .hMaxPositiveTorque = (int16_t)NOMINAL_CURRENT,   /*!< Maximum positive value of motor
                                   torque. This value represents
                                   actually the maximum Iq current
                                   expressed in digit.*/
  .fCurrtRegStabTimeSec = 10.0f,                      /*!< Current regulation stabilization time in seconds.*/
  .fOttLowSpeedPerc = 0.6f,                       /*!< OTT lower speed percentage.*/
  .fOttHighSpeedPerc = 0.8f,                       /*!< OTT higher speed percentage.*/
  .fSpeedStabTimeSec = 20.0f,                      /*!< Speed stabilization time in seconds.*/
  .fTimeOutSec = 10.0f,                      /*!< Timeout for speed stabilization.*/
  .fSpeedMargin = 0.90f,                      /*!< Speed margin percentage to validate speed ctrl.*/
  .wNominalSpeed = MOTOR_MAX_SPEED_RPM,        /*!< Nominal speed set by the user expressed in RPM.*/
  .spdKp = MP_KP,                      /*!< Initial KP factor of the speed regulator to be tuned.*/
  .spdKi = MP_KI,                      /*!< Initial KI factor of the speed regulator to be tuned.*/
  .spdKs = 0.1f,                       /*!< Initial antiwindup factor of the speed regulator to be tuned.*/
  .fRshunt = (float)(RSHUNT),            /*!< Value of shunt resistor.*/
  .fAmplificationGain = (float)(AMPLIFICATION_GAIN) /*!< Current sensing amplification gain.*/

};
  </#if>
</#if><#-- DRIVE_TYPE == "FOC" -->
<#if MC.DRIVE_TYPE == "SIX_STEP">
<#if MC.STSPIN32G4 == false >
  <#if configs[0].peripheralParams.get(_last_word(MC.PWM_TIMER_SELECTION))??>
<#assign PWMTIM = configs[0].peripheralParams.get(_last_word(MC.PWM_TIMER_SELECTION))>
  </#if>
  <#if !PWMTIM??>
#error SORRY, it didn't work
</#if>
  </#if>
  <#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#include "pwmc_6pwm.h"
  <#else>
#include "pwmc_3pwm.h"
  </#if>
  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
  <#if CondFamily_STM32F0>
#include "f0xx_bemf_ADC_fdbk.h"
    </#if>
  <#if CondFamily_STM32G4>
#include "g4xx_bemf_ADC_fdbk.h"
    </#if>
  </#if>
  <#if  MC.DRIVE_MODE == "CM">
#include "current_ref_ctrl.h"
  </#if>
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
  <#if  MC.LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
const SixPwm_Params_t SixPwm_ParamsM1 =
{
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = ${MC.REGULATION_EXECUTION_RATE},
  .TIMx              = ${_last_word(MC.PWM_TIMER_SELECTION)}, 
/* PWM Driving signals initialization ----------------------------------------*/
<#if MC.STSPIN32G4 == false >
  .OCPolarity = LL_TIM_OCPOLARITY_${_last_word(PWMTIM.get("OCPolarity_1"))},
  .OCNPolarity = LL_TIM_OCPOLARITY_${_last_word(PWMTIM.get("OCNPolarity_1"))},
<#else>
  .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
  .OCNPolarity = LL_TIM_OCPOLARITY_HIGH,
</#if>
};
  <#else>
const ThreePwm_Params_t ThreePwm_ParamsM1 =
{
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = ${MC.REGULATION_EXECUTION_RATE},
  .TIMx              = ${_last_word(MC.PWM_TIMER_SELECTION)}, 
/* PWM Driving signals initialization ----------------------------------------*/
  .OCPolarity = LL_TIM_OCPOLARITY_${_last_word(PWMTIM.get("OCPolarity_1"))},
  .pwm_en_u_port  = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin   = M1_PWM_EN_U_Pin,
  .pwm_en_v_port  = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin   = M1_PWM_EN_V_Pin,
  .pwm_en_w_port  = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin   = M1_PWM_EN_W_Pin,
};
  </#if>

  <#if  MC.SPEED_SENSOR_SELECTION == "SENSORLESS_ADC">
/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const Bemf_ADC_Params_t Bemf_ADC_ParamsM1 =
{
  .LfTim = ${_last_word(MC.LF_TIMER_SELECTION)},
  .LfTimerChannel = LL_TIM_CHANNEL_CH1,        /*!< Channel of the LF timer used for speed measurement */ 
<#if  MC.BEMF_DIVIDER_AVAILABLE>  
  .gpio_divider_available = true,         /*!< Availability of the GPIO port enabling the bemf resistor divider */
  .bemf_divider_port = M1_BEMF_DIVIDER_GPIO_Port,        /*!< GPIO port of OnSensing divider enabler */
  .bemf_divider_pin = M1_BEMF_DIVIDER_Pin,
<#else>
  .gpio_divider_available = false,         /*!< Availability of the GPIO port enabling the bemf resistor divider */
</#if>  
  .pAdc = {${MC.PHASE_U_BEMF_ADC}, ${MC.PHASE_V_BEMF_ADC}, ${MC.PHASE_W_BEMF_ADC}},                 /*!< Pointer to the ADC */
  .AdcChannel = {MC_${MC.PHASE_U_BEMF_CHANNEL}, MC_${MC.PHASE_V_BEMF_CHANNEL}, MC_${MC.PHASE_W_BEMF_CHANNEL}},
    <#if CondFamily_STM32F0>
  .DMAx               = DMA1,
  .DMA_ADC_DR_ChannelX = LL_DMA_CHANNEL_1,
    </#if>
};

  </#if>

  <#if  MC.DRIVE_MODE == "CM">
const CurrentRef_Params_t CurrentRef_ParamsM1 =
{
  .TIMx = ${_last_word(MC.CURR_REF_TIMER_SELECTION)},                 /*!< It contains the pointer to the timer
                                           used for current reference PWM generation. */
  .RefTimerChannel = LL_TIM_CHANNEL_CH1,
};
  </#if>
</#if><#-- DRIVE_TYPE == "SIX_STEP" -->


<#if  MC.ESC_ENABLE>
const ESC_Params_t ESC_ParamsM1 =
{
  .Command_TIM = TIM2,
  .Motor_TIM = TIM1,
  .ARMING_TIME = 200,
  .PWM_TURNOFF_MAX  = 500,   
  .TURNOFF_TIME_MAX = 500, 
  .Ton_max =  ESC_TON_MAX,                      /*!<  Maximum ton value for PWM (by default is 1800 us) */
  .Ton_min =  ESC_TON_MIN,                      /*!<  Minimum ton value for PWM (by default is 1080 us) */ 
  .Ton_arming = ESC_TON_ARMING,                 /*!<  Minimum value to start the arming of PWM */ 
  .delta_Ton_max = ESC_TON_MAX - ESC_TON_MIN,      
  .speed_max_valueRPM = MOTOR_MAX_SPEED_RPM,    /*!< Maximum value for speed reference from Workbench */
  .speed_min_valueRPM = 1000,                   /*!< Set the minimum value for speed reference */
  .motor = M1,
};
</#if>


<#if MC.DRIVE_TYPE == "ACIM">

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = <#if MC.M1_CS_ADC_PHASE_SHARED!="U">${MC.M1_CS_ADC_U}<#else>${MC.M1_CS_ADC_V}</#if>,                 
  .ADCx_2 = <#if MC.M1_CS_ADC_PHASE_SHARED!="W">${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>,
  /* Motor Control Kit config */ 
   .ADCConfig1 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",2,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",3,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",4,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",5,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",6,"PHASE_1")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
              },
   .ADCConfig2 = { ( uint32_t )( ${getADC("CFG",1,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",2,"PHASE_2")}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",3,"PHASE_2")}<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",4,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",5,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( ${getADC("CFG",6,"PHASE_2")}<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
              },                   
 
   .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}                          
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}                            
                 },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = ${_last_word(MC.PWM_TIMER_SELECTION)},

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
  <#if MC.LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" >
    <#if MC.SHARED_SIGNAL_ENABLE == false > 
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port, 
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,
    <#else>
 .pwm_en_u_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_v_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_UVW_Pin,
 .pwm_en_w_port      = M1_PWM_EN_UVW_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_UVW_Pin,
    </#if>
  <#else>
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,
  </#if>


/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = ${MC.BKIN2_MODE},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>  
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_CURRENT_PROTECTION_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
        <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
        <#else>
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,  
        </#if> 
      <#else>
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
      </#if>

  <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
    <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
    <#else>
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
    </#if>
   <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
  </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.OVPREF},

};
</#if> <#-- MC.DRIVE_TYPE == "ACIM" --> <#-- Inside CondFamily_STM32G4 --> 





/* USER CODE BEGIN Additional parameters */


/* USER CODE END Additional parameters */  

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

