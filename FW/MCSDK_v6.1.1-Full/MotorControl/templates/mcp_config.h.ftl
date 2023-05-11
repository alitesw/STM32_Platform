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

<#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7>
<#assign DMA_TYPE = "STREAM">
<#else>
<#assign DMA_TYPE = "CHANNEL">
</#if>
/**
  ******************************************************************************
  * @file    mcp_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration definition of the MCP protocol
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
  
#ifndef MCP_CONFIG_H
#define MCP_CONFIG_H

#include "mcp.h"
<#if MC.MCP_ASPEP_OVER_UART>
#include "aspep.h"
</#if>
<#if MC.MCP_OVER_STLNK>
#include "stlink_mcptl.h"
</#if>
<#if MC.MCP_DATALOG_USED >
#include "mcpa.h"
</#if>

<#if MC.MCP_OVER_UART_A >
#define USARTA ${MC.MCP_UART_SELECTION_A}
#define DMA_RX_A ${MC.MCP_DMA_RX_UART_A}
#define DMA_TX_A ${MC.MCP_DMA_TX_UART_A}
#define DMACH_RX_A LL_DMA_${DMA_TYPE}_${MC.MCP_DMACH_RX_UART_A}
#define DMACH_TX_A LL_DMA_${DMA_TYPE}_${MC.MCP_DMACH_TX_UART_A}
</#if>

<#if MC.MCP_OVER_UART_B >
#define USARTB ${MC.MCP_UART_SELECTION_B}
#define DMA_RX_B ${MC.MCP_DMA_RX_UART_B}
#define DMA_TX_B ${MC.MCP_DMA_TX_UART_B}
#define DMACH_RX_B LL_DMA_${DMA_TYPE}_${MC.MCP_DMACH_RX_UART_B}
#define DMACH_TX_B LL_DMA_${DMA_TYPE}_${MC.MCP_DMACH_TX_UART_B}
</#if>

#define MCP_USER_CALLBACK_MAX 2

#define MCP_TX_SYNC_PAYLOAD_MAX ${MC.MCP_TX_SYNC_PAYLOAD_MAX}
#define MCP_RX_SYNC_PAYLOAD_MAX ${MC.MCP_RX_SYNC_PAYLOAD_MAX}
#define MCP_TX_SYNCBUFFER_SIZE (MCP_TX_SYNC_PAYLOAD_MAX+ASPEP_HEADER_SIZE+ASPEP_DATACRC_SIZE)
#define MCP_RX_SYNCBUFFER_SIZE (MCP_RX_SYNC_PAYLOAD_MAX+ASPEP_DATACRC_SIZE) // ASPEP_HEADER_SIZE is not stored in the RX buffer.

<#if MC.MCP_DATALOG_OVER_UART_A>
#define MCP_TX_ASYNC_PAYLOAD_MAX_A ${MC.MCP_DATALOG_PAYLOAD_MAX_UART_A}
#define MCP_TX_ASYNCBUFFER_SIZE_A (MCP_TX_ASYNC_PAYLOAD_MAX_A+ASPEP_HEADER_SIZE+ASPEP_DATACRC_SIZE)
#define MCPA_OVER_UARTA_STREAM ${MC.MCP_DATALOG_OVER_UART_A_SIGNALS}
</#if>
<#if MC.MCP_DATALOG_OVER_UART_B>
#define MCP_TX_ASYNC_PAYLOAD_MAX_B ${MC.MCP_DATALOG_PAYLOAD_MAX_UART_A}
#define MCP_TX_ASYNCBUFFER_SIZE_B (MCP_TX_ASYNC_PAYLOAD_MAX_B+ASPEP_HEADER_SIZE+ASPEP_DATACRC_SIZE)
#define MCPA_OVER_UARTB_STREAM ${MC.MCP_DATALOG_OVER_UART_B_SIGNALS}
</#if>
<#if MC.MCP_DATALOG_OVER_STLNK>
/* 4 first bytes are required to store ptr to MCTL_Buff_t struct  */
/* 1 additional byte is required to identify buffer A and B */
#define STLNK_TX_ASYNCBUFFER_SIZE (${MC.MCP_DATALOG_PAYLOAD_MAX_STLNK} + STLNK_CRC_SIZE + 5U) 
#define MCPA_OVER_STLNK_STREAM ${MC.MCP_DATALOG_OVER_STLNK_SIGNALS}
</#if>

<#if MC.MCP_OVER_UART_A>
extern ASPEP_Handle_t aspepOverUartA;
extern MCP_Handle_t MCP_Over_UartA;
</#if>
<#if MC.MCP_OVER_UART_B>
extern ASPEP_Handle_t aspepOverUartB;
extern MCP_Handle_t MCP_Over_UartB;
</#if>
<#if MC.MCP_OVER_STLNK>
extern STLNK_Handle_t STLNK;
extern MCP_Handle_t MCP_Over_STLNK;
extern STLNK_Control_t stlnkCtrl;
</#if>
<#if MC.MCP_DATALOG_OVER_UART_A>
extern MCPA_Handle_t MCPA_UART_A;
</#if>
<#if MC.MCP_DATALOG_OVER_UART_B>
extern MCPA_Handle_t MCPA_UART_B;
</#if>
<#if MC.MCP_DATALOG_OVER_STLNK >
extern MCPA_Handle_t MCPA_STLNK;
</#if>
extern MCP_user_cb_t MCP_UserCallBack[MCP_USER_CALLBACK_MAX];
#endif /* MCP_CONFIG_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
