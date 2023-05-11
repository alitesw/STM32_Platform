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
/**
  ******************************************************************************
  * @file    mcp_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration information of the MCP protocol
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
  
#include "parameters_conversion.h"
<#if MC.MCP_ASPEP_OVER_UART >
#include "usart_aspep_driver.h"
</#if>
<#if MC.MCP_OVER_ASPEP >
#include "aspep.h"
</#if>
<#if MC.MCP_OVER_STLNK>
#include "stlink_mcptl.h"
</#if>
#include "mcp.h"
<#if MC.MCP_DATALOG_USED >
#include "mcpa.h"
</#if>
#include "mcp_config.h"

static uint8_t MCPSyncTxBuff[MCP_TX_SYNCBUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t MCPSyncRXBuff[MCP_RX_SYNCBUFFER_SIZE] __attribute__((aligned(4)));

<#if MC.MCP_DATALOG_OVER_UART_A >
/* Asynchronous buffer dedicated to UART_A*/
static uint8_t MCPAsyncBuffUARTA_A[MCP_TX_ASYNCBUFFER_SIZE_A] __attribute__((aligned(4)));
static uint8_t MCPAsyncBuffUARTA_B[MCP_TX_ASYNCBUFFER_SIZE_A] __attribute__((aligned(4)));
/* Buffer dedicated to store pointer of data to be streamed over UART_A*/
void * dataPtrTableA[MCPA_OVER_UARTA_STREAM];
void * dataPtrTableBuffA[MCPA_OVER_UARTA_STREAM];
uint8_t dataSizeTableA[MCPA_OVER_UARTA_STREAM];
uint8_t dataSizeTableBuffA[MCPA_OVER_UARTA_STREAM]; /* buffered version of dataSizeTableA */
</#if>
<#if MC.MCP_DATALOG_OVER_UART_B >
/* Asynchronous buffer dedicated to UART_B*/
static uint8_t MCPAsyncBuffUARTB_A[MCP_TX_ASYNCBUFFER_SIZE_B] __attribute__((aligned(4)));
static uint8_t MCPAsyncBuffUARTB_B[MCP_TX_ASYNCBUFFER_SIZE_B] __attribute__((aligned(4)));
/* Buffer dedicated to store pointer of data to be streamed over UART_B*/
void * DataPtrTableB[MCPA_OVER_UARTB_STREAM];
void * DataPtrTableBuffB[MCPA_OVER_UARTB_STREAM];
uint8_t dataSizeTableB[MCPA_OVER_UARTB_STREAM];
uint8_t dataSizeTableBuffB[MCPA_OVER_UARTB_STREAM];
</#if>
<#if MC.MCP_DATALOG_OVER_STLNK >
static uint8_t STLnkAsyncBuffA[STLNK_TX_ASYNCBUFFER_SIZE] __attribute__((aligned(4))); 
static uint8_t STLnkAsyncBuffB[STLNK_TX_ASYNCBUFFER_SIZE] __attribute__((aligned(4)));
/* Buffer dedicated to store pointer of data to be streamed over STLnk*/
void * DataPtrTableSTlnk[MCPA_OVER_STLNK_STREAM];
void * DataPtrTableBuffSTlnk[MCPA_OVER_STLNK_STREAM];
uint8_t dataSizeTableSTlnk[MCPA_OVER_STLNK_STREAM];
uint8_t dataSizeTableBuffSTlnk[MCPA_OVER_STLNK_STREAM];
</#if>

MCP_user_cb_t MCP_UserCallBack[MCP_USER_CALLBACK_MAX];

<#if MC.MCP_OVER_UART_A >
static UASPEP_Handle_t UASPEP_A =
{
 .USARTx = USARTA,
 .rxDMA = DMA_RX_A,
 .txDMA = DMA_TX_A,
 .rxChannel = DMACH_RX_A,
 .txChannel = DMACH_TX_A,
};


ASPEP_Handle_t aspepOverUartA =
{
  ._Super = 
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .HWIp = &UASPEP_A,
  .Capabilities = {
    .DATA_CRC = 0U,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX>>5U)-1U,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX>>5U)-1U,
    .TXA_maxSize = <#if MC.MCP_DATALOG_OVER_UART_A > MCP_TX_ASYNC_PAYLOAD_MAX_A>>6U, <#else> 0, </#if>
    .version = 0x0U,
  },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_DATALOG_OVER_UART_A >
  .asyncBufferA = {
    .buffer = MCPAsyncBuffUARTA_A,
  },
  .asyncBufferB = {
    .buffer = MCPAsyncBuffUARTA_B,
  },
  </#if>
  .rxBuffer = <#if MC.MCP_OVER_STLNK >&MCPSyncRXBuff[4]<#else>MCPSyncRXBuff</#if>, 
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_receive = &UASPEP_RECEIVE_BUFFER,
  .fASPEP_send = &UASPEP_SEND_PACKET,
  .liid = 0,
};

MCP_Handle_t MCP_Over_UartA =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA, //cstat !MISRAC2012-Rule-11.3
};

</#if>

<#if MC.MCP_OVER_UART_B >
UASPEP_Handle_t UASPEP_B =
{
 .USARTx = USARTB,
 .rxDMA = DMA_RX_B,
 .txDMA = DMA_TX_B,
 .rxChannel = DMACH_RX_B,
 .txChannel = DMACH_TX_B,
};

ASPEP_Handle_t aspepOverUartB =
{
  ._Super = 
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .HWIp = &UASPEP_B,
  .Capabilities = {
    .DATA_CRC = 0,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXA_maxSize = <#if MC.MCP_DATALOG_OVER_UART_B > MCP_TX_ASYNC_PAYLOAD_MAX_B>>6, <#else> 0, </#if>
    .version = 0x0,
  },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_DATALOG_OVER_UART_B >
  .asyncBufferA = {
    .buffer = MCPAsyncBuffUARTB_A,
  },
  .asyncBufferB = {
    .buffer = MCPAsyncBuffUARTB_B,
  },
  </#if>
  .rxBuffer = <#if MC.MCP_OVER_STLNK >&MCPSyncRXBuff[4]<#else>MCPSyncRXBuff</#if>, 
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_receive = &UASPEP_RECEIVE_BUFFER,
  .fASPEP_send = &UASPEP_SEND_PACKET,
  .liid = 1,
};

MCP_Handle_t MCP_Over_UartB =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartB, //cstat !MISRAC2012-Rule-11.3
};

</#if>

<#if MC.MCP_OVER_STLNK >

STLNK_Handle_t STLNK =
{
  ._Super = 
   {
     .fGetBuffer = &STLNK_getBuffer,
     .fSendPacket = &STLNK_sendPacket,
     .fRXPacketProcess = &STLNK_RXframeProcess,
   },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_DATALOG_OVER_STLNK >
  .asyncBufferA = {
    .buffer = STLnkAsyncBuffA,
  },
  .asyncBufferB = {
    .buffer = STLnkAsyncBuffB,
  },
  </#if>
  .rxBuffer = MCPSyncRXBuff,

};

MCP_Handle_t MCP_Over_STLNK =
{
  .pTransportLayer = (MCTL_Handle_t *) &STLNK, //cstat !MISRAC2012-Rule-11.3
};

//force Variable to section data in RAM -> variable is place has the beginning of the section and this is what we want

#if defined (__ICCARM__)
#pragma location=0x20000000
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".data")))
#endif
STLNK_Control_t stlnkCtrl =
{
  .rxBuffer = MCPSyncRXBuff,
  .syncBuffer = MCPSyncTxBuff,
  .asyncBufferA = <#if MC.MCP_DATALOG_OVER_STLNK > &STLnkAsyncBuffA[4], <#else> 0, </#if>
  .asyncBufferB = <#if MC.MCP_DATALOG_OVER_STLNK > &STLnkAsyncBuffB[4], <#else> 0, </#if>
}; 
</#if>

<#if MC.MCP_DATALOG_OVER_UART_A >
MCPA_Handle_t MCPA_UART_A =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableA,
  .dataPtrTableBuff = dataPtrTableBuffA,
  .dataSizeTable = dataSizeTableA,
  .dataSizeTableBuff = dataSizeTableBuffA,
  .nbrOfDataLog = MCPA_OVER_UARTA_STREAM,
};
</#if>

<#if MC.MCP_DATALOG_OVER_UART_B >
MCPA_Handle_t MCPA_UART_B =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartB, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableB,
  .dataPtrTableBuff = dataPtrTableBuffB,  
  .dataSizeTable = dataSizeTableB,
  .dataSizeTableBuff = dataSizeTableBuffB,
  .nbrOfDataLog = MCPA_OVER_UARTB_STREAM,
};

</#if>

<#if MC.MCP_DATALOG_OVER_STLNK >
MCPA_Handle_t MCPA_STLNK =
{
  .pTransportLayer = (MCTL_Handle_t *) &STLNK, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableSTlnk,
  .dataPtrTableBuff = dataPtrTableBuffSTlnk,  
  .dataSizeTable = dataSizeTableSTlnk,
  .dataSizeTableBuff = dataSizeTableBuffSTlnk,
  .nbrOfDataLog = MCPA_OVER_STLNK_STREAM,
  
};

</#if>


/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
