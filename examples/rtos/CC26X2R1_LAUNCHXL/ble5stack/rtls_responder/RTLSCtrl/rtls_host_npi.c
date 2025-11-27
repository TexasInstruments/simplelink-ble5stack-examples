/******************************************************************************

 @file  rtls_host_npi.c

 @brief This file contains the uNPI specific implementation of a RTLS Host

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2018-2025, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <string.h>
#include <stdlib.h>

#include "bcomdef.h"

#include <ti_drivers_config.h>
#include "rtls_host.h"
#include "rtls_ctrl.h"
#include "rtls_ctrl_api.h"

#include "npi_data.h"
#include "npi_task.h"
#include "npi_util.h"

#ifndef USE_RCL
#include <driverlib/ioc.h>
#define NPITask_freeFrameData NPITask_freeFrame
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// NPI Task configuration
#define NPI_MSG_BUFF_SIZE         200
#define NPI_TASK_STACK_SIZE       700

// TODO: This still true? Need to add more devices to this list?
#if defined(CC26X2R1_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL) ||\
	(defined (CC13X2P1_LAUNCHXL) || defined (CC13X2P_2_LAUNCHXL) ||\
	defined (CC13X2P_4_LAUNCHXL))
#define MRDY_GPIO                  IOID_UNUSED
#define SRDY_GPIO                  IOID_UNUSED
#endif //CC2650DK_7ID

// NPI Message types
#define NPI_ASYNC_REQ             ((NPI_MSG_TYPE_ASYNC << 5) ^ RPC_SYS_RTLS_CTRL)
#define NPI_ASYNC_RSP             ((NPI_MSG_TYPE_ASYNC << 5) ^ RPC_SYS_RTLS_CTRL)
#define NPI_SYNC_REQ              ((NPI_MSG_TYPE_SYNCREQ << 5) ^ RPC_SYS_RTLS_CTRL)
#define NPI_SYNC_RSP              ((NPI_MSG_TYPE_SYNCRSP << 5) ^ RPC_SYS_RTLS_CTRL)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// uNPI Parameters for opening serial port to RTLS Control
NPI_Params npiPortParams;

// RTLS Control message processing function
pfnRtlsCtrlProcessMsgCb gRtlsCtrlProcessMsgCb = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void RTLSHost_processNpiMessage(_npiFrame_t *pNpiMsg);
uint8_t RTLSHost_createAndSendNpiMessage(uint8_t cmdId, uint8_t cmdTypeNpi, uint8_t *pData, uint16_t dataLen);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      RTLSHost_openHostIf
 *
 * @brief   Open NPI interfaces
 *
 * @param   none
 *
 * @return  Status of NPITask_open
 */
void RTLSHost_openHostIf(pfnRtlsCtrlProcessMsgCb rtlsAppCb)
{
#ifdef RTLS_HOST_EXTERNAL
  // Initialize NPI Interface
  NPITask_Params_init(NPI_SERIAL_TYPE_UART, &npiPortParams);

  npiPortParams.stackSize = NPI_TASK_STACK_SIZE;
#ifdef USE_RCL
  npiPortParams.mrdyGpioIndex = 0 /*MRDY_GPIO*/;
  npiPortParams.srdyGpioIndex = 0 /*SRDY_GPIO*/;
#else
  npiPortParams.mrdyGpioIndex = MRDY_GPIO;
  npiPortParams.srdyGpioIndex = SRDY_GPIO;
#endif
  npiPortParams.bufSize   = NPI_MSG_BUFF_SIZE;
  npiPortParams.portParams.uartParams.baudRate = 460800;

  // Kick off NPI task
  NPITask_open(&npiPortParams);

  // Register callback and subsystem with NPI task
  NPITask_regSSFromHostCB(RPC_SYS_RTLS_CTRL ,RTLSHost_processNpiMessage);

  // Change NPI assert subsystemId and command
  NPITask_chgAssertHdr(RPC_SYS_RTLS_CTRL+(NPI_MSG_TYPE_ASYNC<<5), RTLS_EVT_ASSERT);

  gRtlsCtrlProcessMsgCb = (pfnRtlsCtrlProcessMsgCb)rtlsAppCb;

  if(gRtlsCtrlProcessMsgCb == NULL)
  {
      AssertHandler(RTLS_CTRL_ASSERT_CAUSE_NULL_POINTER_EXCEPT, 0);
  }
#endif
}

#ifdef USE_RCL
uint8_t RTLSHost_createAndSendNpiMessage(uint8_t cmdId, uint8_t cmdTypeNpi, uint8_t *pData, uint16_t dataLen)
{
    _npiFrame_t npiMsg;

    // Build and send the NPI message
    npiMsg.dataLen = dataLen;
    npiMsg.cmd0 = cmdTypeNpi;
    npiMsg.cmd1 = cmdId;

    // If we have any data to send
    if ((pData != NULL) && (0 != dataLen))
    {
      npiMsg.pData = NPIUtil_malloc(dataLen);
      if (NULL != npiMsg.pData)
      {
        memcpy(npiMsg.pData, pData, dataLen);
      }
    }
    else
    {
      npiMsg.pData = NULL;
    }

    // Forward npiFrame to uNPI
    if (NPITask_sendToHost(&npiMsg) != NPI_SUCCESS)
    {
      NPITask_freeFrameData(&npiMsg);
      return FAILURE;
    }
    return SUCCESS;
}
#else
uint8_t RTLSHost_createAndSendNpiMessage(uint8_t cmdId, uint8_t cmdTypeNpi, uint8_t *pData, uint16_t dataLen)
{
    _npiFrame_t *npiMsg = NULL;

    npiMsg = (_npiFrame_t *)NPIUtil_malloc(sizeof(_npiFrame_t) + dataLen);

    // Build and send the NPI message
    if (npiMsg != NULL)
    {
      npiMsg->dataLen = dataLen;
      npiMsg->cmd0 = cmdTypeNpi;
      npiMsg->cmd1 = cmdId;

      // If we have any data to send
      if ((pData != NULL) && (0 != dataLen))
      {
        npiMsg->pData = (uint8_t *)((uint32_t)npiMsg + sizeof(_npiFrame_t));
        memcpy(npiMsg->pData, pData, dataLen);
      }

      // Forward npiFrame to uNPI
      if (NPITask_sendToHost(npiMsg) != NPI_SUCCESS)
      {
       NPIUtil_free((uint8_t *)npiMsg);

       return FAILURE;
      }
    }
    return SUCCESS;
}
#endif
/*********************************************************************
 * @fn      RTLSHost_sendMsg
 *
 * @brief   Build and send a uNPI command
 *
 * @param   cmdId - Command Id requested by Host
 * @param   cmdType - Sync/Async commands
 * @param   dataLen - Length of pData buffer
 * @param   pData - Pointer to a data buffer
 *
 * @return  status - 0 = success, 1 = failed
 */
uint8_t RTLSHost_sendMsg(uint8_t cmdId, uint8_t cmdType, uint8_t *pData, uint16_t dataLen)
{
#ifdef RTLS_HOST_EXTERNAL
  uint8_t cmdTypeNpi;

  if (cmdId <= RTLS_CMD_BLE_LOG_STRINGS_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : RTLS host send cmdType=%d, cmdId=%s\n", cmdType, rtlsCmd_BleLogStrings[cmdId]);
  }
  else
  {
    if (cmdId == RTLS_EVT_CONN_INFO)
    {
      // too much printing filter it BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : RTLS host send cmdType=%d, cmdId=%s\n", cmdType, "RTLS_EVT_CONN_INFO");
    }
    else
    {
      BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLS host send cmdType=0x%x, cmdId=0x%x\n", cmdType, cmdId);
    }
  }
  // First, translate Host message to NPI message
  switch (cmdType)
  {
    case HOST_ASYNC_RSP:
    {
      cmdTypeNpi = NPI_ASYNC_RSP;
    }
    break;

    case HOST_SYNC_RSP:
    {
      cmdTypeNpi = NPI_SYNC_RSP;
    }
    break;

    default:
      return FAILURE;
  }

  return RTLSHost_createAndSendNpiMessage( cmdId, cmdTypeNpi, pData, dataLen);
#else
  return SUCCESS;
#endif
}

/*********************************************************************
 * @fn      RTLSHost_processNpiMessage
 *
 * @brief   Process an incoming uNPI Message to RTLS Control
 *          This is a uNPI abstraction of the real processing function
 *
 * @param   pMsg - message to process
 *
 */
void RTLSHost_processNpiMessage(_npiFrame_t *pNpiMsg)
{
#ifdef RTLS_HOST_EXTERNAL
  rtlsHostMsg_t *pHostMsg;

  if (pNpiMsg == NULL)
  {
    return;
  }

  pHostMsg = (rtlsHostMsg_t *)NPIUtil_malloc(sizeof(rtlsHostMsg_t));

  // If we could not allocate space for the message, drop it
  if (!pHostMsg)
  {
    NPITask_freeFrameData(pNpiMsg);
    return;
  }

  pHostMsg->cmdId = pNpiMsg->cmd1;
  pHostMsg->dataLen = pNpiMsg->dataLen;

  if ((pNpiMsg->dataLen != 0) && (NULL != pNpiMsg->pData))
  {
    pHostMsg->pData = (uint8_t *)NPIUtil_malloc(pNpiMsg->dataLen);

    // Check that we could allocate payload (only if the message is not empty)
    if (NULL == pHostMsg->pData)
    {
      NPITask_freeFrameData(pNpiMsg);

      NPIUtil_free((uint8_t *)pHostMsg);
      return;
    }

    memcpy(pHostMsg->pData, pNpiMsg->pData, pNpiMsg->dataLen);
  }

  // NPI to Host
  switch (pNpiMsg->cmd0)
  {
    case NPI_SYNC_REQ:
    {
      pHostMsg->cmdType = HOST_SYNC_REQ;
    }
    break;

    case NPI_ASYNC_REQ:
    {
      pHostMsg->cmdType = HOST_ASYNC_REQ;
    }
    break;
  }

  NPITask_freeFrameData(pNpiMsg);

  gRtlsCtrlProcessMsgCb(pHostMsg);
#endif
}
