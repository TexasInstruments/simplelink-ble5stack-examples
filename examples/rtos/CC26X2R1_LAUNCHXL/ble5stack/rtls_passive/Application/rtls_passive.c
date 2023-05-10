/**************************************************************************************************
  Filename:       rtls_passive.c

  Description:    This file contains the RTLS Passive application
                  sample application definitions and prototypes.

* Copyright (c) 2017, Texas Instruments Incorporated
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* *  Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* *  Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* *  Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#include "rtls_passive.h"

#include <string.h>
#include <stdlib.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>

#include "bcomdef.h"
#include <hal_assert.h>
#include "micro_ble_cm.h"

// DriverLib
#include <driverlib/aon_batmon.h>
#include "uble.h"
#include "ugap.h"
#include "urfc.h"

// RTLS
#include "rtls_ble.h"
#include "rtls_ctrl_api.h"
#ifdef RTLS_CTE
#include "rtls_aoa_api.h"
#endif /* RTLS_CTE */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Stack Task configuration
#define UBT_TASK_PRIORITY                     3

#ifndef UBT_TASK_STACK_SIZE
#define UBT_TASK_STACK_SIZE                   800
#endif

// App Task configuration
#define UCA_TASK_PRIORITY                     2

#ifndef UCA_TASK_STACK_SIZE
#define UCA_TASK_STACK_SIZE                   800
#endif

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
// Event globally used to post local events and pend on local events.
Event_Handle syncAppEvent;

// Event globally used to post local events and pend on local events.
static Event_Handle syncStackEvent;

Task_Struct ubtTask;
uint8 ubtTaskStack[UBT_TASK_STACK_SIZE];

Task_Struct ucaTask;
uint8 ucaTaskStack[UCA_TASK_STACK_SIZE];

// Queue object used for app messages
static Queue_Struct stackMsg;
static Queue_Handle stackMsgQueue;

// Queue object used for app messages
Queue_Struct appMsg;
Queue_Handle appMsgQueue;

// Flag to indicate whether we are reporting to RTLS Control or not
uint8_t gRtlsSyncEnabled = RTLS_FALSE;

// CM is tracking
uint8_t gMonitorTracking[CM_MAX_SESSIONS] = {RTLS_FALSE};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void RTLSPassive_taskFxn(UArg a0, UArg a1);

//ubStack High Priority Task
static void ubStack_taskFxn(UArg a0, UArg a1);

static void uBLEStack_eventProxy(void);
static bStatus_t RTLSPassive_enqueueStackMsg(uint16 event, uint8 data);

// General message handling
static void RTLSPassive_processRtlsPassiveMsg(uint8_t *pMsg);
#ifdef RTLS_CTE
static void RTLSPassive_processUrtlsMsg(uint8_t *pMsg);
#endif /* RTLS_CTE */

// Handling CM events
static void RTLSPassive_processCmMsg(uint8_t *pMsg);
static void RTLSPassive_monitorIndicationEvt(uint8_t *pData);
static void RTLSPassive_monitorCompleteEvt(uint8_t *pData);
static void RTLSPassive_monitorStateChangeEvt(ugapMonitorState_t newState);

// Handling RTLS Control events
static void RTLSPassive_processRtlsCtrlMsg(uint8_t *pMsg);
static void RTLSPassive_cmStartReq(uint8_t *pConnInfo);
static void RTLSPassive_enableRtlsSync(rtlsEnableSync_t *enable);
static void RTLSPassive_terminateLinkReq(uint8_t sessionId);
#ifdef RTLS_CTE
static void RTLSPassive_setAoaParamsReq(rtlsAoaConfigReq_t *pConfig);
static void RTLSPassive_enableAoaReq(rtlsAoaEnableReq_t *pReq);
#endif /* RTLS_CTE */
uint16_t RTLSPassive_getHostConnHandle(uint8_t sessionId);
uint8_t RTLSPassive_getSessionId(uint16_t hostConnHandle);

/*********************************************************************
 * @fn      RTLSPassive_processRtlsPassiveMsg
 *
 * @brief   General handling of application events
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void RTLSPassive_processRtlsPassiveMsg(uint8_t *pMsg)
{
  rtlsPassiveEvt_t *pEvt = (rtlsPassiveEvt_t *)pMsg;
  volatile uint32_t keyHwi;

  switch(pEvt->event)
  {
    case RTLS_PASSIVE_RTLS_CTRL_EVT:
    {
      RTLSPassive_processRtlsCtrlMsg((uint8_t *)pEvt->pData);
    }
    break;

    case RTLS_PASSIVE_CM_EVT:
    {
      RTLSPassive_processCmMsg((uint8_t *)pEvt->pData);
    }
    break;

#ifdef RTLS_CTE
    case RTLS_PASSIVE_URTLS_EVT:
    {
      RTLSPassive_processUrtlsMsg((uint8_t *)pEvt->pData);
    }
    break;
#endif /* RTLS_CTE */

    default:
      break;
  }

  keyHwi = Hwi_disable();
  free(pEvt->pData);
  Hwi_restore(keyHwi);
}

/*********************************************************************
 * @fn      RTLSPassive_processRtlsCtrlMsg
 *
 * @brief   Handle processing messages from RTLS Control
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void RTLSPassive_processRtlsCtrlMsg(uint8_t *pMsg)
{
  rtlsCtrlReq_t *pReq = (rtlsCtrlReq_t *)pMsg;
  volatile uint32_t keyHwi;

  if (pReq->reqOp <= RTLS_REQ_BLE_LOG_STRINGS_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : RTLS Ctrl Msg status=%d, event=%s\n", 0, rtlsReq_BleLogStrings[pReq->reqOp]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLS Ctrl Msg status=%d, event=0x%x\n", 0, pReq->reqOp);
  }

  switch(pReq->reqOp)
  {
    case RTLS_REQ_ENABLE_SYNC:
    {
      RTLSPassive_enableRtlsSync((rtlsEnableSync_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_CONN:
    {
      RTLSPassive_cmStartReq(pReq->pData);
    }
    break;

#ifdef RTLS_CTE
    case RTLS_REQ_SET_AOA_PARAMS:
    {
      RTLSPassive_setAoaParamsReq((rtlsAoaConfigReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_AOA_ENABLE:
    {
      RTLSPassive_enableAoaReq((rtlsAoaEnableReq_t *)pReq->pData);
    }
    break;
#endif /* RTLS_CTE */

    case RTLS_REQ_TERMINATE_LINK:
    {

      rtlsTerminateLinkReq_t *termInfo = (rtlsTerminateLinkReq_t *)pReq->pData;
      uint8_t sessionId;

      sessionId = RTLSPassive_getSessionId(termInfo->connHandle);
      if (sessionId == CM_INVALID_SESSION_ID)
      {
        // Ignore the command, session was not started or already stopped
        return;
      }

      RTLSPassive_terminateLinkReq(sessionId);
    }
    break;

    default:
      break;
  }

  keyHwi = Hwi_disable();
  free(pReq->pData);
  Hwi_restore(keyHwi);
}

#ifdef RTLS_CTE
/*********************************************************************
 * @fn      RTLSPassive_processUrtlsMsg
 *
 * @brief   Handle processing messages from urtls
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void RTLSPassive_processUrtlsMsg(uint8_t *pMsg)
{
  urtls_evt_t *pEvt = (urtls_evt_t *)pMsg;
  volatile uint32_t keyHwi;

  switch(pEvt->evtType)
  {
    case URTLS_CONNECTION_CTE_IQ_REPORT_EVT:
    {
      uint16_t hostConnHandle;
      urtls_connectionIQReport_t *pReport = (urtls_connectionIQReport_t *)pEvt->evtData;

      hostConnHandle = RTLSPassive_getHostConnHandle(pReport->sessionId);
      if (hostConnHandle == RTLS_CONNHANDLE_INVALID)
      {
        return;
      }

      RTLSAoa_processAoaResults(hostConnHandle,
                                pReport->rssi,
                                pReport->dataChIndex,
                                pReport->sampleCount,
                                pReport->sampleRate,
                                pReport->sampleSize,
                                pReport->sampleCtrl,
                                pReport->slotDuration,
                                pReport->numAnt,
                                pReport->iqSamples);
    }
    break;

    default:
      break;
  }

  keyHwi = Hwi_disable();
  free(pEvt->evtData);
  Hwi_restore(keyHwi);
}
#endif /* RTLS_CTE */

/*********************************************************************
 * @fn      RTLSPassive_processCmMsg
 *
 * @brief   Handle processing messages from Connection Monitor
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void RTLSPassive_processCmMsg(uint8_t *pMsg)
{
  cmEvt_t *pEvt = (cmEvt_t *)pMsg;
  volatile uint32_t keyHwi;

  //BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : CM msg status=%d, event=0x%x\n", 0, pEvt->event);
  switch(pEvt->event)
  {
    case CM_MONITOR_STATE_CHANGED_EVT:
    {
      RTLSPassive_monitorStateChangeEvt(*pEvt->pData);
    }
    break;

    case CM_PACKET_RECEIVED_EVT:
    {
      RTLSPassive_monitorIndicationEvt(pEvt->pData);
    }
    break;

    case CM_CONN_EVT_COMPLETE_EVT:
    {
      RTLSPassive_monitorCompleteEvt(pEvt->pData);
    }
    break;

    default:
      break;
  }

  keyHwi = Hwi_disable();
  free(pEvt->pData);
  Hwi_restore(keyHwi);
}

/*********************************************************************
 * @fn      RTLSPassive_enableRtlsSync
 *
 * @brief   Enable RTLS synchronization
 *
 * @param   enable - enable/disable sync
 *
 * @return  none
 */
static void RTLSPassive_enableRtlsSync(rtlsEnableSync_t *enable)
{
  if (enable->enable == RTLS_TRUE)
  {
    gRtlsSyncEnabled = RTLS_TRUE;
  }
}

/*********************************************************************
 * @fn      RTLSPassive_cmStartReq
 *
 * @brief   Start Monitoring a BLE connection
 *
 * @param   pConnInfo - BLE connection parameters
 *
 * @return  none
 */
static void RTLSPassive_cmStartReq(uint8_t *pConnInfo)
{
  bleConnInfo_t *pBleConnInfo = (bleConnInfo_t *)pConnInfo;
  uint8_t sessionId;

  sessionId = RTLSPassive_getSessionId(pBleConnInfo->connHandle);

  // When a session already exists, stop monitoring the connection, and try to resync.
  if (ubCM_isSessionActive(sessionId) == CM_SUCCESS)
  {
    if (CM_SUCCESS == ubCM_updateExt(sessionId, pBleConnInfo->connHandle,
                                     pBleConnInfo->accessAddr,
                                     pBleConnInfo->connInterval,
                                     pBleConnInfo->hopValue,
                                     pBleConnInfo->currChan,
                                     pBleConnInfo->chanMap,
                                     pBleConnInfo->crcInit))
    {
      return;
    }
    ubCM_stop(sessionId);
    gMonitorTracking[sessionId - 1] = RTLS_FALSE;
  }

  // Kick CM
  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : cmStartReq start new hopValue=%d, currChan=%d\n", pBleConnInfo->hopValue, pBleConnInfo->currChan);
  ubCM_startNewSession(pBleConnInfo->connHandle,
                       pBleConnInfo->accessAddr,
                       pBleConnInfo->connRole,
                       pBleConnInfo->connInterval,
                       pBleConnInfo->hopValue,
                       pBleConnInfo->currChan,
                       pBleConnInfo->chanMap,
                       pBleConnInfo->crcInit);

}

#ifdef RTLS_CTE
/*********************************************************************
 * @fn       RTLSPassive_setAoaParamsReq
 *
 * @brief   Configure AoA parameters to the uStack
 *
 * @param   config - Parameters to configure
 *
 * @return  none
 */
static void RTLSPassive_setAoaParamsReq(rtlsAoaConfigReq_t *pConfig)
{
  uint8_t sessionId;
  uint8_t status;
  bStatus_t antConfStatus;

  // Sanity check
  if (pConfig == NULL)
  {
    return;
  }

  sessionId = RTLSPassive_getSessionId(pConfig->connHandle);
  if (ubCM_isSessionActive(sessionId) != CM_SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Connection Handle: %d is not active", pConfig->connHandle);
    return;
  }

  // Initialize GPIO's specified in urtls.c (antennaTbl)
  // Initialize one of the antenna ID's as the main antenna (in this case the first antenna in the pattern)
  antConfStatus = urtls_initAntArray(pConfig->pAntPattern[0]);
  if (antConfStatus == FAILURE)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Antenna array configuration invalid", 0);
    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
  }

  // Configure AoA receiver parameters
  status = urtls_setConnCteReceiveParams(sessionId,
                                         pConfig->samplingEnable,
                                         pConfig->slotDurations,
                                         pConfig->numAnt,
                                         pConfig->pAntPattern);
  if (status == URTLS_FAIL)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Fails to Configure AoA receiver parameters", 0);
  }

  // Configure sample accuracy
  status = urtls_setCteSampleAccuracy(sessionId,
                                      pConfig->sampleRate,
                                      pConfig->sampleSize,
                                      pConfig->sampleRate,
                                      pConfig->sampleSize,
                                      pConfig->sampleCtrl);
  if (status == URTLS_FAIL)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Fails to configure sample accuracy", 0);
  }
}

/*********************************************************************
 * @fn      RTLSPassive_enableAoaReq
 *
 * @brief   Enable sampling AoA
 *
 * @param   config - Parameters to configure
 *
 * @return  none
 */
static void RTLSPassive_enableAoaReq(rtlsAoaEnableReq_t *pReq)
{
  uint8_t sessionId;
  uint8_t status;

  // Sanity check
  if (pReq == NULL)
  {
    return;
  }

  sessionId = RTLSPassive_getSessionId(pReq->connHandle);
  if (ubCM_isSessionActive(sessionId) != CM_SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Session ID: %d is not active", sessionId);
  }

  // Enable or disable CTE for a connection
  status = urtls_setConnCteRequestEnableCmd(sessionId,
                                            pReq->enableAoa,
                                            pReq->cteInterval,
                                            pReq->cteLength,
                                            URTLS_CTE_TYPE_AOA);
  if (status == URTLS_FAIL)
  {
     RTLSCtrl_sendDebugEvt((uint8_t *)"Fails to enable/disable CTE", 0);
  }
}
#endif /* RTLS_CTE */

/*********************************************************************
 * @fn      RTLSPassive_terminateLinkReq
 *
 * @brief   Stop Monitoring a BLE connection
 *
 * @return  none
 */
static void RTLSPassive_terminateLinkReq(uint8_t sessionId)
{
  uint16_t hostConnHandle;

  // Get connection handle before ubCM_stop
  hostConnHandle = RTLSPassive_getHostConnHandle(sessionId);

  ubCM_stop(sessionId);

  gMonitorTracking[sessionId - 1] = RTLS_FALSE;

#ifdef RTLS_CTE
  // Free urtls struct
  urtls_freeConn(sessionId, ubCMConnInfo.numHandles);
#endif /* RTLS_CTE */

  // Link terminated
  RTLSCtrl_connResultEvt(hostConnHandle, RTLS_LINK_TERMINATED);
}

/*********************************************************************
 * @fn      RTLSPassive_monitorIndicationEvt
 *
 * @brief   This function will be called for each BLE packet received
 *
 * @param   pData - Packet payload
 *
 * @return  none
 */
static void RTLSPassive_monitorIndicationEvt(uint8_t *pData)
{

}

/*********************************************************************
 * @fn      RTLSPassive_monitorCompleteEvt
 *
 * @brief   A single connection event has ended
 *
 * @param   pData - Session Id and Status of the session
 *
 * @return  none
 */
static void RTLSPassive_monitorCompleteEvt(uint8_t *pData)
{
  monitorCompleteEvt_t *pCompleteEvt = (monitorCompleteEvt_t *)pData;
  uint32_t nextStartTime;
  uint32_t currentTime;
  uint32_t nextEventTimeUs;
  int8_t responderRssi;
  uint8_t channel;
  rtlsStatus_e status;
  uint16_t hostConnHandle;
  port_key_t key;
  port_key_t key_h;

  // Convert CM Status to RTLS Status
  if (pCompleteEvt->status != CM_FAILED_NOT_ACTIVE)
  {
    if (gMonitorTracking[pCompleteEvt->sessionId - 1] == RTLS_FALSE)
    {
      hostConnHandle = RTLSPassive_getHostConnHandle(pCompleteEvt->sessionId);
      if (hostConnHandle == RTLS_CONNHANDLE_INVALID)
      {
        return;
      }
      RTLSCtrl_connResultEvt(hostConnHandle, RTLS_SUCCESS);

      gMonitorTracking[pCompleteEvt->sessionId - 1] = RTLS_TRUE;
    }
    status = RTLS_SUCCESS;
  }
  else if (pCompleteEvt->status == CM_FAILED_NOT_ACTIVE)
  {
    // This connection was lost
    RTLSPassive_terminateLinkReq(pCompleteEvt->sessionId);
    status = RTLS_FAIL;
  }

  // The timing of the next event is critical
  // Enter CS
  key_h = port_enterCS_HW();
  key = port_enterCS_SW();

  if (ubCM_isSessionActive(pCompleteEvt->sessionId) == CM_SUCCESS &&
      gMonitorTracking[pCompleteEvt->sessionId - 1] == RTLS_TRUE &&
      gRtlsSyncEnabled && status != RTLS_FAIL)
  {
    ubCM_ConnInfo_t connInfo = ubCMConnInfo.ArrayOfConnInfo[pCompleteEvt->sessionId - 1];

    // Get the next start time
    nextStartTime = connInfo.nextStartTime;

    currentTime = RF_getCurrentTime();
    nextEventTimeUs = RF_convertRatTicksToUs(nextStartTime - currentTime);

    // We are interested in the Responder RSSI and channel
    channel = pCompleteEvt->channel;
    hostConnHandle = connInfo.hostConnHandle;

    // If the Responder RSSI value is from the last monitoring session
    if (connInfo.connRole == BLE_ROLE_PERIPHERAL && uble_timeCompare(connInfo.timeStampCentral, connInfo.lastStartTime))
    {
      responderRssi = connInfo.rssiCentral;
    }
    else if (connInfo.connRole == BLE_ROLE_CENTRAL && uble_timeCompare(connInfo.timeStampPeripheral, connInfo.lastStartTime))
    {
      responderRssi = connInfo.rssiPeripheral;
    }
    else
    {
      // The Responder RSSI value is not from the last monitoring session
      responderRssi= CM_RSSI_NOT_AVAILABLE;
    }

    RTLSCtrl_syncNotifyEvt(hostConnHandle, status, nextEventTimeUs, responderRssi, channel);
  }

  // Exit CS
  port_exitCS_SW(key);
  port_exitCS_HW(key_h);
}

/*********************************************************************
 * @fn      RTLSPassive_getHostConnHandle
 *
 * @brief   get hostConnHandle from sessionId using ubCMConnInfo data base
 *
 * @param   sessionId - Session Id
 *
 * @return  hostConnHandle or 0xFF in case of not active session Id
 */
uint16_t RTLSPassive_getHostConnHandle(uint8_t sessionId)
{
  if (ubCM_isSessionActive(sessionId) != CM_SUCCESS)
  {
    return RTLS_CONNHANDLE_INVALID;
  }
  return ubCMConnInfo.ArrayOfConnInfo[sessionId - 1].hostConnHandle;
}

/*********************************************************************
 * @fn      RTLSPassive_getSessionId
 *
 * @brief   get sessionId from hostConnHandle using a search on ubCMConnInfo data base
 *
 * @param   hostConnHandle - host connection handle
 *
 * @return  sessionId or CM_INVALID_SESSION_ID in case of not found
 */
uint8_t RTLSPassive_getSessionId(uint16_t hostConnHandle)
{
  uint8_t sessionId;

  //set sessionId to first inactive session id
  for (sessionId=1; sessionId <= CM_MAX_SESSIONS; sessionId++)
  {
    if(ubCM_isSessionActive(sessionId) == CM_SUCCESS)
    {
      if (ubCMConnInfo.ArrayOfConnInfo[sessionId - 1].hostConnHandle == hostConnHandle)
      {
        // connection handle found, return sessionId
        break;
      }
    }
  }

  // check that session id is not out of bounds
  if (sessionId > CM_MAX_SESSIONS)
  {
    return CM_INVALID_SESSION_ID;
  }

  return sessionId;
}

/*********************************************************************
 * @fn      RTLSPassive_monitorStateChangeEvt
 *
 * @brief   This function will be called for each BLE CM state change
 *
 * @param   newState - The new state
 *
 * @return  none
 */
static void RTLSPassive_monitorStateChangeEvt(ugapMonitorState_t newState)
{
  switch (newState)
  {
    case UGAP_MONITOR_STATE_INITIALIZED:
    {

    }
    break;

    case UGAP_MONITOR_STATE_IDLE:
    {

    }
    break;

    case UGAP_MONITOR_STATE_MONITORING:
    {

    }
    break;

    default:
      break;
  }
}


/*********************************************************************
 *  @fn      RTLSPassive_init
 *
 *  @brief   Called during initialization and contains application
 *           specific initialization (ie. hardware initialization/setup,
 *           table initialization, power up notification, etc), and
 *           profile initialization/setup.
 *
 *  @param   None
 *
 *  @return  None
 */
void RTLSPassive_init(void)
{
  Task_Params ucaTaskParams;

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", UCA_TASK_PRIORITY);
  // Create an RTOS event used to wake up this application to process events.
  syncAppEvent = Event_create(NULL, NULL);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Configure App task
  Task_Params_init(&ucaTaskParams);
  ucaTaskParams.stack = ucaTaskStack;
  ucaTaskParams.stackSize = UCA_TASK_STACK_SIZE;
  ucaTaskParams.priority = UCA_TASK_PRIORITY;
  Task_construct(&ucaTask, RTLSPassive_taskFxn, &ucaTaskParams, NULL);
}

/*********************************************************************
 * @fn      RTLSPassive_taskFxn
 *
 * @brief   Main CM application RTOS task loop, handles new CM command requests
 *          and continous monitor calls.
 *
 * @param   a0 - Standard TI RTOS taskFxn arguments.
 * @param   a1 -
 *
 * @return  none
 */
static void RTLSPassive_taskFxn(UArg a0, UArg a1)
{
#ifdef RTLS_CTE
  urtls_register(RTLSPassive_urtlsCb);
#endif /* RTLS_CTE */

  for(;;)
  {
    volatile uint32 keyHwi;
    uint32_t events = Event_pend(syncAppEvent, Event_Id_NONE, UCA_ALL_EVENTS, BIOS_WAIT_FOREVER);

    // If RTOS queue is not empty, process app message.
    while (!Queue_empty(appMsgQueue))
    {
      keyHwi = Hwi_disable();
      rtlsPassiveEvt_t *pMsg = (rtlsPassiveEvt_t *)Util_dequeueMsg(appMsgQueue);
      Hwi_restore(keyHwi);

      if (pMsg)
      {
        // Process message.
        RTLSPassive_processRtlsPassiveMsg((uint8_t *)pMsg);

        keyHwi = Hwi_disable();
        free(pMsg);
        Hwi_restore(keyHwi);
      }
    }
  }
}

/*********************************************************************
 * @fn      RTLSPassive_rtlsCtrlMsgCb
 *
 * @brief   Callback given to RTLS Control
 *
 * @param  cmd - the command to be enqueued
 *
 * @return  none
 */
void RTLSPassive_rtlsCtrlMsgCb(uint8_t *pCmd)
{
  // Enqueue the message to switch context
  RTLSPassive_enqueueAppMsg(RTLS_PASSIVE_RTLS_CTRL_EVT, (uint8_t *)pCmd);
}

/*********************************************************************
 * @fn      RTLSPassive_cmCb
 *
 * @brief   Callback given to Connection Monitor
 *
 * @param   pCmd - the command to be enqueued
 *
 * @return  none
 */
void RTLSPassive_cmCb(uint8_t *pCmd)
{
  // Enqueue the message to switch context
  RTLSPassive_enqueueAppMsg(RTLS_PASSIVE_CM_EVT, (uint8_t *)pCmd);
}

/*********************************************************************
 * @fn      RTLSPassive_urtlsCb
 *
 * @brief   Callback given to urtls
 *
 * @param   pUrtlsEvt - the command to be enqueued
 *
 * @return  none
 */
void RTLSPassive_urtlsCb(uint8_t *pUrtlsEvt)
{
  // Enqueue the message to switch context
  RTLSPassive_enqueueAppMsg(RTLS_PASSIVE_URTLS_EVT, (uint8_t *)pUrtlsEvt);
}
/*********************************************************************
 * @fn      RTLSPassive_enqueueMsg
 *
 * @design /ref 159098678
 *
 * @brief   Enqueue a message to RTLS Passive task function
 *
 * @param   pMsg - pointer to a message
 * @param   eventId - needed to send message to correct handler
 *
 * @return  none
 */
void RTLSPassive_enqueueAppMsg(uint16_t eventId, uint8_t *pMsg)
{
  rtlsPassiveEvt_t *qMsg;
  volatile uint32 keyHwi;

  keyHwi = Hwi_disable();
  if((qMsg = (rtlsPassiveEvt_t *)malloc(sizeof(rtlsPassiveEvt_t))))
  {
    qMsg->event = eventId;
    qMsg->pData = pMsg;

    Util_enqueueMsg(appMsgQueue, syncAppEvent, (uint8_t *)qMsg);
  }
  else
  {
    // Free pMsg if we failed to enqueue
    if (pMsg)
    {
      free(pMsg);
    }
  }
  Hwi_restore(keyHwi);
}

/*********************************************************************
 Setup and Run Required High Priority RTOS Task for ubStack functionality
 *********************************************************************/

/*********************************************************************
 * @fn      RTLSPassive_stack_init
 *
 * @brief   Initialize ubtTask.
 *
 * @param   none
 *
 * @return  none
 */
void RTLSPassive_stack_init(void)
{
  Task_Params stackTaskParams;

  // Configure Stack task
  Task_Params_init(&stackTaskParams);
  stackTaskParams.stack = ubtTaskStack;
  stackTaskParams.stackSize = UBT_TASK_STACK_SIZE;
  stackTaskParams.priority = UBT_TASK_PRIORITY;
  Task_construct(&ubtTask, ubStack_taskFxn, &stackTaskParams, NULL);
}

/*********************************************************************
 * @fn      ubStack_taskFxn
 *
 * @brief   Stack task entry point for the micro_ble.
 *
 * @param   none
 *
 * @return  none
 */
static void ubStack_taskFxn(UArg a0, UArg a1)
{
  volatile uint32 keyHwi;

  // Create an RTOS event used to wake up this application to process events.
  syncStackEvent = Event_create(NULL, NULL);

  if (syncStackEvent == NULL)
  {
    AssertHandler(RTLS_CTRL_ASSERT_CAUSE_NULL_POINTER_EXCEPT, 0);
  }

  // Create an RTOS queue for message from profile to be sent to app.
  stackMsgQueue = Util_constructQueue(&stackMsg);

  uble_stackInit(UBLE_ADDRTYPE_PUBLIC, NULL, uBLEStack_eventProxy,
                 RF_TIME_CRITICAL);

  ubCm_init(RTLSPassive_cmCb);

  for (;;)
  {
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    Event_pend(syncStackEvent, Event_Id_NONE, UBT_QUEUE_EVT, BIOS_WAIT_FOREVER);

    // If RTOS queue is not empty, process app message.
    while (!Queue_empty(stackMsgQueue))
    {
      ubtEvt_t *pMsg;

      // malloc() is not thread safe. Must disable HWI.
      keyHwi = Hwi_disable();
      pMsg = (ubtEvt_t *) Util_dequeueMsg(stackMsgQueue);
      Hwi_restore(keyHwi);

      if (pMsg)
      {
        // Only expects UBT_EVT_MICROBLESTACK from ubStack.
        if (pMsg->event == RTLS_PASSIVE_USTACK_EVT)
        {
          uble_processMsg();
        }

        // free() is not thread safe. Must disable HWI.
        keyHwi = Hwi_disable();

        // Free the space from the message.
        free(pMsg);
        Hwi_restore(keyHwi);
      }
    }
  }
}

/*********************************************************************
 * @fn      uBLEStack_eventProxy
 *
 * @brief   Required event_post for ubStack operation.
 *
 */
void uBLEStack_eventProxy(void)
{
  if (RTLSPassive_enqueueStackMsg(RTLS_PASSIVE_USTACK_EVT, 0) == FALSE)
  {
    // post event anyway when heap is out to avoid malloc error
    Event_post(syncStackEvent, UTIL_QUEUE_EVENT_ID);
  }
}

/*********************************************************************
 * @fn      RTLSPassive_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   data - message data.
 *
 * @return  TRUE or FALSE
 */
static bStatus_t RTLSPassive_enqueueStackMsg(uint16 event, uint8 data)
{
  volatile uint32 keyHwi;
  ubtEvt_t *pMsg;
  uint8_t status = FALSE;

  // malloc() is not thread safe. Must disable HWI.
  keyHwi = Hwi_disable();

  // Create dynamic pointer to message.
  pMsg = (ubtEvt_t*) malloc(sizeof(ubtEvt_t));
  if (pMsg != NULL)
  {
    pMsg->event = event;
    pMsg->data = data;

    // Enqueue the message.
    status = Util_enqueueMsg(stackMsgQueue, syncStackEvent, (uint8*) pMsg);
  }
  Hwi_restore(keyHwi);
  return status;
}

/*********************************************************************
 *********************************************************************/
