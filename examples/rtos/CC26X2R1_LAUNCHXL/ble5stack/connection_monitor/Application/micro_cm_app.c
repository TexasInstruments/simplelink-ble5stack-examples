/**************************************************************************************************
  Filename:       micro_cm_app.c

  Description:    This file contains the Connection monitor application
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

#include "micro_cm_app.h"

#include <string.h>
#include <stdlib.h>

#include "bcomdef.h"
#include <hal_assert.h>
#include "micro_ble_cm.h"

// DriverLib
#include "uble.h"
#include "ugap.h"
#include "ull.h"
#include "micro_ble_cm.h"

// RTLS
#include "rtls_ble.h"
#include "rtls_ctrl.h"
#include "rtls_ctrl_api.h"

#ifdef USE_RCL
#include <ti/drivers/rcl/RCL_Scheduler.h>
#else
#include <urfc.h>
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Stack Task configuration
#define UBT_TASK_PRIORITY                     3

#ifndef UBT_TASK_STACK_SIZE
#define UBT_TASK_STACK_SIZE                   1024
#endif

// App Task configuration
#define UCA_TASK_PRIORITY                     2

#ifndef UCA_TASK_STACK_SIZE
#define UCA_TASK_STACK_SIZE                   1024
#endif

// 4 units equals to 3ms
#define MIN_CONN_INTERVAL_CHANGE              4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/** @data structure for the thread entity */
typedef struct
{
    pthread_t  app_threadId;
    mqd_t      app_queueHandle;
    pthread_t  stack_threadId;
    mqd_t      stack_queueHandle;
    // Flag to indicate whether we are reporting to RTLS Control or not
    uint8_t    rtlsSyncEnabled;
    // CM is tracking
    uint8_t    monitorTracking[CM_MAX_SESSIONS];
} micro_cm_CtrlData_t;

micro_cm_CtrlData_t gCmCtrlData =
{
    .app_threadId           = NULL,
    .app_queueHandle        = NULL,
    .stack_threadId         = NULL,
    .stack_queueHandle      = NULL,
    .rtlsSyncEnabled        = RTLS_FALSE,
    .monitorTracking        = {RTLS_FALSE},
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void *MicroCmApp_taskFxn(void *arg);

//ubStack High Priority Task
static void *ubStack_taskFxn(void *arg);

static void uBLEStack_eventProxy(void);
static bStatus_t MicroCmApp_enqueueStackMsg(uint16 event, uint8 data);

// General message handling
static void MicroCmApp_processMicroCmAppMsg(uint8_t *pMsg);

// Handling CM events
static void MicroCmApp_processCmMsg(uint8_t *pMsg);
static void MicroCmApp_monitorIndicationEvt(uint8_t *pData);
static void MicroCmApp_monitorCompleteEvt(uint8_t *pData);
static void MicroCmApp_monitorStateChangeEvt(uint8_t *pData);

// Handling RTLS Control events
static void MicroCmApp_processRtlsCtrlMsg(uint8_t *pMsg);
static void MicroCmApp_cmStartReq(uint8_t *pConnInfo);
static void MicroCmApp_enableRtlsSync(rtlsEnableSync_t *enable);
static void MicroCmApp_terminateLinkReq(uint8_t sessionId);

uint16_t MicroCmApp_getHostConnHandle(uint8_t sessionId);
uint8_t MicroCmApp_getSessionId(uint16_t hostConnHandle);

/*********************************************************************
 * @fn      MicroCmApp_processMicroCmAppMsg
 *
 * @brief   General handling of application events
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void MicroCmApp_processMicroCmAppMsg(uint8_t *pMsg)
{
  microCmAppEvt_t *pEvt = (microCmAppEvt_t *)pMsg;
  volatile uint32_t keyHwi;

  switch(pEvt->event)
  {
    case MICRO_CM_APP_RTLS_CTRL_EVT:
    {
      MicroCmApp_processRtlsCtrlMsg((uint8_t *)pEvt->pData);
    }
    break;

    case MICRO_CM_APP_CM_EVT:
    {
      MicroCmApp_processCmMsg((uint8_t *)pEvt->pData);
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
 * @fn      MicroCmApp_processRtlsCtrlMsg
 *
 * @brief   Handle processing messages from RTLS Control
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void MicroCmApp_processRtlsCtrlMsg(uint8_t *pMsg)
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
      MicroCmApp_enableRtlsSync((rtlsEnableSync_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_CONN:
    {
      MicroCmApp_cmStartReq(pReq->pData);
    }
    break;

    case RTLS_REQ_TERMINATE_LINK:
    {
      rtlsTerminateLinkReq_t *termInfo = (rtlsTerminateLinkReq_t *)pReq->pData;
      uint8_t sessionId;

      sessionId = MicroCmApp_getSessionId(termInfo->connHandle);
      if (sessionId != CM_INVALID_SESSION_ID)
      {
        MicroCmApp_terminateLinkReq(sessionId);
      }

    }
    break;

    default:
      break;
  }

  keyHwi = Hwi_disable();
  free(pReq->pData);
  Hwi_restore(keyHwi);
}

/*********************************************************************
 * @fn      MicroCmApp_processCmMsg
 *
 * @brief   Handle processing messages from Connection Monitor
 *
 * @param   pMsg - a pointer to the message
 *
 * @return  none
 */
static void MicroCmApp_processCmMsg(uint8_t *pMsg)
{
  cmEvt_t *pEvt = (cmEvt_t *)pMsg;
  volatile uint32_t keyHwi;

  //BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : CM msg status=%d, event=0x%x\n", 0, pEvt->event);
  switch(pEvt->event)
  {
    case CM_MONITOR_STATE_CHANGED_EVT:
    {
      MicroCmApp_monitorStateChangeEvt(pEvt->pData);
    }
    break;

    case CM_PACKET_RECEIVED_EVT:
    {
      MicroCmApp_monitorIndicationEvt(pEvt->pData);
    }
    break;

    case CM_CONN_EVT_COMPLETE_EVT:
    {
      MicroCmApp_monitorCompleteEvt(pEvt->pData);
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
 * @fn      MicroCmApp_enableRtlsSync
 *
 * @brief   Enable RTLS synchronization
 *
 * @param   enable - enable/disable sync
 *
 * @return  none
 */
static void MicroCmApp_enableRtlsSync(rtlsEnableSync_t *enable)
{
    gCmCtrlData.rtlsSyncEnabled = enable->enable;
}

/*********************************************************************
 * @fn      MicroCmApp_cmStartReq
 *
 * @brief   Start Monitoring a BLE connection
 *
 * @param   pConnInfo - BLE connection parameters
 *
 * @return  none
 */
static void MicroCmApp_cmStartReq(uint8_t *pConnInfo)
{
  bleConnInfo_t *pBleConnInfo = (bleConnInfo_t *)pConnInfo;
  uint8_t sessionId;

  sessionId = MicroCmApp_getSessionId(pBleConnInfo->connHandle);

  // When a session already exists, stop monitoring the connection, and try to re-sync.
  if (ubCM_isSessionActive(sessionId) == CM_SUCCESS && pBleConnInfo->accessAddr == ubCMConnInfo.ArrayOfConnInfo[sessionId-1].accessAddr)
  {
    if (CM_FAILED_TO_START == ubCM_updateExt(sessionId, pBleConnInfo->connHandle,
                                                        pBleConnInfo->accessAddr,
                                                        pBleConnInfo->connInterval,
                                                        pBleConnInfo->hopValue,
                                                        pBleConnInfo->currChan,
                                                        pBleConnInfo->chanMap,
                                                        pBleConnInfo->crcInit))
    {
      //connection interval has changed
      // In case :
      // 1) the CM didn't meet the first packet,
      // 2) the new connection interval event close ~ 3 ms ( 4 units ) from the past connection interval (for the same access address, close interval might catch the events after changing
      //    the connection param update with the same interval).
      // 3) there  was any param update request
      // then don't take into consideration the miss events from the last param update because they might be not relevant.
      if (ubCMConnInfo.ArrayOfConnInfo[sessionId-1].timeStampCentral == 0                       ||
         (MIN(pBleConnInfo->connInterval,ubCMConnInfo.ArrayOfConnInfo[sessionId-1].connInterval) + MIN_CONN_INTERVAL_CHANGE >=
          MAX(pBleConnInfo->connInterval,ubCMConnInfo.ArrayOfConnInfo[sessionId-1].connInterval)) ||
         (ubCM_isAnchorRelevant(ubCMConnInfo.ArrayOfConnInfo[sessionId-1].timeStampCentral) == FALSE)) // check if there was a param update since the last anchor from other handle
      {
        gUpdateSessionMissEvents[ubCMConnInfo.ArrayOfConnInfo[sessionId-1].hostConnHandle] = 0;
      }
      else
      {
        // calculate the number of events missed due to the param update.
        uint32_t currTime = ull_getCurrentTime();
        uint32_t deltaTime = uble_timeDelta(currTime, ubCMConnInfo.ArrayOfConnInfo[sessionId-1].timeStampCentral);
        gUpdateSessionMissEvents[ubCMConnInfo.ArrayOfConnInfo[sessionId-1].hostConnHandle] = (uint16_t) (deltaTime / (pBleConnInfo->connInterval * BLE_TO_RAT) + 1);
      }

      ubCM_startNewSession(pBleConnInfo->connHandle,
                           pBleConnInfo->accessAddr,
                           pBleConnInfo->connRole,
                           pBleConnInfo->connInterval,
                           pBleConnInfo->hopValue,
                           pBleConnInfo->currChan,
                           pBleConnInfo->chanMap,
                           pBleConnInfo->crcInit);

      // stop the current session after creating the new one with the new parameters
      ubCM_stop(sessionId);
      gCmCtrlData.monitorTracking[sessionId - 1] = RTLS_FALSE;
    }
  }
  else
  {

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

}

/*********************************************************************
 * @fn      MicroCmApp_terminateLinkReq
 *
 * @brief   Stop Monitoring a BLE connection
 *
 * @return  none
 */
static void MicroCmApp_terminateLinkReq(uint8_t sessionId)
{
  uint16_t hostConnHandle;

  // Get connection handle before ubCM_stop
  hostConnHandle = MicroCmApp_getHostConnHandle(sessionId);

  ubCM_stop(sessionId);

  gCmCtrlData.monitorTracking[sessionId - 1] = RTLS_FALSE;

  // Link terminated
  if (RTLS_CONNHANDLE_INVALID != hostConnHandle)
  {
    RTLSCtrl_connResultEvt(hostConnHandle, RTLS_LINK_TERMINATED);
  }
}

/*********************************************************************
 * @fn      MicroCmApp_monitorIndicationEvt
 *
 * @brief   This function will be called for each BLE packet received
 *
 * @param   pData - Packet payload
 *
 * @return  none
 */
static void MicroCmApp_monitorIndicationEvt(uint8_t *pData)
{
  volatile uint32_t keyHwi;
  if (pData != NULL)
  {
    packetReceivedEvt_t *pPacketInfo = (packetReceivedEvt_t *)pData;
    switch(pPacketInfo->status)
    {
      case SUCCESS:
      {
        if (pPacketInfo->pPayload != NULL)
        {
          keyHwi = Hwi_disable();
          free(pPacketInfo->pPayload);
          Hwi_restore(keyHwi);
        }
        break;
      }

      case MSG_BUFFER_NOT_AVAIL:
      {
        break;
      }

      default:
        break;
    }
  }
}

/*********************************************************************
 * @fn      MicroCmApp_monitorCompleteEvt
 *
 * @brief   A single connection event has ended
 *
 * @param   pData - Session Id and Status of the session
 *
 * @return  none
 */
static void MicroCmApp_monitorCompleteEvt(uint8_t *pData)
{
  monitorCompleteEvt_t *pCompleteEvt = (monitorCompleteEvt_t *)pData;
  uint32_t nextStartTime;
  uint32_t currentTime;
  uint32_t nextEventTimeUs;
  int8_t responderRssi;
  uint8_t channel;
  rtlsStatus_e status;
  uint16_t hostConnHandle;
  volatile port_key_t key;
  volatile port_key_t key_h;

  // Convert CM Status to RTLS Status
  if (pCompleteEvt->status != CM_FAILED_NOT_ACTIVE)
  {
    if (gCmCtrlData.monitorTracking[pCompleteEvt->sessionId - 1] == RTLS_FALSE)
    {
      hostConnHandle = MicroCmApp_getHostConnHandle(pCompleteEvt->sessionId);
      if (hostConnHandle == RTLS_CONNHANDLE_INVALID)
      {
        return;
      }
      RTLSCtrl_connResultEvt(hostConnHandle, RTLS_SUCCESS);
      gCmCtrlData.monitorTracking[pCompleteEvt->sessionId - 1] = RTLS_TRUE;
    }
    status = RTLS_SUCCESS;
  }
  else if (pCompleteEvt->status == CM_FAILED_NOT_ACTIVE)
  {
    // This connection was lost
    MicroCmApp_terminateLinkReq(pCompleteEvt->sessionId);
    status = RTLS_FAIL;
  }

  // The timing of the next event is critical
  // Enter CS
  key_h = port_enterCS_HW();
  key = port_enterCS_SW();


  if (ubCM_isSessionActive(pCompleteEvt->sessionId) == CM_SUCCESS &&
      gCmCtrlData.monitorTracking[pCompleteEvt->sessionId - 1] == RTLS_TRUE &&
      gCmCtrlData.rtlsSyncEnabled && status != RTLS_FAIL && pCompleteEvt->status == MONITOR_SUCCESS)
  {
    ubCM_ConnInfo_t connInfo = ubCMConnInfo.ArrayOfConnInfo[pCompleteEvt->sessionId - 1];

    // Get the next start time
    nextStartTime = connInfo.nextStartTime;

    currentTime = ull_getCurrentTime();
    nextEventTimeUs = ull_convertRatTicksToUs(nextStartTime - currentTime);

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

    if (responderRssi != CM_RSSI_NOT_AVAILABLE)
    {
      RTLSCtrl_syncNotifyEvt(hostConnHandle, status, nextEventTimeUs, responderRssi, channel);
    }
    else
    {
        //don't send the notify sync
    }
  }

  // Exit CS
  port_exitCS_SW(key);
  port_exitCS_HW(key_h);
}

/*********************************************************************
 * @fn      MicroCmApp_getHostConnHandle
 *
 * @brief   get hostConnHandle from sessionId using ubCMConnInfo data base
 *
 * @param   sessionId - Session Id
 *
 * @return  hostConnHandle or 0xFF in case of not active session Id
 */
uint16_t MicroCmApp_getHostConnHandle(uint8_t sessionId)
{
  if (ubCM_isSessionActive(sessionId) != CM_SUCCESS)
  {
    return RTLS_CONNHANDLE_INVALID;
  }
  return ubCMConnInfo.ArrayOfConnInfo[sessionId - 1].hostConnHandle;
}

/*********************************************************************
 * @fn      MicroCmApp_getSessionId
 *
 * @brief   get sessionId from hostConnHandle using a search on ubCMConnInfo data base
 *
 * @param   hostConnHandle - host connection handle
 *
 * @return  sessionId or CM_INVALID_SESSION_ID in case of not found
 */
uint8_t MicroCmApp_getSessionId(uint16_t hostConnHandle)
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
 * @fn      MicroCmApp_monitorStateChangeEvt
 *
 * @brief   This function will be called for each BLE CM state change
 *
 * @param   pData - a pointer to the message
 *
 * @return  none
 */
static void MicroCmApp_monitorStateChangeEvt(uint8_t *pData)
{
  ugapMonitorState_t* pNewState = (ugapMonitorState_t *)pData;
  switch (*pNewState)
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
 *  @fn      MicroCmApp_init
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
void MicroCmApp_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", UCA_TASK_PRIORITY);

  // Create an RTOS queue for message from profile to be sent to app.
  RTLSCtrl_createPQueue(&gCmCtrlData.app_queueHandle,
                        "MicroCmApp_CmAppQueue",
                         UCA_QUEUE_SIZE,
                         sizeof(microCmAppEvt_t),
                         0 /* Blocking */);

  char *ucaTaskStack;
  ucaTaskStack = (char *) RTLSCtrl_malloc( UCA_TASK_STACK_SIZE );

  RTLSCtrl_createPTask(&gCmCtrlData.app_threadId,
                        MicroCmApp_taskFxn,
                        UCA_TASK_PRIORITY,
                        ucaTaskStack,
                        UCA_TASK_STACK_SIZE);
}

/*********************************************************************
 * @fn      MicroCmApp_taskFxn
 *
 * @brief   Main CM application RTOS task loop, handles new CM command requests
 *          and continous monitor calls.
 *
 * @param   a0 - Standard TI RTOS taskFxn arguments.
 * @param   a1 -
 *
 * @return  none
 */
static void *MicroCmApp_taskFxn(void *arg)
{
  microCmAppEvt_t appEvt;

  for(;;)
  {
    // If RTOS queue is not empty, process app message.
    if (mq_receive(gCmCtrlData.app_queueHandle, (char*)&appEvt, sizeof(appEvt), NULL) == sizeof(appEvt))
    {
      // Process message.
      MicroCmApp_processMicroCmAppMsg((uint8_t *)&appEvt);
    }
  }
}

/*********************************************************************
 * @fn      MicroCmApp_rtlsCtrlMsgCb
 *
 * @brief   Callback given to RTLS Control
 *
 * @param  cmd - the command to be enqueued
 *
 * @return  none
 */
void MicroCmApp_rtlsCtrlMsgCb(uint8_t *pCmd)
{
  // Enqueue the message to switch context
  MicroCmApp_enqueueAppMsg(MICRO_CM_APP_RTLS_CTRL_EVT, (uint8_t *)pCmd);
}

/*********************************************************************
 * @fn      MicroCmApp_cmCb
 *
 * @brief   Callback given to Connection Monitor
 *
 * @param   pCmd - the command to be enqueued
 *
 * @return  none
 */
void MicroCmApp_cmCb(uint8_t *pCmd)
{
  // Enqueue the message to switch context
  MicroCmApp_enqueueAppMsg(MICRO_CM_APP_CM_EVT, (uint8_t *)pCmd);
}

/*********************************************************************
 * @fn      MicroCmApp_enqueueMsg
 *
 * @design /ref 159098678
 *
 * @brief   Enqueue a message to connection monitor task function
 *
 * @param   pMsg - pointer to a message
 * @param   eventId - needed to send message to correct handler
 *
 * @return  none
 */
void MicroCmApp_enqueueAppMsg(uint16_t eventId, uint8_t *pMsg)
{
  microCmAppEvt_t AppEvt;

  AppEvt.event = eventId;
  AppEvt.pData = pMsg;

  mq_send(gCmCtrlData.app_queueHandle, (char*)&AppEvt, sizeof(AppEvt), 1);
}

/*********************************************************************
 Setup and Run Required High Priority RTOS Task for ubStack functionality
 *********************************************************************/

/*********************************************************************
 * @fn      MicroCmApp_stack_init
 *
 * @brief   Initialize micro_cm_Stack threadId.
 *
 * @param   none
 *
 * @return  none
 */
void MicroCmApp_stack_init(void)
{
  char *ubtTaskStack;
  ubtTaskStack = (char *) RTLSCtrl_malloc( UBT_TASK_STACK_SIZE );

  RTLSCtrl_createPTask(&gCmCtrlData.stack_threadId,
                        ubStack_taskFxn,
                        UBT_TASK_PRIORITY,
                        ubtTaskStack,
                        UBT_TASK_STACK_SIZE);
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
static void *ubStack_taskFxn(void *arg)
{
  // Create an RTOS queue for message from profile to be sent to app.
  RTLSCtrl_createPQueue(&gCmCtrlData.stack_queueHandle,
                        "MicroCmApp_CmStackQueue",
                         UBT_QUEUE_SIZE,
                         sizeof(ubtEvt_t),
                         0 /* Blocking */);

  uble_stackInit(UBLE_ADDRTYPE_PUBLIC, NULL, uBLEStack_eventProxy,
                 RF_TIME_CRITICAL);

  ubCm_init(MicroCmApp_cmCb);

  // If RTOS queue is not empty, process app message.
  ubtEvt_t ubtEvt;
  for(;;)
  {
    if (mq_receive(gCmCtrlData.stack_queueHandle, (char*)&ubtEvt, sizeof(ubtEvt), NULL) == sizeof(ubtEvt))
    {
      // Only expects UBT_EVT_MICROBLESTACK from ubStack.
      if (ubtEvt.event == MICRO_CM_APP_USTACK_EVT)
      {
        uble_processMsg();
      }
    }
  }
}

/*********************************************************************
 * @fn      uBLEStack_eventProxy
 *
 * @brief   Required for ubStack operation.
 *
 */
void uBLEStack_eventProxy(void)
{
  MicroCmApp_enqueueStackMsg(MICRO_CM_APP_USTACK_EVT, 0);
}

/*********************************************************************
 * @fn      MicroCmApp_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   data - message data.
 *
 * @return  TRUE or FALSE
 */
static bStatus_t MicroCmApp_enqueueStackMsg(uint16 event, uint8 data)
{
  ubtEvt_t ubtEvt;
  uint8_t status = FALSE;

  ubtEvt.event = event;
  ubtEvt.data = data;

  // Enqueue the message.
  status = mq_send(gCmCtrlData.stack_queueHandle, (char*)&ubtEvt, sizeof(ubtEvt), 1);

  return status;
}

/*********************************************************************
 *********************************************************************/
