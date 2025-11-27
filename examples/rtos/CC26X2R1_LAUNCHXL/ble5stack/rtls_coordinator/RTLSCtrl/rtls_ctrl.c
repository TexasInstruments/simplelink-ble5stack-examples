/******************************************************************************

 @file  rtls_ctrl.c

 @brief This file contains all functions and definitions related to RTLS Control

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

#include "util.h"
#include "rtls_host.h"
#include "rtls_ctrl.h"
#include "rtls_ctrl_api.h"
#include "onboard.h"

#ifdef RTLS_CTE
#include "rtls_ctrl_aoa.h"
#endif /* RTLS_CTE */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// RTLS Control Remote Command to Responder opcodes
#define RTLS_REMOTE_CMD_RESERVED1          0x01
#define RTLS_REMOTE_CMD_RESERVED2          0x02
#define RTLS_REMOTE_CMD_RESERVED3          0x03
#define RTLS_REMOTE_CMD_RESERVED4          0x04
#define RTLS_REMOTE_CMD_RESERVED5          0x05

// The maximum value for alpha in the RSSI filter
#define RTLS_CTRL_ALPHA_FILTER_MAX_VALUE  16

// The larger this number is, the effect which the last
// sample will have on RSSI is greater
#define RTLS_CTRL_ALPHA_FILTER_VALUE      4

// Initial RSSI value for the alpha filter (first dummy sample)
#define RTLS_CTRL_FILTER_INITIAL_RSSI     -55

// RSSI check
#define RTLS_IS_VALID_RSSI(rssi)          ((rssi) < 127 && (rssi > -127))
#define RSSI_NOT_AVAILABLE                0x7F  // report to user

// Max string length on debug event
#define DEBUG_STRING_SIZE       64

/*********************************************************************
 * TYPEDEFS
 */

// RTLS Control states
typedef enum
{
  RTLS_STATE_CONNECTED          = 0x00000001,
  RTLS_STATE_RESERVED           = 0x00000002,
  RTLS_STATE_AOA_ENABLED        = 0x00000004,
  RTLS_STATE_CONN_INFO_ENABLED  = 0x00000008,
} rtlsConnState_e;

// RSSI alpha filter structure
typedef struct
{
  int8_t currentRssi;
  uint8_t alphaValue;
} rssiAlphaFilter_t;

// RTLS Run Event
typedef struct
{
  uint16_t connHandle;
  uint8_t status;
  uint32_t timeToNextEvent;
  int8_t rssi;
  uint8_t channel;
} rtlsRunEvt_t;

// RTLS Connection Info Event
typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  int8_t rssi;
  uint8_t channel;
} rtlsConnInfoEvt_t;

// RTLS Connection Status changed event
typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  uint8_t status;
} rtlsConnStatusEvt_t;

// RTLS Control Data Structures
typedef struct __attribute__((packed))
{
  rtlsCapabilities_e capab;             // Capabilities
  uint16_t revNum;                      // Revision
  uint8_t devId;                        // Device ID
  uint8_t identifier[CHIP_ID_SIZE];     // Unique identifier
  uint8_t maxNumConns;                  // Maximum number of supported connections
} rtlsCapabilities_t;

// General data structure used for various RTLS Control operations
typedef struct
{
  pthread_t threadId;
  mqd_t     queueHandle;
  uint16_t  termSyncHandle;
  pfnRtlsAppCb appCb;                   // RTLS Control callback to RTLS Application
#ifdef RTLS_CTE
  rtlsAoa_t aoaControlBlock;            // This contains all AoA information
#endif /* RTLS_CTE */
  rtlsConnState_e *connStateBm;         // State of the connection managed by the RTLS Application
  rtlsCapabilities_t rtlsCapab;         // Capabilities of the device
  rssiAlphaFilter_t rssiFilter;         // RSSI value gathered from different sources
  uint8_t numActiveConns;               // Number of currently active connections
  uint8_t syncEnabled;                  // We are receiving sync events from RTLS Aplication
} rtlsCtrlData_t;

// RTLS Control message types
typedef enum
{
  HOST_MSG_EVENT,
  RTLS_RUN_EVENT,
  AOA_RESULTS_EVENT
} rtlsEvtType_e;

// RTLS Control RTOS Events
typedef struct
{
  rtlsEvtType_e event; // Event Id
  uint8_t *pData;      // Pointer to the data
} rtlsEvt_t;

typedef struct __attribute__((packed))
{
  uint32_t debug_value;
  uint8_t  debug_string[DEBUG_STRING_SIZE];
} debugInfo_t;

// Set RTLS param request
typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  uint8_t rtlsParamType;
  uint8_t dataLen;
  uint8_t data[];
} setRtlsParamRequest_t;

// Set RTLS param response
typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  uint8_t rtlsParamType;
  uint8_t status;
} setRtlsParamResponse_t;

typedef struct __attribute__((packed))
{
  uint8_t status;
  uint8_t listSize;
} rtlsReadListSizeEvt_t;

typedef struct __attribute__((packed))
{
  uint8  opcode;
  uint8  status;
  uint16 syncHandle;
  uint8  advSid;
  uint8  advAddrType;
  uint8  advAddress[6];
  uint8  advPhy;
  uint16 periodicAdvInt;
  uint8  advClockAccuracy;
} rtlsSyncEstEvt_t;

typedef struct __attribute__((packed))
{
  uint8  opcode;
  uint16 syncHandle;
} rtlsSyncLostEvt_t;

typedef struct __attribute__((packed))
{
  uint8_t  opcode;
  uint16_t syncHandle;
  int8_t   txPower;
  int8_t   rssi;
  uint8_t  cteType;
  uint8_t  dataStatus;
  uint8_t  dataLen;
  uint8_t  pData[];
} rtlsPeriodicAdvRpt_t;

typedef struct __attribute__((packed))
{
  uint8_t  status;
  uint16_t syncHandle;
} rtlsClAoaEnableEvt_t;

#ifdef MAX_NUM_CTE_BUFS
uint8_t maxNumClCteBufs = MAX_NUM_CTE_BUFS;
#else
uint8_t maxNumClCteBufs = 1;
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */
rtlsCtrlData_t gRtlsData =
{
  .threadId                 = NULL,
  .queueHandle              = NULL,
  .appCb                    = NULL,
  .connStateBm              = (rtlsConnState_e *)0x00000000,
  .rtlsCapab.capab          = RTLS_CAP_NOT_INITIALIZED,
  .rtlsCapab.identifier     = {0},
  .rssiFilter               = {0},
  .termSyncHandle           = 0xFFFF
};

char *rtlsReq_BleLogStrings[] = {
  "RTLS_REQ_ZERO                   ",
  "RTLS_REQ_ENABLE_SYNC            ",
  "RTLS_REQ_CONN                   ",
  "RTLS_REQ_SCAN                   ",
  "RTLS_REQ_SEND_DATA              ",
  "RTLS_REQ_TERMINATE_LINK         ",
  "RTLS_REQ_SET_AOA_PARAMS         ",
  "RTLS_REQ_AOA_ENABLE             ",
  "RTLS_REQ_UPDATE_CONN_INTERVAL   ",
  "RTLS_REQ_GET_ACTIVE_CONN_INFO   ",
};

char *rtlsCmd_BleLogStrings[] = {
  "RTLS_CMD_IDENTIFY              ",
  "RTLS_CMD_RESERVED              ",
  "RTLS_CMD_CONN_PARAMS           ",
  "RTLS_CMD_CONNECT               ",
  "RTLS_CMD_SCAN                  ",
  "RTLS_CMD_SCAN_STOP             ",
  "RTLS_CMD_TOF_RESULT_DIST       ",
  "RTLS_CMD_TOF_RESULT_STAT       ",
  "RTLS_CMD_TOF_RESULT_RAW        ",
  "RTLS_CMD_TOF_SET_SEC_SEED      ",
  "RTLS_CMD_UNKNOWN_0x0A          ",
  "RTLS_CMD_UNKNOWN_0x0B          ",
  "RTLS_CMD_UNKNOWN_0x0C          ",
  "RTLS_CMD_UNKNOWN_0x0D          ",
  "RTLS_CMD_UNKNOWN_0x0E          ",
  "RTLS_CMD_UNKNOWN_0x0F          ",
  "RTLS_CMD_TOF_GET_SEC_SEED      ",
  "RTLS_CMD_TOF_SET_PARAMS        ",
  "RTLS_CMD_TOF_ENABLE            ",
  "RTLS_CMD_AOA_SET_PARAMS        ",
  "RTLS_CMD_AOA_ENABLE            ",
  "RTLS_CMD_UNKNOWN_0x15          ",
  "RTLS_CMD_UNKNOWN_0x16          ",
  "RTLS_CMD_UNKNOWN_0x17          ",
  "RTLS_CMD_UNKNOWN_0x18          ",
  "RTLS_CMD_UNKNOWN_0x19          ",
  "RTLS_CMD_UNKNOWN_0x1A          ",
  "RTLS_CMD_UNKNOWN_0x1B          ",
  "RTLS_CMD_UNKNOWN_0x1C          ",
  "RTLS_CMD_UNKNOWN_0x1D          ",
  "RTLS_CMD_UNKNOWN_0x1E          ",
  "RTLS_CMD_UNKNOWN_0x1F          ",
  "RTLS_CMD_RESET_DEVICE          ",
  "RTLS_CMD_UNKNOWN_0x21          ",
  "RTLS_CMD_TERMINATE_LINK        ",
  "RTLS_CMD_AOA_RESULT_RAW        ",
  "RTLS_CMD_TOF_CALIBRATE         ",
  "RTLS_CMD_CONN_INFO             ",
  "RTLS_CMD_SET_RTLS_PARAM        ",
  "RTLS_CMD_GET_RTLS_PARAM        ",
  "RTLS_CMD_UNKNOWN_0x2A          ",
  "RTLS_CMD_UNKNOWN_0x2B          ",
  "RTLS_CMD_UNKNOWN_0x2C          ",
  "RTLS_CMD_UNKNOWN_0x2D          ",
  "RTLS_CMD_UNKNOWN_0x2E          ",
  "RTLS_CMD_UNKNOWN_0x2F          ",
  "RTLS_CMD_TOF_CALIB_NV_READ     ",
  "RTLS_CMD_TOF_SWITCH_ROLE       ",
  "RTLS_CMD_GET_ACTIVE_CONN_INFO  ",
};

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// RTLS Control specific
void RTLSCtrl_createTask(void);
void *RTLSCtrl_taskFxn(void *);
void RTLSCtrl_processMessage(rtlsEvt_t *pMsg);
void RTLSCtrl_enqueueMsg(uint16_t eventId, uint8_t *pMsg);

// Host Command Handlers
void RTLSCtrl_getActiveConnInfoCmd(rtlsGetActiveConnInfo_t *pReq);
void RTLSCtrl_connReqCmd(uint8_t *connParams);
void RTLSCtrl_scanReqCmd(void);
void RTLSCtrl_sendRtlsRemoteCmd(uint16_t connHandle, uint8_t cmdOp, uint8_t *pData, uint16_t dataLen);
void RTLSCtrl_terminateLinkCmd(uint8_t *connHandle);
void RTLSCtrl_enableConnInfoCmd(rtlsEnableSync_t *enableConnInfoCmd);
void RTLSCtrl_periodicAdvCreateSyncCmd(uint8_t *pParams);
void RTLSCtrl_periodicAdvSyncCancelCmd(void);
void RTLSCtrl_periodicAdvTerminateSyncCmd(uint8_t *syncHandle);
void RTLSCtrl_periodicAdvReceiveEnableCmd(uint8_t *pParams);
void RTLSCtrl_addDeviceToPeriodicAdvListCmd(uint8_t *pParams);
void RTLSCtrl_removeDeviceFromPeriodicAdvListCmd(uint8_t *pParams);
void RTLSCtrl_readPeriodicAdvListSizeCmd( void );
void RTLSCtrl_clearPeriodicAdvListCmd( void );
void RTLSCtrl_processSyncLost( void );
#ifdef RTLS_CTE
void RTLSCtrl_setAoaParamsCmd(uint8_t *pParams);
void RTLSCtrl_enableAoaCmd(uint8_t *enableAoaCmd);
void RTLSCtrl_CLAoaEnableCmd(uint8_t *pParams);
#endif /* RTLS_CTE*/

// Internal functions
void RTLSCtrl_processHostMessage(rtlsHostMsg_t *pHostMsg);
void RTLSCtrl_hostMsgCB(rtlsHostMsg_t *pMsg);
void RTLSCtrl_callRtlsApp(uint8_t reqOp, uint8_t *data);
rtlsStatus_e RTLSCtrl_processSyncEvent(uint8_t *pMsg);
rtlsStatus_e RTLSCtrl_updateConnState(rtlsConnState_e connState, uint8_t enableDisableFlag, uint16_t connHandle);
rtlsStatus_e RTLSCtrl_updateConnInterval(uint16_t connHandle, uint8_t dataLen, uint8_t *pMsg);

// Board specific
void RTLSCtrl_resetDevice(void);

// RSSI Trigger specific
void RTLSCtrl_calculateRSSI(int lastRssi);

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * @fn      RTLSCtrl_open
 *
 * @design /ref 159098678
 *
 * @brief   RTLS Initialization function from application side
 *          Application needs to specify the mandatory function
 *          required by RTLS Control
 *
 * @param   appCBs - Struct already filled by the application
 * @param   ctrlCBs - Will be filled by RTLS Control
 *
 * @return  none
 */
void RTLSCtrl_open(rtlsConfiguration_t *rtlsConfig)
{
  // Open Host I/F
  RTLSHost_openHostIf(&RTLSCtrl_hostMsgCB);

  gRtlsData.appCb = rtlsConfig->rtlsAppCb;
  gRtlsData.rtlsCapab.devId = rtlsConfig->devId;
  gRtlsData.rtlsCapab.revNum = rtlsConfig->revNum;
  gRtlsData.rtlsCapab.capab = rtlsConfig->rtlsCapab;

  memcpy(gRtlsData.rtlsCapab.identifier, rtlsConfig->identifier, CHIP_ID_SIZE);

  // Allocate space for connection state and save maximum number of connections
  gRtlsData.connStateBm = RTLSCtrl_malloc(sizeof(rtlsConnState_e) * rtlsConfig->maxNumConns);
  memset(gRtlsData.connStateBm, 0, sizeof(rtlsConnState_e) * rtlsConfig->maxNumConns);

  // Save maximum number of supported connections
  gRtlsData.rtlsCapab.maxNumConns = rtlsConfig->maxNumConns;

  // Create RTLS Control task
  RTLSCtrl_createTask();
}

/*********************************************************************
 * @fn      RTLSCtrl_scanResultEvt
 *
 * @design /ref 159098678
 *
 * @brief   Application will call this function once scan results have
 *          been collected
 *
 * @param   scanResults - Pointer to the scan results
 * @param   size - size of scanResult pointer
 *
 * @return  none
 */
void RTLSCtrl_scanResultEvt(rtlsStatus_e status, uint8_t *scanResult, uint8_t size)
{
  rtlsStatus_e scanStatus = status;

  if (scanResult == NULL)
  {
    RTLSHost_sendMsg(RTLS_CMD_SCAN_STOP, HOST_ASYNC_RSP, (uint8_t *)&scanStatus, sizeof(uint8_t));
  }
  else
  {
    RTLSHost_sendMsg(RTLS_CMD_SCAN, HOST_ASYNC_RSP, scanResult, size);
  }
}

/*********************************************************************
 * @fn      RTLSCtrl_connResultEvt
 *
 * @design /ref 159098678
 *
 * @brief   RTLS Control will use this function to notify Node Manager that
 *          a connection has been formed
 *
 * @param   connHandle - Connection Handle
 * @param   status - Connection successful or not
 *
 * @return  none
 */
void RTLSCtrl_connResultEvt(uint16_t connHandle, uint8_t status)
{
  rtlsConnStatusEvt_t connStatus;

  connStatus.connHandle = connHandle;
  connStatus.status = status;

  // If connection handle is valid we can take care of it
  // If it's not valid then we just output the status to the host
  if (connHandle != RTLS_CONNHANDLE_ALL && connHandle != RTLS_CONNHANDLE_INVALID)
  {
    if (status == RTLS_SUCCESS)
    {
      if ((gRtlsData.connStateBm[connHandle] & RTLS_STATE_CONNECTED) != RTLS_STATE_CONNECTED)
      {
        gRtlsData.connStateBm[connHandle] |= RTLS_STATE_CONNECTED;
        gRtlsData.numActiveConns++;
      }
    }
    else if (status == RTLS_LINK_TERMINATED)
    {
      if(gRtlsData.connStateBm[connHandle] & RTLS_STATE_CONNECTED)
      {
        gRtlsData.numActiveConns--;
      }
      // We were disconnected, if AOA is enabled we should disable it
      if (gRtlsData.connStateBm[connHandle] & RTLS_STATE_AOA_ENABLED ||
          gRtlsData.connStateBm[connHandle] & RTLS_STATE_CONN_INFO_ENABLED)
      {
        RTLSCtrl_updateConnState((rtlsConnState_e)RTLS_STATE_CONNECTED, RTLS_FALSE, connHandle);
      }

      gRtlsData.connStateBm[connHandle] = (rtlsConnState_e)0;
    }
  }

  RTLSHost_sendMsg(RTLS_CMD_CONNECT, HOST_ASYNC_RSP, (uint8_t *)&connStatus, sizeof(rtlsConnStatusEvt_t));
}

/*********************************************************************
 * @fn      RTLSCtrl_connInfoEvt
 *
 * @design /ref 159098678
 *
 * @brief   This function will send out connection information to Node Manager
 *
 * @param   connInfo - Connection information
 * @param   connInfoLen - Length of connInfo array
 *
 * @return  none
 */
void RTLSCtrl_connInfoEvt(uint8_t *connInfo, uint16_t connInfoLen)
{
  RTLSHost_sendMsg(RTLS_CMD_CONN_PARAMS, HOST_ASYNC_RSP, connInfo, connInfoLen);
}

/*********************************************************************
 * @fn      RTLSCtrl_syncNotifyEvt
 *
 * @design /ref 159098678
 *
 * @brief   Application will call this function on each sync event
 *
 * @param   connHandle - connection handle
 * @param   status - the status of the sync event (tells us if the RF has received a sync packet)
 * @param   timeToNextEvent - the time to the next sync event
 * @param   rssi - current rssi at the time of the sync event
 * @param   channel - channel on which the sync event was received
 *
 * @return  none
 */
void RTLSCtrl_syncNotifyEvt(uint16_t connHandle, rtlsStatus_e status, uint32_t timeToNextEvent, int8_t rssi, uint8_t channel)
{
  rtlsRunEvt_t *pMsg;

  if ((pMsg = (rtlsRunEvt_t *)RTLSCtrl_malloc(sizeof(rtlsRunEvt_t))) == NULL)
  {
    // We failed to allocate, host was already notified, just exit
    return;
  }

  pMsg->connHandle = connHandle;
  pMsg->status = status;
  pMsg->timeToNextEvent = timeToNextEvent;
  pMsg->rssi = rssi;
  pMsg->channel = channel;

  RTLSCtrl_enqueueMsg(RTLS_RUN_EVENT, (uint8_t *)pMsg);
}

/*********************************************************************
 * @fn      RTLSCtrl_dataSentEvt
 *
 * @design /ref 159098678
 *
 * @brief   This function is used by the RTLS Application to report the status of a sent packet
 *
 * @param   connHandle - connection handle
 * @param   status - Packet transmitted/failed
 *
 * @return  none
 */
void RTLSCtrl_dataSentEvt(uint16_t connHandle, uint8_t status)
{
}

/*********************************************************************
 * @fn      RTLSCtrl_processSyncEvent
 *
 * @design /ref 159098678
 *
 * @brief   Application will call this function on each sync event
 *
 * @param   timeToNextEvent - the time to the next sync event
 *
 * @return  RTLS status
 */
rtlsStatus_e RTLSCtrl_processSyncEvent(uint8_t *pMsg)
{
  rtlsRunEvt_t *runEvt = (rtlsRunEvt_t *)pMsg;

  // Sanity check
  if (pMsg == NULL)
  {
    return RTLS_FAIL;
  }

  if (RTLS_IS_VALID_RSSI(runEvt->rssi))
  {
    RTLSCtrl_calculateRSSI(runEvt->rssi);

    if (gRtlsData.connStateBm[runEvt->connHandle] & RTLS_STATE_CONN_INFO_ENABLED)
    {
        rtlsConnInfoEvt_t connInfoEvt;
        connInfoEvt.connHandle = runEvt->connHandle;
        connInfoEvt.rssi = runEvt->rssi;
        connInfoEvt.channel = runEvt->channel;

        RTLSHost_sendMsg(RTLS_EVT_CONN_INFO, HOST_ASYNC_RSP, (uint8_t *)&connInfoEvt, sizeof(rtlsConnInfoEvt_t));
    }
  }
  else if ((gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_PASSIVE || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_CONNECTION_MONITOR) &&
           (runEvt->rssi == RSSI_NOT_AVAILABLE))
  {
    if (gRtlsData.connStateBm[runEvt->connHandle] & RTLS_STATE_CONN_INFO_ENABLED)
    {
      rtlsConnInfoEvt_t connInfoEvt;
      connInfoEvt.connHandle = runEvt->connHandle;
      connInfoEvt.rssi = runEvt->rssi;
      connInfoEvt.channel = runEvt->channel;

      RTLSHost_sendMsg(RTLS_EVT_CONN_INFO, HOST_ASYNC_RSP, (uint8_t *)&connInfoEvt, sizeof(rtlsConnInfoEvt_t));
    }
  }

  return (RTLS_SUCCESS);
}

/*********************************************************************
 * @fn      RTLSCtrl_scanReqCmd
 *
 * @design /ref 159098678
 *
 * @brief   Handles a scan request from RTLS Node Manager
 *          Once the request is received, RTLS Control will call the
 *          registered application's scan function and notify Node Manager
 *          that a scan has started
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCtrl_scanReqCmd(void)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  RTLSHost_sendMsg(RTLS_CMD_SCAN, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(uint8_t));

  RTLSCtrl_callRtlsApp(RTLS_REQ_SCAN, NULL);
}

/*********************************************************************
 * @fn      RTLSCtrl_connReqCmd
 *
 * @design /ref 159098678
 *
 * @brief   RTLS Control will use this function when Node Manager asks to
 *          form a connection
 *
 * @param   connParams - Pointer to connection parameters
 *
 * @return  none
 */
void RTLSCtrl_connReqCmd(uint8_t *connParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  if (gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_COORDINATOR || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_RESPONDER)
  {
    RTLSHost_sendMsg(RTLS_CMD_CONNECT, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
  }
  else if (gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_PASSIVE || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_CONNECTION_MONITOR)
  {
    // For a connection monitor the command is a bit different since it contains not only
    // the address to connect to but also different stack specific parameters that allow tracking
    RTLSHost_sendMsg(RTLS_CMD_CONN_PARAMS, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
  }

  RTLSCtrl_callRtlsApp(RTLS_REQ_CONN, connParams);
}

/*********************************************************************
 * @fn      RTLSCtrl_updateConnInterval
 *
 * @design /ref 159098678
 *
 * @brief   RTLS Control will use this function when Node Manager asks to
 *          dynamically change the connection interval (mSec)
 *
 * @param   connHandle - Connection handle
 * @param   dataLen - length of msg
 * @param   pMsg - Pointer to connection interval (mSec)
 *
 * @return  rtlsStatus_e
 */
rtlsStatus_e RTLSCtrl_updateConnInterval(uint16_t connHandle, uint8_t dataLen, uint8_t *pMsg)
{
  rtlsStatus_e status = RTLS_FAIL;
  rtlsUpdateConnIntReq_t *pConnIntUpdate;

  if ((gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_COORDINATOR) && pMsg != NULL)
  {
    if (gRtlsData.connStateBm[connHandle] & RTLS_STATE_CONNECTED)
    {
      if ((pConnIntUpdate = RTLSCtrl_malloc(sizeof(rtlsUpdateConnIntReq_t))) == NULL)
      {
        // RTLSCtrl_malloc handles sending error codes to the host
        return status;
      }

      // Copy param data
      pConnIntUpdate->connHandle = connHandle;
      memcpy(&pConnIntUpdate->connInterval, pMsg, dataLen);

      // Send param to RTLS Application
      RTLSCtrl_callRtlsApp(RTLS_REQ_UPDATE_CONN_INTERVAL, (uint8_t *)pConnIntUpdate);
      status = RTLS_SUCCESS;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      RTLSCtrl_terminateLinkCmd
 *
 * @design /ref 159098678
 *
 * @brief   Terminate active link
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
void RTLSCtrl_terminateLinkCmd(uint8_t *connHandle)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  RTLSHost_sendMsg(RTLS_CMD_TERMINATE_LINK, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(status));

  RTLSCtrl_callRtlsApp(RTLS_REQ_TERMINATE_LINK, connHandle);
}

/*********************************************************************
 * @fn      RTLSCtrl_enableConnInfoCmd
 *
 * @design /ref 159098678
 *
 * @brief   Enable report of RSSI and channels of BLE connection
 *
 * @param   enableConnInfoCmd - enable parameters
 *
 * @return  none
 */
void RTLSCtrl_enableConnInfoCmd(rtlsEnableSync_t *enableConnInfoCmd)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Sanity check
  if (enableConnInfoCmd == NULL)
  {
    status = RTLS_FAIL;
    RTLSHost_sendMsg(RTLS_CMD_CONN_INFO, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
    return;
  }

  // Update RTLS connection state
  if (RTLSCtrl_updateConnState((rtlsConnState_e)RTLS_STATE_CONN_INFO_ENABLED, enableConnInfoCmd->enable, enableConnInfoCmd->connHandle) == RTLS_FAIL)
  {
    // We failed to allocate, host was already notified, just exit
    return;
  }

  RTLSHost_sendMsg(RTLS_CMD_CONN_INFO, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_getActiveConnInfoCmd
 *
 * @design /ref 159098678
 *
 * @brief   Request information about an active connection
 *
 * @param   pReq - Active connection information request
 *
 * @return  none
 */
void RTLSCtrl_getActiveConnInfoCmd(rtlsGetActiveConnInfo_t *pReq)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Sanity check
  if (pReq == NULL || gRtlsData.connStateBm == NULL)
  {
    status = RTLS_FAIL;
    RTLSHost_sendMsg(RTLS_CMD_GET_ACTIVE_CONN_INFO, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
    return;
  }

  // Check that we are connected before requesting
  if (gRtlsData.connStateBm[pReq->connHandle] & RTLS_STATE_CONNECTED)
  {
    RTLSCtrl_callRtlsApp(RTLS_REQ_GET_ACTIVE_CONN_INFO, (uint8_t *)pReq);
  }
  else
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"ConnHandle does not exist: ", pReq->connHandle);
    status = RTLS_FAIL;
  }

  RTLSHost_sendMsg(RTLS_CMD_GET_ACTIVE_CONN_INFO, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_updateConnState
 *
 * @design /ref 159098678
 *
 * @brief   Update RTLS connection state and enable/disable sync event
 *
 * @param   connState - RTLS control state
 * @param   enableDisableFlag - Enable/disable connState
 * @param   connHandle - connection handle
 *
 * @return  RTLS status
 */
rtlsStatus_e RTLSCtrl_updateConnState(rtlsConnState_e connState, uint8_t enableDisableFlag, uint16_t connHandle)
{
  rtlsEnableSync_t *syncReq;

  // Enable RTLS control state
  if (enableDisableFlag == RTLS_TRUE)
  {
    gRtlsData.connStateBm[connHandle] |= connState;
  }
  // Disable RTLS control state
  else
  {
    gRtlsData.connStateBm[connHandle] &= ~(connState);
  }

  // Ask the RTLS Application to trigger RTLS Control module periodically
  if ((syncReq = (rtlsEnableSync_t *)RTLSCtrl_malloc(sizeof(rtlsEnableSync_t))) == NULL)
  {
    // We failed to allocate, host was already notified, just exit
    return RTLS_FAIL;
  }

  syncReq->enable = ((enableDisableFlag == RTLS_TRUE) && (gRtlsData.numActiveConns > 0)) ? RTLS_TRUE : RTLS_FALSE;
  syncReq->connHandle = connHandle;

  // Enable/disable sync events
  RTLSCtrl_callRtlsApp(RTLS_REQ_ENABLE_SYNC, (uint8_t *)syncReq);

  gRtlsData.syncEnabled = syncReq->enable;

  return RTLS_SUCCESS;
}

/*********************************************************************
 * @fn      RTLSCtrl_softReset
 *
 * @design /ref 159098678
 *
 * @brief   SW Reset for device - debug purposes,
 *          FREERTOS implementation
 *
 * @param   none
 *
 * @return  none
 */
//extern void resetISR(void);
//
//void RTLSCtrl_softReset(void)
//{
//    __disable_irq();
//    resetISR();
//}

/*********************************************************************
 * @fn      RTLSCtrl_resetDevice
 *
 * @design /ref 159098678
 *
 * @brief   Resets device
 *
 * @param   none
 *
 * @return  none
 */

void RTLSCtrl_resetDevice(void)
{
//    RTLSCtrl_softReset();

    SystemReset();
}

#ifdef RTLS_CTE
/*********************************************************************
 * @fn      RTLSCtrl_setAoaParamsCmd
 *
 * @design /ref 159098678
 *
 * @brief   Handle configuring AoA parameters
 *
 * @param   pParams - AoA parameters
 *
 * @return  none
 */
void RTLSCtrl_setAoaParamsCmd(uint8_t *pParams)
{
  rtlsAoaParams_t *pAoaParams;
  rtlsStatus_e status = RTLS_SUCCESS;
  rtlsAoaConfigReq_t *pSetAoaConfigReq;
  uint8_t numAnt;

  // Set RTLS Ctrl parameters
  pAoaParams = (rtlsAoaParams_t *)pParams;

  // Get number of antennas
  pSetAoaConfigReq = (rtlsAoaConfigReq_t *)&pAoaParams->config;
  numAnt = pSetAoaConfigReq->numAnt;

  gRtlsData.aoaControlBlock.aoaRole = pAoaParams->aoaRole;
  gRtlsData.aoaControlBlock.resultMode = pAoaParams->resultMode;
  gRtlsData.aoaControlBlock.sampleCtrl = pSetAoaConfigReq->sampleCtrl;

  // Allocate parameters to send to application
  if ((pSetAoaConfigReq = (rtlsAoaConfigReq_t *)RTLSCtrl_malloc(sizeof(rtlsAoaConfigReq_t) + sizeof(uint8_t)*numAnt)) == NULL)
  {
    return;
  }

  // Copy config and pass to application
  memcpy(pSetAoaConfigReq, &pAoaParams->config, sizeof(rtlsAoaConfigReq_t) + sizeof(uint8_t)*numAnt);

  // Initialize AoA post processing module
  status = RTLSCtrl_initAoa(gRtlsData.rtlsCapab.maxNumConns, gRtlsData.aoaControlBlock.sampleCtrl, pSetAoaConfigReq->numAnt, pSetAoaConfigReq->pAntPattern, gRtlsData.aoaControlBlock.resultMode);
  if (status == RTLS_CONFIG_NOT_SUPPORTED)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"AoA failed to init, status = ", status);
    RTLSHost_sendMsg(RTLS_CMD_AOA_SET_PARAMS, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
    return;
  }

  // Request RTLS App to set it's own AoA parameters
  RTLSCtrl_callRtlsApp(RTLS_REQ_SET_AOA_PARAMS, (uint8_t *)pSetAoaConfigReq);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_AOA_SET_PARAMS, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_enableAoaCmd
 *
 * @design /ref 159098678
 *
 * @brief   Enable coordinator and responder
 *
 * @param   enableAoaCmd - enable parameters
 *
 * @return  none
 */
void RTLSCtrl_enableAoaCmd(uint8_t *enableAoaCmd)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Sanity check
  if (enableAoaCmd == NULL)
  {
    if (gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_COORDINATOR || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_PASSIVE)
    {
      status = RTLS_FAIL;
      RTLSHost_sendMsg(RTLS_CMD_AOA_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
    }
    return;
  }

  if (gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_COORDINATOR || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_PASSIVE)
  {
    RTLSHost_sendMsg(RTLS_CMD_AOA_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
  }

  if (gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_COORDINATOR || gRtlsData.rtlsCapab.capab & RTLS_CAP_RTLS_PASSIVE)
  {
    RTLSCtrl_callRtlsApp(RTLS_REQ_AOA_ENABLE, enableAoaCmd);
  }


#ifdef RTLS_PASSIVE //defined(RTLS_PASSIVE)
  rtlsAoaEnableReq_t *enable = (rtlsAoaEnableReq_t *)enableAoaCmd;

  RTLSCtrl_updateConnState((rtlsConnState_e)RTLS_STATE_AOA_ENABLED, enable->enableAoa, enable->connHandle);
#endif
}

/**
 * @brief RTLSCtrl_aoaResultEvt
 *
 * RTLS Control I/Q samples processing function
 * Results will be output to RTLS Node Manager after processing
 *
 * @param handle - connection/sync handle
 * @param rssi - rssi for this CTE
 * @param channel - channel this CTE was captured on
 * @param numIqSamples- Number of I/Q samples
 * @param sampleRate - Sampling rate that was used for the run
 * @param sampleCtrl - RAW RF mode, 1 = RAW RF, 0 = Filtered (switching omitted)
 * @param numAnt - Number of Antennas that were used for the run
 * @param pIQ - Pointer to IQ samples
 */
void RTLSCtrl_aoaResultEvt(uint16_t handle, int8_t rssi, uint8_t channel, uint16_t numIqSamples,
                           uint8_t sampleRate, uint8_t sampleSize, uint8_t sampleCtrl, uint8_t slotDuration,
                           uint8_t numAnt, int8_t *pIQ)
{
  rtlsAoaIqEvt_t *pEvt;

  // Allocate event
  if ((pEvt = (rtlsAoaIqEvt_t *)RTLSCtrl_malloc(sizeof(rtlsAoaIqEvt_t))) == NULL)
  {
    return;
  }

  // Copy data
  pEvt->handle = handle;
  pEvt->rssi = rssi;
  pEvt->channel = channel;
  pEvt->numIqSamples = numIqSamples;
  pEvt->sampleCtrl = sampleCtrl;
  pEvt->sampleRate = sampleRate;
  pEvt->sampleSize = sampleSize;
  pEvt->slotDuration = slotDuration;
  pEvt->numAnt = numAnt;
  pEvt->pIQ = pIQ;

  // Enqueue the event
  RTLSCtrl_enqueueMsg(AOA_RESULTS_EVENT, (uint8_t *)pEvt);
}

/*********************************************************************
 * @fn      RTLSCtrl_CLAoaEnableCmd
 *
 * @brief   Enable coordinator and responder
 *
 * @param   pParams - Connectionless AoA parameters
 *
 * @return  none
 */
void RTLSCtrl_CLAoaEnableCmd(uint8_t *pParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;
  rtlsCLAoaParams_t *info = (rtlsCLAoaParams_t *) pParams;
  rtlsCLAoaEnableReq_t *pEnableParams;
  uint8_t numAnt = info->clParams.numAnt;

  if( info->clParams.enable )
  {
    // Save the role, result mode, and sample control
    gRtlsData.aoaControlBlock.aoaRole = info->aoaRole;
    gRtlsData.aoaControlBlock.resultMode = info->resultMode;
    gRtlsData.aoaControlBlock.sampleCtrl = info->clParams.sampleCtrl;

    status = RTLSCtrl_clInitAoa(info->resultMode, info->clParams.sampleCtrl, info->clParams.numAnt, info->clParams.pAntPattern, info->clParams.syncHandle);

    if( status != RTLS_SUCCESS)
    {
      // Return status to the host
      RTLSHost_sendMsg(RTLS_CMD_CL_AOA_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
      return;
    }
  }

  // Allocate parameters to send to application
  if ((pEnableParams = (rtlsCLAoaEnableReq_t *)RTLSCtrl_malloc(sizeof(rtlsCLAoaEnableReq_t) + sizeof(uint8_t)*numAnt)) == NULL)
  {
    status = RTLS_OUT_OF_MEMORY;
    // Return status to the host
    RTLSHost_sendMsg(RTLS_CMD_CL_AOA_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
    return;
  }

  // Copy the enable parameters
  memcpy(pEnableParams, &info->clParams, sizeof(rtlsCLAoaEnableReq_t)+sizeof(uint8_t)*numAnt);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_CL_AOA_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));

  // Request RTLS App
  RTLSCtrl_callRtlsApp(RTLS_REQ_CL_AOA_ENABLE, (uint8_t *)pEnableParams);
}

/*********************************************************************
 * @brief RTLSCtrl_processClAoaEnableEvt
 *
 * Process CL AoA sampling enable event
 *
 * @param opcode - Command opcode
 * @paran status - Command complete status
 * @param syncHandle - Handle identifying the periodic advertising train
 *
 * @return  none
 */
void RTLSCtrl_processClAoaEnableEvt(uint16_t opcode,
                                    uint8_t status,
                                    uint16_t syncHandle)
{
  rtlsClAoaEnableEvt_t *pClAoaEnable = NULL;

  // Disabling/enabling CL CTE sampling wasn't successfull
  if( status != SUCCESS )
  {

    if((pClAoaEnable = (rtlsClAoaEnableEvt_t *)RTLSCtrl_malloc(sizeof(rtlsClAoaEnableEvt_t ))) == NULL)
    {
      return;
    }
    pClAoaEnable->status = status;
    pClAoaEnable->syncHandle = syncHandle;

    // Send the event only when an error occurs
    RTLSHost_sendMsg(RTLS_EVT_CL_AOA_ENABLE, HOST_ASYNC_RSP, (uint8_t *)pClAoaEnable, sizeof(rtlsClAoaEnableEvt_t));
  }

  if( pClAoaEnable )
  {
    RTLSUTIL_FREE(pClAoaEnable);
  }
}
#endif /* RTLS_CTE*/

/*********************************************************************
 * @fn      RTLSCtrl_callRtlsApp
 *
 * @design /ref 159098678
 *
 * @brief   Calls the callback provided by the RTLS Application
 *
 * @param   cmdOp - command to be executed
 * @param   data  - pointer to the data
 *
 * @return  none
 */
void RTLSCtrl_callRtlsApp(uint8_t reqOp, uint8_t *data)
{
  rtlsCtrlReq_t *appReq;

  if ((appReq = (rtlsCtrlReq_t *)RTLSCtrl_malloc(sizeof(rtlsCtrlReq_t))) == NULL)
  {
    // We failed to allocate, host was already notified, just exit
    return;
  }

  appReq->reqOp = reqOp;
  appReq->pData = data;

  if (gRtlsData.appCb != NULL)
  {
    gRtlsData.appCb((uint8_t *)appReq);
  }
}

/*********************************************************************
 * @fn      RTLSCtrl_sendRtlsRemoteCmd
 *
 * @design /ref 159098678
 *
 * @brief   Send a command to RTLS Responder
 *
 * @param   connHandle - Connection handle
 * @param   cmdOp - command to be executed
 * @param   data  - pointer to the data
 * @param   dataLen - length of data to send
 *
 * @return  none
 */
void RTLSCtrl_sendRtlsRemoteCmd(uint16_t connHandle, uint8_t cmdOp, uint8_t *pData, uint16_t dataLen)
{
  rtlsPacket_t *pRemoteCmd;

  // Create the RTLS remote command
  if ((pRemoteCmd = (rtlsPacket_t *)RTLSCtrl_malloc(sizeof(rtlsPacket_t) + dataLen)) == NULL)
  {
    // We failed to allocate, host was already notified, just exit
    return;
  }

  pRemoteCmd->connHandle = connHandle;
  pRemoteCmd->cmdOp = cmdOp;
  pRemoteCmd->payloadLen = sizeof(rtlsPacket_t) + dataLen;
  memcpy(pRemoteCmd->pPayload, pData, dataLen);

  RTLSCtrl_callRtlsApp(RTLS_REQ_SEND_DATA, (uint8_t *)pRemoteCmd);
}

/*********************************************************************
 * @fn      RTLSCtrl_rtlsPacketEvt
 *
 * @design /ref 159098678
 *
 * @brief   Process an incoming RTLS packet (contains a remote command)
 *          Used only by devices NOT connected via uNPI (RTLS Responder)
 *
 * @param   pPkt - Pointer to the packet
 *
 * @return  none
 */
void RTLSCtrl_rtlsPacketEvt(uint8_t *pPkt)
{
  rtlsPacket_t *pRtlsPkt = (rtlsPacket_t*)pPkt;

  switch (pRtlsPkt->cmdOp)
  {
    // Add processing of opcodes here
  }

  RTLSUTIL_FREE(pRtlsPkt);
}

/*********************************************************************
 * @fn      RTLSCtrl_processHostMessage
 *
 * @design /ref 159098678
 *
 * @brief   Process an incoming message to RTLS Control
 *
 * @param   pHostMsg - pointer to the message
 *
 * @return  none
 */
void RTLSCtrl_processHostMessage(rtlsHostMsg_t *pHostMsg)
{
  // Note that messages that stop in this module should be freed here
  // Messages that are passed to the application should NOT be freed here, they are freed by the receiver
  // Messages that do not have payload are not freed either
  if (pHostMsg->cmdType == HOST_SYNC_REQ)
  {
    if (pHostMsg->cmdId <= RTLS_CMD_BLE_LOG_STRINGS_MAX)
    {
      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : RTLS host msg cmdType=%d, cmdId=%s\n", pHostMsg->cmdType, rtlsCmd_BleLogStrings[pHostMsg->cmdId]);
    }
    else
    {
      BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLS host msg cmdType=%d, cmdId=0x%x\n", pHostMsg->cmdType, pHostMsg->cmdId);
    }

    switch(pHostMsg->cmdId)
    {
      case RTLS_CMD_IDENTIFY:
      {
        RTLSHost_sendMsg(RTLS_CMD_IDENTIFY, HOST_SYNC_RSP, (uint8_t *)&gRtlsData.rtlsCapab, sizeof(rtlsCapabilities_t));
      }
      break;

      case RTLS_CMD_SCAN:
      {
        RTLSCtrl_scanReqCmd();
      }
      break;

      case RTLS_CMD_CREATE_SYNC:
      {
        RTLSCtrl_periodicAdvCreateSyncCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_CREATE_SYNC_CANCEL:
      {
        RTLSCtrl_periodicAdvSyncCancelCmd();
      }
      break;

      case RTLS_CMD_TERMINATE_SYNC:
      {
        RTLSCtrl_periodicAdvTerminateSyncCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_PERIODIC_RECEIVE_ENABLE:
      {
        RTLSCtrl_periodicAdvReceiveEnableCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_ADD_DEVICE_ADV_LIST:
      {
        RTLSCtrl_addDeviceToPeriodicAdvListCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_REMOVE_DEVICE_ADV_LIST:
      {
        RTLSCtrl_removeDeviceFromPeriodicAdvListCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_READ_ADV_LIST_SIZE:
      {
        RTLSCtrl_readPeriodicAdvListSizeCmd();
      }
      break;

      case RTLS_CMD_CLEAR_ADV_LIST:
      {
        RTLSCtrl_clearPeriodicAdvListCmd();
      }
      break;

      case RTLS_CMD_CONNECT:
      {
        RTLSCtrl_connReqCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_TERMINATE_LINK:
      {
        RTLSCtrl_terminateLinkCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_RESET_DEVICE:
      {
        RTLSCtrl_resetDevice();
      }
      break;

#ifdef RTLS_CTE
      case RTLS_CMD_AOA_SET_PARAMS:
      {
        RTLSCtrl_setAoaParamsCmd(pHostMsg->pData);

        RTLSUTIL_FREE(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_AOA_ENABLE:
      {
        RTLSCtrl_enableAoaCmd(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_CL_AOA_ENABLE:
      {
        RTLSCtrl_CLAoaEnableCmd(pHostMsg->pData);

        RTLSUTIL_FREE(pHostMsg->pData);
      }
      break;
#endif /* RTLS_CTE */

      case RTLS_CMD_CONN_INFO:
      {
        RTLSCtrl_enableConnInfoCmd((rtlsEnableSync_t *)pHostMsg->pData);

        RTLSUTIL_FREE(pHostMsg->pData);
      }
      break;

#if defined(RTLS_PASSIVE) || defined(RTLS_CONNECTION_MONITOR)
      case RTLS_CMD_CONN_PARAMS:
      {
        // Note that connection monitor/passive receives a different data structure than the coordinator
        RTLSCtrl_connReqCmd(pHostMsg->pData);
      }
      break;
#endif

      case RTLS_CMD_SET_RTLS_PARAM:
      {
        rtlsStatus_e status = RTLS_SUCCESS;
        setRtlsParamRequest_t *req = (setRtlsParamRequest_t *)pHostMsg->pData;

        switch (req->rtlsParamType)
        {
          case RTLS_PARAM_CONNECTION_INTERVAL:
          {
            status = RTLSCtrl_updateConnInterval(req->connHandle, req->dataLen, req->data);
          }
          break;

          default:
          {
            status = RTLS_ILLEGAL_CMD;
          }
          break;
        }

        // Return response with type and status
        setRtlsParamResponse_t response = {req->connHandle, req->rtlsParamType, status};
        RTLSHost_sendMsg(RTLS_CMD_SET_RTLS_PARAM, HOST_SYNC_RSP, (uint8_t *)&response, sizeof(response));

        RTLSUTIL_FREE(pHostMsg->pData);
      }
      break;

      case RTLS_CMD_GET_ACTIVE_CONN_INFO:
      {
        RTLSCtrl_getActiveConnInfoCmd((rtlsGetActiveConnInfo_t *)pHostMsg->pData);
      }
      break;

      case RTLS_CMD_HEAP_SIZE:
      {
#ifdef USE_ICALL
        ICall_heapStats_t heap_stat;

        ICall_getHeapStats(&heap_stat);
        heapStat_t heapNow;
        heapNow.totalHeap = heap_stat.totalSize;
        heapNow.freeHeap = heap_stat.totalFreeSize;
        RTLSHost_sendMsg(RTLS_CMD_HEAP_SIZE, HOST_SYNC_RSP, (uint8_t *)&heapNow, sizeof(heapStat_t));
#else // !USE_ICALL
        rtlsStatus_e status = RTLS_CONFIG_NOT_SUPPORTED;
        RTLSHost_sendMsg(RTLS_EVT_ERROR, HOST_ASYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
#endif
      }
      break;

      default:
      {
        rtlsStatus_e status = RTLS_ILLEGAL_CMD;
        RTLSHost_sendMsg(RTLS_EVT_ERROR, HOST_ASYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));

        // If the opcode does not match anything, we need to free pData if it exists
        if (pHostMsg->pData)
        {
          RTLSUTIL_FREE(pHostMsg->pData);
        }
      }
      break;
    }
  }
}

/*********************************************************************
* @fn      RTLSCtrl_calculateRSSI
*
* @brief   This function will calculate the current RSSI based on RSSI
*          history and the current measurement
*
* @param   Last measured RSSI
*
* @return  none
*/
void RTLSCtrl_calculateRSSI(int lastRssi)
{
  gRtlsData.rssiFilter.currentRssi =
        ((RTLS_CTRL_ALPHA_FILTER_MAX_VALUE - gRtlsData.rssiFilter.alphaValue) * (gRtlsData.rssiFilter.currentRssi) + gRtlsData.rssiFilter.alphaValue * lastRssi) >> 4;
}

/*********************************************************************
* @fn      RTLSCtrl_malloc
*
* @brief   This function will allocate memory, if we were unable to allocate
*          we will report to RTLS Host
*
* @param   Allocated pointer - has to be cast
*
* @return  none
*/
void* RTLSCtrl_malloc(uint32_t sz)
{
  void *pPointer;

  RTLSUTIL_MALLOC(pPointer, sz);

  if (pPointer == NULL)
  {
    AssertHandler(RTLS_CTRL_ASSERT_CAUSE_OUT_OF_MEMORY, 0);
    return NULL;
  }

  return pPointer;
}

/*********************************************************************
 * @fn      RTLSCtrl_hostMsgCB
 *
 * @design /ref 159098678
 *
 * @brief   Callback from host when a message is available
 *          RTLS Control will enqueue the message and handle it in RTLS Control context
 *
 * @param   pMsg - uNPI message
 *
 * @return  none
 */
void RTLSCtrl_hostMsgCB(rtlsHostMsg_t *pMsg)
{
  RTLSCtrl_enqueueMsg(HOST_MSG_EVENT, (uint8_t *)pMsg);
}

/*********************************************************************
 * @fn      RTLSCtrl_enqueueMsg
 *
 * @design /ref 159098678
 *
 * @brief   Callback from RTLS host when an Rx msg is available
 *          Enqueue the message to switch the context from SWI to application
 *
 * @param   pMsg - pointer to a message
 * @param   eventId - needed to send message to correct handler
 *
 * @return  none
 */
void RTLSCtrl_enqueueMsg(uint16_t eventId, uint8_t *pMsg)
{
  rtlsEvt_t rtlsCtrlMsg;
  uint8_t enqueueStatus;
  rtlsStatus_e status = RTLS_FAIL;

  rtlsCtrlMsg.event = (rtlsEvtType_e)eventId;
  rtlsCtrlMsg.pData = pMsg;

  enqueueStatus = mq_send(gRtlsData.queueHandle,  (char*)&rtlsCtrlMsg, sizeof(rtlsCtrlMsg), 1);

  // Util failed to enqueue, report to host
  if (enqueueStatus != 0)
  {
    RTLSHost_sendMsg(RTLS_EVT_ERROR, HOST_ASYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
  }
}

/*********************************************************************
 * @fn      RTLSCtrl_processMessage
 *
 * @design /ref 159098678
 *
 * @brief   Process an incoming message
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
void RTLSCtrl_processMessage(rtlsEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case HOST_MSG_EVENT:
    {
      RTLSCtrl_processHostMessage((rtlsHostMsg_t *)pMsg->pData);
    }
    break;

    case RTLS_RUN_EVENT:
    {
      RTLSCtrl_processSyncEvent((uint8_t *)pMsg->pData);
    }
    break;

#if RTLS_CTE
    case AOA_RESULTS_EVENT:
    {
      RTLSCtrl_postProcessAoa((rtlsAoaIqEvt_t *)pMsg->pData);

      // Free I/Q array
      RTLSUTIL_FREE(((rtlsAoaIqEvt_t *)pMsg->pData)->pIQ);
    }
    break;
#endif /* RTLS_CTE */

    default:
      // Do nothing.
      break;
  }

  if (pMsg->pData)
  {
    RTLSUTIL_FREE(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      RTLSCtrl_createTask
 *
 * @design /ref 159098678
 *
 * @brief   Task creation function for the RTLS Control
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCtrl_createTask(void)
{
  char *rtlsTaskStack;
  rtlsTaskStack = (char *) RTLSCtrl_malloc( RTLS_CTRL_TASK_STACK_SIZE );

  RTLSCtrl_createPTask(&gRtlsData.threadId,
                        RTLSCtrl_taskFxn,
                        RTLS_CTRL_TASK_PRIORITY,
                        rtlsTaskStack,
                        RTLS_CTRL_TASK_STACK_SIZE);
}

/*********************************************************************
 * @fn      RTLSCtrl_taskFxn
 *
 * @design /ref 159098678
 *
 * @brief   Main RTLS Control application RTOS task loop,
 *          handles starting NPI task and receiving messages
 *
 * @param   a0 - Standard TI RTOS taskFxn arguments.
 * @param   a1 -
 *
 * @return  none
 */
void *RTLSCtrl_taskFxn(void *arg)
{
  // Create an RTOS queue for messages
  RTLSCtrl_createPQueue(&gRtlsData.queueHandle,
                        "RTLSCtrl_rtlsCtrlMsgQueue",
                         RTLS_QUEUE_SIZE,
                         sizeof(rtlsEvt_t),
                         0 /* Blocking */);

  // Initialize internal rssi alpha filter
  gRtlsData.rssiFilter.alphaValue = RTLS_CTRL_ALPHA_FILTER_VALUE;
  gRtlsData.rssiFilter.currentRssi = RTLS_CTRL_FILTER_INITIAL_RSSI;

  RTLSHost_sendMsg(RTLS_CMD_RESET_DEVICE, HOST_ASYNC_RSP, 0, 0);

  rtlsEvt_t rtlsCtrlMsg;
  for(;;)
  {
    if (mq_receive(gRtlsData.queueHandle, (char*)&rtlsCtrlMsg, sizeof(rtlsCtrlMsg), NULL) == sizeof(rtlsCtrlMsg))
    {
      // Process message.
      RTLSCtrl_processMessage(&rtlsCtrlMsg);
    }
  }
}

/*********************************************************************
 * @fn      RTLSCtrl_sendDebugEvt
 *
 * @brief   Send debug info
 *
 * @param   debug_string
 * @param   debug_value
 *
 * @return  none
 */
void RTLSCtrl_sendDebugEvt(uint8_t *debug_string, uint32_t debug_value)
{
  debugInfo_t debugInfo;

  memcpy(debugInfo.debug_string, debug_string, DEBUG_STRING_SIZE-1);
  debugInfo.debug_string[DEBUG_STRING_SIZE-1] = 0;
  debugInfo.debug_value = debug_value;

  RTLSHost_sendMsg(RTLS_EVT_DEBUG, HOST_ASYNC_RSP, (uint8_t *)&debugInfo, sizeof(debugInfo_t));
}

/*********************************************************************
 * @brief RTLSCtrl_sendPeriodicListSize
 *
 * Send periodic advertisers list size
 *
 * @param pEvt - Read list size structure
 */
void RTLSCtrl_sendPeriodicListSize(uint8_t *pEvt)
{
  //rtlsSrv_readListSizeEvt_t *pMsg = (rtlsSrv_readListSizeEvt_t *)pEvt;
  RTLSHost_sendMsg(RTLS_CMD_READ_ADV_LIST_SIZE, HOST_ASYNC_RSP, (uint8_t*)pEvt, sizeof(rtlsReadListSizeEvt_t));
}

/*********************************************************************
 * @brief RTLSCtrl_processSyncEstEvt
 *
 * Process sync establish event
 *
 * @param opcode - Command opcode
 * @param status - Periodic advertising sync HCI status
 * @param syncHandle - Handle identifying the periodic advertising train
 * @param advSid - Value of the Advertising SID
 * @param advAddrType - Advertiser address type
 * @param advAddress - Advertiser address
 * @param advPhy - Advertiser PHY
 * @param periodicAdvInt - Periodic advertising interval
 * @param advClockAccuracy - Accuracy of the periodic advertiser's clock
 *
 * @return  none
 */
void RTLSCtrl_processSyncEstEvt(uint8_t opcode,
                                uint8_t  status,
                                uint16_t syncHandle,
                                uint8_t  advSid,
                                uint8_t  advAddrType,
                                uint8_t  advAddress[6],
                                uint8_t  advPhy,
                                uint16_t periodicAdvInt,
                                uint8_t  advClockAccuracy)
{
  rtlsSyncEstEvt_t *syncEst;

  if((syncEst = (rtlsSyncEstEvt_t *)RTLSCtrl_malloc(sizeof(rtlsSyncEstEvt_t ))) == NULL)
  {
    return;
  }

  syncEst->opcode = opcode;
  syncEst->status = status;
  syncEst->syncHandle = syncHandle;
  syncEst->advSid = advSid;
  syncEst->advAddrType = advAddrType;
  memcpy(syncEst->advAddress, advAddress, 6);
  syncEst->advPhy = advPhy;
  syncEst->periodicAdvInt = periodicAdvInt;
  syncEst->advClockAccuracy = advClockAccuracy;

  RTLSHost_sendMsg(RTLS_EVT_SYNC_EST, HOST_ASYNC_RSP, (uint8_t *)syncEst, sizeof(rtlsSyncEstEvt_t));

  RTLSUTIL_FREE(syncEst);
}

/*********************************************************************
 * @brief RTLSCtrl_processSyncLostEvt
 *
 * Process sync lost event
 *
 * @param opcode - Command opcode
 * @param syncHandle - Handle identifying the periodic advertising train
 *
 * @return  none
 */
void RTLSCtrl_processSyncLostEvt(uint8_t opcode,
                                 uint16_t syncHandle)
{
  rtlsSyncLostEvt_t *syncLost;

  if((syncLost = (rtlsSyncLostEvt_t *)RTLSCtrl_malloc(sizeof(rtlsSyncLostEvt_t ))) == NULL)
  {
    return;
  }

  syncLost->opcode = opcode;
  syncLost->syncHandle = syncHandle;

  RTLSHost_sendMsg(RTLS_EVT_SYNC_LOST, HOST_ASYNC_RSP, (uint8_t *)syncLost, sizeof(rtlsSyncLostEvt_t));

  RTLSUTIL_FREE(syncLost);
}

/*********************************************************************
 * @brief  RTLSCtrl_processTerminateSyncEvt
 *
 * Process terminate sync event
 *
 * @param  none
 *
 * @return none
 */
void RTLSCtrl_processTerminateSyncEvt( void )
{
  rtlsStatus_e status = RTLS_SUCCESS;
  // Terminate sync was successful. Free the relevant node

  // send hostMsg
  RTLSHost_sendMsg(RTLS_EVT_TERMINATE_SYNC, HOST_ASYNC_RSP, (uint8_t *)&status, sizeof(status));

  // change the gRtlsData.termSyncHandle back
  gRtlsData.termSyncHandle = 0xFFFF;
}

/*********************************************************************
 * @brief RTLSCTRL_processPeriodicAdvReport
 *
 * Process prtiodic advertising report event
 *
 * @param opcode - Command opcode
 * @param syncHandle - Handle identifying the periodic advertising train
 * @param txPower - Tx Power information
 * @param rssi - RSSI value for the received packet
 * @param cteType - Constant Tone Extension type
 * @param dataStatus - Data status
 * @param dataLen - Length of the Data field
 * @param pData - Data received from a Periodic Advertising packet
 *
 * @return  none
 */
void RTLSCTRL_processPeriodicAdvReport( uint8_t opcode,
                                        uint16_t syncHandle,
                                        int8_t   txPower,
                                        int8_t   rssi,
                                        uint8_t  cteType,
                                        uint8_t  dataStatus,
                                        uint8_t  dataLen,
                                        uint8_t  *pData)
{
  rtlsPeriodicAdvRpt_t *pReport;
  uint16_t totSize = sizeof(rtlsPeriodicAdvRpt_t) + dataLen;

  if((pReport = (rtlsPeriodicAdvRpt_t *)RTLSCtrl_malloc(totSize)) == NULL)
  {
    return;
  }

  pReport->opcode = opcode;
  pReport->syncHandle = syncHandle;
  pReport->txPower = txPower;
  pReport->rssi = rssi;
  pReport->cteType = cteType;
  pReport->dataStatus = dataStatus;
  pReport->dataLen = dataLen;
  if(pData != NULL)
  {
    memcpy(pReport->pData, pData, dataLen);
  }

  RTLSHost_sendMsg(RTLS_EVT_PERIODIC_ADV_RPT, HOST_ASYNC_RSP, (uint8_t *)pReport, totSize);

  RTLSUTIL_FREE(pReport);
}

/*********************************************************************
 * @fn      RTLSCtrl_createSyncCmd
 *
 * @brief   Configure periodic scan parameters
 *
 * @param   pParams - Create sync parameters
 *
 * @return  none
 */
void RTLSCtrl_periodicAdvCreateSyncCmd(uint8_t *pParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to set the sync parameters
  RTLSCtrl_callRtlsApp(RTLS_REQ_SET_CREATE_SYNC_PARAMS, (uint8_t *)pParams);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_CREATE_SYNC, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_syncCancelCmd
 *
 * @brief   Handles a create sync cancel request from RTLS Node Manager
 *          Once the request is received, RTLS Control will call the
 *          registered application's scan function and notify Node Manager
 *          that a scan has started
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCtrl_periodicAdvSyncCancelCmd( void )
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to cancel sync
  RTLSCtrl_callRtlsApp(RTLS_REQ_CREATE_SYNC_CANCEL, NULL);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_CREATE_SYNC_CANCEL, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_periodicAdvTerminateSyncCmd
 *
 * @brief   Stop reception of the periodic advertising train
 *          identified by the syncHandle parameter.
 *
 * @param   syncHandle - Handle identifying the periodic advertising train
 *
 * @return  none
 */
void RTLSCtrl_periodicAdvTerminateSyncCmd(uint8_t *syncHandle)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  if( gRtlsData.termSyncHandle != 0xFFFF )
  {
    status = RTLS_FAIL;
  }
  else
  {
    gRtlsData.termSyncHandle = *((uint16_t*)syncHandle);
    RTLSCtrl_callRtlsApp(RTLS_REQ_TERMINATE_SYNC, (uint8_t *)syncHandle);
  }

  RTLSHost_sendMsg(RTLS_CMD_TERMINATE_SYNC, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(status));
}

/*********************************************************************
 * @fn      RTLSCtrl_periodicAdvReceiveEnableCmd
 *
 * @brief   Enable/Disable reports from periodic train
 *
 * @param   pParams - Receive Enable parameters
 *
 * @return  none
 */
void RTLSCtrl_periodicAdvReceiveEnableCmd(uint8_t *pParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to set the sync parameters
  RTLSCtrl_callRtlsApp(RTLS_REQ_PERIODIC_RECEIVE_ENABLE, (uint8_t *)pParams);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_PERIODIC_RECEIVE_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_addDeviceToPeriodicAdvListCmd
 *
 * @brief   Add device to periodic advertisers list
 *
 * @param   pParams - Device information
 *
 * @return  none
 */
void RTLSCtrl_addDeviceToPeriodicAdvListCmd(uint8_t *pParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to set the sync parameters
  RTLSCtrl_callRtlsApp(RTLS_REQ_ADD_DEVICE_ADV_LIST, (uint8_t *)pParams);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_ADD_DEVICE_ADV_LIST, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_removeDeviceFromPeriodicAdvListCmd
 *
 * @brief   Remove device from periodic advertisers list
 *
 * @param   pParams - Device information
 *
 * @return  none
 */
void RTLSCtrl_removeDeviceFromPeriodicAdvListCmd(uint8_t *pParams)
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to set the sync parameters
  RTLSCtrl_callRtlsApp(RTLS_REQ_REMOVE_DEVICE_ADV_LIST, (uint8_t *)pParams);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_REMOVE_DEVICE_ADV_LIST, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_readPeriodicAdvListSizeCmd
 *
 * @brief   Read the periodic advertises list's size
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCtrl_readPeriodicAdvListSizeCmd( void )
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to cancel sync
  RTLSCtrl_callRtlsApp(RTLS_REQ_READ_ADV_LIST_SIZE, NULL);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_READ_ADV_LIST_SIZE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

/*********************************************************************
 * @fn      RTLSCtrl_clearPeriodicAdvListCmd
 *
 * @brief   Cleat the periodic advertises list
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCtrl_clearPeriodicAdvListCmd( void )
{
  rtlsStatus_e status = RTLS_SUCCESS;

  // Request RTLS App to cancel sync
  RTLSCtrl_callRtlsApp(RTLS_REQ_CLEAR_ADV_LIST, NULL);

  // Return status to the host
  RTLSHost_sendMsg(RTLS_CMD_CLEAR_ADV_LIST, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}

// -----------------------------------------------------------------------------
//! \brief      Create a POSIX task.
//              In case the stackaddr is not provided, allocate it on the heap.
//!
//! \param    newthread     threadId
//! \param    startroutine  Pointer to the task entry function
//! \param    priority
//! \param    stackaddr
//! \param    stacksize
//!
//! \return   void
// -----------------------------------------------------------------------------
int RTLSCtrl_createPTask(pthread_t *newthread, void *(*startroutine)(void *), int priority, void *stackaddr, size_t stacksize)
{

  int retVal = 0;
  pthread_attr_t param_attribute;
  struct sched_param param;

  // Task Stack was not pre-allocated by user.
  // Allocate it.
  if ((char *)stackaddr == NULL)
  {
    // Allocated space for task stack.
    RTLSCtrl_malloc(stacksize);

    if ((char *)stackaddr == NULL)
    {
      // Failed to allocate
        return -1 /*ERROR*/;
    }
  }
  retVal =  pthread_attr_init(&param_attribute);
  param.sched_priority = priority;

  retVal |= pthread_attr_setschedparam(&param_attribute, &param);
  retVal |= pthread_attr_setstack(&param_attribute, stackaddr, stacksize);
  retVal |= pthread_attr_setdetachstate(&param_attribute, PTHREAD_CREATE_DETACHED);

  retVal |= pthread_create(newthread,
                        &param_attribute,
                        startroutine,
                        NULL);
  return retVal;
}

// -----------------------------------------------------------------------------
//! \brief      Create a POSIX queue.
//!
//! \param    queueHandle     queue handle
//! \param    mq_name         name
//! \param    mq_size         number of elements for the queue
//! \param    mq_msgsize      size of queue element
//! \param    mq_flags        flags
//!
//! \return   void
// -----------------------------------------------------------------------------
int RTLSCtrl_createPQueue(mqd_t *queueHandle, char *mq_name, uint32_t mq_size, uint32_t mq_msgsize, uint32_t mq_flags)
{
  struct mq_attr attr;

  attr.mq_flags =  O_CREAT | O_RDWR | mq_flags;
  attr.mq_curmsgs = 0;
  attr.mq_maxmsg = mq_size;
  attr.mq_msgsize = mq_msgsize;

  /* Create the message queue */
  *queueHandle = mq_open(mq_name, O_CREAT | O_RDWR | mq_flags, 0, &attr);

  return SUCCESS;
}
