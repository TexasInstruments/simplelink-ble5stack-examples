/******************************************************************************

 @file  rtls_coordinator.c

 @brief This file contains the RTLS Coordinator sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2025, Texas Instruments Incorporated
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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "l2cap.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include "osal_list.h"
#include "board_key.h"
#include <ti_drivers_config.h>

#include "ble_user_config.h"

#include "rtls_coordinator.h"

#include "rtls_ctrl_api.h"
#include "rtls_ble.h"
#ifdef RTLS_CTE
#include "rtls_aoa_api.h"
#endif /* RTLS_CTE*/

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define RC_EVT_SCAN_ENABLED        0x01
#define RC_EVT_SCAN_DISABLED       0x02
#define RC_EVT_ADV_REPORT          0x03
#define RC_EVT_ADV_EVT             0x04
#define RC_EVT_PAIR_STATE          0x05
#define RC_EVT_PASSCODE_NEEDED     0x06
#define RC_EVT_INSUFFICIENT_MEM    0x07
#define RC_EVT_RTLS_CTRL_MSG_EVT   0x08
#define RC_EVT_RTLS_SRV_MSG_EVT    0x09
#define RC_EVT_CONN_EVT            0x0A

// RTLS Coordinator Task Events
#define RC_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define RC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define RC_ALL_EVENTS                        (RC_ICALL_EVT | RC_QUEUE_EVT)

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                 ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    104

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_1M

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                200 // 2 sec

// Default supervision timeout in 10ms
#define DEFAULT_UPDATE_CONN_TIMEOUT          200

// Task configuration
#define RC_TASK_PRIORITY                     1

#ifndef RC_TASK_STACK_SIZE
#define RC_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define RC_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Connection event registration
typedef enum
{
  NOT_REGISTERED     = 0x0,
  FOR_RTLS            = 0x2,
} connectionEventRegisterCause_u;

// Spin if the expression is not true
#define RTLSCOORDINATOR_ASSERT(expr) if (!(expr)) rtls_coordinator_spin();

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connEventRegCauseBitmap |= RegisterCause)
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connEventRegCauseBitmap &= (~RegisterCause))
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connEventRegCauseBitmap > 0)
// Gets whether the RegisterCause was registered to receive connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connEventRegCauseBitmap & RegisterCause)

// Hard coded PSM for passing data between central and peripheral
#define RTLS_PSM      0x0080
#define RTLS_PDU_SIZE MAX_PDU_SIZE

#define SYNC_HANDLE_MASK 0x1000

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} rcEvt_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint16_t connHandle;
  uint8_t  status;
} rcPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} rcPasscodeData_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

typedef struct
{
  uint16_t cocCID;
  uint8_t  isActive;
} rcConnCB_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} rcGapAdvEventData_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
#define APP_EVT_BLE_LOG_STRINGS_MAX  0xA
char *appEvent_BleLogStrings[] = {
  "APP_EVT_ZERO              ",
  "APP_EVT_SCAN_ENABLED      ",
  "APP_EVT_SCAN_DISABLED     ",
  "APP_EVT_ADV_REPORT        ",
  "APP_EVT_ADV_EVT           ",
  "APP_EVT_PAIR_STATE        ",
  "APP_EVT_PASSCODE_NEEDED   ",
  "APP_EVT_INSUFFICIENT_MEM  ",
  "APP_EVT_RTLS_CTRL_MSG_EVT ",
  "APP_EVT_RTLS_SRV_MSG_EVT  ",
  "APP_EVT_CONN_EVT          ",
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct rcTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(rcTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t rcTaskStack[RC_TASK_STACK_SIZE];

// Array of connection handles and information for each handle
static rcConnCB_t rcConnCB[MAX_NUM_BLE_CONNS];

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Number of scan results and scan result index
static uint8_t scanRes = 0;

// Scan result list
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];

// Advertisement data
static uint8_t advertData[] =
{
  0x10,                         // Length of this data
  GAP_ADTYPE_LOCAL_NAME_SHORT,  // Type of this data
  'R',
  'T',
  'L',
  'S',
  'C',
  'o',
  'o',
  'r',
  'd',
  'i',
  'n',
  'a',
  't',
  'o',
  'r',
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// Scan Response Data
static uint8_t scanRspData[] =
{
  16,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, // Type of this data
  'R',
  'T',
  'L',
  'S',
  'C',
  'o',
  'o',
  'r',
  'd',
  'i',
  'n',
  'a',
  't',
  'o',
  'r',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connEventRegCauseBitmap = NOT_REGISTERED;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void RTLSCoordinator_init(void);
static void RTLSCoordinator_taskFxn(uintptr_t a0, uintptr_t a1);

static uint8_t RTLSCoordinator_processStackMsg(ICall_Hdr *pMsg);
static void RTLSCoordinator_processAppMsg(rcEvt_t *pMsg);
static void RTLSCoordinator_processGapMsg(gapEventHdr_t *pMsg);
static void RTLSCoordinator_processPairState(uint8_t state, rcPairStateData_t* pPairStateData);
static void RTLSCoordinator_processPasscode(rcPasscodeData_t *pData);
static void RTLSCoordinator_processAdvEvent(rcGapAdvEventData_t *pEventData);
static void RTLSCoordinator_advertInit(void);
static void RTLSCoordinator_scanInit(void);
static void RTLSCoordinator_addDeviceInfo(GapScan_Evt_AdvRpt_t *pEvent);
static void RTLSCoordinator_advCb(uint32_t event, void *pBuf, uintptr_t arg);
static void RTLSCoordinator_scanCb(uint32_t evt, void* msg, uintptr_t arg);
static void RTLSCoordinator_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void RTLSCoordinator_pairStateCb(uint16_t connHandle, uint8_t state, uint8_t status);
static status_t RTLSCoordinator_enqueueMsg(uint8_t event, uint8_t status, uint8_t *pData);

// RTLS specific functions
static bStatus_t RTLSCoordinator_sendRTLSData(rtlsPacket_t *pMsg);
static void RTLSCoordinator_processRTLSScanReq(void);
static void RTLSCoordinator_processRTLSScanRes(GapScan_Evt_AdvRpt_t *deviceInfo);
static void RTLSCoordinator_processRTLSConnReq(bleConnReq_t *bleConnReq);
static void RTLSCoordinator_processRTLSConnInfo(uint16_t connHandle);
static void RTLSCoordinator_setCreateSyncParams(rtlsCreateSyncParams_t *pParams);
static void RTLSCoordinator_syncCancelCmd(void);
static void RTLSCoordinator_terminateSync(rtlsTerminateSync_t *handle);
static void RTLSCoordinator_periodicReceiveEnable(rtlsReceiveEnableParams_t *pParams);
static void RTLSCoordinator_addDeviceToPeriodicAdvList(rtlsAdvListDeviceParams_t *pParams);
static void RTLSCoordinator_removeDeviceFromPeriodicAdvList(rtlsAdvListDeviceParams_t *pParams);
static void RTLSCoordinator_readPeriodicAdvListSize( void );
static void RTLSCoordinator_clearPeriodicAdvList( void );
static void RTLSCoordinator_enableRtlsSync(rtlsEnableSync_t *enable);
static void RTLSCoordinator_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void RTLSCoordinator_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void RTLSCoordinator_processRtlsCtrlMsg(uint8_t *pMsg);
static void RTLSCoordinator_terminateLinkReq(rtlsTerminateLinkReq_t *termInfo);
static void RTLSCoordinator_rtlsSrvlMsgCb(rtlsSrv_evt_t *pRtlsSrvEvt);
static void RTLSCoordinator_processRtlsSrvMsg(rtlsSrv_evt_t *pEvt);
static void RTLSCoordinator_processRTLSUpdateConnInterval(rtlsUpdateConnIntReq_t *updateReq);
#ifdef RTLS_CTE
static void RTLSCoordinator_setAoaParamsReq(rtlsAoaConfigReq_t *config);
static void RTLSCoordinator_enableAoaReq(rtlsAoaEnableReq_t *pReq);
static void RTLSCoordinator_CLAoaSamplingEnableReq(rtlsCLAoaEnableReq_t *pReq);
#endif /* RTLS_CTE*/

// L2CAP COC Handling
static bStatus_t RTLSCoordinator_openL2CAPChanCoc(uint16_t connHandle);
static void RTLSCoordinator_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg);
static void RTLSCoordinator_processL2CAPDataEvent(l2capDataEvent_t *pMsg);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static gapBondCBs_t RTLSCoordinator_bondMgrCBs =
{
 (pfnPasscodeCB_t)RTLSCoordinator_passcodeCb, // Passcode callback
  RTLSCoordinator_pairStateCb // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      rtls_coordinator_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void rtls_coordinator_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_createTask
 *
 * @brief   Task creation function for the RTLS Coordinator.
 *
 * @param   none
 *
 * @return  none
 */
void RTLSCoordinator_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = rcTaskStack;
  taskParams.stackSize = RC_TASK_STACK_SIZE;
  taskParams.priority = RC_TASK_PRIORITY;

  Task_construct(&rcTask, RTLSCoordinator_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      RTLSCoordinator_Init
 *
 * @brief   Initialization function for the RTLS Coordinator App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSCoordinator_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", RC_TASK_PRIORITY);
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  // in build_config.opt in stack project.
  {
    //Change initial values of RX/TX PDU and Time, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_RX_PDU_SIZE 251     //default is 251 octets(RX)
    #define APP_SUGGESTED_RX_TIME     17000   //default is 17000us(RX)
    #define APP_SUGGESTED_TX_PDU_SIZE 27      //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME     328     //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_EXT_SetMaxDataLenCmd(APP_SUGGESTED_TX_PDU_SIZE, APP_SUGGESTED_TX_TIME, APP_SUGGESTED_RX_PDU_SIZE, APP_SUGGESTED_RX_TIME);
  }

  // Set Bond Manager parameters
  {
    // Don't wait, initiate a pairing request or peripheral security request
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    // Do not use authenticated pairing
    uint8_t mitm = TRUE;
    // This is a display only device
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Create a bond during the pairing process
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&RTLSCoordinator_bondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_CENTRAL);
  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL | GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  //Read the LE locally supported features
  HCI_LE_ReadLocalSupportedFeaturesCmd();

  // Initialize RTLS Services
  RTLSSrv_init(MAX_NUM_BLE_CONNS);
  RTLSSrv_register(RTLSCoordinator_rtlsSrvlMsgCb);
}

/*********************************************************************
 * @fn      RTLSCoordinator_taskFxn
 *
 * @brief   Application task entry point for the RTLS Coordinator.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void RTLSCoordinator_taskFxn(uintptr_t a0, uintptr_t a1)
{
  // Initialize application
  RTLSCoordinator_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, RC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = RTLSCoordinator_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & RC_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          rcEvt_t *pMsg = (rcEvt_t *)Util_dequeueMsg(appMsgQueue);
          if(pMsg)
          {
            // Process message
            RTLSCoordinator_processAppMsg(pMsg);

            // Free the space from the message
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RTLSCoordinator_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      RTLSCoordinator_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case L2CAP_SIGNAL_EVENT:
      RTLSCoordinator_processL2CAPSignalEvent((l2capSignalEvent_t *)pMsg);
      break;

    case L2CAP_DATA_EVENT:
      RTLSCoordinator_processL2CAPDataEvent((l2capDataEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        {
          // Parse Command Complete Event for opcode and status
          hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;

          //find which command this command complete is for
          switch (command_complete->cmdOpcode)
          {
            case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
            {
              uint8_t featSet[8];

              // Get current feature set from received event (byte 1-8)
              memcpy( featSet, &command_complete->pReturnParam[1], 8 );

              // Clear the CSA#2 feature bit
              CLR_FEATURE_FLAG( featSet[1], LL_FEATURE_CHAN_ALGO_2 );

              // CM does not currently support 2M, clear 2M PHY bit
              CLR_FEATURE_FLAG( featSet[1], LL_FEATURE_2M_PHY );

              // Enable CTE
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_CONNECTION_CTE_REQUEST );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_CONNECTION_CTE_RESPONSE );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_ANTENNA_SWITCHING_DURING_CTE_RX );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_RECEIVING_CTE );

              // Update controller with modified features
              HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
            }
            break;

            default:
              break;
          }
        }
        break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        {
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
        }
        break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEChanMapUpdate_t *pCMU = (hciEvt_BLEChanMapUpdate_t*) pMsg;

          // Update the host on channel map changes
          if (pCMU->BLEEventCode == HCI_BLE_CHANNEL_MAP_UPDATE_EVENT)
          {
            if (pCMU->connHandle != LINKDB_CONNHANDLE_INVALID)
            {
              BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg HCI_GAP_EVENT_EVENT HCI_LE_EVENT_CODE, HCI_BLE_CHANNEL_MAP_UPDATE_EVENT %d,0x%x\n", pMsg->status, pCMU->BLEEventCode);
              // Upon param update, resend connection information
              RTLSCoordinator_processRTLSConnInfo(pCMU->connHandle);
            }
          }
        }
        break;

        case HCI_VE_EVENT_CODE:
        {
          hciEvt_VSCmdComplete_t *pkt  = (hciEvt_VSCmdComplete_t*) pMsg;
          switch (pkt->cmdOpcode)
          {
            case HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES:
            {
              uint8_t numActive = linkDB_NumActive();
              // If the local supported features set has been changed while we are advertising,
              // Restart the current advertising and start again with the updated features set.
              if ((numActive < MAX_NUM_BLE_CONNS))
              {
                GapAdv_disable(advHandleLegacy);
                GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
              }
            }
            break;
          }
        }
        break;

        default:
          break;
      }
      break;
    }
    default:
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RTLSCoordinator_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void RTLSCoordinator_processAppMsg(rcEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;

  if (pMsg->hdr.event <= APP_EVT_BLE_LOG_STRINGS_MAX)
  {
    if (pMsg->hdr.event != RC_EVT_CONN_EVT)
    {
      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEvent_BleLogStrings[pMsg->hdr.event]);
    }
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->hdr.event);
  }

  switch (pMsg->hdr.event)
  {
    case RC_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

      char responderScanRsp[] = {'R','T','L','S','R','e','s','p','o','n','d','e','r'};

      // Filter results by the responder's scanRsp array
      if (memcmp(&pAdvRpt->pData[2], responderScanRsp, sizeof(responderScanRsp)) == 0)
      {
        RTLSCoordinator_addDeviceInfo(pAdvRpt);
      }

      // Free report payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
    }
    break;

    case RC_EVT_SCAN_DISABLED:
    {
      if(((gapEstLinkReqEvent_t*) pMsg)->hdr.status == SUCCESS)
      {
        // Scan stopped (no more results)
        RTLSCtrl_scanResultEvt(RTLS_SUCCESS, 0, 0);
      }
      else
      {
        // Scan stopped (failed due to wrong parameters)
        RTLSCtrl_scanResultEvt(RTLS_FAIL, 0, 0);
      }
    }
    break;

    // Pairing event
    case RC_EVT_PAIR_STATE:
    {
      RTLSCoordinator_processPairState(pMsg->hdr.state, (rcPairStateData_t*)(pMsg->pData));
    }
    break;

    // Passcode event
    case RC_EVT_PASSCODE_NEEDED:
    {
      RTLSCoordinator_processPasscode((rcPasscodeData_t *)(pMsg->pData));
    }
    break;

    case RC_EVT_ADV_EVT:
    {
      RTLSCoordinator_processAdvEvent((rcGapAdvEventData_t*)(pMsg->pData));
    }
    break;

    case RC_EVT_RTLS_CTRL_MSG_EVT:
    {
      RTLSCoordinator_processRtlsCtrlMsg((uint8_t *)pMsg->pData);
    }
    break;

    case RC_EVT_RTLS_SRV_MSG_EVT:
    {
      RTLSCoordinator_processRtlsSrvMsg((rtlsSrv_evt_t *)pMsg->pData);
    }
    break;

    case RC_EVT_CONN_EVT:
    {
      RTLSCoordinator_processConnEvt((Gap_ConnEventRpt_t *)pMsg->pData);
    }
    break;

    default:
      // Do nothing.
    break;
  }

  if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void RTLSCoordinator_processGapMsg(gapEventHdr_t *pMsg)
{
  uint16_t connHandle;

  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        //Setup and start advertising
        RTLSCoordinator_advertInit();
      }

      //Setup scanning
      RTLSCoordinator_scanInit();
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
      // The amount of current connections
      uint8_t numActive = linkDB_NumActive();

      connHandle = ((gapEstLinkReqEvent_t *) pMsg)->connectionHandle;

      if (connHandle != LINKDB_CONNHANDLE_INVALID && connHandle < MAX_NUM_BLE_CONNS &&
          ((gapEstLinkReqEvent_t *) pMsg)->hdr.status == SUCCESS)
      {
        rcConnCB[connHandle].isActive = TRUE;

        HCI_LE_ReadRemoteUsedFeaturesCmd(connHandle);

        // We send out the connection information at this point
        // Note: we are not yet connected (will be after pairing)
        RTLSCoordinator_processRTLSConnInfo(connHandle);

      }
      else
      {
        // Link failed to establish
        RTLSCtrl_connResultEvt(LINKDB_CONNHANDLE_INVALID, RTLS_LINK_ESTAB_FAIL);
      }

      if ((numActive < MAX_NUM_BLE_CONNS))
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLegacy);
      }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      connHandle = ((gapTerminateLinkEvent_t *) pMsg)->connectionHandle;

      if (connHandle != LINKDB_CONNHANDLE_INVALID && connHandle < MAX_NUM_BLE_CONNS)
      {
        // This connection is inactive
        rcConnCB[connHandle].isActive = FALSE;

        // Link terminated
        RTLSCtrl_connResultEvt(connHandle, RTLS_LINK_TERMINATED);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
    }
    break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg status=%d, opcode=%s\n", 0, "GAP_LINK_PARAM_UPDATE_EVENT");
      connHandle = ((gapLinkUpdateEvent_t *) pMsg)->connectionHandle;

      if (connHandle != LINKDB_CONNHANDLE_INVALID && connHandle < MAX_NUM_BLE_CONNS &&
          ((gapLinkUpdateEvent_t *) pMsg)->hdr.status == SUCCESS)
      {
        // Upon param update, resend connection information
        RTLSCoordinator_processRTLSConnInfo(connHandle);
      }
    }
    break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void RTLSCoordinator_processPairState(uint8_t state, rcPairStateData_t* pPairData)
{
  uint8_t status = pPairData->status;

#ifdef RTLS_DEBUG
  RTLSCtrl_sendDebugEvt("RTLSCoordinator_processPairState", state);
#endif

  switch (state)
  {
    // Once Coordinator and Responder are paired, we can open a COC channel
    case GAPBOND_PAIRING_STATE_COMPLETE:
    case GAPBOND_PAIRING_STATE_ENCRYPTED:
    {
      if (status == SUCCESS)
      {
        // We are paired, open a L2CAP channel to pass data
        if (RTLSCoordinator_openL2CAPChanCoc(pPairData->connHandle) != SUCCESS)
        {
          // We could not establish an L2CAP link, drop the connection
          // We will notify host that this connection is terminated when the GAP_LINK_TERMINATED_EVENT returns
          GAP_TerminateLinkReq(pPairData->connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
        }
      }
      else
      {
        // We could not establish an L2CAP link, drop the connection
        // We will notify host that this connection is terminated when the GAP_LINK_TERMINATED_EVENT returns
        GAP_TerminateLinkReq(pPairData->connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
      }
    }
    break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void RTLSCoordinator_processPasscode(rcPasscodeData_t *pData)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      RTLSCoordinator_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void RTLSCoordinator_processAdvEvent(rcGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

/*********************************************************************
* @fn      RTLSCoordinator_advertInit
*
* @brief   Setup initial advertisement and start advertising from device init.
*
* @return  None.
*/
static void RTLSCoordinator_advertInit(void)
{
  bStatus_t status = FAILURE;

  // Setup and start Advertising
  // For more information, see the GAP section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/

  // Temporary memory for advertising parameters for set #1. These will be copied
  // by the GapAdv module
  GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 0, 0);
  // Create Advertisement set #1 and assign handle
  status = GapAdv_create(&RTLSCoordinator_advCb, &advParamLegacy, &advHandleLegacy);
  RTLSCOORDINATOR_ASSERT(status == SUCCESS);

  // Load advertising data for set #1 that is statically allocated by the app
  status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                               sizeof(advertData), advertData);
  RTLSCOORDINATOR_ASSERT(status == SUCCESS);

  // Load scan response data for set #1 that is statically allocated by the app
  status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                               sizeof(scanRspData), scanRspData);
  RTLSCOORDINATOR_ASSERT(status == SUCCESS);

  // Set event mask for set #1
  status = GapAdv_setEventMask(advHandleLegacy,
                               GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                               GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                               GAP_ADV_EVT_MASK_SET_TERMINATED);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
  // Enable legacy advertising for set #1
  status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
  RTLSCOORDINATOR_ASSERT(status == SUCCESS);
}

/*********************************************************************
* @fn      RTLSCoordinator_scanInit
*
* @brief   Setup initial device scan settings.
*
* @return  None.
*/
static void RTLSCoordinator_scanInit(void)
{
  uint8_t temp8;
  uint16_t temp16;

  // Setup scanning
  // For more information, see the GAP section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/

  // Register callback to process Scanner events
  GapScan_registerCb(RTLSCoordinator_scanCb, NULL);

  // Set Scanner Event Mask
  GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                       GAP_EVT_ADV_REPORT);

  // Set Scan PHY parameters
  GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_PASSIVE,
                       200, 100);

  // Set Advertising report fields to keep
  temp16 = RC_ADV_RPT_FIELDS;
  GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
  // Set Scanning Primary PHY
  temp8 = DEFAULT_SCAN_PHY;
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
  // Set LL Duplicate Filter
  temp8 = SCAN_FLT_DUP_ENABLE;
  GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

  // Set PDU type filter -
  // Only 'Complete' packets are desired.
  // It doesn't matter if received packets are
  // whether Scannable or Non-Scannable, whether Directed or Undirected,
  // whether Connectable or Non-Connectable, whether Scan_Rsp's or Advertisements,
  // and whether Legacy or Extended.
  temp16 = SCAN_FLT_PDU_COMPLETE_ONLY;
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapScan_setParam", 0);
  GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);
}

/*********************************************************************
 * @fn      RTLSCoordinator_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void RTLSCoordinator_addDeviceInfo(GapScan_Evt_AdvRpt_t *deviceInfo)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(deviceInfo->addr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Send the device info to RTLS Control
    RTLSCoordinator_processRTLSScanRes(deviceInfo);

    // Add addr to scan result list
    memcpy(scanList[scanRes].addr, deviceInfo->addr, B_ADDR_LEN);
    scanList[scanRes].addrType = deviceInfo->addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_advCb
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void RTLSCoordinator_advCb(uint32_t event, void *pBuf, uintptr_t arg)
{
  rcGapAdvEventData_t *pData = ICall_malloc(sizeof(rcGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(RTLSCoordinator_enqueueMsg(RC_EVT_ADV_EVT, SUCCESS, (uint8_t*)pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
static void RTLSCoordinator_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = RC_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = RC_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = RC_EVT_SCAN_DISABLED;
  }
  else
  {
    return;
  }

  if (RTLSCoordinator_enqueueMsg(event, SUCCESS, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }
}

/*********************************************************************
* @fn      RTLSCoordinator_passcodeCb
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
* @param   connHandle - the connection handle
* @param   uiInputs - pairing User Interface Inputs
* @param   uiOutputs - pairing User Interface Outputs
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void RTLSCoordinator_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
  rcPasscodeData_t *pData = ICall_malloc(sizeof(rcPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData)
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if (RTLSCoordinator_enqueueMsg(RC_EVT_PASSCODE_NEEDED, 0,(uint8_t *) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param connectionHandle - connection handle of current pairing process
 * @param state - @ref GAPBondMgr_Events
 * @param status - pairing status
 *
 * @return  none
 */
static void RTLSCoordinator_pairStateCb(uint16_t connHandle, uint8_t state, uint8_t status)
{
  rcPairStateData_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(rcPairStateData_t))))
  {
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if (RTLSCoordinator_enqueueMsg(RC_EVT_PAIR_STATE, state, (uint8_t*) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t RTLSCoordinator_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  uint8_t success;
  rcEvt_t *pMsg = ICall_malloc(sizeof(rcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      RTLSCoordinator_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param   pReport pointer to connection event report
 */
static void RTLSCoordinator_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if (RTLSCoordinator_enqueueMsg(RC_EVT_CONN_EVT, SUCCESS, (uint8_t *)pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processConnEvt
 *
 * @brief   Connection event callback.
 *
 * @param   pReport - pointer to connection event report
 *
 * @return  none
 */
static void RTLSCoordinator_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Sanity check
  if (!pReport)
  {
  return;
  }

  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_RTLS) && rcConnCB[pReport->handle].isActive)
  {
    rtlsStatus_e status;

    // Convert BLE specific status to RTLS Status
    if (pReport->status != GAP_CONN_EVT_STAT_MISSED)
    {
      status = RTLS_SUCCESS;
    }
    else
    {
      status = RTLS_FAIL;
    }

#ifdef RTLS_TEST_CHAN_MAP_DYNAMIC_CHANGE
    // For testing - do dynamic change of the channel map after 10 connection events
    {
        static int connectionEventCount = 0;

        if (++connectionEventCount == 10)
        {
          uint8_t chanMap[5] = {0xFF, 0xFF, 0xFF, 0x00, 0x1F};      // unmap channels 24..31
          HCI_LE_SetHostChanClassificationCmd(chanMap);
        }
    }
#endif //RTLS_TEST_CHAN_MAP_DYNAMIC_CHANGE

    RTLSCtrl_syncNotifyEvt(pReport->handle, status, pReport->nextTaskTime, pReport->lastRssi, pReport->channel);
  }

  if (pReport != NULL)
  {
    // Free the report once we are done using it
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_sendRTLSData
 *
 * @brief   Send RTLS data to the peer
 *
 * @param   pMsg - pointer to the message to send
 *
 * @return  none
 */
static bStatus_t RTLSCoordinator_sendRTLSData(rtlsPacket_t *pMsg)
{
  l2capPacket_t pkt;
  bStatus_t status = SUCCESS;

  // Sanity check
  if (!pMsg)
  {
    return FAILURE;
  }

  // Tell L2CAP the desired Channel ID
  pkt.CID = rcConnCB[pMsg->connHandle].cocCID;

  // Allocate space for payload
  pkt.pPayload = L2CAP_bm_alloc(pMsg->payloadLen);

  if (pkt.pPayload != NULL)
  {
    // The request is the payload for the L2CAP SDU
    memcpy(pkt.pPayload, pMsg, pMsg->payloadLen);
    pkt.len = pMsg->payloadLen;
    status = L2CAP_SendSDU(&pkt);

    // Check that the packet was sent
    if (SUCCESS != status)
    {
      // If SDU wasn't sent, free
      BM_free(pkt.pPayload);
    }
  }
  else
  {
    status = bleMemAllocError;
  }

  return (status);
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRTLSScanRes
 *
 * @brief   Process a scan response and forward to RTLS Control
 *
 * @param   deviceInfo - a single scan response
 *
 * @return  none
 */
static void RTLSCoordinator_processRTLSScanRes(GapScan_Evt_AdvRpt_t *deviceInfo)
{
  GapScan_Evt_AdvRpt_t *devInfo;
  size_t resSize;
  bleScanInfo_t *scanResult;

  // Sanity check
  if (!deviceInfo)
  {
    return;
  }

  devInfo = deviceInfo;

  // Assign and allocate space
  resSize = sizeof(bleScanInfo_t) + devInfo->dataLen;
  scanResult = (bleScanInfo_t *)ICall_malloc(resSize);

  // We could not allocate memory, report to host and exit
  if (!scanResult)
  {
    RTLSCtrl_scanResultEvt(RTLS_OUT_OF_MEMORY, NULL, 0);
    return;
  }

  memcpy(scanResult->addr, devInfo->addr, B_ADDR_LEN);
  scanResult->addrType = devInfo->addrType;
  scanResult->eventType = devInfo->evtType;
  scanResult->dataLen = devInfo->dataLen;
  scanResult->rssi = devInfo->rssi;
  scanResult->advSID = devInfo->advSid;
  scanResult->periodicAdvInt = devInfo->periodicAdvInt;
  memcpy(scanResult->pEvtData, devInfo->pData, devInfo->dataLen);

  RTLSCtrl_scanResultEvt(RTLS_SUCCESS, (uint8_t*)scanResult, resSize);

  ICall_free(scanResult);
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRTLSScanReq
 *
 * @brief   Process a scan request
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSCoordinator_processRTLSScanReq(void)
{
  scanRes = 0;

  // Start discovery
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRTLSConnReq
 *
 * @brief   Start the connection process with another device
 *
 * @param   bleConnReq - pointer from RTLS control containing connection params
 *
 * @return  none
 */
static void RTLSCoordinator_processRTLSConnReq(bleConnReq_t *bleConnReq)
{
  bStatus_t status;

  // Sanity check
  if (!bleConnReq)
  {
   return;
  }

  //Set connection interval and supervision timeout
  GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED, INIT_PHYPARAM_CONN_INT_MAX, bleConnReq->connInterval);
  GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED, INIT_PHYPARAM_CONN_INT_MIN, bleConnReq->connInterval);
  GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED, INIT_PHYPARAM_SUP_TIMEOUT, DEFAULT_UPDATE_CONN_TIMEOUT);

  status = GapInit_connect(bleConnReq->addrType & MASK_ADDRTYPE_ID, bleConnReq->addr, DEFAULT_INIT_PHY, 0);
  if (status != SUCCESS)
  {
    // Notify RTLS Control that we are not connected
    RTLSCtrl_connResultEvt(0, RTLS_FAIL);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRTLSConnRes
 *
 * @brief   Process a connection established event - send conn info to RTLS Control
 *
 * @param   connHandle - connection handle
 *
 * @return  none
 */
static void RTLSCoordinator_processRTLSConnInfo(uint16_t connHandle)
{
  hciActiveConnInfo_t connInfo;
  linkDBInfo_t addrInfo;
  bleConnInfo_t rtlsConnInfo = {0};

  // Get BD Address of the requested Responder
  linkDB_GetInfo(connHandle, &addrInfo);
  memcpy(rtlsConnInfo.addr, addrInfo.addr, B_ADDR_LEN);

  // Get current active connection information
  HCI_EXT_GetActiveConnInfoCmd(connHandle, &connInfo);

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLSConnInfo hopValue=%d, currChan=%d\n", connInfo.hopValue, connInfo.nextChan);
  rtlsConnInfo.connHandle = connHandle;
  rtlsConnInfo.accessAddr = connInfo.accessAddr;
  rtlsConnInfo.connRole = addrInfo.connRole;
  rtlsConnInfo.connInterval = connInfo.connInterval;
  rtlsConnInfo.currChan = connInfo.nextChan;
  rtlsConnInfo.hopValue = connInfo.hopValue;
  rtlsConnInfo.cSCA = connInfo.mSCA;
  rtlsConnInfo.crcInit = BUILD_UINT32(connInfo.crcInit[0], connInfo.crcInit[1], connInfo.crcInit[2], 0);
  memcpy(rtlsConnInfo.chanMap, connInfo.chanMap, LL_NUM_BYTES_FOR_CHAN_MAP);

  RTLSCtrl_connInfoEvt((uint8_t*)&rtlsConnInfo, sizeof(bleConnInfo_t));
}

/*********************************************************************
 * @fn      RTLSCoordinator_openL2CAPChanCoc
 *
 * @brief   Opens a communication channel between RTLS Coordinator/Responder
 *
 * @param   connHandle - connection handle
 *
 * @return  status - 0 = success, 1 = failed
 */
static bStatus_t RTLSCoordinator_openL2CAPChanCoc(uint16_t connHandle)
{
  uint8_t ret;
  l2capPsm_t psm;
  l2capPsmInfo_t psmInfo;

  if (L2CAP_PsmInfo(RTLS_PSM, &psmInfo) == INVALIDPARAMETER)
  {
    // Prepare the PSM parameters
    psm.initPeerCredits = L2CAP_MAX_NOF_CREDITS;
    psm.maxNumChannels = MAX_NUM_BLE_CONNS;
    psm.mtu = RTLS_PDU_SIZE;
    psm.peerCreditThreshold = 0;
    psm.pfnVerifySecCB = NULL;
    psm.psm = RTLS_PSM;
    psm.taskId = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity);

    // Register PSM with L2CAP task
    ret = L2CAP_RegisterPsm(&psm);

    if (ret == SUCCESS)
    {
      // Send the connection request to RTLS responder
      ret = L2CAP_ConnectReq(connHandle, RTLS_PSM, RTLS_PSM);
    }
  }
  else
  {
    // Send the connection request to RTLS responder
    ret = L2CAP_ConnectReq(connHandle, RTLS_PSM, RTLS_PSM);
  }

  return ret;
}

/*********************************************************************
 * @fn      RTLSCoordinator_processL2CAPSignalEvent
 *
 * @brief   Handle L2CAP signal events
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  none
 */
static void RTLSCoordinator_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg)
{
  // Sanity check
  if (!pMsg)
  {
    return;
  }

  switch (pMsg->opcode)
  {
    case L2CAP_CHANNEL_ESTABLISHED_EVT:
    {
      l2capChannelEstEvt_t *pEstEvt = &(pMsg->cmd.channelEstEvt);

      // Connection established, save the CID
      if (pMsg->connHandle != LINKDB_CONNHANDLE_INVALID && pMsg->connHandle < MAX_NUM_BLE_CONNS)
      {
        rcConnCB[pMsg->connHandle].cocCID = pEstEvt->CID;

        // Give max credits to the other side
        L2CAP_FlowCtrlCredit(pEstEvt->CID, L2CAP_MAX_NOF_CREDITS);

        // L2CAP establishing a COC channel means that both Coordinator and Responder are ready
        // Tell RTLS Control that we are ready for more commands
        RTLSCtrl_connResultEvt(pMsg->connHandle, RTLS_SUCCESS);
      }
      else
      {
        // We could not establish an L2CAP link, drop the connection
        RTLSCtrl_sendDebugEvt((uint8_t *)"L2CAP COC: could not establish", pMsg->connHandle);
        GAP_TerminateLinkReq(pMsg->connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
      }
    }
    break;

    case L2CAP_SEND_SDU_DONE_EVT:
    {
      if (pMsg->hdr.status == SUCCESS)
      {
        RTLSCtrl_dataSentEvt(pMsg->connHandle, RTLS_SUCCESS);
      }
      else
      {
        RTLSCtrl_dataSentEvt(pMsg->connHandle, RTLS_FAIL);
      }
    }
    break;

    case L2CAP_CHANNEL_TERMINATED_EVT:
    {
      // Terminate the connection
      GAP_TerminateLinkReq(pMsg->connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
      RTLSCtrl_sendDebugEvt((uint8_t *)"L2CAP COC: terminated connHandle: ", pMsg->connHandle);
    }
    break;
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processL2CAPDataEvent
 *
 * @brief   Handles incoming L2CAP data
 *          RTLS Coordinator does not expect any incoming data
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  none
 */
static void RTLSCoordinator_processL2CAPDataEvent(l2capDataEvent_t *pMsg)
{
  // Sanity check
  if (!pMsg)
  {
    return;
  }

  // Free the payload (must use BM_free here according to L2CAP documentation)
  BM_free(pMsg->pkt.pPayload);
}

/*********************************************************************
 * @fn      RTLSCoordinator_enableRtlsSync
 *
 * @brief   This function is used by RTLS Control to notify the RTLS application
 *          to start sending synchronization events (for BLE this is a connection event)
 *
 * @param   enable - start/stop synchronization
 *
 * @return  none
 */
static void RTLSCoordinator_enableRtlsSync(rtlsEnableSync_t *enable)
{
  bStatus_t status = RTLS_FALSE;

  if (enable->enable == RTLS_TRUE)
  {
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
      status = Gap_RegisterConnEventCb(RTLSCoordinator_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, LINKDB_CONNHANDLE_ALL);
    }

    if (status == SUCCESS)
    {
      CONNECTION_EVENT_REGISTER_BIT_SET(FOR_RTLS);
    }
  }
  else if (enable->enable == RTLS_FALSE)
  {
    CONNECTION_EVENT_REGISTER_BIT_REMOVE(FOR_RTLS);

    // If there is nothing registered to the connection event, request to unregister
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
      Gap_RegisterConnEventCb(RTLSCoordinator_connEvtCB, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, LINKDB_CONNHANDLE_ALL);
    }
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_terminateLinkReq
 *
 * @brief   Terminate active link
 *
 * @param   termInfo - information about the connection to terminate
 *
 * @return  none
 */
static void RTLSCoordinator_terminateLinkReq(rtlsTerminateLinkReq_t *termInfo)
{
  if (termInfo->connHandle != LINKDB_CONNHANDLE_INVALID && termInfo->connHandle < MAX_NUM_BLE_CONNS)
  {
    L2CAP_DisconnectReq(rcConnCB[termInfo->connHandle].cocCID);
  }
  else
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Connection Handle invalid", LINKDB_CONNHANDLE_INVALID);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_setCreateSyncParams
 *
 * @brief   Configure sync parameters
 *
 * @param   pParams - Pointer to create sync parameters
 *
 * @return  none
 */
static void RTLSCoordinator_setCreateSyncParams(rtlsCreateSyncParams_t *pParams)
{
  uint32_t status;
  GapScan_PeriodicAdvCreateSyncParams_t pSyncParams;

  pSyncParams.options = pParams->options;
  pSyncParams.advAddrType = pParams->advAddrType;
  MAP_osal_memcpy(pSyncParams.advAddress, pParams->advAddress, B_ADDR_LEN);
  pSyncParams.skip = pParams->skip;
  pSyncParams.syncTimeout = pParams->syncTimeout;
  pSyncParams.syncCteType = pParams->syncCteType;

  status = GapScan_PeriodicAdvCreateSync(pParams->advSID, &pSyncParams);

  if( status != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Set create sync params failed", status);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_syncCancelCmd
 *
 * @brief   Cancels the create sync command
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSCoordinator_syncCancelCmd( void )
{
  GapScan_PeriodicAdvCreateSyncCancel();
}

/*********************************************************************
 * @fn      RTLSCoordinator_terminateSync
 *
 * @brief   Terminate synchronization with the periodic advertising
 *          identified by the syncHandle given
 *
 * @param   handle - The syncHandle of the periodic advertising
 *
 * @return  none
 */
static void RTLSCoordinator_terminateSync(rtlsTerminateSync_t *handle)
{
  uint32_t status;

  status = GapScan_PeriodicAdvTerminateSync(handle->syncHandle);

  if( status != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Terminate sync failed", status);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_periodicReceiveEnable
 *
 * @brief   Disable/Enable periodic advertising report from the periodic
 *          advetising identified by the syncHandle given
 *
 * @param   pParams - The parameter given to enable/disable the periodic
 *                    reports
 *
 * @return  none
 */
static void RTLSCoordinator_periodicReceiveEnable(rtlsReceiveEnableParams_t *pParams)
{
  uint32_t status;

  status = GapScan_SetPeriodicAdvReceiveEnable(pParams->syncHandle, pParams->enable);

  if( status != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Receive enable failed", status);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_addDeviceToPeriodicAdvList
 *
 * @brief   Adding a device to the advertisers list
 *
 * @param   pParams - Device information
 *
 * @return  none
 */
static void RTLSCoordinator_addDeviceToPeriodicAdvList(rtlsAdvListDeviceParams_t *pParams)
{
  uint32_t status;
  rtlsAdvListDeviceParams_t *pAddParams = (rtlsAdvListDeviceParams_t *)pParams;

  status = GapScan_AddDeviceToPeriodicAdvList(pAddParams->advAddrType, pAddParams->advAddress, pAddParams->advSID);

  if( status != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Add to adv list failed", status);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_removeDeviceFromPeriodicAdvList
 *
 * @brief   Removing a device from the advertisers list
 *
 * @param   pParams - Device information
 *
 * @return  none
 */
static void RTLSCoordinator_removeDeviceFromPeriodicAdvList(rtlsAdvListDeviceParams_t *pParams)
{
  uint32_t status;
  rtlsAdvListDeviceParams_t *pAddParams = (rtlsAdvListDeviceParams_t *)pParams;

  status = GapScan_RemoveDeviceFromPeriodicAdvList(pAddParams->advAddrType, pAddParams->advAddress, pAddParams->advSID);

  if( status != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Remove from adv list failed", status);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_readPeriodicAdvListSize
 *
 * @brief   Read the advertisers list size
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSCoordinator_readPeriodicAdvListSize( void )
{
  GapScan_ReadPeriodicAdvListSize();
}

/*********************************************************************
 * @fn      RTLSCoordinator_clearPeriodicAdvList
 *
 * @brief   Clear the advertisers list
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSCoordinator_clearPeriodicAdvList( void )
{
  GapScan_ClearPeriodicAdvList();
}

#ifdef RTLS_CTE
/*********************************************************************
 * @fn      RTLSCoordinator_setAoaParamsReq
 *
 * @brief   Configure AoA parameters to the BLE Stack
 *
 * @param   config - Parameters to configure
 *
 * @return  none
 */
static void RTLSCoordinator_setAoaParamsReq(rtlsAoaConfigReq_t *pConfig)
{
  bStatus_t status;
  // Initialize GPIO's specified in sysConfig or in ble_user_config.c (antennaTbl)
  // Initialize one of the antenna ID's as the main antenna (in this case the first antenna in the pattern)
  status = RTLSSrv_initAntArray(pConfig->pAntPattern[0]);

  if (status == FAILURE)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Antenna array configuration invalid", 0);
    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
  }

  // Configure AoA receiver parameters
  RTLSSrv_setConnCteReceiveParams(pConfig->connHandle,
                                  pConfig->samplingEnable,
                                  pConfig->slotDurations,
                                  pConfig->numAnt,
                                  pConfig->pAntPattern);

  // Configure sample accuracy
  RTLSSrv_setCteSampleAccuracy(pConfig->connHandle,
                               pConfig->sampleRate,
                               pConfig->sampleSize,
                               pConfig->sampleRate,
                               pConfig->sampleSize,
                               pConfig->sampleCtrl);
}

/*********************************************************************
 * @fn      RTLSCoordinator_enableAoaReq
 *
 * @brief   Enable sampling AoA
 *
 * @param   config - Parameters to configure
 *
 * @return  none
 */
static void RTLSCoordinator_enableAoaReq(rtlsAoaEnableReq_t *pReq)
{
  // Sanity check
  if (pReq == NULL)
  {
    return;
  }

  // Request CTE from our peer
  RTLSSrv_setConnCteRequestEnableCmd(pReq->connHandle,
                                     pReq->enableAoa,
                                     pReq->cteInterval,
                                     pReq->cteLength,
                                     RTLSSRV_CTE_TYPE_AOA);
}

/*********************************************************************
 * @fn      RTLSCoordinator_CLAoaSamplingEnableReq
 *
 * @brief   Enable sampling AoA
 *
 * @param   config - Parameters to configure
 *
 * @return  none
 */
static void RTLSCoordinator_CLAoaSamplingEnableReq(rtlsCLAoaEnableReq_t *pReq)
{
  bStatus_t status;
  uint8_t cmdStatus;
  uint16_t syncHandle;

  // Sanity check
  if (pReq == NULL)
  {
    return;
  }

  // Initialize GPIO's specified in ble_user_config.c (antennaTbl)
  // Initialize one of the antenna ID's as the main antenna (in this case the first antenna in the pattern)
  status = RTLSSrv_initAntArray(pReq->pAntPattern[0]);

  if (status == FAILURE)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Antenna array configuration invalid", 0);
    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
  }

  // Mask the syncHandle to notify the controller it is CL CTE req
  syncHandle = pReq->syncHandle | SYNC_HANDLE_MASK;

  // Configure sample accuracy
  RTLSSrv_setCteSampleAccuracy( syncHandle,
                                pReq->sampleRate,
                                pReq->sampleSize,
                                pReq->sampleRate,
                                pReq->sampleSize,
                                pReq->sampleCtrl );

  // Request connectionless CTE from our peer
  cmdStatus = RTLSSrv_setCLCteSamplingEnableCmd( pReq->syncHandle,
                                                 pReq->enable,
                                                 pReq->slotDuration,
                                                 pReq->maxSampleCte,
                                                 pReq->numAnt,
                                                 pReq->pAntPattern );

  if( cmdStatus != SUCCESS)
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"CL AoA enable request failed", cmdStatus);
    // We need to remove the CL AoA node
    RTLSCtrl_processClAoaEnableEvt(RTLSSRV_SET_CL_IQ_SAMPLING_ENABLE,
                                   cmdStatus,
                                   pReq->syncHandle);
  }
}
#endif /* RTLS_CTE */

/*********************************************************************
 * @fn      RTLSCoordinator_processRTLSUpdateConnInterval
 *
 * @brief   Update connection interval
 *
 * @param   updateReq - pointer from RTLS control containing connection params
 *
 * @return  none
 */
static void RTLSCoordinator_processRTLSUpdateConnInterval(rtlsUpdateConnIntReq_t *updateReq)
{
  gapUpdateLinkParamReq_t params;
  linkDBInfo_t linkInfo;

  // Sanity check
  if (!updateReq)
  {
    return;
  }

  if (linkDB_GetInfo(updateReq->connHandle, &linkInfo) == SUCCESS)
  {
    params.connLatency = linkInfo.connLatency;
    params.connTimeout = linkInfo.connTimeout;
    params.connectionHandle = updateReq->connHandle;

    // Min/Max set to the same value
    params.intervalMax = updateReq->connInterval;
    params.intervalMin = updateReq->connInterval;

    GAP_UpdateLinkParamReq(&params);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRtlsCtrlMsg
 *
 * @brief   Handle processing messages from RTLS Control
 *
 * @param   msg - a pointer to the message
 *
 * @return  none
 */
static void RTLSCoordinator_processRtlsCtrlMsg(uint8_t *pMsg)
{
  rtlsCtrlReq_t *pReq;

  // Sanity check
  if (!pMsg)
  {
    return;
  }

  // Cast to appropriate struct
  pReq = (rtlsCtrlReq_t *)pMsg;

  if (pReq->reqOp <= RTLS_REQ_BLE_LOG_STRINGS_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : RTLS msg status=%d, event=%s\n", 0, rtlsReq_BleLogStrings[pReq->reqOp]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLS msg status=%d, event=0x%x\n", 0, pReq->reqOp);
  }

  switch(pReq->reqOp)
  {
    case RTLS_REQ_CONN:
    {
      RTLSCoordinator_processRTLSConnReq((bleConnReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_SCAN:
    {
      RTLSCoordinator_processRTLSScanReq();
    }
    break;

    case RTLS_REQ_ENABLE_SYNC:
    {
      RTLSCoordinator_enableRtlsSync((rtlsEnableSync_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_SEND_DATA:
    {
      RTLSCoordinator_sendRTLSData((rtlsPacket_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_TERMINATE_LINK:
    {
      RTLSCoordinator_terminateLinkReq((rtlsTerminateLinkReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_SET_CREATE_SYNC_PARAMS:
    {
      RTLSCoordinator_setCreateSyncParams((rtlsCreateSyncParams_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_CREATE_SYNC_CANCEL:
    {
      RTLSCoordinator_syncCancelCmd();
    }
    break;

    case RTLS_REQ_TERMINATE_SYNC:
    {
      RTLSCoordinator_terminateSync((rtlsTerminateSync_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_PERIODIC_RECEIVE_ENABLE:
    {
      RTLSCoordinator_periodicReceiveEnable((rtlsReceiveEnableParams_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_ADD_DEVICE_ADV_LIST:
    {
      RTLSCoordinator_addDeviceToPeriodicAdvList((rtlsAdvListDeviceParams_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_REMOVE_DEVICE_ADV_LIST:
    {
      RTLSCoordinator_removeDeviceFromPeriodicAdvList((rtlsAdvListDeviceParams_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_READ_ADV_LIST_SIZE:
    {
      RTLSCoordinator_readPeriodicAdvListSize();
    }
    break;

    case RTLS_REQ_CLEAR_ADV_LIST:
    {
      RTLSCoordinator_clearPeriodicAdvList();
    }
    break;

    case RTLS_REQ_UPDATE_CONN_INTERVAL:
    {
      RTLSCoordinator_processRTLSUpdateConnInterval((rtlsUpdateConnIntReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_GET_ACTIVE_CONN_INFO:
    {
      rtlsGetActiveConnInfo_t *pConnInfoReq = (rtlsGetActiveConnInfo_t *)pReq->pData;

      if (pConnInfoReq)
      {
        RTLSCoordinator_processRTLSConnInfo(pConnInfoReq->connHandle);
      }
    }
    break;

#ifdef RTLS_CTE
    case RTLS_REQ_SET_AOA_PARAMS:
    {
      RTLSCoordinator_setAoaParamsReq((rtlsAoaConfigReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_AOA_ENABLE:
    {
      RTLSCoordinator_enableAoaReq((rtlsAoaEnableReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_CL_AOA_ENABLE:
    {
      RTLSCoordinator_CLAoaSamplingEnableReq((rtlsCLAoaEnableReq_t *)pReq->pData);
    }
    break;
#endif /* RTLS_CTE */

    default:
      break;
  }

  // Free the payload
  if (pReq->pData)
  {
    ICall_free(pReq->pData);
  }
}

/*********************************************************************
 * @fn      RTLSCoordinator_processRtlsSrvMsg
 *
 * @brief   Handle processing messages from RTLS Services host module
 *
 * @param   pEvt - a pointer to the event
 *
 * @return  none
 */
static void RTLSCoordinator_processRtlsSrvMsg(rtlsSrv_evt_t *pEvt)
{
  if (!pEvt)
  {
    return;
  }

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : RTLSsrv msg status=%d, event=0x%x\n", 0, pEvt->evtType);
  switch (pEvt->evtType)
  {
    case RTLSSRV_ANTENNA_INFORMATION_EVT:
    {
      // This is for demonstration purposes - we could either use RTLSCtrl_sendDebugEvent
      // Or read the information by using a debugger
      // rtlsSrv_antennaInfo_t *pAntInfo = (rtlsSrv_antennaInfo_t)pEvt->evtData;
    }
    break;

    case RTLSSRV_ERROR_EVT:
    {
      rtlsSrv_errorEvt_t *pError = (rtlsSrv_errorEvt_t *)pEvt->evtData;
      RTLSCtrl_sendDebugEvt((uint8_t *)"RTLS Services Error", (uint32_t)pError->errCause);
    }
    break;

    case RTLSSRV_READ_PERIODIC_LIST_SIZE_EVT:
    {
      RTLSCtrl_sendPeriodicListSize(pEvt->evtData);
    }
    break;

    case RTLSSRV_SYNC_EST_EVT:
    {
      rtlsSrv_SyncEstEvt_t *pEvent = (rtlsSrv_SyncEstEvt_t *)pEvt->evtData;

      RTLSCtrl_processSyncEstEvt(pEvent->opcode,
                                 pEvent->status,
                                 pEvent->syncHandle,
                                 pEvent->advSid,
                                 pEvent->advAddrType,
                                 pEvent->advAddress,
                                 pEvent->advPhy,
                                 pEvent->periodicAdvInt,
                                 pEvent->advClockAccuracy);
    }
    break;

    case RTLSSRV_SYNC_LOST_EVT:
    {
      rtlsSrv_SyncLostEvt_t *pEvent = (rtlsSrv_SyncLostEvt_t *)pEvt->evtData;

      RTLSCtrl_processSyncLostEvt(pEvent->opcode,
                                  pEvent->syncHandle);
    }
    break;

    case RTLSSRV_PERIODIC_ADV_TERMINATE_SYNC:
    {
      RTLSCtrl_processTerminateSyncEvt();
    }
    break;

    case RTLSSRV_PERIODIC_ADV_RPT:
    {
      rtlsSrv_PeriodicAdvRpt_t *pReport = (rtlsSrv_PeriodicAdvRpt_t *)pEvt->evtData;

      RTLSCTRL_processPeriodicAdvReport(pReport->opcode,
                                        pReport->syncHandle,
                                        pReport->txPower,
                                        pReport->rssi,
                                        pReport->cteType,
                                        pReport->dataStatus,
                                        pReport->dataLen,
                                        pReport->pData);

      if(pReport->pData != NULL)
      {
        ICall_free(pReport->pData);
      }
    }
    break;

#ifdef RTLS_CTE
    case RTLSSRV_CONNECTION_CTE_IQ_REPORT_EVT:
    {
      rtlsSrv_connectionIQReport_t *pReport = (rtlsSrv_connectionIQReport_t *)pEvt->evtData;

      RTLSAoa_processAoaResults(pReport->connHandle,
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

    case RTLSSRV_CTE_REQUEST_FAILED_EVT:
    {
      rtlsSrv_cteReqFailed_t *pReqFail = (rtlsSrv_cteReqFailed_t *)pEvt->evtData;
      RTLSCtrl_sendDebugEvt((uint8_t *)"RTLS Services CTE Req Fail", (uint32_t)pReqFail->status);

    }
    break;

    case RTLSSRV_CL_AOA_ENABLE_EVT:
    {
      rtlsSrv_ClAoAEnableEvt_t *pEvent = (rtlsSrv_ClAoAEnableEvt_t *)pEvt->evtData;
      RTLSCtrl_processClAoaEnableEvt(pEvent->opcode,
                                     pEvent->status,
                                     pEvent->syncHandle);
    }
    break;

    case RTLSSRV_CL_CTE_IQ_REPORT_EVT:
    {
      rtlsSrv_clIQReport_t *pReport = (rtlsSrv_clIQReport_t *)pEvt->evtData;

      RTLSAoa_processAoaResults( pReport->syncHandle,
                                 pReport->rssi,
                                 pReport->channelIndex,
                                 pReport->sampleCount,
                                 pReport->sampleRate,
                                 pReport->sampleSize,
                                 pReport->sampleCtrl,
                                 pReport->slotDuration,
                                 pReport->numAnt,
                                 pReport->iqSamples );
    }
    break;
#endif /* RTLS_CTE */

    default:
      break;
  }

  // Free the payload
  if (pEvt->evtData)
  {
    ICall_free(pEvt->evtData);
  }
}


/*********************************************************************
 * @fn      RTLSCoordinator_rtlsCtrlMsgCb
 *
 * @brief   Callback given to RTLS Control
 *
 * @param  cmd - the command to be enqueued
 *
 * @return  none
 */
void RTLSCoordinator_rtlsCtrlMsgCb(uint8_t *cmd)
{
  // Enqueue the message to switch context
  RTLSCoordinator_enqueueMsg(RC_EVT_RTLS_CTRL_MSG_EVT, SUCCESS, (uint8_t *)cmd);
}

/*********************************************************************
 * @fn      RTLSCoordinator_rtlsSrvlMsgCb
 *
 * @brief   Callback given to RTLS Services
 *
 * @param   pRtlsSrvEvt - the command to be enqueued
 *
 * @return  none
 */
void RTLSCoordinator_rtlsSrvlMsgCb(rtlsSrv_evt_t *pRtlsSrvEvt)
{
  // Enqueue the message to switch context
  RTLSCoordinator_enqueueMsg(RC_EVT_RTLS_SRV_MSG_EVT, SUCCESS, (uint8_t *)pRtlsSrvEvt);
}

/*********************************************************************
*********************************************************************/
