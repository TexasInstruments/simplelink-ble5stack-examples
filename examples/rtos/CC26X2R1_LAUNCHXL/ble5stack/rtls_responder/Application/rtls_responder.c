/******************************************************************************

 @file  rtls_responder.c

 @brief This file contains the RTLS Responder sample application for use
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

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>

#include "rtls_responder.h"

#include "rtls_ctrl_api.h"
#include "rtls_ble.h"

#include "ble_user_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define RR_EVT_SCAN_ENABLED        0x01
#define RR_EVT_SCAN_DISABLED       0x02
#define RR_EVT_ADV_REPORT          0x03
#define RR_EVT_ADV_EVT             0x04
#define RR_EVT_PAIR_STATE          0x05
#define RR_EVT_PASSCODE_NEEDED     0x06
#define RR_EVT_INSUFFICIENT_MEM    0x07
#define RR_EVT_RTLS_CTRL_MSG_EVT   0x08
#define RR_EVT_RTLS_SRV_MSG_EVT    0x09
#define RR_EVT_CONN_EVT            0x0A

// Internal Events for RTOS application
#define RR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define RR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define RR_PERIODIC_EVT                      Event_Id_00

// Bitwise OR of all RTOS events to pend on
#define RR_ALL_EVENTS                        (RR_ICALL_EVT             | \
                                              RR_QUEUE_EVT             | \
                                              RR_PERIODIC_EVT)
// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     104

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_ACCEPT_ALL

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_1M

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                200 // 2 sec

// Default supervision timeout in 10ms
#define DEFAULT_UPDATE_CONN_TIMEOUT          200

// Task configuration
#define RR_TASK_PRIORITY                     1

#ifndef RR_TASK_STACK_SIZE
#define RR_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define RR_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Connection event registration
typedef enum
{
   NOT_REGISTERED     = 0x0,
   FOR_RTLS            = 0x2,
} connectionEventRegisterCause_u;

// Spin if the expression is not true
#define RTLSRESPONDER_ASSERT(expr) if (!(expr)) rtls_responder_spin();

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

// Hard coded PSM for passing data between central and peripheral
#define RTLS_PSM      0x0080
#define RTLS_PDU_SIZE MAX_PDU_SIZE

#ifdef USE_PERIODIC_ADV
// Periodic Advertising Intervals
#define PERIDIC_ADV_INTERVAL_MIN    240  // 300 ms
#define PERIDIC_ADV_INTERVAL_MAX    240  // 300 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} rrEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} rrPairStateData_t;

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
} rrPasscodeData_t;

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
} rrConnCB_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} rrGapAdvEventData_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

#define APP_EVT_EVENT_MAX  0xA
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
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Task configuration
Task_Struct rrTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(rrTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t rrTaskStack[RR_TASK_STACK_SIZE];

// Array of connection handles and information for each handle
static rrConnCB_t rrConnCB[MAX_NUM_BLE_CONNS];

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Number of scan results and scan result index
static uint8_t scanRes = 0;

// Scan result list
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];

// Advertisement data
static uint8_t advertData[] =
{
  0x0E,							// Length of this data
  GAP_ADTYPE_LOCAL_NAME_SHORT,  // Type of this data
  'R',
  'T',
  'L',
  'S',
  'R',
  'e',
  's',
  'p',
  'o',
  'n',
  'd',
  'e',
  'r',
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// Scan Response Data
static uint8_t scanRspData[] =
{
  14,   						  // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, // Type of this data
  'R',
  'T',
  'L',
  'S',
  'R',
  'e',
  's',
  'p',
  'o',
  'n',
  'd',
  'e',
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

#ifdef USE_PERIODIC_ADV
// Periodic Advertising Data
static uint8_t periodicData[] =
{
  'P',
  'e',
  'r',
  'i',
  'o',
  'd',
  'i',
  'c',
  'A',
  'd',
  'v'
};
#endif

// Advertising handles
static uint8 advHandleLegacy;
#ifdef USE_PERIODIC_ADV
static uint8 advHandleNCNS;      // Non-Connactable & Non-Scannable
#endif

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t  connectionEventRegisterCauseBitMap = NOT_REGISTERED;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void RTLSResponder_init( void );
static void RTLSResponder_taskFxn(UArg a0, UArg a1);

static uint8_t RTLSResponder_processStackMsg(ICall_Hdr *pMsg);
static void RTLSResponder_processAppMsg(rrEvt_t *pMsg);
static void RTLSResponder_processGapMessage(gapEventHdr_t *pMsg);
static void RTLSResponder_processPairState(rrPairStateData_t *pPairState);
static void RTLSResponder_processPasscode(rrPasscodeData_t *pPasscodeData);
static void RTLSResponder_processAdvEvent(rrGapAdvEventData_t *pEventData);
static void RTLSResponder_advertInit(void);
static void RTLSResponder_scanInit(void);
static void RTLSResponder_addDeviceInfo(GapScan_Evt_AdvRpt_t *deviceInfo);
static void RTLSResponder_advCb(uint32_t event, void *pBuf, uintptr_t arg);
static void RTLSResponder_scanCb(uint32_t evt, void* pMsg, uintptr_t arg);
static void RTLSResponder_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void RTLSResponder_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static status_t RTLSResponder_enqueueMsg(uint8_t event, void *pData);

// RTLS specific functions
static void RTLSResponder_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void RTLSResponder_processRtlsSrvMsg(rtlsSrv_evt_t *pEvt);
static void RTLSResponder_rtlsSrvlMsgCb(rtlsSrv_evt_t *pRtlsSrvEvt);
static void RTLSResponder_processRtlsCtrlMsg(uint8_t *pMsg);
static void RTLSResponder_processRTLSScanRes(GapScan_Evt_AdvRpt_t *deviceInfo);
static void RTLSResponder_processRTLSScanReq(void);
static void RTLSResponder_processRTLSConnReq(bleConnReq_t *bleConnReq);
static void RTLSResponder_processRTLSConnInfo(uint16_t connHandle);
static void RTLSResponder_processRTLSUpdateConnInterval(rtlsUpdateConnIntReq_t *updateReq);
static void RTLSResponder_terminateLinkReq(rtlsTerminateLinkReq_t *termInfo);
static void RTLSResponder_enableRtlsSync(rtlsEnableSync_t *enable);
static void RTLSResponder_connEvtCB(Gap_ConnEventRpt_t *pReport);

// L2CAP COC Handling
static void RTLSResponder_openL2CAPChanCoc(void);
static void RTLSResponder_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg);
static uint8_t RTLSResponder_processL2CAPDataEvent(l2capDataEvent_t *pMsg);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t RTLSResponder_BondMgrCBs =
{
  RTLSResponder_passcodeCb,       // Passcode callback
  RTLSResponder_pairStateCb       // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      rtls_responder_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void rtls_responder_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_createTask
 *
 * @brief   Task creation function for the RTLS Responder.
 */
void RTLSResponder_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = rrTaskStack;
  taskParams.stackSize = RR_TASK_STACK_SIZE;
  taskParams.priority = RR_TASK_PRIORITY;

  Task_construct(&rrTask, RTLSResponder_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      RTLSResponder_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void RTLSResponder_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", RR_TASK_PRIORITY);
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  // in build_config.opt in stack project.
  {
    //Change initial values of RX/TX PDU and Time, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251  //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120  //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Set Bond Manager parameters
  {
    // Don't send a pairing request after connecting; the peer device must initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
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
  VOID GAPBondMgr_Register(&RTLSResponder_BondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL);
  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL | GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  //Read the LE locally supported features
  HCI_LE_ReadLocalSupportedFeaturesCmd();

  // Initialize RTLS Services
  RTLSSrv_init(MAX_NUM_BLE_CONNS);
  RTLSSrv_register(RTLSResponder_rtlsSrvlMsgCb);
}

/*********************************************************************
 * @fn      RTLSResponder_taskFxn
 *
 * @brief   Application task entry point for the RTLS Responder.
 *
 * @param   a0, a1 - not used.
 */
static void RTLSResponder_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  RTLSResponder_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, RR_ALL_EVENTS,
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
            safeToDealloc = RTLSResponder_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & RR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          rrEvt_t *pMsg = (rrEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            RTLSResponder_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RTLSResponder_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      RTLSResponder_processGapMessage((gapEventHdr_t*) pMsg);
    break;

    case L2CAP_SIGNAL_EVENT:
     RTLSResponder_processL2CAPSignalEvent((l2capSignalEvent_t *)pMsg);
    break;

    case L2CAP_DATA_EVENT:
      safeToDealloc = RTLSResponder_processL2CAPDataEvent((l2capDataEvent_t *)pMsg);
    break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
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

#ifdef RTLS_CTE
              // Enable CTE
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_CONNECTION_CTE_REQUEST );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_CONNECTION_CTE_RESPONSE );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_ANTENNA_SWITCHING_DURING_CTE_RX );
              SET_FEATURE_FLAG( featSet[2], LL_FEATURE_RECEIVING_CTE );
#endif /* RTLS_CTE */

              // Update controller with modified features
              HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
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
                  RTLSResponder_processRTLSConnInfo(pCMU->connHandle);
                }
              }
            }
            break;

            default:
              break;
          }
        }
        break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
        break;

        default:
          break;
      }
    }

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RTLSResponder_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void RTLSResponder_processAppMsg(rrEvt_t *pMsg)
{
  if (pMsg->event <= APP_EVT_EVENT_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
  }

  switch (pMsg->event)
  {

    case RR_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

      char responderScanRsp[] = {'R','T','L','S','C','o','o','r','d','i','n','a','t','o','r'};

      // Filter results by the responder's scanRsp array
      if (memcmp(&pAdvRpt->pData[2], responderScanRsp, sizeof(responderScanRsp)) == 0)
      {
        RTLSResponder_addDeviceInfo(pAdvRpt);
      }

      // Free report payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
    }
    break;

    case RR_EVT_SCAN_DISABLED:
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

    case RR_EVT_ADV_EVT:
      RTLSResponder_processAdvEvent((rrGapAdvEventData_t*)(pMsg->pData));
      break;

    case RR_EVT_PAIR_STATE:
      RTLSResponder_processPairState((rrPairStateData_t*)(pMsg->pData));
      break;

    case RR_EVT_PASSCODE_NEEDED:
      RTLSResponder_processPasscode((rrPasscodeData_t*)(pMsg->pData));
      break;

    case RR_EVT_CONN_EVT:
      RTLSResponder_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    case RR_EVT_RTLS_CTRL_MSG_EVT:
      RTLSResponder_processRtlsCtrlMsg((uint8_t *)pMsg->pData);
      break;

    case RR_EVT_RTLS_SRV_MSG_EVT:
      RTLSResponder_processRtlsSrvMsg((rtlsSrv_evt_t *)pMsg->pData);
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if (pMsg->pData != NULL)
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void RTLSResponder_processGapMessage(gapEventHdr_t *pMsg)
{
  uint16_t connHandle;

  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        //Setup and start advertising
        RTLSResponder_advertInit();
      }

      //Setup scanning
      RTLSResponder_scanInit();
    }
    break;

#ifdef USE_PERIODIC_ADV
    case GAP_ADV_SET_PERIODIC_ADV_PARAMS_EVENT:
    {
      bStatus_t status = FAILURE;

      GapAdv_periodicAdvEvt_t *pPkt = (GapAdv_periodicAdvEvt_t*)pMsg;
      if( pPkt->status == SUCCESS )
      {
        // Set Periodic Advertising Data
        GapAdv_periodicAdvData_t periodicDataParams = {0x03, sizeof(periodicData), periodicData};
        status = GapAdv_SetPeriodicAdvData(advHandleNCNS, &periodicDataParams);
        RTLSRESPONDER_ASSERT(status == SUCCESS);
      }
    }
    break;

    case GAP_ADV_SET_PERIODIC_ADV_DATA_EVENT:
    {
      bStatus_t status = FAILURE;

      GapAdv_periodicAdvEvt_t *pPkt = (GapAdv_periodicAdvEvt_t*)pMsg;
      if( pPkt->status == SUCCESS )
      {
        // Enable the periodic advertising
        status = GapAdv_SetPeriodicAdvEnable(1, advHandleNCNS);
        RTLSRESPONDER_ASSERT(status == SUCCESS);
      }
    }
    break;

    case GAP_ADV_SET_PERIODIC_ADV_ENABLE_EVENT:
    {
#ifdef RTLS_CTE
      bStatus_t status = FAILURE;

      GapAdv_periodicAdvEvt_t *pPkt = (GapAdv_periodicAdvEvt_t*)pMsg;
      if( pPkt->status == SUCCESS )
      {
        // Set connectionless CTE transmit parameters
        status = RTLSSrv_SetCLCteTransmitParams(advHandleNCNS, 20, 0, 1, 0, NULL);
        RTLSRESPONDER_ASSERT(status == SUCCESS);
      }
#endif /* RTLS_CTE */
    }
    break;
#endif /* USE_PERIODIC_ADV */

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();

      connHandle = ((gapEstLinkReqEvent_t *) pMsg)->connectionHandle;

      if (connHandle != LINKDB_CONNHANDLE_INVALID && connHandle < MAX_NUM_BLE_CONNS &&
          ((gapEstLinkReqEvent_t *) pMsg)->hdr.status == SUCCESS)
      {
        rrConnCB[connHandle].isActive = TRUE;

        RTLSResponder_openL2CAPChanCoc();

#ifdef RTLS_CTE
        // Once connection is established we are ready to receive CTE requests
        RTLSSrv_setConnCteTransmitParams(connHandle, RTLSSRV_CTE_TYPE_AOA, 0, NULL);
        RTLSSrv_setConnCteResponseEnableCmd(connHandle, TRUE);
#endif /* RTLS_CTE */

        // We send out the connection information at this point
        // Note: we are not yet connected (will be after pairing)
        RTLSResponder_processRTLSConnInfo(connHandle);
      }
      else
      {
        // Notify RTLS Control that we are not connected
         RTLSCtrl_connResultEvt(0, RTLS_FAIL);
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
       rrConnCB[connHandle].isActive = FALSE;

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
        RTLSResponder_processRTLSConnInfo(connHandle);
      }
    }
    break;


    default:
      break;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void RTLSResponder_processPairState(rrPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;

  switch (state)
  {

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void RTLSResponder_processPasscode(rrPasscodeData_t *pPasscodeData)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS, passcode);
}


/*********************************************************************
 * @fn      RTLSResponder_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void RTLSResponder_processAdvEvent(rrGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
      break;

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
* @fn      RTLSResponder_advertInit
*
* @brief   Setup initial advertisement and start advertising from device init.
*
* @return  None.
*/
static void RTLSResponder_advertInit(void)
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
  status = GapAdv_create(&RTLSResponder_advCb, &advParamLegacy, &advHandleLegacy);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Load advertising data for set #1 that is statically allocated by the app
  status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                               sizeof(advertData), advertData);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Load scan response data for set #1 that is statically allocated by the app
  status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanRspData), scanRspData);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Set event mask for set #1
  status = GapAdv_setEventMask(advHandleLegacy,
                               GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                               GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                               GAP_ADV_EVT_MASK_SET_TERMINATED);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
  // Enable legacy advertising for set #1
  status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

#ifdef USE_PERIODIC_ADV
  // Create non connectable & non scannable advertising set #3
  GapAdv_params_t advParamNonConn = GAPADV_PARAMS_AE_NC_NS;

  // Create Advertisement set #3 and assign handle
  status = GapAdv_create(&RTLSResponder_advCb, &advParamNonConn,
                                                   &advHandleNCNS);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Load advertising data for set #3 that is statically allocated by the app
  status = GapAdv_loadByHandle(advHandleNCNS, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Set event mask for set #3
  status = GapAdv_setEventMask(advHandleNCNS,
                               GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                               GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                               GAP_ADV_EVT_MASK_SET_TERMINATED);

  // Enable non connectable & non scannable advertising for set #3
  status = GapAdv_enable(advHandleNCNS, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
  RTLSRESPONDER_ASSERT(status == SUCCESS);

  // Set Periodic Advertising parameters
  GapAdv_periodicAdvParams_t perParams = {PERIDIC_ADV_INTERVAL_MIN,
                                                PERIDIC_ADV_INTERVAL_MAX, 0x40};
  status = GapAdv_SetPeriodicAdvParams(advHandleNCNS, &perParams);
  RTLSRESPONDER_ASSERT(status == SUCCESS);
#endif
}

/*********************************************************************
* @fn      RTLSResponder_scanInit
*
* @brief   Setup initial device scan settings.
*
* @return  None.
*/
static void RTLSResponder_scanInit(void)
{
  uint8_t temp8;
  uint16_t temp16;

  // Setup scanning
  // For more information, see the GAP section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/

  // Register callback to process Scanner events
  GapScan_registerCb(RTLSResponder_scanCb, NULL);

  // Set Scanner Event Mask
  GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                       GAP_EVT_ADV_REPORT);

  // Set Scan PHY parameters
  GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_PASSIVE,
                       200, 100);

  // Set Advertising report fields to keep
  temp16 = RR_ADV_RPT_FIELDS;
  GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
  // Set Scanning Primary PHY
  temp8 = DEFAULT_SCAN_PHY;
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
  // Set LL Duplicate Filter
  temp8 = SCAN_FLT_DUP_ENABLE;
  GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

  // Set PDU type filter -
  // Only 'Connectable' and 'Complete' packets are desired.
  // It doesn't matter if received packets are
  // whether Scannable or Non-Scannable, whether Directed or Undirected,
  // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
  temp16 = SCAN_FLT_PDU_COMPLETE_ONLY;
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapScan_setParam", 0);
  GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);
}

/*********************************************************************
 * @fn      RTLSResponder_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void RTLSResponder_addDeviceInfo(GapScan_Evt_AdvRpt_t *deviceInfo)
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
    RTLSResponder_processRTLSScanRes(deviceInfo);

    // Add addr to scan result list
    memcpy(scanList[scanRes].addr, deviceInfo->addr, B_ADDR_LEN);
    scanList[scanRes].addrType = deviceInfo->addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_advCb
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void RTLSResponder_advCb(uint32_t event, void *pBuf, uintptr_t arg)
{
  rrGapAdvEventData_t *pData = ICall_malloc(sizeof(rrGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(RTLSResponder_enqueueMsg(RR_EVT_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSResponder_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
static void RTLSResponder_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = RR_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = RR_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = RR_EVT_SCAN_DISABLED;
  }
  else
  {
    return;
  }

  if (RTLSResponder_enqueueMsg(event,pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void RTLSResponder_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  rrPasscodeData_t *pData = ICall_malloc(sizeof(rrPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(RTLSResponder_enqueueMsg(RR_EVT_PASSCODE_NEEDED, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSResponder_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void RTLSResponder_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  rrPairStateData_t *pData = ICall_malloc(sizeof(rrPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(RTLSResponder_enqueueMsg(RR_EVT_PAIR_STATE, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      RTLSResponder_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t RTLSResponder_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  rrEvt_t *pMsg = ICall_malloc(sizeof(rrEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      RTLSResponder_processRtlsSrvMsg
 *
 * @brief   Handle processing messages from RTLS Services host module
 *
 * @param   pEvt - a pointer to the event
 *
 * @return  none
 */
static void RTLSResponder_processRtlsSrvMsg(rtlsSrv_evt_t *pEvt)
{
  if(!pEvt)
  {
    return;
  }

  switch (pEvt->evtType)
  {
#if defined (USE_PERIODIC_ADV) && defined (RTLS_CTE)
    case RTLSSRV_CL_CTE_EVT:
    {
      bStatus_t status = FAILURE;
      rtlsSrv_ClCmdCompleteEvt_t *pEvent = (rtlsSrv_ClCmdCompleteEvt_t *)pEvt->evtData;

      // Set Connectionless CTE parameters command complete
      if( pEvent->opcode == RTLSSRV_SET_CL_CTE_TRANSMIT_PARAMS )
      {
        if( pEvent->status == SUCCESS )
        {
          // Enable Connectionless CTE
          status = RTLSSrv_CLCteTransmitEnable(advHandleNCNS, 1);
          RTLSRESPONDER_ASSERT(status == SUCCESS);
        }
      }
      break;
    }
#endif /* USE_PERIODIC_ADV && RTLS_CTE */

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void RTLSResponder_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(RTLSResponder_enqueueMsg(RR_EVT_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void RTLSResponder_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Do a TOF Run, at the end of the active connection period
  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_RTLS) && rrConnCB[pReport->handle].isActive)
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

	RTLSCtrl_syncNotifyEvt(pReport->handle, status, pReport->nextTaskTime, pReport->lastRssi, pReport->channel);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_openL2CAPChanCoc
 *
 * @brief   Opens a communication channel between RTLS Coordinator/Responder
 *
 * @param   pairState - Verify that devices are paired
 *
 * @return  none
 */
static void RTLSResponder_openL2CAPChanCoc(void)
{
  l2capPsm_t psm;
  l2capPsmInfo_t psmInfo;

  if (L2CAP_PsmInfo(RTLS_PSM, &psmInfo) == INVALIDPARAMETER)
  {
    // Prepare the PSM parameters
    psm.initPeerCredits = L2CAP_MAX_NOF_CREDITS;
    psm.maxNumChannels = 1;
    psm.mtu = RTLS_PDU_SIZE;
    psm.peerCreditThreshold = 0;
    psm.pfnVerifySecCB = NULL;
    psm.psm = RTLS_PSM;
    psm.taskId = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity);

    // Register PSM with L2CAP task
    L2CAP_RegisterPsm(&psm);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processL2CAPSignalEvent
 *
 * @brief   Handle L2CAP signal events
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  none
 */
static void RTLSResponder_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case L2CAP_CHANNEL_ESTABLISHED_EVT:
    {
      l2capChannelEstEvt_t *pEstEvt = &(pMsg->cmd.channelEstEvt);

      // Connection established, save the CID
      if (pMsg->connHandle != LINKDB_CONNHANDLE_INVALID && pMsg->connHandle < MAX_NUM_BLE_CONNS)
      {
        rrConnCB[pMsg->connHandle].cocCID = pEstEvt->CID;

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
        RTLSCtrl_dataSentEvt(0, RTLS_SUCCESS);
      }
      else
      {
        RTLSCtrl_dataSentEvt(0, RTLS_FAIL);
      }
    }
    break;
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processL2CAPDataEvent
 *
 * design /ref 159098678
 * @brief   Handles incoming L2CAP data
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  the return value determines whether pMsg can be freed or not
 */
static uint8_t RTLSResponder_processL2CAPDataEvent(l2capDataEvent_t *pMsg)
{
  rtlsPacket_t *pRtlsPkt;
  static uint16_t packetCounter;
  (void) packetCounter;

  if (!pMsg)
  {
    // Caller needs to figure out by himself that pMsg is NULL
    return TRUE;
  }

  // This application doesn't care about other L2CAP data events other than RTLS
  // It is possible to expand this function to support multiple COC CID's
  pRtlsPkt = (rtlsPacket_t *)ICall_malloc(pMsg->pkt.len);

  // Check for malloc error
  if (!pRtlsPkt)
  {
    // Free the payload (must use BM_free here according to L2CAP documentation)
    BM_free(pMsg->pkt.pPayload);
    return TRUE;
  }

  // Copy the payload
  memcpy(pRtlsPkt, pMsg->pkt.pPayload, pMsg->pkt.len);

  // Free the payload (must use BM_free here according to L2CAP documentation)
  BM_free(pMsg->pkt.pPayload);

  // RTLS Control will handle the information in the packet
  RTLSCtrl_rtlsPacketEvt((uint8_t *)pRtlsPkt);

  return TRUE;
}

/*********************************************************************
 * @fn      RTLSResponder_enableRtlsSync
 *
 * @brief   This function is used by RTLS Control to notify the RTLS application
 *          to start sending synchronization events (for BLE this is a connection event)
 *
 * @param   enable - start/stop synchronization
 *
 * @return  none
 */
static void RTLSResponder_enableRtlsSync(rtlsEnableSync_t *enable)
{
  bStatus_t status = RTLS_FALSE;

  if (enable->enable == RTLS_TRUE)
  {
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
      status = Gap_RegisterConnEventCb(RTLSResponder_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, LINKDB_CONNHANDLE_ALL);
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
      Gap_RegisterConnEventCb(RTLSResponder_connEvtCB, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, LINKDB_CONNHANDLE_ALL);
    }
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processRtlsCtrlMsg
 *
 * @brief   Handle processing messages from RTLS Control
 *
 * @param   msg - a pointer to the message
 *
 * @return  none
 */
static void RTLSResponder_processRtlsCtrlMsg(uint8_t *pMsg)
{
  rtlsCtrlReq_t *pReq;

  // Sanity check
  if (!pMsg)
  {
    return;
  }

  // Cast to appropriate struct
  pReq = (rtlsCtrlReq_t *)pMsg;

  switch(pReq->reqOp)
  {
    case RTLS_REQ_CONN:
    {
      RTLSResponder_processRTLSConnReq((bleConnReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_SCAN:
    {
      RTLSResponder_processRTLSScanReq();
    }
    break;

    case RTLS_REQ_ENABLE_SYNC:
    {
      RTLSResponder_enableRtlsSync((rtlsEnableSync_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_TERMINATE_LINK:
    {
      RTLSResponder_terminateLinkReq((rtlsTerminateLinkReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_UPDATE_CONN_INTERVAL:
    {
      RTLSResponder_processRTLSUpdateConnInterval((rtlsUpdateConnIntReq_t *)pReq->pData);
    }
    break;

    case RTLS_REQ_GET_ACTIVE_CONN_INFO:
    {
      rtlsGetActiveConnInfo_t *pConnInfoReq = (rtlsGetActiveConnInfo_t *)pReq->pData;

      if (pConnInfoReq)
      {
        RTLSResponder_processRTLSConnInfo(pConnInfoReq->connHandle);
      }
    }
    break;

    default:
      break;
  }

  if (pReq->pData != NULL)
  {
    ICall_free(pReq->pData);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_processRTLSScanRes
 *
 * @brief   Process a scan response and forward to RTLS Control
 *
 * @param   deviceInfo - a single scan response
 *
 * @return  none
 */
static void RTLSResponder_processRTLSScanRes(GapScan_Evt_AdvRpt_t *deviceInfo)
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
 * @fn      RTLSResponder_processRTLSScanReq
 *
 * @brief   Process a scan request
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSResponder_processRTLSScanReq(void)
{
  scanRes = 0;

  // Start discovery
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
}

/*********************************************************************
 * @fn      RTLSResponder_processRTLSConnReq
 *
 * @brief   Start the connection process with another device
 *
 * @param   bleConnReq - pointer from RTLS control containing connection params
 *
 * @return  none
 */
static void RTLSResponder_processRTLSConnReq(bleConnReq_t *bleConnReq)
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
 * @fn      RTLSResponder_processRTLSConnRes
 *
 * @brief   Process a connection established event - send conn info to RTLS Control
 *
 * @param   connHandle - connection handle
 *
 * @return  none
 */
static void RTLSResponder_processRTLSConnInfo(uint16_t connHandle)
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
 * @fn      RTLSResponder_processRTLSUpdateConnInterval
 *
 * @brief   Update connection interval
 *
 * @param   updateReq - pointer from RTLS control containing connection params
 *
 * @return  none
 */
static void RTLSResponder_processRTLSUpdateConnInterval(rtlsUpdateConnIntReq_t *updateReq)
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
 * @fn      RTLSResponder_terminateLinkReq
 *
 * @brief   Terminate active link
 *
 * @param   termInfo - information about the connection to terminate
 *
 * @return  none
 */
static void RTLSResponder_terminateLinkReq(rtlsTerminateLinkReq_t *termInfo)
{
  if (termInfo->connHandle != LINKDB_CONNHANDLE_INVALID && termInfo->connHandle < MAX_NUM_BLE_CONNS)
  {
    L2CAP_DisconnectReq(rrConnCB[termInfo->connHandle].cocCID);
  }
  else
  {
    RTLSCtrl_sendDebugEvt((uint8_t *)"Connection Handle invalid", LINKDB_CONNHANDLE_INVALID);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_rtlsCtrlMsgCb
 *
 * @brief   Callback given to RTLS Control
 *
 * @param  cmd - the command to be enqueued
 *
 * @return  none
 */
void RTLSResponder_rtlsCtrlMsgCb(uint8_t *cmd)
{
  // Enqueue the message to switch context
  if(RTLSResponder_enqueueMsg(RR_EVT_RTLS_CTRL_MSG_EVT, (uint8_t *)cmd) != SUCCESS)
  {
    ICall_free(cmd);
  }
}

/*********************************************************************
 * @fn      RTLSResponder_rtlsSrvlMsgCb
 *
 * @brief   Callback given to RTLS Services
 *
 * @param   pRtlsSrvEvt - the command to be enqueued
 *
 * @return  none
 */
static void RTLSResponder_rtlsSrvlMsgCb(rtlsSrv_evt_t *pRtlsSrvEvt)
{
  // Enqueue the message to switch context
  RTLSResponder_enqueueMsg(RR_EVT_RTLS_SRV_MSG_EVT, (uint8_t *)pRtlsSrvEvt);
}

/*********************************************************************
*********************************************************************/
