/******************************************************************************

   @file  multi_sensor.c

   This file contains the Launchpad SensorTag Kit sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

   Group: WCS, BTS
   Target Device: cc13x2_26x2

 ******************************************************************************

 Copyright (c) 2015-2019, Texas Instruments Incorporated
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
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/utils/List.h>
#include <ti/drivers/apps/LED.h>
#include <ti/common/cc26xx/uartlog/UartLog.h>
#include <ti/display/AnsiColor.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include <icall.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
/* Bluetooth Services */
#include <devinfoservice.h>
#include <led_service.h>
#include <profiles/oad/cc26xx/oad.h>
/* Application specific includes */
#include <ti_drivers_config.h>
#include <sensor_common.h>
#include <multi_sensor.h>
#include <util.h>

/*********************************************************************
 * MACROS
 */

#define MS_TO_TICKS(ms) (ms * (1000/Clock_tickPeriod))


/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define MS_TASK_PRIORITY                     2
#ifndef MS_TASK_STACK_SIZE
  #define MS_TASK_STACK_SIZE                 4000
#endif

// Internal Events used by OAD profile
#define MS_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define MS_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02

// Internal Events for RTOS application
#define MS_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define MS_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define MS_ALL_EVENTS                        (MS_ICALL_EVT         | \
                                                 MS_APP_MSG_EVT       | \
                                                 MS_OAD_QUEUE_EVT     | \
                                                 MS_OAD_COMPLETE_EVT)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define MS_PAIRSTATE_EVT         1  /* The pairing state is updated                */
#define MS_PASSCODE_EVT          2  /* A pass-code/PIN is requested during pairing */
#define MS_ADV_EVT               3  /* A subscribed advertisement activity         */
#define MS_START_ADV_EVT         4  /* Request advertisement start from task ctx   */
#define MS_CONN_EVT              5  /* Connection Event End notice                 */
#define MS_ADV_STATE_CHANGE_EVT  6  /* Used to switch to task context to change adv state */
#define MS_DEFERRED_EVT          7  /* Used to switch context to app */

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0

// Default values for Data Length Extension
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

// 100 ms
#define DEFAULT_ADV_INT 160

// Boot check timings in milliseconds
#define BOOTCHECK_SLEEP_DURATION_MS       5000
#define BOOTCHECK_SLEEP_INTERVAL_MS        200

/*********************************************************************
 * TYPEDEFS
 */
// Struct for messages sent to the application task
typedef struct
{
    uint8_t event;
    void    *pData;
} msMsg_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} msGapAdvEventData_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                    // Connection Handle
} msConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task configuration
Task_Struct msTask;
#if defined __TI_COMPILER_VERSION__
  #pragma DATA_ALIGN(appTaskStack, 8)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__ ((aligned (8)))
#else
  #pragma data_alignment=8
#endif
uint8_t appTaskStack[MS_TASK_STACK_SIZE];

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

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Multi-Sensor";

// Advertisement data
static uint8_t advertData[] =
{
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // complete name
    13, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'M',
    'u',
    'l',
    't',
    'i',
    '-',
    'S',
    'e',
    'n',
    's',
    'o',
    'r'
};

// Scan Response Data
static uint8_t scanRspData[] =
{
    // service UUID, to notify central devices what services are included
    // in this peripheral
    (ATT_UUID_SIZE + 0x01),   // length of this data, LED service UUID + header
    GAP_ADTYPE_128BIT_MORE,   // some of the UUID's, but not all
    TI_BASE_UUID_128(LED_SERVICE_SERV_UUID),
};

// Per-handle connection info
static msConnRec_t connList[MAX_NUM_BLE_CONNS];

// Advertising handles
static uint8_t advHandleLegacy;
static uint8_t advHandleLongRange;

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;
static bool oadWaitReboot = false;

// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t sendSvcChngdOnNextBoot = FALSE;

// Flag to indicate that OAD is in progress
static bool oad_in_progress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/* Task functions */
static void      MultiSensor_init(void);
static void      MultiSensor_taskFxn(UArg a0, UArg a1);

/* Event message processing functions */
static void      MultiSensor_processApplicationMessage(msMsg_t *pMsg);
static uint8_t   MultiSensor_processGATTMsg(gattMsgEvent_t *pMsg);
static void      MultiSensor_processGapMessage(gapEventHdr_t *pMsg);
static void      MultiSensor_processHCIMsg(ICall_HciExtEvt *pMsg);
static void      MultiSensor_processAdvEvent(msGapAdvEventData_t *pEventData);

/* Stack or profile callback function */
static void      MultiSensor_advCallback(uint32_t event, void *pBuf, uintptr_t arg);

/* Connection handling functions */
static uint8_t   MultiSensor_getConnIndex(uint16_t connHandle);
static uint8_t   MultiSensor_clearConnListEntry(uint16_t connHandle);
static uint8_t   MultiSensor_addConn(uint16_t connHandle);
static uint8_t   MultiSensor_removeConn(uint16_t connHandle);
static void      MultiSensor_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);
static void      MultiSensor_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void      MultiSensor_connEvtCB(Gap_ConnEventRpt_t *pReport);

/* Utility functions */
static bStatus_t MultiSensor_enqueueMsg(uint8_t event,  void *pData);
static void      MultiSensor_processOadWriteCB(uint8_t event, uint16_t arg);
static void      MultiSensor_processL2CAPMsg(l2capSignalEvent_t *pMsg);
static void      MultiSensor_bootManagerCheck(void);
static void      MultiSensor_eraseExternalFlash(void);
static void      MultiSensor_revertToFactoryImage(void);
static char    * util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst,
                            uint8_t dst_len, uint8_t reverse);
static char    * util_getLocalNameStr(const uint8_t *advData, uint8_t len);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// OAD Service callback handler.
static oadTargetCBs_t MultiSensor_oadCBs =
{
    .pfnOadWrite = MultiSensor_processOadWriteCB // Write Callback.
};

/*********************************************************************
 * PUBLIC FUNCTION DEFINITIONS
 */

void MultiSensor_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = MS_TASK_STACK_SIZE;
    taskParams.priority = MS_TASK_PRIORITY;

    Task_construct(&msTask, MultiSensor_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * PRIVATE FUNCTION DEFINITIONS
 */

/**
 * Main stack configuration and initialization function for the application.
 *
 * Contains application specific initialization (ie. hardware initialization /
 * setup, table initialization, power up notification, etc), and profile
 * initialization / setup.
 */
static void MultiSensor_init(void)
{
    // Separator to distinguish resets in log
    Log_info0("==============================================================");

    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // Set the Device Name characteristic in the GAP GATT Service
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configure GAP to accept all parameter updates.
    uint16_t paramUpdateDecision = GAP_UPDATE_REQ_ACCEPT_ALL;
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);

    // This application does not support pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);

    // BLE Service initialization
    GGS_AddService(GAP_SERVICE);               // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT Service
    DevInfo_AddService();                      // Device Information Service

    /*
     * NOTE!!
     * OAD_open() must be called before Sensors_init() because there
     * is no arbitration between the Flash and Accelerometer SPI writes
     */
    // Open the OAD module and add the OAD service to the application
    if(OAD_SUCCESS == OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
    {
        // Register the OAD callback with the application
        OAD_register(&MultiSensor_oadCBs);
        Log_info0("OAD Opened successfully");

        // Capture the current OAD version and log it
        static uint8_t versionStr[OAD_SW_VER_LEN + 1];
        OAD_getSWVersion(versionStr, OAD_SW_VER_LEN);
        versionStr[OAD_SW_VER_LEN] = 0;

        // Display Image version
        Log_info1("OAD Image version: %s", (uintptr_t)versionStr);
    }
    else
    {
        Log_error0("OAD failed to open");
    }

    // Initialize sensors, sensor profiles, sensor services
    // This will automatically initialize GATT profiles for which ever
    // sensor successfully enable
    if (Sensors_init() != SUCCESS)
    {
      Log_error0("Sensor task failed to initialize");
    }

    // Check for reset to factory image. This must be called after Sensors_init
    // since it will use SAIl drivers
    MultiSensor_bootManagerCheck();

    // Start Bond Manager
    VOID GAPBondMgr_Register(NULL);

    // Register with GAP for HCI/Host messages in order to receive HCI events.
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Extended Data Length Feature is already enabled by default
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone are essentially needing 251/2120, so we set them here.
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);

    // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
    GATT_InitClient();

    // Initialize Connection List
    MultiSensor_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

    // Set to use max tx power
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, ADDRMODE_PUBLIC, NULL);
}

/**
 * Application task entry point
 *
 * @param a0 not used.
 * @param a1 not used
 */
static void MultiSensor_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    MultiSensor_init();

    // Application main loop
    for(;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, MS_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if(events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if(ICall_fetchServiceMsg(&src, &dest, (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    // Call appropriate processing function based on layer of stack message
                    switch(pMsg->hdr.event)
                    {
                    case GAP_MSG_EVENT:
                        MultiSensor_processGapMessage((gapEventHdr_t*) pMsg);
                        break;

                    case GATT_MSG_EVENT:
                        safeToDealloc = MultiSensor_processGATTMsg((gattMsgEvent_t *)pMsg);
                        break;

                    case HCI_GAP_EVENT_EVENT:
                        MultiSensor_processHCIMsg(pMsg);
                        break;

                    case L2CAP_SIGNAL_EVENT:
                        MultiSensor_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
                        break;
                    }
                }

                if(pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // Process messages sent from another task or another context.
            while(!Queue_empty(appMsgQueueHandle))
            {
                msMsg_t *pMsg = (msMsg_t *)Util_dequeueMsg(appMsgQueueHandle);
                if(pMsg)
                {
                    // Process application-layer message probably sent from ourselves.
                    MultiSensor_processApplicationMessage(pMsg);
                    // Free the received message.
                    ICall_free(pMsg);
                }
            }

            // OAD events
            if(events & MS_OAD_QUEUE_EVT)
            {
                if (oad_in_progress  == FALSE)
                {
                  oad_in_progress = TRUE;

                  /*
                   * If this is the beginning of an OAD (first event received),
                   * disable sensors because:
                   *  - the accelerometer SPI and external flash SPI can not
                   *    be used simultaneously
                   *  - this will maximize over-the-air bandwidth for OAD
                   *    as there will be no sensor profile communications
                    */
                  Sensors_disable();

                  /*
                   * Reset the base configuration of the SPI pins. The sensor
                   * controller interface files will have modified the config
                   * for accelerometer usage.
                   */
                  GPIO_resetConfig(CONFIG_GPIO_0);
                  GPIO_resetConfig(CONFIG_GPIO_1);
                  GPIO_resetConfig(CONFIG_GPIO_2);
                  GPIO_resetConfig(CONFIG_GPIO_3);
                }

                // Process the OAD Message Queue
                uint8_t status = OAD_processQueue();

                // If the OAD state machine encountered an error, print it
                if(status == OAD_DL_COMPLETE)
                {
                    Log_info0("OAD DL Complete, wait for enable");
                }
                else if(status == OAD_IMG_ID_TIMEOUT)
                {
                    Log_info0("ImgID Timeout, disconnecting");

                    // This may be an attack, terminate the link,
                    // Note HCI_DISCONNECT_REMOTE_USER_TERM seems to most closet reason for
                    // termination at this state
                    MAP_GAP_TerminateLinkReq( OAD_getactiveCxnHandle(),
                                              HCI_DISCONNECT_REMOTE_USER_TERM);
                }
                else if(status != OAD_SUCCESS)
                {
                    Log_info1("OAD Error: %d", status);
                }
            }

            if(events & MS_OAD_COMPLETE_EVT)
            {
                // Register for L2CAP Flow Control Events
                L2CAP_RegisterFlowCtrlTask(selfEntity);
                oad_in_progress = FALSE;
            }
        }
    }
}

/**
 * Process L2CAP messages and events.
 *
 * @param   pMsg - L2CAP signal buffer from stack
 */
static void MultiSensor_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
    static bool firstRun = TRUE;

    if ((pMsg->opcode) == L2CAP_NUM_CTRL_DATA_PKT_EVT)
    {
          /*
           * We cannot reboot the device immediately after receiving
           * the enable command, we must allow the stack enough time
           * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
           * command. This command will determine the number of
           * packets currently queued up by the LE controller.
           */
          if(firstRun)
          {
              firstRun = false;

              // We only want to set the numPendingMsgs once
              numPendingMsgs = MAX_NUM_PDU -
                               pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

              // Wait until all PDU have been sent on cxn events
              Gap_RegisterConnEventCb(MultiSensor_connEvtCB,
                                      GAP_CB_REGISTER,
                                      GAP_CB_CONN_EVENT_ALL,
                                      OAD_getactiveCxnHandle());

              // Set the flag so that the connection event callback will
              // be processed in the context of a pending OAD reboot
              oadWaitReboot = true;
          }
      }
}

/**
 * Process GATT messages and events.
 *
 * @param pMsg - message to process
 *
 * @return TRUE if safe to deallocate incoming message
 * @return FALSE otherwise.
 */
static uint8_t MultiSensor_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if(pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if(pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
        Log_info1("Updated MTU Size: %d", pMsg->msg.mtuEvt.MTU);

        if (io.led.ready)
        {
          // Waiting until the MTU has been updated is a long enough time
          // to indicate that a connection has been formed
          LED_setOff(io.led.green);
        }
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return(TRUE);
}

/**
 * MultiSensor_processApplicationMessage
 *
 * Handle application messages
 *
 * These are messages not from the BLE stack, but from the application itself.
 *
 * For example, in a Software Interrupt (Swi) it is not possible to call any
 * BLE APIs, so instead the Swi function must send a message to the application Task
 * for processing in Task context.
 *
 * @param pMsg  Pointer to the message of type msMsg_t.
 */
static void MultiSensor_processApplicationMessage(msMsg_t *pMsg)
{
    switch(pMsg->event)
    {
      case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

      case MS_ADV_EVT:
          MultiSensor_processAdvEvent((msGapAdvEventData_t*)(pMsg->pData));
          break;

      case MS_START_ADV_EVT:
          if(linkDB_NumActive() < MAX_NUM_BLE_CONNS)
          {
              // Enable advertising if there is room for more connections
              GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
          }
          break;

      case MS_CONN_EVT:
        MultiSensor_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        break;

      default:
        break;
    }

    if(pMsg->pData != NULL)
    {
        ICall_free(pMsg->pData);
    }
}

/**
 * Process an incoming GAP event.
 *
 * @param pMsg message to process
 */
static void MultiSensor_processGapMessage(gapEventHdr_t *pMsg)
{
    switch(pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

        if(pPkt->hdr.status == SUCCESS)
        {
            // Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

            // Set Device Info Service Parameter with system ID.
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);

            // Display device address
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("Device has started with address: " \
                      ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);

            // Display advertising name
            Log_info1("Name in advertData array: " \
                      ANSI_COLOR(FG_YELLOW) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)util_getLocalNameStr(advertData,
                                                      sizeof(advertData)));

            // Setup and start Advertising
            GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;
            advParamLegacy.primIntMin = DEFAULT_ADV_INT;
            advParamLegacy.primIntMax = DEFAULT_ADV_INT;

            // Create Advertisement set #1 and assign handle
            bStatus_t status = GapAdv_create(&MultiSensor_advCallback, &advParamLegacy,
                                             &advHandleLegacy);

            // Load advertising data for set #1 that is statically allocated by the app
            status |= GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);

            // Load scan response data for set #1 that is statically allocated by the app
            status |= GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                         sizeof(scanRspData),
                                         scanRspData);

            // Set event mask for set #1
            status |= GapAdv_setEventMask(advHandleLegacy,
                                         GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                         GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                         GAP_ADV_EVT_MASK_SET_TERMINATED);

            // Enable legacy advertising for set #1
            status |= GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);

            if (status != SUCCESS)
            {
              Log_error0("Legacy advertising set failed to enable.");
            }

            // Setup and start Advertising
            GapAdv_params_t advParamLongRange = GAPADV_PARAMS_AE_LONG_RANGE_CONN;
            advParamLongRange.primIntMin = DEFAULT_ADV_INT;
            advParamLongRange.primIntMax = DEFAULT_ADV_INT;
            advParamLongRange.txPower = 126; // Use max Tx power
            advParamLongRange.eventProps |= GAP_ADV_PROP_TX_POWER;

            // Create Advertisement set #2 and assign handle
            status = GapAdv_create(&MultiSensor_advCallback, &advParamLongRange,
                                   &advHandleLongRange);

            // Load advertising data for set #2 that is statically allocated by the app
            status |= GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);

            // Set event mask for set #2
            status |= GapAdv_setEventMask(advHandleLongRange,
                                         GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                         GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                         GAP_ADV_EVT_MASK_SET_TERMINATED);

            // Enable long range advertising for set #2
            status |= GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
            if (status != SUCCESS)
            {
              Log_error0("Extended advertising set failed to enable.");
            }
        }

        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info2("Link establish event, status 0x%02x. Num Conns: %d",
                  pPkt->hdr.status, linkDB_NumActive());

        if(pPkt->hdr.status == SUCCESS)
        {
            if (io.led.ready)
            {
              // Turn on green LED. This will be turned on after the MTU
              // updated event is received
              LED_stopBlinking(io.led.blue);
              LED_setOff(io.led.blue);
              LED_setOff(io.led.red);
              LED_setOn(io.led.green, 50);
            }

            // Add connection to list
            MultiSensor_addConn(pPkt->connectionHandle);

            // Display the address of this connection
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("Connected. Peer address: " \
                        ANSI_COLOR(FG_GREEN)"%s"ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info0("Device Disconnected!");
        Log_info1("Num Conns: %d", linkDB_NumActive());

        // Remove the connection from the list
        MultiSensor_removeConn(pPkt->connectionHandle);

        // Cancel the OAD if one is going on
        // A disconnect forces the peer to re-identify
        OAD_cancel();

        // Disable profiles / sensors
        Sensors_disable();

        // Enable advertising since there is now an open connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }
    break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
        MultiSensor_handleUpdateLinkEvent((gapLinkUpdateEvent_t *)pMsg);
        break;
    }
}

void MultiSensor_processHCIMsg(ICall_HciExtEvt *pEvt)
{
    ICall_Hdr *pMsg = (ICall_Hdr *)pEvt;

    // Process HCI message
    switch(pMsg->status)
    {
    // HCI Command Complete Events here
    case HCI_VE_EVENT_CODE:
    {
       hciEvt_VSCmdComplete_t *myEvent = (hciEvt_VSCmdComplete_t *)pEvt;
       if (myEvent->cmdOpcode == HCI_EXT_SET_TX_POWER)
       {
         uint8_t status = myEvent->pEventParam[2];

         if (status == SUCCESS)
         {
            Log_info0("Tx Power successfully updated");
         }
         else
         {
            Log_info1("Tx Power failed with status %x", status);
         }
       }
       break;
    }

    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
        break;

    // HCI Commands Events
    case HCI_COMMAND_STATUS_EVENT_CODE:
    {
        hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
        switch(pMyMsg->cmdOpcode)
        {
        case HCI_LE_SET_PHY:
        {
            if(pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                Log_info0("PHY Change failure, peer does not support this");
            }
            else
            {
                Log_info1("PHY Update Status Event: 0x%x",
                          pMyMsg->cmdStatus);
            }
        }
        break;
        }
    }
    break;

    // LE Events
    case HCI_LE_EVENT_CODE:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        // A Phy Update Has Completed or Failed
        if(pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
        {
            if(pPUC->status != SUCCESS)
            {
                Log_info0("PHY Change failure");
            }
            else
            {
                // Only symmetrical PHY is supported.
                // rxPhy should be equal to txPhy.
                Log_info1("PHY Updated to %s",
                          (uintptr_t)((pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value"));
            }
        }
    }
    break;
    }
}

/**
 * Process advertising event in app context
 *
 * @param pEventData
 *
 * @return RUE if safe to deallocate incoming message
 * @return FALSE otherwise.
 */
static void MultiSensor_processAdvEvent(msGapAdvEventData_t *pEventData)
{
    switch(pEventData->event)
    {
    /* Sent on the first advertisement after a GapAdv_enable */
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        Log_info1("Adv Set %d Enabled", *(uint8_t *)(pEventData->pBuf));

        if (io.led.ready)
        {
          // Slow blink blue LED
          LED_setOff(io.led.green);
          LED_setOff(io.led.red);
          LED_setOn(io.led.blue, 50);
          LED_startBlinking(io.led.blue, 2000, LED_BLINK_FOREVER);
        }
        break;

    /* Sent after advertising stops due to a GapAdv_disable */
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        Log_info1("Adv Set %d Disabled", *(uint8_t *)(pEventData->pBuf));
        break;

    /* Sent when an advertisement set is terminated due to a
     * connection establishment */
    case GAP_EVT_ADV_SET_TERMINATED:
    {
        GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

        Log_info2("Adv Set %d disabled after conn %d",
                  advSetTerm->handle, advSetTerm->connHandle);
    }
    break;

    /* Sent when an operation could not complete because of a lack of memory.
       This message is not allocated on the heap and must not be freed */
    case GAP_EVT_INSUFFICIENT_MEMORY:
        Log_error0("GAP Advertising module has encountered a heap exhaustion.");
        break;

    default:
        break;
    }

  // All events have associated memory to free except the insufficient memory event
  if ((pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY) && (pEventData->pBuf != NULL))
  {
    ICall_free(pEventData->pBuf);
  }
}

/**
 * Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void MultiSensor_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // If we are waiting for an OAD Reboot, process connection events to ensure
  // that we are not waiting to send data before restarting
  if(oadWaitReboot)
  {
      // Wait until all pending messages are sent
      if(numPendingMsgs == 0)
      {
          // Store the flag to indicate that a service changed IND will
          // be sent at the next boot
          sendSvcChngdOnNextBoot = TRUE;

          uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                          sizeof(sendSvcChngdOnNextBoot),
                                          (uint8 *)&sendSvcChngdOnNextBoot);
          if(status != SUCCESS)
          {
              Log_error1("SNV WRITE FAIL: %d", status);
          }

          // Reset the system
          SysCtrlSystemReset();
      }
      else
      {
        numPendingMsgs--;
      }
  }
}

/**
 * Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void MultiSensor_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if((MultiSensor_enqueueMsg(MS_CONN_EVT, pReport) != SUCCESS) &&
     (pReport != NULL))
  {
    ICall_free(pReport);
  }
}

/**
 * Receive and parse a parameter update that has occurred.
 *
 * @param pEvt pointer to stack event message
 */
static void MultiSensor_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt)
{
    // Get the address from the connection handle
    linkDBInfo_t linkInfo;
    linkDB_GetInfo(pEvt->connectionHandle, &linkInfo);

    static uint8_t addrStr[3 * B_ADDR_LEN + 1];
    util_arrtohex(linkInfo.addr, B_ADDR_LEN, addrStr, sizeof addrStr,
                  UTIL_ARRTOHEX_REVERSE);

    if(pEvt->status == SUCCESS)
    {
        uint8_t ConnIntervalFracture = 25*(pEvt->connInterval % 4);
        // Display the address of the connection update
        Log_info5(
            "Updated params for %s, interval: %d.%d ms, latency: %d, timeout: %d ms",
            (uintptr_t)addrStr,
            (uintptr_t)(pEvt->connInterval*CONN_INTERVAL_MS_CONVERSION),
            ConnIntervalFracture,  pEvt->connLatency,
            pEvt->connTimeout*CONN_TIMEOUT_MS_CONVERSION);
    }
    else
    {
        // Display the address of the connection update failure
        Log_info2("Update Failed 0x%02x: %s", pEvt->opcode, (uintptr_t)addrStr);
    }
}

/**
 * Add a device to the connected device list
 *
 * @param connHandle connection handle
 *
 * @return bleMemAllocError a param update event could not be sent
 * @return SUCCESS otherwise
 */
static uint8_t MultiSensor_addConn(uint16_t connHandle)
{
    uint8_t i;
    uint8_t status = bleNoResources;

    // Try to find an available entry
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;
            status = SUCCESS;
            break;
        }
    }

    return(status);
}

/**
 * Find index in the connected device list by connHandle
 *
 * @param connHandle connection handle
 *
 * @return the index of the entry that has the given connection handle.
 * @return MAX_NUM_BLE_CONNS if there is no match
 */
static uint8_t MultiSensor_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == connHandle)
        {
            return(i);
        }
    }

    return(MAX_NUM_BLE_CONNS);
}

/**
 * Clear the connection information structure held locally.
 *
 * @Note LINKDB_CONNHANDLE_ALL will always succeed.
 *
 * @param connHandle connection handle
 *
 * @return  SUCCESS if connHandle found valid index
 * @return bleInvalidRange if index wasn't found
 */
static uint8_t MultiSensor_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if(connHandle != LINKDB_CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = MultiSensor_getConnIndex(connHandle);
        if(connIndex >= MAX_NUM_BLE_CONNS)
        {
            return(bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
        }
    }

    return(SUCCESS);
}

/**
 * Remove a device from the connected device list
 *
 * @param connHandle connection handle
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 * @return MAX_NUM_BLE_CONNS will if connHandle is not found
 */
static uint8_t MultiSensor_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = MultiSensor_getConnIndex(connHandle);

    if(connIndex < MAX_NUM_BLE_CONNS)
    {
      // Clear Connection List Entry
      MultiSensor_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/******************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 *****************************************************************************/

/**
 * GapAdv module callback (in stack context)
 *
 * @param pMsg message to process
 * @param pBuf data potentially accompanying event
 * @param arg not used
 */
static void MultiSensor_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    msGapAdvEventData_t *eventData = ICall_malloc(sizeof(msGapAdvEventData_t));

    if(eventData != NULL)
    {
        eventData->event = event;
        eventData->pBuf = pBuf;

        if((MultiSensor_enqueueMsg(MS_ADV_EVT, eventData) != SUCCESS) &&
                (eventData != NULL))
        {
          ICall_free(eventData);
        }
    }
}

/**
 * Process an OAD event
 *
 * @param connHandle the connection Handle this request is from.
 * @param bim_var    bim_var to set before resetting.
 */
static void MultiSensor_processOadWriteCB(uint8_t event, uint16_t arg)
{
  Event_post(syncEvent, event);
}

/******************************************************************************
 *
 *  Utility functions
 *
 *****************************************************************************/

/**
 * Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 *
 * @return SUCCESS message was allocated and queued,
 * @return FAILURE enqueueing failed
 * @return bleMemAllocError message couldn't be allocated
 */
static bStatus_t MultiSensor_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    msMsg_t *pMsg = ICall_malloc(sizeof(msMsg_t));

    if(pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return(bleMemAllocError);
}

/**
 * util_arrtohex
 *
 * Convert {0x01, 0x02} to "01:02"
 *
 * @param src source byte-array
 * @param src_len length of array
 * @param dst destination string-array
 * @param dst_len length of array
 *
 * @return array as string
 */
char * util_arrtohex(uint8_t const *src, uint8_t src_len,
                     uint8_t *dst, uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if(reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while(src_len && avail > 3)
    {
        if(avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if(src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return((char *)dst);
}

/**
 * Extract the LOCALNAME from Scan/AdvData
 *
 * @param data Pointer to the advertisement or scan response data
 * @param len Length of advertisement or scan response data
 *
 * @return Pointer to null-terminated string with the adv local name.
 */
static char * util_getLocalNameStr(const uint8_t *data, uint8_t len)
{
    uint8_t nuggetLen = 0;
    uint8_t nuggetType = 0;
    uint8_t advIdx = 0;

    static char localNameStr[32] = { 0 };
    memset(localNameStr, 0, sizeof(localNameStr));

    for(advIdx = 0; advIdx < len; )
    {
        nuggetLen = data[advIdx++];
        nuggetType = data[advIdx];
        if((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
            nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) )
        {
            uint8_t len_temp = nuggetLen < (sizeof(localNameStr)-1)? (nuggetLen - 1):(sizeof(localNameStr)-2);
            // Only copy the first 31 characters, if name bigger than 31.
            memcpy(localNameStr, &data[advIdx + 1], len_temp);
            break;
        }
        else
        {
            advIdx += nuggetLen;
        }
    }

    return(localNameStr);
}

/**
 * Used to check if both buttons are held on reset..
 *
 * If both buttons are held, invalidate this image and reset, thus
 * causing the BIM to jump to the factory image from external flash.
 */
static void MultiSensor_bootManagerCheck(void)
{
  // Check if buttons are held on reset
  // GPIO driver is used because SAIL has no way to poll button and the
  // interrupt would have triggered before SAIL was initialized
  uint8_t revertIoInit = !(GPIO_read(GPIO_BTN1));
  uint8_t eraseIoInit = !(GPIO_read(GPIO_BTN2));

  if (revertIoInit ||  eraseIoInit)
  {
    if (revertIoInit && eraseIoInit)
    {
        Log_warning0("Both buttons were held on reset.");
        Log_warning0("Continue holding for 5 seconds to erase external flash.");
        Log_warning0("Note that this will also remove the \"factory image\".");
    }
    else if (revertIoInit)
    {
        Log_warning0("Left button was held on reset.");
        Log_warning0("Continue holding for 5 seconds to revert to factory image.");
    }

    extern const uint8_t LED_count;
    LED_Handle *pLeds = (LED_Handle *)&io.led;

    // Check to see if buttons are continued to be held
    for (uint32_t i = 0; i < BOOTCHECK_SLEEP_DURATION_MS / BOOTCHECK_SLEEP_INTERVAL_MS; ++i)
    {
        LED_Handle *pCurrentLed = pLeds + (i % LED_count);

        if (io.led.ready)
        {
          LED_setOn(*pCurrentLed, 50);
        }

        Task_sleep(MS_TO_TICKS(BOOTCHECK_SLEEP_INTERVAL_MS));

        if (io.led.ready)
        {
          LED_setOff(*pCurrentLed);
        }

        if (revertIoInit && GPIO_read(GPIO_BTN1))
        {
            revertIoInit = FALSE;
            eraseIoInit = FALSE;
            Log_info0("Left button released");
            Log_info0("Continuing with boot");
            break;
        }
    }

    if (eraseIoInit)
    {
        MultiSensor_eraseExternalFlash();
    }
    else if(revertIoInit)
    {
        MultiSensor_revertToFactoryImage();
    }
  }

  return;
}

/**
 * Utility function that invalidates the current image and resets the device,
 * thus causing a jump to the factory image
 */
static void MultiSensor_revertToFactoryImage(void)
{
    extern const imgHdr_t _imgHdr;

    uint32_t key = HwiP_disable();

    uint8_t invalidCrc = CRC_INVALID;
    // Invalidate  CRC in current image header
    uint32_t retVal = FlashProgram(&invalidCrc,
                                 (uint32_t)&_imgHdr.fixedHdr.crcStat,
                                 sizeof(invalidCrc));
    if (retVal == FAPI_STATUS_SUCCESS)
    {
      Log_info0("CRC Status invalidated. Rebooting into BIM.");
      // Sleep to allow log print
      Task_sleep(MS_TO_TICKS(50));
      SysCtrlSystemReset();
      // We never reach here.
    }
    else
    {
      Log_error1("CRC Invalidate write failed. Status 0x%x", retVal);
      Log_error0("Continuing with on-chip application");
    }

    HwiP_restore(key);
}

/**
 * Utility function that erases external flash content
 */
static void MultiSensor_eraseExternalFlash(void)
{
    NVS_Handle nvsHandle;
    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;

    NVS_init();
    NVS_Params_init(&nvsParams);

    Log_info0("Opening external flash for mass erase");
    nvsHandle = NVS_open(CONFIG_NVSEXTERNAL, &nvsParams);
    if (nvsHandle == NULL)
    {
        Log_error0("Failed to open external flash");
        return;
    }
    NVS_getAttrs(nvsHandle, &regionAttrs);
    Log_info3("ExtFlash regionBase: 0x%x, regionSize: 0x%x, sectorSize: 0x%x",
              (uintptr_t)regionAttrs.regionBase,
              (uintptr_t)regionAttrs.regionSize,
              (uintptr_t)regionAttrs.sectorSize);

    // Do sector by sector erase
    for (uint32_t offset = 0; offset < regionAttrs.regionSize; offset += regionAttrs.sectorSize)
    {
        uint16_t status = NVS_erase(nvsHandle, offset, regionAttrs.sectorSize);

        // Sleep every 16th erase to print log and serve other things like network
        uint32_t curSector = (offset / regionAttrs.sectorSize);
        if ((curSector & 0xf) == 0xf)
        {
            Log_info3("Erase sector %d..%d, status: %d", curSector-0xf, curSector, status);
            Task_sleep(20 * (1000/Clock_tickPeriod));
        }
    }

    // Block until ready with meaningless read.
    uint32_t buf;
    NVS_read(nvsHandle, 0, &buf, sizeof(buf));

    Log_info0("External flash erase complete");
    NVS_close(nvsHandle);
}
