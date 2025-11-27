/******************************************************************************

 @file  simple_broadcaster.c

 @brief This file contains the Simple Broadcaster sample application for
        use with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2011-2025, Texas Instruments Incorporated
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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include "util.h"
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include <devinfoservice.h>

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include "ti_ble_config.h"
#include "simple_broadcaster.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task configuration
#define SB_TASK_PRIORITY                     1

#ifndef SB_TASK_STACK_SIZE
#define SB_TASK_STACK_SIZE                   1024
#endif

#define SB_ADV_EVT                           1

// Internal Events for RTOS application
#define SB_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SB_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SB_ALL_EVENTS                        (SB_ICALL_EVT | \
                                               SB_QUEUE_EVT)

// Spin if the expression is not true
#define SIMPLEBROADCASTER_ASSERT(expr) if (!(expr)) HAL_ASSERT_SPINLOCK;
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} sbEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

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
Task_Struct sbTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t sbTaskStack[SB_TASK_STACK_SIZE];

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} sbGapAdvEventData_t;

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void     SimpleBroadcaster_init(void);
static void     SimpleBroadcaster_taskFxn(UArg a0, UArg a1);

static void     SimpleBroadcaster_processStackMsg(ICall_Hdr *pMsg);
static void     SimpleBroadcaster_processGapMessage(gapEventHdr_t *pMsg);
static void     SimpleBroadcaster_processAppMsg(sbEvt_t *pMsg);
static void     SimpleBroadcaster_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static bool     SimpleBroadcaster_processAdvEvent(sbGapAdvEventData_t *pEventData);
static status_t SimpleBroadcaster_enqueueMsg(uint8_t event, void *pData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
/*
static gapRolesCBs_t simpleBroadcaster_BroadcasterCBs =
{
  SimpleBroadcaster_stateChangeCB   // Profile State Change Callbacks
};
*/
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBroadcaster_createTask
 *
 * @brief   Task creation function for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBroadcaster_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbTaskStack;
  taskParams.stackSize = SB_TASK_STACK_SIZE;
  taskParams.priority = SB_TASK_PRIORITY;

  Task_construct(&sbTask, SimpleBroadcaster_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBroadcaster_init
 *
 * @brief   Initialization function for the Simple Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBroadcaster_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Open LCD
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_BROADCASTER, selfEntity, addrMode, &pRandomAddress);

  Display_printf(dispHandle, 0, 0, "BLE Broadcaster");
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processEvent
 *
 * @brief   Application task entry point for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBroadcaster_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBroadcaster_init();

  // Application main loop
  for (;;)
  {
    // Get the ticks since startup
    uint32_t tickStart = Clock_getTicks();

    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SB_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBroadcaster_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SB_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          sbEvt_t *pMsg = (sbEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimpleBroadcaster_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBroadcaster_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBroadcaster_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimpleBroadcaster_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

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

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_printf(dispHandle, 1, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));
        Display_printf(dispHandle, 2, 0, "Initialized");

        if (addrMode > ADDRMODE_RANDOM)
        {
          // Display the RPA.
          Display_printf(dispHandle, 3, 0, "RP Addr: %s",
                         Util_convertBdAddr2Str(GAP_GetDevAddress(FALSE)));
        }

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        #ifndef BEACON_FEATURE
          advParams1.eventProps = GAP_ADV_PROP_SCANNABLE | GAP_ADV_PROP_LEGACY;
        #else
          advParams1.eventProps = GAP_ADV_PROP_LEGACY;
        #endif // !BEACON_FEATURE

        // Create Advertisement set
        status = GapAdv_create(&SimpleBroadcaster_advCallback, &advParams1,
                               &advHandleLegacy);
        SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

        // Load advertising data
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

        // Load scan response data
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

        // Set event mask
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);
        SIMPLEBROADCASTER_ASSERT(status == SUCCESS);

        // Enable legacy advertising
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEBROADCASTER_ASSERT(status == SUCCESS);
      }

      break;
    }

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBroadcaster_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimpleBroadcaster_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  sbEvt_t *pMsg = ICall_malloc(sizeof(sbEvt_t));

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
 * @fn      SimpleBroadcaster_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimpleBroadcaster_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  sbGapAdvEventData_t *pData = ICall_malloc(sizeof(sbGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimpleBroadcaster_enqueueMsg(SB_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBroadcaster_processAppMsg(sbEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case SB_ADV_EVT:
      safeToDealloc = SimpleBroadcaster_processAdvEvent((sbGapAdvEventData_t*)(pMsg->pData));
      break;

    default:
      // Do nothing.
      break;
  }

  if (safeToDealloc)
  {
    // Free message data
    if (pMsg->pData)
    {
      ICall_free(pMsg->pData);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBroadcaster_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static bool SimpleBroadcaster_processAdvEvent(sbGapAdvEventData_t *pEventData)
{
  bool safeToDealloc = TRUE;

  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, 2, 0, "Advertising");
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      safeToDealloc = FALSE;
      break;

    default:
      // Do nothing.
      break;
  }

  // Free event data buffer
  if (safeToDealloc)
  {
    if (pEventData->pBuf != NULL)
    {
      ICall_free(pEventData->pBuf);
    }
  }

  return(safeToDealloc);
}

/*********************************************************************
*********************************************************************/
