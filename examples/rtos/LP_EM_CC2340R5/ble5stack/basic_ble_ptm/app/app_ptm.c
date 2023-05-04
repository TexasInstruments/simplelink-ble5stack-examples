/******************************************************************************

@file  app_PTM.c

@brief This file contains the application data functionality

Group:
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>



//*****************************************************************************
//! Prototypes
//*****************************************************************************
void PTM_handleNPIRxInterceptEvent(uint8_t *pMsg);
static void PTM_sendToNPI(uint8_t *buf, uint16_t len);
void PTM_CTRLToHostEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************
BLEAppUtil_EventHandler_t ptmHandler =
{
    .handlerType    = BLEAPPUTIL_HCI_CTRL_TO_HOST_TYPE,
    .pEventHandler  = PTM_CTRLToHostEventHandler,
    .eventMask      = BLEAPPUTIL_HCI_EVENT_PACKET |
                      BLEAPPUTIL_HCI_ACL_DATA_PACKET,
};

//*****************************************************************************
//! Functions
//****************************************************************************


//****************************************************************************
/*********************************************************************
 * @fn      PTM_CTRLToHostEventHandler
 *
 * @brief   The purpose of this function is to handle events
 *          that rise from the external host and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void PTM_CTRLToHostEventHandler (uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    hciPacket_t *pBuf = (hciPacket_t *)pMsgData;
    uint16_t len = 0;

      // Determine the packet length
      switch(event)
      {
        case BLEAPPUTIL_HCI_EVENT_PACKET:
          len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
          break;

        case BLEAPPUTIL_HCI_ACL_DATA_PACKET:
          len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
          break;

        default:
          break;
      }
      // Send to Remote Host.
      PTM_sendToNPI(pBuf->pData, len);
    }


/*
* @fn      PTM_peripheral_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void PTM_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // The data is stored as a message, free this first.
 ICall_freeMsg(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free container.
 ICall_free(pMsg);
}

/*********************************************************************
* @fn     PTM_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
void PTM_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}

/*********************************************************************
* @fn      PTM_start
*
* @brief   This function is called after stack initialization,
*          the purpose of this function is to initialize and
*          register the specific events handlers of the PTM module
*
* @return  SUCCESS, errorInfo
*/
bStatus_t PTM_start(void)
{
    bStatus_t status;

    /* Start task for NPI task */
    NPITask_createTask(ICALL_SERVICE_CLASS_BLE);

    status = BLEAppUtil_registerEventHandler(&ptmHandler);

   // Intercept NPI RX events.
   NPITask_registerIncomingRXEventAppCB(PTM_handleNPIRxInterceptEvent, INTERCEPT);

   // Register for Command Status information
   HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) PTM_sendToNPI, NULL, BLEAppUtil_getSelfEntity());

   // Register for Events
   HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, BLEAppUtil_getSelfEntity()));

   // Inform Stack to Initialize PTM
   HCI_EXT_EnablePTMCmd();
   return status;

}
