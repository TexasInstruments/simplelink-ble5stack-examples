/******************************************************************************

@file  app_PTM.c

@brief This file contains the application data functionality

Group:
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
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

 // Free the NPI message
 NPITask_freeNpiMsg(pMsg);
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
