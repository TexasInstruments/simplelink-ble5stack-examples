/******************************************************************************

@file  app_central.c

@brief This example file demonstrates how to activate the central role with
the help of BLEAppUtil APIs.

Two structures are used for event handling, one for connection events and one
for scanning events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, structures must be provided for scan init parameters,
scan start and connection init.

In the events handler functions, write what actions are done after each event.
In this example, When a peer is found (Advertise report), an RSSI check is
performed, and if the peer is closer than the peer saved as a candidate,
a candidate replacement is performed. At the end of the scan window, an attempt
is made to connect to the candidate, and then a rescan after resetting the
candidate's information.

In the Central_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

Group: WCS, BTS
Target Device: cc13xx_cc26xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <stdarg.h>

#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Central_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
//*****************************************************************************
//! Globals
//*****************************************************************************

// Events handlers struct, contains the handlers and event masks
// of the application central role module
BLEAppUtil_EventHandler_t centralConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = Central_GAPConnEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT,
};

BLEAppUtil_EventHandler_t centralScanHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
    .pEventHandler  = Central_ScanEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED |
                      BLEAPPUTIL_SCAN_DISABLED |
                      BLEAPPUTIL_ADV_REPORT |
                      BLEAPPUTIL_SCAN_WND_ENDED
};

//! Store connection candidate with lowest rssi
BLEAppUtil_connCandidate_t candidate =
{
    .address  = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa },
    .addrType = 0xFF,
    .rssi     = 0xFF
};

BLEAppUtil_ConnectParams_t centralConnParams =
{
    .phys = INIT_PHY_1M,
    .timeout = 0
};

const BLEAppUtil_ScanInit_t centralScanInitParams =
{
    /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
    .primPhy                    = SCAN_PRIM_PHY_1M,

    /*! Opt SCAN_TYPE_ACTIVE | SCAN_TYPE_PASSIVE */
    .scanType                   = SCAN_TYPE_ACTIVE,

    /*! Scan interval shall be greater than or equal to scan window */
    .scanInterval               = 800, /* Units of 625 us */

    /*! Scan window shall be less than or equal to scan interval */
    .scanWindow                 = 800, /* Units of 625 us */

    /*! Select which fields of an advertising report will be stored */
    /*! in the AdvRptList, For mor field see @ref Gap_scanner.h     */
    .advReportFields            = SCAN_ADVRPT_FLD_ADDRESS |
                                  SCAN_ADVRPT_FLD_ADDRTYPE,

    /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
    .scanPhys                   = SCAN_PRIM_PHY_1M,

    /*! Opt SCAN_FLT_POLICY_ALL | SCAN_FLT_POLICY_WL |   */
    /*! SCAN_FLT_POLICY_ALL_RPA | SCAN_FLT_POLICY_WL_RPA */
    .fltPolicy                  = SCAN_FLT_POLICY_ALL,

    /*! For more filter PDU @ref Gap_scanner.h */
    .fltPduType                 = SCAN_FLT_PDU_CONNECTABLE_ONLY |
                                  SCAN_FLT_PDU_COMPLETE_ONLY,

    /*! Opt SCAN_FLT_RSSI_ALL | SCAN_FLT_RSSI_NONE */
    .fltMinRssi                 = SCAN_FLT_RSSI_ALL,

    /*! Opt SCAN_FLT_DISC_NONE | SCAN_FLT_DISC_GENERAL | SCAN_FLT_DISC_LIMITED
     *  | SCAN_FLT_DISC_ALL | SCAN_FLT_DISC_DISABLE */
    .fltDiscMode                = SCAN_FLT_DISC_DISABLE,

    /*! Opt SCAN_FLT_DUP_ENABLE | SCAN_FLT_DUP_DISABLE | SCAN_FLT_DUP_RESET */
    .fltDup                     = SCAN_FLT_DUP_ENABLE,
};

const BLEAppUtil_ScanStart_t centralScanStartParams =
{
    /*! Zero for continuously scanning */
    .scanPeriod     = 0, /* Units of 1.28sec */

    /*! Scan Duration shall be greater than to scan interval,*/
    /*! Zero continuously scanning. */
    .scanDuration   = 0, /* Units of 10ms */

    /*! If non-zero, the list of advertising reports will be */
    /*! generated and come with @ref GAP_EVT_SCAN_DISABLED.  */
    .maxNumReport   = 0
};

const BLEAppUtil_ConnParams_t centralConnInitParams =
{
     /*! Opt INIT_PHY_ALL | INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED */
    .initPhys                   = INIT_PHY_1M,

    .scanInterval          = 16,   /* Units of 0.625ms */
    .scanWindow            = 16,   /* Units of 0.625ms */
    .minConnInterval       = 80,   /* Units of 1.25ms */
    .maxConnInterval       = 80,   /* Units of 1.25ms */
    .connLatency           = 0,
    .supTimeout            = 2000  /* Units of 10ms */
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Central_ScanEventHandler
 *
 * @brief   The purpose of this function is to handle scan events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    bStatus_t status;
    BLEAppUtil_ScanEventData_t *scanMsg = (BLEAppUtil_ScanEventData_t *)pMsgData;
    switch (event)
    {
        /*! This event happens after detecting peer, an event for each peer */
        case BLEAPPUTIL_ADV_REPORT:
        {
            bleStk_GapScan_Evt_AdvRpt_t *pScanRpt = &scanMsg->pBuf->pAdvReport;
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    GAP_EVT_ADV_REPORT: Discover",
                           dispIndex); dispIndex++;

            /*! Search for BLE device with lowest rssi */
            if(pScanRpt->rssi < candidate.rssi)
            {
                Display_printf(dispHandle, dispIndex, 0,
                               "#%5d    GAP_EVT_ADV_REPORT:keep candidate, "
                               "BD address %s, RSSI = %d",
                               dispIndex,
                               BLEAppUtil_convertBdAddr2Str(pScanRpt->addr),
                               pScanRpt->rssi); dispIndex++;

                /*! Store candidate address and address type */
                candidate.addrType = pScanRpt->addrType;
                memcpy(&candidate.address, pScanRpt->addr, B_ADDR_LEN);
                candidate.rssi = pScanRpt->rssi;
            }

            break;
        }

        case BLEAPPUTIL_SCAN_ENABLED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    GAP_EVT_SCAN_ENABLED: scan discovering...",
                           dispIndex); dispIndex++;

            break;
        }

        case BLEAPPUTIL_SCAN_DISABLED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    GAP_EVT_SCAN_DISABLED: scan done",
                           dispIndex); dispIndex++;
            break;
        }

        /*! Scan window has ended. */
        case BLEAPPUTIL_SCAN_WND_ENDED:
        {
            /*! If a candidate was found try to connect, else continue scan */
            if(candidate.rssi != 0xFF)
            {
                /*! Connect to the candidate*/

                centralConnParams.peerAddrType = (GAP_Peer_Addr_Types_t)(candidate.addrType & MASK_ADDRTYPE_ID);
                memcpy(&centralConnParams.pPeerAddress, candidate.address, B_ADDR_LEN);
                status = BLEAppUtil_Connect(&centralConnParams);
                Display_printf(dispHandle, dispIndex, 0,
                               "#%5d    GAP_EVT_SCAN_WND_ENDED: try to connect",
                               dispIndex); dispIndex++;
                if(status == SUCCESS)
                {
                    candidate.rssi = 0xFF;
                }
            }
            else
            {
                status = BLEAppUtil_scanStart(&centralScanStartParams);
                // TODO: Check status error
            }

            break;
        }

        default:
        {
            break;
        }

    }
}

/*********************************************************************
 * @fn      Central_GAPConnEventHandler
 *
 * @brief   The purpose of this function is to handle connection related
 *          events that rise from the GAP and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Central_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            gapEstLinkReqEvent_t *gapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;

            if(gapEstMsg->connRole == BLEAPPUTIL_CENTRAL_ROLE)
            {
                /*! Print the peer address and connection handle number */
                Display_printf(dispHandle, dispIndex, 0,
                               "#%5d    LINK_ESTABLISHED_EVENT: "
                               "Central role Connected to %s, connectionHandle = %d",
                               dispIndex,
                               BLEAppUtil_convertBdAddr2Str(gapEstMsg->devAddr),
                               gapEstMsg->connectionHandle); dispIndex++;

                /*! Print the number of current connections */
                Display_printf(dispHandle, dispIndex, 0,
                               "#%5d    LINK_ESTABLISHED_EVENT: "
                               "Central role Num Conns = %d",
                               dispIndex,
                               linkDB_NumActive()); dispIndex++;

                /*! If we not reached the max connections number, start scanning */
                if(linkDB_NumActive() < linkDB_NumConns())
                {
                    BLEAppUtil_scanStart(&centralScanStartParams);
                }
            }
            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
            gapTerminateLinkEvent_t *gapTermMsg = (gapTerminateLinkEvent_t *)pMsgData;

            /*! Print the connHandle and termination reason */
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    LINK_TERMINATED_EVENT: "
                           "Central role connectionHandle = %d, reason = %d",
                           dispIndex,
                           gapTermMsg->connectionHandle,
                           gapTermMsg->reason); dispIndex++;

            /*! Print the number of current connections */
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    LINK_TERMINATED_EVENT: "
                           "Central role Num Conns = %d",
                           dispIndex,
                           linkDB_NumActive()); dispIndex++;

            BLEAppUtil_scanStart(&centralScanStartParams);
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      Central_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the central
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Central_start()
{
    bStatus_t status;

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Central_start: Register Handlers",
                   dispIndex); dispIndex++;
    // Register the handlers
    status = BLEAppUtil_registerEventHandler(&centralScanHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&centralConnHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Central_start: Init Scan Params",
                   dispIndex); dispIndex++;

    status = BLEAppUtil_scanInit(&centralScanInitParams);
    if(status != SUCCESS)
    {
        return(status);
    }

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Central_start: Set Conn Params",
                   dispIndex); dispIndex++;

    status = BLEAppUtil_SetConnParams(&centralConnInitParams);
    if(status != SUCCESS)
    {
        return(status);
    }

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Central_start: Scan Start",
                   dispIndex); dispIndex++;

    status = BLEAppUtil_scanStart(&centralScanStartParams);
    if(status != SUCCESS)
    {
        return(status);
    }

    return SUCCESS;
}

#endif // ( HOST_CONFIG & ( CENTRAL_CFG ) )
