/******************************************************************************

@file  app_observer.c

@brief This example file demonstrates how to activate the observer role with
the help of BLEAppUtil APIs.

ObserverScanInitParams structure is used for define scanning event handling
callback function and eventMask is used to specify the events that
will be received and handled.
In addition, structures must be provided for scan init parameters and
scan start.

In the events handler functions, write what actions are done after each event.
In this example, When a peer is found (Advertise report), A message will be
printed after enabling scanning or when advertise report

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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( OBSERVER_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

//*****************************************************************************
//! Local Functions
//*****************************************************************************

void Observer_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

BLEAppUtil_EventHandler_t observerScanHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
    .pEventHandler  = Observer_ScanEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED |
                      BLEAPPUTIL_ADV_REPORT
};

const BLEAppUtil_ScanInit_t observerScanInitParams =
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
    .advReportFields            = SCAN_ADVRPT_FLD_ADDRESS | SCAN_ADVRPT_FLD_ADDRTYPE,

    /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
    .scanPhys                   = SCAN_PRIM_PHY_1M,

    /*! Opt SCAN_FLT_POLICY_ALL | SCAN_FLT_POLICY_WL | SCAN_FLT_POLICY_ALL_RPA
     *  | SCAN_FLT_POLICY_WL_RPA */
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
    .fltDup                     = SCAN_FLT_DUP_ENABLE
};

const BLEAppUtil_ScanStart_t observerScanStartParams =
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

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Observer_ScanEventHandler
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
void Observer_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch (event)
    {
        /*! This event happens after detecting peer, an event for each peer */
        case BLEAPPUTIL_ADV_REPORT:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    GAP_EVT_ADV_REPORT: Discover",
                           dispIndex); dispIndex++;

            break;
        }

        case BLEAPPUTIL_SCAN_ENABLED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    GAP_EVT_SCAN_ENABLED: scan discovering...",
                           dispIndex); dispIndex++;

            break;
        }

        default:
        {
            break;
        }
    }

}

/*********************************************************************
 * @fn      Observer_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the observer
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Observer_start()
{
    bStatus_t status;

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Observer_start: Register Handlers",
                   dispIndex); dispIndex++;

    // Register the handlers
    status = BLEAppUtil_registerEventHandler(&observerScanHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Observer_start: Init Scan Params",
                   dispIndex); dispIndex++;

    status = BLEAppUtil_scanInit(&observerScanInitParams);
    if(status != SUCCESS)
    {
        return(status);
    }

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Observer_start: Scan Start",
                   dispIndex); dispIndex++;

    status = BLEAppUtil_scanStart(&observerScanStartParams);
    if(status != SUCCESS)
    {
        return(status);
    }

    return SUCCESS;
}

#endif // ( HOST_CONFIG & ( OBSERVER_CFG ) )
