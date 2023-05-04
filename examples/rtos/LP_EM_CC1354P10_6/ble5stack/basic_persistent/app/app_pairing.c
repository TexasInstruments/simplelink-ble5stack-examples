/******************************************************************************

@file  app_pairing.c

@brief This file contains the application pairing role functionality

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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void Pairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Pairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

BLEAppUtil_EventHandler_t pairingPasscodeHandler =
{
    .handlerType    = BLEAPPUTIL_PASSCODE_TYPE,
    .pEventHandler  = Pairing_passcodeHandler
};

BLEAppUtil_EventHandler_t PairingPairStateHandler =
{
    .handlerType    = BLEAPPUTIL_PAIR_STATE_TYPE,
    .pEventHandler  = Pairing_pairStateHandler,
    .eventMask      = BLEAPPUTIL_PAIRING_STATE_STARTED |
                      BLEAPPUTIL_PAIRING_STATE_COMPLETE |
                      BLEAPPUTIL_PAIRING_STATE_ENCRYPTED |
                      BLEAPPUTIL_PAIRING_STATE_BOND_SAVED,
};
//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Pairing_passcodeHandler
 *
 * @brief   The purpose of this function is to handle passcode data
 *          that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Pairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_PasscodeData_t *pData = (BLEAppUtil_PasscodeData_t *)pMsgData;

    // Send passcode response
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      Pairing_pairStateHandler
 *
 * @brief   The purpose of this function is to handle pairing state
 *          events that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Pairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_PAIRING_STATE_STARTED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_STARTED: connectionHandle = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle); dispIndex++;

            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_STARTED: status = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->status); dispIndex++;

            break;
        }
        case BLEAPPUTIL_PAIRING_STATE_COMPLETE:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_COMPLETE: connectionHandle = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle); dispIndex++;

            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_COMPLETE: status = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->status); dispIndex++;
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_ENCRYPTED: connectionHandle = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle); dispIndex++;

            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_ENCRYPTED: status = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->status); dispIndex++;
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_BOND_SAVED: connectionHandle = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle); dispIndex++;

            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    PAIRING_STATE_BOND_SAVED: status = %d",
                           dispIndex,
                           ((BLEAppUtil_PairStateData_t *)pMsgData)->status); dispIndex++;
            break;
        }

        default:
        {
            break;
        }
    }

}

/*********************************************************************
 * @fn      Pairing_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the pairing
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Pairing_start()
{
    bStatus_t status;

    Display_printf(dispHandle, dispIndex, 0,
                   "#%5d    Pairing_start: Register Handlers",
                   dispIndex); dispIndex++;

    // Register the handlers
    status = BLEAppUtil_registerEventHandler(&pairingPasscodeHandler);
    status = BLEAppUtil_registerEventHandler(&PairingPairStateHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    return SUCCESS;
}

#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
