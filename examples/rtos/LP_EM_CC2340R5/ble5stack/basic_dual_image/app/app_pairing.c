/******************************************************************************

@file  app_pairing.c

@brief This file contains the application pairing role functionality

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

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
            MenuModule_printf(APP_MENU_PAIRING_EVENT, 0, "Pairing Status: Started - "
                              "connectionHandle = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "status = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->status);

            break;
        }
        case BLEAPPUTIL_PAIRING_STATE_COMPLETE:
        {
            MenuModule_printf(APP_MENU_PAIRING_EVENT, 0, "Pairing Status: Complete - "
                              "connectionHandle = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "status = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->status);

            // The pairing is completed, so update the entry in connection list
            // to the ID address instead of the RP address
            linkDBInfo_t linkInfo;
            // Get the list of connected devices
            App_connInfo* connList = Connection_getConnList();
            if (linkDB_GetInfo(((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle, &linkInfo) == SUCCESS)
            {
              // If the peer was using private address, update with ID address
              if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
                   linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
                   !osal_isbufset(linkInfo.addrPriv, 0, B_ADDR_LEN))
              {

                // Get the index of connection list by connHandle
                uint8_t connIdx = Connection_getConnIndex(((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle);

                // Verify that there is a match of connection handle
                if (connIdx != LL_INACTIVE_CONNECTIONS)
                {
                  // Update the connection list with the ID address
                  memcpy(connList[connIdx].peerAddress, linkInfo.addr, B_ADDR_LEN);
                }
              }
            }
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
        {
            MenuModule_printf(APP_MENU_PAIRING_EVENT, 0, "Pairing Status: Encrypted - "
                              "connectionHandle = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "status = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->status);
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
        {
            MenuModule_printf(APP_MENU_PAIRING_EVENT, 0, "Pairing Status: Bond saved - "
                              "connectionHandle = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "status = "MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle,
                              ((BLEAppUtil_PairStateData_t *)pMsgData)->status);
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
    bStatus_t status = SUCCESS;

    // Register the handlers
    status = BLEAppUtil_registerEventHandler(&pairingPasscodeHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&PairingPairStateHandler);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    // Return status value
    return(status);
}

#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
