/******************************************************************************

@file  app_menu.c

Group: WCS, BTS
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
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include "ti_ble_config.h"

#if !defined(Display_DISABLE_ALL)
//*****************************************************************************
//! Prototypes
//*****************************************************************************
// Scanning callbacks
void Menu_scanningCB(uint8 index);
void Menu_scanStartCB(uint8 index);
void Menu_scanStopCB(uint8 index);
// Connection callbacks
void Menu_connectionCB(uint8 index);
void Menu_connectCB(uint8 index);
void Menu_connectToDeviceCB(uint8 index);
void Menu_workWithCB(uint8 index);
void Menu_selectedDeviceCB(uint8 index);
void Menu_connPhyCB(uint8 index);
void Menu_connPhyChangeCB(uint8 index);
void Menu_paramUpdateCB(uint8 index);
void Menu_disconnectCB(uint8 index);

//*****************************************************************************
//! Globals
//*****************************************************************************
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
// The current connection handle the menu is working with
static uint16 menuCurrentConnHandle;
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
// Scan Menu
const MenuModule_Menu_t scanningMenu[] =
{
 {"Scan", &Menu_scanStartCB, "Scan for devices"},
 {"Stop Scan", &Menu_scanStopCB, "Stop Scanning for devices"}
};

MENU_MODULE_MENU_OBJECT("Scanning Menu", scanningMenu);
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
// Connection Menu
const MenuModule_Menu_t connectionMenu[] =
{
#if ( HOST_CONFIG & ( CENTRAL_CFG ) )
 {"Connect", &Menu_connectCB, "Connect to a device"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )
 {"Work with", &Menu_workWithCB, "Work with a peer device"}
};

MENU_MODULE_MENU_OBJECT("Connection Menu", connectionMenu);

// Work with menu
const MenuModule_Menu_t workWithMenu[] =
{
 {"Change conn phy", &Menu_connPhyCB, "1M, Coded or 2M"},
 {"Param update", &Menu_paramUpdateCB, "Send connection param update req"},
 {"Disconnect", &Menu_disconnectCB, "Disconnect a specific connection"}
};

MENU_MODULE_MENU_OBJECT("Work with Menu", workWithMenu);

// Phy selection menu
const MenuModule_Menu_t connPhyMenu[] =
{
 {"1 Mbps", &Menu_connPhyChangeCB, ""},
 {"2 Mbps", &Menu_connPhyChangeCB, ""},
 {"1 & 2 Mbps", &Menu_connPhyChangeCB, ""},
 {"Coded", &Menu_connPhyChangeCB, ""},
 {"1 & 2 Mbps & Coded", &Menu_connPhyChangeCB, ""},
};

MENU_MODULE_MENU_OBJECT("Set Conn PHY Preference", connPhyMenu);

#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

// Main menu
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
const MenuModule_Menu_t mainMenu[] =
{
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG) )
 {"Scanning", &Menu_scanningCB, "Scan menu"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
 {"Connection", &Menu_connectionCB, "Connection menu"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
};

MENU_MODULE_MENU_OBJECT("Basic BLE Menu", mainMenu);
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )

//*****************************************************************************
//! Functions
//*****************************************************************************

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/*********************************************************************
 * @fn      Menu_scanningCB
 *
 * @brief   A callback that will be called once the Scanning item in
 *          the main menu is selected.
 *          Calls MenuModule_startSubMenu to display the scanning menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanningCB(uint8 index)
{
  MenuModule_startSubMenu(&scanningMenuObject);
}

/*********************************************************************
 * @fn      Menu_scanStartCB
 *
 * @brief   A callback that will be called once the scan item in
 *          the scanningMenu is selected.
 *          Sets the parameters needed for a scan and starts the scan.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanStartCB(uint8 index)
{
    bStatus_t status;
    const BLEAppUtil_ScanStart_t centralScanStartParams =
    {
        /*! Zero for continuously scanning */
        .scanPeriod     = 0, /* Units of 1.28sec */

        /*! Scan Duration shall be greater than to scan interval,*/
        /*! Zero continuously scanning. */
        .scanDuration   = 1000, /* Units of 10ms */

        /*! If non-zero, the list of advertising reports will be */
        /*! generated and come with @ref GAP_EVT_SCAN_DISABLED.  */
        .maxNumReport   = APP_MAX_NUM_OF_ADV_REPORTS
    };

    status = BLEAppUtil_scanStart(&centralScanStartParams);

    // Print the status of the scan
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ScanStart = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}

/*********************************************************************
 * @fn      Menu_scanStopCB
 *
 * @brief   A callback that will be called once the stop scan item in
 *          the scanningMenu is selected.
 *          Calls BLEAppUtil_scanStop and display the returned status.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanStopCB(uint8 index)
{
    bStatus_t status;

    status = BLEAppUtil_scanStop();

    // Print the status of the scan
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ScanStop = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
/*********************************************************************
 * @fn      Menu_connectionCB
 *
 * @brief   A callback that will be called once the connection item in
 *          the main menu is selected.
 *          Calls MenuModule_startSubMenu to display the connection menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectionCB(uint8 index)
{
  MenuModule_startSubMenu(&connectionMenuObject);
}

#if ( HOST_CONFIG & ( CENTRAL_CFG ) )
/*********************************************************************
 * @fn      Menu_connectCB
 *
 * @brief   A callback that will be called once the connect item in
 *          the connectionMenu is selected.
 *          Gets the list of scanned devices and calls
 *          MenuModule_printStringList to display the list of addresses.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectCB(uint8 index)
{
    uint8 i = 0;
    // Create a static list for the devices addresses strings
    static char addressList[APP_MAX_NUM_OF_ADV_REPORTS][BLEAPPUTIL_ADDR_STR_SIZE] = {0};
    // Create a static menu that will contain the addresses
    static MenuModule_Menu_t peerAddr[APP_MAX_NUM_OF_ADV_REPORTS];

    // Get the scan results list
    App_scanResults *menuScanRes;
    uint8 size = Scan_getScanResList(&menuScanRes);

    // If the scan result list is empty, pring a msg
    if(size == 0)
    {
        MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: No devices in scan list");
    }
    // If the list is not empty, copy the addresses and fill up the menu
    else
    {
        for(i = 0; i < size; i++)
        {
            // Convert the addresses to strings
            memcpy(addressList[i], BLEAppUtil_convertBdAddr2Str(menuScanRes[i].address), BLEAPPUTIL_ADDR_STR_SIZE);
            peerAddr[i].itemName = addressList[i];
            peerAddr[i].itemCallback = &Menu_connectToDeviceCB;
            peerAddr[i].itemHelp = "";
        }
        // Create the menu object
        MENU_MODULE_MENU_OBJECT("Addresses List", peerAddr);
        // Display the list
        MenuModule_printStringList(&peerAddrObject, size);
    }
}

/*********************************************************************
 * @fn      Menu_connectToDeviceCB
 *
 * @brief   A callback that will be called once a device address in
 *          the scan results list is selected.
 *          Gets the list of scanned devices and connect to the address
 *          in the provided index.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectToDeviceCB(uint8 index)
{
    bStatus_t status;

    // Get the scan results list
    App_scanResults *menuScanRes;
    uint8 size = Scan_getScanResList(&menuScanRes);

    // Set the connection parameters
    BLEAppUtil_ConnectParams_t connParams =
    {
     .peerAddrType = menuScanRes[index].addressType,
     .phys = INIT_PHY_1M,
     .timeout = 1000
    };

    // Copy the selected address
    memcpy(connParams.pPeerAddress, menuScanRes[index].address, B_ADDR_LEN);
    status = BLEAppUtil_connect(&connParams);

    // Print the status of the connect call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Connect = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the last menu
    MenuModule_goBack();
}
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )

/*********************************************************************
 * @fn      Menu_workWithCB
 *
 * @brief   A callback that will be called once the work with item in
 *          the connectionMenu is selected.
 *          Gets the list of connected devices and calls
 *          MenuModule_printStringList to display the list of addresses.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_workWithCB(uint8 index)
{
    uint8 i;
    uint8 numConns = linkDB_NumActive();

    // If the connection list is empty, pring a msg
    if(numConns == 0)
    {
        MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: No connected devices");
    }
    // If the list is not empty, copy the addresses and fill up the menu
    else
    {
        // Create a static list for the devices addresses strings
        static char connAddrsses[MAX_NUM_BLE_CONNS][BLEAPPUTIL_ADDR_STR_SIZE] = {0};
        // Create a static menu that will contain the addresses
        static MenuModule_Menu_t connAddrList[MAX_NUM_BLE_CONNS];
        // Get the list of connected devices
        App_connInfo * currConnList = Connection_getConnList();
        for(i = 0; i < numConns; i++)
        {
            // Convert the addresses to strings
            memcpy(connAddrsses[i], BLEAppUtil_convertBdAddr2Str(currConnList[i].peerAddress), BLEAPPUTIL_ADDR_STR_SIZE);
            connAddrList[i].itemName = connAddrsses[i];
            connAddrList[i].itemCallback = &Menu_selectedDeviceCB;
            connAddrList[i].itemHelp = "";
        }

        // Create the menu object
        MENU_MODULE_MENU_OBJECT("Connected devices List", connAddrList);
        // Display the list
        MenuModule_printStringList(&connAddrListObject, numConns);
    }
}

/*********************************************************************
 * @fn      Menu_selectedDeviceCB
 *
 * @brief   A callback that will be called once a device address in
 *          the connected devices list is selected.
 *          Go back to the last menu and display the work with menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_selectedDeviceCB(uint8 index)
{
    menuCurrentConnHandle = Connection_getConnhandle(index);
    // Go to the last menu
    MenuModule_goBack();
    // Display the work with menu options
    MenuModule_startSubMenu(&workWithMenuObject);
}

/*********************************************************************
 * @fn      Menu_connPhyCB
 *
 * @brief   A callback that will be called once the Change conn phy item
 *          in the workWithMenu is selected.
 *          Display the connPhy options menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connPhyCB(uint8 index)
{
  MenuModule_startSubMenu(&connPhyMenuObject);
}

/*********************************************************************
 * @fn      Menu_connPhyChangeCB
 *
 * @brief   A callback that will be called once a phy item in the
 *          connPhyMenu is selected.
 *          Calls BLEAppUtil_setConnPhy to set the selected phy and
 *          go back to the work with menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connPhyChangeCB(uint8 index)
{
    bStatus_t status;
    static uint8_t phy[] = {
      HCI_PHY_1_MBPS,
      HCI_PHY_2_MBPS,
      HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
      HCI_PHY_CODED,
      HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED
    };

    BLEAppUtil_ConnPhyParams_t phyParams =
    {
     .connHandle = menuCurrentConnHandle,
     .allPhys = 0,
     .txPhy = phy[index],
     .rxPhy = phy[index],
     .phyOpts = 0
    };

    // Set the connection phy selected in the menu
    status = BLEAppUtil_setConnPhy(&phyParams);

    // Print the status of the set conn phy call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: SetConnPhy = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the "work with" menu
    MenuModule_goBack();
}

/*********************************************************************
 * @fn      Menu_paramUpdateCB
 *
 * @brief   A callback that will be called once the Param update item
 *          in the workWithMenu is selected.
 *          Calls BLEAppUtil_paramUpdateReq to send the parameters
 *          update request.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_paramUpdateCB(uint8 index)
{
    bStatus_t status;
    gapUpdateLinkParamReq_t pParamUpdateReq =
    {
     .connectionHandle = menuCurrentConnHandle,
     .intervalMin = 400,
     .intervalMax = 800,
     .connLatency = 0,
     .connTimeout = 600
    };

    // Send a connection param update request
    status = BLEAppUtil_paramUpdateReq(&pParamUpdateReq);

    // Print the status of the param update call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ParamUpdateReq = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}

/*********************************************************************
 * @fn      Menu_disconnectCB
 *
 * @brief   A callback that will be called once the Disconnect item
 *          in the workWithMenu is selected.
 *          Calls BLEAppUtil_disconnect to disconnect from the
 *          menuCurrentConnHandle.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_disconnectCB(uint8 index)
{
    bStatus_t status;

    // Disconnect from the selected connection
    status = BLEAppUtil_disconnect(menuCurrentConnHandle);

    // Print the status of the set conn phy call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Disconnect = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the "connection" menu
    MenuModule_goBack();
}

#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

#endif // #if !defined(Display_DISABLE_ALL)

/*********************************************************************
 * @fn      Menu_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize the
 *          menu
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Menu_start()
{
  bStatus_t status = SUCCESS;

#if !defined(Display_DISABLE_ALL)
  MenuModule_params_t params = {
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
    .mode = MenuModule_Mode_MENU_WITH_BUTTONS
#else
    .mode = MenuModule_Mode_PRINTS_ONLY
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
  };

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
  status = MenuModule_init(&mainMenuObject, &params);
#else
  status = MenuModule_init(NULL, &params);
  if(status == SUCCESS)
  {

    // Print the application name
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0,
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_CYAN
                      "Basic BLE" MENU_MODULE_COLOR_RESET);
  }
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )

#endif // #if !defined(Display_DISABLE_ALL)
  return status;
}
