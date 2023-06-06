/******************************************************************************

@file  app_dev_info.c

@brief This file contains the device info application functionality

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
#include "ti/bleapp/profiles/oad/oad_profile.h"
#include "ti/bleapp/util/sw_update/sw_update.h"

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

//*****************************************************************************
//! Functions
//*****************************************************************************

OADProfile_AppCommand_e App_OADCallback(OADProfile_App_Msg_e msg)
{
    OADProfile_AppCommand_e cmd = OAD_PROFILE_PROCEED;
    /*switch(msg)
    {
        case OAD_PROFILE_MSG_REVOKE_IMG_HDR:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Revoke image header");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_NEW_IMG_IDENDIFY:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: New image identify");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_START_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download new image");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_FINISH_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download complete");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_RESET_REQ:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Reset device");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
    }*/
    return (cmd);
}

/*********************************************************************
 * @fn      OAD_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the OAD profile and service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t OAD_start(void)
{
  bStatus_t status = SUCCESS;

  OADProfile_start(&App_OADCallback);

  //APP_HDR_ADDR is the place in the flash memory of APP header, and it is imported from the predefined symbols
  //SwUpdate_GetSWVersion function extract image version struct from given address
  struct image_version * img_ver = (struct image_version *)SwUpdate_GetSWVersion(APP_HDR_ADDR);
  MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "App version: %d.%d.%d.%d",
                    img_ver->iv_major,
                    img_ver->iv_minor,
                    img_ver->iv_revision,
                    img_ver->iv_build_num);
  return ( status );
}
