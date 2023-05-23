/******************************************************************************

@file  app_dev_info.c

@brief This file contains the device info application functionality

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
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
    switch(msg)
    {
        case OAD_PROFILE_MSG_REVOKE_IMG_HDR:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    App_OADCallback: Revoke image header",
                           dispIndex
                           ); dispIndex++;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_NEW_IMG_IDENDIFY:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    App_OADCallback: New image identify",
                           dispIndex
                           ); dispIndex++;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_START_DOWNLOAD:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    App_OADCallback: Download new image",
                           dispIndex
                           ); dispIndex++;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_FINISH_DOWNLOAD:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    App_OADCallback: Download complete",
                           dispIndex
                           ); dispIndex++;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_RESET_REQ:
        {
            Display_printf(dispHandle, dispIndex, 0,
                           "#%5d    App_OADCallback: Reset device",
                           dispIndex
                           ); dispIndex++;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
    }
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
  Display_printf(dispHandle, dispIndex, 0,
                 "#%5d    App version: %d.%d.%d.%d",
                 dispIndex,
                 img_ver->iv_major,
                 img_ver->iv_minor,
                 img_ver->iv_revision,
                 img_ver->iv_build_num
                 ); dispIndex++;
  return ( status );
}
