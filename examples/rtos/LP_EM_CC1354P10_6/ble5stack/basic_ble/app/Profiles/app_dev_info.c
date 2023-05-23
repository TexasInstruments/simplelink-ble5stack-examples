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
#include <ti/bleapp/services/dev_info/dev_info_service.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      DevInfo_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Device Info service.
 *
 * @return  SUCCESS or stack call status
 */

bStatus_t DevInfo_start(void)
{
  bStatus_t status = SUCCESS;
  uint8_t  *devAddr = NULL;
  uint8_t systemId[DEVINFO_SYSTEM_ID_LEN] = {0};

  // Device Information Service
  status = DevInfo_addService();
  if ( status != SUCCESS )
  {
    // Return status value
    return( status );
  }

  devAddr = GAP_GetDevAddress( TRUE );

  memcpy( &systemId[0], &devAddr[0], 3 );   // use 6 bytes of device address for 8 bytes of system ID value
  memset( &systemId[3], 0xFFFE, 2 );        // set middle bytes to 0xFFFE
  memcpy( &systemId[5], &devAddr[3], 3 );   // shift three bytes up

  // Set Device Info Service System ID Parameter
  status = DevInfo_setParameter( DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId );
  if ( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  // Set Device Info Service Manufacturer Name Parameter
  status = DevInfo_setParameter( DEVINFO_MANUFACTURER_NAME, DEVINFO_STR_ATTR_LEN, "Texas Instruments" );

  // Return status value
  return ( status );
}
