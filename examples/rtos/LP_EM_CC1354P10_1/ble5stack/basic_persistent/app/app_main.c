/******************************************************************************

@file  app_main.c

@brief This file contains the application main functionality

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
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/profiles/oad/oad_profile.h>


//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

// Parameters that should be given as input to the BLEAppUtil_init function
BLEAppUtil_GeneralParams_t appMainParams =
{
    .taskPriority = 1,
    .taskStackSize = 1024,
    .profileRole = (BLEAppUtil_Profile_Roles_e)HOST_CONFIG,
    .addressMode = DEFAULT_ADDRESS_MODE,
    .deviceNameAtt = attDeviceName,
};

BLEAppUtil_PeriCentParams_t appMainPeriCentParams =
{
 .connParamUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION,
 .gapBondParams = &gapBondParams
};

// Display Interface
uint8 dispIndex = 0;
Display_Handle dispHandle = NULL;

//*****************************************************************************
//! Functions
//*****************************************************************************
extern bStatus_t Peripheral_start(void);
extern bStatus_t Pairing_start(void);

/*********************************************************************
 * @fn      criticalErrorHandler
 *
 * @brief   Application task entry point
 *
 * @return  none
 */
void criticalErrorHandler(int32 errorCode , void* pInfo)
{
//    trace();
//
//#ifdef DEBUG_ERR_HANDLE
//
//    while (1);
//#else
//    SystemReset();
//#endif

}

/*********************************************************************
 * @fn      App_StackInitDone
 *
 * @brief   This function will be called when the BLE stack init is
 *          done.
 *          It should call the applications modules start functions.
 *
 * @return  none
 */
void App_StackInitDoneHandler(gapDeviceInitDoneEvent_t *deviceInitDoneData)
{
    bStatus_t status = SUCCESS;

    status = Peripheral_start();

    status = Pairing_start();

    status = OADProfile_start(NULL);

    if(status != SUCCESS)
    {
        // TODO: Call Error Handler for each call
    }
}

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Application main function
 *
 * @return  none
 */
void appMain(void)
{
    // Call the BLEAppUtil module init function
    BLEAppUtil_init(&criticalErrorHandler, &App_StackInitDoneHandler,
                    &appMainParams, &appMainPeriCentParams);
}
