/******************************************************************************

 @file  sail_sysconfig_workaround.h

 @brief This file contains the profile implementation for the light sensor
        and light service.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************

 Copyright (c) 2015-2018, Texas Instruments Incorporated
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
 Release Name: ble_sdk_2_02_02_25_s
 Release Date: 2018-04-02 18:04:02
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#ifndef SAIL_SYSCFG_H
#define SAIL_SYSCFG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ti_drivers_config.h"

/* Driver Header files */
#include <ti/common/sail/opt3001/opt3001.h>
#include <ti/common/sail/hdc2010/hdc2010.h>

/*********************************************************************
 * Constants
 */

#define BOARD_OPT3001_LIGHT     0
#define OPT3001_COUNT           1

#define BOARD_HDC2010           0
#define HDC2010_COUNT           1

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern const OPT3001_HWAttrs OPT3001_hwAttrs[OPT3001_COUNT];
extern OPT3001_Config OPT3001_config[];

extern const HDC2010_HWAttrs HDC2010_hwAttrs[HDC2010_COUNT];
extern HDC2010_Config HDC2010_config[];

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAIL_SYSCFG_H */
