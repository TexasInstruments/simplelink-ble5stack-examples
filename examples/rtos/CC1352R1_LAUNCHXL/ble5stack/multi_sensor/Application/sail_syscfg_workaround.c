/******************************************************************************

   @file  sail_syscfg_workaround.c

   The SAIL drivers are not currently configured by SysCfg so this file
   contains the SAIL driver structures to be used by the application.

   Group: WCS, BTS
   Target Device: cc13x2_26x2

 ******************************************************************************

 Copyright (c) 2015-2019, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */

#include "ti_drivers_config.h"

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/common/sail/opt3001/opt3001.h>

#include <sail_syscfg_workaround.h>

/*********************************************************************
 * GLOBAL VARIABLES
 */

// OPT3001
OPT3001_Object OPT3001_object[OPT3001_COUNT];

const OPT3001_HWAttrs OPT3001_hwAttrs[OPT3001_COUNT] = {
        {
            .slaveAddress = OPT3001_SA1,
            .gpioIndex = CONFIG_GPIO_OPT_INT,
        },
};

OPT3001_Config OPT3001_config[] = {
    {
        .hwAttrs = &OPT3001_hwAttrs[0],
        .object  = &OPT3001_object[0],
    },
    {NULL, NULL},
};

// HDC2010
HDC2010_Object HDC2010_object[HDC2010_COUNT];

const HDC2010_HWAttrs HDC2010_hwAttrs[HDC2010_COUNT] = {
        {
            .slaveAddress = HDC2010_SA2,
            .gpioIndex = CONFIG_GPIO_HDC_INT,
        },
};

HDC2010_Config HDC2010_config[] = {
    {
        .hwAttrs = &HDC2010_hwAttrs[0],
        .object =  &HDC2010_object[0],
    },
    {NULL, NULL},
};


 /*****************************************************************************/
