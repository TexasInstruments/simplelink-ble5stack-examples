/******************************************************************************

 @file  simple_peripheral_oad_offchip_menu.h

 @brief This file contains macros, type definitions, and function prototypes
        for two-button menu implementation.

 Group: WCS BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2017-2023, Texas Instruments Incorporated
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

#ifndef OFFCHIP_USR_APP_MENU_H
#define OFFCHIP_USR_APP_MENU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "two_btn_menu.h"

// Menu item indices.
#define SP_ITEM_NONE            TBM_ITEM_NONE
#define SP_ITEM_ALL             TBM_ITEM_ALL

// Note: The defines should be updated accordingly if there is any change
//       in the order of the items of the menu objects the items belong to.
#define SP_ITEM_SELECT_CONN     TBM_ITEM(1)  // "Work with"
#define SP_ITEM_SELECT_OAD_DBG  TBM_ITEM(2)  // "OAD Debug"


/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t spMenuMain;

/* Items of (Main) */
/* Action items are defined in simple_peripheral_menu.c */

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t spMenuMain;
extern tbmMenuObj_t spMenuSelectConn;
extern tbmMenuObj_t spMenuPerConn;
extern tbmMenuObj_t spMenuConnPhy;

extern tbmMenuObj_t spMenuOADDebug;
extern tbmMenuObj_t spMenuEraseImg;
extern tbmMenuObj_t spMenuDisplayImgInfo;

void SimplePeripheral_buildMenu(void);

#ifdef __cplusplus
}
#endif

#endif /* OFFCHIP_USR_APP_MENU_H */
