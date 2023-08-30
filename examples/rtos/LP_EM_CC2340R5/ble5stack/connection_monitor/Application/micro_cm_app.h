/**************************************************************************************************
  Filename:       micro_cm_app.h

  Description:    This file contains the Micro Eddystone Beacon Connection Monitorsample
                  sample application definitions and prototypes.

* Copyright (c) 2017, Texas Instruments Incorporated
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* *  Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* *  Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* *  Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#ifndef MICROCMAPP_H
#define MICROCMAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "hal_types.h"
#include <ti_drivers_config.h>

#include <mqueue.h>
#include <pthread.h>
#include "util.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// RTOS Event to queue stack events
#define UBT_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// CM Event to queue application events
#define UCA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define UCA_ALL_EVENTS                        (UCA_QUEUE_EVT)

#define MICRO_CM_APP_USTACK_EVT               0x1
#define MICRO_CM_APP_RTLS_CTRL_EVT            0x2
#define MICRO_CM_APP_CM_EVT                   0x3

#define UBT_QUEUE_SIZE                        16
#define UCA_QUEUE_SIZE                        16

// Stack event
typedef struct
{
  uint16 event;
  uint8 data;
} ubtEvt_t;

// Connection monitor event
typedef struct
{
  uint16_t event;     // Event Id
  uint8_t *pData;      // Pointer to the data
} microCmAppEvt_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
void MicroCmApp_init(void);
void MicroCmApp_stack_init(void);
void MicroCmApp_enqueueAppMsg(uint16_t eventId, uint8_t *pMsg);
void MicroCmApp_rtlsCtrlMsgCb(uint8_t *pCmd);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MICROCMAPP_H */
