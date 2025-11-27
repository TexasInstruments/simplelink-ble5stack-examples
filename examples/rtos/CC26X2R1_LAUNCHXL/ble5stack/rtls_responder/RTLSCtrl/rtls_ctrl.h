/******************************************************************************

 @file  rtls_ctrl.h

 @brief This file contains internal defines and macros used by rtls_ctrl.c

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2018-2025, Texas Instruments Incorporated
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

/**
 *  @defgroup RTLS_CTRL RTLS_CTRL
 *  @brief This module implements Real Time Localization System (RTLS) Control module
 *
 *  @{
 *  @file  rtls_ctrl.h
 *  @brief      RTLS Control module interface
 */

#ifndef RTLS_CTRL_H_
#define RTLS_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#ifdef USE_ICALL
#include "icall.h"
#endif

// for BLE_LOG
#include "bcomdef.h"
#if defined(RTLS_PASSIVE) || defined(RTLS_CONNECTION_MONITOR)
#include "uble.h"
#else
#include "osal.h"
#endif

#include <mqueue.h>
#include <pthread.h>

#ifdef USE_RCL
#include <drivers/dpl/HwiP.h>
#define Hwi_disable()  HwiP_disable()
#define Hwi_restore(a) HwiP_restore(a)
#include <drivers/dpl/SwiP.h>
#define Swi_disable()  SwiP_disable()
#define Swi_restore(a) SwiP_restore(a)
#include <drivers/dpl/TaskP.h>
#else
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#endif

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// RTLS Task configuration
#define RTLS_CTRL_TASK_PRIORITY   2       //!< RTLS Task configuration variable
#define RTLS_CTRL_TASK_STACK_SIZE 1024     //!< RTLS Task configuration variable

#define RTLS_QUEUE_EVT            UTIL_QUEUE_EVENT_ID   //!< Event_Id_30

#define RTLS_CTRL_ALL_EVENTS      (RTLS_QUEUE_EVT)      //!< RTLS Task configuration
#define RTLS_QUEUE_SIZE           16

#define RTLS_CMD_IDENTIFY                  0x00          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED                  0x01          //!< RTLS Node Manager command
#define RTLS_CMD_CONN_PARAMS               0x02          //!< RTLS Node Manager command
#define RTLS_CMD_CONNECT                   0x03          //!< RTLS Node Manager command
#define RTLS_CMD_SCAN                      0x04          //!< RTLS Node Manager command
#define RTLS_CMD_SCAN_STOP                 0x05          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED1                 0x06          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED2                 0x07          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED3                 0x08          //!< RTLS Node Manager command
#define RTLS_CMD_CREATE_SYNC               0x09          //!< RTLS Node Manager command
#define RTLS_CMD_CREATE_SYNC_CANCEL        0x0A          //!< RTLS Node Manager command
#define RTLS_CMD_TERMINATE_SYNC            0x0B          //!< RTLS Node Manager command
#define RTLS_CMD_PERIODIC_RECEIVE_ENABLE   0x0C          //!< RTLS Node Manager command
#define RTLS_CMD_ADD_DEVICE_ADV_LIST       0x0D          //!< RTLS Node Manager command
#define RTLS_CMD_REMOVE_DEVICE_ADV_LIST    0x0E          //!< RTLS Node Manager command
#define RTLS_CMD_READ_ADV_LIST_SIZE        0x0F          //!< RTLS Node Manager command
#define RTLS_CMD_CLEAR_ADV_LIST            0x10          //!< RTLS Node Manager command
#define RTLS_CMD_AOA_SET_PARAMS            0x13          //!< RTLS Node Manager command
#define RTLS_CMD_AOA_ENABLE                0x14          //!< RTLS Node Manager command
#define RTLS_CMD_CL_AOA_ENABLE             0x15          //!< RTLS Node Manager Command
#define RTLS_CMD_RESET_DEVICE              0x20          //!< RTLS Node Manager command
#define RTLS_CMD_TERMINATE_LINK            0x22          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED4                 0x23          //!< RTLS Node Manager command
#define RTLS_CMD_AOA_RESULT_RAW            0x24          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED5                 0x25          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED6                 0x26          //!< RTLS Node Manager command
#define RTLS_CMD_CONN_INFO                 0x27          //!< RTLS Node Manager command
#define RTLS_CMD_SET_RTLS_PARAM            0x28          //!< RTLS Node Manager command
#define RTLS_CMD_GET_RTLS_PARAM            0x29          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED7                 0x30          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED8                 0x31          //!< RTLS Node Manager command
#define RTLS_CMD_GET_ACTIVE_CONN_INFO      0x32          //!< RTLS Node Manager command
#define RTLS_CMD_CL_AOA_RESULT_RAW         0x33          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED9                 0x34          //!< RTLS Node Manager command
#define RTLS_CMD_RESERVED10                0x35          //!< RTLS Node Manager command
#define RTLS_CMD_HEAP_SIZE                 0x36          //!< RTLS Node Manager command

#define RTLS_CMD_BLE_LOG_STRINGS_MAX 0x32
extern char *rtlsCmd_BleLogStrings[];

/// Maximum number of buffers for CL AoA
extern uint8_t maxNumClCteBufs;

/// Maximum number of buffers for CL AoA
extern uint8_t maxNumClCteBufs;

// RTLS async event
#define RTLS_EVT_ASSERT                   0x80          //!< RTLS async event
#define RTLS_EVT_ERROR                    0x81          //!< RTLS async event
#define RTLS_EVT_DEBUG                    0x82          //!< RTLS async event
#define RTLS_EVT_CONN_INFO                0x83          //!< RTLS async event
#define RTLS_EVT_SYNC_EST                 0x84          //!< RTLS async event
#define RTLS_EVT_SYNC_LOST                0x85          //!< RTLS async event
#define RTLS_EVT_PERIODIC_ADV_RPT         0x86          //!< RTLS async event
#define RTLS_EVT_TERMINATE_SYNC           0x87          //!< RTLS async event
#define RTLS_EVT_CL_AOA_ENABLE            0x88          //!< RTLS async event

// RTLS param types for RTLS_CMD_SET_RTLS_PARAM command
#define RTLS_PARAM_CONNECTION_INTERVAL    0x01          //!< RTLS Param type RTLS_CMD_SET_RTLS_PARAM command
#define RTLS_PARAM_2                      0x02          //!< RTLS Param type RTLS_CMD_SET_RTLS_PARAM command
#define RTLS_PARAM_3                      0x03          //!< RTLS Param type RTLS_CMD_SET_RTLS_PARAM command

/*********************************************************************
 * MACROS
 */
#ifdef USE_ICALL
/// @brief RTLSUTIL_MALLOC memory allocation with icall
#define RTLSUTIL_MALLOC(pAlloc, size) {                               \
                                        pAlloc = ICall_malloc(size);  \
                                      }
/// @brief RTLSUTIL_FREE memory free with icall
#define RTLSUTIL_FREE(pMsg)           {                           \
                                        ICall_free(pMsg);         \
                                        pMsg = NULL;              \
                                      }
#else
/// @brief RTLSUTIL_MALLOC memory allocation without icall
#define RTLSUTIL_MALLOC(pAlloc, size) {                           \
                                        volatile uint32_t keyHwi; \
                                        volatile uint32_t keySwi; \
                                        keyHwi = Hwi_disable();   \
                                        keySwi = Swi_disable();   \
                                        pAlloc = malloc(size);    \
                                        Swi_restore(keySwi);      \
                                        Hwi_restore(keyHwi);      \
                                      }
/// @brief RTLSUTIL_FREE memory free without icall
#define RTLSUTIL_FREE(pFree)          {                           \
                                        volatile uint32_t keyHwi; \
                                        volatile uint32_t keySwi; \
                                        keyHwi = Hwi_disable();   \
                                        keySwi = Swi_disable();   \
                                        free(pFree);              \
                                        pFree = NULL;             \
                                        Swi_restore(keySwi);      \
                                        Hwi_restore(keyHwi);      \
                                      }
#endif
/*********************************************************************
 * TYPEDEFS
 */
#ifdef USE_ICALL
typedef struct
{
  uint32_t totalHeap;
  uint32_t freeHeap;
} heapStat_t;
#endif

/*********************************************************************
 * API FUNCTIONS
 */

/**
* @brief   This function will allocate memory, if we were unable to allocate
*          we will report to RTLS Host
*
* @param   sz - size (Allocated pointer has to be cast)
*
* @return  none
*/
void* RTLSCtrl_malloc(uint32_t sz);

// -----------------------------------------------------------------------------
//! \brief      Create a POSIX task.
//              In case the stackaddr is not provided, allocate it on the heap.
//!
//! \param    newthread     threadId
//! \param    startroutine  Pointer to the task entry function
//! \param    priority
//! \param    stackaddr
//! \param    stacksize
//!
//! \return   void
// -----------------------------------------------------------------------------
int RTLSCtrl_createPTask(pthread_t *newthread, void *(*startroutine)(void *), int priority, void *stackaddr, size_t stacksize);

// -----------------------------------------------------------------------------
//! \brief      Create a POSIX queue.
//!
//! \param    queueHandle     queue handle
//! \param    mq_name         name
//! \param    mq_size         number of elements for the queue
//! \param    mq_msgsize      size of queue element
//! \param    mq_flags        flags
//!
//! \return   void
// -----------------------------------------------------------------------------
int RTLSCtrl_createPQueue(mqd_t *queueHandle, char *mq_name, uint32_t mq_size, uint32_t mq_msgsize, uint32_t mq_flags);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RTLS_CTRL_H_ */

/** @} End RTLS_CTRL */
