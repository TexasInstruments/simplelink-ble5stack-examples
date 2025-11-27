/******************************************************************************

 @file  rtls_host.h

 @brief This file contains the host abstraction layer for a RTLS Host
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
 *  @file  rtls_host.h
 *  @brief      This file contains the host abstraction layer for a RTLS Host
 */

#ifndef RTLS_HOST_H_
#define RTLS_HOST_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/** @defgroup RTLS_CTRL_Structs RTLS Control Structures
 * @{
 */

/// @brief Enumeration for Host Message types
typedef enum
{
  HOST_ASYNC_REQ,
  HOST_ASYNC_RSP,
  HOST_SYNC_REQ,
  HOST_SYNC_RSP
} rtlsHostMsgType_e;

// RTLS Host message structure
typedef struct
{
  uint16_t            cmdId;       //!< Command Id
  rtlsHostMsgType_e   cmdType;     //!< Sync/Async
  uint16_t            dataLen;     //!< Length of data pointed by pData
  uint8_t             *pData;      //!< Pointer to the data
} rtlsHostMsg_t;

/// @brief RTLS Host application callback
typedef void (*pfnRtlsCtrlProcessMsgCb)(rtlsHostMsg_t *pMsg);
/** @} End RTLS_CTRL_Structs */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief   This function opens the host interface and registers
 *          the application callback
 *
 * @param   rtlsHostMsgCb - Callback to use when a message arrives
 *
 * @return  none
 */
void RTLSHost_openHostIf(pfnRtlsCtrlProcessMsgCb rtlsHostMsgCb);

/**
 * @brief   This function is an abstraction of sending messages to a RTLS Host
 *
 * @param   cmdId - RTLS Cmd Id
 * @param   cmdType - Async/Sync message
 * @param   pData - Pointer to message data
 * @param   dataLen - Length of data in pData
 *
 * @return  status - 0 = success, 1 = failed
 */
uint8_t RTLSHost_sendMsg(uint8_t cmdId, uint8_t cmdType, uint8_t *pData, uint16_t dataLen);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RTLS_HOST_H_ */

/** @} End RTLS_CTRL */
