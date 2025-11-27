/******************************************************************************

 @file  rtls_ble.h

 @brief This file contains the stack specific BLE structures needed for RTLS Control

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
 *  @file  rtls_ble.h
 *  @brief      Stack specific BLE structures needed for RTLS Control
 */

#ifndef RTLS_BLE_H_
#define RTLS_BLE_H_

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

/// @brief BLE Connection information
typedef struct __attribute__((packed))
{
  uint16_t connHandle;        //!< Connection handle
  uint32_t accessAddr;        //!< Return error code if failed to get conn info
  uint8_t connRole;           //!< Connection formed as BLE central or BLE peripheral (4 - Peripheral, 8- Central)
  uint16_t connInterval;      //!< Connection interval time, range (7.5ms, 4s), 625us increments
  uint8_t hopValue;           //!< Hop value for conn alg 1, integer range (5,16), can also be used to tell if using alg 2 by sending special code not in normal hop range i.e. 0xff or 0x00.
  uint16_t cSCA;              //!< Coordinator sleep clock accuracy code mapped based on BLE spec
  uint8_t currChan;           //!< Next data channel
  uint8_t chanMap[5];         //!< Bitmap of used BLE channels
  uint32_t crcInit;           //!< Connection CRC initialization value
  uint8_t addr[6];            //!< BD Addr of the Responder
} bleConnInfo_t;              //!< BLE Connection information

/// @brief BLE Scanning information
typedef struct __attribute__((packed))
{
  uint8_t eventType;          //!< Advertisement Type
  uint8_t addrType;           //!< Address type
  uint8_t addr[6];            //!< Address of the advertisement or SCAN_RSP
  int8_t rssi;                //!< Advertisement or SCAN_RSP RSSI
  uint8_t advSID;             //!< Advertisement SID
  uint16_t periodicAdvInt;    //!< Periodic Advertising interval
  uint8_t dataLen;            //!< Length (in bytes) of the data field (evtData)
  uint8_t pEvtData[];         //!< Data field of advertisement or SCAN_RSP
} bleScanInfo_t;              //!< BLE Scanning information

/// @brief BLE Connection request information
typedef struct __attribute__((packed))
{
  uint8_t addrType;           //!< Address type
  uint8_t addr[6];            //!< Address of the advertisement or SCAN_RSP
  uint16  connInterval;       //!< Connection interval time, range (7.5ms, 4s), 625us increments
 } bleConnReq_t;              //!< BLE Connection request information

// Create Sync parameters
typedef struct __attribute__((packed))
{
  uint8_t  advSID;
  uint8_t  options;
  uint8_t  advAddrType;
  uint8_t  advAddress[6];
  uint16_t skip;
  uint16_t syncTimeout;
  uint8_t  syncCteType;
} rtlsCreateSyncParams_t;

// Periodic Adv Receive Enable parameters
typedef struct __attribute__((packed))
{
  uint16_t syncHandle;
  uint8_t  enable;
} rtlsReceiveEnableParams_t;

/// RTLS terminate sync request
typedef struct
{
  uint16_t syncHandle;        //!< Sync handle
} rtlsTerminateSync_t;

// Periodic Adv - Advertiser information
typedef struct __attribute__((packed))
{
  uint8_t  advAddrType;
  uint8_t  advAddress[6];
  uint8_t  advSID;
} rtlsAdvListDeviceParams_t;

/** @} End RTLS_CTRL_Structs */

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RTLS_BLE_H_ */

/** @} End RTLS_CTRL */
