/******************************************************************************

 @file  rtls_ctrl_api.h

 @brief This file contains the RTLS Control API's

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
 *  @file  rtls_ctrl_api.h
 *  @brief      This file contains the RTLS Control API's
 */

#ifndef RTLS_CTRL_API_H_
#define RTLS_CTRL_API_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*********************************************************************
*  EXTERNAL FUNCTIONS
*/

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * CONSTANTS
 */

// Error codes passed through Assert Handler
#define RTLS_CTRL_ASSERT_CAUSE_OUT_OF_MEMORY          0x01    //!< RTLS Assert Error code
#define RTLS_CTRL_ASSERT_CAUSE_INVALID_PARAM          0x02    //!< RTLS Assert Error code
#define RTLS_CTRL_ASSERT_CAUSE_NULL_POINTER_EXCEPT    0x03    //!< RTLS Assert Error code

// RTLS Control version
#define RTLS_CTRL_REV           0x2

// RTLS Application Command Opcodes
#define RTLS_REQ_ENABLE_SYNC              0x01          //!< RTLS Application Command Opcode
#define RTLS_REQ_CONN                     0x02          //!< RTLS Application Command Opcode
#define RTLS_REQ_SCAN                     0x03          //!< RTLS Application Command Opcode
#define RTLS_REQ_SEND_DATA                0x04          //!< RTLS Application Command Opcode
#define RTLS_REQ_TERMINATE_LINK           0x05          //!< RTLS Application Command Opcode
#define RTLS_REQ_SET_AOA_PARAMS           0x06          //!< RTLS Application Command Opcode
#define RTLS_REQ_AOA_ENABLE               0x07          //!< RTLS Application Command Opcode
#define RTLS_REQ_UPDATE_CONN_INTERVAL     0x08          //!< RTLS Application Command Opcode
#define RTLS_REQ_GET_ACTIVE_CONN_INFO     0x09          //!< RTLS Application Command Opcode
#define RTLS_REQ_SET_CREATE_SYNC_PARAMS   0x0A          //!< RTLS Application Command Opcode
#define RTLS_REQ_CREATE_SYNC_CANCEL       0x0B          //!< RTLS Application Command Opcode
#define RTLS_REQ_TERMINATE_SYNC           0x0C          //!< RTLS Application Command Opcode
#define RTLS_REQ_PERIODIC_RECEIVE_ENABLE  0x0D          //!< RTLS Application Command Opcode
#define RTLS_REQ_ADD_DEVICE_ADV_LIST      0x0E          //!< RTLS Application Command Opcode
#define RTLS_REQ_REMOVE_DEVICE_ADV_LIST   0x0F          //!< RTLS Application Command Opcode
#define RTLS_REQ_READ_ADV_LIST_SIZE       0x10          //!< RTLS Application Command Opcode
#define RTLS_REQ_CLEAR_ADV_LIST           0x11          //!< RTLS Application Command Opcode
#define RTLS_REQ_CL_AOA_ENABLE            0x12          //!< RTLS Application Command Opcode

#define RTLS_REQ_BLE_LOG_STRINGS_MAX 0x9
extern char *rtlsReq_BleLogStrings[];

// BADDR Flash Address Offset in CCA (i.e. flash programmer BLE address)
#if !defined(CC26X2) && !defined(CC13X2) && !defined(CC13X2P)
#define BADDR_PAGE_OFFSET                           0xFD0     // in CCFG (CCA); LSB..MSB
#else //Agama CC26X2 || CC13X2 || CC13X2P
#define BADDR_PAGE_OFFSET                           0x1FD0    // in CCFG (CCA); LSB..MSB
#endif ////Agama CC26X2 || CC13X2 || CC13X2P
// BADDR Address Offset in FCFG1 (i.e. permanent BLE address)
#define BDADDR_OFFSET                               0x2E8     // in FCFG; LSB..MSB
#define CHIP_ID_SIZE  6                                       //!< Chip Identifier Size

#define BDADDR_VALID( bdAddr )                                                 \
  ( !(                                                                         \
       ((bdAddr)[0] == 0xFF) &&                                                \
       ((bdAddr)[1] == 0xFF) &&                                                \
       ((bdAddr)[2] == 0xFF) &&                                                \
       ((bdAddr)[3] == 0xFF) &&                                                \
       ((bdAddr)[4] == 0xFF) &&                                                \
       ((bdAddr)[5] == 0xFF)                                                   \
     )                                                                         \
  )

#define RTLS_FALSE    0        //!< FALSE
#define RTLS_TRUE     1        //!< TRUE

// Connection Handle defines
#define RTLS_CONNHANDLE_ALL             0xFFFD // All connection handles
#define RTLS_CONNHANDLE_INVALID         0xFFFF // Invalid connection handle, used for no connection handle

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/// @brief Application callback function
typedef void (*pfnRtlsAppCb)(uint8_t *cmd);

/// @brief Enumeration for RTLS Status
typedef enum
{
  RTLS_SUCCESS,
  RTLS_FAIL,
  RTLS_LINK_ESTAB_FAIL,
  RTLS_LINK_TERMINATED,
  RTLS_OUT_OF_MEMORY,
  RTLS_CONFIG_NOT_SUPPORTED,
  RTLS_ILLEGAL_CMD
} rtlsStatus_e;

/// @brief Enumeration for RTLS Capabilities
typedef enum
{
  RTLS_CAP_NOT_INITIALIZED         = 0x00000000,
  RTLS_CAP_CM                      = 0x00000001,
  RTLS_CAP_AOA_TX                  = 0x00000002,
  RTLS_CAP_AOA_RX                  = 0x00000004,
  RTLS_CAP_RESERVED1               = 0x00000008,
  RTLS_CAP_CL_AOA                  = 0x00000010,
  RTLS_CAP_RESERVED3               = 0x00000020,
  RTLS_CAP_RTLS_RESPONDER          = 0x00000040,
  RTLS_CAP_RTLS_COORDINATOR        = 0x00000080,
  RTLS_CAP_RTLS_PASSIVE            = 0x00000100,
  RTLS_CAP_RTLS_CONNECTION_MONITOR = 0x00000200
} rtlsCapabilities_e;

/// @brief RTLS Configuration structure
typedef struct
{
  rtlsCapabilities_e  rtlsCapab;                   //!< RTLS Capabilities
  uint8_t             identifier[CHIP_ID_SIZE];    //!< Chip Identifier
  uint16_t            revNum;                      //!< Revision
  uint8_t             devId;                       //!< Device ID
  uint8_t             maxNumConns;                 //!< Maximum number of connections to support
  pfnRtlsAppCb        rtlsAppCb;                   //!< Application callback function
} rtlsConfiguration_t;

/// @brief RTLS Control request
typedef struct
{
  uint8_t reqOp;           //!< Request Opcode
  uint8_t *pData;          //!< Pointer to request
} rtlsCtrlReq_t;

/// @brief RTLS Packet - Generic structure that will be translated to a specific command
typedef struct
{
  uint16_t connHandle;     //!< Connection handle
  uint8_t  cmdOp;          //!< RTLS Application Command Opcode
  uint16_t payloadLen;     //!< Length
  uint8_t  pPayload[];     //!< Payload
} rtlsPacket_t;

/// @brief RTLS Enable Sync - This will tell the RTLS Application to start notifying RTLS Control about a specific connection
typedef struct
{
  uint16_t connHandle;
  uint8_t  enable;
} rtlsEnableSync_t;

/// @brief RTLS update connection interval request
typedef struct
{
  uint16_t connHandle;        //!< Connection handle
  uint16_t connInterval;      //!< Connection interval
} rtlsUpdateConnIntReq_t;

/// @brief RTLS terminate link request
typedef struct
{
  uint16_t connHandle;        //!< Connection handle
} rtlsTerminateLinkReq_t;

/// @brief RTLS Get Active Connection Info - get the connection information for a specific handle
typedef struct
{
  uint16_t connHandle;
} rtlsGetActiveConnInfo_t;

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief RTLSCtrl_open
 *
 * Used to create and initialize the RTLS Control task
 *
 * @param rtlsConfig - pointer to the RTLS Application callbacks and configuration
 */
void RTLSCtrl_open(rtlsConfiguration_t *rtlsConfig);

/**
 * @brief RTLSCtrl_scanResultEvt
 *
 * This function notifies the RTLS Control that a connection has been formed
 *
 * @param scanResult - Pointer to scan results array
 * @param size - Size of scanResults array
 * @param status - scanResults status
 *
 */
void RTLSCtrl_scanResultEvt(rtlsStatus_e status, uint8_t *scanResult, uint8_t size);

/**
 * @brief RTLSCtrl_connResultEvt
 *
 * This function notifies the RTLS Control that a connection has been formed
 *
 * @param connHandle - Connection handle
 * @param status - Whether the connection was formed successfully
 */
void RTLSCtrl_connResultEvt(uint16_t connHandle, uint8_t status);

/**
 * @brief RTLSCtrl_connInfoEvt
 *
 * This function sends connection information and parameters to RTLS Control
 *
 * @param connInfo - Connection information
 * @param connInfoLen - Connection Information length (needed to send the packet over NPI)
 */
void RTLSCtrl_connInfoEvt(uint8_t *connInfo, uint16_t connInfoLen);

/**
 * @brief RTLSCtrl_rtlsPacketEvt
 *
 * This function is used by the RTLS Application to notify RTLS Control that a RTLS packet has been received
 *
 * @param pPkt - The RTLS packet to parse
 */
void RTLSCtrl_rtlsPacketEvt(uint8_t *pPkt);

/**
 * @brief RTLSCtrl_syncNotifyEvt
 *
 * Synchronization function provided to the Application by RTLS Control
 * This is the function that the application will be calling when a sync event is in place
 * e.g BLE Connection Event
 *
 * @param connHandle - connection handle
 * @param timeToNextEvent - the time until the next sync event
 * @param rssi - rssi measurement against the RTLS Responder
 * @param channel - the channel on which the syncEvent was received
 * @param status
 */
void RTLSCtrl_syncNotifyEvt(uint16_t connHandle, rtlsStatus_e status, uint32_t timeToNextEvent, int8_t rssi, uint8_t channel);

/**
 * @brief RTLSCtrl_dataSentEvt
 *
 * This function is used by the RTLS Application to report the status of a sent packet
 * This is used to let RTLS Control know whether RTLS data has been successfully transmitted
 *
 * @param status - SUCCESS or FAILURE for sending data to our peer
 * @param connHandle - Connection handle
 */
void RTLSCtrl_dataSentEvt(uint16_t connHandle, uint8_t status);

/**
 * @brief RTLSCtrl_sendDebugEvt
 *
 * Send debug info
 *
 * @param debug_string - 32 bytes debug string
 * @param debug_value - 32 bits debug value
 */
void RTLSCtrl_sendDebugEvt(uint8_t *debug_string, uint32_t debug_value);

/**
 * @brief RTLSCtrl_sendPeriodicListSize
 *
 * Send periodic advertisers list size
 *
 * @param pEvt - Read list size structure
 */
void RTLSCtrl_sendPeriodicListSize(uint8_t *pEvt);

/**
 * @brief RTLSCtrl_processSyncEstEvt
 *
 * Process sync establish event
 *
 * @param opcode - Command opcode
 * @param status - Periodic advertising sync HCI status
 * @param syncHandle - Handle identifying the periodic advertising train
 * @param advSid - Value of the Advertising SID
 * @param advAddrType - Advertiser address type
 * @param advAddress - Advertiser address
 * @param advPhy - Advertiser PHY
 * @param periodicAdvInt - Periodic advertising interval
 * @param advClockAccuracy - Accuracy of the periodic advertiser's clock
 */
void RTLSCtrl_processSyncEstEvt(uint8_t opcode,
                                uint8_t  status,
                                uint16_t syncHandle,
                                uint8_t  advSid,
                                uint8_t  advAddrType,
                                uint8_t  advAddress[6],
                                uint8_t  advPhy,
                                uint16_t periodicAdvInt,
                                uint8_t  advClockAccuracy);

/**
 * RTLSCtrl_processSyncLostEvt
 *
 * Process sync lost event
 *
 * @param opcode - Command opcode
 * @param syncHandle - Handle identifying the periodic advertising train
 */
void RTLSCtrl_processSyncLostEvt(uint8_t opcode,
                                 uint16_t syncHandle);

#ifdef RTLS_CTE
/**
 * @brief RTLSCtrl_aoaResultEvt
 *
 * RTLS Control I/Q samples processing function
 * Results will be output to RTLS Node Manager after processing
 *
 * @param connHandle - connection/sync handle
 * @param rssi - rssi for this CTE
 * @param channel - channel this CTE was captured on
 * @param numIqSamples- Number of I/Q samples
 * @param sampleRate - Sampling rate that was used for the run
 * @param sampleSize - Sample Size 1 = 8 bit, 2 = 16 bit
 * @param sampleCtrl - Sampling control flags
 * @param numAnt - Number of Antennas that were used for the run
 * @param pIQ - Pointer to IQ samples
 */
void RTLSCtrl_aoaResultEvt(uint16_t handle, int8_t rssi, uint8_t channel, uint16_t numIqSamples, uint8_t sampleRate, uint8_t sampleSize, uint8_t sampleCtrl, uint8_t slotDuration, uint8_t numAnt, int8_t *pIQ);

/**
 * RTLSCtrl_processClAoaEnableEvt
 *
 * Process sampling enable command complete event
 *
 * @param opcode - Command opcode
 * @param status - Command status
 * @param syncHandle - Handle identifying the periodic advertising train
 */
void RTLSCtrl_processClAoaEnableEvt(uint16_t opcode,
                                    uint8_t status,
                                    uint16_t syncHandle);
#endif /* RTLS_CTE*/

/**
 * RTLSCtrl_processTerminateSyncEvt
 *
 * Process terminate sync command complete event
 *
 * @param None
 */
void RTLSCtrl_processTerminateSyncEvt( void );

/**
 * RTLSCTRL_processPeriodicAdvReport
 *
 * Process prtiodic advertising report event
 *
 * @param opcode - Command opcode
 * @param syncHandle - Handle identifying the periodic advertising train
 * @param txPower - Tx Power information
 * @param rssi - RSSI value for the received packet
 * @param cteType - Constant Tone Extension type
 * @param dataStatus - Data status
 * @param dataLen - Length of the Data field
 * @param pData - Data received from a Periodic Advertising packet
 */
void RTLSCTRL_processPeriodicAdvReport( uint8_t opcode,
                                        uint16_t syncHandle,
                                        int8_t   txPower,
                                        int8_t   rssi,
                                        uint8_t  cteType,
                                        uint8_t  dataStatus,
                                        uint8_t  dataLen,
                                        uint8_t  *pData);

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* RTLS_CTRL_API_H_ */

/** @} End RTLS_CTRL */
