/**************************************************************************************************
  Filename:       micro_ble_cm.h

  Description:    This file contains the Connection Monitor sample application
                  definitions and prototypes.

* Copyright (c) 2015, Texas Instruments Incorporated
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

#ifndef MICROBLECM_H
#define MICROBLECM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "uble.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/


/*********************************************************************
 * CONSTANTS
 */
#define CM_SUCCESS                    0
#define CM_FAILED_TO_START            1
#define CM_FAILED_NOT_FOUND           2
#define CM_FAILED_NOT_ACTIVE          3
#define CM_FAILED_OUT_OF_RANGE        4

#define CM_MAX_SESSIONS               UBLE_MAX_MONITOR_HANDLE
#define CM_NUM_BYTES_FOR_CHAN_MAP     5
#define CM_DEVICE_ADDR_LEN            6

#define CM_INVALID_SESSION_ID         0
#define CM_CONN_INTERVAL_MIN          12   // 7.5ms in 625ms
#define CM_CONN_INTERVAL_MAX          6400 // 4s in 625ms

#define CM_CHAN_MASK                  0x3F
#define CM_TIMESTAMP_OFFSET           (-4)
#define CM_RFSTATUS_OFFSET            (-5) // see rfStatus_t for individual bit info
#define CM_RSSI_OFFSET                (-6)

// RSSI
#define CM_RF_RSSI_INVALID            0x81  // reported by RF
#define CM_RF_RSSI_UNDEFINED          0x80  // reported by RF
#define CM_RSSI_NOT_AVAILABLE         0x7F  // report to user

#define CM_BITS_PER_BYTE              8
#define CM_MAX_NUM_DATA_CHAN          37       // 0..36

#define CM_MAX_MISSED_PACKETS         3        // Number of consecutive missed
                                               // packets before giving up

#define CM_SESSION_DATA_RSP           4

#define CM_MAX_COLLISION_COMPRISON    5

// CM Event types
#define CM_PACKET_RECEIVED_EVT        1
#define CM_MONITOR_STATE_CHANGED_EVT  2
#define CM_CONN_EVT_COMPLETE_EVT      3

/*********************************************************************
 * MACROS
 */
#define CM_GET_RSSI_OFFSET()       (0)

/* corrects RSSI if valid, otherwise returns not available */
#define CM_CHECK_LAST_RSSI( rssi )                                             \
          ((rssi) == CM_RF_RSSI_UNDEFINED || (rssi) == CM_RF_RSSI_INVALID)  ?  \
          (int8)CM_RSSI_NOT_AVAILABLE                                       :  \
          ((rssi) - CM_GET_RSSI_OFFSET())

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t   sessionId;                             //! Number 1-255 assigned as they are created identifying each connection monitor session
  uint8_t   timesScanned;                          //! track count of recent events monitored to determine next priority CM session to avoid starvation
  uint8_t   timesMissed;                           //! missed count of recent events monitored
  uint8_t   consecutiveTimesMissed;                //! consecutive missed count of recent events monitored
  uint8_t   missedEvents;                          //! missed count of events since last monitoring session
  uint8_t   numEvents;                             //! how many connection events have passed since the last anchor point.
  uint32_t  accessAddr;                            //! return error code if failed to get conn info
  uint8_t   connRole;                              //! RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
  uint16_t  connInterval;                          //! connection interval time, range 12 to 6400 in  625us increments (7.5ms to 4s)
  uint16_t  scanDuration;                          //! Required scan window to capture minimum of 1 packet from Central and Peripheral up to max possible packet size
  uint8_t   hopValue;                              //! Hop value for conn alg 1, integer range (5,16)
  uint16_t  combSCA;                               //! mSCA + cmSCA
  uint8_t   currentChan;                           //! current unmapped data channel
  uint8_t   lastUnmappedChannel;                   //! last used unmapped channel when received data packet from the central
  uint8_t   nextChan;                              //! next data channel
  uint8_t   numUsedChans;                          //! count of the number of usable data channels
  uint8_t   chanMap[CM_MAX_NUM_DATA_CHAN];         //! current channel map table that is in use for this connection
  uint8_t   rssiCentral;                           //! last Rssi value Central
  uint8_t   rssiPeripheral;                        //! last Rssi value Peripheral
  uint32_t  timeStampCentral;                      //! last timeStamp Central
  uint32_t  timeStampCentral2;                     //! last timeStamp Central
  uint32_t  timeStampPeripheral;                   //! last timeStamp Peripheral
  uint32_t  currentStartTime;                      //! Current anchor point
  uint32_t  nextStartTime;                         //! Record next planned scan anchor point to compare with competing CM sessions
  uint32_t  lastStartTime;                         //! Record last planned scan anchor point to compare with competing CM sessions
  uint32_t  timerDrift;                            //! Clock timer drift
  uint32_t  crcInit;                               //! crcInit value for this connection
  uint16_t  hostConnHandle;                        //! keep connHandle from host requests
  uint8_t   outOfSyncFlag;                         //! flag indicating that the CM session is trying to get in sync
  uint32_t  outOfSyncTimeStamp;                    //! timeStamp for trying to get in sync
  uint8_t   stopRequested;                         //! Indicates if the session needs to be stopped
  uint8_t   isFirstPktMet;                         //! Indicates that first packet met and the first anchor point updated
  bStatus_t indicationStatus;                      //! Indicates if we got the first packet in the RX window as expected or not
} ubCM_ConnInfo_t;

// Structure for the connection monitor status and data
typedef struct
{
  uint32_t lastIndPacketTime;           //! The time of the last packet indication received
  uint32_t lastNewSessionStartTime;     //! The time of the last new session started, used to enhance the calculation of the next channel calculation
  uint8_t cmCentral;                    //! Flag to identify a packet from Central
  uint8_t newSessionActive;             //! Indication of monitoring a new connection
  uint8_t isSessionMonitored;           //! Indication whether there is a session is monitored or not
  uint8_t newSessionId;                 //! The session ID of the new connection
  uint8_t nextSessionId;                //! The session ID of the next monitored connection
  uint8_t failedSessionToMonitor;       //! If not zero, then it holds a session that didn't schedule because there was no enough time to the rcl to start.
} ubCM_Data_t;

typedef struct
{
  uint8_t         numHandles;                       //! number of active handles corresponds to number of instances of ubCM_ConnInfo_t
  ubCM_ConnInfo_t ArrayOfConnInfo[CM_MAX_SESSIONS]; //! pointer to set of connection information for all active connections
} ubCM_GetConnInfoComplete_t;

typedef void (*pfnAppCb)(uint8_t *pCmd);

// CM Event
typedef struct
{
  uint8_t event;
  uint8_t *pData;
} cmEvt_t;

// Monitor Complete Event
typedef struct
{
  uint8_t status;
  uint8_t sessionId;
  uint8_t channel;
} monitorCompleteEvt_t;

// New connection parameters
typedef struct cmNewConn
{
  uint16_t connHandle;        //!< Connection handle
  uint32_t accessAddr;        //!< Return error code if failed to get conn info
  uint8_t connRole;           //!< RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
  uint16_t connInterval;      //!< Connection interval time, range (7.5ms, 4s), 625us increments
  uint8_t hopValue;           //!< Hop value for conn alg 1, integer range (5,16), can also be used to tell if using alg 2 by sending special code not in normal hop range i.e. 0xff or 0x00.
  uint8_t currChan;           //!< Next data channel
  uint8_t chanMap[5];         //!< Bitmap of used BLE channels
  uint32_t crcInit;           //!< Connection CRC initialization value
  uint32_t reqTime;           //!< Time stamp of the new connection request
  struct cmNewConn *next;
} cmNewConn_t;

// Packet received
typedef struct
{
  bStatus_t status;
  uint8_t   centralPacket; // If not, then this is a peripheral packet
  uint8_t   seesionId;
  uint8_t   len;
  uint8_t   *pPayload;
} packetReceivedEvt_t;

// RF Status
typedef struct
{
  uint8_t channel:6; // The channel on which the packet was received
  uint8_t bRxIgnore:1; // 1 if the packet is marked as ignored, 0 otherwise
  uint8_t bCrcErr:1; // 1 if the packet was received with CRC error, 0 otherwise
} rfStatus_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern ubCM_GetConnInfoComplete_t ubCMConnInfo;
extern uint8 gUpdateSessionMissEvents[];
/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      ubCm_init
 *
 * @brief   Initialization function for micro BLE connection monitor.
 *          This function initializes the callbacks and default connection
 *          info.
 *
 * @param   appCb - callback to pass messages to application
 *
 * @return  true: CM initialization successful
 *          false: CM initialization failed
 */
bool ubCm_init(pfnAppCb appCb);

/*********************************************************************
 * @fn      ubCM_isSessionActive
 *
 * @brief   Check if the CM sessionId is active
 *
 * @param   sessionId - Value identifying the CM session to check.
 *
 * @return  CM_SUCCESS = 0: CM session active
 *          CM_FAILED_NOT_FOUND = 2: CM session not active
 */
uint8_t ubCM_isSessionActive(uint8_t sessionId);

/*********************************************************************
 * @fn      ubCM_startNewSession
 *
 * @brief   Check if a new CM session is active. If so, add the new connection
 *          parameters to the pending connection list. Otherwise, initializes  the
 *          new connection data to start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 * @param   accessAddress - accessAddress for requested connection
 * @param   connRole - RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
 * @param   connInterval - connection interval
 * @param   hopValue - hop value
 * @param   currChan - the next channel that the requested connection will transmit on
 * @param   chanMap - channel map
 * @param   crcInit - crcInit value
 *
 * @return  A valid index:         A valid index will be less than CM_MAX_SESSIONS
 *          CM_MAX_SESSIONS + 1:   The connection is pending
 *          CM_INVALID_SESSION_ID: We have reached our session limit
 */
uint8_t ubCM_startNewSession(uint8_t hostConnHandle, uint32_t accessAddress,
                             uint8_t connRole, uint16_t connInterval,
                             uint8_t hopValue, uint8_t nextChan,
                             uint8_t *chanMap, uint32_t crcInit);

/*********************************************************************
 * @fn      ubCM_start
 *
 * @brief   To establish a new CM session or continue monitor an
 *          existing CM session.
 *
 * @param   sessionId - Value identifying the CM session requested
 *          to start.
 *
 * @return  CM_SUCCESS = 0: CM session started.
 *          CM_FAILED_TO_START = 1: Failed to start because next
 *          anchor point is missed.
 */
uint8_t ubCM_start(uint8_t sessionId);

/*********************************************************************
 * @fn      ubCM_stop
 *
 * @brief   To discontinue a CM session.
 *
 * @param   sessionId - Value identifying the CM session requested to stop.
 *          For an invalid sessionId, this function will return
 *          CM_FAILED_NOT_FOUND status.
 *
 * @return  CM_SUCCESS = 0: CM session ended
 *          CM_FAILED_NOT_FOUND = 2: Could not find CM session to stop
 */
uint8_t ubCM_stop(uint8_t sessionId);

/*********************************************************************
 * @fn      ubCM_startExt
 *
 * @brief   Initializes new connection data to start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 * @param   accessAddress - accessAddress for requested connection
 * @param   connInterval - connection interval
 * @param   hopValue - hop value
 * @param   nextChan - the next channel that the requested connection will transmit on
 * @param   chanMap - channel map
 * @param   crcInit - crcInit value
 * @param   reqTime - time stamp of the new connection request
 *
 * @return CM_SUCCESS = 0: CM session started
 *         CM_FAILED_TO_START = 1: Could not start CM session
 */
uint8_t ubCM_startExt(uint8_t hostConnHandle, uint32_t accessAddress,
                      uint16_t connInterval, uint8_t hopValue,
                      uint8_t nextChan, uint8_t *chanMap,
                      uint32_t crcInit, uint32_t reqTime);

/*********************************************************************
 * @fn      ubCM_InitCmSession
 *
 * @brief   Initializes new connection data to start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 * @param   accessAddress - accessAddress for requested connection
 * @param   connRole - RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
 * @param   connInterval - connection interval
 * @param   hopValue - hop value
 * @param   nextChan - the next channel that the requested connection will transmit on
 * @param   chanMap - channel map
 * @param   crcInit - crcInit value
 * @param   reqTime - time stamp of the new connection request
 *
 * @return CM_SUCCESS = 0: CM session started
 *         CM_FAILED_TO_START = 1: Could not start CM session
 */
uint8_t ubCM_InitCmSession(uint8_t hostConnHandle, uint32_t accessAddress,
                           uint8_t connRole, uint16_t connInterval,
                           uint8_t hopValue, uint8_t nextChan, uint8_t *chanMap,
                           uint32_t crcInit, uint32_t reqTime);

/*********************************************************************
 * @fn      ubCM_updateExt
 *
 * @brief   Initializes new connection data without start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 *          accessAddress - accessAddress for requested connection
 *          sessionId - Value identifying the CM session requested to stop.
 *          connInterval - connection interval
 *          hopValue - hop value
 *          nextChan - the next channel that the requested connection will transmit on
 *          chanMap - channel map
 *          crcInit - crcInit valu
 *
 * @return  CM_SUCCESS = 0: Update channel map successfully
 *          CM_FAILED_TO_START = 1: Could not update new parameters
 */
uint8_t ubCM_updateExt(uint8_t sessionId, uint8_t hostConnHandle, uint32_t accessAddress, uint16_t connInterval, uint8_t hopValue, uint8_t nextChan, uint8_t *chanMap, uint32_t crcInit);

/*********************************************************************
* @fn     ubCM_isThereACollisionBetweenConn
*
* @brief  This function check if there is a collision or not
*         between two connections?
*
* @param sessionId1 - The first sessionId id.
* @param sessionId2 - The second sessionId id.
*
* @return TRUE  - There is a collision.
*         FALSE - There is no collision.
*         CM_INVALID_SESSION_ID - Invalid input.
*/
uint8_t ubCM_isThereACollisionBetweenConn(uint8_t sessionId1, uint8_t sessionId2);

/*********************************************************************
* @fn     ubCM_selectConn
*
* @brief  This is the function that chooses the correct connection
*         out of 2 input connections.
*         The chosen connection would depend on the following parameters:
*         - Connection interval and missed air time.
*         - The miss count of the connections.
*
* @param sessionId1 - The First sessionId for comparison.
* @param sessionId2 - The Second sessionId id for comparison.
*
* @return Connection ID of the next connection.
*         CM_INVALID_SESSION_ID - In case both connections are inactive.
*/
uint8_t ubCM_selectConn(uint8_t sessionId1, uint8_t sessionId2);

/*********************************************************************
 * @fn      ubCM_findNextPriorityEvt
 *
 * @brief   Find the next connection event.
 *
 * @param   None.
 *
 * @return  next sessionId.
 */
uint8_t ubCM_findNextPriorityEvt(void);

/*********************************************************************
 * @fn      ubCM_setupNextCMEvent
 *
 * @brief   The connection updates will be managed by the Host device
 *          and we are interested in connection events even when the
 *          peripheral may not send data. The following connInfo are updated
 *          after the call: currentChan, ScanDuration, currentStartTime,
 *          and nextStartTime.
 *
 * @param   sessionId - Value identifying the CM session.
 *
 * @return  None.
 */
void ubCM_setupNextCMEvent(uint8_t sessionId);

/*********************************************************************
 * @fn      ubCM_isAnchorRelevant
 *
 * @brief   Check if the given anchor time is greater or not from the last new session start time
 *
 * @param   anchorTime - last anchor time.
 *
 * @return      TRUE:  When anchor time is greater than the last new session start time ; otherwise
 *              return FALSE.
 */
uint8_t ubCM_isAnchorRelevant(uint32_t anchorTime);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MICROBLECM_H */
