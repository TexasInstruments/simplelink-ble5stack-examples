/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdlib.h>

#include <ti/display/Display.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/BIOS.h>
#include "bcomdef.h"

#include <ti_drivers_config.h>
#include "board_key.h"

#include "ugap.h"
#include "urfc.h"

#include "util.h"

#include "micro_ble_cm.h"
#include "rtls_ctrl_api.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BLE_RAT_IN_16US              64   // Connection Jitter
#define BLE_RAT_IN_64US              256  // Radio Rx Settle Time
#define BLE_RAT_IN_100US             400  // 1M / 2500 RAT ticks (SCA PPM)
#define BLE_RAT_IN_140US             560  // Rx Back-end Time
#define BLE_RAT_IN_150US             600  // T_IFS
#define BLE_RAT_IN_256US             1024 // Radio Overhead + FS Calibration
#define BLE_RAT_IN_1515US            6063 // Two thrid of the maximum packet size
#define BLE_RAT_IN_2021US            8084 // Maximum packet size

#define BLE_SYNTH_CALIBRATION        (BLE_RAT_IN_256US)
#define BLE_RX_SETTLE_TIME           (BLE_RAT_IN_64US)
#define BLE_RX_RAMP_OVERHEAD         (BLE_SYNTH_CALIBRATION + \
                                      BLE_RX_SETTLE_TIME)
#define BLE_RX_SYNCH_OVERHEAD        (BLE_RAT_IN_140US)
#define BLE_JITTER_CORRECTION        (BLE_RAT_IN_16US)

#define BLE_OVERLAP_TIME             (BLE_RAT_IN_150US)
#define BLE_EVENT_PAD_TIME           (BLE_RX_RAMP_OVERHEAD +  \
                                      BLE_JITTER_CORRECTION + \
                                      BLE_RAT_IN_100US)

#define BLE_HOP_VALUE_MAX            16
#define BLE_HOP_VALUE_MIN            5
#define BLE_COMB_SCA_MAX             540
#define BLE_COMB_SCA_MIN             40

// The most significant nibble for possible "nextStartTime" wrap around.
#define BLE_MS_NIBBLE                0xF0000000

// Threshold for consecutive missed packets in monitor session before terminated link
#define BLE_CONSECUTIVE_MISSED_CONN_EVT_THRES       30

// Bus latency - used to calculate number of connection events to skip
#define BUS_LATENCY_IN_MS            320

#define BITS_PER_BYTE                8

/*********************************************************************
 * TYPEDEFS
 */
// Element of connection's list
typedef struct cmListElem
{
  uint8_t           sessionId;        //! Connection session Id
  uint32_t          startTime;        //! Current anchor point
  struct cmListElem *next;
} cmListElem_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
Display_Handle dispHandle;
extern uint8 tbmRowItemLast;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
ubCM_GetConnInfoComplete_t ubCMConnInfo;

pfnAppCb gAppCb = NULL;

uint8_t cmCentral = TRUE; //! Flag to identify a packet from Central

uint8_t newSessionActive  = FALSE;                    //! Indication of monitoring a new connection
uint8_t newSessionId      = CM_INVALID_SESSION_ID;    //! The session ID of the new connection
uint8_t nextSessionId     = CM_INVALID_SESSION_ID;    //! The session ID of the next monitored connection

cmListElem_t *cmConnSortedList = NULL;   //! Sorted list active connections by their start time
cmNewConn_t  *cmNewConnPend    = NULL;   //! List of new connection requests
cmNewConn_t  *cmLastConnPend   = NULL;   //! pointer to the last element of the new connection requests

/*********************************************************************
 * LOCAL FUNCTIONS
 */

uint8_t ubCM_callApp(uint8_t eventId, uint8_t *data);
static uint8_t setNextDataChan(uint8_t sessionId, uint16_t numEvents);
static void processChanMap(uint8_t sessionId, uint8 *chanMap);
static void addConnSortedList(cmListElem_t *newElem);
static uint8_t removeConnSortedList(int8_t sessionId, int8_t freeElem);
static void realignConnSortedList(void);
static void monitor_stateChangeCB(ugapMonitorState_t newState);
static void monitor_indicationCB(bStatus_t status, uint8_t sessionId,
                                 uint8_t len, uint8_t *pPayload);

/*********************************************************************
 * @fn      setNextDataChan
 *
 * @brief   This function returns the next data channel for a CM connection
 *          based on the previous data channel, the hop length, and one
 *          connection interval to the next active event. If the
 *          derived channel is "used", then it is returned. If the derived
 *          channel is "unused", then a remapped data channel is returned.
 *
 *          Note: nextChan is updated, and must remain that way, even if the
 *                remapped channel is returned.
 *
 * @param   sessionId - Value identifying the CM session.
 * @param   numEvents - Number of events.
 *
 * @return  Next data channel.
 */
static uint8_t setNextDataChan(uint8_t sessionId, uint16_t numEvents)
{
  uint8_t i;
  ubCM_ConnInfo_t *connInfo;

  // get pointer to connection info
  connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

  // check the starting case  where nextEvent and currentEvent are both zero
  numEvents = (numEvents == 0) ? 1 : numEvents;

  // calculate the data channel based on hop and number of events to next event
  // Note: currentChan is now called lastUnmappedChannel in the spec.
  connInfo->nextChan = (connInfo->nextChan + (connInfo->hopValue * numEvents)) %
                         CM_MAX_NUM_DATA_CHAN;

  //BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "N-e=%d, n=%d\n", numEvents, connPtr->nextChan);

  // check if next channel is a used data channel
  for (i = 0; i < connInfo->numUsedChans; i++)
  {
    // check if next channel is in the channel map table
    if ( connInfo->nextChan == connInfo->chanMap[i] )
    {
      // it is, so return the used channel
      return (connInfo->nextChan);
    }
  }

  // next channel is unused, so return the remapped channel
  return (connInfo->chanMap[connInfo->nextChan % connInfo->numUsedChans]);
}

/*********************************************************************
 * @fn          processChanMap
 *
 * @brief       This function is used to convert a channel map in bit map
 *              format of used data channels to a table of consecutively
 *              ordered entries of used data channels. This is used by the
 *              getNextDataChan algorithm, and should be done whenever the
 *              data channel map is updated.
 * *
 * @param       sessionId - Value identifying the CM session.
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used".
 * @return      None.
 */
static void processChanMap(uint8_t sessionId, uint8_t *chanMap)
{
  uint8_t i, j;
  ubCM_ConnInfo_t *connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

  // channels 37..39 are not data channels and these bits should not be set
  chanMap[CM_NUM_BYTES_FOR_CHAN_MAP-1] &= 0x1F;

  // clear the count of the number of used channels
  connInfo->numUsedChans = 0;

  // the used channel map uses 1 bit per data channel, or 5 bytes for 37 chans
  for (i = 0; i < CM_NUM_BYTES_FOR_CHAN_MAP; i++)
  {
    // save each valid channel for every channel map bit that's set
    // Note: When i is on the last byte, only 5 bits need to be checked, but
    //       it is easier here to check all 8 with the assumption that the rest
    //       of the reserved bits are zero.
    for (j = 0; j < BITS_PER_BYTE; j++)
    {
      // check if the channel is used; only interested in used channels
      if ( (chanMap[i] >> j) & 1 )
      {
        // sequence used channels in ascending order
        connInfo->chanMap[ connInfo->numUsedChans ] = (i*8U)+j;

        // count it
        connInfo->numUsedChans++;
      }
    }
  }
}

/*********************************************************************
 * @fn      addConnSortedList
 *
 * @brief   Function to add an element into a sorted linked list by start time
 *
 * @param   newElem - New element to insert
 *
 * @return  None.
 */
static void addConnSortedList(cmListElem_t *newElem)
{
  cmListElem_t *elem = cmConnSortedList;

  // In case of empty list or newElemt start time is the smallest
  if (cmConnSortedList == NULL || uble_timeCompare(cmConnSortedList->startTime, newElem->startTime))
  {
    newElem->next = cmConnSortedList;
    cmConnSortedList = newElem;
    return;
  }

  // Find the location of the newElem in the sorted list
  while (elem->next != NULL && uble_timeCompare(newElem->startTime, elem->next->startTime))
    elem = elem->next;

  // Add new element to the sorted list
  newElem->next = elem->next;
  elem->next = newElem;

  return;
}

/*********************************************************************
 * @fn      removeConnSortedList
 *
 * @brief   Function to remove an element into from sorted linked list
 *
 * @param   sessionId - Connection session Id
 * @param   freeElem - If true, free the allocated memory of this connection
 *
 * @return  None.
 */
static uint8_t removeConnSortedList(int8_t sessionId, int8_t freeElem)
{
  cmListElem_t *elem = cmConnSortedList;
  cmListElem_t *pPrev;
  volatile uint32_t keyHwi;

  while (elem != NULL)
  {
    if (elem->sessionId == sessionId)
    {
      // Check if we need to delete the head
      if (elem == cmConnSortedList)
      {
        cmConnSortedList = cmConnSortedList->next;
      }
      else
      {
        pPrev->next = elem->next;
      }

      if (freeElem)
      {
        // Free allocated memory
        keyHwi = Hwi_disable();
        free(elem);
        Hwi_restore(keyHwi);
      }
      return CM_SUCCESS;
    }
    pPrev = elem;
    elem = elem->next;
  }

  return CM_FAILED_NOT_FOUND;
}

/*********************************************************************
 * @fn      realignConnSortedList
 *
 * @brief   Function to realign connections sorted linked list
 *
 * @return  None.
 */
static void realignConnSortedList(void)
{
  uint32_t     currentTime = RF_getCurrentTime();
  cmListElem_t *elem;

  while (uble_timeCompare(currentTime, cmConnSortedList->startTime))
  {
    elem = cmConnSortedList;

    // Setup connection's parameters and add it to the connections sorted list
    ubCM_setupNextCMEvent(elem->sessionId);

    // Remove connection from the connection sorted list
    removeConnSortedList(elem->sessionId, FALSE);

    // Update startTime and add connection to the connections sorted list
    elem->startTime = ubCMConnInfo.ArrayOfConnInfo[elem->sessionId-1].nextStartTime;
    addConnSortedList(elem);
  }

  return;
}

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
uint8_t ubCM_isThereACollisionBetweenConn(uint8_t sessionId1, uint8_t sessionId2)
{
  uint32_t         delta     = 0.4 * BLE_TO_RAT; //! processing time 0.4 in 625us increments (250us)
  ubCM_ConnInfo_t *connPtr1 = &ubCMConnInfo.ArrayOfConnInfo[sessionId1-1];
  ubCM_ConnInfo_t *connPtr2 = &ubCMConnInfo.ArrayOfConnInfo[sessionId2-1];


  /********************************************/
  /************ Check Valid Input *************/
  /********************************************/
  // Check if these connections are valid and active?
  if (ubCM_isSessionActive(sessionId1) == CM_SUCCESS)
  {
    if (ubCM_isSessionActive(sessionId2) != CM_SUCCESS)
    {
      // Connection 1 is active, connection 2 is inactive.
      return FALSE;
    }
  }
  else if (ubCM_isSessionActive(sessionId2) == CM_SUCCESS)
  {
    // Connection 2 is active, connection 1 is inactive.
    return FALSE;
  }
  else
  {
    // Connection 1 & 2 are not-active.
    return CM_INVALID_SESSION_ID;
  }

  /********************************************/
  /************** Collision Check *************/
  /********************************************/
  // Check if the second start before the first?
  if (connPtr1->nextStartTime > connPtr2->nextStartTime)
  {
    // Check for collision between the two (in RAT TICKS).
    if (connPtr2->nextStartTime + connPtr2->scanDuration * BLE_TO_RAT + delta < connPtr1->nextStartTime)
    {
      // no collision.
      return FALSE;
    }
    else // There is a collision.
    {
      return TRUE;
    }
  }
  else // First connection starts before the second
  {
    // Check for collision between the two (in RAT TICKS).
    if (connPtr1->nextStartTime + connPtr1->scanDuration * BLE_TO_RAT + delta < connPtr2->nextStartTime)
    {
      // no collision.
      return FALSE;
    }
    else // There is a collision.
    {
      return TRUE;
    }
  }
}

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
uint8_t ubCM_selectConn(uint8_t sessionId1, uint8_t sessionId2)
{
  ubCM_ConnInfo_t *connPtr1 = &ubCMConnInfo.ArrayOfConnInfo[sessionId1-1];
  ubCM_ConnInfo_t *connPtr2 = &ubCMConnInfo.ArrayOfConnInfo[sessionId2-1];

  /********************************************/
  /************ Check Valid Input *************/
  /********************************************/
  // Check if these connections are valid and active?
  if (ubCM_isSessionActive(sessionId1) == CM_SUCCESS)
  {
    if (ubCM_isSessionActive(sessionId2) != CM_SUCCESS)
    {
      // Connection 1 is active, connection 2 is inactive.
      return FALSE;
    }
  }
  else if (ubCM_isSessionActive(sessionId2) == CM_SUCCESS)
  {
    // Connection 2 is active, connection 1 is inactive.
    return FALSE;
  }
  else
  {
    // Connection 1 & 2 are not-active.
    return CM_INVALID_SESSION_ID;
  }

  /* NOTE: Increment Miss Count of the un-selected connection would be done in the RealignConn() function. */

  /*  In this Stage - Both connections are active */

  /********************************************/
  /**** Check Conn Interval and Miss Count ****/
  /********************************************/
  // Is Conn 1 interval larger then Conn 2 interval?
  if (connPtr1->connInterval > connPtr2->connInterval)
  {
    // Is (Conn 2 interval * Conn 2 miss count) larger then Conn 1 interval?
    if (connPtr1->connInterval <= connPtr2->connInterval * connPtr2->missedEvents)
    {
      // The interval of Conn 2 is smaller and missed more air time then the interval of Conn 1.
      return sessionId2;
    }
    else
    {
      return sessionId1;
    }
  }
  // Is Conn 2 interval larger then Conn 1 interval?
  if (connPtr2->connInterval > connPtr1->connInterval)
  {
    // Is (Conn 1 interval * Conn 1 miss count) larger then Conn 2 interval?
    if (connPtr2->connInterval <= connPtr1->connInterval * connPtr1->missedEvents)
    {
     // The interval of Conn 1 is smaller and missed more air time then the interval of Conn 2.
      return sessionId1;
    }
    else
    {
      return sessionId2;
    }
  }

  /*  In this Stage - Both connections have the same interval */

  /**********************************************************/
  /****** Check if one of the sessions is out of sync *******/
  /**********************************************************/

  // Check if the CM is getting out of sync due to opposite scheduling decisions from the Coordinator.
  // Try to switch between two CM's monitoring sessions for two connection events to get back in sync with the Coordinator.

  // Check if one of the session is in the process of getting back in sync.
  if (connPtr1->outOfSyncFlag != 0 || connPtr2->outOfSyncFlag != 0)
  {
    if (connPtr1->outOfSyncFlag != 0 &&
        (uble_timeDelta(connPtr1->nextStartTime, connPtr1->outOfSyncTimeStamp) > 2*connPtr1->connInterval))
    {
      connPtr1->outOfSyncFlag = 0;
      connPtr1->outOfSyncTimeStamp = 0;
    }

    if (connPtr2->outOfSyncFlag != 0 &&
        (uble_timeDelta(connPtr2->nextStartTime, connPtr2->outOfSyncTimeStamp) > 2*connPtr2->connInterval))
    {
      connPtr2->outOfSyncFlag = 0;
      connPtr2->outOfSyncTimeStamp = 0;
    }
  }

  // Check if one of the sessions is out of sync.
  // If one of the sessions is out of sync due to a different reason (other than opposite scheduling decisions from the Coordinator),
  // switching between two sessions won't help, and the consecutiveTimesMissed parameter will continue to increase.
  // To not interfere with the session that is still in sync, we will try to switch between the sessions once every four consecutiveTimesMissed.
  else if ((connPtr1->consecutiveTimesMissed > 0 && connPtr1->consecutiveTimesMissed % 4 == 0) ||
           (connPtr2->consecutiveTimesMissed > 0 && connPtr2->consecutiveTimesMissed % 4 == 0))
  {
    connPtr1->outOfSyncFlag = 1;
    connPtr2->outOfSyncFlag = 1;

    connPtr1->outOfSyncTimeStamp = connPtr1->nextStartTime;
    connPtr2->outOfSyncTimeStamp = connPtr2->nextStartTime;

    // Switch between the connection to change the scheduling decision and try to get the session back in sync.
    connPtr1 = &ubCMConnInfo.ArrayOfConnInfo[sessionId2-1];
    connPtr2 = &ubCMConnInfo.ArrayOfConnInfo[sessionId1-1];
  }

  /********************************************/
  /*********** Check Conn Miss Count **********/
  /********************************************/
  // Is Conn 1 miss count larger then Conn 2 miss count?
  if (connPtr1->missedEvents > connPtr2->missedEvents)
  {
    // Conn 1 was missed more times then Conn 2.
    return sessionId1;
  }
  // Is Conn 2 miss count larger then Conn 1 miss count?
  else if (connPtr2->missedEvents > connPtr1->missedEvents)
  {
    // Conn 2 was missed more time then Conn 1.
    return sessionId2;
  }

  /*  In this Stage - Both connections have the same missed count */

  /********************************************/
  /*********** Check Conn Start Time **********/
  /********************************************/
  // Choose the connection that starts first.
  if (connPtr1->nextStartTime <= connPtr2->nextStartTime)
  {
    return sessionId1;
  }
  else
  {
    return sessionId2;
  }
}

/*********************************************************************
 * @fn      ubCM_findNextPriorityEvt
 *
 * @brief   Find the next connection event.
 *
 * @param   None.
 *
 * @return  next sessionId.
 */
uint8_t ubCM_findNextPriorityEvt(void)
{
  cmListElem_t  *activeConnElem1;
  cmListElem_t  *activeConnElem2;
  uint8_t       selectedSessionId = CM_INVALID_SESSION_ID;
  uint8_t       earliestSessionId = CM_INVALID_SESSION_ID;
  uint8_t       numOfCollisionComprison = 0;
  uint32_t      currentTime = RF_getCurrentTime();

  // Realign connections sorted list.
  realignConnSortedList();

  /********************************************/
  /********* Find First Active Conn ID ********/
  /********************************************/

  // In case we have only one connection - return it.
  if ( ubCMConnInfo.numHandles == 1 )
  {
    ubCMConnInfo.ArrayOfConnInfo[cmConnSortedList->sessionId - 1].currentChan = setNextDataChan(cmConnSortedList->sessionId, ubCMConnInfo.ArrayOfConnInfo[cmConnSortedList->sessionId - 1].numEvents);
    nextSessionId = cmConnSortedList->sessionId;

    // Return the only connection found.
    return (cmConnSortedList->sessionId);
  }

  // Set the first active connection.
  selectedSessionId = cmConnSortedList->sessionId;

  // Set the earliest connection id.
  earliestSessionId = selectedSessionId;

  // Set sessionId of first two active connections.
  activeConnElem1 = cmConnSortedList;
  activeConnElem2 = cmConnSortedList->next;

  /********************************************/
  /********* Find the next Connection *********/
  /********************************************/
  // Look for the most appropriate connection among all active connection
  // Increment the to the nextActiveConnId and also increment the number of comparisons.
  // Go over the active connections.
  while ( ((activeConnElem1 != NULL) && (activeConnElem2 != NULL))
          && ( numOfCollisionComprison < CM_MAX_COLLISION_COMPRISON) )
  {
    /***********************************/
    /********* Collision Check *********/
    /***********************************/
    // Check if the first connection event overlaps the second connection event?
    if (ubCM_isThereACollisionBetweenConn(activeConnElem1->sessionId, activeConnElem2->sessionId))
    {
      // There is a collision - choose the highest priority connection.
      // Valid Output: No need to check that selectedConnId - because we are going over active
      // connections only.
      selectedSessionId = ubCM_selectConn(activeConnElem1->sessionId, activeConnElem2->sessionId);
    }
    else
    {
      // Exit - We have found the selected connection between overlapping connections.
      break;
    }

    // Next iteration.
    // In case the selectedSessionId is origin from activeConnElem2
    if (selectedSessionId == activeConnElem2->sessionId )
    {
      // increment the second active connection index.
      activeConnElem1 = activeConnElem2;
    }

    // increment the second active connection index.
    activeConnElem2 = activeConnElem2->next;

    // Increment the number of comparisons index.
    // It is limited to be up to CM_MAX_COLLISION_COMPRISON.
    numOfCollisionComprison++;
  }

  if ((earliestSessionId == selectedSessionId) || (ubCM_isThereACollisionBetweenConn(earliestSessionId,selectedSessionId) == TRUE))
  {
    // Return the selectedConnId because it has higher priority than earliestConnId and they are colliding.
    ubCMConnInfo.ArrayOfConnInfo[selectedSessionId - 1].currentChan = setNextDataChan(selectedSessionId, ubCMConnInfo.ArrayOfConnInfo[selectedSessionId - 1].numEvents );
    nextSessionId = selectedSessionId;

    return selectedSessionId;
  }
  else
  {
    ubCMConnInfo.ArrayOfConnInfo[earliestSessionId - 1].currentChan = setNextDataChan(earliestSessionId, ubCMConnInfo.ArrayOfConnInfo[earliestSessionId - 1].numEvents );
    nextSessionId = earliestSessionId;

    // Return the earliestConnId
    return earliestSessionId;
  }
}

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
 * @param   currentTime  - Time reference to update connection's parameters.
 *
 * @return  None.
 */
void ubCM_setupNextCMEvent(uint8_t sessionId)
{
  ubCM_ConnInfo_t   *connInfo;
  uint32_t          timeToNextEvt;
  uint32_t          scanDuration;
  uint32_t          deltaTime;
  volatile uint32   keySwi;
  volatile uint32_t keyHwi;
  uint32_t          currentTime = RF_getCurrentTime();

  keySwi = Swi_disable();

  // get pointer to connection info
  connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

  /********************************************/
  /************ Calculate numEvents ***********/
  /********************************************/

  // deltaTime is the distance between the current time and
  // the last anchor point.
  deltaTime = uble_timeDelta(currentTime, connInfo->timeStampCentral);

  // Figure out how many connection events have passed since the last anchor point.
  connInfo->numEvents = deltaTime / (connInfo->connInterval * BLE_TO_RAT) + 1;

  /********************************************/
  /*********** Calculate missedEvents *********/
  /********************************************/

  // deltaTime is the distance between the current time and
  // the last monitoring session of this connection.
  deltaTime = uble_timeDelta(currentTime, connInfo->lastStartTime);

  // Figure out how many connection events have passed since last monitoring session.
  connInfo->missedEvents = deltaTime / (connInfo->connInterval * BLE_TO_RAT) + 1;

  /*********************************************************************/
  /** Calculate connection parameters for the next monitoring session **/
  /*********************************************************************/

  // Update start time to the new anchor point.
  connInfo->currentStartTime = connInfo->timeStampCentral;

  // time to next event is just the connection intervals in 625us ticks
  timeToNextEvt = connInfo->connInterval * connInfo->numEvents;

  // advance the anchor point in RAT ticks
  connInfo->currentStartTime += (timeToNextEvt * BLE_TO_RAT);

  // account for radio startup overhead and jitter per the spec, pull values
  // from BLE stack. Also need to compensate for "missing central and tracking
  // peripheral" scenario. The RX window will be opened ahead of time so that
  // the central is guaranteed to be captured.
  connInfo->currentStartTime -= (BLE_RX_RAMP_OVERHEAD +
                                 BLE_JITTER_CORRECTION +
                                 BLE_RAT_IN_256US +
                                 BLE_RAT_IN_150US);

  // Calc timerDrift, scaFactor is (CM ppm + Central ppm).
  connInfo->timerDrift = ( (timeToNextEvt * connInfo->combSCA) / BLE_RAT_IN_100US ) + 1;

  // If not enough time to start scan, bump to next connection interval
  while (uble_timeCompare(currentTime, (connInfo->currentStartTime - connInfo->timerDrift)) ||
      uble_timeCompare(currentTime, (connInfo->currentStartTime + BLE_EVENT_PAD_TIME - connInfo->timerDrift)))
  {
    connInfo->numEvents++;
    connInfo->missedEvents++;
    timeToNextEvt +=  connInfo->connInterval;
    connInfo->currentStartTime += connInfo->connInterval * BLE_TO_RAT;
    connInfo->timerDrift = ((timeToNextEvt * connInfo->combSCA) / BLE_RAT_IN_100US) + 1;
  }

  // setup the Start Time of the receive window
  // Note: In the case we don't receive a packet at the first connection
  //       event, (and thus, don't have an updated anchor point), this anchor
  //       point will be used for finding the start of the connection event
  //       after that. That is, the update is relative to the last valid anchor
  //       point.
  // Note: If the AP is valid, we have to adjust the AP by timer drift. If the
  //       AP is not valid, we still have to adjust the AP based on the amount
  //       of timer drift that results from a widened window. Since SL is
  //       disabled when the AP is invalid (i.e. a RX Timeout means no packet
  //       was received, and by the spec, SL is discontinued until one is),
  //       the time to next event is the connection interval, and timer drift
  //       was re-calculated based on (SL+1)*CI where SL=0.
  connInfo->nextStartTime = connInfo->currentStartTime - connInfo->timerDrift;

  // setup the receiver Timeout time
  // Note: If the AP is valid, then timeoutTime was previously cleared and any
  //       previous window widening accumulation was therefore reset to zero.
  // Note: Timeout trigger remains as it was when connection was formed.
  scanDuration = (2 * connInfo->timerDrift);

  // additional widening based on whether the AP is valid
  // Note: The overhead to receive a preamble and synch word to detect a
  //       packet is added in case a packet arrives right at the end of the
  //       receive window.
  scanDuration += BLE_RX_RAMP_OVERHEAD + (2 * BLE_JITTER_CORRECTION) + BLE_RX_SYNCH_OVERHEAD;

  // The CM endTime is a bit different than a standard peripheral. We only want to keep Rx on
  // long enough to receive 2 packets at each AP, one Central and one Peripheral if present.
  // Therefore we only need to at most remain active for the full timeoutTime + the duration
  // it takes to receive to max size packets + 300 us or 2*T_IFS for the min inter frame timing.
  // 1515 us is based on the time it takes to receive 2 packets with ~160 byte payloads + headers.
  scanDuration += (BLE_RAT_IN_1515US + 2 * BLE_RAT_IN_150US);
  connInfo->scanDuration = (scanDuration / BLE_TO_RAT) + 1;

  // Next channel should be calculated by the num of events that passed since the last time
  // we got an indication from the central.
  connInfo->nextChan = connInfo->lastUnmappedChannel;

  Swi_restore(keySwi);
}

/*********************************************************************
 * CALLBACKS
 */

/*********************************************************************
 * @fn      monitor_stateChangeCB
 *
 * @brief   Callback from Micro Monitor indicating a state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void monitor_stateChangeCB(ugapMonitorState_t newState)
{
  volatile uint32_t keyHwi;

  keyHwi = Hwi_disable();
  ugapMonitorState_t *pNewState = malloc(sizeof(ugapMonitorState_t));
  Hwi_restore(keyHwi);

  // Drop if we could not allocate
  if (!pNewState)
  {
    return;
  }

  *pNewState = newState;

  // Notify application that monitor state has changed
  if (FALSE == ubCM_callApp(CM_MONITOR_STATE_CHANGED_EVT, pNewState))
  {
    // Calling App failed, free the message
    keyHwi = Hwi_disable();
    free(pNewState);
    Hwi_restore(keyHwi);
  }
}

/*********************************************************************
 * @fn      monitor_indicationCB
 *
 * @brief   Callback from Micro monitor notifying that a data
 *          packet is received.
 *
 * @param   status - status of a monitoring scan
 * @param   sessionId - session ID
 * @param   len - length of the payload
 * @param   pPayload - pointer to payload
 *
 * @return  None.
 */
static void monitor_indicationCB(bStatus_t status, uint8_t sessionId,
                                 uint8_t len, uint8_t *pPayload)
{
  uint8_t  rawRssi;
  int8_t   rssi;
  uint32_t timeStamp;
  ubCM_ConnInfo_t *connInfo;
  packetReceivedEvt_t *pPacketInfo;
  rfStatus_t rfStatus;
  volatile uint32_t keyHwi;
  uint32_t currentTime = RF_getCurrentTime();

  // Access the connection info array
  connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

  // Copy RF status
  memcpy(&rfStatus, pPayload + len + CM_RFSTATUS_OFFSET, sizeof(rfStatus));

  //  We would like to check if the RSSI that we received is valid
  //  To do this we need to first verify if the CRC on the packet was correct
  //  If it is not correct then the validity of the RSSI cannot be trusted and it should be discarded
  if (!rfStatus.bCrcErr)
  {
    // CRC is good
    rawRssi = *(pPayload + len + CM_RSSI_OFFSET);

    // Corrects RSSI if valid
    rssi = CM_CHECK_LAST_RSSI( rawRssi );
  }
  else
  {
    // CRC is bad
    rssi = CM_RSSI_NOT_AVAILABLE;
  }

  timeStamp = *(uint32_t *)(pPayload + len + CM_TIMESTAMP_OFFSET);

  keyHwi = Hwi_disable();
  pPacketInfo = malloc(sizeof(packetReceivedEvt_t));
  Hwi_restore(keyHwi);

  // Drop the packet if we could not allocate
  if (!pPacketInfo)
  {
    return;
  }

  pPacketInfo->len = len;
  pPacketInfo->pPayload = pPayload;
  pPacketInfo->seesionId = sessionId;

  if (newSessionActive == TRUE &&
      cmCentral == FALSE &&
      (uble_timeDelta(currentTime, connInfo->timeStampCentral) > (CM_CONN_INTERVAL_MIN * BLE_TO_RAT)))
  {
    connInfo->timeStampCentral2 = connInfo->timeStampCentral;
    connInfo->timeStampCentral  = timeStamp;
    connInfo->rssiCentral       = rssi;
    pPacketInfo->centralPacket  = TRUE;
  }

  if (cmCentral == TRUE)
  {
    // Central packet
    connInfo->timeStampCentral     = timeStamp;
    connInfo->timeStampCentral2    = 0;
    connInfo->lastUnmappedChannel  = connInfo->nextChan;
    connInfo->rssiCentral          = rssi;
    connInfo->timesScanned++;
    cmCentral = FALSE;

    pPacketInfo->centralPacket = TRUE;
  }
  else
  {
    // Peripheral packet
    connInfo->timeStampPeripheral  = timeStamp;
    connInfo->rssiPeripheral       = rssi;

    pPacketInfo->centralPacket = FALSE;
  }

  pPacketInfo->status = status;

  // Notify application that a packet was received
  if (FALSE == ubCM_callApp(CM_PACKET_RECEIVED_EVT, (uint8_t *)pPacketInfo))
  {
    // Calling App failed, free the message
    keyHwi = Hwi_disable();
    free(pPacketInfo);
    Hwi_restore(keyHwi);
  }
}

/*********************************************************************
 * @fn      monitor_completeCB
 *
 * @brief   Callback from Micro Monitor notifying that a monitoring
 *          scan is completed.
 *
 * @param   status - How the last event was done. SUCCESS or FAILURE.
 * @param   sessionId - Session ID
 *
 * @return  None.
 */
static void monitor_completeCB(bStatus_t status, uint8_t sessionId)
{
  ubCM_ConnInfo_t *connInfo;
  monitorCompleteEvt_t *pMonitorCompleteEvt;
  volatile uint32_t keyHwi;

  keyHwi = Hwi_disable();
  pMonitorCompleteEvt = malloc(sizeof(monitorCompleteEvt_t));
  Hwi_restore(keyHwi);

  // Drop if we could not allocate
  if (!pMonitorCompleteEvt)
  {
    return;
  }

  // Access the connection info array
  connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId - 1];

  // Update the last start time
  connInfo->lastStartTime = connInfo->nextStartTime;

  // Set event
  pMonitorCompleteEvt->status = status;
  pMonitorCompleteEvt->sessionId = sessionId;
  pMonitorCompleteEvt->channel = connInfo->currentChan;

  // If the threshold for consecutive missing packets is reached, terminate the connection
  if (connInfo->consecutiveTimesMissed > BLE_CONSECUTIVE_MISSED_CONN_EVT_THRES)
  {
    pMonitorCompleteEvt->status = CM_FAILED_NOT_ACTIVE;
  }

  if (pMonitorCompleteEvt->status != CM_FAILED_NOT_ACTIVE)
  {
    if (cmCentral == FALSE)
    {
      // An C->P packet was received. We are not sure about the P->C packet
      // Reset the cmCentral to true for the next connection interval and consecutiveTimesMissed to zero
      cmCentral = TRUE;
      connInfo->consecutiveTimesMissed = 0;
    }
    else
    {
      // Keep a record of missing packets in this monitor session
      connInfo->timesMissed++;
      connInfo->consecutiveTimesMissed++;
    }
  }

  if(connInfo->timeStampCentral2 != 0 && connInfo->consecutiveTimesMissed > 2)
  {
    connInfo->timeStampCentral  = connInfo->timeStampCentral2;
    connInfo->timeStampCentral2 = 0;
  }

  // If there is a new connection stop monitor other sessions
  if (newSessionActive)
  {
    if (newSessionId == sessionId)
    {
      newSessionId = CM_INVALID_SESSION_ID;

      // If there are new pending connections
      while (cmNewConnPend != NULL && newSessionId == CM_INVALID_SESSION_ID)
      {
        cmNewConn_t *elem = cmNewConnPend;

        // Attempt to monitor new session
        newSessionId = ubCM_InitCmSession(elem->connHandle,
                                          elem->accessAddr,
                                          elem->connRole,
                                          elem->connInterval,
                                          elem->hopValue,
                                          elem->currChan,
                                          elem->chanMap,
                                          elem->crcInit,
                                          elem->reqTime);

        // Promote the head of the list
        cmNewConnPend = cmNewConnPend->next;

        if (cmNewConnPend == NULL)
        {
          // This was the last element in the connection's pending list
          // Update the pointer to the last element of the list
          cmLastConnPend = NULL;
        }

        // Free allocated memory
        keyHwi = Hwi_disable();
        free(elem);
        Hwi_restore(keyHwi);
      }

      // If no new connections.
      if (newSessionId == CM_INVALID_SESSION_ID)
      {
        newSessionActive = FALSE;

        // Continue attempt to monitor other sessions
        // The context of this callback is from the uStack task
        (void)ubCM_start(ubCM_findNextPriorityEvt());
      }
    }
    else
    {
      // The ongoing monitoring session ended, start monitoring the new connection
      nextSessionId = newSessionId;
      (void)ubCM_start(newSessionId);
    }
  }
  else if(ubCMConnInfo.numHandles > 0)
  {
    // Continue attempt to monitor other sessions
    // The context of this callback is from the uStack task
    (void)ubCM_start(ubCM_findNextPriorityEvt());
  }

  // Notify application that a monitor session was complete
  if (FALSE == ubCM_callApp(CM_CONN_EVT_COMPLETE_EVT, (uint8_t *)pMonitorCompleteEvt))
  {
    // Calling App failed, free the message.
    keyHwi = Hwi_disable();
    free(pMonitorCompleteEvt);
    Hwi_restore(keyHwi);
  }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ubCm_init
 *
 * @brief   Initialization function for micro BLE connection monitor.
 *          This function initializes the callbacks and default connection
 *          info.
 *
 * @param   none
 *
 * @return  true: CM initialization successful
 *          false: CM initialization failed
 */
bool ubCm_init(pfnAppCb appCb)
{
  uint8_t i;

  ugapMonitorCBs_t monitorCBs = {
    monitor_stateChangeCB,
    monitor_indicationCB,
    monitor_completeCB };

  // Initialize default connection info
  ubCMConnInfo.numHandles = 0;
  for (i = 0; i < CM_MAX_SESSIONS; i++)
  {
    ubCMConnInfo.ArrayOfConnInfo[i].sessionId = CM_INVALID_SESSION_ID;
    ubCMConnInfo.ArrayOfConnInfo[i].connInterval = CM_CONN_INTERVAL_MAX;
    ubCMConnInfo.ArrayOfConnInfo[i].timesScanned = 0;
  }

  if (appCb != NULL)
  {
    // Save application callback
    gAppCb = appCb;
  }
  else
  {
    return FALSE;
  }

  // Initilaize Micro GAP Monitor
  if (SUCCESS == ugap_monitorInit(&monitorCBs))
  {
    return TRUE;
  }

  return FALSE;
}

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
uint8_t ubCM_isSessionActive(uint8_t sessionId)
{
  ubCM_ConnInfo_t *connInfo;

  if (sessionId == 0 || sessionId > CM_MAX_SESSIONS)
  {
    // Not a valid sessionId or no CM session has started
    return CM_FAILED_NOT_FOUND;
  }

  // Access the connection info array
  connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];
  if (connInfo->sessionId == 0)
  {
    // CM session has not been started
    return CM_FAILED_NOT_ACTIVE;
  }

  // CM session is indeed active
  return CM_SUCCESS;
}

/*********************************************************************
 * @fn      ubCM_startNewSession
 *
 * @brief   Check if a new CM session is active. If so, add the new connection
 *          parameters to the pending connection list. Otherwise, initializes  the
 *          new connection data to start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 *          accessAddress - accessAddress for requested connection
 *          connRole - RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
 *          connInterval - connection interval
 *          hopValue - hop value
 *          currChan - the next channel that the requested connection will transmit on
 *          chanMap - channel map
 *          crcInit - crcInit value
 *
 * @return  A valid index:         A valid index will be less than CM_MAX_SESSIONS
 *          CM_MAX_SESSIONS + 1:   The connection is pending
 *          CM_INVALID_SESSION_ID: We have reached our session limit
 */
uint8_t ubCM_startNewSession(uint8_t hostConnHandle, uint32_t accessAddress,
                             uint8_t connRole, uint16_t connInterval,
                             uint8_t hopValue, uint8_t currChan,
                             uint8_t *chanMap, uint32_t crcInit)
{
  uint32_t          reqTime = RF_getCurrentTime();
  volatile uint32_t keyHwi;
  cmNewConn_t       *elem;

  // Check if the CM is monitoring new session.
  if (newSessionActive == TRUE)
  {
    // Insert the connection data to the conncection's pending list.

    // Allocate new element for sorted list.
    keyHwi = Hwi_disable();
    elem = (cmNewConn_t *)malloc(sizeof(cmNewConn_t));
    Hwi_restore(keyHwi);

    if (elem == NULL)
    {
      return CM_INVALID_SESSION_ID;
    }

    // Initializes new connection data.
    elem->connHandle = hostConnHandle;
    elem->accessAddr = accessAddress;
    elem->connRole = connRole;
    elem->connInterval = connInterval;
    elem->hopValue = hopValue;
    elem->currChan = currChan;
    elem->crcInit = crcInit;
    elem->reqTime = reqTime;
    elem->next = NULL;
    memcpy(elem->chanMap, chanMap, CM_NUM_BYTES_FOR_CHAN_MAP);

    if (cmNewConnPend == NULL)
    {
      cmNewConnPend  = elem;
      cmLastConnPend = elem;
    }
    else
    {
      cmLastConnPend->next = elem;
      cmLastConnPend = cmLastConnPend->next;
    }

    return (CM_SUCCESS);
  }
  else
  {
    // Initializes new connection data to start new CM session.
    return (ubCM_InitCmSession(hostConnHandle, accessAddress, connRole, connInterval, hopValue, currChan, chanMap, crcInit, reqTime));
  }
}

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
uint8_t ubCM_start(uint8_t sessionId)
{
  uint8_t status = CM_FAILED_TO_START;
  ubCM_ConnInfo_t *connInfo;

  if (ubCM_isSessionActive(sessionId) == CM_SUCCESS ||
      ubCM_isSessionActive(sessionId) == CM_FAILED_NOT_ACTIVE)
  {
    // Access the connection info array
    connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

    // Range checks
    if (connInfo->hopValue > BLE_HOP_VALUE_MAX ||
        connInfo->hopValue < BLE_HOP_VALUE_MIN ||
        connInfo->combSCA > BLE_COMB_SCA_MAX ||
        connInfo->combSCA < BLE_COMB_SCA_MIN)
    {
      return CM_FAILED_OUT_OF_RANGE;
    }

    // Save the next sessionId
    uble_setParameter(UBLE_PARAM_SESSIONID, sizeof(uint8_t), &sessionId);

    // Kick off the monitor request
    if (ugap_monitorRequest(connInfo->currentChan,
                            connInfo->accessAddr,
                            connInfo->nextStartTime,
                            connInfo->scanDuration,
                            connInfo->crcInit) == SUCCESS)
    {
      // Is the sessionId new?
      if (ubCM_isSessionActive(sessionId) == CM_FAILED_NOT_ACTIVE &&
          ubCMConnInfo.numHandles < CM_MAX_SESSIONS)
      {
        // Activate this session ID
        connInfo->sessionId = sessionId;
        ubCMConnInfo.numHandles++;
      }
      status = CM_SUCCESS;
    }
  }
  return status;
}

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
uint8_t ubCM_stop(uint8_t sessionId)
{
  ubCM_ConnInfo_t *connInfo;

  if (ubCM_isSessionActive(sessionId) == CM_SUCCESS &&
      ubCMConnInfo.numHandles > 0)
  {
    // Access the connection info array
    connInfo = &ubCMConnInfo.ArrayOfConnInfo[sessionId-1];

    // Remove and free from sorted list
    removeConnSortedList(sessionId, TRUE);

    // Mark the sessionId as invalid,
    // Initialize default connection info and
    // decrement the number of connection handles.
    // Note this will leave a hole in the array.
    memset(connInfo, 0, sizeof(ubCM_ConnInfo_t));
    ubCMConnInfo.numHandles--;

    return CM_SUCCESS;
  }
  else
  {
    return CM_FAILED_NOT_FOUND;
  }
}

/*********************************************************************
 * @fn      ubCM_InitCmSession
 *
 * @brief   Initializes new connection data to start new CM sessions
 *
 * @param   hostConnHandle - connHandle from host requests
 *          accessAddress - accessAddress for requested connection
 *          connRole - RTLS Coordinator BLE role (4 - Peripheral, 8- Central)
 *          connInterval - connection interval
 *          hopValue - hop value
 *          nextChan - the next channel that the requested connection will transmit on
 *          chanMap - channel map
 *          crcInit - crcInit value
 *          reqTime - time stamp of the new connection request
 *
 * @return CM_SUCCESS = 0: CM session started
 *         CM_FAILED_TO_START = 1: Could not start CM session
 */
uint8_t ubCM_InitCmSession(uint8_t hostConnHandle, uint32_t accessAddress,
                           uint8_t connRole, uint16_t connInterval,
                           uint8_t hopValue, uint8_t nextChan, uint8_t *chanMap,
                           uint32_t crcInit, uint32_t reqTime)
{
  //i's initial value will make cmStart() fail if ubCMConnInfo.numHandles >= CM_MAX_SESSIONS
  uint8_t           i = ubCMConnInfo.numHandles;
  uint32_t          currTime = RF_getCurrentTime();
  uint32_t          processingTime  = 0.4 * BLE_TO_RAT; // processing time 0.4 in 625us increments (250us)
  uint32_t          deltaTime;
  uint8_t           sessionId;
  uint16_t          chanSkip;
  volatile uint32_t keyHwi;
  cmListElem_t      *elem;

  //First make sure we have not reached our session limit
  if (ubCMConnInfo.numHandles <= CM_MAX_SESSIONS)
  {
    //set sessionId to first inactive session id
    for (sessionId=1; sessionId <= CM_MAX_SESSIONS; sessionId++)
    {
      if(ubCM_isSessionActive(sessionId) == CM_FAILED_NOT_ACTIVE)
      {
        i = sessionId-1;//index of session id is 1 less than actual id
        break;
      }
    }

    // check that session id is not out of bounds
    if (sessionId > CM_MAX_SESSIONS)
    {
      return CM_FAILED_TO_START;
    }

    ubCMConnInfo.ArrayOfConnInfo[i].hostConnHandle = hostConnHandle;
    ubCMConnInfo.ArrayOfConnInfo[i].accessAddr = accessAddress;
    ubCMConnInfo.ArrayOfConnInfo[i].connRole = connRole;

    ubCMConnInfo.ArrayOfConnInfo[i].currentStartTime = currTime + 20 * BLE_TO_RAT;
    ubCMConnInfo.ArrayOfConnInfo[i].nextStartTime = ubCMConnInfo.ArrayOfConnInfo[i].currentStartTime;

    ubCMConnInfo.ArrayOfConnInfo[i].connInterval = connInterval;

    if(ubCMConnInfo.ArrayOfConnInfo[i].connInterval < CM_CONN_INTERVAL_MIN ||
       ubCMConnInfo.ArrayOfConnInfo[i].connInterval > CM_CONN_INTERVAL_MAX)
    {
      return CM_FAILED_TO_START;
    }

    ubCMConnInfo.ArrayOfConnInfo[i].crcInit = crcInit;
    ubCMConnInfo.ArrayOfConnInfo[i].hopValue = hopValue;

    ubCMConnInfo.ArrayOfConnInfo[i].combSCA = 90; // Central = 50 + Peripheral = 40 leave hard coded for now

    ubCMConnInfo.ArrayOfConnInfo[i].currentChan = nextChan;
    ubCMConnInfo.ArrayOfConnInfo[i].nextChan = nextChan;

    processChanMap(sessionId, chanMap);

    // With a 100ms connection interval we will skip 3 channels into the future (in each connection event we can monitor 1 channel)
    // This is mainly done to ensure that even a very slow bus is able to send the connection information in time
    // In time: before the Central/Peripheral already go past the channel that we chose to listen on
    // If Central/Peripheral are already past this point, we will have to wait numChannels*connInterval until we catch the
    // connection once again
    chanSkip = (uint16_t)((BUS_LATENCY_IN_MS/connInterval) + 1);

    // Catch anchor point n+2 connection intervals in the future
    ubCMConnInfo.ArrayOfConnInfo[i].scanDuration = (uint16_t)(connInterval*(chanSkip + 1));

    // DeltaTime is the distance between the current time and time we received the data for the new connection.
    deltaTime = uble_timeDelta(currTime, reqTime);

    // Figure out how many connection events have passed since the last anchor point
    if (deltaTime > connInterval * BLE_TO_RAT)
    {
      chanSkip += (uint16_t) (deltaTime / (connInterval * BLE_TO_RAT) + 1);
    }

    ubCMConnInfo.ArrayOfConnInfo[i].currentChan = setNextDataChan(sessionId, chanSkip); //jump some [ms] channels into the future

    newSessionActive = TRUE;
    newSessionId = sessionId;

    // Allocate new element for sorted list
    keyHwi = Hwi_disable();
    elem = (cmListElem_t *)malloc(sizeof(cmListElem_t));
    Hwi_restore(keyHwi);

    if (elem == NULL)
    {
      return CM_FAILED_TO_START;
    }

    // Add connection to the sorted list
    elem->sessionId = sessionId;
    elem->startTime = ubCMConnInfo.ArrayOfConnInfo[sessionId-1].nextStartTime;
    addConnSortedList(elem);

    // If this is the first connection we are monitoring or if the start time of the nextSessionId is in the future (the RF is not busy)
    // Otherwise, the RF is busy, the monitoring of the new connection will start once the current RF command completes.
    if(ubCMConnInfo.numHandles == 0 || uble_timeCompare(ubCMConnInfo.ArrayOfConnInfo[nextSessionId-1].nextStartTime, currTime + processingTime))
    {
      // Start monitoring the new connection
      // If i+1 > CM_MAX_SESSIONS then we know this function failed. Let cmStart()
      // return a failure message since i will be an out of range index
      if (CM_SUCCESS != ubCM_start(sessionId))
      {
        return CM_FAILED_TO_START;
      }
    }
    return CM_SUCCESS;
  }
  return CM_FAILED_TO_START;
}

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
                      uint32_t crcInit, uint32_t reqTime)
{
    uint8_t connRoleBleCentral = BLE_ROLE_CENTRAL;

    return ubCM_InitCmSession(hostConnHandle, accessAddress,
                              connRoleBleCentral, connInterval,
                              hopValue, nextChan, chanMap,
                              crcInit, reqTime);
}

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
uint8_t ubCM_updateExt(uint8_t sessionId, uint8_t hostConnHandle, uint32_t accessAddress, uint16_t connInterval, uint8_t hopValue, uint8_t nextChan, uint8_t *chanMap, uint32_t crcInit)
{
  uint8_t i;

  i = sessionId-1; // index of session id is 1 less than acctual id

  if (ubCMConnInfo.ArrayOfConnInfo[i].connInterval != connInterval)
  {
    // The connection interval has changed, need to re-sync on the connection
    return CM_FAILED_TO_START;
  }

  // Process the new channel map
  memset(ubCMConnInfo.ArrayOfConnInfo[i].chanMap, 0, (sizeof(uint8_t) * CM_MAX_NUM_DATA_CHAN));
  processChanMap(sessionId, chanMap);

  return CM_SUCCESS;
}

/*********************************************************************
 * @fn      ubCM_callApp
 *
 * @brief   Calls the callback provided by the application
 *
 * @param   eventId - CM event
 * @param   data  - pointer to the data
 *
 * @return  status
 */
uint8_t ubCM_callApp(uint8_t eventId, uint8_t *data)
{
  cmEvt_t *cmEvt;
  volatile uint32_t keyHwi;

  keyHwi = Hwi_disable();
  cmEvt = (cmEvt_t *)malloc(sizeof(cmEvt_t));
  Hwi_restore(keyHwi);

  // Check if we could alloc message
  if (!cmEvt)
  {
    return FALSE;
  }

  cmEvt->event = eventId;
  cmEvt->pData = data;

  if (gAppCb != NULL)
  {
    gAppCb((uint8_t *)cmEvt);
  }
  else
  {
    // Program cannot run without a CB
    while(1);
  }

  return TRUE;
}
/*********************************************************************
 *********************************************************************/
