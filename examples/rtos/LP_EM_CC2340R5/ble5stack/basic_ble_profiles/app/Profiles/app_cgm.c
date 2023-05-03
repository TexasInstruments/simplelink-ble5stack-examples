/******************************************************************************

@file  app_cgm.c

@brief This file contains the Data Stream application functionality.

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include "util.h"
#include <ti/bleapp/profiles/continuous_glucose_monitoring/cgm_profile.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

#define CGM_CCC_UPDATE_NOTIFICATION_ENABLED  1
#define CGM_CCC_UPDATE_INDICATION_ENABLED    2

#define CGM_MEAS_DEFAUALT_GLUC_CONC  50   // CGM Measurement interval in milliseconds

//*****************************************************************************
//! TYPEDEF
//*****************************************************************************

//*****************************************************************************
//!  LOCAL VARIABLES
//*****************************************************************************

// Initial value of time offset
uint16 cgm_curTimeOffset = 0;

// CGM measurement interval in milliseconds
static uint16 cgm_measIntervalMsec = CGM_MEAS_DEFAUALT_INTERVAL_MSEC;
// CGM measurement interval in minutes
static uint16 cgm_measIntervalMin = CGM_MEAS_DEFAUALT_INTERVAL;
// CGM measurement clock
static Clock_Struct cgm_measClk;

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************
static void CGM_measOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_racpOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_cgmcpOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_sstUpdateCB( CGMS_sst_t *pValue );
static void CGM_measClkTimeout(char *pData);

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************

// Data Stream application callback function for incoming data
static CGMP_cb_t cgm_profileCB =
{
 CGM_measOnCccUpdateCB,
 CGM_racpOnCccUpdateCB,
 CGM_cgmcpOnCccUpdateCB,
 CGM_sstUpdateCB,
};

//*****************************************************************************
//! Functions
//*****************************************************************************
/*********************************************************************
 * @fn      CGM_measOnCccUpdateCB
 *
 * @brief   Callback from CGM profile indicating CCC update for Measurement characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void CGM_measOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{

 if ( pValue == GATT_CLIENT_CFG_NOTIFY)
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Measurement characteristic - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Notifications enabled", connHandle);
  }
  else
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Measurement characteristic - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Notifications disabled", connHandle);
  }
}

/*********************************************************************
 * @fn      CGM_racpOnCccUpdateCB
 *
 * @brief   Callback from CGM profile indicating CCC update for
 *          Record Access Control Point characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void CGM_racpOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{

 if ( pValue == GATT_CLIENT_CFG_INDICATE)
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Record Access Control Point - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Indication enabled", connHandle);
  }
  else
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Record Access Control Point - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Indication disabled", connHandle);
  }
}

/*********************************************************************
 * @fn      CGM_cgmcpOnCccUpdateCB
 *
 * @brief   Callback from CGM profile indicating CCC update for CGM
 *          Specific Ops Control Point characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void CGM_cgmcpOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{

 if ( pValue == GATT_CLIENT_CFG_INDICATE)
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Specific Ops Control Point - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Indication enabled", connHandle);
  }
  else
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0,
                      "CGM app: Specific Ops Control Point - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Indication disabled", connHandle);
  }
}

/*********************************************************************
 * @fn      CGM_sstUpdateCB
 *
 * @brief   Callback from CGM profile indicating session start time (SST) value
 *          has been updated
 *
 * @param   pValue - pointer to the SST value
 *
 * @return  None
 */
static void CGM_sstUpdateCB( CGMS_sst_t *pValue )
{
  // Verify input parameters
  if ( pValue != NULL)
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE3, 0,
                      "CGM app: Session start time (SST) value has been updated: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Year: %d, Month: %d, Day: %d, Hours: %d, Minutes: %d, Seconds: %d, "
                      "Time Zone: %d, DST Offset: %d",
                      pValue->year, pValue->month, pValue->day,
                      pValue->hours, pValue->minutes, pValue->seconds,
                      pValue->timeZone, pValue->dstOffset);
  }

  return;
}

/*********************************************************************
 * @fn      CGM_measClkTimeout
 *
 * @brief   This function is triggered when the clock expires
 *
 * @param   pData - pointer to data
 *
 * @return  SUCCESS or stack call status
 */
static void CGM_measClkTimeout( char *pData)
{
  bStatus_t status = SUCCESS;
  uint16 glucoseConcen = CGM_MEAS_DEFAUALT_GLUC_CONC;

  // Add CGM measurement to CGM data base
  status = CGMP_addMeaserment( glucoseConcen, cgm_curTimeOffset );
  if ( status == SUCCESS )
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                      "CGM app: New measurement created - time offset: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Glucose concentration: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                      cgm_curTimeOffset, glucoseConcen);
  }
  else
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                      "CGM app: Failed to add new measurement - error code: "
                      MENU_MODULE_COLOR_RED "%d " MENU_MODULE_COLOR_RESET,
                      status);
  }

  // Increase time offset
  cgm_curTimeOffset += cgm_measIntervalMin;
}

/*********************************************************************
 * @fn      CGM_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t CGM_start( void )
{
  bStatus_t status = SUCCESS;
  CGMS_feat_t feat = {0};

  // Init CGM Feature characteristic parameters
  feat.typeLoc = BUILD_UINT8( CGMS_FEAT_LOC_FINGER, CGMS_FEAT_TYPE_CAP_BLOOD);
  feat.e2eCrc = CGMS_FEAT_E2E_NOT_SUPPORTED;

  // Initialize CGM Profile
  status = CGMP_start( &cgm_profileCB, feat, cgm_curTimeOffset );
  if( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  // Create and start CGM activity timer
  Util_constructClock(&cgm_measClk, (void *)BLEAppUtil_invokeFunctionNoData,
                      cgm_measIntervalMsec, cgm_measIntervalMsec,
                      TRUE, (uint32)CGM_measClkTimeout);

  MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                    "CGM start: Initial offset: "
                    MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                    "measurement interval (minutes): "
                    MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                    cgm_curTimeOffset, cgm_measIntervalMin);

  // Return status value
  return ( status );
}
