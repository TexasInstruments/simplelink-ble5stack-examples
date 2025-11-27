/******************************************************************************

@file  app_cgm.c

@brief This file contains the CGM application functionality.

Group: WCS, BTS
Target Device: cc13xx_cc26xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

#define CGM_MEAS_DEFAUALT_GLUC_CONC  50   // Default glucose concentration in mg/dL units

//*****************************************************************************
//! TYPEDEF
//*****************************************************************************

//*****************************************************************************
//!  LOCAL VARIABLES
//*****************************************************************************

// Initial time offset value
uint16 cgm_curTimeOffset = 0;
// CGM measurement interval in milliseconds
static uint16 cgm_measIntervalMsec = CGMS_MEAS_DEFAUALT_INTERVAL_MSEC;
// CGM measurement interval in minutes
static uint16 cgm_measIntervalMin = CGMS_MEAS_DEFAUALT_INTERVAL;
// CGM measurement clock
static Clock_Struct cgm_measClk;
// CGM session run time clock
static Clock_Struct cgm_srtClk;

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void CGM_measOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_racpOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_cgmcpOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void CGM_sstUpdateCB( CGMS_sst_t *pValue );
static void CGM_measClkTimeout( char *pData );
static void CGM_addMeas( void );
static void CGM_srtClkTimeout( char *pData );

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
 if ( pValue == GATT_CLIENT_CFG_NOTIFY )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Notifications enabled", connHandle );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Notifications disabled", connHandle );
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
  if ( pValue == GATT_CLIENT_CFG_INDICATE )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Record Access Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication enabled", connHandle );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Record Access Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication disabled", connHandle );
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
  if ( pValue == GATT_CLIENT_CFG_INDICATE )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Specific Ops Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication enabled", connHandle );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "CGM app: Specific Ops Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication disabled", connHandle );
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
  if ( pValue != NULL )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE3, 0,
                       "CGM app: Session start time (SST) value has been updated: " );

    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE4, 0,
                       "Year: %d, Month: %d, Day: %d, Hours: %d, Minutes: %d, Seconds: %d, "
                       "Time Zone: %d, DST Offset: %d",
                       pValue->year, pValue->month, pValue->day,
                       pValue->hours, pValue->minutes, pValue->seconds,
                       pValue->timeZone, pValue->dstOffset );
  }
}

/*********************************************************************
 * @fn      CGM_measClkTimeout
 *
 * @brief   This function is triggered when the measurement clock expires
 *
 * @param   pData - pointer to data
 *
 * @return  None
 */
static void CGM_measClkTimeout( char *pData )
{
  // Increase time offset value
  cgm_curTimeOffset++;

  // Add CGM measurement
  CGM_addMeas();
}

/*********************************************************************
 * @fn      CGM_addMeas
 *
 * @brief   This function adds CGM measurement to CGM data base
 *
 * @return  None
 */
static void CGM_addMeas( void )
{
  bStatus_t status = SUCCESS;
  uint16 glucoseConcen = CGM_MEAS_DEFAUALT_GLUC_CONC;  // Glucose concentration in mg/dL units

  // Add CGM measurement to data base
  status = CGMP_addMeaserment( glucoseConcen, cgm_curTimeOffset );
  if ( status == SUCCESS )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "CGM app: New measurement created - time offset: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Glucose concentration: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                       cgm_curTimeOffset, glucoseConcen );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "CGM app: Failed to add new measurement - error code: "
                       MENU_MODULE_COLOR_RED "%d " MENU_MODULE_COLOR_RESET,
                       status );
  }
}

/*********************************************************************
 * @fn      CGM_srtClkTimeout
 *
 * @brief   This function is triggered when the session run time clock expires
 *
 * @param   pData - pointer to data
 *
 * @return  None
 */
static void CGM_srtClkTimeout( char *pData )
{
  bStatus_t status = SUCCESS;

  // Update SRT parameter
  status = CGMP_updateSessionRunTime( CGMS_SRT_INTERVAL );

  if ( status != SUCCESS )
  {
    // If the run time ended, stop the SRT timer
    Util_stopClock( &cgm_srtClk );
  }
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

  // Add CGM measurement
  CGM_addMeas();

  // Create and start CGM session run time timer
  Util_constructClock( &cgm_srtClk, CGMP_clockCB,
                       CGMS_SRT_INTERVAL_MSEC, CGMS_SRT_INTERVAL_MSEC,
                       TRUE, (uint32)CGM_srtClkTimeout );

  // Create and start CGM measurement timer
  Util_constructClock( &cgm_measClk, CGMP_clockCB,
                       cgm_measIntervalMsec, cgm_measIntervalMsec,
                       TRUE, (uint32)CGM_measClkTimeout );

  MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE, 0,
                     "CGM start: Initial offset: "
                     MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                     "measurement interval (minutes): "
                     MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                     cgm_curTimeOffset, cgm_measIntervalMin );

  // Return status value
  return ( status );
}
