/******************************************************************************

@file  app_gl.c

@brief This file contains the GL application functionality.

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
#include <ti/bleapp/profiles/glucose/glucose_profile.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

#define GL_CCC_UPDATE_NOTIFICATION_ENABLED  1
#define GL_CCC_UPDATE_INDICATION_ENABLED    2

#define GL_MEAS_DEFAUALT_GLUC_CONC                  50   // Default glucose concentration in mg/dL units
#define GL_TYPE_DEFAUALT_CAPILLARY_WHOLE_BLOODCONC  0x01
#define GL_LOCATION_DEFAULT_FINGER                  0x01
#define GL_DEFAULT_TIME_OFFSET                      0x01
#define GL_DEFAULT_SENSOR_STATUS                    0x00

#define GL_DEFAULT_TIME_BASE_DAY                    0x01
#define GL_DEFAULT_TIME_BASE_MONTH                  0x01
#define GL_DEFAULT_TIME_BASE_YEAR                   2023
#define GL_DEFAULT_TIME_BASE_HOUR                   0x01
#define GL_DEFAULT_TIME_BASE_MINUTE                 0x01
#define GL_DEFAULT_TIME_BASE_SECOND                 0x01

#define GL_FEAT_DEFAULT_FOR_PTS                     0x08 // for PTS
//*****************************************************************************
//! TYPEDEF
//*****************************************************************************


//*****************************************************************************
//!  LOCAL VARIABLES
//*****************************************************************************

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void GL_measOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void GL_racpOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void GL_addMeas( uint16 glucoseConcen, uint8 type, uint8 location, uint16 sensorStatus );

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************

// Data Stream application callback function for incoming data
static GLP_cb_t gl_profileCB =
{
 GL_measOnCccUpdateCB,
 GL_racpOnCccUpdateCB,
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      GL_measOnCccUpdateCB
 *
 * @brief   Callback from GL profile indicating CCC update for Measurement characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void GL_measOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{
 if ( pValue == GATT_CLIENT_CFG_NOTIFY )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Glucose app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Notifications enabled", connHandle );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Glucose app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Notifications disabled", connHandle );
  }
}

/*********************************************************************
 * @fn      GL_racpOnCccUpdateCB
 *
 * @brief   Callback from GL profile indicating CCC update for
 *          Record Access Control Point characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void GL_racpOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{
  if ( pValue == GATT_CLIENT_CFG_INDICATE )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Glucose app: Record Access Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication enabled", connHandle );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Glucose app: Record Access Control Point - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indication disabled", connHandle );
  }
}

/*********************************************************************
 * @fn      GL_addMeas
 *
 * @brief   This function adds GL measurement to GL data base and send it to any peer that connect and enable notifications.
 *
 *
 * @param   glucoseConcen - glucose concentration measurement in mg/l
 * @param   type - type of measurement
 * @param   location - location of the taken measurement
 * @param   sensorStatus - sensor status information (depend on the feature values).
 * @return  None
 */
static void GL_addMeas( uint16 glucoseConcen, uint8 type, uint8 location, uint16 sensorStatus )
{
  bStatus_t status = SUCCESS;

  // Add GL measurement to data base
  status = GLP_addMeaserment( glucoseConcen, GL_DEFAULT_TIME_OFFSET, type, location, sensorStatus );
  if ( status == SUCCESS )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "Glucose app: New measurement created - time offset: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Glucose concentration: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                       GL_DEFAULT_TIME_OFFSET, glucoseConcen );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "Glucose app: Failed to add new measurement - error code: "
                       MENU_MODULE_COLOR_RED "%d " MENU_MODULE_COLOR_RESET,
                       status );
  }
}

/*********************************************************************
 * @fn      GL_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t GL_start( void )
{
  bStatus_t status = SUCCESS;
  //feature base example
  uint16 feat = GL_FEAT_DEFAULT_FOR_PTS; //for PTS - should decided by the User

  //time base example
  GLP_timeBase glp_timebase;
  glp_timebase.year = GL_DEFAULT_TIME_BASE_YEAR;
  glp_timebase.month = GL_DEFAULT_TIME_BASE_MONTH;
  glp_timebase.day = GL_DEFAULT_TIME_BASE_DAY;
  glp_timebase.hours = GL_DEFAULT_TIME_BASE_HOUR;
  glp_timebase.minutes = GL_DEFAULT_TIME_BASE_MINUTE;
  glp_timebase.seconds = GL_DEFAULT_TIME_BASE_SECOND;

  // Initialize GL Profile
  status = GLP_start( &gl_profileCB, feat, &glp_timebase);
  if( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }
  MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE, 0,
                     "GL start: Initial offset: "
                     MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                     status );
  // Example of adding measurment
  GL_addMeas(GL_MEAS_DEFAUALT_GLUC_CONC, GL_TYPE_DEFAUALT_CAPILLARY_WHOLE_BLOODCONC, GL_LOCATION_DEFAULT_FINGER, GL_DEFAULT_SENSOR_STATUS);
  // Return status value
  return ( status );
}
