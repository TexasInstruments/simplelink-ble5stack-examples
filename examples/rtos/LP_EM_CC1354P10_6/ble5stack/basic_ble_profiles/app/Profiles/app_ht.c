/******************************************************************************

@file  app_ht.c

@brief This file contains the HT application functionality.

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
#include <health_thermometer_profile.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************
#define HT_CCC_UPDATE_INDICATION_ENABLED            1

// Example values of temperature measurement
#define HT_MEAS_EXAMPLE_TEMP                        37   // Example of temperature measurement value in Celsius units
#define HT_EXAMPLE_TIME_STAMP                       0x01
#define HT_TYPE_EXAMPLE_BODY                        0x02 // Represent the location of a temperature measurement
// Time Stamp example
#define HT_DEFAULT_TIME_BASE_DAY                    0x01
#define HT_DEFAULT_TIME_BASE_MONTH                  0x01
#define HT_DEFAULT_TIME_BASE_YEAR                   2023
#define HT_DEFAULT_TIME_BASE_HOUR                   0x01
#define HT_DEFAULT_TIME_BASE_MINUTE                 0x01
#define HT_DEFAULT_TIME_BASE_SECOND                 0x01

//*****************************************************************************
//! TYPEDEF
//*****************************************************************************


//*****************************************************************************
//!  LOCAL VARIABLES
//*****************************************************************************

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************
static void HT_measOnCccUpdateCB( uint16 connHandle, uint16 pValue );
static void HT_sendMeas( uint32 tempMeasur, uint8 type );

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
// Health Thermometer application callback function for incoming data
static HTP_cb_t ht_profileCB =
{
 HT_measOnCccUpdateCB,
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      HT_measOnCccUpdateCB
 *
 * @brief   Callback from HT profile indicating CCC update for Measurement characteristic
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to the CCC value
 *
 * @return  None
 */
static void HT_measOnCccUpdateCB( uint16 connHandle, uint16 pValue )
{
 if ( pValue == GATT_CLIENT_CFG_INDICATE )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "Health Thermometer app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indications enabled", connHandle );
    // Example of sending measurement
    HT_sendMeas( HT_MEAS_EXAMPLE_TEMP, HT_TYPE_EXAMPLE_BODY );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE2, 0,
                       "Health Thermometer app: Measurement characteristic - connectionHandle: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Indications disabled", connHandle );
  }
}

/*********************************************************************
 * @fn      HT_sendMeas
 *
 * @brief   This function sends HT measurement to any peer that connect and enable indications.
 *
 *
 * @param   tempMeasur - Temperature Measurement in Celsius
 * @param   type - type of measurement - represent to the location of a temperature measurement
 * @return  None
 */
static void HT_sendMeas( uint32 tempMeasur, uint8 type )
{
  bStatus_t status = SUCCESS;

  // Send HT measurement
  status = HTP_sendMeasurement( tempMeasur, HT_EXAMPLE_TIME_STAMP, type );
  if ( status == SUCCESS )
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Health Thermometer app: New measurement created - Temperature: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                       "Type of measurement: "
                       MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                       tempMeasur, type );
  }
  else
  {
    MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE1, 0,
                       "Health Thermometer app: Failed to add new measurement - error code: "
                       MENU_MODULE_COLOR_RED "%d " MENU_MODULE_COLOR_RESET,
                       status );
  }
}

/*********************************************************************
 * @fn      HT_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Health Thermometer profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t HT_start( void )
{
  bStatus_t status = SUCCESS;


  HTP_timeStamp htp_timeStamp;
  //time base example
  htp_timeStamp.year = HT_DEFAULT_TIME_BASE_YEAR;
  htp_timeStamp.month = HT_DEFAULT_TIME_BASE_MONTH;
  htp_timeStamp.day = HT_DEFAULT_TIME_BASE_DAY;
  htp_timeStamp.hours = HT_DEFAULT_TIME_BASE_HOUR;
  htp_timeStamp.minutes = HT_DEFAULT_TIME_BASE_MINUTE;
  htp_timeStamp.seconds = HT_DEFAULT_TIME_BASE_SECOND;

  // Initialize HT Profile
  status = HTP_start( &ht_profileCB, &htp_timeStamp);
  if( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  MenuModule_printf( APP_MENU_PROFILE_STATUS_LINE, 0, "HT start" );
  // Return status value
  return ( status );
}
