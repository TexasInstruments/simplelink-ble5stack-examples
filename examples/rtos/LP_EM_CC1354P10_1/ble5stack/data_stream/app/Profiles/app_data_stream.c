/******************************************************************************

@file  app_data.c

@brief This file contains the Data Stream application functionality.

Group: WCS, BTS
Target Device: cc13xx_cc26xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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
#include <ti/drivers/GPIO.h>
#include <ti/bleapp/profiles/data_stream/data_stream_profile.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <time.h>

//*****************************************************************************
//! Defines
//*****************************************************************************
#define DS_CCC_UPDATE_NOTIFICATION_ENABLED  1

//*****************************************************************************
//! Globals
//*****************************************************************************

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void DS_onCccUpdateCB( uint16 connHandle, uint16 pValue );
static void DS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len );

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
// Data Stream application callback function for incoming data
static DSP_cb_t ds_profileCB =
{
  DS_onCccUpdateCB,
  DS_incomingDataCB
};

//*****************************************************************************
//! Functions
//*****************************************************************************
/*********************************************************************
 * @fn      DS_onCccUpdateCB
 *
 * @brief   Callback from Data_Stream_Profile indicating ccc update
 *
 * @param   cccUpdate - pointer to data structure used to store ccc update
 *
 * @return  SUCCESS or stack call status
 */
static void DS_onCccUpdateCB( uint16 connHandle, uint16 pValue )
{
  if ( pValue == DS_CCC_UPDATE_NOTIFICATION_ENABLED)
  {
    Display_printf( dispHandle, dispIndex, 0,
                    "#%5d    DataStream_cccUpdate: connectionHandle:%d Notifications enabled",
                    dispIndex, connHandle ); dispIndex++;
  }
  else
  {
    Display_printf( dispHandle, dispIndex, 0,
                    "#%5d    DataStream_cccUpdate: connectionHandle:%d Notifications disabled ",
                    dispIndex, connHandle ); dispIndex++;
  }
}

/*********************************************************************
 * @fn      DS_incomingDataCB
 *
 * @brief   Callback from Data_Stream_Profile indicating incoming data
 *
 * @param   dataIn - pointer to data structure used to store incoming data
 *
 * @return  SUCCESS or stack call status
 */
static void DS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len )
{
  bStatus_t status = SUCCESS;
  char dataOut[] = "Data size is too long";
  char printData[len+1];
  uint16 i = 0;

  // Toggle LEDs to indicate that data was received
  GPIO_toggle( CONFIG_GPIO_LED_RED );
  GPIO_toggle( CONFIG_GPIO_LED_GREEN );

  // The incoming data length was too large
  if ( len == 0 )
  {
    Display_printf( dispHandle, dispIndex, 0,
                   "#%5d    DataStream_IncomingData: connectionHandle %d: Error: %s ",
                   dispIndex, connHandle, dataOut ); dispIndex++;

    // Send error message over GATT notification
    status = DSP_sendData( (uint8 *)dataOut, sizeof( dataOut ) );
  }

  // New data received from peer device
  else
  {
    // Copy the incoming data to buffer before printing it
    memcpy (printData, pValue, len );
    printData[len] ='\0';

    // Print the incoming data
    Display_printf( dispHandle, dispIndex, 0,
                    "#%5d    DataStream_IncomingData: connectionHandle %d: length: %d data: %s",
                    dispIndex, connHandle, len, printData ); dispIndex++;

    // Change upper case to lower case and lower case to upper case
    for ( i = 0; i < len; i++ )
    {
      if ( pValue[i] >= 'a' && pValue[i] <= 'z' )
      {
        pValue[i] = pValue[i] - 32;
      }
      else if ( pValue[i] >= 'A' && pValue[i] <= 'Z' )
      {
        pValue[i] = pValue[i] + 32;
      }
    }

    // Echo the incoming data over GATT notification
    status = DSP_sendData( (uint8 *)pValue, len );
    if ( status == SUCCESS )
    {
      // Copy the changed data to buffer before printing it
      memcpy (printData, pValue, len );

      // Print the echo data
      Display_printf( dispHandle, dispIndex, 0,
                      "#%5d    DataStream_OutgoingData: connectionHandle %d: Echo data: %s",
                      dispIndex, connHandle, printData ); dispIndex++;
    }
    else
    {
      // Print error message
      Display_printf( dispHandle, dispIndex, 0,
                      "#%5d    DataStream_Pofile_SendData: status %d",
                      dispIndex, status ); dispIndex++;
    }
  }
}

/*********************************************************************
 * @fn      DataStream_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DataStream_start( void )
{
  bStatus_t status = SUCCESS;

  Display_printf( dispHandle, dispIndex, 0,
                  "#%5d    DataStream_start: Add Services",
                  dispIndex ); dispIndex++;

  status = DSP_start( &ds_profileCB );
  if( status != SUCCESS )
  {
    // Return status value
    return status;
  }

  // Set LEDs
  GPIO_write( CONFIG_GPIO_LED_RED, CONFIG_LED_OFF );
  GPIO_write( CONFIG_GPIO_LED_GREEN, CONFIG_LED_ON );

  return ( SUCCESS );
}
