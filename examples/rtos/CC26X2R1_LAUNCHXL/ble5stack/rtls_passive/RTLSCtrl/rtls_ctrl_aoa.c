/******************************************************************************

 @file  rtls_ctrl_aoa.c

 @brief This file contains the functions and structures specific to AoA
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

/*********************************************************************
 * INCLUDES
 */

#include <stdint.h>
#include <stdlib.h>

#include "rtls_ctrl_aoa.h"
#include "rtls_host.h"
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <string.h>

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * LOCAL VARIABLES
 */

AoA_controlBlock_t gAoaCb = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
* @fn      RTLSCtrl_postProcessAoa
*
* @brief   Called at the end of each connection event to extract I/Q samples
*
* @param   aoaControlBlock - AoA information saved by RTLS Control
* @param   rssi - rssi to be reported to RTLS Host
* @param   channel - channel that was used for this AoA run
* @param   sampleCtrl - sample control configs: 0x01 = RAW RF, 0x00 = Filtered results (switching period omitted), bit 4,5 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
*
* @return  none
*/

void RTLSCtrl_postProcessAoa(rtlsAoaIqEvt_t *pEvt)
{
  uint8_t antenna=0;

  uint16_t handle = pEvt->handle;
  int8_t rssi = pEvt->rssi;
  uint8_t channel = pEvt->channel;

  switch (gAoaCb.resultMode)
  {
    case AOA_MODE_RAW:
    {
      rtlsAoaResultRaw_t *aoaResult;
      uint16_t samplesToOutput;

      // The samples may be of either type, depending on sampleSize
      // For the sake of simplicity, just set both to point to the samples and decide the output format later
      AoA_IQSample_Ext_t *pIterExt = (AoA_IQSample_Ext_t *)pEvt->pIQ;
      AoA_IQSample_t *pIter = (AoA_IQSample_t *)pEvt->pIQ;

      // Allocate result structure to consider both options of sampleSize
      aoaResult = RTLSCtrl_malloc(sizeof(rtlsAoaResultRaw_t) + (MAX_SAMPLES_SINGLE_CHUNK * sizeof(AoA_IQSample_Ext_t)));

      // Sanity check
      if (aoaResult == NULL)
      {
        return;
      }

      aoaResult->samplesLength = pEvt->numIqSamples;

      // Set various parameters
      aoaResult->handle = handle;
      aoaResult->channel = channel;
      aoaResult->rssi = rssi;
      aoaResult->antenna = antenna;

      // Set offset of the result set within the total bulk of samples
      aoaResult->offset = 0;

      do
      {
        // If the remainder is larger than buff size, tx maximum buff size
        if (aoaResult->samplesLength - aoaResult->offset > MAX_SAMPLES_SINGLE_CHUNK)
        {
          samplesToOutput = MAX_SAMPLES_SINGLE_CHUNK;
        }
        else
        {
          // If not, then output the remaining data
          samplesToOutput = aoaResult->samplesLength - aoaResult->offset;
        }

        // Copy the samples to output buffer
        for (int i = 0; i < samplesToOutput; i++)
        {
            if (pEvt->sampleSize == 2)
            {
              aoaResult->samples[i].i = pIterExt[i].i;
              aoaResult->samples[i].q = pIterExt[i].q;
            }
            else // sampleSize = 1
            {
              aoaResult->samples[i].i = pIter[i].i;
              aoaResult->samples[i].q = pIter[i].q;
            }
        }


        if( (handle & SYNC_HANDLE_MASK) == 0 )
        {
          RTLSHost_sendMsg(RTLS_CMD_AOA_RESULT_RAW, HOST_ASYNC_RSP, (uint8_t *)aoaResult, sizeof(rtlsAoaResultRaw_t) + (sizeof(AoA_IQSample_Ext_t) * samplesToOutput));
        }
        else
        {
          aoaResult->handle &= REVERSE_SYNC_HANDLE;
          RTLSHost_sendMsg(RTLS_CMD_CL_AOA_RESULT_RAW, HOST_ASYNC_RSP, (uint8_t *)aoaResult, sizeof(rtlsAoaResultRaw_t) + (sizeof(AoA_IQSample_Ext_t) * samplesToOutput));
        }
        // Update offset
        aoaResult->offset += samplesToOutput;
        pIter += MAX_SAMPLES_SINGLE_CHUNK;
        pIterExt += MAX_SAMPLES_SINGLE_CHUNK;
      }
      while (aoaResult->offset < aoaResult->samplesLength);

      if (aoaResult)
      {
        RTLSUTIL_FREE(aoaResult);
      }
    }
    break;

    default:
      break;
  } // Switch
}

/*********************************************************************
* RTLSCtrl_clInitAoa
*
* Initialize connectionless AoA - has to be called before running AoA
*
* @param   resultMode  - AOA_MODE_RAW
* @param   sampleCtrl  - sample control configs: 0x01 = RAW RF, 0x00 = Filtered results (switching period omitted),
*                        bit 4,5 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
* @param   numAnt      - Number of items in Antenna array
* @param   pAntPattern - Pointer to Antenna array
* @param   syncHandle  - Handle identifying the periodic advertising train
*
* @return  status - RTLS_CONFIG_NOT_SUPPORTED/RTLS_SUCCESS
*/
rtlsStatus_e RTLSCtrl_clInitAoa(aoaResultMode_e resultMode, uint8_t sampleCtrl, uint8_t numAnt, uint8_t *pAntPattern, uint16_t syncHandle)
{
  if ( resultMode != AOA_MODE_RAW )
  {
    return RTLS_CONFIG_NOT_SUPPORTED;
  }
  // Save the result mode
  gAoaCb.resultMode = resultMode;

   // Save sampleCtrl flags
  gAoaCb.sampleCtrl = sampleCtrl;

  return RTLS_SUCCESS;
}

/*********************************************************************
* @fn      RTLSCtrl_initAoa
*
* @brief   Initialize AoA - has to be called before running AoA
*
* @param   sampleCtrl - sample control configs: 0x01 = RAW RF, 0x00 = Filtered results (switching period omitted), bit 4,5 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
* @param   maxConnections - number of connections we need to keep results for
* @param   numAnt - number of antennas in pAntPattern
* @param   pAntPattern - antenna pattern provided by the user
* @param   resultMode - AOA_MODE_RAW
*
* @return  status - RTLS_CONFIG_NOT_SUPPORTED/RTLS_SUCCESS
*/
rtlsStatus_e RTLSCtrl_initAoa(uint8_t maxConnections, uint8_t sampleCtrl, uint8_t numAnt, uint8_t *pAntPattern, aoaResultMode_e resultMode)
{
  if ( resultMode != AOA_MODE_RAW )
  {
    return RTLS_CONFIG_NOT_SUPPORTED;
  }

  // Set result mode
  gAoaCb.resultMode = resultMode;

  // Save sampleCtrl flags
  gAoaCb.sampleCtrl = sampleCtrl;

  return RTLS_SUCCESS;
}
