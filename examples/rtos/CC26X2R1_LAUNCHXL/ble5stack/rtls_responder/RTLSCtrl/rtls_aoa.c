/******************************************************************************

 @file  rtls_aoa.c

 @brief RTLS Services Manager

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2011-2025, Texas Instruments Incorporated
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

#include "rtls_aoa_api.h"
#include "rtls_ctrl_api.h"

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
 * LOCAL VARIABLES
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**
 * @brief RTLSAoa_processAoaResults
 *
 * Used to post process AoA results
 *
 * @param handle - connection/sync handle
 * @param rssi - rssi for this CTE
 * @param channel - channel this CTE was captured on
 * @param numIqSamples- Number of I/Q samples
 * @param sampleRate - Sampling rate that was used for the run
 * @param sampleSize - Amount of samples
 * @param sampleCtrl - RAW RF mode, 1 = RAW RF, 0 = Filtered (switching omitted)
 * @param slotDuration - Slot duration (1/2 us)
 * @param numAnt - Number of Antennas that were used for the run
 * @param pIQ - Pointer to IQ samples
 *
 * @return none
 */
void RTLSAoa_processAoaResults(uint16_t handle, int8_t rssi, uint8_t channel, uint16_t numIqSamples, uint8_t sampleRate, uint8_t sampleSize, uint8_t sampleCtrl, uint8_t slotDuration, uint8_t numAnt, int8_t *pIQ)
{
  RTLSCtrl_aoaResultEvt(handle, rssi, channel, numIqSamples, sampleRate, sampleSize, sampleCtrl, slotDuration, numAnt, pIQ);
}
