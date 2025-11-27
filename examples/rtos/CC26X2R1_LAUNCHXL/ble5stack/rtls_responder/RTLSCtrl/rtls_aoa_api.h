/******************************************************************************

 @file  rtls_aoa_api.h

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

/**
 *  @defgroup RTLSAoa_aoaAPI
 *  This module implements post processing for AoA
 *  @{
 *  @file  rtls_aoa_api.h
 *       RTLS Ctrl AoA API
 */

#ifndef RTLS_AOA_API_H_
#define RTLS_AOA_API_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**
 * @defgroup RTLSAoa_aoaStructs RTLS Ctrl AoA Structures
 * @{
 */
#define AOA_CONFIG_SAMPLING_CONTROL_RF_RAW_NO_FILTERING 0x01
#define IS_AOA_CONFIG_RF_RAW(sampleCtrl)                ((sampleCtrl) & AOA_CONFIG_SAMPLING_CONTROL_RF_RAW_NO_FILTERING)
#define AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_MASK       0x30
#define AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_1          0x10
#define AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_2          0x20
#define AOA_CONFIG_SAMPLING_CONTROL_BOTH_ANT_1_AND_2    0x00
#define IS_AOA_CONFIG_ONLY_ANT_1(sampleCtrl)            (((sampleCtrl) & AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_MASK) == AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_1)
#define IS_AOA_CONFIG_ONLY_ANT_2(sampleCtrl)            (((sampleCtrl) & AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_MASK) == AOA_CONFIG_SAMPLING_CONTROL_ONLY_ANT_2)

// List of AoA parameters
typedef struct __attribute__((packed))
{
  uint16_t connHandle;      // Connection handle
  uint8_t  slotDurations;   // 1us/2us sampling slots
  uint8_t  sampleRate;      // 1Mhz (BT5.1 spec), 2Mhz, 3Mhz or 4Mhz - this enables oversampling
  uint8_t  sampleSize;      // 8 bit sample (as defined by BT5.1 spec), 16 bit sample (higher accuracy)
  uint8_t  sampleCtrl;      // sample control flags 0x00-default filtering, 0x01-RAW_RF no filtering, , bit 4,5 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
  uint8_t  samplingEnable;  // 0 = mask CTE even if enabled, 1 = don't mask CTE, even if disabled (support Unrequested CTE)
  uint8_t  numAnt;          // Number of antennas in antenna pattern
  uint8_t  pAntPattern[];   // Antenna Id's in the pattern
} rtlsAoaConfigReq_t;

// AoA Enable command
typedef struct __attribute__((packed))
{
  uint16_t connHandle;   // Connection handle
  uint8_t  enableAoa;    // Enable or disable AoA
  uint16_t cteInterval;  // 0 = run once, > 0 = sample CTE every cteInterval until told otherwise
  uint8_t  cteLength;    // Length of the tone (2 - 20), used for AoA receiver
} rtlsAoaEnableReq_t;

// Connectionless AoA Enable command
typedef struct __attribute__((packed))
{
  uint16_t syncHandle;     // Sync handle
  uint8_t  enable;         // Enable or disable CL AoA
  uint8_t  slotDuration;   // Switching and sampling slots 0x01 for 1us, 0x02 for 2us
  uint8_t  sampleRate;     // 1Mhz (BT5.1 spec), 2Mhz, 3Mhz or 4Mhz - this enables oversampling
  uint8_t  sampleSize;     // 8 bit sample (as defined by BT5.1 spec), 16 bit sample (higher accuracy)
  uint8_t  sampleCtrl;     // sample control flags 0x00-default filtering, 0x01-RAW_RF no filtering, , bit 4,5 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
  uint8_t  maxSampleCte;   // 0x00 sample all available CTEs, 0x01-0x10 - max no of CTE to sample and report
  uint8_t  numAnt;         // The number of Antenna IDs in the pattern. Range: 0x02-0x4B.
  uint8_t  pAntPattern[];  // Antenna Id's in the pattern
} rtlsCLAoaEnableReq_t;


/** @} End RTLSAoa_aoaStructs */

/**
 * @brief RTLSAoa_processAoaResults
 *
 * Used to post process AoA results
 *
 * @param connHandle - connection handle
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
void RTLSAoa_processAoaResults(uint16_t connHandle, int8_t rssi, uint8_t channel, uint16_t numIqSamples, uint8_t sampleRate, uint8_t sampleSize, uint8_t sampleCtrl, uint8_t slotDuration, uint8_t numAnt, int8_t *pIQ);
#ifdef __cplusplus
}
#endif

#endif /* RTLS_AOA_API_H_ */

/** @} End RTLSAoa_aoaAPI */
