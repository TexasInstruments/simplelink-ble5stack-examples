/******************************************************************************

 @file  main_freertos.c

 @brief main entry of the BLE stack sample application.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2020-2024, Texas Instruments Incorporated
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

/*******************************************************************************
 * INCLUDES
 */

/* RTOS header files */
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>

#include <ti/drivers/Power.h>
#include <ti/display/Display.h>
#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#include "host_test_app.h"
#include "inc/npi_task.h"

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"

// BLE user defined configuration
#ifdef ICALL_JT
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#else
bleUserCfg_t user0Cfg = BLE_USER_CFG;
#endif

#endif // USE_DEFAULT_USER_CFG
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#ifdef ICALL_LITE
#include "icall_hci_tl.h"
#endif /* ICALL_LITE */

#ifndef HCI_TL_FULL
#error "The Host test app need to be build with HCI_TL_FULL"
#endif

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */
// All Assert are forwarded over the TL has a vendor specific event.
#define HOST_TEST_ASSERT_MESSAGE       0

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{
  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();

#ifdef ICALL_JT
  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = ICall_getTickPeriod();
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();
#endif

  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images */
  ICall_createRemoteTasks();

  /* Kick off application */
  HostTestApp_createTask();

  /* Kick off NPI */
  NPITask_createTask(ICALL_SERVICE_CLASS_BLE);

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();

  return 0;

}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack Wrapper
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // check the assert cause
  switch (assertCause)
  {
    // This assert is raised from the BLE Stack when a malloc failure occurs.
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
    {
      // ERROR: OUT OF MEMORY
      HAL_ASSERT_SPINLOCK;
      break;
    }

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
    {
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        // ERROR: INTERNAL FW ERROR
        HAL_ASSERT_SPINLOCK;
      }
      else
      {
        // ERROR: INTERNAL ERROR
        HAL_ASSERT_SPINLOCK;
      }
      break;
    }

    // An assert originating from an ICall failure.
    case HAL_ASSERT_CAUSE_ICALL_ABORT:
    {
      // ERROR: ICALL ABORT
      HAL_ASSERT_SPINLOCK;
      break;
    }

    case HAL_ASSERT_CAUSE_HARDWARE_ERROR:
    {
      if ( assertSubcause != HW_FAIL_START_EXT_ADV_ERROR )
      {
          HAL_ASSERT_SPINLOCK;
      }
    }
    default:
    {
     // ERROR: DEFAULT SPINLOCK
      HAL_ASSERT_SPINLOCK;
      break;
    }
  }

  return;
}

/*******************************************************************************
 */
