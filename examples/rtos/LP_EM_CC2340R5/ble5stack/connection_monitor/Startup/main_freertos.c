/******************************************************************************

 @file  main.c

 @brief main entry of the BLE stack sample application.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2023 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

/* RTOS header files */

#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC23X0.h>
#include <ti/drivers/GPIO.h>

#include <ti/display/Display.h>

#include <ti/devices/DeviceFamily.h>

#include <pthread.h>

#include "npi_data.h"

#include <hal_assert.h>
#include <bcomdef.h>
#include <string.h>

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include "rtls_ctrl_api.h"
#include "micro_ble_cm.h"

#include "micro_cm_app.h"
#include "rcl_settings_ble.h"


/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

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
extern assertCback_t halAssertCback;

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;

extern void NPIData_postAssertNpiMsgEvent(uint8_t assertType);

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
  halAssertCback = AssertHandler;

  Board_init();

  /* Create Application task. */
  MicroCmApp_init();
  MicroCmApp_stack_init();

  /* Kick off RTLS Control module - Priority 2 */
  rtlsConfiguration_t rtlsConfig;

  rtlsConfig.rtlsCapab = (rtlsCapabilities_e)(RTLS_CAP_RTLS_CONNECTION_MONITOR | RTLS_CAP_CM);

  // Device ID and revision
  rtlsConfig.devId = DeviceFamily_ID;
  rtlsConfig.revNum = RTLS_CTRL_REV;

  rtlsConfig.maxNumConns = CM_MAX_SESSIONS;

  // fetch BDADDR from FCFG
  memcpy( rtlsConfig.identifier, (uint8 *)fcfg->deviceInfo.bleAddr, CHIP_ID_SIZE );

  rtlsConfig.rtlsAppCb = MicroCmApp_rtlsCtrlMsgCb;

  RTLSCtrl_open(&rtlsConfig);

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();

  return 0;
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the callback handler for asserts raised in the stack or
 *              the application.
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
  // Free NPI task to send final NPI assert message
  NPIData_postAssertNpiMsgEvent(assertCause);

  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_ANY, NULL);
  }

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
      #if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
      #endif
      }
      else
      {
      #if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
      #endif
      }

      break;

    default:
    #if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
    #endif
      HAL_ASSERT_SPINLOCK;
  }

  return;
}

/*******************************************************************************
 */
