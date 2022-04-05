/******************************************************************************

 @file       main.c

 @brief main entry of the BLE stack sample application.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_02_20_00_49
 Release Date: 2018-07-16 13:19:56
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/display/Display.h>

#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "tida_01624.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_prcm.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
#ifdef ICALL_JT
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#else  /* ! ICALL_JT */
bleUserCfg_t user0Cfg = BLE_USER_CFG;
#endif /* ICALL_JT */

#endif // USE_DEFAULT_USER_CFG

#ifdef USE_FPGA
#include <inc/hw_prcm.h>
#endif // USE_FPGA

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#if defined( USE_FPGA )
  #define RFC_MODE_BLE                 PRCM_RFCMODESEL_CURR_MODE1
  #define RFC_MODE_ANT                 PRCM_RFCMODESEL_CURR_MODE4
  #define RFC_MODE_EVERYTHING_BUT_ANT  PRCM_RFCMODESEL_CURR_MODE5
  #define RFC_MODE_EVERYTHING          PRCM_RFCMODESEL_CURR_MODE6
  //
  #define SET_RFC_BLE_MODE(mode) HWREG( PRCM_BASE + PRCM_O_RFCMODESEL ) = (mode)
#endif // USE_FPGA

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
/* Task and tast stack */
Task_Struct myTask;
Char myTaskStack[512];

/* Pin driver handles */
static PIN_Handle tmpPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State tmpPinState;

/* Wake-up Button pin table */
PIN_Config ButtonTableWakeUp[] = {
    Board_PIN_BUTTON0     | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE                                 /* Terminate list */
};

#ifdef CC1350_LAUNCHXL
#ifdef POWER_SAVING
// Power Notify Object for wake-up callbacks
Power_NotifyObj rFSwitchPowerNotifyObj;
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg);
#endif //POWER_SAVING

PIN_State  radCtrlState;
PIN_Config radCtrlCfg[] =
{
  Board_DIO1_RFSW   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* RF SW Switch defaults to 2.4GHz path*/
  Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Power to the RF Switch */
  PIN_TERMINATE
};
PIN_Handle radCtrlHandle;
#endif //CC1350_LAUNCHXL

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;

/*!*****************************************************************************
 *  @brief      Task which runs when the device is powered up for the first time
 *
 *  @param      UArg a0 : User argument that can be passed to the task
 *
 *  @param      UArg a1 : User argument that can be passed to the task
 *
 *  @return     none - Should never return
 ******************************************************************************/
static void taskFxn(UArg a0, UArg a1)
{
    /* Configure DIO for wake up from shutdown */
    PINCC26XX_setWakeup(ButtonTableWakeUp);

    /* Go to shutdown */
    Power_shutdown(0, 0);

    /* Should never get here, since shutdown will reset. */
    while (1);
}

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
uint32_t rSrc;

int main()
{

#if defined( USE_FPGA )
  HWREG(PRCM_BASE + PRCM_O_PDCTL0) &= ~PRCM_PDCTL0_RFC_ON;
  HWREG(PRCM_BASE + PRCM_O_PDCTL1) &= ~PRCM_PDCTL1_RFC_ON;
#endif // USE_FPGA

  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();

  /* Get the reason for reset */
  rSrc = SysCtrlResetSourceGet();

  /* If not coming from shutdown, use special gpio table.*/
  if (rSrc == RSTSRC_WAKEUP_FROM_SHUTDOWN)
  {
    tmpPinHandle = PIN_open(&tmpPinState, BoardGpioInitTable);

    PIN_setOutputValue(tmpPinHandle, Board_DIO0, 1);

    #ifdef CC1350_LAUNCHXL
      // Enable 2.4GHz Radio
      radCtrlHandle = PIN_open(&radCtrlState, radCtrlCfg);

    #ifdef POWER_SAVING
      Power_registerNotify(&rFSwitchPowerNotifyObj,
                           PowerCC26XX_ENTERING_STANDBY | PowerCC26XX_AWAKE_STANDBY,
                           (Power_NotifyFxn) rFSwitchNotifyCb, NULL);
    #endif //POWER_SAVING
    #endif //CC1350_LAUNCHXL

    #if defined( USE_FPGA )
      // set RFC mode to support BLE
      // Note: This must be done before the RF Core is released from reset!
      SET_RFC_BLE_MODE(RFC_MODE_BLE);
    #endif // USE_FPGA

    #ifdef CACHE_AS_RAM
      // retain cache during standby
      Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
      Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    #else
      // Enable iCache prefetching
      VIMSConfigure(VIMS_BASE, TRUE, TRUE);
      // Enable cache
      VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
    #endif //CACHE_AS_RAM

    #if !defined( POWER_SAVING ) || defined( USE_FPGA )
      /* Set constraints for Standby, powerdown and idle mode */
      // PowerCC26XX_SB_DISALLOW may be redundant
      Power_setConstraint(PowerCC26XX_SB_DISALLOW);
      Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
    #endif // POWER_SAVING | USE_FPGA

    #ifdef ICALL_JT
      /* Update User Configuration of the stack */
      user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
      user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();
    #endif  /* ICALL_JT */
      /* Initialize ICall module */
      ICall_init();

      /* Start tasks of external images - Priority 5 */
      ICall_createRemoteTasks();

      /* Kick off profile - Priority 3 */
      GAPRole_createTask();

      SimplePeripheral_createTask();
  }
  else
  {
      Task_Params taskParams;

      /* Configure task. */
      Task_Params_init(&taskParams);
      taskParams.stack = myTaskStack;
      taskParams.stackSize = sizeof(myTaskStack);
      Task_construct(&myTask, taskFxn, &taskParams, NULL);
  }

  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return 0;
}


/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
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
#if !defined(Display_DISABLE_ALL)
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_LCD, NULL);
  }

  Display_print0(dispHandle, 0, 0, ">>>STACK ASSERT");
#endif // ! Display_DISABLE_ALL

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> OUT OF MEMORY!");
#endif // ! Display_DISABLE_ALL
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
#if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
#endif // ! Display_DISABLE_ALL
      }
      else
      {
#if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
#endif // ! Display_DISABLE_ALL
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL ABORT!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL TIMEOUT!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> WRONG API CALL!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

  default:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

#if defined (CC1350_LAUNCHXL) && defined (POWER_SAVING)
/*******************************************************************************
 * @fn          rFSwitchNotifyCb
 *
 * @brief       Power driver callback to toggle RF switch on Power state
 *              transitions.
 *
 * input parameters
 *
 * @param   eventType - The state change.
 * @param   eventArg  - Not used.
 * @param   clientArg - Not used.
 *
 * @return  Power_NOTIFYDONE to indicate success.
 */
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg)
{
  if (eventType == PowerCC26XX_ENTERING_STANDBY)
  {
    // Power down RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 0);
  }
  else if (eventType == PowerCC26XX_AWAKE_STANDBY)
  {
    // Power up RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 1);
  }

  // Notification handled successfully
  return Power_NOTIFYDONE;
}
#endif //CC1350_LAUNCHXL || POWER_SAVING


/*******************************************************************************
 */
