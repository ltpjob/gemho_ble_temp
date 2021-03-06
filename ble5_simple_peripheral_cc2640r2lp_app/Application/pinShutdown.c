//*****************************************************************************
//! @file       pinShutdown.c
//! @brief      Pin shutdown example.
//!
//  Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/

//******************************************************************************
//! The example will cycle the device between active mode and shutdown mode.
//! The cycling is controlled by pin interrupts, i.e. buttons.
//!
//! The mapping is:
//!             CC2640R2_LAUNCHXL_PIN_BTN2 will wake up the chip to active mode.
//!             CC2640R2_LAUNCHXL_PIN_BTN3 will put the device in shutdown mode.
//!
//! The example is using a simple debounce logic, so it does not support
//! multiple buttons pushed at the same time.
//!
//! The debounce work like this: When the pin interrupt is triggered, a button
//! clock is configured to trigger 50 ms ahead in time. After 50 ms, when the
//! clock interrupt is triggered, the LED connected to the active/pushed button
//! will be inverted if the button is still pushed.
//*****************************************************************************/

//******************************************************************************
// Includes
//*****************************************************************************/
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include "Board.h"
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/devices/DeviceFamily.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include <ti/drivers/ADC.h>
#include "VDDS_process.h"
#include <driverlib/aon_batmon.h>
#include DeviceFamily_constructPath(inc/hw_prcm.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))


//******************************************************************************
// Globals
//*****************************************************************************/
/* Task and tast stack */
Task_Struct myTask;
Char myTaskStack[512];

Task_Struct ledTask;
volatile bool is_ledTaskQuit;
Char ledTaskStack[512];

/* Semaphore used to gate for shutdown */
Semaphore_Struct shutdownSem;

/* Clock used for debounce logic */
Clock_Struct buttonClock;
Clock_Handle hButtonClock;

/* Pin driver handles */
PIN_Handle hPins;
PIN_Handle hButtons;

/* LED pin state */
PIN_State LedPinState;

/* Button pin state */
PIN_State buttonState;

/* Flag to store whether we woke up from shutdown or not */
bool isWakingFromShutdown;

/* PIN_Id for active button (in debounce period) */
PIN_Id activeButtonPinId;

/* Led pin table used when waking from reset*/
PIN_Config LedPinTable[] = {
    Board_LED0    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    IOID_11       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    Board_BUTTON2 | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN | PIN_NOPULL | PIN_HYSTERESIS,
    Board_BUTTON1 | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN | PIN_NOPULL | PIN_HYSTERESIS,
    PIN_TERMINATE                                                    /* Terminate list */
};


/* Wake-up Button pin table */
PIN_Config ButtonTableWakeUp[] = {
    Board_BUTTON0     | PIN_GPIO_OUTPUT_DIS |PIN_INPUT_EN | PIN_NOPULL | PINCC26XX_WAKEUP_NEGEDGE,
    Board_BUTTON1     | PIN_GPIO_OUTPUT_DIS |PIN_INPUT_EN | PIN_NOPULL | PINCC26XX_WAKEUP_POSEDGE,
    PIN_TERMINATE                                 /* Terminate list */
};

/* Shutdown Button pin table */
PIN_Config ButtonTableShutdown[] = {
    Board_BUTTON0   | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE,
//    Board_BUTTON1   | PIN_INPUT_EN | PIN_NOPULL,
    PIN_TERMINATE                                 /* Terminate list */
};

/*!*****************************************************************************
 *  @brief      Button clock callback
 *
 *  Called when the debounce periode is over. Stopping the clock, toggling
 *  the device mode based on activeButtonPinId:
 *
 *              CC2640R2_LAUNCHXL_PIN_BTN3 will put the device in shutdown mode.
 *
 *  Reenabling the interrupts and resetting the activeButtonPinId.
 *
 *  @param      arg  argument (PIN_Handle) connected to the callback
 *
 ******************************************************************************/
static void buttonClockCb(UArg arg) {
    PIN_Handle buttonHandle = (PIN_State *) arg;

    /* Stop the button clock */
    Clock_stop(hButtonClock);

    /* Check that there is active button for debounce logic*/
    if (activeButtonPinId != PIN_TERMINATE) {
        /* Debounce logic, only toggle if the button is still pushed (low) */
        if (!PIN_getInputValue(activeButtonPinId)) {
            /* Toggle LED based on the button pressed */
            switch (activeButtonPinId) {
            case Board_BUTTON0:
                Semaphore_post(Semaphore_handle(&shutdownSem));
                break;
            default:
                /* Do nothing */
                break;
            }
        }
    }

    /* Re-enable interrupts to detect button release. */
    PIN_setConfig(buttonHandle, PIN_BM_IRQ, activeButtonPinId | PIN_IRQ_NEGEDGE);

    /* Set activeButtonPinId to none... */
    activeButtonPinId = PIN_TERMINATE;
}

/*!*****************************************************************************
 *  @brief      Button callback
 *
 *  Initiates the debounce period by disabling interrupts, setting a timeout
 *  for the button clock callback and starting the button clock.
 *  Sets the activeButtonPinId.
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void buttonCb(PIN_Handle handle, PIN_Id pinId) {
    /* Set current pinId to active */
    activeButtonPinId = pinId;

    /* Disable interrupts during debounce */
    PIN_setConfig(handle, PIN_BM_IRQ, activeButtonPinId | PIN_IRQ_DIS);

    /* Set timeout 50 ms from now and re-start clock */
    Clock_setTimeout(hButtonClock, (50 * (1000 / Clock_tickPeriod)));
    Clock_start(hButtonClock);
}

static double measure_VDDS()
{
    double adcVDDSMicroVolt = 0;
    uint32_t percent = 0;

    percent = AONBatMonBatteryVoltageGet();

    percent = (percent * 125) >> 5;

    adcVDDSMicroVolt = percent/1000.0 + 0.4;

    return adcVDDSMicroVolt;
}

/*!*****************************************************************************
 *  @brief      Task which runs when the device is active
 *
 *  @param      UArg a0 : User argument that can be passed to the task
 *
 *  @param      UArg a1 : User argument that can be passed to the task
 *
 *  @return     none - Should never return
 ******************************************************************************/
static void taskFxn(UArg a0, UArg a1)
{
    while(1)
    {
        /* Pend on semaphore before going to shutdown */
        Semaphore_pend(Semaphore_handle(&shutdownSem), BIOS_WAIT_FOREVER);

        if(PIN_getInputValue(Board_BUTTON1)==1)
        {
            continue;
        }

        /* Turn off LED0 */
        is_ledTaskQuit = 1;
        PIN_setOutputValue(hPins, Board_LED0, 0);

        DELAY_MS(650);

        /* Configure DIO for wake up from shutdown */
        PINCC26XX_setWakeup(ButtonTableWakeUp);

        /* Go to shutdown */
        Power_shutdown(0, 0);

        /* Should never get here, since shutdown will reset. */
        while (1);
    }
}

static void taskFxn_led(UArg a0, UArg a1)
{
    hPins = PIN_open(&LedPinState, LedPinTable);
    uint32 tick1, tick2;
    int first_check = 1;

    tick1 = Clock_getTicks();

    while (is_ledTaskQuit == 0)
    {
        //if not charging
        if(PIN_getInputValue(Board_BUTTON1) != 1)
        {
            tick2 = Clock_getTicks();

            if((tick2-tick1)*Clock_tickPeriod/1000000 >= 120 || first_check == 1)
            {
                double po_volt = 3.30;
                if(first_check == 1)
                    po_volt += 0.05;

                first_check = 0;
                DELAY_MS(20);
                double adcVDDSMicroVolt = measure_VDDS();
                set_mem_vdds(adcVDDSMicroVolt);
                if(adcVDDSMicroVolt <= po_volt)
                {
                    PIN_setOutputValue(hPins, Board_LED0, 0);

                    /* Configure DIO for wake up from shutdown */
                    PINCC26XX_setWakeup(ButtonTableWakeUp);

                    /* Go to shutdown */
                    Power_shutdown(0, 0);

                    /* Should never get here, since shutdown will reset. */
                    while (1);
                }

                tick1 = Clock_getTicks();
            }
        }

        PIN_setOutputValue(hPins, Board_LED0, 1);
        DELAY_MS(1000);


        //Board_BUTTON2 chg      Board_BUTTON1 vcc
        if(linkDB_NumActive() >0 && PIN_getInputValue(Board_BUTTON1)!=1)
        {
            continue;
        }
        else if(PIN_getInputValue(Board_BUTTON2)==1 && PIN_getInputValue(Board_BUTTON1)==1)
        {
            continue;
        }
        else
        {
            PIN_setOutputValue(hPins, Board_LED0, 0);
            DELAY_MS(1000);
        }

        //Board_BUTTON2 chg      Board_BUTTON1 vcc
        if(PIN_getInputValue(Board_BUTTON2)==0 && PIN_getInputValue(Board_BUTTON1)==1)
        {
            DELAY_MS(2000);
        }



    }
}

/*!*****************************************************************************
 *  @brief      Application main entry point
 *
 *  @param      none
 *
 *  @return     int - Should never return
 ******************************************************************************/
int board_initPinShutdown(void)
{
     /* Locals */
    Task_Params taskParams;
    Semaphore_Params semParams;

    /* Call driver init functions */
//    Board_initGeneral();

    /* Get the reason for reset */
    uint32_t rSrc = SysCtrlResetSourceGet();

    /* Do pin init before starting BIOS */
    /* If coming from shutdown, use special gpio table.*/
    if (rSrc == RSTSRC_WAKEUP_FROM_SHUTDOWN) {
        /* The shutdown table has LED1 on to ensure no glitch on the
         * output.
         */
        isWakingFromShutdown = true;
    } else {
        /* When not waking from shutdown, use default init table. */
        isWakingFromShutdown = false;
    }

    /* Setup button pins with ISR */
    hButtons = PIN_open(&buttonState, ButtonTableShutdown);
    PIN_registerIntCb(hButtons, buttonCb);

    /* Construct clock for debounce */
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.arg = (UArg)hButtons;
    Clock_construct(&buttonClock, buttonClockCb, 0, &clockParams);
    hButtonClock = Clock_handle(&buttonClock);

    /* Configure task. */
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = sizeof(myTaskStack);
    taskParams.priority = 4;
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

    Task_Params taskParams_led;
    Task_Params_init(&taskParams_led);
    taskParams_led.stack = ledTaskStack;
    taskParams_led.stackSize = sizeof(ledTaskStack);
    taskParams_led.priority = 3;
    is_ledTaskQuit = 0;
    Task_construct(&ledTask, taskFxn_led, &taskParams_led, NULL);

    /* Configure shutdown semaphore. */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&shutdownSem, 0, &semParams);

    return (0);
}
