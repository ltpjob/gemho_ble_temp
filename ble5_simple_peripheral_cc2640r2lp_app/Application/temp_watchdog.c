/*
 * temp_watchdog.c
 *
 *  Created on: 2018年11月20日
 *      Author: Administrator
 */

#include <stdio.h>
#include "Board.h"
#include <ti/drivers/Watchdog.h>

static Watchdog_Handle watchdogHandle;

static void watchdogCallback(uintptr_t unused)
{
    /* Clear watchdog interrupt flag */
    Watchdog_clear(watchdogHandle);
}

int temp_watchdog_init()
{
    Watchdog_init();
    Watchdog_Params params;
    params.callbackFxn = (Watchdog_Callback)watchdogCallback;
    params.resetMode = Watchdog_RESET_ON;
    params.debugStallMode = Watchdog_DEBUG_STALL_ON;
    watchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);

    if(watchdogHandle == NULL)
    {
        return -1;
    }
    else
    {
        uint32_t ticks = Watchdog_convertMsToTicks(watchdogHandle, 5*1000);
        Watchdog_setReload(watchdogHandle, ticks);
    }

    return 0;
}
