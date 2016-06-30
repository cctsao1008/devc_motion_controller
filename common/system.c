/**
 * @file system.c
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "system.h"
#include "..\platform\platform.h"

static system_data data;
static bool initialized = false;

system_data* system_init(void)
{
    MSG(data.log, "%-45s", "[INFO] system_init... ");

    if(initialized == false)
    {
        initialized = true;
        MSG(data.log, "PASSED \n");
    }

    return &data;
}

/* parameters */

/* FreeRTOS */
void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
    
}

void vApplicationIdleHook( void )
{
    
}

void vApplicationMallocFailedHook( void )
{
    
}

void vApplicationTickHook( void )
{
    
}

void vConfigureTimerForRunTimeStats( void )
{
    /* This function is not used by the Blinky build configuration, but needs
    to be defined as the Blinky and Full build configurations share a
    FreeRTOSConfig.h header file. */
}
/*-----------------------------------------------------------*/

unsigned long ulGetRunTimeCounterValue( void )
{
    /* This function is not used by the Blinky build configuration, but needs
    to be defined as the Blinky and Full build configurations share a
    FreeRTOSConfig.h header file. */
    return 0UL;
}

