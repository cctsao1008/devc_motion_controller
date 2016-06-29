/**
 * @file motor_control.c
 *
 * motor control
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "system.h"
#include "..\platform\platform.h"

static bool initialized = false;

float AGR2RPM(float w)
{
    return (w/M_PI);
}

float RPM2PWM(float rpm)
{
    return rpm;
}

bool motor_control_init(system_data* sd)
{
    printf("%-45s", "[INFO] motor_control_init... ");

    if(initialized == false)
    {
        if(sd == NULL)
        {
            printf("FAILED \n");
            return false;
        }

        initialized = true;
        printf("PASSED \n");
    }

    return true;
}

bool motor_control_update(system_data* sd)
{
    return true;
}

