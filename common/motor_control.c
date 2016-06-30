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
#include <string.h>
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
    MSG(sd->log, "%s", "[INFO] motor_control_init... \n");

    if(initialized == false)
    {
        if(sd == NULL)
        {
            MSG(sd->log, "[ERROR] motor_control_init, failed! \n");
        }
        
        if(!motor_driver_init(sd))
        {
            MSG(sd->log, "[ERROR] motor_control_init, failed! \n");
        }

        initialized = true;
    }

    return true;
}

bool motor_control_update(system_data* sd)
{
    return true;
}

