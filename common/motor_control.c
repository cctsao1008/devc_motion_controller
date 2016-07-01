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

#define DEBUG true

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
    float w1 = sd->mot.in.w1;
    float w2 = sd->mot.in.w1;
    float w3 = sd->mot.in.w1;
    float w4 = sd->mot.in.w1;

    if(sd == NULL)
        return false;
    
    #if 1
    /* fake data */
    sd->mot.out.w1 = sd->mot.in.w1;
    sd->mot.out.w2 = sd->mot.in.w2;
    sd->mot.out.w3 = sd->mot.in.w3;
    sd->mot.out.w4 = sd->mot.in.w4;
    #endif

    sd->mot.in.rpm1 = w1 / M_PI * MIN2S;
    sd->mot.in.rpm2 = w2 / M_PI * MIN2S;
    sd->mot.in.rpm3 = w3 / M_PI * MIN2S;
    sd->mot.in.rpm4 = w4 / M_PI * MIN2S;
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update = \n");
    MSG(sd->log, "input : w1, w2, w3, w4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);
    MSG(sd->log, "input : rpm1, rpm2, rpm3, rpm4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.in.rpm1, sd->mot.in.rpm2,
                                                 sd->mot.in.rpm3, sd->mot.in.rpm4);
    #endif

    return true;
}

