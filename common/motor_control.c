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

/* motor control */
bool motor_control_init(system_data* sd)
{
    MSG(sd->log, "%s", "[INFO] motor_control_init... \n");

    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] motor_control_init, failed! \n");
        return false;
    }

    if(initialized == true)
        return true;
   
    if(!motor_driver_init(sd))
        return false;

    initialized = true;

    return true;
}

bool motor_control_update(system_data* sd)
{
    float w1 = sd->mot.in.w1;
    float w2 = sd->mot.in.w2;
    float w3 = sd->mot.in.w3;
    float w4 = sd->mot.in.w4;

    if((sd == NULL) || (initialized != true))
    {
        MSG(sd->log, "[ERROR] motor_control_update, failed! \n");
        return false;
    }

    sd->mot.fr1 = (w1 > 0) ? 1 : 0;
    sd->mot.fr2 = (w2 > 0) ? 1 : 0;
    sd->mot.fr3 = (w3 > 0) ? 1 : 0;
    sd->mot.fr4 = (w4 > 0) ? 1 : 0;

    sd->mot.in.rpm1 = fabs (w1 / M_PI) * MIN2S;
    sd->mot.in.rpm2 = fabs (w2 / M_PI) * MIN2S;
    sd->mot.in.rpm3 = fabs (w3 / M_PI) * MIN2S;
    sd->mot.in.rpm4 = fabs (w4 / M_PI) * MIN2S;

    sd->mot.in.pwm1 = sd->mot.in.rpm1 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.in.pwm2 = sd->mot.in.rpm2 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.in.pwm3 = sd->mot.in.rpm3 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.in.pwm4 = sd->mot.in.rpm4 * DEFAULT_RPM2PWM_SLOPE;
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update = \n");
    MSG(sd->log, "input : w1, w2, w3, w4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);

    MSG(sd->log, "input : rpm1, rpm2, rpm3, rpm4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.in.rpm1, sd->mot.in.rpm2,
                                                 sd->mot.in.rpm3, sd->mot.in.rpm4);

    MSG(sd->log, "input : pwm1, pwm2, pwm3, pwm4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.in.pwm1, sd->mot.in.pwm2,
                                                 sd->mot.in.pwm3, sd->mot.in.pwm4);

    MSG(sd->log, "input : fr1, fr2, fr3, fr4 = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", sd->mot.fr1, sd->mot.fr2,
                                         sd->mot.fr3, sd->mot.fr4);
    #endif

    #if 1
    /* fake data */
    sd->mot.out.w1 = w1;
    sd->mot.out.w2 = w2;
    sd->mot.out.w3 = w3;
    sd->mot.out.w4 = w4;
    #endif

    motor_driver_update(sd);

    return true;
}

