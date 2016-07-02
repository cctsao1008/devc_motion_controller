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

uint8_t pwm_limiter(uint8_t pwm)
{
    if(pwm < 0)
        return 0;
    else if(pwm > 100)
        return 100;
}

bool motor_control_update(system_data* sd)
{
    float w1 = sd->mot.out.w1;
    float w2 = sd->mot.out.w2;
    float w3 = sd->mot.out.w3;
    float w4 = sd->mot.out.w4;

    if((sd == NULL) || (initialized != true))
    {
        MSG(sd->log, "[ERROR] motor_control_update, failed! \n");
        return false;
    }

    sd->mot.fr1 = (w1 > 0) ? 1 : 0;
    sd->mot.fr2 = (w2 > 0) ? 1 : 0;
    sd->mot.fr3 = (w3 > 0) ? 1 : 0;
    sd->mot.fr4 = (w4 > 0) ? 1 : 0;

    sd->mot.out.rpm1 = fabs (w1 / M_PI) * MIN2S;
    sd->mot.out.rpm2 = fabs (w2 / M_PI) * MIN2S;
    sd->mot.out.rpm3 = fabs (w3 / M_PI) * MIN2S;
    sd->mot.out.rpm4 = fabs (w4 / M_PI) * MIN2S;

    sd->mot.out.pwm1 = sd->mot.out.rpm1 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.out.pwm2 = sd->mot.out.rpm2 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.out.pwm3 = sd->mot.out.rpm3 * DEFAULT_RPM2PWM_SLOPE;
    sd->mot.out.pwm4 = sd->mot.out.rpm4 * DEFAULT_RPM2PWM_SLOPE;
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update = \n");
    MSG(sd->log, "output : w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);

    MSG(sd->log, "output : rpm1, rpm2, rpm3, rpm4 (rev/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.out.rpm1, sd->mot.out.rpm2,
                                                 sd->mot.out.rpm3, sd->mot.out.rpm4);

    MSG(sd->log, "output : pwm1, pwm2, pwm3, pwm4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.out.pwm1, sd->mot.out.pwm2,
                                                 sd->mot.out.pwm3, sd->mot.out.pwm4);

    MSG(sd->log, "output : fr1, fr2, fr3, fr4 (1 forward, 0 reverse) = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", sd->mot.fr1, sd->mot.fr2,
                                         sd->mot.fr3, sd->mot.fr4);
    #endif

    #if 1
    /* fake data */
    sd->mot.in.w1 = 10.8110f;
    sd->mot.in.w2 = 10.8110f;
    sd->mot.in.w3 = 10.8110f;
    sd->mot.in.w4 = 10.8110f;
    #else
    sd->mot.in.w1 = 0.0f;
    sd->mot.in.w2 = 0.0f;
    sd->mot.in.w3 = 0.0f;
    sd->mot.in.w4 = 0.0f;
    #endif
    
    if(sd->mot.mode == 0) // 0 : controlled by motion, VX, VY, W0
    {
        MSG(sd->log, "[INFO] motor_control_updat, motion \n");
        
        pwm_limiter(sd->mot.out.pwm1);
        pwm_limiter(sd->mot.out.pwm2);
        pwm_limiter(sd->mot.out.pwm3);
        pwm_limiter(sd->mot.out.pwm4);
    }
    else if(sd->mot.mode == 1) // 1 : controlled by manual, RPM
    {
        MSG(sd->log, "[INFO] motor_control_updat, manual RPM \n");

        sd->mot.man.pwm1 = sd->mot.man.rpm1 * DEFAULT_RPM2PWM_SLOPE;
        sd->mot.man.pwm2 = sd->mot.man.rpm2 * DEFAULT_RPM2PWM_SLOPE;
        sd->mot.man.pwm3 = sd->mot.man.rpm3 * DEFAULT_RPM2PWM_SLOPE;
        sd->mot.man.pwm4 = sd->mot.man.rpm4 * DEFAULT_RPM2PWM_SLOPE;

        pwm_limiter(sd->mot.man.pwm1);
        pwm_limiter(sd->mot.man.pwm2);
        pwm_limiter(sd->mot.man.pwm3);
        pwm_limiter(sd->mot.man.pwm4);
    }
    else if(sd->mot.mode == 2) // 2 : controlled by manual, PWM
    {
        MSG(sd->log, "[INFO] motor_control_updat, manual PWM \n");

        pwm_limiter(sd->mot.man.pwm1);
        pwm_limiter(sd->mot.man.pwm2);
        pwm_limiter(sd->mot.man.pwm3);
        pwm_limiter(sd->mot.man.pwm4);
    }

    motor_driver_update(sd);

    return true;
}

