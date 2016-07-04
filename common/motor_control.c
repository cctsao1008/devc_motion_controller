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
    return fabs(rpm * DEFAULT_RPM2PWM_SLOPE);
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
    if(pwm < DEFAULT_MIN_PWM)
        return DEFAULT_MIN_PWM;
    else if(pwm > DEFAULT_MAX_PWM)
        return DEFAULT_MAX_PWM;
}

bool motor_control_update(system_data* sd)
{
    float w1 = sd->mot.out.w1;
    float w2 = sd->mot.out.w2;
    float w3 = sd->mot.out.w3;
    float w4 = sd->mot.out.w4;

    float fr1, fr2, fr3, fr4;
    float rpm1, rpm2, rpm3, rpm4;
    float pwm1, pwm2, pwm3, pwm4;

    static float filter[4];

    if((sd == NULL) || (initialized != true))
    {
        MSG(sd->log, "[ERROR] motor_control_update, failed! \n");
        return false;
    }

    fr1 = (w1 > 0) ? 1 : 0; fabs(w1);
    fr2 = (w2 > 0) ? 1 : 0; fabs(w2);
    fr3 = (w3 > 0) ? 1 : 0; fabs(w3);
    fr4 = (w4 > 0) ? 1 : 0; fabs(w4);

    rpm1 = AGR2RPM(w1) * MIN2S;
    rpm2 = AGR2RPM(w2) * MIN2S;
    rpm3 = AGR2RPM(w3) * MIN2S;
    rpm4 = AGR2RPM(w4) * MIN2S;

    pwm1 = RPM2PWM(rpm1);
    pwm2 = RPM2PWM(rpm2);
    pwm3 = RPM2PWM(rpm3);
    pwm4 = RPM2PWM(rpm4);

    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update = \n");
    MSG(sd->log, "out, w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);

    MSG(sd->log, "out, rpm1, rpm2, rpm3, rpm4 (rev/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.out.rpm1, sd->mot.out.rpm2,
                                                 sd->mot.out.rpm3, sd->mot.out.rpm4);

    MSG(sd->log, "out, pwm1, pwm2, pwm3, pwm4 = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.out.pwm1, sd->mot.out.pwm2,
                                                 sd->mot.out.pwm3, sd->mot.out.pwm4);

    MSG(sd->log, "out, fr1, fr2, fr3, fr4 (1 forward, 0 reverse) = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", sd->mot.fr1, sd->mot.fr2,
                                         sd->mot.fr3, sd->mot.fr4);
    #endif

    #if 0
    /* fake data */
    filter[0] += sd->mot.out.w1;
    filter[1] += sd->mot.out.w2;
    filter[2] += sd->mot.out.w3;
    filter[3] += sd->mot.out.w4;

    sd->mot.in.w1 = filter[0] / 5.0f;
    sd->mot.in.w2 = filter[1] / 5.0f;
    sd->mot.in.w3 = filter[2] / 5.0f;
    sd->mot.in.w4 = filter[3] / 5.0f;
    #else
    sd->mot.in.w1 = 0.0f;
    sd->mot.in.w2 = 0.0f;
    sd->mot.in.w3 = 0.0f;
    sd->mot.in.w4 = 0.0f;
    #endif

    #if DEBUG
    MSG(sd->log, "filter = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", filter[0], filter[1], filter[2], filter[3]);
    #endif

    sd->mot.out.rpm1 = rpm1;
    sd->mot.out.rpm2 = rpm2;
    sd->mot.out.rpm3 = rpm3;
    sd->mot.out.rpm4 = rpm4;

    sd->mot.fr1 = fr1;
    sd->mot.fr2 = fr2;
    sd->mot.fr3 = fr3;
    sd->mot.fr4 = fr4;

    if(sd->mot.mode == 0) // 0 : controlled by motion, VX, VY, W0
    {
        MSG(sd->log, "[INFO] motor_control_updat, motion \n");

        sd->mot.out.pwm1 = pwm_limiter(pwm1);
        sd->mot.out.pwm2 = pwm_limiter(pwm2);
        sd->mot.out.pwm3 = pwm_limiter(pwm3);
        sd->mot.out.pwm4 = pwm_limiter(pwm4);
    }
    else if(sd->mot.mode == 1) // 1 : controlled by manual, RPM
    {
        MSG(sd->log, "[INFO] motor_control_updat, manual RPM \n");

        pwm1 = sd->mot.man.rpm1 * DEFAULT_RPM2PWM_SLOPE;
        pwm2 = sd->mot.man.rpm2 * DEFAULT_RPM2PWM_SLOPE;
        pwm3 = sd->mot.man.rpm3 * DEFAULT_RPM2PWM_SLOPE;
        pwm4 = sd->mot.man.rpm4 * DEFAULT_RPM2PWM_SLOPE;

        sd->mot.out.pwm1 = pwm_limiter(pwm1);
        sd->mot.out.pwm2 = pwm_limiter(pwm2);
        sd->mot.out.pwm3 = pwm_limiter(pwm3);
        sd->mot.out.pwm4 = pwm_limiter(pwm4);
    }
    else if(sd->mot.mode == 2) // 2 : controlled by manual, PWM
    {
        MSG(sd->log, "[INFO] motor_control_updat, manual PWM \n");

        sd->mot.out.pwm1 = pwm_limiter(sd->mot.man.pwm1);
        sd->mot.out.pwm2 = pwm_limiter(sd->mot.man.pwm2);
        sd->mot.out.pwm3 = pwm_limiter(sd->mot.man.pwm3);
        sd->mot.out.pwm4 = pwm_limiter(sd->mot.man.pwm4);
    }

    motor_driver_update(sd);

    return true;
}

