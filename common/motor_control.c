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

#define DEBUG false

#define MAF_MAX 30
float moving_average_filter_1(float data);
float moving_average_filter_2(float data);
float moving_average_filter_3(float data);
float moving_average_filter_4(float data);

static bool initialized = false;

float AGR2RPM(float w)
{
    return (w/M_PI);
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

float limiter(float w)
{
    //if((w < 0.628f) && (w >= 0.0f))
    //	return 0.628f;
    //else if((w > -0.628f) && (w <= 0.0f))
    //	return -0.628f;

    return w;
}

bool motor_control_update(system_data* sd)
{
    float w1 = sd->mot.out.w1;
    float w2 = sd->mot.out.w2;
    float w3 = sd->mot.out.w3;
    float w4 = sd->mot.out.w4;

    bool fr1, fr2, fr3, fr4;
    float rpm1, rpm2, rpm3, rpm4;
    float pwm1, pwm2, pwm3, pwm4;

    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] motor_control_update, failed! \n");
        return false;
    }

    /* fake data */
    sd->mot.in.w1 = moving_average_filter_1(w1);
    sd->mot.in.w2 = moving_average_filter_2(w2);
    sd->mot.in.w3 = moving_average_filter_3(w3);
    sd->mot.in.w4 = moving_average_filter_4(w4);

    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update, moving average filter \n");
    MSG(sd->log, "w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.in.w1, sd->mot.in.w2,
                                                 sd->mot.in.w3, sd->mot.in.w4);
    #endif


    w1 = limiter(w1);
    w2 = limiter(w2);
    w3 = limiter(w3);
    w4 = limiter(w4);

    fr1 = (w1 > 0.0f) ? 1 : 0; w1 = fabs(w1);
    fr2 = (w2 > 0.0f) ? 1 : 0; w2 = fabs(w2);
    fr3 = (w3 > 0.0f) ? 1 : 0; w3 = fabs(w3);
    fr4 = (w4 > 0.0f) ? 1 : 0; w4 = fabs(w4);

    rpm1 = AGR2RPM(w1) * MIN2S;
    rpm2 = AGR2RPM(w2) * MIN2S;
    rpm3 = AGR2RPM(w3) * MIN2S;
    rpm4 = AGR2RPM(w4) * MIN2S;

    #if DEBUG
    MSG(sd->log, "[DEBUG] motor_control_update = \n");
    MSG(sd->log, "out, w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);

    MSG(sd->log, "out, rpm1, rpm2, rpm3, rpm4 (rev/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", rpm1, rpm2, rpm3, rpm4);

    MSG(sd->log, "out, fr1, fr2, fr3, fr4 (1 forward, 0 reverse) = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", fr1, fr2, fr3, fr4);
    #endif

    sd->mot.out.rpm1 = rpm1;
    sd->mot.out.rpm2 = rpm2;
    sd->mot.out.rpm3 = rpm3;
    sd->mot.out.rpm4 = rpm4;

    sd->mot.out.fr1 = fr1;
    sd->mot.out.fr2 = fr2;
    sd->mot.out.fr3 = fr3;
    sd->mot.out.fr4 = fr4;

    motor_driver_update(sd);

    return true;
}



float moving_average_filter_1(float data)
{
    static float buf[MAF_MAX];
    static uint8_t cursor;
    float sum = 0;
    uint8_t i = 0;

    //printf("cursor = %d, data = %f \n", cursor, data);

    buf[cursor++] = data;

    //printf("buf[cursor - 1] = %f \n", buf[cursor - 1]);

    if(cursor == MAF_MAX)
        cursor = 0;


    for(i = 0;  i < MAF_MAX ; i++)
        sum += buf[i];

    //printf("sum = %f \n", sum);

    sum = sum/(MAF_MAX);

    return sum;
}

float moving_average_filter_2(float data)
{
    static float buf[MAF_MAX];
    static uint8_t cursor;
    float sum = 0;
    uint8_t i = 0;

    buf[cursor++] = data;

    if(cursor == MAF_MAX)
        cursor = 0;


    for(i = 0;  i < MAF_MAX ; i++)
        sum += buf[i];

    sum = sum/(MAF_MAX);

    return sum;
}

float moving_average_filter_3(float data)
{
    static float buf[MAF_MAX];
    static uint8_t cursor;
    float sum = 0;
    uint8_t i = 0;

    buf[cursor++] = data;

    if(cursor == MAF_MAX)
        cursor = 0;


    for(i = 0;  i < MAF_MAX ; i++)
        sum += buf[i];

    sum = sum/(MAF_MAX);

    return sum;
}

float moving_average_filter_4(float data)
{
    static float buf[MAF_MAX];
    static uint8_t cursor;
    float sum = 0;
    uint8_t i = 0;

    buf[cursor++] = data;

    if(cursor == MAF_MAX)
        cursor = 0;


    for(i = 0;  i < MAF_MAX ; i++)
        sum += buf[i];

    sum = sum/(MAF_MAX);

    return sum;
}

