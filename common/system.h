/**
 * @file system.h
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#include <stdbool.h>

#define MM2M    1000    // convert mm to m

typedef struct _system_state
{
    /* vx, vy, w0 & yaw in vehicle frame */
    float vx;
    float x;
    float vy;
    float y;
    float w0;
    float yaw;

    /* angular rate for each wheel */
    float w1;
    float w2;
    float w3;
    float w4;
} system_state;

typedef struct _motor_data
{
    struct
    {
        float w1;
        float w2;
        float w3;
        float w4;

        float rpm1;
        float rpm2;
        float rpm3;
        float rpm4;

        float pwm1;
        float pwm2;
        float pwm3;
        float pwm4;
    } in, out;
} motor_data;

typedef struct _system_data
{
    system_state    sv;
    system_state    cv;
    system_state    pv;
    motor_data      mot;

    float mat_inverse[4][4];
    float mat_forward[4][4];

    /* platform */

} system_data;

system_data* system_init(void);

/* functions for motion control */
bool motion_control_init(system_data* sd);
bool motion_control_update(system_data* sd);

/* functions for motor control */


