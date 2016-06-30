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

/* message */
//#define MSG(buf, fmt, args...)	do { sprintf(buf, "" fmt "", ##args); } while(0)
#define MSG(buf, fmt, args...)	do { printf("" fmt "", ##args); } while(0)

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
        bool  fr[4];
        float w[4];
        float rpm[4];
        float pwm[4];

    } in, out;
} motor_data;

typedef struct _system_data
{
    char log[4096];

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
bool motor_control_init(system_data* sd);
bool motor_control_update(system_data* sd);

/* functions for motor driver */
bool motor_driver_init(system_data* sd);
bool motor_driver_update(system_data* sd);

