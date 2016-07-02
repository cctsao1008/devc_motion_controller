/**
 * @file system.h
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#include <stdbool.h>

#define MM2M    1000  // convert mm to m
#define MIN2S   60    // convert mm to m

/* message */
//#define MSG(buf, fmt, args...)	do { sprintf(buf, "" fmt "", ##args); } while(0)
#define MSG(buf, fmt, args...)	do { printf("" fmt "", ##args); } while(0)

typedef struct _system_state
{
    /* vx, vy, w0 & yaw in vehicle frame */
    float x, y, yaw;
    float vx, vy, w0;
} system_state;

typedef struct _mot_data
{
    uint8_t mode; // 0 : controlled by motion, VX, VY, W0
                  // 1 : controlled by manual, RPM
                  // 2 : controlled by manual, PWM

    bool fr1, fr2, fr3, fr4;

    struct
    {
        bool  fr1, fr2, fr3, fr4;
        float rpm1, rpm2, rpm3, rpm4;
        float pwm1, pwm2, pwm3, pwm4;

    } man;

    struct
    {
        float w1, w2, w3, w4;
        float rpm1, rpm2, rpm3, rpm4;
        float pwm1, pwm2, pwm3, pwm4;

    } in, out;
} mot_data;

typedef struct _pid_data
{
    float sv, cv, pv;
    float t1, t2;
    float kp, ki, kd;
} pid_data;

typedef struct _system_data
{
    /* system */
    char log[256];

    /* motion control */
    system_state sv, cv, pv;
    float vx_err, vy_err, w0_err;

    float mat_inverse[4][4];
    float mat_forward[4][4];

    /* motor control */
    mot_data mot;
    pid_data pid[3];

    /* motor driver */
    void* hComm;

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

