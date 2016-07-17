/**
 * @file system.h
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#ifdef __cplusplus
    extern "C" {
#endif

#pragma once

#include <stdbool.h>

#define MM2M    1000  // convert mm to m
#define MIN2S   60    // convert mm to m

/* message */
//#define MSG(buf, fmt, args...)	do { sprintf(buf, "" fmt "", ##args); } while(0)
#define MSG(buf, fmt, args...)	do { printf("" fmt "", ##args); } while(0)

enum {VX, VY, W0};
enum {M1, M2, M3, M4};

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

    struct
    {
        bool  fr1, fr2, fr3, fr4;
        float rpm1, rpm2, rpm3, rpm4;
        float pwm1, pwm2, pwm3, pwm4;

    } man;

    struct
    {
        bool fr1, fr2, fr3, fr4;
        float w1, w2, w3, w4;
        float rpm1, rpm2, rpm3, rpm4;
        float pwm1, pwm2, pwm3, pwm4;

    } in, out;
} mot_data;

typedef struct _pid_data
{
    float kp, ki, kd;
} pid_data;

typedef struct _system_data
{
    /* system */
    char log[256];
    double loop_time;
    double sys_usage;
    double sys_elaps;

    /* motion control */
    system_state sv, cv, pv;

    float mat_inverse[4][4];
    float mat_forward[4][4];

    /* motor control */
    mot_data mot;
    pid_data pid[3];
    double t_prev, t_curr, t_delta;

    /* motor driver */
    void* hComm;
    int fd;

    /* platform */

} system_data;

system_data* system_init(void);

int msleep(unsigned long milisec);

/* functions for aommander */
bool commander_init(system_data* sd);

/* functions for motion control */
bool motion_control_init(system_data* sd);
bool motion_control_update(system_data* sd);

/* functions for motor control */
bool motor_control_init(system_data* sd);
bool motor_control_update(system_data* sd);

/* functions for motor driver */
bool motor_driver_init(system_data* sd);
bool motor_driver_update(system_data* sd);

/* pid control */
bool pid_control_init(system_data* sd);
bool pid_control_update(system_data* sd);

/* fuzzy control */
bool fuzzy_control_init(system_data* sd);
bool fuzzy_control_update(system_data* sd);

/* neural network */
bool neural_network_init(system_data* sd);
bool neural_network_update(system_data* sd);

#ifdef __cplusplus
    }
#endif

