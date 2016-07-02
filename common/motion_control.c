/**
 * @file motion_control.c
 *
 * motion control
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

#include "small-matrix-inverse\invert4x4_c.h"

#define DEBUG true

static bool initialized = false;
static float R = DEFAULT_R;

/* kinematics equations */
bool kinematics_init(system_data* sd);
bool forward_kinematics(system_data* sd);
bool inverse_kinematics(system_data* sd);

/* pid control */
bool pid_control_init(system_data* sd);
bool pid_control_update(system_data* sd);

/* motion control */
bool motion_control_init(system_data* sd)
{
    MSG(sd->log, "%s", "[INFO] motion_control_init... \n");

    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] motion_control_update, failed! \n");
        return false;
    }

    if(initialized == true)
        return true;
    
    if(!kinematics_init(sd))
        return false;

    pid_control_init(sd);
    
    initialized = true;

    return true;
}

bool motion_control_update(system_data* sd)
{
    if((sd == NULL) || (initialized != true))
    {
        MSG(sd->log, "[ERROR] motion_control_update, failed! \n");
        return false;
    }

    if(sd->sv.vx > DEFAULT_MAX_VX)
        sd->sv.vx = DEFAULT_MAX_VX;

    if(sd->sv.vy > DEFAULT_MAX_VY)
        sd->sv.vy = DEFAULT_MAX_VY;
 
    if(sd->sv.w0 > DEFAULT_MAX_W0)
        sd->sv.w0 = DEFAULT_MAX_W0;

    /* calculating feedback signals, vx, vy, w0 */
    forward_kinematics(sd);

    /* calculating output signals, vx, vy, w0 */
    pid_control_update(sd);

    /* calculating output signals, w1, w2, w3, w4 */
    inverse_kinematics(sd);
    
    return true;
}

/* get the inverted matrix of RV(4x4) */ 
bool kinematics_init(system_data* sd)
{
    uint16_t i = 0, row = 0, col = 0;

    float mat_inverse[4][4] = {{1.0f,  1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f, -1.0f,  (DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f, -1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f,  1.0f,  (DEFAULT_L1 + DEFAULT_L2), 1},};

    float mat_forward[4][4] = {0};

    float mat_src[16] = {0};
    float mat_dst[16] = {0};

    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] kinematics_init, failed! \n");
        return false;
    }

    if(initialized == true)
        return true;

    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i < 1)
            MSG(sd->log, "[DEBUG] mat_inverse(2D) : \n");

        row = (i / 4); col = (i % 4);
        MSG(sd->log, "%9.4f(%d%d) ", mat_inverse[row][col], row, col);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }

    for(i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_src[i] = mat_inverse[row][col];

        if(i < 1)
            MSG(sd->log, "[DEBUG] mat_inverse(2D) to  mat_src(1D) \n");

        MSG(sd->log, "%9.4f(%2d) ", mat_src[i], i);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }
    #endif

    if(!invert4x4(mat_src, mat_dst))
    {
        MSG(sd->log, "[ERROR] matrix singular! \n");
        return false;
    }

    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i < 1)
            MSG(sd->log, "[DEBUG] mat_dst(1D) = \n");

        row = (i / 4); col = (i % 4);
        MSG(sd->log, "%9.4f(%2d) ", mat_dst[i], i);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }

    for(i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_forward[row][col] = mat_dst[i];

        if(i < 1)
            MSG(sd->log, "[DEBUG] mat_dst(1D) to  mat_forward(2D) \n");

        MSG(sd->log, "%9.4f(%d%d) ", mat_forward[row][col], row, col);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }
    #endif

    memcpy(sd->mat_inverse, mat_inverse, sizeof(mat_inverse));
    memcpy(sd->mat_forward, mat_forward, sizeof(mat_forward));

    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i < 1)
            MSG(sd->log, "[DEBUG] sd->mat_inverse(2D) = \n");

        row = (i / 4); col = (i % 4);
        MSG(sd->log, "%9.4f(%d%d) ", sd->mat_inverse[row][col], row, col);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }

    for(i = 0 ; i < 16 ; i++)
    {
        if(i < 1)
            MSG(sd->log, "[DEBUG] sd->mat_forward(2D) = \n");

        row = (i / 4); col = (i % 4);
        MSG(sd->log, "%9.4f(%d%d) ", sd->mat_forward[row][col], row, col);
        
        if(((i + 1) % 4 == 0) || (i == 15))
            MSG(sd->log, "%s", (i < 15)? "\n":"\n\n");
    }
    #endif

    return true;
}

/* inverse kinematics equation */
bool inverse_kinematics(system_data* sd)
{
    float vx = sd->cv.vx;
    float vy = sd->cv.vy;
    float w0 = sd->cv.w0;
    
    float mat[4][4] = {0};
    
    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] inverse_kinematics, failed! \n");
        return false;
    }
    
    memcpy(mat, sd->mat_inverse, sizeof(mat));
    
    sd->mot.out.w1 = (1 / R) * (vx * mat[0][0] + vy * mat[0][1] + w0 * mat[0][2]);
    sd->mot.out.w2 = (1 / R) * (vx * mat[1][0] + vy * mat[1][1] + w0 * mat[1][2]);
    sd->mot.out.w3 = (1 / R) * (vx * mat[2][0] + vy * mat[2][1] + w0 * mat[2][2]);
    sd->mot.out.w4 = (1 / R) * (vx * mat[3][0] + vy * mat[3][1] + w0 * mat[3][2]);

    #if DEBUG
    MSG(sd->log, "[DEBUG] inverse_kinematics : \n");
    MSG(sd->log, "vx, vy (m/s), w0 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f \n\n", vx, vy, w0);
    MSG(sd->log, "w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", sd->mot.out.w1, sd->mot.out.w2,
                                                 sd->mot.out.w3, sd->mot.out.w4);
    #endif

    return true;
}

/* forward kinematics equation */
bool forward_kinematics(system_data* sd)
{
    float w1 = sd->mot.in.w1;
    float w2 = sd->mot.in.w2;
    float w3 = sd->mot.in.w3;
    float w4 = sd->mot.in.w4;
    
    float mat[4][4] = {0};
    
    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] forward_kinematics, failed! \n");
        return false;
    }
    
    memcpy(mat, sd->mat_forward, sizeof(mat));

    sd->pv.vx = (mat[0][0] * w1 + mat[0][1] * w2 + mat[0][2] * w3 + mat[0][3] * w4) * R;
    sd->pv.vy = (mat[1][0] * w1 + mat[1][1] * w2 + mat[1][2] * w3 + mat[1][3] * w4) * R;
    sd->pv.w0 = (mat[2][0] * w1 + mat[2][1] * w2 + mat[2][2] * w3 + mat[2][3] * w4) * R;

    #if DEBUG
    MSG(sd->log, "[DEBUG] forward_kinematics : \n");
    MSG(sd->log, "w1, w2, w3, w4 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);
    MSG(sd->log, "vx, vy (m/s), w0 (rad/s) = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f \n\n", sd->pv.vx, sd->pv.vy, sd->pv.w0);
    #endif

    return true;
}

bool pid_control_init(system_data* sd)
{
    sd->vx_ga.kp = 1.0f;
    sd->vx_ga.ki = 0.0f;
    sd->vx_ga.kd = 0.0f;
    
    sd->vy_ga.kp = 1.0f;
    sd->vy_ga.ki = 0.0f;
    sd->vy_ga.kd = 0.0f;
    
    sd->w0_ga.kp = 1.0f;
    sd->w0_ga.ki = 0.0f;
    sd->w0_ga.kd = 0.0f;

    return true;
}

bool pid_control_update(system_data* sd)
{	
    static float vx_err_last, vy_err_last, w0_err_last;
    static float vx_err, vy_err, w0_err;

    static float vx_err_sum_last, vy_err_sum_last, w0_err_sum_last;
    static float vx_err_sum, vy_err_sum, w0_err_sum;

    static float vx_err_diff, vy_err_diff, w0_err_diff;


    sd->t_last = sd->t_curr;
    sd->t_curr = clock();

    sd->t_delta = sd->t_curr - sd->t_last;
    
    if(sd->t_delta < 0.0f)
    {
        MSG(sd->log, "[ERROR] pid_control_update, failed! \n");
        return false;
    }
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] pid_control_update : \n");
    MSG(sd->log, "t_last, t_curr, t_delta (ms) = \n");
    MSG(sd->log, "%9.4ld %9.4ld %9.4ld \n\n", sd->t_last, sd->t_curr, sd->t_delta);
    #endif

    vx_err = sd->sv.vx - sd->pv.vx;
    vy_err = sd->sv.vy - sd->pv.vy;
    w0_err = sd->sv.w0 - sd->pv.w0;

    vx_err_sum = vx_err * sd->t_delta + vx_err_sum_last;
    vy_err_sum = vy_err * sd->t_delta + vy_err_sum_last;
    w0_err_sum = w0_err * sd->t_delta + w0_err_sum_last;

    vx_err_diff = (vx_err - vx_err_last) / sd->t_delta;
    vy_err_diff = (vy_err - vy_err_last) / sd->t_delta;
    w0_err_diff = (w0_err - w0_err_last) / sd->t_delta;

    /* PID */
    sd->cv.vx = (sd->vx_ga.kp * vx_err) + (sd->vx_ga.ki * vx_err_sum) + (sd->vx_ga.kd * vx_err_diff);
    sd->cv.vy = (sd->vy_ga.kp * vy_err) + (sd->vy_ga.ki * vy_err_sum) + (sd->vy_ga.kd * vy_err_diff);
    sd->cv.w0 = (sd->w0_ga.kp * w0_err) + (sd->w0_ga.ki * w0_err_sum) + (sd->w0_ga.kd * w0_err_diff);
    
    vx_err_last = vx_err;
    vy_err_last = vy_err;
    w0_err_last = w0_err;
    
    vx_err_sum_last = vx_err_sum;
    vy_err_sum_last = vy_err_sum;
    w0_err_sum_last = w0_err_sum;

    return true;
}

