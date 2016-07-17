/**
 * @file pid.c
 *
 * pid
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
#include <pthread.h>

#include "system.h"
#include "../platform/platform.h"

#define DEBUG false

static struct timespec t;

bool pid_control_init(system_data* sd)
{
    sd->pid[VX].kp = 0.1f;
    sd->pid[VX].ki = 0.0f;
    sd->pid[VX].kd = 0.05f;

    sd->pid[VY].kp = 0.1f;
    sd->pid[VY].ki = 0.0f;
    sd->pid[VY].kd = 0.05f;

    sd->pid[W0].kp = 0.1f;
    sd->pid[W0].ki = 0.0f;
    sd->pid[W0].kd = 0.05f;

    //sd->t_curr = clock();
    clock_gettime(CLOCK_MONOTONIC, &t);
    sd->t_curr = t.tv_sec * 1000 + t.tv_nsec / 1000000;
    sd->t_delta = 0;

    return true;
}

bool pid_control_update(system_data* sd)
{
    static float vx_err_prev, vy_err_prev, w0_err_prev;
    static float vx_err, vy_err, w0_err;

    static float vx_err_sum_prev, vy_err_sum_prev, w0_err_sum_prev;
    static float vx_err_sum, vy_err_sum, w0_err_sum;

    static float vx_err_dif, vy_err_dif, w0_err_dif;

    static float p_out[3], i_out[3], d_out[3];

    sd->t_prev = sd->t_curr;
    //sd->t_curr = clock();
    clock_gettime(CLOCK_MONOTONIC, &t);
    sd->t_curr = t.tv_sec * 1000 + t.tv_nsec/1000000.0;

    sd->t_delta = sd->t_curr - sd->t_prev;

    if(sd->t_delta <= 0.0f)
    {
        MSG(sd->log, "[ERROR] pid_control_update, failed! perv = %f, curr = %f \n",
            sd->t_prev, sd->t_curr);

        sd->t_delta = 0;
        return false;
    }

    #if DEBUG
    MSG(sd->log, "[DEBUG] pid_control_update : \n");
    MSG(sd->log, "t_prev, t_curr, t_delta (ms) = \n");
    MSG(sd->log, "%9.4ld %9.4ld %9.4ld \n\n", sd->t_prev, sd->t_curr, sd->t_delta);
    #endif

    /* proportional */
    vx_err = sd->sv.vx - sd->pv.vx;
    vy_err = sd->sv.vy - sd->pv.vy;
    w0_err = sd->sv.w0 - sd->pv.w0;

    /* integral */
    vx_err_sum = vx_err * (sd->t_delta / 1000.0f) + vx_err_sum_prev;
    vy_err_sum = vy_err * (sd->t_delta / 1000.0f) + vy_err_sum_prev;
    w0_err_sum = w0_err * (sd->t_delta / 1000.0f) + w0_err_sum_prev;

    /* derivative */
    vx_err_dif = (vx_err - vx_err_prev) / (sd->t_delta / 1000.0f) ;
    vy_err_dif = (vy_err - vy_err_prev) / (sd->t_delta / 1000.0f) ;
    w0_err_dif = (w0_err - w0_err_prev) / (sd->t_delta / 1000.0f) ;

    /* gain */
    p_out[VX] = sd->pid[VX].kp * vx_err;
    i_out[VX] = sd->pid[VX].ki * vx_err_sum;
    d_out[VX] = sd->pid[VX].kd * vx_err_dif;

    p_out[VY] = sd->pid[VY].kp * vy_err;
    i_out[VY] = sd->pid[VY].ki * vy_err_sum;
    d_out[VY] = sd->pid[VY].kd * vy_err_dif;

    p_out[W0] = sd->pid[W0].kp * w0_err;
    i_out[W0] = sd->pid[W0].ki * w0_err_sum;
    d_out[W0] = sd->pid[W0].kd * w0_err_dif;

    /* summation */
    sd->cv.vx += p_out[VX] + i_out[VX] + d_out[VX];
    sd->cv.vy += p_out[VY] + i_out[VY] + d_out[VY];
    sd->cv.w0 += p_out[W0] + i_out[W0] + d_out[W0];

    #if DEBUG
    MSG(sd->log, "[DEBUG] pid_control_update : \n");
    MSG(sd->log, "vx, p_out, i_out, d_out = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f, %9.4f %9.4f, %9.4f \n\n", p_out[VX], i_out[VX], d_out[VX], vx_err, vx_err_prev, sd->cv.vx);
    #endif

    vx_err_prev = vx_err;
    vy_err_prev = vy_err;
    w0_err_prev = w0_err;

    vx_err_sum_prev = vx_err_sum;
    vy_err_sum_prev = vy_err_sum;
    w0_err_sum_prev = w0_err_sum;

    return true;
}

