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
#include <conio.h>

#include "system.h"
#include "..\platform\platform.h"

#define DEBUG false

bool pid_control_init(system_data* sd)
{
    sd->vx_ga.kp = 0.1f;
    sd->vx_ga.ki = 0.0f;
    sd->vx_ga.kd = 0.05f;

    sd->vy_ga.kp = 0.1f;
    sd->vy_ga.ki = 0.0f;
    sd->vy_ga.kd = 0.05f;

    sd->w0_ga.kp = 0.1f;
    sd->w0_ga.ki = 0.0f;
    sd->w0_ga.kd = 0.05f;

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
    sd->t_curr = clock();

    sd->t_delta = sd->t_curr - sd->t_prev;

    if(sd->t_delta < 0.0f)
    {
        MSG(sd->log, "[ERROR] pid_control_update, failed! \n");
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
    p_out[0] = sd->vx_ga.kp * vx_err;
    i_out[0] = sd->vx_ga.ki * vx_err_sum;
    d_out[0] = sd->vx_ga.kd * vx_err_dif;

    p_out[1] = sd->vy_ga.kp * vy_err;
    i_out[1] = sd->vy_ga.ki * vy_err_sum;
    d_out[1] = sd->vy_ga.kd * vy_err_dif;

    p_out[2] = sd->w0_ga.kp * w0_err;
    i_out[2] = sd->w0_ga.ki * w0_err_sum;
    d_out[2] = sd->w0_ga.kd * w0_err_dif;

    /* summation */
    sd->cv.vx += p_out[0] + i_out[0] + d_out[0];
    sd->cv.vy += p_out[1] + i_out[1] + d_out[1];
    sd->cv.w0 += p_out[2] + i_out[2] + d_out[2];

    //soft_brake(sd);

    #if DEBUG
    MSG(sd->log, "[DEBUG] pid_control_update : \n");
    MSG(sd->log, "vx, p_out, i_out, d_out = \n");
    MSG(sd->log, "%9.4f %9.4f %9.4f, %9.4f %9.4f, %9.4f \n\n", p_out[0], i_out[0], d_out[0], vx_err, vx_err_prev, sd->cv.vx);
    #endif

    vx_err_prev = vx_err;
    vy_err_prev = vy_err;
    w0_err_prev = w0_err;

    vx_err_sum_prev = vx_err_sum;
    vy_err_sum_prev = vy_err_sum;
    w0_err_sum_prev = w0_err_sum;

    return true;
}
