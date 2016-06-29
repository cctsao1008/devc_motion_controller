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

bool  kinematics_init(system_data* sd);

bool  forward_kinematics(system_data* sd);
bool  inverse_kinematics(system_data* sd);

bool motion_control_init(system_data* sd)
{
    if(initialized == false)
    {
        if(sd == NULL)
            return false;

        if(!kinematics_init(sd))
        {
            return false;
        }
        
        initialized = true;
    }
    
    return true;
}

bool motion_control_update(system_data* sd)
{
    if(!initialized)
        return false;

    inverse_kinematics(sd);
    
    #if 1
    /* fake data */
    sd->pv.w1 = sd->sv.w1;
    sd->pv.w2 = sd->sv.w2;
    sd->pv.w3 = sd->sv.w3;
    sd->pv.w4 = sd->sv.w4;
    #endif
    
    forward_kinematics(sd);
    
    return true;
}

/* get the inverted matrix of RV(4x4) */ 
bool  kinematics_init(system_data* sd)
{
    uint16_t i = 0, row = 0, col = 0;
    
    float mat_inverse[4][4] = {{1.0f,  1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f, -1.0f,  (DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f, -1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                               {1.0f,  1.0f,  (DEFAULT_L1 + DEFAULT_L2), 1},};

    float mat_forward[4][4] = {0};
    
    float mat_src[16] = {0};
    float mat_dst[16] = {0};
    
    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i == 0)
            printf("[MOTION-DEBUG] mat_inverse(2D) : \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%d%d) ", mat_inverse[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    #endif

    for(i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_src[i] = mat_inverse[row][col];

        #if DEBUG
        if(i == 0)
            printf("[MOTION-DEBUG] mat_inverse(2D) to  mat_src(1D) \n");

        printf("%9.4f(%2d) ", mat_src[i], i);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
        #endif
    }
     
    if(!invert4x4(mat_src, mat_dst))
    {
        printf("[MOTION-DEBUG] matrix singular! \n");
        return false;
    }
    
    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i == 0)
            printf("[MOTION-DEBUG] mat_dst(1D) = \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%2d) ", mat_dst[i], i);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    #endif

    for(i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_forward[row][col] = mat_dst[i];

        #if DEBUG
        if(i == 0)
            printf("[MOTION-DEBUG] mat_dst(1D) to  mat_forward(2D) \n");

        printf("%9.4f(%d%d) ", mat_forward[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
        #endif
    }

    memcpy(sd->mat_inverse, mat_inverse, sizeof(mat_inverse));
    memcpy(sd->mat_forward, mat_forward, sizeof(mat_forward));
    
    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i == 0)
            printf("[MOTION-DEBUG] sd->mat_inverse(2D) = \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%d%d) ", sd->mat_inverse[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    
    for(i = 0 ; i < 16 ; i++)
    {
        if(i == 0)
            printf("[MOTION-DEBUG] sd->mat_forward(2D) = \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%d%d) ", sd->mat_forward[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    #endif

    return true;
}

/* inverse kinematics equation */
bool  inverse_kinematics(system_data* sd)
{
    float vx = sd->sv.vx;
    float vy = sd->sv.vy;
    float w0 = sd->sv.w0;
    
    float mat[4][4] = {0};
    
    memcpy(mat, sd->mat_inverse, sizeof(mat));
    
    sd->sv.w1 = (1 / R) * (vx * mat[0][0] + vy * mat[0][1] + w0 * mat[0][2]);
    sd->sv.w2 = (1 / R) * (vx * mat[1][0] + vy * mat[1][1] + w0 * mat[1][2]);
    sd->sv.w3 = (1 / R) * (vx * mat[2][0] + vy * mat[2][1] + w0 * mat[2][2]);
    sd->sv.w4 = (1 / R) * (vx * mat[3][0] + vy * mat[3][1] + w0 * mat[3][2]);

    #if DEBUG
    printf("[MOTION-DEBUG] inverse_kinematics = \n");
    printf("input : vx, vy, w0 = \n");
    printf("%9.4f %9.4f %9.4f \n\n", vx, vy, w0);
    printf("output : w1, w2, w3, w4 = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n\n", sd->sv.w1, sd->sv.w2,
                                           sd->sv.w3, sd->sv.w4);
    #endif

    return true;
}

/* forward kinematics equation */
bool  forward_kinematics(system_data* sd)
{
    float w1 = sd->pv.w1;
    float w2 = sd->pv.w2;
    float w3 = sd->pv.w3;
    float w4 = sd->pv.w4;
    
    float mat[4][4] = {0};
    
    memcpy(mat, sd->mat_forward, sizeof(mat));

    sd->pv.vx = (mat[0][0] * w1 + mat[0][1] * w2 + mat[0][2] * w3 + mat[0][3] * w4) * R;
    sd->pv.vy = (mat[1][0] * w1 + mat[1][1] * w2 + mat[1][2] * w3 + mat[1][3] * w4) * R;
    sd->pv.w0 = (mat[3][0] * w1 + mat[3][1] * w2 + mat[3][2] * w3 + mat[3][3] * w4) * R;

    #if DEBUG
    printf("[MOTION-DEBUG] forward_kinematics = \n");
    printf("input : w1, w2, w3, w4 = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n\n", w1, w2, w3, w4);
    printf("output : vx, vy, w0 = \n");
    printf("%9.4f %9.4f %9.4f \n\n", sd->pv.vx, sd->pv.vy, sd->pv.w0);
    #endif

    return true;
}

