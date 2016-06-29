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
            printf("[DEBUG] mat_inverse(2D) : \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%d%d) ", mat_inverse[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    #endif

    for (i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_src[i] = mat_inverse[row][col];

        #if DEBUG
        if(i == 0)
            printf("[DEBUG] mat_inverse(2D) to  mat_src(1D) \n");

        printf("%9.4f(%2d) ", mat_src[i], i);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
        #endif
    }
     
    if(!invert4x4(mat_src, mat_dst))
    {
        printf("[DEBUG] matrix singular! \n");
        return false;
    }
    
    #if DEBUG
    for(i = 0 ; i < 16 ; i++)
    {
        if(i == 0)
            printf("[DEBUG] mat_dst(1D) = \n");

        row = (i / 4); col = (i % 4);
        printf("%9.4f(%2d) ", mat_dst[i], i);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
    }
    #endif

    for (i = 0; i < 16; i++)
    {
        row = (i / 4); col = (i % 4);
        mat_forward[row][col] = mat_dst[i];

        #if DEBUG
        if(i == 0)
            printf("[DEBUG] mat_dst(1D) to  mat_forward(2D) \n");

        printf("%9.4f(%d%d) ", mat_forward[row][col], row, col);
        
        if((i + 1) % 4 == 0)
            printf("\n");
            
        if(i == 15)
             printf("\n");
        #endif
    }

    #if DEBUG


    
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

    //sd->pv.vx = (INV_RV1[0] * w1 + INV_RV1[1] * w2 + INV_RV1[2] * w3 + INV_RV1[3] * w4) * R;
    //sd->pv.vy = (INV_RV2[0] * w1 + INV_RV2[1] * w2 + INV_RV2[2] * w3 + INV_RV2[3] * w4) * R;
    //sd->pv.w0 = (INV_RV3[0] * w1 + INV_RV3[1] * w2 + INV_RV3[2] * w3 + INV_RV3[3] * w4) * R;

    #if DEBUG
    printf("[DEBUG] forward_kinematics = \n");
    printf("pv : w1, w2, w3, w4 = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n", w1, w2, w3, w4);
    printf("pv : vx, vy, w0 = \n");
    printf("%9.4f %9.4f %9.4f \n\n", sd->pv.vx, sd->pv.vy, sd->pv.w0);
    #endif

    return true;
}

/* inverse kinematics equation */
bool  inverse_kinematics(system_data* sd)
{
    float vx = sd->sv.vx;
    float vy = sd->sv.vy;
    float w0 = sd->sv.w0;
    
    //sd->sv.w1 = (1 / R) * (vx * RV1[0] + vy * RV1[1] + w0 * RV1[2]);
    //sd->sv.w2 = (1 / R) * (vx * RV2[0] + vy * RV2[1] + w0 * RV2[2]);
    //sd->sv.w3 = (1 / R) * (vx * RV3[0] + vy * RV3[1] + w0 * RV3[2]);
    //sd->sv.w4 = (1 / R) * (vx * RV4[0] + vy * RV4[1] + w0 * RV4[2]);

    #if DEBUG
    printf("[DEBUG] inverse_kinematics = \n");
    printf("sv : vx, vy, w0 = \n");
    printf("%9.4f %9.4f %9.4f \n", vx, vy, w0);
    printf("sv : w1, w2, w3, w4 = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n\n", sd->sv.w1, sd->sv.w2, sd->sv.w3, sd->sv.w4);
    #endif

    return true;
}

