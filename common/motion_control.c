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
#include <math.h>
#include <time.h>

#include "system.h"
#include "..\platform\platform.h"

#include "small-matrix-inverse\invert4x4_c.h"

#define DEBUG true

static float RV1[4] = {1.0f,  1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0};
static float RV2[4] = {1.0f, -1.0f,  (DEFAULT_L1 + DEFAULT_L2), 0};
static float RV3[4] = {1.0f, -1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0};
static float RV4[4] = {1.0f,  1.0f,  (DEFAULT_L1 + DEFAULT_L2), 1};

static float INV_RV1[4], INV_RV2[4], INV_RV3[4], INV_RV4[4];

static float mat_inverse[4][4] = {{1.0f,  1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                                  {1.0f, -1.0f,  (DEFAULT_L1 + DEFAULT_L2), 0},
                                  {1.0f, -1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0},
                                  {1.0f,  1.0f,  (DEFAULT_L1 + DEFAULT_L2), 1},};

static float mat_forward[4][4] = {0};

static float R = DEFAULT_R;

static bool initialized = false;

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
    uint16_t i = 0, idx = 0;

    float m_src[16] = { RV1[0], RV1[1], RV1[2], RV1[3],
                        RV2[0], RV2[1], RV2[2], RV2[3],
                        RV3[0], RV3[1], RV3[2], RV3[3],
                        RV4[0], RV4[1], RV4[2], RV4[3]};
                        
    float m_dst[16];
    
    if(!invert4x4(m_src, m_dst))
	{
		printf("[DEBUG] matrix singular! \n");
    	return false;
	}
    	

    #if DEBUG
    printf("[DEBUG] m_src = \n");
    for(i = 0 ; i < 4 ; i++)
    {
        printf("%9.4f %9.4f %9.4f %9.4f \n", m_src[i*4 + 0], m_src[i*4 + 1], m_src[i*4 + 2], m_src[i*4 + 3]);

        if(i == 3)
        printf("\n");
    } 

    printf("[DEBUG] m_dst = \n");
    for(i = 0 ; i < 4 ; i++)
    {
        printf("%9.4f %9.4f %9.4f %9.4f \n", m_dst[i*4 + 0], m_dst[i*4 + 1], m_dst[i*4 + 2], m_dst[i*4 + 3]);

        if(i == 3)
        printf("\n");
    }
    #endif
    
    for(i = 0 ; i < 3 ; i++)
    {
        INV_RV1[i] = m_dst[i +  0];
        INV_RV2[i] = m_dst[i +  4];
        INV_RV3[i] = m_dst[i +  8];
        INV_RV4[i] = m_dst[i + 12];
    }

    #if DEBUG
    printf("[DEBUG] INV_RV = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n"  , INV_RV1[0], INV_RV1[1], INV_RV1[2], INV_RV1[3]);
    printf("%9.4f %9.4f %9.4f %9.4f \n"  , INV_RV2[0], INV_RV2[1], INV_RV2[2], INV_RV2[3]);
    printf("%9.4f %9.4f %9.4f %9.4f \n"  , INV_RV3[0], INV_RV3[1], INV_RV3[2], INV_RV3[3]);
    printf("%9.4f %9.4f %9.4f %9.4f \n\n", INV_RV4[0], INV_RV4[1], INV_RV4[2], INV_RV4[3]);
    #endif
    
    /*  */
    #if 1
    printf("[DEBUG] using arrary method \n");
    sd->mat_inverse = (float*)mat_inverse;
    
    for(i = 0 ; i < 4 ; i++)
        printf("%9.4f ",  mat_inverse[0][i]);
        
    printf("\n\n");
    
    for(i = 0 ; i < 4 ; i++)
        printf("%9.4f ",  sd->mat_inverse[ 0 + i]);
    printf("\n");
        
    for(i = 0 ; i < 4 ; i++)
        printf("%9.4f ",  sd->mat_inverse[ 4 + i]);
    printf("\n");

    for(i = 0 ; i < 4 ; i++)
        printf("%9.4f ",  sd->mat_inverse[ 8 + i]);
    printf("\n");
    
    for(i = 0 ; i < 4 ; i++)
        printf("%9.4f ",  sd->mat_inverse[12 + i]);
    printf("\n");
        
    printf("\n\n");
        
    for(i = 0 ; i < 16 ; i++)
    {
        idx = (((uint16_t)(i / 4) * 4) + (i % 4));
        printf("%9.4f(%2d) ",  *(sd->mat_inverse + idx), idx);
        
        if((i + 1) % 4 == 0)
            printf("\n");
    }

    printf("\n\n");

    #endif
    /*  */

    return true;
}

/* forward kinematics equation */
bool  forward_kinematics(system_data* sd)
{
    float w1 = sd->pv.w1;
    float w2 = sd->pv.w2;
    float w3 = sd->pv.w3;
    float w4 = sd->pv.w4;

    sd->pv.vx = (INV_RV1[0] * w1 + INV_RV1[1] * w2 + INV_RV1[2] * w3 + INV_RV1[3] * w4) * R;
    sd->pv.vy = (INV_RV2[0] * w1 + INV_RV2[1] * w2 + INV_RV2[2] * w3 + INV_RV2[3] * w4) * R;
    sd->pv.w0 = (INV_RV3[0] * w1 + INV_RV3[1] * w2 + INV_RV3[2] * w3 + INV_RV3[3] * w4) * R;

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
    
    sd->sv.w1 = (1 / R) * (vx * RV1[0] + vy * RV1[1] + w0 * RV1[2]);
    sd->sv.w2 = (1 / R) * (vx * RV2[0] + vy * RV2[1] + w0 * RV2[2]);
    sd->sv.w3 = (1 / R) * (vx * RV3[0] + vy * RV3[1] + w0 * RV3[2]);
    sd->sv.w4 = (1 / R) * (vx * RV4[0] + vy * RV4[1] + w0 * RV4[2]);

    #if DEBUG
    printf("[DEBUG] inverse_kinematics = \n");
    printf("sv : vx, vy, w0 = \n");
    printf("%9.4f %9.4f %9.4f \n", vx, vy, w0);
    printf("sv : w1, w2, w3, w4 = \n");
    printf("%9.4f %9.4f %9.4f %9.4f \n\n", sd->sv.w1, sd->sv.w2, sd->sv.w3, sd->sv.w4);
    #endif

    return true;
}

