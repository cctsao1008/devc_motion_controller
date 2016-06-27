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
#include <math.h>

#include "system.h"
#include "small-matrix-inverse\invert4x4_c.h"
#include "..\platform\platform.h"

float RV1[4] = {1.0f,  1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0};
float RV2[4] = {1.0f, -1.0f,  (DEFAULT_L1 + DEFAULT_L2), 0};
float RV3[4] = {1.0f, -1.0f, -(DEFAULT_L1 + DEFAULT_L2), 0};
float RV4[4] = {1.0f,  1.0f,  (DEFAULT_L1 + DEFAULT_L2), 1};

float INV_RV1[4], INV_RV2[4], INV_RV3[4], INV_RV4[4];

float R = DEFAULT_R;

/* get the inversed matrix of RV(4x4) */ 
void kinematics_init(void)
{
	uint16_t i = 0;

	float m_src[16] = { RV1[0], RV1[1], RV1[2], RV1[3],
						RV2[0], RV2[1], RV2[2], RV2[3],
						RV3[0], RV3[1], RV3[2], RV3[3],
						RV4[0], RV4[1], RV4[2], RV4[3]};
						
	float m_dst[16];
	
	invert4x4(m_src, m_dst);

	#if 0
	printf("m_src = \n");
	for(i = 0 ; i < 4 ; i++)
	{
		printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", m_src[i*4 + 0], m_src[i*4 + 1], m_src[i*4 + 2], m_src[i*4 + 3]);
	} 

    printf("m_dst = \n");
	for(i = 0 ; i < 4 ; i++)
	{
		printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", m_dst[i*4 + 0], m_dst[i*4 + 1], m_dst[i*4 + 2], m_dst[i*4 + 3]);
	}
	#endif
	
	for(i = 0 ; i < 3 ; i++)
	{
		INV_RV1[i] = m_dst[i +  0];
		INV_RV2[i] = m_dst[i +  4];
		INV_RV3[i] = m_dst[i +  8];
		INV_RV4[i] = m_dst[i + 12];
	}

	#if 0
	printf("INV_RV = \n");
	printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", INV_RV1[0], INV_RV1[1], INV_RV1[2], INV_RV1[3]);
	printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", INV_RV2[0], INV_RV2[1], INV_RV2[2], INV_RV2[3]);
	printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", INV_RV3[0], INV_RV3[1], INV_RV3[2], INV_RV3[3]);
	printf("%+5.4f %+5.4f %+5.4f %+5.4f \n", INV_RV4[0], INV_RV4[1], INV_RV4[2], INV_RV4[3]);
	#endif
}

/* forward kinematics equation */
void forward_kinematics(system_state* ss)
{
	float w1 = ss->w1;
	float w2 = ss->w2;
	float w3 = ss->w3;
	float w4 = ss->w4;

	ss->vx = (INV_RV1[0] * w1 + INV_RV1[1] * w2 + INV_RV1[2] * w3 + INV_RV1[3] * w4) * R;
	ss->vy = (INV_RV2[0] * w1 + INV_RV2[1] * w2 + INV_RV2[2] * w3 + INV_RV2[3] * w4) * R;
	ss->w0 = (INV_RV3[0] * w1 + INV_RV3[1] * w2 + INV_RV3[2] * w3 + INV_RV3[3] * w4) * R;
}

/* inverse kinematics equation */
void inverse_kinematics(system_state* ss)
{
	float vx = ss->vx;
	float vy = ss->vy;
	float w0 = ss->w0;
	
	ss->w1 = (1 / R) * (vx * RV1[0] + vy * RV1[1] + w0 * RV1[2]);
	ss->w2 = (1 / R) * (vx * RV2[0] + vy * RV2[1] + w0 * RV2[2]);
	ss->w3 = (1 / R) * (vx * RV3[0] + vy * RV3[1] + w0 * RV3[2]);
	ss->w4 = (1 / R) * (vx * RV4[0] + vy * RV4[1] + w0 * RV4[2]);
}

