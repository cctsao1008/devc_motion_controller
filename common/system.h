/**
 * @file system.h
 *
 * configuration file for OMR10 4WD mecanum wheel platform
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#define MM2M    1000    // convert mm to m

typedef struct _system_state
{
	/* vx, vy, w0 & yaw in vehicle frame */
	float vx;
	float vy;
	float w0;
	float yaw;
    
    /* angular rate for each wheel */
    float w1;
    float w2;
    float w3;
    float w4;
} system_state;

void sys_get_status(system_state* ss);
void sys_set_status(system_state* ss);

void kinematics_init(void);
void forward_kinematics(system_state* ss);
void inverse_kinematics(system_state* ss);


