/**
 * @file system.h
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#include <stdbool.h>

#define MM2M	1000    // convert mm to m
//#define TRUE	1
//#define FALSE	0

typedef struct _system_state
{
	struct
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
	}sv;

	struct
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
	}pv;
} system_state;

void sys_get_status(system_state* ss);
void sys_set_status(system_state* ss);

/* functions for motion control */
bool motion_control_init();
bool kinematics_init(void);
bool motion_control_update(system_state ss);
bool forward_kinematics(system_state* ss);
bool inverse_kinematics(system_state* ss);

/* functions for motor control */


