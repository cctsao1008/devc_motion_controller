/**
 * @file system.c
 *
 * configuration file for OMR10 4WD mecanum wheel platform
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include "system.h"

//system_state ss;

/* status */
system_state _ss = {
    .vx = 0.0f,
    .vy = 0.0f,
    .w0 = 0.0f,
    .yaw = 0.0f,
};

void sys_get_status(system_state* ss)
{
	 *ss = _ss;
}

void sys_set_status(system_state* ss)
{
	_ss = *ss;
}

/* parameters */
