/**
 * @file system.c
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include "system.h"

//system_state ss;

/* status */
system_state _ss = {
    .sv.vx = 0.0f,
    .sv.vy = 0.0f,
    .sv.w0 = 0.0f,
    .sv.yaw = 0.0f,
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
