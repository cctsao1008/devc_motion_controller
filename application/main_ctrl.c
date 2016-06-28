/**
 * @file main_ctrl.c
 *
 * main control
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "..\common\system.h"
#include "..\platform\platform.h"

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char *argv[]) {
	/* setting value, control value, process value */
	system_state ss;
	
	srand((unsigned) time(NULL) + getpid());

	ss.sv.vx = 1.0f;
	motion_control_init();
	motion_control_update(ss);

	while(1)
	{
		sys_get_status(&ss);
		//printf("ss.pv.vx = %f, ss.pv.vy = %f, ss.pv.w0 = %f \n", ss.pv.vx, ss.pv.vy, ss.pv.w0);
		clock_t start = clock();
		sleep(1);
		usleep(501000);
		clock_t end = clock();
		double d = (double)(end - start) / CLOCKS_PER_SEC;
		//printf("%f \n", d);
	}

	system("pause");
	return 0;
}

