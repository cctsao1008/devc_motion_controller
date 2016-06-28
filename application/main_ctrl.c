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

#define perf_begin  clock_t start = clock()
#define perf_end    clock_t end = clock()

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char *argv[]) {
    /* setting value, control value, process value */
    system_state ss;
    struct timespec hrt = {
        /**//* seconds */
        .tv_sec  = 0,
        /**//* nanoseconds */  
        .tv_nsec = 500000000UL,
    };
    
    srand((unsigned) time(NULL) + getpid());

    ss.sv.vx = 1.0f;
    motion_control_init();

    while(1)
    {
        perf_begin;
        sys_get_status(&ss);
        motion_control_update(ss);
        usleep(250000);
        perf_end;
        
        double d = (double)(end - start) / CLOCKS_PER_SEC;
        printf("%f, %2.2f %% \n", d, (float)(d / 0.5f * 100));

        hrt.tv_nsec = 500000000UL - (d * 1000000000UL);
        nanosleep(&hrt, NULL);
    }

    system("pause");
    return 0;
}

