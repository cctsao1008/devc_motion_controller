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
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "..\common\system.h"
#include "..\platform\platform.h"

#define perf_begin()  clock_t start = clock()
#define perf_end()    clock_t end = clock()

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

void mdelay(unsigned int mseconds)
{
    clock_t ticks = mseconds + clock();
    while (ticks > clock());
}

int main(int argc, char *argv[]) {
    /* setting value, control value, process value */
    system_data* sd;

    srand((unsigned) time(NULL) + getpid());

    #if 0  // REF.
    int t1, t2, ts;
    QueryPerformanceFrequency(&ts);
    QueryPerformanceCounter(&t1);
    Sleep(1234);
    QueryPerformanceCounter(&t2);
    printf("Lasting Time: %lf\n",(t2.QuadPart-t1.QuadPart)/(double)(ts.QuadPart));
    #endif
    
    sd = system_init();
    
    if(sd == NULL)
    {
        MSG(sd->log, "sd = NULL !");
        return true;
    }

    struct timespec hrt = {
        /* seconds */
        .tv_sec  = 1,
        /* nanoseconds */  
        .tv_nsec = 1000000000UL,
    };

    sd->mot.mode = 0;
    //sd->sv.vx = 0.6f;
    sd->sv.vy = 0.6f;
    //sd->sv.w0 = 0.6f;
    
    motion_control_init(sd);
    motor_control_init(sd);

    #if 0
    motion_control_update(sd);
    motor_control_update(sd);
    
    motion_control_update(sd);
    motor_control_update(sd);
    #endif

    while(1)
    {
        perf_begin();
        motion_control_update(sd);
        motor_control_update(sd);
        //MSG(sd->log, "%s \n", sd->log);
        memset(sd->log, 0, sizeof(sd->log));
        //usleep(200000);
        //mdelay(200);
        perf_end();
        
        double d = (double)(end - start) / CLOCKS_PER_SEC;
        MSG(sd->log, "%f, %2.2f %% \n", d, (float)(d / 2.0f * 100));

        hrt.tv_nsec = 1000000000UL - (d * 1000000000UL);
        nanosleep(&hrt, NULL);
    }

    system("pause");
    return 0;
}

