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
#include <assert.h>
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

int main(int argc, char *argv[])
{
    FILE *pLog;
    char log[128] = {"log/"};

    /* use date & time as file name. */
    char log_name[64];

    time_t t = time(NULL);
    struct tm tm= *localtime(&t);

    sprintf(log_name, "%d%02d%02d%02d%02d%02d.csv", tm.tm_year+1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    pLog = fopen("log/log.txt", "w");

    if(pLog == NULL)
    {
        printf("[ERROR] fopen failure!");
        exit(0);
    }
    else
    {
        fprintf(pLog, log_name);
        fclose(pLog);
    }

    sprintf(log, "log/%s", log_name);
    printf("[INFO] log to ... %s\n", log_name);

    pLog = fopen(log, "w");

    if(pLog == NULL)
    {
        printf("[ERROR] fopen failure!");
        exit(0);
    }

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
        exit(0);

    struct timespec hrt = {
        /* seconds */
        .tv_sec  = 0,
        /* nanoseconds */
        .tv_nsec = 1000000000UL,
    };

    sd->mot.mode = 0;

    commander_init(sd);
    motion_control_init(sd);
    motor_control_init(sd);



    while(1)
    {
        perf_begin();
        motion_control_update(sd);
        motor_control_update(sd);
        memset(sd->log, 0, sizeof(sd->log));
        perf_end();

        double d = (double)(end - start) / CLOCKS_PER_SEC;
        //MSG(sd->log, "%f, %2.2f %% \n", d, (float)(d / 1.0f * 100));
        //hrt.tv_nsec = 1000000000UL - (d * 1000000000UL);
        //nanosleep(&hrt, NULL);

        sprintf(log, "%ld, %9.4f, %9.4f, %9.4f \n", sd->t_curr, sd->sv.vx, sd->cv.vx, sd->pv.vx);
        fprintf(pLog, log);
        mdelay(120);
    }

    return 0;
}

