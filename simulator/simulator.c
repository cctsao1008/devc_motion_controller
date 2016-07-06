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

#define LOOP_TIME 120

#define perf_begin()  clock_t start = clock()
#define perf_end()    clock_t end = clock()

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

void mdelay(unsigned int ticks)
{
    while (ticks > clock());
}

int main(int argc, char *argv[])
{
	double loading;
    clock_t ticks;
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

	mdelay(clock() + 2000);

    while(1)
    {
        perf_begin();
        motion_control_update(sd);
        motor_control_update(sd);
        memset(sd->log, 0, sizeof(sd->log));

        ticks = clock();

        fprintf(pLog, "%10ld, ", sd->t_curr);
        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ", sd->sv.vx, sd->cv.vx, sd->pv.vx);
        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ", sd->sv.vy, sd->cv.vy, sd->pv.vy);
        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ", sd->sv.w0, sd->cv.w0, sd->pv.w0);
        fprintf(pLog, "\n");

		system("cls");

        if((clock() - ticks) > LOOP_TIME)
            printf("[ERROR] log write time > LOOP_TIME !! \n");
        //else
        //    printf("[INFO] log write time = %d (ms) %d \n", clock() - ticks, ticks);

		printf("-------------------------------------------------------- \n");
		printf(" Motion control system running.. %4.2f %% %ld ms \n", (float)(loading / LOOP_TIME) * 100, ticks);
		printf("-------------------------------------------------------- \n\n");

		printf(" [INFO] PID : \n");
		printf(" [INFO] loop time = %9d (ms) \n\n", sd->t_delta);
		
        printf(" [INFO] vx : \n");
        printf(" [INFO] sv = %9.4f, cv = %9.4f, pv = %9.4f \n\n", sd->sv.vx, sd->cv.vx, sd->pv.vx);

        printf(" [INFO] vy : \n");
        printf(" [INFO] sv = %9.4f, cv = %9.4f, pv = %9.4f \n\n", sd->sv.vy, sd->cv.vy, sd->pv.vy);

        printf(" [INFO] w0 : \n");
        printf(" [INFO] sv = %9.4f, cv = %9.4f, pv = %9.4f \n\n", sd->sv.w0, sd->cv.w0, sd->pv.w0);

		//mdelay(ticks + 100);

		perf_end(); 
		
		loading = (double)(end - start);      

        mdelay(ticks + LOOP_TIME);
    }

    return 0;
}

