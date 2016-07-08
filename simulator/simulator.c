/**
 * @file simulator.c
 *
 * simulator
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

#define DEFAULT_LOOP_TIME 120
//#define DEFAULT_LOOP_TIME 500

#define BILLION 1000000000L

void print_banner(char *data);

unsigned char banner[] = {
0x20, 0x5F, 0x5F, 0x20, 0x20, 0x5F, 0x5F, 0x20, 0x20, 0x20, 0x20, 0x20,
0x20, 0x20, 0x5F, 0x20, 0x20, 0x20, 0x5F, 0x20, 0x20, 0x20, 0x20, 0x20,
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x5F, 0x5F,
0x5F, 0x5F, 0x20, 0x5F, 0x5F, 0x5F, 0x20, 0x5F, 0x5F, 0x20, 0x20, 0x5F,
0x5F, 0x20, 0x0A, 0x7C, 0x20, 0x20, 0x5C, 0x2F, 0x20, 0x20, 0x7C, 0x20,
0x5F, 0x5F, 0x5F, 0x20, 0x7C, 0x20, 0x7C, 0x5F, 0x28, 0x5F, 0x29, 0x20,
0x5F, 0x5F, 0x5F, 0x20, 0x20, 0x5F, 0x20, 0x5F, 0x5F, 0x20, 0x20, 0x20,
0x2F, 0x20, 0x5F, 0x5F, 0x5F, 0x7C, 0x5F, 0x20, 0x5F, 0x7C, 0x20, 0x20,
0x5C, 0x2F, 0x20, 0x20, 0x7C, 0x0A, 0x7C, 0x20, 0x7C, 0x5C, 0x2F, 0x7C,
0x20, 0x7C, 0x2F, 0x20, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x5F, 0x5F, 0x7C,
0x20, 0x7C, 0x2F, 0x20, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x27, 0x5F, 0x20,
0x5C, 0x20, 0x20, 0x5C, 0x5F, 0x5F, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x7C,
0x7C, 0x20, 0x7C, 0x5C, 0x2F, 0x7C, 0x20, 0x7C, 0x0A, 0x7C, 0x20, 0x7C,
0x20, 0x20, 0x7C, 0x20, 0x7C, 0x20, 0x28, 0x5F, 0x29, 0x20, 0x7C, 0x20,
0x7C, 0x5F, 0x7C, 0x20, 0x7C, 0x20, 0x28, 0x5F, 0x29, 0x20, 0x7C, 0x20,
0x7C, 0x20, 0x7C, 0x20, 0x7C, 0x20, 0x20, 0x5F, 0x5F, 0x5F, 0x29, 0x20,
0x7C, 0x20, 0x7C, 0x7C, 0x20, 0x7C, 0x20, 0x20, 0x7C, 0x20, 0x7C, 0x0A,
0x7C, 0x5F, 0x7C, 0x20, 0x20, 0x7C, 0x5F, 0x7C, 0x5C, 0x5F, 0x5F, 0x5F,
0x2F, 0x20, 0x5C, 0x5F, 0x5F, 0x7C, 0x5F, 0x7C, 0x5C, 0x5F, 0x5F, 0x5F,
0x2F, 0x7C, 0x5F, 0x7C, 0x20, 0x7C, 0x5F, 0x7C, 0x20, 0x7C, 0x5F, 0x5F,
0x5F, 0x5F, 0x2F, 0x5F, 0x5F, 0x5F, 0x7C, 0x5F, 0x7C, 0x20, 0x20, 0x7C,
0x5F, 0x7C, 0x0A
};

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

void mdelay(unsigned int ticks)
{
    while (ticks > clock());
}

int main(int argc, char *argv[])
{
    char log[128] = {0};
    float t_elapsed;
    struct timespec start, end;

    uint64_t t_diff;
    clock_t ticks;
    FILE *pLog;

    /* use date & time as file name. */
    char log_name[64];

    time_t t = time(NULL);
    struct tm tm= *localtime(&t);

    sprintf(log_name, "%d%02d%02d%02d%02d%02d.csv", tm.tm_year + 1900,
													tm.tm_mon + 1,
													tm.tm_mday,
													tm.tm_hour,
													tm.tm_min,
													tm.tm_sec);

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

    print_banner(banner);

    #if 0  // REF.
    int t1, t2, ts;
    QueryPerformance Frequency(&ts);
    QueryPerformanceCounter(&t1);
    Sleep(1234);
    QueryPerformanceCounter(&t2);
    printf("Lasting Time: %lf\n",(t2.QuadPart-t1.QuadPart)/(double)(ts.QuadPart));
    #endif

    sd = system_init();

    if(sd == NULL)
        exit(0);

    sd->mot.mode = 0;
    sd->loop_time = DEFAULT_LOOP_TIME;

    motion_control_init(sd);
    motor_control_init(sd);

    commander_init(sd);

    mdelay(clock() + 3000);

    while(1)
    {

        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &start);	/* mark start time */

        motion_control_update(sd);
        motor_control_update(sd);
        //memset(sd->log, 0, sizeof(sd->log));

        ticks = clock();

        t_elapsed += sd->t_delta;

        /* display system information */
        system("cls");

        print_banner(banner);

        printf(" Motion Simulator running.. \n");
        printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n", (t_diff / (float)((sd->loop_time) * 1000000)) * 100, t_elapsed / 1000);

        printf(" [loop time] ms                 : ");
        printf(" %9d \n",
            sd->t_delta);

        printf(" [vx] sv, cv, pv (m/s)          : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->sv.vx, sd->cv.vx, sd->pv.vx);

        printf(" [vy] sv, cv, pv (m/s)          : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->sv.vy, sd->cv.vy, sd->pv.vy);

        printf(" [w0] sv, cv, pv (rad/s)        : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->sv.w0, sd->cv.w0, sd->pv.w0);

        printf(" [rpm/o] m1, m2, m3, m4 (r/min) : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.out.rpm1, sd->mot.out.rpm2,
            sd->mot.out.rpm3, sd->mot.out.rpm4);

        printf(" [rpm/i] m1, m2, m3, m4 (r/min) : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.in.rpm1, sd->mot.in.rpm2,
            sd->mot.in.rpm3, sd->mot.in.rpm4);

        //mdelay(ticks + 50);

        /* log system data */
        fprintf(pLog, "%10ld, ",
            sd->t_curr);

        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ",
            sd->sv.vx, sd->cv.vx, sd->pv.vx);

        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ",
            sd->sv.vy, sd->cv.vy, sd->pv.vy);

        fprintf(pLog, "%9.4f, %9.4f, %9.4f, ",
            sd->sv.w0, sd->cv.w0, sd->pv.w0);

        fprintf(pLog, "\n");

        if((clock() - ticks) > (sd->loop_time))
            printf("[ERROR] log write time > LOOP_TIME !! \n");
        //else
        //    printf("[INFO] log write time = %d (ms) %d \n", clock() - ticks, ticks);

        clock_gettime(CLOCK_MONOTONIC, &end);	/* mark the end time */

        t_diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
        //printf(" [INFO] elapsed time = %llu nanoseconds\n", (long long unsigned int) t_diff);

        mdelay(ticks + (sd->loop_time));
    }

    return 0;
}

void print_banner(char *data)
{
    printf("%s \n", data);
}

