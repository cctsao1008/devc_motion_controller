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
#include <unistd.h>
#include <pthread.h>
#include <graphics.h>
//#include <winbgim.h>

#include "..\common\system.h"
#include "..\platform\platform.h"

#define DEFAULT_LOOP_TIME 120
//#define DEFAULT_LOOP_TIME 500

#define BILLION 1000000000L

using namespace std;

char banner[] = {
    0x20, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x0A, 0x20, 0x20, 0x5F, 0x5F, 0x20, 0x20, 0x5F,
    0x5F, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x5F, 0x20, 0x20, 0x20,
    0x5F, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x5F, 0x5F, 0x5F, 0x5F, 0x20, 0x5F, 0x5F, 0x5F,
    0x20, 0x5F, 0x5F, 0x20, 0x20, 0x5F, 0x5F, 0x20, 0x0A, 0x20, 0x7C, 0x20,
    0x20, 0x5C, 0x2F, 0x20, 0x20, 0x7C, 0x20, 0x5F, 0x5F, 0x5F, 0x20, 0x7C,
    0x20, 0x7C, 0x5F, 0x28, 0x5F, 0x29, 0x20, 0x5F, 0x5F, 0x5F, 0x20, 0x20,
    0x5F, 0x20, 0x5F, 0x5F, 0x20, 0x20, 0x20, 0x2F, 0x20, 0x5F, 0x5F, 0x5F,
    0x7C, 0x5F, 0x20, 0x5F, 0x7C, 0x20, 0x20, 0x5C, 0x2F, 0x20, 0x20, 0x7C,
    0x20, 0x20, 0x20, 0x4D, 0x6F, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x53, 0x49,
    0x4D, 0x20, 0x3A, 0x0A, 0x20, 0x7C, 0x20, 0x7C, 0x5C, 0x2F, 0x7C, 0x20,
    0x7C, 0x2F, 0x20, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x5F, 0x5F, 0x7C, 0x20,
    0x7C, 0x2F, 0x20, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x27, 0x5F, 0x20, 0x20,
    0x5C, 0x20, 0x5C, 0x5F, 0x5F, 0x5F, 0x20, 0x5C, 0x7C, 0x20, 0x7C, 0x7C,
    0x20, 0x7C, 0x5C, 0x2F, 0x7C, 0x20, 0x7C, 0x20, 0x20, 0x20, 0x20, 0x4D,
    0x6F, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x63, 0x6F, 0x6E, 0x74, 0x72, 0x6F,
    0x6C, 0x6C, 0x65, 0x72, 0x20, 0x73, 0x69, 0x6D, 0x75, 0x6C, 0x61, 0x74,
    0x6F, 0x72, 0x2E, 0x0A, 0x20, 0x7C, 0x20, 0x7C, 0x20, 0x20, 0x7C, 0x20,
    0x7C, 0x20, 0x28, 0x5F, 0x29, 0x20, 0x7C, 0x20, 0x7C, 0x5F, 0x7C, 0x20,
    0x7C, 0x20, 0x28, 0x5F, 0x29, 0x20, 0x7C, 0x20, 0x7C, 0x20, 0x7C, 0x20,
    0x7C, 0x20, 0x20, 0x5F, 0x5F, 0x5F, 0x29, 0x20, 0x7C, 0x20, 0x7C, 0x7C,
    0x20, 0x7C, 0x20, 0x20, 0x7C, 0x20, 0x7C, 0x0A, 0x20, 0x7C, 0x5F, 0x7C,
    0x20, 0x20, 0x7C, 0x5F, 0x7C, 0x5C, 0x5F, 0x5F, 0x5F, 0x2F, 0x20, 0x5C,
    0x5F, 0x5F, 0x7C, 0x5F, 0x7C, 0x5C, 0x5F, 0x5F, 0x5F, 0x2F, 0x7C, 0x5F,
    0x7C, 0x20, 0x7C, 0x5F, 0x7C, 0x20, 0x7C, 0x5F, 0x5F, 0x5F, 0x5F, 0x2F,
    0x5F, 0x5F, 0x5F, 0x7C, 0x5F, 0x7C, 0x20, 0x20, 0x7C, 0x5F, 0x7C, 0x20,
    0x20, 0x20, 0x52, 0x69, 0x63, 0x61, 0x72, 0x64, 0x6F, 0x2C, 0x20, 0x74,
    0x73, 0x61, 0x6F, 0x2E, 0x72, 0x69, 0x63, 0x61, 0x72, 0x64, 0x6F, 0x40,
    0x69, 0x61, 0x63, 0x2E, 0x63, 0x6F, 0x6D, 0x2E, 0x74, 0x77, 0x0A, 0x20,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F,
    0x5F, 0x5F, 0x5F
};

void print_banner(char *data);

/* plot chart */
void plot_init(system_data *sd);
void* plot_chart(void*);

void* print_info(void *arg);

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

    //srand((unsigned) time(NULL) + getpid());

    print_banner(banner);

    //while(1);

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

    plot_init(sd);

    mdelay(clock() + 3000);

    while(1)
    {

        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */

        motion_control_update(sd);
        motor_control_update(sd);
        //memset(sd->log, 0, sizeof(sd->log));

        ticks = clock();

        #if 1
        //printf(" Motion Simulator running.. \n");
        //printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n", (t_diff / (float)((sd->loop_time) * 1000000)) * 100, t_elapsed / 1000);

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
        #endif

        if((clock() - ticks) > (sd->loop_time))
            printf("[ERROR] log write time > LOOP_TIME !! \n");
        //else
        //    printf("[INFO] log write time = %d (ms) %d \n", clock() - ticks, ticks);

        clock_gettime(CLOCK_MONOTONIC, &end);   /* mark the end time */

        t_diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
        //printf(" [INFO] elapsed time = %llu nanoseconds\n", (long long unsigned int) t_diff);

        t_elapsed += sd->t_delta;

        sd->sys_usage = (t_diff / (float)((sd->loop_time) * 1000000)) * 100;
        sd->sys_elapsed_t = t_elapsed / 1000;

        mdelay(ticks + (sd->loop_time));
    }

    return 0;
}

void print_banner(char *data)
{
    printf("%s \n\n", data);
}

void* plot_chart(void *arg)
{
    int gd = DETECT, gm, x, y, color, angle = 0;
    initgraph(&gd, &gm, (char *)"C:\\TC\\BGI");

    int line_x[3] = {0};
    int line_y[3] = {0};

    printf("[INFO] plot_chart !! \n");
    printf("maxx = %d, maxy = %d \n", getmaxx(), getmaxy());
    moveto(0, getmaxy());

    line_y[0] = getmaxy(); line_y[1] = getmaxy(); line_y[2] = getmaxy();

    delay(2000);

    while(1)
    {
        // line 1
        setcolor(RED);

        moveto(	line_x[0], 	line_y[0]);
        line_x[0] = line_x[0] + 1;  line_y[0] =  line_y[0] -1;
        linerel(1, -1);

        // line 2
        setcolor(GREEN);

        moveto(	line_x[1], 	line_y[1]);
        line_x[1] = line_x[1] + 1;  line_y[1] =  line_y[1] -2;
        linerel(1, -2);

        delay(1000);
    }
}

void* print_info(void *arg)
{
    system_data *sd = (system_data *) arg;

    printf("[INFO] print_info !! \n");

    while(1)
    {
        #if 1
        /* display system information */
        system("cls");

        print_banner(banner);
        printf(" Motion Simulator running.. \n");
        printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n", sd->sys_usage, sd->sys_elapsed_t);

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

        printf(" [pwm/o] m1, m2, m3, m4         : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.out.pwm1, sd->mot.out.pwm2,
            sd->mot.out.pwm3, sd->mot.out.pwm4);

        printf(" [w/i] m1, m2, m3, m4 (r/min)   : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.in.w1, sd->mot.in.w2,
            sd->mot.in.w3, sd->mot.in.w4);
        #endif

        usleep(500000); // 50ms
    }
}

void plot_init(system_data *sd)
{
    /* test winbgim */
    pthread_t tid[2];

    pthread_create(&tid[0], NULL, &print_info, (void *)sd);
    pthread_create(&tid[1], NULL, &plot_chart, (void *)sd);

}

