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

#define DEFAULT_LOOP_TIME           120
//#define DEFAULT_LOOP_TIME       500

#define BILLION                     1000000000L

#define EN_LOGGER                   false
#define EN_INFO                     false
#define EN_CHART                    true

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
void data_logger(FILE *fp, system_data *sd);

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
    struct timespec start, end;

    float t_elap;
    uint64_t t_diff;
    clock_t ticks;
    FILE *pLog;

    /* use date & time as file name. */
    char log_name[64];

    time_t t = time(NULL);
    struct tm tm= *localtime(&t);

    #if EN_LOGGER
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
    #endif

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

    for(;;)
    {

        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */

        motion_control_update(sd);
        motor_control_update(sd);

        ticks = clock();

        #if EN_LOGGER
        data_logger(pLog, sd);
        #endif

        if((clock() - ticks) > (sd->loop_time))
            printf("[ERROR] log write time > LOOP_TIME !! \n");

        clock_gettime(CLOCK_MONOTONIC, &end);   /* mark the end time */

        t_diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
        t_elap += sd->t_delta;

        sd->sys_usage = (t_diff / (float)((sd->loop_time) * 1000000)) * 100;
        sd->sys_elaps = t_elap / 1000;

        mdelay(ticks + (sd->loop_time));
    }

    return 0;
}

void print_banner(char *data)
{
    printf("%s \n\n", data);
}

typedef struct _pos
{
    int x;
    int y;
} pos;

/*
   _________________________________
  | window 1       | window 2       |
  | vx             | vy             |
  |                |                |
  |________________|________________|
  | window 3       | window 4       |
  | w0             | vecotor        |
  |                |                |
  |________________|________________|

 */

#define SW1 0
#define SW2 1
#define SW3 2
#define SW4 3

void* plot_chart(void *arg)
{
    system_data *sd = (system_data *) arg;
    void *bitimage[4];

    int gd = DETECT, gm, size = 0;
    int sw_x1[4], sw_y1[4], sw_x2[4], sw_y2[4], sw_ymid[4];
    int x1[4], y1[4], x2[4], y2[4];

    initgraph(&gd, &gm, (char *)"C:\\TC\\BGI");
    //initwindow(640, 480, "First Sample");
    //setbkcolor(BLUE);

    pos p[4] = {0};

    int maxx, maxy;

    printf("[INFO] plot_chart !! \n");
    maxx = getmaxx(); maxy = getmaxy();

    printf("maxx = %d, maxy = %d \n", maxx, maxy);

    printf("%d %d %d %d \n", SW1, SW2, SW3, SW4);


    delay(2000);

    /* plot sub windows */

    /* sub window 1 */
    sw_x1[SW1] = 1;
    sw_y1[SW1] = 0;
    sw_x2[SW1] = maxx / 2;
    sw_y2[SW1] = maxy / 2;
    sw_ymid[SW1] = maxy / 4;

    x1[SW1] = (maxx / 2) - 1;
    y1[SW1] = maxy / 4;
    x2[SW1] = maxx / 2;
    y2[SW1] = maxy / 4;

    size = imagesize(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1]);

    bitimage[SW1] = malloc(size);
    getimage(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1], bitimage[SW1]);

    /* sub window 2 */

    sw_x1[SW2] = maxx / 2 + 1;
    sw_y1[SW2] = 0;
    sw_x2[SW2] = maxx;
    sw_y2[SW2] = maxy / 2;
    sw_ymid[SW2] = maxy / 4;

    x1[SW2] = maxx - 1;
    y1[SW2] = maxy / 4;
    x2[SW2] = maxx;
    y2[SW2] = maxy / 4;

    size = imagesize(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2]);

    bitimage[SW2] = malloc(size);
    getimage(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2], bitimage[SW2]);

    /* sub window 3 */
    sw_x1[SW3] = 1;
    sw_y1[SW3] = maxy / 2;
    sw_x2[SW3] = maxx / 2;
    sw_y2[SW3] = maxy;
    sw_ymid[SW3] = (maxy / 4) * 3;

    x1[SW3] = (maxx / 2) - 1;
    y1[SW3] = (maxy / 4) * 3;
    x2[SW3] = maxx / 2;
    y2[SW3] = (maxy / 4) * 3;

    size = imagesize(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3]);

    bitimage[SW3] = malloc(size);
    getimage(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3], bitimage[SW3]);

    /* sub window 4 */

    /* update all sub windows */
    for(;;)
    {
        clearviewport();

        /*
             update sub window 1, vx
         */
        setcolor(WHITE);

        moveto(x1[SW1], y1[SW1]);

        putimage(sw_x1[SW1] - 1, sw_y1[SW1], bitimage[SW1], COPY_PUT);

        y2[SW1] = sw_ymid[SW1] - 50;

        lineto(x2[SW1], y2[SW1]);

        y1[SW1] = y2[SW1];

        getimage(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1], bitimage[SW1]);

        /*
             update sub window 2, vy
         */
        #if 1
        setcolor(WHITE);

        moveto(x1[SW2], y1[SW2]);

        putimage(sw_x1[SW2] - 1, sw_y1[SW2], bitimage[SW2], COPY_PUT);

        y2[SW2] = sw_ymid[SW2] - 50;

        lineto(x2[SW2], y2[SW2]);

        y1[SW2] = y2[SW2];

        getimage(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2], bitimage[SW2]);
        #endif

        /*
             update sub window 3, w0
         */
        #if 1
        setcolor(WHITE);

        moveto(x1[SW3], y1[SW3]);

        putimage(sw_x1[SW3] - 1, sw_y1[SW3], bitimage[SW3], COPY_PUT);

        y2[SW3] = sw_ymid[SW3] - 50;

        lineto(x2[SW3], y2[SW3]);

        y1[SW3] = y2[SW3];

        getimage(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3], bitimage[SW3]);
        #endif

        /*
             update sub window 4, vector
         */

        #if 1
        /* draw axis x */
        moveto(0, maxy / 4);
        setcolor(GREEN);
        lineto(maxx, maxy / 4);

        /* draw axis y */
        moveto(0, (maxy / 4) * 3 + 2);
        setcolor(GREEN);
        lineto(maxx / 2, (maxy / 4) * 3 + 2);

        /* split window */
        moveto(0, maxy / 2);
        setcolor(RED);
        lineto(maxx, maxy / 2);

        moveto(maxx / 2, 0);
        setcolor(RED);
        lineto(maxx / 2, maxy);
        #endif

        delay(sd->loop_time - 20);
    }
}

void plot_init(system_data *sd)
{
    /* test winbgim */
    pthread_t tid[2];

    #if EN_INFO
    pthread_create(&tid[0], NULL, &print_info, (void *)sd);
    #endif

    #if EN_CHART
    pthread_create(&tid[1], NULL, &plot_chart, (void *)sd);
    #endif

}

void data_logger(FILE *fp, system_data *sd)
{
    /* data logger */
    fprintf(fp, "%10ld, ",
        sd->t_curr);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.vx, sd->cv.vx, sd->pv.vx);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.vy, sd->cv.vy, sd->pv.vy);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.w0, sd->cv.w0, sd->pv.w0);

    fprintf(fp, "\n");
}

void* print_info(void *arg)
{
    system_data *sd = (system_data *) arg;

    printf("[INFO] print_info !! \n");

    for(;;)
    {
        #if 1
        /* display system information */
        system("cls");

        print_banner(banner);
        printf(" Motion Simulator running.. \n");
        printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n", sd->sys_usage, sd->sys_elaps);

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

        printf("\n\n");
        #endif

        usleep(500000); // 50ms
    }
}

