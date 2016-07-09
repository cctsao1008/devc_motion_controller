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
#define EN_INFO                     true
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

void plot_reset_pxy(pos *ps)
{
    int maxx, maxy;
    pos *p;

    p = ps;

    maxx = getmaxx(); maxy = getmaxy();

    clearviewport();

    /* split window */
    // x
    moveto(   0, maxy / 2);
    setcolor(WHITE);
    lineto(maxx, maxy / 2);

    // y
    moveto(maxx / 2,    0);
    setcolor(WHITE);
    lineto(maxx / 2, maxy);

    /* draw axis */
    // x
    moveto(   0, maxy / 4);
    setcolor(GREEN);
    lineto(maxx, maxy / 4);

    moveto(       0, (maxy / 4) * 3);
    setcolor(GREEN);
    lineto(maxx / 2, (maxy / 4) * 3);



    /* reset x, y in each wiodow */
    // figure 1
    p[0].x = 0;
    p[0].y = maxy / 2;

    // figure 2
    p[1].x = maxx / 2;
    p[1].y = maxy / 2;

    // figure 3
    p[2].x = 0;
    p[2].y = maxy;

    // figure 4
    p[3].x = maxx / 2;
    p[3].y = maxy;
}
void* plot_chart(void *arg)
{
    system_data *sd = (system_data *) arg;

    int gd = DETECT, gm, x, y[4], color, angle = 0;
    int count = 0;

    initgraph(&gd, &gm, (char *)"C:\\TC\\BGI");

    pos p[4] = {0};

    int maxx, maxy;

    printf("[INFO] plot_chart !! \n");
    maxx = getmaxx(); maxy = getmaxy();

    printf("maxx = %d, maxy = %d \n", maxx, maxy);

    plot_reset_pxy(p);

    delay(2000);

    // figure 1
    y[0] = maxy / 4;

    // figure 2
    y[1] = maxy / 4;

    // figure 3
    y[2] = (maxy / 4) * 3;

    // figure 4
    y[3] = (maxy / 4) * 3;

    for(;;)
    {
        /* update figure 1, vx */
        setcolor(RED);


        moveto(p[0].x, y[0]);

        y[0] = p[0].y - (int)(sd->cv.vx * maxy / 4) - maxy / 4;
        lineto(++p[0].x, y[0]);

        /* update figure 2, vy */
        setcolor(RED);

        moveto(p[1].x, y[1]);

        y[1] = p[1].y - (int)(sd->cv.vy * maxy / 4) - maxy / 4;
        lineto(++p[1].x, y[1]);


        /* update figure 3, w0 */
        setcolor(RED);

        moveto(p[2].x, y[2]);

        y[2] = p[2].y - (int)(sd->cv.w0 * maxy / 4) - maxy / 4;
        lineto(++p[2].x, y[2]);


        /* update figure 4, vector */
        #if 0
        setcolor(RED);

        moveto(	p[3].x++,   p[3].y--);
        linerel(1, -1);
        #endif

        delay(sd->loop_time - 20);

        //if((count++ >= maxx) || (count++ >= maxy))
        if((count++ >= maxx/2))
        {
            plot_reset_pxy(p);
            count = 0;
        }

    }
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

