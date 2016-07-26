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
#include <SDL2/SDL.h>

#include "../common/system.h"
#include "../platform/platform.h"

#define DEFAULT_SERIAL_PORT         "\\\\.\\COM6"
#define DEFAULT_LOOP_TIME           50
//#define DEFAULT_LOOP_TIME       500

#define BILLION      1000000000L

#define EN_INFO_F    false
#define EN_INFO_T    true
#define EN_INFO_C    true

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

/* print info */
void* print_info_c(void *arg); // chart
void* print_info_t(void *arg); // text
void* print_info_f(FILE *fp, system_data *sd); // file

pthread_mutex_t mutex[2];

int main(int argc, char *argv[])
{
    /* time, clock... */
    struct timespec start, end;
    time_t t = time(NULL);
    struct tm tm= *localtime(&t);
    double  t_elap = 0.0f;
    uint64_t t_diff = 0;
    clock_t ticks = 0;

    /* file... */
    char log[128] = {0};
    char log_name[64];
    FILE *pLog = NULL;

    /* thread... */
    pthread_t tid[2] = {0};

    /* system... */
    system_data *sd = NULL;

    /* serial port */
    char port[64] = DEFAULT_SERIAL_PORT;

    /* SDL2 test code */
    #if 0   // REF.
    const int SCREEN_WIDTH = 640;
    const int SCREEN_HEIGHT = 480;
    SDL_Window *window = NULL;
    SDL_Surface *screenSurface = NULL;

    if(SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
    }
    else
    {
        window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

        if(window == NULL)
        {
            printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        }
        else
        {
            screenSurface = SDL_GetWindowSurface(window);
            SDL_FillRect(screenSurface, NULL, SDL_MapRGB( screenSurface->format, 0xFF, 0xFF, 0xFF));
            SDL_UpdateWindowSurface(window);
            SDL_Delay(2000);
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    exit(0);
    #endif

    /* use date & time as file name. */
    #if EN_INFO_F
    sprintf(log_name, "%d%02d%02d%02d%02d%02d.csv", tm.tm_year + 1900,
                                                    tm.tm_mon + 1,
                                                    tm.tm_mday,
                                                    tm.tm_hour,
                                                    tm.tm_min,
                                                    tm.tm_sec);
    /* data logger */
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

    /* REF */
    #if 0
    srand((unsigned) time(NULL) + getpid());
    #endif

    print_banner(banner);

    /* REF */
    #if 0
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

    memset(sd, 0, sizeof(system_data));
    memset(&start, 0, sizeof(timespec));
    memset(&end, 0, sizeof(timespec));

    sd->mot.mode = 0;
    sd->loop_time = DEFAULT_LOOP_TIME;
    sd->port = port;

    motion_control_init(sd);
    motor_control_init(sd);
    commander_init(sd);

    pthread_mutex_init(&mutex[0], NULL);
    pthread_mutex_init(&mutex[1], NULL);

    /* Text info */
    #if EN_INFO_T
    pthread_create(&tid[0], NULL, &print_info_t, (void *)sd);
    #endif

    /* plot chart */
    #if EN_INFO_C
    pthread_create(&tid[1], NULL, &print_info_c, (void *)sd);
    #endif

    for(;;)
    {
        ticks = clock();

        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */

        motion_control_update(sd);
        motor_control_update(sd);

        #if EN_INFO_F
        print_info_f(pLog, sd);
        #endif

        if((clock() - ticks) > (sd->loop_time))
            printf("[ERROR] log write time > LOOP_TIME !! \n");

        clock_gettime(CLOCK_MONOTONIC, &end);   /* mark the end time */

        t_diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
        t_elap += sd->t_delta;

        sd->sys_usage = (t_diff / (float)((sd->loop_time) * 1000000)) * 100;
        sd->sys_elaps = t_elap / 1000.0f;

        pthread_mutex_unlock(&mutex[0]);
        pthread_mutex_unlock(&mutex[1]);

        msleep(DEFAULT_LOOP_TIME);
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
//#define SW4 3

enum{SV, CV, PV};

void* print_info_c(void *arg)
{
    system_data *sd = (system_data *) arg;
    void *bitimage[4];
    char text[64];

    int gd = DETECT, gm, size = 0;
    int sw_x1[4], sw_y1[4], sw_x2[4], sw_y2[4], sw_ymid[4];
    int x1[4], y1[4][3], x2[4], y2[4];

    initwindow(640, 480 + 30, "Motion Simulator V1.0 (tsao.ricardo@iac.com.tw)", true, false);

    pos p[4] = {0};

    int maxx, maxy;

    printf("[INFO] plot_chart !! \n");
    maxx = getmaxx(); maxy = getmaxy() - 30;

    printf("maxx = %d, maxy = %d \n", maxx, maxy);

    /* plot sub windows */

    /* sub window 1 */
    #ifdef SW1
    printf("[INFO] initializing sub window 1... \n");
    sw_x1[SW1] = 1;
    sw_y1[SW1] = 0;
    sw_x2[SW1] = maxx / 2;
    sw_y2[SW1] = maxy / 2;
    sw_ymid[SW1] = maxy / 4;

    x1[SW1] = (maxx / 2) - 1;
    y1[SW1][SV] = maxy / 4;
    y1[SW1][CV] = maxy / 4;
    y1[SW1][PV] = maxy / 4;
    x2[SW1] = maxx / 2;
    y2[SW1] = maxy / 4;

    size = imagesize(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1]);

    bitimage[SW1] = malloc(size);
    getimage(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1], bitimage[SW1]);
    #endif

    /* sub window 2 */
    #ifdef SW2
    printf("[INFO] initializing sub window 2... \n");
    sw_x1[SW2] = maxx / 2 + 1;
    sw_y1[SW2] = 0;
    sw_x2[SW2] = maxx;
    sw_y2[SW2] = maxy / 2;
    sw_ymid[SW2] = maxy / 4;

    x1[SW2] = maxx - 1;
    y1[SW2][SV] = maxy / 4;
    y1[SW2][CV] = maxy / 4;
    y1[SW2][PV] = maxy / 4;
    x2[SW2] = maxx;
    y2[SW2] = maxy / 4;

    size = imagesize(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2]);

    bitimage[SW2] = malloc(size);
    getimage(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2], bitimage[SW2]);
    #endif

    /* sub window 3 */
    #ifdef SW3
    printf("[INFO] initializing sub window 3... \n");
    sw_x1[SW3] = 1;
    sw_y1[SW3] = maxy / 2;
    sw_x2[SW3] = maxx / 2;
    sw_y2[SW3] = maxy;
    sw_ymid[SW3] = (maxy / 4) * 3;

    x1[SW3] = (maxx / 2) - 1;
    y1[SW3][SV] = (maxy / 4) * 3;
    y1[SW3][CV] = (maxy / 4) * 3;
    y1[SW3][PV] = (maxy / 4) * 3;
    x2[SW3] = maxx / 2;
    y2[SW3] = (maxy / 4) * 3;

    size = imagesize(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3]);

    bitimage[SW3] = malloc(size);
    getimage(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3], bitimage[SW3]);
    #endif

    /* sub window 4 */
    #ifdef SW4
    printf("[INFO] initializing sub window 4... \n");
    #endif

    /* update all sub windows */
    for(;;)
    {
        pthread_mutex_lock(&mutex[0]);

        clearviewport();
        //cleardevice();
        setlinestyle(SOLID_LINE, 0xFFFF, 1);

        /*
             update sub window 1, vx
         */
        #ifdef SW1
        setcolor(WHITE);

        moveto(x1[SW1], y1[SW1][PV]);
        putimage(sw_x1[SW1] - 1, sw_y1[SW1], bitimage[SW1], XOR_PUT);

        y2[SW1] = sw_ymid[SW1] - (sd->pv.vx * (maxy / 4));
        lineto(x2[SW1], y2[SW1]);

        y1[SW1][PV] = y2[SW1];
        getimage(sw_x1[SW1], sw_y1[SW1], sw_x2[SW1], sw_y2[SW1], bitimage[SW1]);
        #endif

        /*
             update sub window 2, vy
         */
        #ifdef SW2
        setcolor(WHITE);
        moveto(x1[SW2], y1[SW2][PV]);
        putimage(sw_x1[SW2] - 1, sw_y1[SW2], bitimage[SW2], XOR_PUT);

        y2[SW2] = sw_ymid[SW2] - (sd->pv.vy * (maxy / 4));
        lineto(x2[SW2], y2[SW2]);

        y1[SW2][PV] = y2[SW2];
        getimage(sw_x1[SW2], sw_y1[SW2], sw_x2[SW2], sw_y2[SW2], bitimage[SW2]);
        #endif

        /*
             update sub window 3, w0
         */
        #ifdef SW3
        setcolor(WHITE);
        moveto(x1[SW3], y1[SW3][PV]);
        putimage(sw_x1[SW3] - 1, sw_y1[SW3], bitimage[SW3], XOR_PUT);

        y2[SW3] = sw_ymid[SW3] - (sd->pv.w0 * (maxy / 4));
        lineto(x2[SW3], y2[SW3]);

        y1[SW3][PV] = y2[SW3];
        getimage(sw_x1[SW3], sw_y1[SW3], sw_x2[SW3], sw_y2[SW3], bitimage[SW3]);
        #endif

        /*
             update sub window 4, vector
         */
        #ifdef SW4
        //printf("[INFO] updating sub window 4... \n");
        #endif

        #if 1
        setlinestyle(SOLID_LINE, 0xFFFF, 2);
        /* draw axis x */
        moveto(0, maxy / 4 + 1);
        setcolor(GREEN);
        lineto(maxx, maxy / 4 + 1);

        /* draw axis y */
        moveto(0, (maxy / 4) * 3 + 1);
        setcolor(GREEN);
        lineto(maxx / 2, (maxy / 4) * 3 + 1);

        /* split window */
        setlinestyle(SOLID_LINE, 0xFFFF, 2);
        moveto(0, maxy / 2);
        setcolor(RED);
        lineto(maxx, maxy / 2);
        rectangle(1, 1, maxx, maxy);

        moveto(maxx / 2, 0);
        setcolor(RED);
        lineto(maxx / 2, maxy);

        /* add text (SV) */
        setcolor(WHITE);
        settextstyle(BOLD_FONT, HORIZ_DIR, 1);
        sprintf(text, "vx = %5.2f (m/s)", sd->sv.vx);
        outtextxy(sw_x1[SW1] + 8, sw_y1[SW1] + 6, text);
        sprintf(text, "vy = %5.2f (m/s)", sd->sv.vy);
        outtextxy(sw_x1[SW2] + 8, sw_y1[SW2] + 6, text);
        sprintf(text, "w0 = %5.2f (rad/s)", sd->sv.w0);
        outtextxy(sw_x1[SW3] + 8, sw_y1[SW3] + 6, text);

        /* add text (PV) */
        setcolor(YELLOW);
        settextstyle(BOLD_FONT, HORIZ_DIR, 1);
        sprintf(text, "%5.2f", sd->pv.vx);
        outtextxy(sw_x2[SW1] - 60, y2[SW1] - 30, text);
        sprintf(text, "%5.2f", sd->pv.vy);
        outtextxy(sw_x2[SW2] - 60, y2[SW2] - 30, text);
        sprintf(text, "%5.2f", sd->pv.w0);
        outtextxy(sw_x2[SW3] - 60, y2[SW3] - 30, text);

        /* add status bar */
        setfillstyle(SOLID_FILL, RED);
        bar(0, maxy + 1, maxx + 1, maxy + 30);
        setcolor(WHITE);
        setbkcolor(RED);
        sprintf(text, " time %7.2f sec | perf %5.2f%% | loop %7.2f ms" , sd->sys_elaps, sd->sys_usage, sd->t_delta);
        outtextxy(0, maxy + 6, text);
        setbkcolor(BLACK);
        #endif

        swapbuffers();
        //delay(DEFAULT_LOOP_TIME / 2); // nyquist sample theorem
        delay(200);
    }
}

void* print_info_t(void *arg)
{
    system_data *sd = (system_data *) arg;

    printf("[INFO] print_info !! \n");

    for(;;)
    {
        //pthread_mutex_lock(&mutex[1]);

        /* display system information */
        system("cls");

        print_banner(banner);
        printf(" Motion Simulator is running.. \n");
        printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n",
            sd->sys_usage, sd->sys_elaps);

        printf(" [SYSTEM] loop time (ms)        : ");
        printf(" %9.2f \n",
            sd->t_delta);

        printf(" [SV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->sv.vx, sd->sv.vy, sd->sv.w0);

        printf(" [CV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->cv.vx, sd->cv.vy, sd->cv.w0);

        printf(" [PV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->pv.vx, sd->pv.vy, sd->pv.w0);

        printf(" [fr] m1, m2, m3, m4 (f/r)      : ");
        printf(" %9d %9d %9d %9d \n",
            sd->mot.out.fr1, sd->mot.out.fr2,
            sd->mot.out.fr3, sd->mot.out.fr4);

        printf(" [w/o] m1, m2, m3, m4 (rad/s)   : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.out.w1, sd->mot.out.w2,
            sd->mot.out.w3, sd->mot.out.w4);

        printf(" [w/i] m1, m2, m3, m4 (rad/s)   : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.in.w1, sd->mot.in.w2,
            sd->mot.in.w3, sd->mot.in.w4);

        printf(" [rpm/o] m1, m2, m3, m4 (r/min) : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.out.rpm1, sd->mot.out.rpm2,
            sd->mot.out.rpm3, sd->mot.out.rpm4);

        printf(" [rpm/i] m1, m2, m3, m4 (r/min) : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.in.rpm1, sd->mot.in.rpm2,
            sd->mot.in.rpm3, sd->mot.in.rpm4);

        printf(" [pwm/o] m1, m2, m3, m4         : ");
        printf(" %9.4f %9.4f %9.4f %9.4f \n",
            sd->mot.out.pwm1, sd->mot.out.pwm2,
            sd->mot.out.pwm3, sd->mot.out.pwm4);

        printf("\n\n");

        sleep(1);
        //msleep(500);
    }
}

void* print_info_f(FILE *fp, system_data *sd)
{
    /* data logger */
    fprintf(fp, "%9.4f, ",
        sd->sys_elaps);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.vx, sd->cv.vx, sd->pv.vx);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.vy, sd->cv.vy, sd->pv.vy);

    fprintf(fp, "%9.4f, %9.4f, %9.4f, ",
        sd->sv.w0, sd->cv.w0, sd->pv.w0);

    fprintf(fp, "\n");
}

