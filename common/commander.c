/**
 * @file commander.c
 *
 * commander
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <conio.h>

#include "system.h"
#include "..\platform\platform.h"

#define KEY_0 '0'
#define KEY_1 '1'
#define KEY_2 '2'
#define KEY_3 '3'
#define KEY_4 '4'
#define KEY_5 '5'
#define KEY_6 '6'
#define KEY_7 '7'
#define KEY_8 '8'
#define KEY_9 '9'
#define KEY_DOT '.'
#define KEY_s 's' // step input
#define KEY_r 'r' // ramp input
#define KEY_i 'i' // impulse input

#define zero(x) (x) = 0
#define inc(x) ((x += 0.02f) >  0.6f)?          DEFAULT_MAX_V : (x)
#define dec(x) ((x -= 0.02f) < -0.6f)?   (-1) * DEFAULT_MAX_V : (x)

bool keypad_input_check(system_data* sd);
bool auto_speed_test(system_data* sd);

float vx_limiter(float vx)
{
    return vx;
}

float vy_limiter(float vy)
{
    return vy;
}

float w0_limiter(float w0)
{
    return w0;
}

bool commander_init(system_data* sd)
{
    pthread_t tid[4];

    //printf("[INFO] commander_init \n");

    pthread_create(&tid[0], NULL, (void *)&keypad_input_check, (void *)sd);
    //pthread_create(&tid[1], NULL, (void *)&auto_speed_test, (void *)sd);

    return true;
}

bool keypad_input_check(system_data* sd)
{
    float vx, vy, w0, d = 0.02f;
    char c;

    //printf("[INFO] keypad_input_check \n");

    while(1)
    {
        vx = sd->sv.vx, vy = sd->sv.vy, w0 = sd->sv.w0;

        c = getche();
        system("cls");

        MSG(sd->log, "[DEBUG] keypad_input_check, loop... (%c) \n", c);

        if (c == KEY_DOT)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_DOT : \n");
            sd->sv.w0 = dec(w0);

            printf("w0 = %f \n", sd->sv.w0);
        }

        if (c == KEY_0)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_0 : \n");
            sd->sv.w0 = inc(w0);

            printf("vy = %f \n", sd->sv.w0);
        }

        if (c == KEY_1)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_1 : \n");
            sd->sv.vx = dec(vx);
            sd->sv.vy = dec(vy);

            printf("vx = %f \n", sd->sv.vx);
            printf("vy = %f \n", sd->sv.vy);
        }

        if (c == KEY_2)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_2 : \n");
            sd->sv.vx = dec(vx);

            printf("vx = %f \n", sd->sv.vx);
        }

        if (c == KEY_3)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_3 : \n");

        }

        if (c == KEY_4)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_4 : \n");
            sd->sv.vy = dec(vy);

            printf("vy = %f \n", sd->sv.vy);
        }

        if (c == KEY_5)
        {
            sd->sv.vx = zero(vx);
            sd->sv.vy = zero(vy);
            sd->sv.w0 = zero(w0);
        }

        if (c == KEY_6)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_6 : \n");
            sd->sv.vy = inc(vy);

            printf("vy = %f \n", sd->sv.vy);
        }

        if (c == KEY_7)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_7 : \n");

        }

        if (c == KEY_8)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_8 : \n");
            sd->sv.vx = inc(vx);

            printf("vx = %f \n", sd->sv.vx);
        }

        if (c == KEY_9)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_9 : \n");
            sd->sv.vx = inc(vx);
            sd->sv.vy = inc(vy);

            printf("vx = %f \n", sd->sv.vx);
            printf("vy = %f \n", sd->sv.vy);
        }

        if (c == KEY_s)
        {
            sd->sv.vx = DEFAULT_MAX_VX;
            sd->sv.vy = DEFAULT_MAX_VY;
            sd->sv.vy = DEFAULT_MAX_W0;
        }

        if (c == KEY_i)
        {
            sd->sv.vx = zero(vx);
            sd->sv.vy = zero(vy);
            sd->sv.w0 = zero(w0);

            msleep(120*2);

            sd->sv.vx = DEFAULT_MAX_VX;
            sd->sv.vy = DEFAULT_MAX_VY;
            sd->sv.w0 = DEFAULT_MAX_W0;

            msleep(120*2);

            sd->sv.vx = 0.0f;
            sd->sv.vy = 0.0f;
            sd->sv.w0 = 0.0f;
        }

        msleep(50);
    }

    return true;
}

bool auto_speed_test(system_data* sd)
{
    static bool up = 1;
    static uint8_t count = 30;

    float vx, vy, w0;

    vx = sd->sv.vx, vy = sd->sv.vy, w0 = sd->sv.w0;

    sleep(6);

    //printf("[INFO] auto_speed_test \n");

    for(;;)
    {
        MSG(sd->log, "[INFO] auto_speed_test... \n");

        if(up)
        {
            sd->sv.vx = inc(vx);

            printf("up %2d, vx = %9.4f \n", count, sd->sv.vx);
        }
        else
        {
            sd->sv.vx = dec(vx);

            printf("dn %2d, vx = %9.4f \n", count, sd->sv.vx);
        }

        count++;

        if(count > 70)
        {
            count = 0;
            Sleep(1000);

            up = !up;
        }

        sleep(2);

        if(sd->sv.vx == 0.0f)
            sleep(1);
    }
}

