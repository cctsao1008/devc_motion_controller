/**
 * @file commander.c
 *
 * commander
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

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
    pthread_create(&tid[0], NULL, (void *)&keypad_input_check, (void *)sd);
    pthread_create(&tid[1], NULL, (void *)&auto_speed_test, (void *)sd);

    return true;
}

bool keypad_input_check(system_data* sd)
{
    float vx, vy, w0, d = 0.02f;
    int c;

    while(1)
    {
        vx = sd->sv.vx, vy = sd->sv.vy, w0 = sd->sv.w0;

        c = getch();
        //MSG(sd->log, "[DEBUG] keypad_input_check, loop... (0x%X) \n", c);

        if (c == KEY_DOT)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_DOT : \n");
        }

        if (c == KEY_0)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_0 : \n");
        }

        if (c == KEY_1)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_1 : \n");
            if((vx -= d) < DEFAULT_MAX_VX * (-1.0f))
                vx = DEFAULT_MAX_VX * (-1.0f);

            if((vy -= d) < DEFAULT_MAX_VY * (-1.0f))
                vy = DEFAULT_MAX_VY * (-1.0f);

            sd->sv.vx = vx;
            sd->sv.vy = vy;

            printf("vx = %f \n", vx);
            printf("vy = %f \n", vy);
        }

        if (c == KEY_2)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_2 : \n");
            if((vx -= d) < DEFAULT_MAX_VX * (-1.0f))
                vx = DEFAULT_MAX_VX * (-1.0f);

            sd->sv.vx = vx;

            printf("vx = %f \n", vx);
        }

        if (c == KEY_3)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_3 : \n");

        }

        if (c == KEY_4)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_4 : \n");
            if((vy -= d) < DEFAULT_MAX_VY * (-1.0f))
                vy = DEFAULT_MAX_VY * (-1.0f);

            sd->sv.vy = vy;

            printf("vy = %f \n", vy);
        }

        if (c == KEY_5)
        {
            sd->sv.vx = 0.0f;
            sd->sv.vy = 0.0f;
            sd->sv.w0 = 0.0f;
        }

        if (c == KEY_6)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_6 : \n");
            if((vy += d) > DEFAULT_MAX_VY * (1.0f))
                vy = DEFAULT_MAX_VY * (1.0f);

            sd->sv.vy = vy;

            printf("vy = %f \n", vy);
        }

        if (c == KEY_7)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_7 : \n");

        }

        if (c == KEY_8)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_8 : \n");
            if((vx += d) > DEFAULT_MAX_VX * (1.0f))
                vx = DEFAULT_MAX_VX * (1.0f);

            sd->sv.vx = vx;

            printf("vx = %f \n", vx);
        }

        if (c == KEY_9)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_9 : \n");
            if((vx += d) > DEFAULT_MAX_VX)
                vx = DEFAULT_MAX_VX;

            if((vy += d) > DEFAULT_MAX_VY)
                vy = DEFAULT_MAX_VY;

            sd->sv.vx = vx;
            sd->sv.vy = vy;

            printf("vx = %f \n", vx);
            printf("vy = %f \n", vy);
        }

        usleep(50000);
    }

    return true;
}

bool auto_speed_test(system_data* sd)
{
    static bool up = 1;
    static uint8_t count = 30;

    float vx, vy, w0, d = 0.02f;

    vx = sd->sv.vx, vy = sd->sv.vy, w0 = sd->sv.w0;

    while(1)
    {
        MSG(sd->log, "[INFO] auto_speed_test... \n");

        if(up)
        {
            if((vx += d) > DEFAULT_MAX_VX)
                vx = DEFAULT_MAX_VX;

            printf("up %2d, vx = %9.4f \n", count, vx);
        }
        else
        {
            if((vx -= d) < DEFAULT_MAX_VX * (-1.0f))
                vx = DEFAULT_MAX_VX * (-1.0f);

            printf("dn %2d, vx = %9.4f \n", count, vx);
        }

        sd->sv.vx = vx;


        count++;

        if(count > 70)
        {
            count = 0;
            Sleep(1000);

            up = !up;
        }

        Sleep(1500);

        if(sd->sv.vx == 0.0f)
            Sleep(1000);
    }
}

