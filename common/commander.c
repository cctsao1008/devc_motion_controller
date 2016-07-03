/**
 * @file commander.c
 *
 * commander
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

bool commander_init(system_data* sd)
{
    pthread_t tid;
    pthread_create(&tid, NULL, (void *)&keypad_input_check, (void *)sd);

    return true;
}

bool keypad_input_check(system_data* sd)
{
    int c;

    while(1)
    {
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
        }

        if (c == KEY_2)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_2 : \n");
        }

        if (c == KEY_3)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_3 : \n");
        }

        if (c == KEY_4)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_4 : \n");
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
        }

        if (c == KEY_7)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_7 : \n");
        }

        if (c == KEY_8)
        {
            sd->sv.vx += 0.1f;
        }

        if (c == KEY_9)
        {
            //MSG(sd->log, "[DEBUG] keypad_input_check, KEY_9 : \n");
        }

        usleep(50000);
    }



    return true;
}

