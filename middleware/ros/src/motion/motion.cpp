/**
 * @file motion_ros.cpp
 *
 * motion ros
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

#include "../common/system.h"
#include "../platform/platform.h"

#include "ros/ros.h"

#include <sstream>

#define DEFAULT_LOOP_TIME           200  // ms
#define BILLION                     1000000000L

void print_banner(char *data);
void* print_info_t(void *arg);

/* system... */
system_data *sd = NULL;

void timer1_cb(const ros::TimerEvent&)
{
    //ROS_INFO("update");
    motion_control_update(sd);
    motor_control_update(sd);
}

void timer2_cb(const ros::TimerEvent&)
{
    print_info_t(sd);
}

#if 0
int msleep(unsigned long milisec)
{
    struct timespec req = {0};
    time_t sec = (int)(milisec / 1000);
    milisec = milisec - (sec * 1000);
    req.tv_sec = sec;
    req.tv_nsec = milisec * 1000000L;
    while(nanosleep(&req,&req) == (-1))
         continue;
    return 1;
}
#endif

int main(int argc,char** argv)
{
    /* time, clock... */
    struct timespec start, end;
    time_t t = time(NULL);
    struct tm tm= *localtime(&t);
    float t_elap = 0.0f;
    uint64_t t_diff = 0;
    clock_t ticks = 0;

    /* Registering a node in ros master */
    ros::init(argc,argv,"motion");

    ros::NodeHandle n;

    sd = system_init();

    if(sd == NULL)
        return 0;

    /* Timers allow you to get a callback at a specified rate. */
    //ros::Timer timer1 = n.createTimer(ros::Duration(0, 80.0f * 1000 * 1000), timer1_cb);
    ros::Timer timer2 = n.createTimer(ros::Duration(1.0f), timer2_cb);

    //ROS_INFO("Welcome to ROS!");

    memset(sd, 0, sizeof(system_data));

    sd->mot.mode = 0;
    sd->loop_time = DEFAULT_LOOP_TIME;

    motion_control_init(sd);
    motor_control_init(sd);
    commander_init(sd);

    ros::Rate r(1.0); // 1 hz

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //ros::spin();

    //#if 0
    while(n.ok())
    {
        //ros::Time time = ros::Time::now();

        motion_control_update(sd);
        motor_control_update(sd);

        //Wait a duration of one second.
        ros::Duration d = ros::Duration(0, 80 * 1000 * 1000);
        d.sleep();
        ros::spinOnce();
    }
    //#endif

    return 0;
}

void print_banner(char *data)
{
    printf("%s \n\n", data);
}

void* print_info_t(void *arg)
{
    system_data *sd = (system_data *) arg;

    printf("[INFO] print_info !! \n");

    //for(;;)
    //{
        /* display system information */
        system("clear");

        //print_banner(banner);
        printf(" ROS motion is running.. \n");
        printf(" Loading %4.2f %%, Elapsed time %6.2f sec \n\n",
            sd->sys_usage, sd->sys_elaps);

        printf(" [SYSTEM] loop time (ms)        : ");
        printf(" %9d \n",
            sd->t_delta);

        printf(" [SV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->sv.vx, sd->sv.vx, sd->sv.vx);

        printf(" [CV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->cv.vy, sd->cv.vy, sd->cv.vy);

        printf(" [PV] vx, vy (m/s), w0 (rad/s)  : ");
        printf(" %9.4f %9.4f %9.4f \n",
            sd->pv.w0, sd->pv.w0, sd->pv.w0);

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
    //}
}

