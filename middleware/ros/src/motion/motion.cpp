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
#include <geometry_msgs/Twist.h>

#include <sstream>

#define DEFAULT_LOOP_TIME           80.0f  // ms
#define BILLION                     1000000000L

void print_banner(char *data);
void print_info_t(void *arg);

/* system... */
system_data *sd = NULL;

void timer1_cb(const ros::TimerEvent&)
{
    //ROS_INFO("update");
    //motion_control_update(sd);
    //motor_control_update(sd);
}

void timer2_cb(const ros::TimerEvent&)
{
    print_info_t(sd);
}

void vel_cb(const geometry_msgs::Twist& msg)
{
    sd->sv.vx = msg.linear.x;
    sd->sv.vy = msg.linear.y;
    sd->sv.w0 = msg.angular.z;

}

void* ros_cb(void *arg)
{
    for(;;)
    {
        //ros::spin();
        ros::spinOnce();
        msleep(50);
    }
}

#if 1
int mdelay(unsigned long milisec)
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

    pthread_t tid;

    /* Registering a node in ros master */
    ros::init(argc,argv,"motion");

    ros::NodeHandle n;
    ros::Subscriber s;

    sd = system_init();

    if(sd == NULL)
        return 0;

    memset(sd, 0, sizeof(system_data));

    sd->mot.mode = 0;
    sd->loop_time = DEFAULT_LOOP_TIME;

    motion_control_init(sd);
    motor_control_init(sd);
    commander_init(sd);

    pthread_create(&tid, NULL, &ros_cb, (void *)sd);

    /* Timers allow you to get a callback at a specified rate. */
    //ros::Timer timer1 = n.createTimer(ros::Duration(0, 80.0f * 1000 * 1000), timer1_cb);
    ros::Timer timer2 = n.createTimer(ros::Duration(0.5f), timer2_cb);

    ros::NodeHandle nh_param("~");
    s = n.subscribe("cmd_vel", 100, &vel_cb);

    //ROS_INFO("Welcome to ROS!");

    ros::Rate r(1.0); // 1 hz
    ros::Duration d = ros::Duration(0, 20 * 1000 * 1000);

    ros::Time current_time, last_time((sd->loop_time /2) * 1000 * 1000);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //ros::spin();

    //#if 0
    while(ros::ok())
    //for(;;)
    {
        ticks = clock();

        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */

        motion_control_update(sd);
        motor_control_update(sd);

        //ros::spinOnce();

        //if((clock() - ticks) > (sd->loop_time))
        //    printf("\n\n\n[ERROR] time > LOOP_TIME !! \n");

        clock_gettime(CLOCK_MONOTONIC, &end);   /* mark the end time */

        t_diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
        t_elap += (float)sd->t_delta;

        sd->sys_usage = (t_diff / (float)((sd->loop_time) * 1000000)) * 100;
        sd->sys_elaps = t_elap / 1000.0f;

        //d.sleep();
        //usleep(5 * 1000);
        #if 1
        struct timeval delay;
        delay.tv_sec = 0;
        delay.tv_usec = 10 * 1000;
        select(0, NULL, NULL, NULL, &delay);
        #endif
        //mdelay(sd->loop_time);
    }
    //#endif

    return 0;
}

void print_banner(char *data)
{
    printf("%s \n\n", data);
}

void print_info_t(void *arg)
{
    system_data *sd = (system_data *) arg;

    printf("[INFO] print_info !! \n");

    //for(;;)
    //{
        /* display system information */
        system("clear");

        //print_banner(banner);
        printf(" ROS motion running.. \n");
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

        //if(sd->loop_time > sd->loop_time)
        if(sd->t_delta > sd->loop_time)
            printf("[ERROR] sd->t_delta > sd->loop_time !! \n");
    //}
}

