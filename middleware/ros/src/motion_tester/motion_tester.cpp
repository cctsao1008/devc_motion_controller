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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <motion_msgs/Data.h>


#include <sstream>

#define DEFAULT_LOOP_TIME           200.0  // ms
#define BILLION                     1000000000L

void print_banner(char *data);
void print_info_t(void *arg);

/* system... */
system_data *sd = NULL;

void vel_cmd_sub_cb(const geometry_msgs::Twist &msg)
{
    sd->sv.vx = msg.linear.x;
    sd->sv.vy = msg.linear.y;
    sd->sv.w0 = msg.angular.z;

}

int main(int argc,char** argv)
{
    /* time, clock, perf... */
    struct timespec t_s, t_e;
    uint64_t t_elap = 0;
    double t_diff = 0;

    //commander_init(sd);

    /* ROS Support */

    /* Registering a node in ros master */
    ros::init(argc,argv,"motion_tester");

    ros::NodeHandle nh;
    ros::Publisher pub_geometry;

    geometry_msgs::Twist msg_twist;

    pub_geometry = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::NodeHandle nh_param("~");

    ros::Rate loop_rate(1.0/(double)(DEFAULT_LOOP_TIME/1000.0));

    while(ros::ok())
    {
        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &t_s);
        ros::spinOnce();
        clock_gettime(CLOCK_MONOTONIC, &t_e);

        //t_diff = BILLION * (t_e.tv_sec - t_s.tv_sec) + t_e.tv_nsec - t_s.tv_nsec;
        //t_elap += sd->t_delta;

        msg_twist.linear.x = 0.111;
        msg_twist.linear.y = 0.222;
        msg_twist.angular.z = 0.333;

        pub_geometry.publish(msg_twist);

        loop_rate.sleep();
    }

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

     /* display system information */
     system("clear");

     //print_banner(banner);
     printf(" ROS motion is running.. \n");
     printf(" Loading %4.2f %%, Elapsed time %9d sec \n\n",
         sd->sys_usage, (uint16_t)sd->sys_elaps);

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

     if(sd->t_delta > sd->loop_time)
         printf("[ERROR] sd->t_delta > sd->loop_time !! \n");
}

