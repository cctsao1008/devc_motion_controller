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

#define DEFAULT_LOOP_TIME           80.0  // ms
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

void timer2_cb(const ros::TimerEvent &event)
{
    print_info_t(sd);
}

void timer1_cb(const ros::TimerEvent &event)
{
    #if 0
    //printf("vel_tx_cb \n");
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::NodeHandle nh_param("~");

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //we will be sending commands of type "twist"
    geometry_msgs::Twist msg;

    msg.linear.x = 0.2;
    msg.linear.x = 0.2;
    msg.angular.z = 0.4;

    pub.publish(msg);
    ros::spinOnce();
    #endif
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
    /* time, clock, perf... */
    struct timespec t_s, t_e;
    uint64_t t_elap = 0;
    double t_diff = 0;
    pthread_t tid;

    /* motion control */
    sd = system_init();

    if(sd == NULL)
        return 0;

    memset(sd, 0, sizeof(system_data));

    sd->mot.mode = 0;
    sd->loop_time = DEFAULT_LOOP_TIME;

    motion_control_init(sd);
    motor_control_init(sd);
    commander_init(sd);

    /* ROS Support */

    /* Registering a node in ros master */
    ros::init(argc,argv,"motion");

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_geometry;
    ros::Publisher pub_motion;

    geometry_msgs::Twist msg_twist;
    motion_msgs::Data msg_motion;

    msg_twist.linear.x = 0.2;
    msg_twist.linear.y = 0.2;
    msg_twist.angular.z = 0.4;

    pub_geometry = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_motion = nh.advertise<motion_msgs::Data>("motion_data", 1);

    /* Timers allow you to get a callback at a specified rate. */
    //ros::Timer timer1 = nh.createTimer(ros::Duration(0.5), timer1_cb, &nh);
    ros::Timer timer2 = nh.createTimer(ros::Duration(1, 0), timer2_cb);

    ros::NodeHandle nh_param("~");
    sub = nh.subscribe("cmd_vel", 100, &vel_cmd_sub_cb);

    ros::Rate loop_rate(1.0/(double)(sd->loop_time/1000.0));

    while(ros::ok())
    {
        /* measure monotonic time */
        clock_gettime(CLOCK_MONOTONIC, &t_s);
        ros::spinOnce();
        clock_gettime(CLOCK_MONOTONIC, &t_e);

        t_diff = BILLION * (t_e.tv_sec - t_s.tv_sec) + t_e.tv_nsec - t_s.tv_nsec;
        //printf("t_diff = %f ms \n", t_diff / 1000000);

        motion_control_update(sd);
        motor_control_update(sd);

        t_elap += sd->t_delta;

        sd->sys_usage = (t_diff / (double)((sd->loop_time) * 1000000.0)) * 100.0;
        sd->sys_elaps = t_elap / 1000;

        /* test ONLY */
        #if 0
        pub_geometry.publish(msg_twist);
        #endif

        msg_motion.loop_time = sd->loop_time;
        /* sv */
        msg_motion.sv.vx = sd->sv.vx;
        msg_motion.sv.vy = sd->sv.vy;
        msg_motion.sv.w0 = sd->sv.w0;
        /* cv */
        msg_motion.cv.vx = sd->cv.vx;
        msg_motion.cv.vy = sd->cv.vy;
        msg_motion.cv.w0 = sd->cv.w0;
        /* pv */
        msg_motion.pv.vx = sd->pv.vx;
        msg_motion.pv.vy = sd->pv.vy;
        msg_motion.pv.w0 = sd->pv.w0;
        /* mot output */
        msg_motion.mot_o.fr1 = sd->mot.out.fr1;
        msg_motion.mot_o.fr2 = sd->mot.out.fr2;
        msg_motion.mot_o.fr3 = sd->mot.out.fr3;
        msg_motion.mot_o.fr4 = sd->mot.out.fr4;
        msg_motion.mot_o.w1 = sd->mot.out.w1;
        msg_motion.mot_o.w2 = sd->mot.out.w2;
        msg_motion.mot_o.w3 = sd->mot.out.w3;
        msg_motion.mot_o.w4 = sd->mot.out.w4;
        msg_motion.mot_o.rpm1 = sd->mot.out.rpm1;
        msg_motion.mot_o.rpm2 = sd->mot.out.rpm2;
        msg_motion.mot_o.rpm3 = sd->mot.out.rpm3;
        msg_motion.mot_o.rpm4 = sd->mot.out.rpm4;
        msg_motion.mot_o.pwm1 = sd->mot.out.pwm1;
        msg_motion.mot_o.pwm2 = sd->mot.out.pwm2;
        msg_motion.mot_o.pwm3 = sd->mot.out.pwm3;
        msg_motion.mot_o.pwm4 = sd->mot.out.pwm4;
        /* mot input */
        msg_motion.mot_i.fr1 = sd->mot.out.fr1;
        msg_motion.mot_i.fr2 = sd->mot.out.fr2;
        msg_motion.mot_i.fr3 = sd->mot.out.fr3;
        msg_motion.mot_i.fr4 = sd->mot.out.fr4;
        msg_motion.mot_i.w1 = sd->mot.in.w1;
        msg_motion.mot_i.w2 = sd->mot.in.w2;
        msg_motion.mot_i.w3 = sd->mot.in.w3;
        msg_motion.mot_i.w4 = sd->mot.in.w4;
        msg_motion.mot_i.rpm1 = sd->mot.in.rpm1;
        msg_motion.mot_i.rpm2 = sd->mot.in.rpm2;
        msg_motion.mot_i.rpm3 = sd->mot.in.rpm3;
        msg_motion.mot_i.rpm4 = sd->mot.in.rpm4;
        msg_motion.mot_i.pwm1 = sd->mot.in.pwm1;
        msg_motion.mot_i.pwm2 = sd->mot.in.pwm2;
        msg_motion.mot_i.pwm3 = sd->mot.in.pwm3;
        msg_motion.mot_i.pwm4 = sd->mot.in.pwm4;

        pub_motion.publish(msg_motion);

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

    //for(;;)
    //{
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

        //if(sd->loop_time > sd->loop_time)
        if(sd->t_delta > sd->loop_time)
            printf("[ERROR] sd->t_delta > sd->loop_time !! \n");
    //}
}

