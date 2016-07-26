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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <motion_msgs/Data.h>


#include <sstream>

#define DEFAULT_LOOP_TIME           200.0  // ms
#define BILLION                     1000000000L

void print_banner(char *data);
void print_info_t(void *arg);

/* system... */
system_data *sd = NULL;

void sub_motion_cb(const motion_msgs::Data &msg)
{

}

int main(int argc,char** argv)
{
    /* time, clock, perf... */
    struct timespec t_s, t_e;
    uint64_t t_elap = 0;
    double t_diff = 0;

    /* ROS Support */

    /* Registering a node in ros master */
    ros::init(argc,argv,"motion_tester");

    ros::NodeHandle nh;
	ros::Subscriber sub_motion;
    ros::Publisher pub_geometry;
	ros::Publisher pub_odom;
	ros::Publisher pub_joint;

    geometry_msgs::Twist msg_twist;
	sensor_msgs::JointState joint_state;  

    //pub_geometry = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	sub_motion = nh.subscribe("motion_data", 100, &sub_motion_cb);

	// odom
	//pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

	pub_joint = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	pub_geometry = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::NodeHandle nh_param("~");

    ros::Rate loop_rate(1.0/(double)(DEFAULT_LOOP_TIME/1000.0));

	ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

    while(ros::ok())
    {
        msg_twist.linear.x = 0.111;
        msg_twist.linear.y = 0.222;
        msg_twist.angular.z = 0.333;

        pub_geometry.publish(msg_twist);

		current_time = ros::Time::now();

		//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);

		//first, we'll publish the transform over tf
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_link";
 
    	odom_trans.transform.translation.x = 0.0;
    	odom_trans.transform.translation.y = 0.0;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = odom_quat;
 
    	//next, we'll publish the odometry message over ROS
    	nav_msgs::Odometry odom;
    	odom.header.stamp = current_time;
    	odom.header.frame_id = "odom";
 
    	//set the position
    	odom.pose.pose.position.x = 0.0;
    	odom.pose.pose.position.y = 0.0;
    	odom.pose.pose.position.z = 0.0;
    	odom.pose.pose.orientation = odom_quat;
 
    	//set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = 2.0;
    	odom.twist.twist.linear.y = 0.0;
    	odom.twist.twist.angular.z = 0.0;
 
    	//publish the message
    	//pub_odom.publish(odom);

		//update joint_state  
        joint_state.header.stamp = ros::Time::now();  
        joint_state.name.resize(4);  
        joint_state.position.resize(4);  
        joint_state.name[0] ="joint_base_wheel1";  
        joint_state.position[0] = 0;  
        joint_state.name[1] ="joint_base_wheel2";  
        joint_state.position[1] = 0;  
        joint_state.name[2] ="joint_base_wheel3";  
        joint_state.position[2] = 0;  
        joint_state.name[3] ="joint_base_wheel4";  
        joint_state.position[3] = 0; 

		pub_joint.publish(joint_state);

		//send the transform
        odom_broadcaster.sendTransform(odom_trans);

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

