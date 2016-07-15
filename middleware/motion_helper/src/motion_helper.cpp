/**
 * @file motion_helper.cpp
 *
 * motion helper
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include<ros/ros.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"motion_helper"); // Registering a node in ros master
  ROS_INFO("Welcome to ROS!");
  return 0;
}
