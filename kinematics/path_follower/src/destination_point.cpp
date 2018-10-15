#include <algorithm>
#include <stdlib.h>     /* abs */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robo7_msgs/WheelAngularVelocities.h"

geometry_msgs::Twist des_twist;
robo7_msgs::WheelAngularVelocities ref_vels;

int freq = 10;
float pi = 3.14159;
int break_scalar = 5;

double msg_timeout = 0.004; // acepted delay between msgs before stopping
float des_v;
float des_w;

// From rosparam get /robot_description/
double wheel_separation = 0.2198; // center of track to center of track
double wheel_radius = 0.049;
double max_allowed_speed = 25; //rad.s-1

float x_dest;
float y_dest;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "destination_point");

  ros::NodeHandle n("~");

  n.param<float>("/destination_point/x_destination", x_dest, 0.215);
  n.param<float>("/destination_point/y_destination", y_dest, 0.2);

  ros::Publisher dest_point = n.advertise<geometry_msgs::Twist>("/destination_point", 1);

  ros::Rate loop_rate(freq);

  ROS_INFO("Running twist_interpreter");

  geometry_msgs::Twist twist_point;

  twist_point.linear.x = x_dest;
  twist_point.linear.y = y_dest;

  while (ros::ok())
  {
    ros::spinOnce();
    dest_point.publish(twist_point);
    loop_rate.sleep();
  }

  return 0;
}
