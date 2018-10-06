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
float des_v;
float des_w;

// From rosparam get /robot_description/
double wheel_separation = 0.2198; // center of track to center of track
double wheel_radius = 0.049;


void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  des_twist = *msg;
  des_v = des_twist.linear.x;
  des_w = des_twist.angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_interpreter");

  ros::NodeHandle n;

  ros::Subscriber twist_sub = n.subscribe("/desired_velocity", 1, TwistCallback);

  ros::Publisher ref_vel_pub = n.advertise<robo7_msgs::WheelAngularVelocities>("/ref_vels", 1);

  ros::Rate loop_rate(freq);

  ROS_INFO("Running twist_interpreter");


  while (ros::ok())
  {
    ros::spinOnce();

    float des_l = (des_v - (wheel_separation / 2) * des_w) / wheel_radius;
    float des_r = (des_v + (wheel_separation / 2) * des_w) / wheel_radius;

    //ROS_INFO("des_l: %e , des_r: %e", des_l, des_r);

    ref_vels.W_l = des_l;
    ref_vels.W_r = des_r;

    ref_vel_pub.publish(ref_vels);

    // Smooth breaking when no input is recieved
    if (abs(des_v) > 0.001) {
      des_v = des_v / break_scalar;
    }
    else if( des_v != 0){
      des_v = 0;
      ROS_DEBUG("STOP V");
    }

    if (abs(des_w) > 0.001) {
      des_w =  des_w / break_scalar;
    }
    else if( des_w != 0){
      des_w = 0;
      ROS_DEBUG("STOP W");
    }


    loop_rate.sleep();

  }

  return 0;
}
