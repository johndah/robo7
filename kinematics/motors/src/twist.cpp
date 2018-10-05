#include "ros/ros.h"

#include <robo7_msgs/WheelAngularVelocities.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist");
  ros::NodeHandle n;

  ros::Publisher twist_pub = n.advertise<robo7_msgs::WheelAngularVelocities>("ref_vels", 100);

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    robo7_msgs::WheelAngularVelocities msg;
    msg.W_l = 0.3;
    msg.W_r = 0.3;

    ROS_INFO("W_l: %f", msg.W_l);
    ROS_INFO("W_r: %f", msg.W_r);
    twist_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
