#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include <robo7_msgs/trajectory.h>
#include <robo7_msgs/trajectory_point.h>
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/distanceTo.h"
#include <robo7_srvs/path_planning.h>

float pi = 3.14159265;

class Exploration
{

public:
  ros::NodeHandle nh;

  Exploration(ros::NodeHandle nh)
  {
    this->nh = nh;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration");

  ros::NodeHandle nh;
  nh = ros::NodeHandle("~");

  double control_frequency = 1.0;

  ROS_INFO("Init exploration");

  Exploration path_planning = Exploration(nh);

  ros::Rate loop_rate(control_frequency);

  ros::spin();

  return 0;
}
