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
#include "robo7_srvs/explore.h"
#include <robo7_srvs/exploration.h>

float pi = 3.14159265358979323846;

class Exploration
{

public:
  ros::NodeHandle nh;
  ros::ServiceServer exploration_srv;
  ros::ServiceClient occupancy_client, exploration_client;
  robo7_srvs::IsGridOccupied occupancy_srv;
  robo7_srvs::explore explore_srv;
  ros::Subscriber robot_pose;
  bool position_updated;

  float x0_default, y0_default, theta0_default;
  float x, y, theta;

  Exploration(ros::NodeHandle nh)
  {
    this->nh = nh;

    exploration_srv = nh.advertiseService("exploration_service", &Exploration::performExploration, this);

    robot_pose = nh.subscribe("localization/kalman_filter/position_timed", 1000, &Exploration::getPositionCallBack, this);

    occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
    exploration_client = nh.serviceClient<robo7_srvs::explore>("/exploration_grid/explore");
  }

  void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
    x0_default = msg->linear.x;
    y0_default = msg->linear.y;
    theta0_default = msg->angular.z;

    position_updated = true;
  }
  bool performExploration(robo7_srvs::exploration::Request &req, robo7_srvs::exploration::Response &res)
  {
    geometry_msgs::Twist robot_pose = req.robot_pose;

    x = robot_pose.linear.x;
    y = robot_pose.linear.y;
    theta = robot_pose.angular.z;

    if (position_updated && x == 0 && y == 0 && theta == 0)
    {
      //x = .215;
      //y = .2;
      //theta = pi/2;
      x = x0_default;
      y = y0_default;
      theta = theta0_default;
      ROS_INFO("Exploration uses default parameters \nx0: %f \ny0: %f \ntheta0: %f", x, y, theta);
    }
    
    explore_srv.request.x = x;
    explore_srv.request.y = y;
    explore_srv.request.theta = theta;

    if (exploration_client.call(explore_srv))
    {
      ROS_INFO("Explored here: %d", explore_srv.response.explored);
    }

    res.success = true;
    res.destination_pose.linear.x = x;
    res.destination_pose.linear.y = y;
    res.destination_pose.linear.z = theta;

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration");

  ros::NodeHandle nh;
  nh = ros::NodeHandle("~");

  double control_frequency = 100;

  ROS_INFO("Init exploration");

  Exploration exploration = Exploration(nh);

  ros::Rate loop_rate(control_frequency);

  ros::spin();

  return 0;
}
