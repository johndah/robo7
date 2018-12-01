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
#include <robo7_srvs/path_planning.h>

float pi = 3.14159265358979323846;

class Exploration
{

public:
  ros::NodeHandle nh;
  ros::ServiceServer exploration_srv;
  ros::ServiceClient exploration_client, occupancy_client, path_planning_client;
  robo7_srvs::IsGridOccupied occupancy_srv;
  robo7_srvs::explore explore_srv;
  ros::Subscriber robot_pose_subs;
  bool position_updated;

  float x0_default, y0_default, theta0_default;
  float x, y, theta;

  Exploration(ros::NodeHandle nh)
  {
    this->nh = nh;

    exploration_srv = nh.advertiseService("exploration_service", &Exploration::performExploration, this);

    path_planning_client = nh.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");

    //robot_pose_subs = nh.subscribe("localization/kalman_filter/position_timed", 1000, &Exploration::getPositionCallBack, this);
    robot_pose_subs = nh.subscribe("/localization/kalman_filter/position", 1000, &Exploration::getPositionCallBack, this);

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
    }

    robo7_srvs::path_planning::Request path_req;
    robo7_srvs::path_planning::Response path_res;

    bool exploration_done = false;

    while (!exploration_done)
    {

      explore_srv.request.x = x;
      explore_srv.request.y = y;
      explore_srv.request.theta = theta;
      explore_srv.request.get_frontier = true;

      exploration_client.call(explore_srv);

      if (explore_srv.response.exploration_done)
        break;

      x = explore_srv.response.frontier_destination_pose.linear.x;
      y = explore_srv.response.frontier_destination_pose.linear.y;

      // ROS_INFO("rx %f  ry %f  dx %f  dy %f", robot_pose.linear.x, robot_pose.linear.y, x, y);
      path_req.exploring = true;
      path_req.robot_position = robot_pose;
      path_req.destination_position.x = x;
      path_req.destination_position.y = y;

      if (path_planning_client.call(path_req, path_res))
      {
        res.success = path_res.success;
      }

      robo7_msgs::trajectory trajectory_array;

      trajectory_array = path_res.path_planned;

      float partial_x, partial_y, partial_theta;
      partial_theta = theta;
      for (int j = 0; j < trajectory_array.trajectory_points.size(); j++)
      {
        partial_x = trajectory_array.trajectory_points[j].pose.linear.x;
        partial_y = trajectory_array.trajectory_points[j].pose.linear.y;
        partial_theta = trajectory_array.trajectory_points[j].pose.angular.z;
        explore_srv.request.x = partial_x;
        explore_srv.request.y = partial_y;
        explore_srv.request.theta = partial_theta;
        explore_srv.request.get_frontier = false;

        exploration_client.call(explore_srv);
      }

      robot_pose.linear.x = partial_x;
      robot_pose.linear.y = partial_y;
      robot_pose.angular.z = partial_theta;
    }

    ROS_INFO("Exploration Done");

    res.success = true;
    res.frontier_destination_pose = explore_srv.response.frontier_destination_pose;

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
