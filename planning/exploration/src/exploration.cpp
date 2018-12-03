#include <math.h>
#include <algorithm>
#include <vector>
#include <thread>
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
#include "robo7_srvs/getFrontier.h"
#include <robo7_srvs/exploration.h>
#include <robo7_srvs/path_planning.h>
#include <robo7_srvs/PathFollower2.h>

float pi = 3.14159265358979323846;

class Exploration
{

public:
  ros::NodeHandle nh;
  ros::ServiceServer exploration_srv;
  ros::ServiceClient exploration_client, get_frontier_client, occupancy_client, path_planning_client, path_follower2_srv;
  robo7_srvs::IsGridOccupied occupancy_srv;
  robo7_srvs::explore explore_srv;
  robo7_srvs::getFrontier get_frontier_srv;
  ros::Subscriber robot_pose_subs;
  bool position_updated;

  float x_current, y_current, theta_current;
  float x, y, theta;
  std::vector<robo7_msgs::trajectory> path_msgs;

  Exploration(ros::NodeHandle nh)
  {
    this->nh = nh;

    exploration_srv = nh.advertiseService("exploration_service", &Exploration::performExploration, this);
    path_follower2_srv = nh.serviceClient<robo7_srvs::PathFollower2>("/kinematics/path_follower/path_follower_v2");

    path_planning_client = nh.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");

    robot_pose_subs = nh.subscribe("localization/kalman_filter/position_timed", 1000, &Exploration::getPositionCallBack, this);
    //robot_pose_subs = nh.subscribe("/localization/kalman_filter/position", 1000, &Exploration::getPositionCallBack, this);

    occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
    get_frontier_client = nh.serviceClient<robo7_srvs::getFrontier>("/exploration/getFrontier");
    exploration_client = nh.serviceClient<robo7_srvs::explore>("/exploration/explore");
  }

  void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
    x_current = msg->linear.x;
    y_current = msg->linear.y;
    theta_current = msg->angular.z;

    position_updated = true;
  }

  bool followPath(robo7_msgs::trajectory trajectory_array)
  {
    robo7_srvs::PathFollower2::Request req;
    robo7_srvs::PathFollower2::Response res;
    req.traject = trajectory_array;
    path_follower2_srv.call(req, res);

    return res.success;
  }

  bool performExploration(robo7_srvs::exploration::Request &req, robo7_srvs::exploration::Response &res)
  {

    geometry_msgs::Twist robot_pose = req.robot_pose;

    x = robot_pose.linear.x;
    y = robot_pose.linear.y;
    theta = robot_pose.angular.z;

    explore_srv.request.x = x;
    explore_srv.request.y = y;
    explore_srv.request.theta = theta;

    if (!exploration_client.call(explore_srv))
      ROS_WARN("Local exploration not successful");

    if (x == 0 && y == 0 && theta == 0)
    {
      x = .215;
      y = .2;
      theta = pi / 2;
      // x = x_current;
      // y = y_current;
      // theta = theta_current;
    }

    robo7_srvs::path_planning::Request path_req;
    robo7_srvs::path_planning::Response path_res;

    bool exploration_done = false;

    while (!exploration_done)
    {

      get_frontier_client.call(get_frontier_srv);

      if (get_frontier_srv.response.exploration_done)
      {
        exploration_done = true;
        break;
      }

      x = get_frontier_srv.response.frontier_destination_pose.linear.x;
      y = get_frontier_srv.response.frontier_destination_pose.linear.y;

      path_req.exploring = true;
      path_req.robot_position = robot_pose;
      path_req.destination_position.x = x;
      path_req.destination_position.y = y;

      if (path_planning_client.call(path_req, path_res))
      {
        res.success = path_res.success;
      }
      else
      {
        ROS_WARN("No path found for exploration");
        break;
      }

      robo7_msgs::trajectory trajectory_array;

      path_msgs.push_back(path_res.path_planned);

      if (false && path_msgs.size() > 0)
      {

        for (int i = 0; i < path_msgs.size(); i++)
        {
          for (int j = 0; j < path_msgs[i].trajectory_points.size(); j++)
          {
            trajectory_array.trajectory_points.push_back(path_msgs[i].trajectory_points[j]);
          }
        }
        path_msgs.clear();

        //followPath(trajectory_array);
      }

      // robo7_srvs::PathFollower2::Request req2;
      // robo7_srvs::PathFollower2::Response res2;
      // req2.traject = path_res.path_planned;
      // path_follower2_srv.call(req2, res2);

      //trajectory_array = path_res.path_planned;
      /*
      if (position_updated)
      {

        explore_srv.request.x = partial_x;
        explore_srv.request.y = partial_y;
        explore_srv.request.theta = partial_theta;
        explore_srv.request.get_frontier = false;
        exploration_client.call(explore_srv);
        theta = partial_theta;
      }
      */

      /*
      float partial_x, partial_y, partial_theta;
      partial_x = x;
      partial_y = y;
      partial_theta = theta;
      //for (int j = 0; j < path_res.path_planned.trajectory_points.size(); j++)
      //{
        partial_x = path_res.path_planned.trajectory_points[j].pose.linear.x;
        partial_y = path_res.path_planned.trajectory_points[j].pose.linear.y;
        partial_theta = path_res.path_planned.trajectory_points[j].pose.angular.z;
        explore_srv.request.x = partial_x;
        explore_srv.request.y = partial_y;
        explore_srv.request.theta = partial_theta;
        explore_srv.request.get_frontier = false;
        exploration_client.call(explore_srv);
      //}
      robot_pose.linear.x = partial_x;
      robot_pose.linear.y = partial_y;
      robot_pose.angular.z = partial_theta;
      */
      robot_pose.linear.x = x;
      robot_pose.linear.y = y;
      robot_pose.angular.z = theta;
    }

    ROS_INFO("Exploration done, success: %d", exploration_done);

    res.success = exploration_done;

    return exploration_done;
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