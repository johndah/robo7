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
#include <robo7_msgs/the_robot_position.h>
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/explore.h"
#include "robo7_srvs/getFrontier.h"
#include <robo7_srvs/exploration.h>
#include <robo7_srvs/path_planning.h>
#include <robo7_srvs/PathFollower2.h>
#include <robo7_srvs/SaveAll.h>

float pi = 3.14159265358979323846;

class Exploration
{

public:
  ros::NodeHandle nh;
  ros::ServiceServer exploration_srv;
  ros::ServiceClient exploration_client, get_frontier_client, occupancy_client, path_planning_client, path_follower2_srv, save_datas_srv;
  robo7_srvs::IsGridOccupied occupancy_srv;
  robo7_srvs::explore explore_srv;
  robo7_srvs::getFrontier get_frontier_srv;
  ros::Subscriber robot_pose_subs;
  bool position_updated;

  float x_current, y_current, theta_current;
  float x, y, theta;
  std::vector<robo7_msgs::trajectory> path_msgs;
  robo7_msgs::the_robot_position the_robot_pose;

  Exploration(ros::NodeHandle nh)
  {
    this->nh = nh;

    exploration_srv = nh.advertiseService("exploration_service", &Exploration::performExploration, this);
    path_follower2_srv = nh.serviceClient<robo7_srvs::PathFollower2>("/kinematics/path_follower/path_follower_v2");

    path_planning_client = nh.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");

    robot_pose_subs = nh.subscribe("/localization/kalman_filter/position_timed", 1000, &Exploration::getPositionCallBack, this);
    //robot_pose_subs = nh.subscribe("/localization/kalman_filter/position", 1000, &Exploration::getPositionCallBack, this);

    occupancy_client = nh.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
    get_frontier_client = nh.serviceClient<robo7_srvs::getFrontier>("/exploration/getFrontier");
    exploration_client = nh.serviceClient<robo7_srvs::explore>("/exploration/explore");
    save_datas_srv = nh.serviceClient<robo7_srvs::SaveAll>("/vision/save");
  }

  void getPositionCallBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;

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
    explore_srv.request.x = the_robot_pose.position.linear.x;
    explore_srv.request.y = the_robot_pose.position.linear.y;
    explore_srv.request.theta = the_robot_pose.position.angular.z;

    if (!exploration_client.call(explore_srv))
      ROS_WARN("Local exploration not successful");

    robo7_srvs::path_planning::Request path_req;
    robo7_srvs::path_planning::Response path_res;

    bool exploration_done = false;

    while (!exploration_done)
    {
      ros::spinOnce();

      get_frontier_client.call(get_frontier_srv);

      if (get_frontier_srv.response.exploration_done)
      {
        exploration_done = true;
        break;
      }

      x = get_frontier_srv.response.frontier_destination_pose.linear.x;
      y = get_frontier_srv.response.frontier_destination_pose.linear.y;

      path_req.exploring = true;
      path_req.robot_position = the_robot_pose.position;
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

      if (path_msgs.size() > 0)
      {

        for (int i = 0; i < path_msgs.size(); i++)
        {
          for (int j = 0; j < path_msgs[i].trajectory_points.size(); j++)
          {
            trajectory_array.trajectory_points.push_back(path_msgs[i].trajectory_points[j]);
          }
        }
        path_msgs.clear();

        if(!followPath(trajectory_array))
        {
          ROS_INFO("Wait a little while");
          ros::Rate r(0.3); r.sleep();
        }
      }

      save_datas();
    }

    ROS_INFO("Exploration done, success: %d", exploration_done);

    res.success = exploration_done;

    return exploration_done;
  }

  void save_datas()
  {
    robo7_srvs::SaveAll::Request req1;
    robo7_srvs::SaveAll::Response res1;
    req1.save_obj = true;
    req1.save_obs = true;
    req1.save_walls = true;
    save_datas_srv.call(req1,res1);
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
