#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <robo7_srvs/explore.h>
#include <robo7_srvs/exploration.h>
#include <robo7_msgs/the_robot_position.h>


class LocalExploration
{

public:
  ros::NodeHandle nh;
  ros::ServiceServer exploration_srv;
  ros::ServiceClient exploration_client;
  robo7_srvs::explore explore_srv;
  ros::Subscriber robot_pose_subs;
  bool position_updated;

  float x_current, y_current, theta_current;

  LocalExploration(ros::NodeHandle nh)
  {
    this->nh = nh;

    robot_pose_subs = nh.subscribe("/localization/kalman_filter/position_timed", 1000, &LocalExploration::getPositionCallBack, this);

    exploration_client = nh.serviceClient<robo7_srvs::explore>("/exploration/explore");
  }

  void getPositionCallBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    x_current = msg->position.linear.x;
    y_current = msg->position.linear.y;
    theta_current = msg->position.angular.z;

    // ROS_INFO("Getting position %f  %f", x_current, y_current);
    position_updated = true;
  }

  void exploreHere()
  {
    explore_srv.request.x = x_current;
    explore_srv.request.y = y_current;
    explore_srv.request.theta = theta_current;

    if (!exploration_client.call(explore_srv))
      ROS_WARN("Local exploration not successful");
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_exploration");

  ros::NodeHandle nh;
  nh = ros::NodeHandle("~");

  double control_frequency = 2;

  ROS_INFO("Init local exploration");

  LocalExploration local_exploration = LocalExploration(nh);

  ros::Rate loop_rate(control_frequency);

  while (ros::ok())
  {
    ros::spinOnce();

    // ROS_INFO("Doing local stuff");
    if (local_exploration.position_updated)
    {
      // ROS_INFO("Explore here");
      local_exploration.exploreHere();
      local_exploration.position_updated = false;
    }

    loop_rate.sleep();
  }

  return 0;
}
