
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float32MultiArray.h"
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


typedef std::vector<float> float_vector;
std::vector<float_vector> paths_x, paths_y, trajectory_x, trajectory_y;
float x_target, y_target;
int number_paths = 0;
float path_height = 0.1;
float marker_height = 0.15;

class Paths
{
public:
  ros::Subscriber robot_position, paths_sub, target_sub, goal_path_sub, target_paths_sub;
  ros::Publisher marker_array_pub; //, marker_pub;

  //Initialisation
  float x0, y0, theta0;
  bool paths_received, target_received, trajectory_received;

  Paths(ros::NodeHandle nh, ros::Publisher marker_array_pub)
  {
    this->paths_sub = nh.subscribe("/path_planning/paths_vector", 1000, &Paths::pathsCallback, this);
    this->target_sub = nh.subscribe("/path_planning/target", 1000, &Paths::targetCallback, this);
    this->target_paths_sub = nh.subscribe("/path_planning/target_path", 1000, &Paths::trajectoryCallback, this);

    this->marker_array_pub = marker_array_pub;
    // this->marker_pub = marker_pub;

    this->paths_received = false;
    this->target_received = false;
    this->trajectory_received = false;
  }

  void pathsCallback(const robo7_msgs::paths::ConstPtr &paths_msg)
  {
    paths_x.clear();
    paths_y.clear();

    for (int i = 0; i < paths_msg->paths.size(); i++)
    {
      paths_x.push_back(paths_msg->paths[i].path_x);
      paths_y.push_back(paths_msg->paths[i].path_y);
    }
    paths_received = true;
  }

  void targetCallback(const geometry_msgs::Point::ConstPtr &target_msg)
  {

    x_target = target_msg->x;
    y_target = target_msg->y;

    target_received = true;
  }

  void trajectoryCallback(const robo7_msgs::paths::ConstPtr &target_paths_msg)
  {

    trajectory_x.clear();
    trajectory_y.clear();

    for (int i = 0; i < target_paths_msg->paths.size(); i++)
    {
      trajectory_x.push_back(target_paths_msg->paths[i].path_x);
      trajectory_y.push_back(target_paths_msg->paths[i].path_y);
    }

    trajectory_received = true;
  }

  void updatePaths()
  {

    visualization_msgs::MarkerArray curve_array_msg, points_array_msg, target_msg, goal_paths_msg, target_points_msg, target_paths_msg;

    if (this->paths_received)
    {
      geometry_msgs::Point p;

      curve_array_msg.markers.resize(paths_x.size());

      for (int i = 0; i < paths_x.size(); i++)
      {

        curve_array_msg.markers[i].header.frame_id = "/map";
        curve_array_msg.markers[i].header.stamp = ros::Time::now();
        curve_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        curve_array_msg.markers[i].pose.orientation.w = 1.0;
        curve_array_msg.markers[i].ns = "Curves";
        curve_array_msg.markers[i].id = i;
        curve_array_msg.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

        curve_array_msg.markers[i].scale.x = 0.01;
        curve_array_msg.markers[i].scale.y = 0.01;
        curve_array_msg.markers[i].scale.z = 0.01;

        curve_array_msg.markers[i].color.b = 1.0;
        curve_array_msg.markers[i].color.a = 1.0;

        for (int j = 0; j < paths_x[i].size(); j++)
        {
          p.x = paths_x[i][j];
          p.y = paths_y[i][j];
          p.z = path_height;
          curve_array_msg.markers[i].points.push_back(p);
        }
      }
    }

    if (this->trajectory_received)
    {
      geometry_msgs::Point p;

      target_paths_msg.markers.resize(trajectory_x.size());
      target_points_msg.markers.resize(trajectory_x.size());

      for (int i = 0; i < trajectory_x.size(); i++)
      {

        // target_paths_msg.markers[i].points.clear();

        target_paths_msg.markers[i].header.frame_id = "/map";
        target_paths_msg.markers[i].header.stamp = ros::Time::now();
        target_paths_msg.markers[i].action = visualization_msgs::Marker::ADD;
        target_paths_msg.markers[i].pose.orientation.w = 1.0;
        target_paths_msg.markers[i].ns = "Trajectory";
        target_paths_msg.markers[i].id = i;
        target_paths_msg.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

        target_paths_msg.markers[i].scale.x = 0.02;
        target_paths_msg.markers[i].scale.y = 0.02;
        target_paths_msg.markers[i].scale.z = 0.02;

        target_paths_msg.markers[i].color.g = 1.0;
        target_paths_msg.markers[i].color.a = 1.0;

        target_points_msg.markers[i].header.frame_id = "/map";
        target_points_msg.markers[i].header.stamp = ros::Time::now();
        target_points_msg.markers[i].action = visualization_msgs::Marker::ADD;
        target_points_msg.markers[i].pose.orientation.w = 1.0;
        target_points_msg.markers[i].ns = "Trajectory nodes";
        target_points_msg.markers[i].id = i;
        target_points_msg.markers[i].type = visualization_msgs::Marker::SPHERE;

        target_points_msg.markers[i].scale.x = 0.05;
        target_points_msg.markers[i].scale.y = 0.05;
        target_points_msg.markers[i].scale.z = 0.05;

        target_points_msg.markers[i].color.r = 1.0;
        target_points_msg.markers[i].color.g = .5;
        target_points_msg.markers[i].color.a = 1.0;

        for (int j = 0; j < trajectory_x[i].size(); j++)
        {
          p.x = trajectory_x[i][j];
          p.y = trajectory_y[i][j];
          p.z = path_height;
          target_paths_msg.markers[i].points.push_back(p);
          // target_paths_msg.markers[i].pose.position.x = trajectory_x[i][j];
          // target_paths_msg.markers[i].pose.position.y = trajectory_y[i][j];
          // target_paths_msg.markers[i].pose.position.z = path_height;
        }
        if (trajectory_x[i].size() > 0)
        {
          target_points_msg.markers[i].pose.position.x = p.x;
          target_points_msg.markers[i].pose.position.y = p.y;
          target_points_msg.markers[i].pose.position.z = path_height;
          // target_points_msg.markers[i].pose.position.x = trajectory_x[i][trajectory_x[i].size()-1];
          // target_points_msg.markers[i].pose.position.y = trajectory_y[i][trajectory_x[i].size()-1];
          // target_points_msg.markers[i].pose.position.z = path_height;
        }
      }
    }

    if (this->target_received)
    {
        target_msg.markers.resize(1);

        target_msg.markers[0].header.frame_id = "/map";
        target_msg.markers[0].header.stamp = ros::Time::now();
        target_msg.markers[0].action = visualization_msgs::Marker::ADD;
        target_msg.markers[0].pose.orientation.w = 1.0;
        target_msg.markers[0].id = 0;

        target_msg.markers[0].ns = "Target";
        target_msg.markers[0].type = visualization_msgs::Marker::CUBE;
        target_msg.markers[0].color.g = 1.0;
        target_msg.markers[0].color.a = 1.0;
      

        target_msg.markers[0].scale.x = 0.1;
        target_msg.markers[0].scale.y = 0.1;
        target_msg.markers[0].scale.z = 0.1;

        target_msg.markers[0].pose.position.x = x_target;
        target_msg.markers[0].pose.position.y = y_target;
        target_msg.markers[0].pose.position.z = marker_height;
    }

    marker_array_pub.publish(curve_array_msg);
    marker_array_pub.publish(target_msg);
    marker_array_pub.publish(target_paths_msg);
    marker_array_pub.publish(target_points_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "paths");
  ros::NodeHandle nh;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("Paths", 10);
  //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("Nodes", 10);

  ROS_INFO("Init paths");
  Paths paths = Paths(nh, marker_array_pub);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {

    paths.updatePaths();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
