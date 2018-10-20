
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float32MultiArray.h"
#include <robo7_msgs/paths.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>

std::vector<std::vector<float> > paths_x, paths_y;
std::vector<float> path_x_msg, path_y_msg;
double control_frequency = 10.0;
int number_paths = 0;

class Paths
{
public:
  ros::Subscriber robot_position;
  ros::Subscriber paths_sub;
  ros::Publisher marker_array_pub;

  //Initialisation
  float x0, y0, theta0;
  bool paths_received;

  Paths(ros::NodeHandle nh, ros::Publisher marker_array_pub)
  {
    this->paths_sub = nh.subscribe("/pathplanning/paths_vector", 1000, &Paths::pathsCallback, this);
    this->marker_array_pub = marker_array_pub;
    this->paths_received = false;
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

  void updatePaths()
  {
    if (this->paths_received)
    {

      visualization_msgs::MarkerArray curve_array_msg, points_array_msg;

      geometry_msgs::Point p;

      curve_array_msg.markers.resize(paths_x.size());
      points_array_msg.markers.resize(paths_x.size());

      for (int i = 0; i < paths_x.size(); i++)
      {

        curve_array_msg.markers[i].header.frame_id = points_array_msg.markers[i].header.frame_id = "/map";
        curve_array_msg.markers[i].header.stamp = points_array_msg.markers[i].header.stamp = ros::Time::now();
        curve_array_msg.markers[i].action = points_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        curve_array_msg.markers[i].pose.orientation.w = points_array_msg.markers[i].pose.orientation.w = 1.0;
        curve_array_msg.markers[i].ns = "Curves";
        points_array_msg.markers[i].ns = "Nodes";

        curve_array_msg.markers[i].id = i;
        points_array_msg.markers[i].id = i + paths_x.size();

        curve_array_msg.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        points_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;


        curve_array_msg.markers[i].scale.x = 0.01;
        curve_array_msg.markers[i].scale.y = 0.01;
        curve_array_msg.markers[i].scale.z = 0.01;

        curve_array_msg.markers[i].color.g = 1.0f;
        curve_array_msg.markers[i].color.a = 1.0;

        points_array_msg.markers[i].color.b = 1.0;
        points_array_msg.markers[i].color.a = 1.0;

        for (int j = 0; j < paths_x[i].size(); j++)
        {
          p.x = paths_x[i][j];
          p.y = paths_y[i][j];
          p.z = 0;
          curve_array_msg.markers[i].points.push_back(p);
        }
        points_array_msg.markers[i].points.push_back(p);

      }

      marker_array_pub.publish(curve_array_msg);
      marker_array_pub.publish(points_array_msg);

    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "paths");
  ros::NodeHandle nh;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("Paths", 10);
  ROS_INFO("Init paths");
  Paths paths = Paths(nh, marker_array_pub);

  ros::Rate loop_rate(control_frequency);

  float f = 0.0;

  while (ros::ok())
  {

    paths.updatePaths();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
