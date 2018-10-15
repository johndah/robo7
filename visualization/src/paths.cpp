
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include <robo7_msgs/paths.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>

//std_msgs::Float32MultiArray path_x;
std::vector<std::vector<float> > paths_x, paths_y;
std::vector<float> path_x_msg, path_y_msg;
double control_frequency = 10.0;
int number_paths = 0;

class Paths
{
public:
  ros::Subscriber robot_position;
  ros::Subscriber paths_sub;

  //Initialisation
  float x0, y0, theta0;

  Paths(ros::NodeHandle nh)
  {

    paths_sub = nh.subscribe("/pathplanning/paths_vector", 1000, &Paths::pathsCallback, this);
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
   
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "paths");
  ros::NodeHandle n;

  Paths paths = Paths(n);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Paths", 10);

  ros::Rate loop_rate(control_frequency);

  float f = 0.0;

  while (ros::ok())
  {

    std::vector<visualization_msgs::Marker> point_sets;
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::SPHERE;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    line_strip.scale.z = 0.01;

    line_strip.color.g = 1.0f;
    line_strip.color.a = 1.0;

    points.color.b = 1.0;
    points.color.a = 1.0;


    geometry_msgs::Point p;

    for (int i = 0; i < paths_x.size(); i++)
    {

      for (int j = 0; j < paths_x[i].size(); j++)
      {
        //ROS_INFO("i: %d size: %f", i, (float)paths_x[i][j]);
        p.x = paths_x[i][j];
        p.y = paths_y[i][j];
        p.z = 0;
        line_strip.points.push_back(p);
      }
        point_sets.push_back(line_strip);

        //marker_pub.publish(line_strip);
        p.x = paths_x[i][ paths_x[i].size()-2];
        p.y = paths_y[i][ paths_x[i].size()-2];
        p.z = 0;
        points.points.push_back(p);
    }

 
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    //marker_pub.publish(point_sets);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
