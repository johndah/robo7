
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float32MultiArray.h"
#include <robo7_msgs/path.h>
#include <robo7_msgs/paths.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>

typedef std::vector<float> float_vector;
std::vector<float_vector> paths_x, paths_y, goal_paths_x, goal_paths_y, trajectory_x, trajectory_y; //, goal_paths_theta;
float_vector start_goal_x(2), start_goal_y(2);                                                      //, start_goal_theta(2);
double control_frequency = 10.0;
int number_paths = 0;
float path_height = 0.1;
float marker_height = 0.15;

class Paths
{
public:
  ros::Subscriber robot_position, paths_sub, start_goal_sub, goal_path_sub, target_paths_sub;
  ros::Publisher marker_array_pub, marker_pub;

  //Initialisation
  float x0, y0, theta0;
  bool paths_received, start_goal_received, goal_path_received, trajectory_received;

  Paths(ros::NodeHandle nh, ros::Publisher marker_array_pub, ros::Publisher marker_pub)
  {
    this->paths_sub = nh.subscribe("/pathplanning/paths_vector", 1000, &Paths::pathsCallback, this);
    this->start_goal_sub = nh.subscribe("/pathplanning/start_goal", 1000, &Paths::startGoalCallback, this);
    //this->goal_path_sub = nh.subscribe("/pathplanning/goal_path", 1000, &Paths::goalPathCallback, this);
    this->target_paths_sub = nh.subscribe("/pathplanning/target_path", 1000, &Paths::trajectoryCallback, this);

    this->marker_array_pub = marker_array_pub;
    this->marker_pub = marker_pub;

    this->paths_received = false;
    this->start_goal_received = false;
    //this->goal_path_received = false;
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

  void startGoalCallback(const robo7_msgs::path::ConstPtr &start_goal_msg)
  {
    start_goal_x[0] = start_goal_msg->path_x[0];
    start_goal_x[1] = start_goal_msg->path_x[1];
    start_goal_y[0] = start_goal_msg->path_y[0];
    start_goal_y[1] = start_goal_msg->path_y[1];
    //start_goal_theta[0] = start_goal_msg->path_theta[0];
    //start_goal_theta[1] = start_goal_msg->path_theta[1];

    start_goal_received = true;
  }
  /*
  void goalPathCallback(const robo7_msgs::paths::ConstPtr &goal_path_msg)
  {
    goal_paths_x.clear();
    goal_paths_y.clear();

    for (int i = 0; i < goal_path_msg->paths.size(); i++)
    {
      goal_paths_x.push_back(goal_path_msg->paths[i].path_x);
      goal_paths_y.push_back(goal_path_msg->paths[i].path_y);
    }

    goal_path_received = true;
  }
*/

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

    visualization_msgs::MarkerArray curve_array_msg, points_array_msg, start_goal_msg, goal_paths_msg, target_points_msg, target_paths_msg;

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

    /*
    if (this->goal_path_received)
    {
      geometry_msgs::Point p;

      goal_paths_msg.markers.resize(goal_paths_x.size());

      for (int i = 0; i < goal_paths_x.size(); i++)
      {

        goal_paths_msg.markers[i].header.frame_id = "/map";
        goal_paths_msg.markers[i].header.stamp = ros::Time::now();
        goal_paths_msg.markers[i].action = visualization_msgs::Marker::ADD;
        goal_paths_msg.markers[i].pose.orientation.w = 1.0;
        goal_paths_msg.markers[i].ns = "Goal Path";
        goal_paths_msg.markers[i].id = i;
        goal_paths_msg.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

        goal_paths_msg.markers[i].scale.x = 0.02;
        goal_paths_msg.markers[i].scale.y = 0.02;
        goal_paths_msg.markers[i].scale.z = 0.02;

        goal_paths_msg.markers[i].color.g = 1.0;
        goal_paths_msg.markers[i].color.a = 1.0;

        for (int j = 0; j < goal_paths_x[i].size(); j++)
        {
          p.x = goal_paths_x[i][j];
          p.y = goal_paths_y[i][j];
          p.z = 0.0;
          goal_paths_msg.markers[i].points.push_back(p);
        }
      }
    }

  */
    if (this->trajectory_received)
    {
      geometry_msgs::Point p;

      target_paths_msg.markers.resize(trajectory_x.size());
      target_points_msg.markers.resize(trajectory_x.size());

      for (int i = 0; i < trajectory_x.size(); i++)
      {
        /*
        */
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

        //target_paths_msg.markers[i].color.r = 1.0;
        target_paths_msg.markers[i].color.g = 1.0;
        target_paths_msg.markers[i].color.a = 1.0;

        //target_points_msg.pose.position.x = trajectory_x[i][0];
        //target_points_msg.pose.position.y = trajectory_x[i][0];
        //target_points_msg.pose.position.z = trajectory_x[i][0];

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
        }
        if (trajectory_x[i].size() > 0)
        {
          target_points_msg.markers[i].pose.position.x = p.x;
          target_points_msg.markers[i].pose.position.y = p.y;
          target_points_msg.markers[i].pose.position.z = path_height;
        }
      }
    }

    if (this->start_goal_received)
    {
      start_goal_msg.markers.resize(2);

      for (int i = 0; i < 2; i++)
      {
        start_goal_msg.markers[i].header.frame_id = "/map";
        start_goal_msg.markers[i].header.stamp = ros::Time::now();
        start_goal_msg.markers[i].action = visualization_msgs::Marker::ADD;
        start_goal_msg.markers[i].pose.orientation.w = 1.0;
        start_goal_msg.markers[i].id = i;

        if (i == 0)
        {
          start_goal_msg.markers[i].ns = "Start";
          start_goal_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
          start_goal_msg.markers[i].color.r = 1.0;
          start_goal_msg.markers[i].color.a = 1.0;
        }
        else
        {
          start_goal_msg.markers[i].ns = "Goal";
          start_goal_msg.markers[i].type = visualization_msgs::Marker::CUBE;
          start_goal_msg.markers[i].color.g = 1.0;
          start_goal_msg.markers[i].color.a = 1.0;
        }

        start_goal_msg.markers[i].scale.x = 0.1;
        start_goal_msg.markers[i].scale.y = 0.1;
        start_goal_msg.markers[i].scale.z = 0.1;

        start_goal_msg.markers[i].pose.position.x = start_goal_x[i];
        start_goal_msg.markers[i].pose.position.y = start_goal_y[i];
        start_goal_msg.markers[i].pose.position.z = marker_height;
        //ROS_INFO("i: %d x: %f  y: %f", i, start_goal_msg.markers[i].pose.position.x, start_goal_msg.markers[i].pose.position.y);
      }
    }

    marker_array_pub.publish(curve_array_msg);
    marker_array_pub.publish(start_goal_msg);
    //marker_array_pub.publish(goal_paths_msg);
    marker_array_pub.publish(target_paths_msg);
    marker_array_pub.publish(target_points_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "paths");
  ros::NodeHandle nh;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("Paths", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("Nodes", 10);

  ROS_INFO("Init paths");
  Paths paths = Paths(nh, marker_array_pub, marker_pub);

  ros::Rate loop_rate(control_frequency);

  while (ros::ok())
  {

    paths.updatePaths();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
