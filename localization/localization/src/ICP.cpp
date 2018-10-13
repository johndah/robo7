//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <robo7_msgs/XY_coordinates.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include </usr/include/eigen3/Eigen/Dense>
#include <stdlib.h>
using Eigen::Matrix4f;

// std includes
#include <limits>
#include <iostream>
#include <fstream>

// Boost includes
#include <stdio.h>
#include <stdlib.h>

using namespace std;

// Control @ 10 Hz
double control_frequency = 100.0;


class ICP
{
public:
  ros::NodeHandle n;
  ros::Subscriber lidar_cloud;
  ros::Subscriber robot_position;
  ros::Subscriber map_cloud;
  ros::Publisher lidar_position;
  ros::Publisher matrix_values;

  ICP()
  {
    pi = 3.14159265358979323846;
    length = 0;
    length_wall;

    robot_position = n.subscribe("/localization/Pos", 10, &ICP::position_callBack, this);
    lidar_cloud = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 10, &ICP::point_cloud_callBack, this);
    map_cloud = n.subscribe("/maze_map/wall_coordinates", 1, &ICP::wall_cloud_callBack, this);
    lidar_position = n.advertise<geometry_msgs::Twist>("Pos", 10);
    matrix_values = n.advertise<robo7_msgs::XY_coordinates>("/matrix_values", 10);

  }

  void point_cloud_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
      length = msg->length;
      converted_X_coordinates = msg->X_coordinates;
      converted_Y_coordinates = msg->Y_coordinates;
  }

  void wall_cloud_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
      length_wall = msg->length;
      wall_X_coordinates = msg->X_coordinates;
      wall_Y_coordinates = msg->Y_coordinates;
  }

  void position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      current_x = msg->linear.x;
      current_y = msg->linear.y;
      current_angle = msg->angular.z;
  }

  void updateICP()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    cloud_source->width    = length;
    cloud_source->height   = 1;
    cloud_source->is_dense = false;
    cloud_source->points.resize (cloud_source->width * cloud_source->height);
    for (size_t i = 0; i < converted_X_coordinates.size(); ++i)
    {
      cloud_source->points[i].x = converted_X_coordinates[i];
      cloud_source->points[i].y = converted_Y_coordinates[i];
      cloud_source->points[i].z = 0;
    }

    cloud_target->width    = length_wall;
    cloud_target->height   = 1;
    cloud_target->is_dense = false;
    cloud_target->points.resize (cloud_target->width * cloud_target->height);
    for (size_t i = 0; i < wall_X_coordinates.size(); ++i)
    {
      cloud_target->points[i].x = wall_X_coordinates[i];
      cloud_target->points[i].y = wall_Y_coordinates[i];
      cloud_target->points[i].z = 0;
    }


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource (cloud_source);
    icp.setInputTarget (cloud_target);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-3);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    // Perform the alignment
    icp.align (cloud_source_registered);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    transformation = icp.getFinalTransformation ();

    geometry_msgs::Twist twist_msg;

    mat_val = vector<float>(1, 0);

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        mat_val.push_back(transformation(i,j));
      }
    }

    robo7_msgs::XY_coordinates point_XY;
    point_XY.length = mat_val.size();
    point_XY.trueX_length = 0;
    point_XY.trueY_length = 0;
    point_XY.X_coordinates = mat_val;
    matrix_values.publish(twist_msg);
  }

private:
  //Localization main parameters
  float current_x;
  float current_y;
  float current_angle;

  //ICP parameters
  float delta_T;
  Eigen::Matrix4f transformation;
  pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
  vector<float> mat_val;


  //Lidar function parameters
  int length;
  vector<float> converted_X_coordinates;
  vector<float> converted_Y_coordinates;

  //Walls parameters
  int length_wall;
  vector<float> wall_X_coordinates;
  vector<float> wall_Y_coordinates;

  string _map_file;
  string _map_frame;
  string _map_topic;

  //other parameters
  float pi;
};







int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICP");

    ICP _ICP;

    ros::Rate loop_rate(control_frequency);

    while(_ICP.n.ok())
    {
        _ICP.updateICP();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
