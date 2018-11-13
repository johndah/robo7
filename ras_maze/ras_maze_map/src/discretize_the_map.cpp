// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

//The services
#include <robo7_srvs/discretize_map.h>

//Input all the libraries needed
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <math.h>
#include <ros/ros.h>

//The messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallPoint.h>

//The extra libraries
#include </usr/include/eigen3/Eigen/Dense>
using Eigen::MatrixXd;

// Control @ 10 Hz
double control_frequency = 20.0;

class discretize_the_map
{
public:
  ros::NodeHandle n;
  ros::Publisher point_coordinates;
  ros::Publisher point_cloud_input;
  ros::ServiceServer scan_service;

  discretize_the_map()
  {
    scan_service = n.advertiseService("/maze_map/map_discretization", &discretize_the_map::discretization_Sequence, this);

    point_coordinates = n.advertise<robo7_msgs::XY_coordinates>("/scan/point_cloud_coordinates", 1);
    point_cloud_input = n.advertise<robo7_msgs::cornerList>("/lidar_map/point_cloud", 1);
  }

  bool discretization_Sequence(robo7_srvs::discretize_map::Request &req,
         robo7_srvs::discretize_map::Response &res)
	{
    wall_list = req.walls;

    //Initialisation
    wall_discretized.number = 0;
    wall_discretized.the_points.clear();

    ROS_INFO("nb of walls : %d", wall_list.number);

    for(int i = 0; i < wall_list.number; i++)
    {
        x1 = wall_list.walls[i].init_point.x;
        y1 = wall_list.walls[i].init_point.y;
        x2 = wall_list.walls[i].end_point.x;
        y2 = wall_list.walls[i].end_point.y;

        //Discretized map
        ROS_INFO("nb of inliers : %d", wall_list.walls[i].nb_inliers);
        N_step = wall_list.walls[i].nb_inliers;
        x_step = (x2-x1)/N_step;
        y_step = (y2-y1)/N_step;
        aPoint.x = x1;
        aPoint.y = y1;
        aPoint.z = 0;
        for(int i=0; i<N_step + 1; i++)
        {
          aPoint.x += i*x_step;
          aPoint.y += i*y_step;
          wall_discretized.the_points.push_back(aPoint);
          wall_discretized.number++;
        }
    }

    res.discretized_walls = wall_discretized;
    res.success = true;
    return true;
  }


private:
  //Input and output of the service
  robo7_msgs::wallList wall_list;
  robo7_msgs::wallPoint wall_discretized;

  geometry_msgs::Vector3 aPoint;
  int N_step;

  float x1, y1, x2, y2;
  float x_step, y_step;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "discretize_the_map");

	discretize_the_map discretize_the_map_;

	ros::Rate loop_rate(100);

	ROS_INFO("Discretization of the map running");

	ros::spin();

	return 0;
}
