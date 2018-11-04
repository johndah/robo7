//Input all the libraries needed
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/XY_coordinates.h>
#include </usr/include/eigen3/Eigen/Dense>
using Eigen::MatrixXd;

#include <robo7_srvs/scanCoord.h>

// Control @ 10 Hz
double control_frequency = 20.0;

class lidar_map_coordinates
{
public:
  ros::NodeHandle n;
  ros::Publisher point_coordinates;
  ros::ServiceServer scan_service;

  lidar_map_coordinates()
  {
    //Dealing with the shifted angle of the lidar in the robot frame
    n.param<float>("/lidar_map_coordinates/lidar_angle", shifted_angle, 0);

    // ROS_INFO("Angle : %lf", shifted_angle);

    pi = 3.14159265358979323846;

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations

    point_coordinates = n.advertise<robo7_msgs::XY_coordinates>("/scan/point_cloud_coordinates", 1);
    scan_service = n.advertiseService("/localization/scan_service", &lidar_map_coordinates::scan_Sequence, this);
  }

  bool scan_Sequence(robo7_srvs::scanCoord::Request &req,
         robo7_srvs::scanCoord::Response &res)
	{
    ROS_INFO("start");
    //CallBack all the datas out of the request message
    // ROS_INFO("%lf", req.lidar_scan.angle_min);
    angle_min = req.lidar_scan.angle_min;
    angle_max = req.lidar_scan.angle_max;
    angle_increment = req.lidar_scan.angle_increment;
    points_distances = req.lidar_scan.ranges;
    points_intensities = req.lidar_scan.intensities;
    range_min = req.lidar_scan.range_min;
    range_max = req.lidar_scan.range_max;

    robot_x = req.robot_position.linear.x;
    robot_y = req.robot_position.linear.y;
    robot_theta = req.robot_position.angular.z;

    //Generate the future published twist msg
    robo7_msgs::XY_coordinates point_XY;


    //Initialize the first angle true value
    angle = angle_min + shifted_angle;
    XY_coordinates.resize(points_distances.size(),2);
    length = 0;

    angles_ = std::vector<float>(points_distances.size(), 0);
    converted_X_coordinates = std::vector<float>(points_distances.size(), 0);
    converted_Y_coordinates = std::vector<float>(points_distances.size(), 0);

    if(false)
    {
      // Count the non-zeros instances
      for(int i=0; i < points_distances.size(); i++)
      {
        if(points_intensities[i] > 0.0)
        {
          XY_coordinates(length,0) = points_distances[i] * cos(angle);
          XY_coordinates(length,1) = points_distances[i] * sin(angle);
          length++;
        }
        angles_[i] = angle;
        angle = angle + angle_increment;
      }

      converted_X_coordinates = std::vector<float>(length, 0);
      converted_Y_coordinates = std::vector<float>(length, 0);
      for(int i=0; i < length; i++)
      {
        converted_X_coordinates[i] = XY_coordinates(i,0);
        converted_Y_coordinates[i] = XY_coordinates(i,1);
      }
    }
    else
    {
      for(int i=0; i < points_distances.size(); i++)
      {
        if(points_intensities[i] > 0.0)
        {
          converted_X_coordinates[length] = points_distances[i] * (cos(angle)*cos(robot_theta) - sin(angle)*sin(robot_theta)) + robot_x;
          converted_Y_coordinates[length] = points_distances[i] * (cos(angle)*sin(robot_theta) + sin(angle)*cos(robot_theta)) + robot_y;
          length++;
        }
        angles_[i] = angle;
        angle = angle + angle_increment;
      }
    }


    point_XY.length = length;
    point_XY.trueX_length = converted_X_coordinates.size();
    point_XY.trueY_length = converted_Y_coordinates.size();
    point_XY.X_coordinates = converted_X_coordinates;
    point_XY.Y_coordinates = converted_Y_coordinates;
    point_XY.angles = angles_;

    res.point_cloud_coordinates = point_XY;
    res.success = true;
    point_coordinates.publish( point_XY );

    return true;
  }


private:
  //Dealing with the shifted angle
  float shifted_angle;

  //Datas extracted from LaserScan
  float angle_min;
  float angle_max;
  float range_min;
  float range_max;
  float angle_increment;
  std::vector<float> points_distances;
  std::vector<float> points_intensities;

  //Datas extracted from the robot position
  float robot_x;
  float robot_y;
  float robot_theta;

  //Create the final table
  float angle;
  int length;
  MatrixXd XY_coordinates;
  std::vector<float> converted_X_coordinates;
  std::vector<float> converted_Y_coordinates;
  std::vector<float> angles_;

  //Other useful variables
  float pi;
  float Dt;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_map_coordinates");

	lidar_map_coordinates lidar_map_coordinates_;

	ros::Rate loop_rate(100);

	ROS_INFO("Lidar to Map coordinates running");

	ros::spin();

	return 0;
}
