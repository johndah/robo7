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

// Control @ 10 Hz
double control_frequency = 20.0;

class scan_to_coordinates
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber laser_scan;
  ros::Subscriber robot_position;
  ros::Publisher point_coordinates;

  scan_to_coordinates()
  {
    n = ros::NodeHandle("~");
    a = 2;

    //Dealing with the shifted angle of the lidar in the robot frame
    nh.param<float>("/scan_to_coordinates/lidar_angle", shifted_angle, 0);

    pi = 3.14159265358979323846;

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations

    laser_scan = n.subscribe("/scan", 1, &scan_to_coordinates::laser_scan_callBack, this);
    robot_position = n.subscribe("/dead_reckoning/Pos", 1, &scan_to_coordinates::position_callBack, this);
    point_coordinates = n.advertise<robo7_msgs::XY_coordinates>("point_cloud_coordinates", 1);
  }

  void laser_scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
      angle_min = msg->angle_min;
      angle_max = msg->angle_max;
      angle_increment = msg->angle_increment;
      points_distances = msg->ranges;
      points_intensities = msg->intensities;
      range_min = msg->range_min;
      range_max = msg->range_max;
  }

  void position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      robot_x = msg->linear.x;
      robot_y = msg->linear.y;
      robot_theta = msg->angular.z;
  }


  void updateCoordinates(){
    //Generate the future published twist msg
    robo7_msgs::XY_coordinates point_XY;


    //Initialize the first angle true value
    angle = angle_min + shifted_angle;
    XY_coordinates.resize(points_distances.size(),2);
    length = 0;

    angles_ = std::vector<float>(points_distances.size(), 0);
    converted_X_coordinates = std::vector<float>(points_distances.size(), 0);
    converted_Y_coordinates = std::vector<float>(points_distances.size(), 0);

    if(a==1)
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
    point_coordinates.publish(point_XY);
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
  int a;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_coordinates");

    scan_to_coordinates coordinates;

    ros::Rate loop_rate(control_frequency);

    while(coordinates.n.ok())
    {
        coordinates.updateCoordinates();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
