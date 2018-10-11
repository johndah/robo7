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
#include <stdlib.h>
using Eigen::MatrixXd;

// Control @ 10 Hz
double control_frequency = 10.0;

class ransac
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber coordinates;
  ros::Publisher walls;

  ransac()
  {
    n = ros::NodeHandle("~");

    //Parameter Initialisation
    length = 0;

    //Ransac parameters definition
    n_rand = 2;
    mas_iter = 10;

    XY_random_datas.resize(n_rand, 2);

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations

    coordinates = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 1000, &ransac::point_cloud_callBack, this);
    walls = n.advertise<robo7_msgs::XY_coordinates>("point_cloud_coordinates", 1000);
  }

  void point_cloud_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
    length = msg->length;
    converted_X_coordinates = msg->X_coordinates;
    converted_Y_coordinates = msg->Y_coordinates;
  }

  void ransac_algorithm(std::vector<float>[] XY_datas, "model", int n_rand, int max_iter, float threshold, int min_data)
  {
    iter = 0;


  }

  void random_picked(std::vector<float> XY_datas, int n_rand)
  {
    for(int i = 0; i < n_rand; i++)
    {
        rand_indx = rand() % length;
        XY_random_datas(i) = XY_datas(rand_indx);
    }
  }

  void updateWalls(){
    //Generate the future published twist msg
    robo7_msgs::XY_coordinates point_XY;

    XY_datas.resize(length,2);
    for(int i=0; i < length; i++)
    {
      XY_coordinates(i,0) = converted_X_coordinates[i];
      XY_coordinates(i,1) = converted_Y_coordinates[i];
    }

    point_coordinates.publish(point_XY);
  }


private:
  //Robot position parameters
  std::vector<float> converted_X_coordinates;
  std::vector<float> converted_Y_coordinates;
  int length;

  //Variables for the ransac_algorithm() function
  MatrixXd XY_datas;
  MatrixXd XY_random_datas;
  int iter;

  //Variables for random_picked() function
  int rand_indx;
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
