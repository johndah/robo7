//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Control @ 10 Hz
double control_frequency = 100.0;


class lidar_plot
{
public:
  ros::NodeHandle n;
  ros::Subscriber lidar_scan_sub;
  ros::Publisher lidar_scan_pub;


  lidar_plot()
  {
    lidar_scan_sub = n.subscribe("/scan", 1, &lidar_plot::lidar_callBack, this);

    lidar_scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan2", 1);
  }

  void lidar_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    scan = *msg;
    scan.header.frame_id = "laser2";
    lidar_scan_pub.publish( scan );
  }


private:
  sensor_msgs::LaserScan scan;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_plot");

    lidar_plot lidar_frame_;

    ros::Rate loop_rate(control_frequency);

    while(lidar_frame_.n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
