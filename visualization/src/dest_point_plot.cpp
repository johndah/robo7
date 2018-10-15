//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class dest_point_plot
{
public:
  ros::NodeHandle n;
  ros::Subscriber point_dest;
  ros::Publisher point_rviz;

  dest_point_plot()
  {
    n = ros::NodeHandle("~");

    //XY_coordinates = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 1000, &laserXY::coordinates_callBack, this);
    point_dest = n.subscribe("/point_destination_robot", 1, &dest_point_plot::point_callBack, this);
    point_rviz = n.advertise<geometry_msgs::PointStamped>("/point_ploted", 1);
  }

  void point_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x_point = msg->linear.x;
      y_point = msg->linear.y;
      z_point = msg->linear.z;
  }

  void updatePoint()
  {
    geometry_msgs::PointStamped pointer;
    geometry_msgs::Point point;
    std_msgs::Header _header;

    _header.frame_id = "robot";

    z_point = 0;
    point.x = x_point;
    point.y = y_point;
    point.z = z_point;

    pointer.point = point;
    pointer.header = _header;

    point_rviz.publish( pointer );

  }

private:
  //Robot position parameters
  float x_point;
  float y_point;
  float z_point;

};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "dest_point_plot");

    dest_point_plot _dest_point;

    ros::Rate loop_rate(control_frequency);

    while(_dest_point.n.ok())
    {
        _dest_point.updatePoint();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
