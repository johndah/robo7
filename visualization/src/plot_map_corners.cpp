//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <robo7_msgs/XY_coordinates.h>
#include <std_msgs/Header.h>
#include <robo7_msgs/cornerList.h>
#include <geometry_msgs/Vector3.h>

// Control @ 10 Hz
double control_frequency = 100.0;


class plot_map_corners
{
public:
  ros::NodeHandle n;
  ros::Subscriber XY_coordinates;
  ros::Publisher point_cloud;

  plot_map_corners()
  {
    n = ros::NodeHandle("~");

    // XY_coordinates = n.subscribe("/own_map/map_corners", 1000, &plot_map_corners::lidar_corners_Callback, this);
    XY_coordinates = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1000, &plot_map_corners::lidar_corners_Callback, this);

    point_cloud = n.advertise<sensor_msgs::PointCloud>("/visualization/plot_map_corners", 1000);
  }

  void lidar_corners_Callback(const robo7_msgs::cornerList::ConstPtr &msg)
	{
		corner_list.number = msg->number;
		corner_list.corners = msg->corners;
	}

  void updateCoordinates()
  {
    //Generate the future published twist msg
    sensor_msgs::PointCloud points;
    geometry_msgs::Point32 one_point;

    point_list = std::vector<geometry_msgs::Point32>(corner_list.number, one_point);

    for(int i=0; i < corner_list.number; i++)
    {
      one_point.x = corner_list.corners[i].x;
      one_point.y = corner_list.corners[i].y;
      one_point.z = corner_list.corners[i].z;
      point_list[i] = one_point;
    }

    points.points = point_list;
    points.header.frame_id = "map";

    point_cloud.publish( points );

  }

private:
  //Robot position parameters
  robo7_msgs::cornerList corner_list;

  //Point list vector
  std::vector<geometry_msgs::Point32> point_list;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_map_corners");

    plot_map_corners plot_map_corners_;

    ros::Rate loop_rate(control_frequency);

    while(plot_map_corners_.n.ok())
    {
        plot_map_corners_.updateCoordinates();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
