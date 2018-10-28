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

// Control @ 10 Hz
double control_frequency = 10.0;


class plot_map
{
public:
  ros::NodeHandle n;
  ros::Subscriber XY_coordinates;
  ros::Publisher point_cloud;

  plot_map()
  {
    n = ros::NodeHandle("~");
    length = 0;

    // XY_coordinates = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 1000, &plot_map::coordinates_callBack, this);
    XY_coordinates = n.subscribe("/own_map/wall_coordinates", 1000, &plot_map::coordinates_callBack, this);

    point_cloud = n.advertise<sensor_msgs::PointCloud>("/own_map/plot_map", 1000);
  }

  void coordinates_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
      length = msg->length;
      converted_X_coordinates = msg->X_coordinates;
      converted_Y_coordinates = msg->Y_coordinates;
  }

  void updateCoordinates()
  {
    //Generate the future published twist msg
    sensor_msgs::PointCloud points;
    geometry_msgs::Point32 one_point;

    point_list = std::vector<geometry_msgs::Point32>(length, one_point);

    for(int i=0; i < length; i++)
    {
      one_point.x = converted_X_coordinates[i];
      one_point.y = converted_Y_coordinates[i];
      point_list[i] = one_point;
    }

    points.points = point_list;
    points.header.frame_id = "map";

    point_cloud.publish( points );

  }

private:
  //Robot position parameters
  std::vector<float> converted_X_coordinates;
  std::vector<float> converted_Y_coordinates;
  int length;

  //Point list vector
  std::vector<geometry_msgs::Point32> point_list;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_map");

    plot_map plot_map_;

    ros::Rate loop_rate(control_frequency);

    while(plot_map_.n.ok())
    {
        plot_map_.updateCoordinates();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
