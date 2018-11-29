//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <robo7_msgs/wallPoint.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class plot_exploration_points
{
public:
  ros::NodeHandle n;
  ros::Subscriber unexplored_point_sub, explored_point_sub, goal_point_sub;
  ros::Publisher unexplored_point_pub, explored_point_pub, goal_point_pub;

  plot_exploration_points()
  {
    n = ros::NodeHandle("~");

    // XY_coordinates = n.subscribe("/own_map/map_corners", 1000, &plot_map_corners::lidar_corners_Callback, this);
    unexplored_point_sub = n.subscribe("/path_planning/exploration/the_unexplored_points", 1000, &plot_exploration_points::unexploredPointscallBack, this);
    explored_point_sub = n.subscribe("/path_planning/exploration/the_explored_points", 1000, &plot_exploration_points::exploredPointscallBack, this);
    goal_point_sub = n.subscribe("/path_planning/exploration/the_goal_point", 1000, &plot_exploration_points::goalPointscallBack, this);

    unexplored_point_pub = n.advertise<sensor_msgs::PointCloud>("/visualization/exploration/plot_unexplored_points", 1000);
    explored_point_pub = n.advertise<sensor_msgs::PointCloud>("/visualization/exploration/plot_explored_points", 1000);
    goal_point_pub = n.advertise<geometry_msgs::PointStamped>("/visualization/exploration/plot_goal_point", 1000);
  }

  void unexploredPointscallBack(const robo7_msgs::wallPoint::ConstPtr &msg)
	{
		unexplored_points_msg = *msg;
	}

  void exploredPointscallBack(const robo7_msgs::wallPoint::ConstPtr &msg)
	{
		explored_points_msg = *msg;
	}

  void goalPointscallBack(const geometry_msgs::Vector3::ConstPtr &msg)
	{
		goal_point_msg = *msg;
	}

  void updateCoordinates()
  {
    //Generate the future published twist msg
    sensor_msgs::PointCloud points;
    geometry_msgs::Point32 one_point;

    point_list = std::vector<geometry_msgs::Point32>(unexplored_points_msg.number, one_point);

    for(int i=0; i < unexplored_points_msg.number; i++)
    {
      one_point.x = unexplored_points_msg.the_points[i].x;
      one_point.y = unexplored_points_msg.the_points[i].y;
      one_point.z = unexplored_points_msg.the_points[i].z;
      point_list[i] = one_point;
    }

    points.points = point_list;
    points.header.frame_id = "map";

    unexplored_point_pub.publish( points );

    //Publish the explored points
    point_list = std::vector<geometry_msgs::Point32>(explored_points_msg.number, one_point);

    for(int i=0; i < explored_points_msg.number; i++)
    {
      one_point.x = explored_points_msg.the_points[i].x;
      one_point.y = explored_points_msg.the_points[i].y;
      one_point.z = explored_points_msg.the_points[i].z;
      point_list[i] = one_point;
    }

    points.points = point_list;
    points.header.frame_id = "map";

    explored_point_pub.publish( points );

    //Publish the goal point
    geometry_msgs::PointStamped pointer;
    geometry_msgs::Point point;
    std_msgs::Header _header;
    _header.frame_id = "map";

    point.x = goal_point_msg.x;
    point.y = goal_point_msg.y;
    point.z = goal_point_msg.z;

    pointer.point = point;
    pointer.header = _header;

    goal_point_pub.publish( pointer );
  }

private:
  //Robot position parameters
  robo7_msgs::wallPoint unexplored_points_msg;
  robo7_msgs::wallPoint explored_points_msg;
  geometry_msgs::Vector3 goal_point_msg;

  //Point list vector
  std::vector<geometry_msgs::Point32> point_list;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_map_corners");

    plot_exploration_points plot_exploration_points_;

    ros::Rate loop_rate(control_frequency);

    while(plot_exploration_points_.n.ok())
    {
        plot_exploration_points_.updateCoordinates();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
