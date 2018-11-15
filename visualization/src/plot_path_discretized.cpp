//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <robo7_msgs/trajectory.h>
#include <geometry_msgs/Vector3.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class plot_trajectory
{
public:
  ros::NodeHandle n;
  ros::Subscriber trajectory_cloud;
  ros::Publisher point_cloud;

  plot_trajectory()
  {
    trajectory_cloud = n.subscribe("/path_planning/trajectory", 1, &plot_trajectory::trajectory_Callback, this);

    point_cloud = n.advertise<sensor_msgs::PointCloud>("/visualization/plot_trajectory_cloud", 1);
  }

  void trajectory_Callback(const robo7_msgs::trajectory::ConstPtr &msg)
	{
		trajectory_array = *msg;
	}

  void updateCoordinates()
  {
    //Generate the future published twist msg
    sensor_msgs::PointCloud points;
    geometry_msgs::Point32 one_point;

    point_list = std::vector<geometry_msgs::Point32>(static_cast<int>(trajectory_array.trajectory_points.size()), one_point);

    // ROS_INFO("List size : %d", static_cast<int>(trajectory_array.trajectory_points.size()));

    for(int i=0; i < static_cast<int>(trajectory_array.trajectory_points.size()); i++)
    {
      one_point.x = trajectory_array.trajectory_points[i].point_coord.x;
      one_point.y = trajectory_array.trajectory_points[i].point_coord.y;
      one_point.z = trajectory_array.trajectory_points[i].point_coord.z;
      point_list[i] = one_point;
    }

    points.points = point_list;
    points.header.frame_id = "map";

    point_cloud.publish( points );

  }

private:
  //Robot position parameters
  robo7_msgs::trajectory trajectory_array;

  //Point list vector
  std::vector<geometry_msgs::Point32> point_list;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_trajectory");

    plot_trajectory plot_trajectory_;

    ros::Rate loop_rate(control_frequency);

    while(plot_trajectory_.n.ok())
    {
        plot_trajectory_.updateCoordinates();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
