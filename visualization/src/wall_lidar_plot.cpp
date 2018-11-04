// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>

// std includes
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

double control_frequency = 1;

class wall_lidar_plot
{
public:
  ros::NodeHandle n;
  ros::Subscriber wall_list;
  ros::Publisher wall_pub;

  wall_lidar_plot()
  {

    wall_list = n.subscribe("/localization/ransac/walls", 1, &wall_lidar_plot::wall_list_callBack, this);
    wall_pub = n.advertise<visualization_msgs::MarkerArray>("/walls_array", 1 );

  }

  void wall_list_callBack(const robo7_msgs::wallList::ConstPtr &msg)
  {
    extract_walls = msg->walls;
  }

  void updateWalls()
  {

      visualization_msgs::MarkerArray all_markers;
      visualization_msgs::Marker wall_marker;
      wall_marker.header.frame_id = "/map";
      wall_marker.header.stamp = ros::Time();
      wall_marker.ns = "world";
      wall_marker.type = visualization_msgs::Marker::CUBE;
      wall_marker.action = visualization_msgs::Marker::ADD;
      wall_marker.scale.y = 0.01;
      wall_marker.scale.z = 0.1;
      wall_marker.color.a = 1.0;
      wall_marker.color.r = (0.0/255.0);
      wall_marker.color.g = (0.0/255.0);
      wall_marker.color.b = (255.0/255.0);
      wall_marker.pose.position.z = 0.2;


      wall_id = 0;
      for(int i=0; (i<20)&&(i<static_cast<int>(extract_walls.size())); i++)
      {

          // ROS_INFO("New Wall");
          x2 = extract_walls[i].init_point.x;
          y2 = extract_walls[i].init_point.y;
          x1 = extract_walls[i].end_point.x;
          y1 = extract_walls[i].end_point.y;
          // ROS_INFO("x1 : %lf, y1 : %lf, x2 : %lf, y2 : %lf", x1, y1, x2, y2);

          // angle and distance
          angle = atan2(y2-y1,x2-x1);
          dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

          // set pose
          wall_marker.scale.x = std::max(0.01,dist);
          wall_marker.pose.position.x = (x1+x2)/2;
          wall_marker.pose.position.y = (y1+y2)/2;
          wall_marker.text = "";
          tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
          tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

          // add to array
          wall_marker.id = wall_id;
          all_markers.markers.push_back(wall_marker);
          wall_id++;
      }

      wall_pub.publish( all_markers );
  }

private:
  std::vector<robo7_msgs::aWall> extract_walls;

  int wall_id;

  //wall coordinates
  double x1; double y1;
  double x2; double y2;
  double angle;
  double dist;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_lidar_plot");

    wall_lidar_plot wall_plot_;

    ros::Rate loop_rate(control_frequency);

    while(wall_plot_.n.ok())
    {
        wall_plot_.updateWalls();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
