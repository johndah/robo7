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

class wall_plots
{
public:
  ros::NodeHandle n;
  ros::Subscriber former_wall_sub;
  ros::Subscriber lidar_wall_sub;
  ros::Subscriber new_wall_sub;
  ros::Publisher former_wall_pub;
  ros::Publisher lidar_wall_pub;
  ros::Publisher new_wall_pub;

  wall_plots()
  {
    initialize_markers();

    former_wall_sub = n.subscribe("/localization/mapping/former_map", 1, &wall_plots::former_wall_callBack, this);
    lidar_wall_sub = n.subscribe("/localization/mapping/lidar_map", 1, &wall_plots::lidar_wall_callBack, this);
    new_wall_sub = n.subscribe("/localization/mapping/new_map", 1, &wall_plots::new_wall_callBack, this);

    former_wall_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/mapping/former_wall", 1 );
    lidar_wall_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/mapping/lidar_wall", 1 );
    new_wall_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/mapping/new_wall", 1 );
  }

  void former_wall_callBack(const robo7_msgs::wallList::ConstPtr &msg)
  {
    former_walls = *msg;
  }

  void lidar_wall_callBack(const robo7_msgs::wallList::ConstPtr &msg)
  {
    lidar_walls = *msg;
  }

  void new_wall_callBack(const robo7_msgs::wallList::ConstPtr &msg)
  {
    new_walls = *msg;
  }

  void updateWalls()
  {
    update_former_wall_marker();
    update_lidar_wall_marker();
    update_new_wall_marker();

    former_wall_pub.publish( former_wall_marker_array );
    lidar_wall_pub.publish( lidar_wall_marker_array );
    new_wall_pub.publish( new_wall_marker_array );
  }

private:
  //The subscriber variables
  robo7_msgs::wallList former_walls;
  robo7_msgs::wallList lidar_walls;
  robo7_msgs::wallList new_walls;

  int wall_id;

  //wall coordinates
  double x1; double y1;
  double x2; double y2;
  double angle;
  double dist;

  //All the markers used
  visualization_msgs::MarkerArray former_wall_marker_array;
  visualization_msgs::Marker former_wall_marker;

  visualization_msgs::MarkerArray lidar_wall_marker_array;
  visualization_msgs::Marker lidar_wall_marker;

  visualization_msgs::MarkerArray new_wall_marker_array;
  visualization_msgs::Marker new_wall_marker;


  void initialize_markers()
  {
    //The former_wall markers
    former_wall_marker.header.frame_id = "/map";
    former_wall_marker.header.stamp = ros::Time();
    former_wall_marker.ns = "world";
    former_wall_marker.type = visualization_msgs::Marker::CUBE;
    former_wall_marker.action = visualization_msgs::Marker::ADD;
    former_wall_marker.scale.y = 0.01;
    former_wall_marker.scale.z = 0.1;
    former_wall_marker.color.a = 1.0;
    former_wall_marker.color.r = (0.0/255.0);
    former_wall_marker.color.g = (0.0/255.0);
    former_wall_marker.color.b = (255.0/255.0);
    former_wall_marker.pose.position.z = 0.2;

    //The lidar_wall markers
    lidar_wall_marker = former_wall_marker;
    lidar_wall_marker.color.r = (255.0/255.0);
    lidar_wall_marker.color.g = (0.0/255.0);
    lidar_wall_marker.color.b = (0.0/255.0);

    //The new_wall markers
    new_wall_marker = former_wall_marker;
    new_wall_marker.color.r = (0.0/255.0);
    new_wall_marker.color.g = (255.0/255.0);
    new_wall_marker.color.b = (0.0/255.0);
  }

  void update_former_wall_marker()
  {
    former_wall_marker_array.markers.clear();
    wall_id = 0;
    for(int i=0; i < former_walls.number; i++)
    {
      x2 = former_walls.walls[i].init_point.x;
      y2 = former_walls.walls[i].init_point.y;
      x1 = former_walls.walls[i].end_point.x;
      y1 = former_walls.walls[i].end_point.y;

      // angle and distance
      angle = atan2(y2-y1,x2-x1);
      dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

      // set pose
      former_wall_marker.scale.x = std::max(0.01,dist);
      former_wall_marker.pose.position.x = (x1+x2)/2;
      former_wall_marker.pose.position.y = (y1+y2)/2;
      former_wall_marker.text = "";
      tf::Quaternion quat; quat.setRPY(0.0, 0.0, angle);
      tf::quaternionTFToMsg(quat, former_wall_marker.pose.orientation);

      // add to array
      former_wall_marker.id = wall_id;
      former_wall_marker_array.markers.push_back(former_wall_marker);
      wall_id++;
    }
  }

  void update_lidar_wall_marker()
  {
    lidar_wall_marker_array.markers.clear();
    wall_id = 0;
    for(int i=0; i < lidar_walls.number; i++)
    {
      x2 = lidar_walls.walls[i].init_point.x;
      y2 = lidar_walls.walls[i].init_point.y;
      x1 = lidar_walls.walls[i].end_point.x;
      y1 = lidar_walls.walls[i].end_point.y;

      // angle and distance
      angle = atan2(y2-y1,x2-x1);
      dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

      // set pose
      lidar_wall_marker.scale.x = std::max(0.01,dist);
      lidar_wall_marker.pose.position.x = (x1+x2)/2;
      lidar_wall_marker.pose.position.y = (y1+y2)/2;
      lidar_wall_marker.text = "";
      tf::Quaternion quat; quat.setRPY(0.0, 0.0, angle);
      tf::quaternionTFToMsg(quat, lidar_wall_marker.pose.orientation);

      // add to array
      lidar_wall_marker.id = wall_id;
      lidar_wall_marker_array.markers.push_back(lidar_wall_marker);
      wall_id++;
    }
  }

  void update_new_wall_marker()
  {
    new_wall_marker_array.markers.clear();
    wall_id = 0;
    for(int i=0; i < new_walls.number; i++)
    {
      x2 = new_walls.walls[i].init_point.x;
      y2 = new_walls.walls[i].init_point.y;
      x1 = new_walls.walls[i].end_point.x;
      y1 = new_walls.walls[i].end_point.y;

      // angle and distance
      angle = atan2(y2-y1,x2-x1);
      dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

      // set pose
      new_wall_marker.scale.x = std::max(0.01,dist);
      new_wall_marker.pose.position.x = (x1+x2)/2;
      new_wall_marker.pose.position.y = (y1+y2)/2;
      new_wall_marker.text = "";
      tf::Quaternion quat; quat.setRPY(0.0, 0.0, angle);
      tf::quaternionTFToMsg(quat, new_wall_marker.pose.orientation);

      // add to array
      new_wall_marker.id = wall_id;
      new_wall_marker_array.markers.push_back(new_wall_marker);
      wall_id++;
    }
  }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_lidar_plot");

    wall_plots wall_plots_;

    ros::Rate loop_rate(control_frequency);

    while(wall_plots_.n.ok())
    {
        wall_plots_.updateWalls();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
