#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <string>
#include <vector>

//The messages
#include <tf/tf.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <robo7_msgs/occupancy_matrix.h>
#include <robo7_msgs/mapping_grid.h>
#include <robo7_msgs/occupancy_row.h>
#include <robo7_msgs/grid_matrix.h>
#include <robo7_msgs/grid_row.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/wallPoint.h>

float path_height = 0.0;


class MappingOccupancy
{
  public:
    ros::NodeHandle n;
    //Subscribers
    ros::Subscriber local_occupancy_grid_sub;
    ros::Subscriber current_occupancy_grid_sub;
    ros::Subscriber current_walls_sub;
    ros::Subscriber current_discretized_walls_sub;
    ros::Subscriber current_point_cloud_sub;
    //Publishers
    ros::Publisher local_occupancy_grid_pub;
    ros::Publisher current_occupancy_grid_pub;
    ros::Publisher current_walls_pub;
    ros::Publisher current_discretized_walls_pub;
    ros::Publisher the_cloud_pub;

    MappingOccupancy()
    {
      //Initialization
      initialize_markers();
      pub = false;

      //The Subscribers
      local_occupancy_grid_sub = n.subscribe("/localization/mapping/the_occupancy_grid", 1000, &MappingOccupancy::local_occupancy_Callback, this);

      //The Publishers
      local_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualization/mapping/the_occupancy_grid", 1);
    }

    void local_occupancy_Callback(const robo7_msgs::mapping_grid::ConstPtr &msg)
    {
      local_occupancy_grid_msg = *msg;
      local_grid_received = true;
    }

    void updateLocalOccupancyGrid()
    {
      nav_msgs::OccupancyGrid occupancy_grid;
      occupancy_array.clear();

      for (int j = 0; j < local_occupancy_grid_msg.occupancy_grid.nb_rows; j++)
      {
          for (int i = 0; i < local_occupancy_grid_msg.occupancy_grid.nb_cols; i++)
          {
            occupancy_array.push_back(local_occupancy_grid_msg.occupancy_grid.rows[i].cols[j]);
          }
      }

      if (local_grid_received)
      {
        occupancy_grid.header.frame_id = "/map";
        occupancy_grid.header.stamp = ros::Time::now();

        occupancy_grid.info.resolution = local_occupancy_grid_msg.cell_size;
        occupancy_grid.info.width = local_occupancy_grid_msg.occupancy_grid.nb_cols;
        occupancy_grid.info.height = local_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.origin.position.x = local_occupancy_grid_msg.top_left_corner.x;
        occupancy_grid.info.origin.position.y = local_occupancy_grid_msg.top_left_corner.y;
        occupancy_grid.info.origin.position.z = path_height;

        for (int i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
        {
          int grid_value = (int)100 * occupancy_array[i];
          occupancy_grid.data.push_back(grid_value);
        }

        // local_grid_received = false;
        pub = true;
      }

      if(pub)
      {
        local_occupancy_grid_pub.publish(occupancy_grid);
      }
    }


private:
  //The subscribers msgs
  robo7_msgs::mapping_grid local_occupancy_grid_msg;
  robo7_msgs::mapping_grid current_occupancy_grid_msg;
  robo7_msgs::wallList ransac_walls_msg;
  robo7_msgs::wallPoint discretized_walls_msg;
  robo7_msgs::wallPoint the_point_cloud_msg;

  //Grid publishers
  std::vector<float> occupancy_array, distance_array;
  int occupancy_grid_width, occupancy_grid_height, distance_grid_width, distance_grid_height;

  //Map walls and discretized version publishers
  sensor_msgs::PointCloud discretized_points;
  sensor_msgs::PointCloud cloud_points;
  visualization_msgs::MarkerArray wall_marker_array;
  visualization_msgs::Marker wall_marker;
  int wall_id;
  float x1, y1, x2, y2;

  //The new messages received
  bool local_grid_received, global_grid_received;
  bool pub;

  void initialize_markers()
  {
    //The former_wall markers
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
    wall_marker.pose.position.z = 0;
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_occupancy");

    ROS_INFO("Mapping Occupancy Visualization");

    MappingOccupancy MappingOccupancy_;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      MappingOccupancy_.updateLocalOccupancyGrid();
      // MappingOccupancy_.updateCurrentOccupancyGrid();
      // MappingOccupancy_.updateRansacWalls();
      // MappingOccupancy_.updateDiscretizedMap();
      // MappingOccupancy_.updateCloud();

      ros::spinOnce();
      loop_rate.sleep();
    }
}
