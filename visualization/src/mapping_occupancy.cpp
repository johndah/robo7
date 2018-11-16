
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robo7_msgs/occupancy_matrix.h>
#include <robo7_msgs/mapping_grid.h>
#include <robo7_msgs/occupancy_row.h>
#include "robo7_msgs/grid_matrix.h"
#include "robo7_msgs/grid_row.h"
#include <std_msgs/Int8MultiArray.h>
#include <algorithm>

float path_height = 0.0;


class MappingOccupancy
{
  public:
    ros::NodeHandle n;
    ros::Subscriber occupancy_grid_sub, distance_grid_sub;
    ros::Publisher occupancy_grid_pub, distance_grid_pub;

    MappingOccupancy()
    {
      occupancy_grid_sub = n.subscribe("/mapping/local_occupancy_grid", 1000, &MappingOccupancy::local_occupancy_Callback, this);

      occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualization/mapping/local_occupancy_grid", 1);
    }

    void local_occupancy_Callback(const robo7_msgs::mapping_grid::ConstPtr &msg)
    {
      local_occupancy_grid_msg = *msg;
    }

    void updateOccupancyGrid()
    {
      occupancy_array.clear();

      for (int i = local_occupancy_grid_msg.occupancy_grid.nb_rows - 1; i > -1; i--)
      {
          for (int j = 0; j < local_occupancy_grid_msg.occupancy_grid.nb_cols; j++)
          {
            occupancy_array.push_back(local_occupancy_grid_msg.occupancy_grid.rows[i].cols[j]);
          }
      }

      if (this->occupancy_grid_received)
      {
        nav_msgs::OccupancyGrid occupancy_grid;

        occupancy_grid.header.frame_id = "/map";
        occupancy_grid.header.stamp = ros::Time::now();

        occupancy_grid.info.resolution = local_occupancy_grid_msg.cell_size;
        occupancy_grid.info.width = local_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.height = local_occupancy_grid_msg.occupancy_grid.nb_cols;
        occupancy_grid.info.origin.position.x = local_occupancy_grid_msg.top_left_corner.x;
        occupancy_grid.info.origin.position.y = local_occupancy_grid_msg.top_left_corner.y;
        occupancy_grid.info.origin.position.z = path_height;

        for (int i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
        {
          int grid_value = (int)100 * occupancy_array[i];
          occupancy_grid.data.push_back(grid_value);
        }
        occupancy_grid_pub.publish(occupancy_grid);
      }
    }

    void updateDistanceGrid()
    {

        if (this->distance_grid_received)
        {
            int grid_value;
            nav_msgs::OccupancyGrid distance_grid;

            distance_grid.header.frame_id = "/map";
            distance_grid.header.stamp = ros::Time::now();

            distance_grid.info.resolution = 2.44 / distance_grid_width;
            distance_grid.info.width = distance_grid_width;
            distance_grid.info.height = distance_grid_height;
            distance_grid.info.origin.position.x = 0;
            distance_grid.info.origin.position.y = 0;
            distance_grid.info.origin.position.z = path_height;

            float max_distance = *max_element(distance_array.begin(), distance_array.end());
            for (int i = 0; i < distance_grid_width * distance_grid_height; i++)
            {
                if (distance_array[i] == 0)
                {
                    grid_value = 100;
                }
                else
                    grid_value = (int) 100 * distance_array[i] / max_distance;



                distance_grid.data.push_back(grid_value);
            }

            distance_grid_pub.publish(distance_grid);
        }
    }

private:
  //The subscribers msgs
  robo7_msgs::mapping_grid local_occupancy_grid_msg;

  std::vector<float> occupancy_array, distance_array;
  int occupancy_grid_width, occupancy_grid_height, distance_grid_width, distance_grid_height;
  //Initialisation

  bool occupancy_grid_received, distance_grid_received;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_occupancy");

    ROS_INFO("Mapping Occupancy Visualization");

    MappingOccupancy MappingOccupancy_;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {

        MappingOccupancy_.updateOccupancyGrid();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
