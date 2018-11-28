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

      //The Subscribers
      local_occupancy_grid_sub = n.subscribe("/mapping/local_occupancy_grid", 1000, &MappingOccupancy::local_occupancy_Callback, this);
      current_occupancy_grid_sub = n.subscribe("/mapping/updated_occupancy_grid", 1000, &MappingOccupancy::updated_occupancy_Callback, this);
      current_point_cloud_sub = n.subscribe("/mapping/point_occ_cloud", 1000, &MappingOccupancy::cloud_Callback, this);
      current_walls_sub = n.subscribe("/mapping/ransac_walls", 1000, &MappingOccupancy::ransac_walls_Callback, this);
      current_discretized_walls_sub = n.subscribe("/mapping/discretized_map", 1000, &MappingOccupancy::discretized_map_Callback, this);

      //The Publishers
      local_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualization/mapping/local_occupancy_grid", 1);
      current_occupancy_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualization/mapping/updated_occupancy_grid", 1);
      current_walls_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/mapping/slam_walls", 1);
      current_discretized_walls_pub = n.advertise<sensor_msgs::PointCloud>("/visualization/mapping/slam_walls_discretized", 1);
      the_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/visualization/mapping/slam_cloud", 1);
    }

    void local_occupancy_Callback(const robo7_msgs::mapping_grid::ConstPtr &msg)
    {
      local_occupancy_grid_msg = *msg;
      local_grid_received = true;
    }

    void updated_occupancy_Callback(const robo7_msgs::mapping_grid::ConstPtr &msg)
    {
      current_occupancy_grid_msg = *msg;
      global_grid_received = true;
    }

    void ransac_walls_Callback(const robo7_msgs::wallList::ConstPtr &msg)
    {
      ransac_walls_msg = *msg;
      local_grid_received = true;
    }

    void discretized_map_Callback(const robo7_msgs::wallPoint::ConstPtr &msg)
    {
      discretized_walls_msg = *msg;
      local_grid_received = true;
    }

    void cloud_Callback(const robo7_msgs::wallPoint::ConstPtr &msg)
    {
      the_point_cloud_msg = *msg;
      local_grid_received = true;
    }

    void updateLocalOccupancyGrid()
    {
      occupancy_array.clear();

      for (int i = local_occupancy_grid_msg.occupancy_grid.nb_rows - 1; i > -1; i--)
      {
          for (int j = 0; j < local_occupancy_grid_msg.occupancy_grid.nb_cols; j++)
          {
            occupancy_array.push_back(local_occupancy_grid_msg.occupancy_grid.rows[i].cols[j]);
          }
      }

      if (local_grid_received)
      {
        nav_msgs::OccupancyGrid occupancy_grid;

        occupancy_grid.header.frame_id = "/map";
        occupancy_grid.header.stamp = ros::Time::now();

        occupancy_grid.info.resolution = local_occupancy_grid_msg.cell_size;
        occupancy_grid.info.width = local_occupancy_grid_msg.occupancy_grid.nb_cols;
        occupancy_grid.info.height = local_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.origin.position.x = local_occupancy_grid_msg.top_left_corner.x;
        occupancy_grid.info.origin.position.y = local_occupancy_grid_msg.top_left_corner.y - local_occupancy_grid_msg.cell_size * local_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.origin.position.z = path_height;

        for (int i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
        {
          int grid_value = (int)100 * occupancy_array[i];
          occupancy_grid.data.push_back(grid_value);
        }
        local_occupancy_grid_pub.publish(occupancy_grid);
        local_grid_received = false;
      }
    }

    void updateCurrentOccupancyGrid()
    {
      occupancy_array.clear();

      for (int i = current_occupancy_grid_msg.occupancy_grid.nb_rows - 1; i > -1; i--)
      {
          for (int j = 0; j < current_occupancy_grid_msg.occupancy_grid.nb_cols; j++)
          {
            occupancy_array.push_back(current_occupancy_grid_msg.occupancy_grid.rows[i].cols[j]);
          }
      }

      if (global_grid_received)
      {
        nav_msgs::OccupancyGrid occupancy_grid;

        occupancy_grid.header.frame_id = "/map";
        occupancy_grid.header.stamp = ros::Time::now();

        occupancy_grid.info.resolution = current_occupancy_grid_msg.cell_size;
        occupancy_grid.info.width = current_occupancy_grid_msg.occupancy_grid.nb_cols;
        occupancy_grid.info.height = current_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.origin.position.x = current_occupancy_grid_msg.top_left_corner.x;
        occupancy_grid.info.origin.position.y = current_occupancy_grid_msg.top_left_corner.y - current_occupancy_grid_msg.cell_size * current_occupancy_grid_msg.occupancy_grid.nb_rows;
        occupancy_grid.info.origin.position.z = path_height;

        for (int i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
        {
          int grid_value = (int)100 * occupancy_array[i];
          occupancy_grid.data.push_back(grid_value);
        }
        current_occupancy_grid_pub.publish(occupancy_grid);
        global_grid_received = false;
      }
    }

    void updateRansacWalls()
    {
      update_former_wall_marker();
      current_walls_pub.publish( wall_marker_array );
    }

    void updateDiscretizedMap()
    {
      updateCoordinates();
      current_discretized_walls_pub.publish( discretized_points );
    }

    void updateCloud()
    {
      updateCoordinates2();
      the_cloud_pub.publish( cloud_points );
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

  void update_former_wall_marker()
  {
    wall_marker_array.markers.clear();
    wall_id = 0;
    for(int i=0; i < ransac_walls_msg.number; i++)
    {
      x2 = ransac_walls_msg.walls[i].init_point.x;
      y2 = ransac_walls_msg.walls[i].init_point.y;
      x1 = ransac_walls_msg.walls[i].end_point.x;
      y1 = ransac_walls_msg.walls[i].end_point.y;

      // angle and distance
      float angle = atan2(y2-y1,x2-x1);
      float dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

      // set pose
      wall_marker.scale.x = dist;
      wall_marker.pose.position.x = (x1+x2)/2;
      wall_marker.pose.position.y = (y1+y2)/2;
      wall_marker.text = "";
      tf::Quaternion quat; quat.setRPY(0.0, 0.0, angle);
      tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

      // add to array
      wall_marker.id = wall_id;
      wall_marker_array.markers.push_back(wall_marker);
      wall_id++;
    }
  }

  void updateCoordinates()
  {
    //Generate the future published twist msg
    geometry_msgs::Point32 one_point;

    std::vector<geometry_msgs::Point32> point_list(discretized_walls_msg.number, one_point);

    for(int i=0; i < discretized_walls_msg.number; i++)
    {
      one_point.x = discretized_walls_msg.the_points[i].x;
      one_point.y = discretized_walls_msg.the_points[i].y;
      one_point.z = discretized_walls_msg.the_points[i].z;
      point_list[i] = one_point;
    }

    discretized_points.points = point_list;
    discretized_points.header.frame_id = "map";
  }

  void updateCoordinates2()
  {
    //Generate the future published twist msg
    geometry_msgs::Point32 one_point;

    std::vector<geometry_msgs::Point32> point_list(the_point_cloud_msg.number, one_point);

    for(int i=0; i < the_point_cloud_msg.number; i++)
    {
      one_point.x = the_point_cloud_msg.the_points[i].x;
      one_point.y = the_point_cloud_msg.the_points[i].y;
      one_point.z = the_point_cloud_msg.the_points[i].z;
      point_list[i] = one_point;
    }

    cloud_points.points = point_list;
    cloud_points.header.frame_id = "map";
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
      MappingOccupancy_.updateCurrentOccupancyGrid();
      MappingOccupancy_.updateRansacWalls();
      MappingOccupancy_.updateDiscretizedMap();
      MappingOccupancy_.updateCloud();

      ros::spinOnce();
      loop_rate.sleep();
    }
}
