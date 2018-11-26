
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robo7_msgs/occupancy_matrix.h>
#include <robo7_msgs/occupancy_row.h>
#include "robo7_msgs/grid_matrix.h"
#include "robo7_msgs/grid_row.h"
#include <std_msgs/Int8MultiArray.h>
#include <algorithm>

float path_height = 0.0;

class OccupancyGrid
{
  public:
    ros::Subscriber occupancy_grid_sub, distance_grid_sub, exploration_grid_sub;
    ros::Publisher occupancy_grid_pub, distance_grid_pub, exploration_grid_pub;
    std::vector<float> occupancy_array, distance_array, exploration_array;
    int occupancy_grid_width, occupancy_grid_height, distance_grid_width, distance_grid_height, exploration_grid_width, exploration_grid_height;
    //Initialisation

    bool occupancy_grid_received, distance_grid_received, exploration_grid_received;

    OccupancyGrid(ros::NodeHandle nh, ros::Publisher occupancy_grid_pub, ros::Publisher distance_grid_pub, ros::Publisher exploration_grid_pub)
    {
        //occupancy_grid_sub = nh.subscribe("/mapping_grids_server/wall_occupancy_matrix", 1000, &OccupancyGrid::occupancyCallback, this);
        occupancy_grid_sub = nh.subscribe("/heuristic_grids_server/occupancy_matrix", 1000, &OccupancyGrid::occupancyCallback, this);
        distance_grid_sub = nh.subscribe("/heuristic_grids_server/distance_matrix", 1000, &OccupancyGrid::distanceCallback, this);
        exploration_grid_sub = nh.subscribe("/mapping_grids_server/exploration_matrix", 1000, &OccupancyGrid::explorationCallback, this);
        this->occupancy_grid_pub = occupancy_grid_pub;
        this->distance_grid_pub = distance_grid_pub;
        this->exploration_grid_pub = exploration_grid_pub;

        occupancy_grid_received = false;
        distance_grid_received = false;
        exploration_grid_received = false;
    }

    void occupancyCallback(const robo7_msgs::occupancy_matrix::ConstPtr &occupancy_matrix_msg)
    {
        occupancy_array.clear();

        occupancy_grid_width = occupancy_matrix_msg->occupancy_rows.size();
        occupancy_grid_height = occupancy_matrix_msg->occupancy_rows[0].occupancy_row.size();
        ROS_INFO("Received occupancy");

        for (int j = 0; j < occupancy_grid_height; j++)
        {
            for (int i = 0; i < occupancy_grid_width; i++)
            {
                occupancy_array.push_back(occupancy_matrix_msg->occupancy_rows[i].occupancy_row[j]);
            }
        }

        occupancy_grid_received = true;
    }
    /*
    void occupancyCallback(const robo7_msgs::grid_matrix::ConstPtr &occupancy_matrix_msg)
    {
        occupancy_array.clear();

        occupancy_grid_width = occupancy_matrix_msg->grid_rows.size();
        occupancy_grid_height = occupancy_matrix_msg->grid_rows[0].grid_row.size();
        ROS_INFO("Received occupancy");

        for (int j = 0; j < occupancy_grid_height; j++)
        {
            for (int i = 0; i < occupancy_grid_width; i++)
            {
                occupancy_array.push_back(occupancy_matrix_msg->grid_rows[i].grid_row[j]);
            }
        }

        occupancy_grid_received = true;
    }
*/
    void distanceCallback(const robo7_msgs::occupancy_matrix::ConstPtr &distance_matrix_msg)
    {
        distance_array.clear();

        distance_grid_width = distance_matrix_msg->occupancy_rows.size();
        distance_grid_height = distance_matrix_msg->occupancy_rows[0].occupancy_row.size();
        // ROS_INFO("Received distance");

        for (int j = 0; j < distance_grid_height; j++)
        {
            for (int i = 0; i < distance_grid_width; i++)
            {
                distance_array.push_back(distance_matrix_msg->occupancy_rows[i].occupancy_row[j]);
            }
        }

        distance_grid_received = true;
    }

    void explorationCallback(const robo7_msgs::grid_matrix::ConstPtr &exploration_matrix_msg)
    {
        exploration_array.clear();

        exploration_grid_width = exploration_matrix_msg->grid_rows.size();
        exploration_grid_height = exploration_matrix_msg->grid_rows[0].grid_row.size();
        // ROS_INFO("Received exploration");

        for (int j = 0; j < exploration_grid_height; j++)
        {
            for (int i = 0; i < exploration_grid_width; i++)
            {
                exploration_array.push_back(exploration_matrix_msg->grid_rows[i].grid_row[j]);
            }
        }

        exploration_grid_received = true;
    }

    void updateOccupancyGrid()
    {

        if (this->occupancy_grid_received)
        {

            nav_msgs::OccupancyGrid occupancy_grid;

            occupancy_grid.header.frame_id = "/map";
            occupancy_grid.header.stamp = ros::Time::now();

            occupancy_grid.info.resolution = 2.44 / occupancy_grid_width;
            occupancy_grid.info.width = occupancy_grid_width;
            occupancy_grid.info.height = occupancy_grid_height;
            occupancy_grid.info.origin.position.x = 0;
            occupancy_grid.info.origin.position.y = 0;
            occupancy_grid.info.origin.position.z = path_height;

            for (int i = 0; i < occupancy_grid_width * occupancy_grid_height; i++)
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
                    grid_value = (int)100 * distance_array[i] / max_distance;

                distance_grid.data.push_back(grid_value);
            }

            distance_grid_pub.publish(distance_grid);
        }
    }

    void updateExplorationGrid()
    {

        if (this->exploration_grid_received)
        {
            int grid_value;
            nav_msgs::OccupancyGrid exploration_grid;

            exploration_grid.header.frame_id = "/map";
            exploration_grid.header.stamp = ros::Time::now();

            exploration_grid.info.resolution = 2.44 / exploration_grid_width;
            exploration_grid.info.width = exploration_grid_width;
            exploration_grid.info.height = exploration_grid_height;
            exploration_grid.info.origin.position.x = 0;
            exploration_grid.info.origin.position.y = 0;
            exploration_grid.info.origin.position.z = path_height;

            for (int i = 0; i < exploration_grid_width * exploration_grid_height; i++)
            {
                if (grid_value >= 0.0)
                    grid_value = (int)100 - 100 * exploration_array[i];
                else
                    grid_value = (int)exploration_array[i];
                exploration_grid.data.push_back(grid_value);
            }

            exploration_grid_pub.publish(exploration_grid);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy");
    ros::NodeHandle nh;

    ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("OccupancyGrid/Occupancy_Grid", 100);
    ros::Publisher distance_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("OccupancyGrid/Distance_Grid", 100);
    ros::Publisher exploration_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("OccupancyGrid/Exploration_Grid", 100);

    ROS_INFO("Init Grid Visualization");
    OccupancyGrid occupancy_grid = OccupancyGrid(nh, occupancy_grid_pub, distance_grid_pub, exploration_grid_pub);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {

        occupancy_grid.updateOccupancyGrid();
        occupancy_grid.updateDistanceGrid();
        occupancy_grid.updateExplorationGrid();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
