
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robo7_msgs/occupancy_matrix.h>
#include <robo7_msgs/occupancy_row.h>
#include <std_msgs/Int8MultiArray.h>

typedef std::vector<int> int_vector;
std::vector<int_vector> occupancy;

class OccupancyGrid
{
  public:
    ros::Subscriber occupancy_grid_sub;
    ros::Publisher occupancy_grid_pub;
    std::vector<float> occupancy_array;
    int occupancy_grid_width, occupancy_grid_height;
    //Initialisation

    bool occupancy_grid_received;

    OccupancyGrid(ros::NodeHandle nh, ros::Publisher occupancy_grid_pub)
    {
        this->occupancy_grid_sub = nh.subscribe("/occupancy_grid_server/occupancy_matrix", 1000, &OccupancyGrid::occupancyCallback, this);
        this->occupancy_grid_pub = occupancy_grid_pub;

        this->occupancy_grid_received = false;
    }

    void occupancyCallback(const robo7_msgs::occupancy_matrix::ConstPtr &occupancy_matrix_msg)
    {
        occupancy_array.clear();

        occupancy_grid_width = occupancy_matrix_msg->occupancy_rows.size();
        occupancy_grid_height = occupancy_matrix_msg->occupancy_rows[0].occupancy_row.size();

        for (int j = 0; j < occupancy_grid_height; j++)
        {
            for (int i = 0; i < occupancy_grid_width; i++)
            {
                occupancy_array.push_back(occupancy_matrix_msg->occupancy_rows[i].occupancy_row[j]);
            }
        }

        occupancy_grid_received = true;
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

            for (int i = 0; i < occupancy_grid_width * occupancy_grid_height; i++)
            {
                int grid_value = (int)100 * occupancy_array[i];
                occupancy_grid.data.push_back(grid_value);
            }

            occupancy_grid_pub.publish(occupancy_grid);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy");
    ros::NodeHandle nh;

    ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("OccupancyGrid", 100);

    ROS_INFO("Init occupancy visualization");
    OccupancyGrid occupancy_grid = OccupancyGrid(nh, occupancy_grid_pub);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        occupancy_grid.updateOccupancyGrid();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
