#include <ros/ros.h>

//The messages
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

//The services
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/update_map.h>
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/discretize_map.h>

#include <stdlib.h>

#include <string>
#include <vector>


class UpdateMap
{
public:
	ros::NodeHandle n;
	ros::ServiceServer update_map_server;
	ros::ServiceClient scan_to_coord_srv;
	ros::ServiceClient discretized_map_srv;
  ros::ServiceClient ransac_srv;
  ros::Publisher former_map_pub;
	ros::Publisher lidar_map_pub;
	ros::Publisher new_map_pub;

	UpdateMap()
	{
		n.param<int>("/update_map/choose_method", method, 0);
		// pi = 3.14159265358979323846;

		//ServiceServer
		update_map_server = n.advertiseService("/localization/mapping/update_map", &UpdateMap::update_Sequence, this);

		//Service Clients
		ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
		scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
		discretized_map_srv = n.serviceClient<robo7_srvs::discretize_map>("/maze_map/map_discretization");

		//Visualization of the updated versions of the walls
		former_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/former_map", 1);
		lidar_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/lidar_map", 1);
		new_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/new_map", 1);
	}

	bool update_Sequence(robo7_srvs::update_map::Request &req,
         robo7_srvs::update_map::Response &res)
	{
		//Extract the grid out of the request message
		extract_previous_grid();

		//Extract the pose_timed out of the request message
		//Both lidar scan and corresponding position


		//Create a new matrix that can contains all of the lidar scan measures
		//and previous occupancy grid map. All it cells should be seen as
		//unoccupied cells

		//Stick the previous occupancy grid on the top of the new grid at its
		//right position

		//Define a window around the robot where every single cell is unoccupied
		//until it reaches a wall (do a linearisation for the occupancy regarding
		//how close 2 consecutive points are):
		//In a second time, think about rejecting the outliers before applying the
		//linearisation and the spreading algorithm (see with RANSAC)

		//Stick this new grid on top of the current grid at its right position.
		//Keep in mind that "unknown cells" should take tthe value of the previous
		//grid cells

		//Then return the newly defined grid so that it can be saved in the brain


		return true;
	}

private:
	//Input and output of the main service



};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mapping service");

	UpdateMap UpdateMap_;

	ros::Rate loop_rate(100);

	ROS_INFO("Mapping Service is running");

	ros::spin();

	return 0;
}
