#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robo7_msgs/XY_coordinates.h"

class OccupancyGridServer
{
public:
	ros::NodeHandle n;
	ros::Subscriber map_sub;
	ros::ServiceServer is_occupied_service;

	OccupancyGridServer()
	{
		// Parameters
		n.param<double>("/occupancy_grid/min_distance", min_distance, 0);
		n.param<double>("/occupancy_grid/smooth_distance", smooth_distance, 0);

		map_sub = n.subscribe("/dead_reckoning/Pos", 1, &OccupancyGridServer::mapCallback, this);
		is_occupied_service = n.advertiseService("/occupancy_grid/is_occupied", &OccupancyGridServer::gridRequest, this);

	}

	void mapCallback(const geometry_msgs::Twist::ConstPtr &msg)
	{
    //robot_pos_x = msg->linear.x;
		//robot_pos_y = msg->linear.y;
		//robot_pos_tet = msg->angular.z;
	}


	bool gridRequest(robo7_srvs::IsGridOccupied::Request &req,
         robo7_srvs::IsGridOccupied::Response &res)
	{

	  ROS_DEBUG("New grid occupancy request recieved");

		//geometry_msgs::Twist obejct_pos_robot = req.object_pos;

		//res.success = true;

    return true;
	}

private:
	double min_distance;
	double smooth_distance;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_grid_server");

	OccupancyGridServer gate_controller_server;

	ros::Rate loop_rate(100);

	ROS_INFO("Occupancy grid server running");

	ros::spin();

	return 0;
}
