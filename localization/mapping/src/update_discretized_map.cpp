#include <ros/ros.h>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <string>
#include <vector>

//The messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <robo7_msgs/mapping_grid.h>
#include <robo7_msgs/wallPoint.h>

//The services
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/UpdateDiscretizedMap.h>
#include <robo7_srvs/discretize_map.h>


class Update_Discretized_Map
{
public:
	ros::NodeHandle n;
	//Server
	ros::ServiceServer update_map_server;
	//Client
  ros::ServiceClient ransac_srv;
	ros::ServiceClient discretization_srv;
	//Publisher
	ros::Publisher the_point_cloud_ransac_pub;
  ros::Publisher ransac_walls_pub;
	ros::Publisher discretized_map_pub;

	Update_Discretized_Map()
	{
		//Input parameters definition of the node
		n.param<float>("/update_discretized_map/cell_size", cell_size, 0.05);

		//Occupancy grid values Initialisation
		unknown = 0;
		unoccupied = 1;
		occupied = 2;

		//ServiceServer
		update_map_server = n.advertiseService("/localization/mapping/update_discretized_map", &Update_Discretized_Map::update_Sequence, this);

		//Service Clients
		ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
		discretization_srv = n.serviceClient<robo7_srvs::discretize_map>("/maze_map/map_discretization");

		//Visualization of the updated versions of the walls
		the_point_cloud_ransac_pub = n.advertise<robo7_msgs::wallPoint>("/mapping/point_occ_cloud", 1);
		ransac_walls_pub = n.advertise<robo7_msgs::wallList>("/mapping/ransac_walls", 1);
		discretized_map_pub = n.advertise<robo7_msgs::wallPoint>("/mapping/discretized_map", 1);
	}

	bool update_Sequence(robo7_srvs::UpdateDiscretizedMap::Request &req,
         robo7_srvs::UpdateDiscretizedMap::Response &res)
	{
		//Extract the occupancy grid out of the request message
		extract_grid_msg = req.occupancy_grid;

		//Extract all the point coordinates out of the occupancy grid which
		//represent some occupied cells -> value = 2 in the matrix
		//There coordinates are the ones of the center of the cell
		extract_current_grid();

		//Once those point extracted, we call the RANSAC algorithm in order to
		//Extract the walls out of those points
		extract_walls_with_ransac();

		//Finally, we call the discretization service in order to create a new
		//discretization of the map for the localization
		discretize_the_walls();

		//We finish with publishing a message that the EKF subscribes to for
		//localization node
		//Return / Publish the messages
		res.discretized_walls = current_map_discretized;
		res.success = true;

		the_point_cloud_ransac_pub.publish( the_wall_cloud );
		ransac_walls_pub.publish( the_wall_list );
		discretized_map_pub.publish( current_map_discretized );

		return true;
	}

private:
	//Input and output of the main service
	robo7_msgs::mapping_grid extract_grid_msg;
	robo7_msgs::wallPoint current_map_discretized;

	//The current grid grid
	Eigen::MatrixXf the_occupancy_grid;
	float x_occupancy_min, x_occupancy_max;
	float y_occupancy_min, y_occupancy_max;
	int occupancy_nb_rows, occupancy_nb_cols;

	//The Ransac input and output messages
	robo7_msgs::wallPoint the_wall_cloud;
	robo7_msgs::wallList the_wall_list;

	//The different grid values
	float unknown, occupied, unoccupied;
	float cell_size;

	void extract_current_grid()
	{
		//Initialize the point cloud msg
		the_wall_cloud.number = 0;
		the_wall_cloud.the_points.clear();

		//Extract the values out of the occupancy grid and fill up the point cloud
		cell_size = extract_grid_msg.cell_size;
		occupancy_nb_rows = extract_grid_msg.occupancy_grid.nb_rows;
		occupancy_nb_cols = extract_grid_msg.occupancy_grid.nb_cols;
		the_occupancy_grid.resize(occupancy_nb_rows, occupancy_nb_cols);
		x_occupancy_min = extract_grid_msg.top_left_corner.x;
		x_occupancy_max = x_occupancy_min + cell_size * extract_grid_msg.occupancy_grid.nb_cols;
		y_occupancy_max = extract_grid_msg.top_left_corner.y;
		y_occupancy_min = y_occupancy_max - cell_size * extract_grid_msg.occupancy_grid.nb_rows;
		for(int i=0; i < occupancy_nb_rows; i++)
		{
			for(int j=0; j < occupancy_nb_cols; j++)
			{
				the_occupancy_grid(i,j) = extract_grid_msg.occupancy_grid.rows[i].cols[j];
				if(the_occupancy_grid(i,j) == 2)
				{
					//Add a coordinate in the wall point list
					geometry_msgs::Vector3 coord = corresponding_coordinates(i,j);
					the_wall_cloud.the_points.push_back( coord );
					the_wall_cloud.number++;
				}
			}
		}
	}

	geometry_msgs::Vector3 corresponding_coordinates(int i, int j)
	{
		float x_value = x_occupancy_min + (j+1) * cell_size - cell_size/2;
		float y_value = y_occupancy_max - (i+1) * cell_size + cell_size/2;

		geometry_msgs::Vector3 wall_point;
		wall_point.x = x_value;
		wall_point.y = y_value;
		wall_point.z = 0;

		return wall_point;
	}

	void extract_walls_with_ransac()
	{
		robo7_srvs::RansacWall::Request req1;
		robo7_srvs::RansacWall::Response res1;
		req1.the_cloud = the_wall_cloud;
		ransac_srv.call(req1, res1);
		the_wall_list = res1.ransac_walls;
	}

	void discretize_the_walls()
	{
		robo7_srvs::discretize_map::Request req1;
		robo7_srvs::discretize_map::Response res1;
		req1.walls = the_wall_list;
		discretization_srv.call(req1, res1);
		current_map_discretized = res1.discretized_walls;
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Updating map service");

	Update_Discretized_Map Update_Discretized_Map_;

	ros::Rate loop_rate(100);

	ROS_INFO("Updating map Service is running");

	ros::spin();

	return 0;
}
